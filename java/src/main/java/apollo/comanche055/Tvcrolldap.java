package apollo.comanche055;

import apollo.agc.Agc;

/**
 * TVCROLLDAP.agc (Comanche 055) - the CSM TVC roll digital autopilot (ROLLDAP). Runs every 1/2 sec while the
 * SPS engine is on. Estimates OGA rate, evaluates the phase-plane (switch parabola / switch line regions 1-6),
 * and fires roll RCS jets for a computed duration via TIME6 / CHAN6.
 *
 * Quantities are held as doubles in the same units as the AGC scaling comments (OGA in revolutions,
 * OGARATE in rev/sec, 1/CONACC in sec^2/rev, jet times in 625 microsecond bits); the AGC's fixed-point
 * overflow protection is modelled by clamping to +/-POSMAX.
 */
public class Tvcrolldap {
    private final Agc agc;

    public static final double DB = 0.01388889;        // dead band (5 deg), rev
    public static final double MSLOPE = 0.2;           // -SLOPE: switchline slope (0.2 per sec)
    public static final double LMCRATE = 0.00027778;   // limit cycle rate (0.1 deg/sec), rev/sec
    public static final double INTERCEP = 0.0025;      // DB*(-SLOPE) - LMCRATE
    public static final double MINLIM = 0.00277778;    // rate limit min (1 deg/sec)
    public static final double RECIP_MINLIM = 360.0;   // 1/MINLIM (shifted)
    public static final double MAXLIM = 0.01388889;    // rate limit max (5 deg/sec)
    public static final double TMINFIRE = 1.5;         // 15 ms min fire (16 bits/cs scale)
    public static final double TMAXFIRE = 250.0;       // 2.5 sec max fire
    public static final double T6SCALE_RATIO = 8.0 / 10.24;
    public static final int ROLL_POS_1 = 05;           // +ROLL1 = FIVE: jets 9 and 11
    public static final int ROLL_POS_2 = 0120;         // +ROLL2 = OCT120: jets 13 and 15
    public static final int ROLL_NEG_1 = 012;          // -ROLL1 = TEN: jets 12 and 10
    public static final int ROLL_NEG_2 = 0240;         // -ROLL2: jets 16 and 14
    public static final double POSMAX = 1.0;

    // working registers
    private double ogaRate, temreg, sgnrt, deloga, delogart, iogarate;
    private int i; // I register: +1 (CA) or -1 (CS), 0 for MOREROLL switch use
    private boolean moreroll;

    public Tvcrolldap(Agc agc) {
        this.agc = agc;
    }

    private static double sat(double x) {
        return Math.max(-POSMAX, Math.min(POSMAX, x));
    }

    private double d(String s) {
        return Agc.toSigned(agc.get(s)) / 16384.0;
    }

    /** ROLLDAP: OGA rate estimator, simple first-order difference (sample time 1/2 sec). */
    public void rolldap() {
        double oganow = d("OGANOW"), ogapast = d("OGAPAST");
        ogaRate = (oganow - ogapast) * (1.0 / 32); // MP BIT5, scaled at B-4 rev/sec
        agc.set("OGARATE", Agc.fromSigned((int) Math.round(ogaRate * 16384)));
        duration();
    }

    /** DURATION: skip logic if jets have been on / off less than 1/2 sec. */
    private void duration() {
        double rollfire = d("ROLLFIRE");
        if (rollfire != 0) {
            rolllogic(); // jets now on
            return;
        }
        if (temreg == 0 || temreg < 0) { // BZF: last firing interval examined
            // CAE TEMREG; BZF ROLLOGIC: jets now off
        }
        if (temreg == 0) {
            rolllogic();
            return;
        }
        temreg = 0; // jets have not been off for 1/2 sec - wait
        wait1Half();
    }

    /** WAIT1/2: exit roll DAP (TASKOVER). */
    private void wait1Half() {
        // TCF TASKOVER
    }

    /** ROLLOGIC: phase-plane logic. */
    public void rolllogic() {
        double oga = d("OGA");
        double oneOverConacc = d("1/CONACC");
        // TEMREG = DB - OGARATE^2/CONACC
        temreg = DB - ogaRate * ogaRate * oneOverConacc;
        sgnrt = ogaRate > 0 ? 1 : -1; // SGN(0) is negative
        // DELOGA = OGA - SGN(OGARATE)*TEMREG (positive to right of C-parabola)
        deloga = oga - sgnrt * temreg;
        i = deloga <= 0 ? 1 : -1; // CAF PRIO30 (CA) vs CAF BIT15 (CS)
        double k = i * sgnrt;
        if (k >= 0) { // REG1TST: region 1 (DELOGA and OGARATE same sign)
            rollon();
            return;
        }
        iogarate = i * ogaRate; // must be negative here
        if (iogarate + MAXLIM <= 0) { // REG3TST: region 3, rates too high
            rateLim();
            return;
        }
        double x = i * (MSLOPE * -1 * oga + ogaRate);
        if (x - INTERCEP <= 0) { // REG2TST: region 2, coast side of straight line
            noroll();
            return;
        }
        if (iogarate + MINLIM <= 0) { // REG4TST: region 4
            noroll();
            return;
        }
        if (i * oga - DB <= 0) { // REG5TST: region 5, coast side of DB
            noroll();
            return;
        }
        // REGION6: use actual operating point for tangent
        deloga = oga;
        delogart = ogaRate;
        onroll();
    }

    /** ROLLON: region 1; evaluate state for the indirect line. */
    private void rollon() {
        deloga += i * DB; // DELOGA was distance from switch parabola
        delogart = i * -LMCRATE;
        onroll();
    }

    /** ONROLL: compute desired rate by intersecting switching lines. */
    private void onroll() {
        double oneOverConacc = d("1/CONACC");
        temreg = 2 * oneOverConacc * delogart * -MSLOPE; // 2(-SLOPE)RATE/CONACC
        delogart = temreg * delogart;                      // 2(-SLOPE)(RATE^2)/CONACC
        temreg += i * -(1.0 / 16);                         // RATEDEN: denominator (CS BIT11, indexed)
        delogart += (i * deloga - DB) * -MSLOPE;
        double num = delogart + LMCRATE * (1.0 / 16);      // RATENUM
        double desired;
        if (Math.abs(temreg) < 1e-12 || Math.abs(num / temreg) >= 1.0) {
            desired = (num >= 0) == (temreg >= 0) ? POSMAX : -POSMAX; // MINLIMAP overflow
        } else {
            desired = num / temreg; // DVOK
        }
        rollset(desired);
    }

    /** RATELIM: desired rate is -I*MAXLIM. */
    private void rateLim() {
        rollset(i * -MAXLIM);
    }

    /** ROLLSET: desired rate difference to jet fire time (625 usec/bit). */
    private void rollset(double desiredRate) {
        double diff = sat(desiredRate - ogaRate);
        double t = sat(diff * T6SCALE_RATIO);
        t = sat(t * d("1/CONACC") * 4);
        temreg = t * 16384; // JET FIRE TIME AT 625 MICROSEC/BIT, positive means positive roll torque
        if (temreg == 0) {
            noroll();
            return;
        }
        double rollfire = d("ROLLFIRE");
        double prod = temreg * Math.signum(rollfire);
        if (prod > 0) { // MOREROLL: continue firing with present polarity
            moreroll = true;
            maxtfire();
        } else if (prod == 0 || rollfire == 0) { // NEWROLL: start new firing
            moreroll = false;
            newroll();
        } else { // opposite sign requested: terminate old firing
            noroll();
        }
    }

    /** NEWROLL: new firing must exceed TMINFIRE. */
    private void newroll() {
        double mag = Math.abs(temreg);
        if (TMINFIRE - mag >= 0) { // MINTST
            noroll();
            return;
        }
        maxtfire();
    }

    /** MAXTFIRE / NOMXFIRE: limit to TMAXFIRE, set ROLLFIRE sign. */
    private void maxtfire() {
        if (Math.abs(temreg) >= TMAXFIRE) {
            temreg = temreg > 0 ? TMAXFIRE : -TMAXFIRE;
        }
        int sign = temreg > 0 ? 1 : -1;
        agc.set("ROLLFIRE", Agc.fromSigned(sign * 0x3FFF));
        temreg = Math.abs(temreg) * 1.0; // positive for negative torque after COM; magnitude for TIME6
        firelook(sign);
    }

    /** FIRELOOK / FIREPLUG: for MOREROLL ensure an extension is wanted, else jet roll. */
    private void firelook(int sign) {
        if (moreroll) {
            double time6 = Agc.toSigned(agc.get("TIME6"));
            if (time6 - temreg <= 0) { // EXTENTST: extension not wanted; exit ROLL DAP
                return; // TASKOVER
            }
        }
        jetroll(sign);
    }

    /** NOROLL: coast (negative zero for TIME6). */
    private void noroll() {
        agc.set("ROLLFIRE", Agc.WORD_MASK); // -0
        temreg = 0;
        jetroll(0);
    }

    /** JETROLL: load T6LOC with NOROL1T6, TIME6 with fire time, then select jets. */
    private void jetroll(int sign) {
        agc.set("T6LOC", agc.get("NOROL1T6"));
        agc.set("TIME6", (int) Math.round(temreg) & Agc.WORD_MASK);
        if (moreroll && i == 0) {
            return; // SAMEJETS: jets on, keep same jets
        }
        int rollword = agc.get("ROLLWORD");
        double rollfire = d("ROLLFIRE");
        if (rollfire > 0) { // +TORQUE
            if ((rollword & 1) == 0) { // NO.9-11: use 9-11 this time
                rollword |= 1;
                agc.set("ROLLWORD", rollword);
                agc.set("CHAN6", ROLL_POS_1);
            } else { // NO.13-15
                rollword &= ~1;
                agc.set("ROLLWORD", rollword);
                agc.set("CHAN6", ROLL_POS_2);
            }
        } else if (rollfire < 0) { // -TORQUE
            if ((rollword & 2) == 0) { // NO.12-10
                rollword |= 2;
                agc.set("ROLLWORD", rollword);
                agc.set("CHAN6", ROLL_NEG_1);
            } else { // NO.16-14
                rollword &= ~2;
                agc.set("ROLLWORD", rollword);
                agc.set("CHAN6", ROLL_NEG_2);
            }
        }
        t6enabl();
    }

    /** T6ENABL: enable TIME6 (WOR CHAN13 bit 15). */
    private void t6enabl() {
        agc.set("CHAN13", agc.get("CHAN13") | 0x4000);
        // RDAPEND: TASKOVER
    }

    /** NOROLL1: T6 task called by JETROLL to shut off all roll jets; ROLLFIRE = 0 (jets off). */
    public void noroll1() {
        agc.set("ROLLFIRE", 0);
        killjets();
    }

    /** KILLJETS: WRITE CHAN6 with zero, return via NOQRSM. */
    public void killjets() {
        agc.set("CHAN6", 0);
        // TCF NOQRSM
    }
}
