package apollo.luminary099;

import apollo.agc.Agc;

/**
 * FINDCDUW--GUIDAP_INTERFACE (Luminary 099): FINDCDUW computes the CDU desired angles (gimbal
 * angle changes), commanded body rates and lag angles needed to point the descent engine thrust
 * (UNFC/2) and optionally a window (UNWC/2) in the desired directions, and filters the thrust
 * offset. Also contains the single-precision helper routines LIMITSUB, ONESTO2S, ARCTRGSP and
 * SPARCSIN.
 *
 * Angles here are expressed as doubles in units of PI (1.0 = 180 degrees), matching the "1's, PI"
 * scaling of the original constants. Vectors are double[3].
 */
public class FindcduwGuidapInterface {
    /** GAINFLTR: filter gain without / with CSM docked. */
    public static final double[] GAINFLTR = {.2, .1};
    /** DUNFVLIM: 7 mr max change in F direction in vehicle in 2 secs. */
    public static final double DUNFVLIM = .007 * 2;
    /** UNFVLIM: 129 mr max thrust offset. */
    public static final double UNFVLIM = .129 * 2;
    /** DOTSWFMX: limit collinearity of UNWC/2 and UNFC/2 to 85 degrees (scaled B-4, i.e. squared). */
    public static final double DOTSWFMX = .93302 / 16;
    /** DAXMAX: DELATTX limit to 20 deg in 2 secs (2 deg when CSM docked). */
    public static final double[] DAXMAX = {.11111111111, .0111111111};
    /** DAY/2MAX: likewise for DELATTY. */
    public static final double[] DAY_2MAX = {.05555555555, .0055555555};
    /** DAZMAX = DAXMAX: likewise for DELATTZ. */
    public static final double[] DAZMAX = DAXMAX;
    /** CDUZDLIM: 70 deg limit for MGA. */
    public static final double CDUZDLIM = .3888888888;
    /** DT/DELT: .1 sec / 2 sec autopilot interval ratio. */
    public static final double DT_DELT = .05;
    /** DELERLIM = DAY/2MAX: lag angle limit. */
    public static final double DELERLIM = DAY_2MAX[0];
    /** SPARCSIN polynomial coefficients (DEC integer scaled). */
    public static final int DPL1 = 10502, DPL3 = 432, DPL5 = 7300, DPL7 = -11803, DPL9 = 8397;
    /** 16OCT. */
    public static final int OCT16 = 016;
    /** Alarm codes. */
    public static final int ALARM_NO_ATTITUDE_CONTROL = 00402, ALARM_MGA = 00401;

    /** Interpretive / external routines that FINDCDUW calls. */
    public interface Env {
        double[] delv();
        double[] znbpip();
        double[] xnbpip();
        /** SMNB: transform a vector from stable-member to navigation-base coordinates. */
        double[] smnb(double[] v);
        double[] oneJacc(); // 1JACC per-axis acceleration constants
        boolean channelBit(int channel, int bit); // true if the channel bit is set
        void alarm(int code);
        void stoprate(); // IBNKCALL STOPRATE
    }

    private final Agc agc;
    private Env env;

    // Working state (UNX/2, UNY/2, UNZ/2, UNFV/2, UNWC/2 etc.)
    private final double[] unx = new double[3], uny = new double[3], unz = new double[3];
    private final double[] unfv = new double[3];
    private final double[] unwc = new double[3];
    private boolean flagoodw;
    private int ndxcdUw;
    private boolean flpautno;
    private double sinCduX, cosCduX, sinCduZ, cosCduZ;

    public FindcduwGuidapInterface(Agc agc) {
        this.agc = agc;
    }

    public void setEnv(Env env) {
        this.env = env;
    }

    // ---------------------------------------------------------------- vector helpers

    private static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]};
    }

    private static double[] unit(double[] a) {
        double n = Math.sqrt(dot(a, a));
        return n == 0 ? new double[3] : new double[] {a[0] / n, a[1] / n, a[2] / n};
    }

    private static double[] scale(double[] a, double k) {
        return new double[] {a[0] * k, a[1] * k, a[2] * k};
    }

    private static double[] add(double[] a, double[] b) {
        return new double[] {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
    }

    private static double[] neg(double[] a) {
        return scale(a, -1);
    }

    private static double cduAngle(int word) {
        return (word & Agc.WORD_MASK) / 16384.0; // 15-bit counts, 2*PI full scale -> PI units
    }

    /** INITCDUW: initialise UNFV/2 and UNWC/2 to the X unit vector. */
    public void initcduw() {
        double[] unitX = {1, 0, 0};
        System.arraycopy(unitX, 0, unfv, 0, 3);
        System.arraycopy(unitX, 0, unwc, 0, 3);
    }

    /** FINDCDUW -2: entry when UNFC/2 pre-stored (input vectors need not be semi-unit). */
    public void findcduwMinus2(double[] unfc2) {
        findcduw(unfc2);
    }

    /**
     * FINDCDUW: entry with the thrust command UNFC/2 in MPAC.
     * Sets CPHI, DELCDUX/Y/Z, OMEGAPD/QD/RD and DELPEROR/QEROR/REROR.
     */
    public void findcduw(double[] unfc2) {
        ndxcdUw = (agc.get("DAPBOOLS") & agc.get("CSMDOCKD")) != 0 ? 1 : 0;
        flpautno = true; // PNGCS AUTO NOT
        flagoodw = (agc.get("XOVINHIB") & agc.get("DAPBOOLS")) != 0;

        // INHINT; fetch CDUs, replaced below by CDUXD.. if PGNCS and AUTO.
        int cduSpotX = agc.get("CDUX");
        int cduSpotY = agc.get("CDUY");
        int cduSpotZ = agc.get("CDUZ");
        boolean pgncs = !env.channelBit(030, 010); // bit 10 inverted
        boolean auto = !env.channelBit(031, 020000); // bit 14 inverted
        if (pgncs && auto) {
            flpautno = false;
            cduSpotX = agc.get("CDUXD");
            cduSpotY = agc.get("CDUYD");
            cduSpotZ = agc.get("CDUZD");
        }
        // PAUTNO: back to the interpreter
        double[] fx = unit(unfc2); // NORMUNIT: semi-unit thrust command as initial UNX/2
        System.arraycopy(fx, 0, unx, 0, 3);
        double[] wz = unit(unwc);  // NORMUNIT of window command
        quictrig(cduSpotX, cduSpotZ);
        System.arraycopy(wz, 0, unz, 0, 3);
        double[] delv = env.delv();
        double dn = Math.sqrt(dot(delv, delv));
        boolean attCntOk = dn != 0;
        if (!attCntOk) {
            noattcnt(true);
            return;
        }
        double[] ud = scale(delv, 1 / dn); // UNIT(DELV)
        double[] udVeh = env.smnb(ud); // unit(DELV) in vehicle coordinates for the filter
        // Two filter updates (Y and Z components of the thrust offset UNFV/2)
        unfv[1] = fltrsub(unfv[1], udVeh[1]);
        unfv[2] = fltrsub(unfv[2], udVeh[2]);
        afterFilter();
    }

    /** AFTRFLTR: pick the Z axis (window or ZNBPIP / -XNBPIP) then build the DCM. */
    private void afterFilter() {
        boolean haveZ = false;
        if (flagoodw) {
            haveZ = unwcTest(unz);
        }
        if (!haveZ) {
            double[] z = env.znbpip();
            System.arraycopy(z, 0, unz, 0, 3);
            if (!unwcTest(unz)) {
                double[] x = neg(env.xnbpip()); // Z and -X can't both be parallel to UNFC/2
                System.arraycopy(x, 0, unz, 0, 3);
            }
        }
        dcmcl();
    }

    /**
     * UNWCTEST: window vector acceptable if (UNX.V)^2 &lt; DOTSWFMX. Otherwise clears FLAGOODW
     * and returns false so the caller will fetch an alternate Z axis.
     */
    private boolean unwcTest(double[] v) {
        double d = dot(v, unx);
        if (d * d - DOTSWFMX < 0) {
            return true; // BMN DCMCL
        }
        flagoodw = false; // SSP FLAGOODW 0, RVQ
        return false;
    }

    /** DCMCL: build the direction cosine matrix removing the filtered thrust offset, then rates. */
    private void dcmcl() {
        double[] uy = unit(cross(unz, unx));          // UNY/2 first iteration
        double[] negUz = cross(uy, unx);              // -UNZ/2 first iteration
        double[] corr = add(scale(negUz, unfv[2]), scale(uy, unfv[1])); // UNFVZ*(-UNZ)+UNFVY*UNY
        double[] xn = unit(add(unx, neg(corr)));      // totally eliminates thrust pointing error
        System.arraycopy(xn, 0, unx, 0, 3);
        double[] y = cross(unx, negUz);               // UNY/2
        System.arraycopy(y, 0, uny, 0, 3);
        double[] z = cross(unx, uny);                 // UNZ/2
        System.arraycopy(z, 0, unz, 0, 3);
        double[] cdu = nb2cdusp();                    // {CDUX, CDUY, CDUZ} in PI units
        rates(cdu);
    }

    /**
     * NB2CDUSP: required gimbal angles (units of PI, 2's complement) from the DCM rows UNX, UNY, UNZ.
     * Returns {CDUX, CDUY, CDUZ}.
     */
    private double[] nb2cdusp() {
        double sinZ = unx[1];
        double cos2 = 1.0 / 4 - sinZ * sinZ / 4; // scaled: DP1/4TH - SIN^2 (sin scaled 1/2)
        double cosZ = cos2 > 0 ? Math.sqrt(1 - sinZ * sinZ) : 0; // zero if sin slightly > 1
        double cduZ = arctrgsp(sinZ, cosZ);
        double cduY = arctrgsp(-dvByCosm(unx[2], cosZ), dvByCosm(unx[0], cosZ));
        double cduX = arctrgsp(-dvByCosm(unz[1], cosZ), dvByCosm(uny[1], cosZ));
        return new double[] {cduX, cduY, cduZ};
    }

    /** DVBYCOSM: operand / cos(CDUZ), division skipped if cos exceeds |operand| by under a bit. */
    private static double dvByCosm(double operand, double cosZ) {
        if (cosZ - Math.abs(operand) <= 0) {
            return Math.signum(operand) * Math.min(Math.abs(operand), 1.0); // TSL&TCQ
        }
        return operand / cosZ;
    }

    /** Rates and lag angles part of FINDCDUW (limit MGA, DELGMB loop, OMEGA*D, DELCDU*, DELPEROR). */
    private void rates(double[] cdu) {
        double cduZ = cdu[2];
        double limited = limitsub(CDUZDLIM, cduZ);
        boolean mgaExceeded = limited != cduZ;
        cdu[2] = limited;
        if (mgaExceeded) {
            env.alarm(ALARM_MGA);
        }
        // MGARET
        String[] cduD = {"CDUXD", "CDUYD", "CDUZD"};
        double[] ng = new double[3]; // -DELGMB : -(CDU - CDUD)
        double prev = 0;
        for (int i = 2; i >= 0; i--) {
            if (prev * prev - 0.25 * 0.25 > 0) { // delta beyond ~45 degrees
                flagoodw = false;
            }
            agc.set("CPHI" + (i == 0 ? "" : "+" + i), Agc.fromSigned((int) Math.round(cdu[i] * 16384)));
            ng[i] = -(cdu[i] - Agc.toSigned(agc.get(cduD[i])) / 16384.0);
            prev = ng[i];
        }
        if (flpautno || (agc.get("FLAGWRD5") & agc.get("ENGONBIT")) == 0) {
            noattcnt(true);
            return;
        }
        // Limit gimbal angle changes
        ng[2] = limitsub(DAZMAX[ndxcdUw], ng[2]);
        double ngyOld = ng[1];
        double dAttYprime = limitsub(DAY_2MAX[ndxcdUw], ng[1] * cosCduZ);
        ng[1] = dAttYprime / cosCduZ;
        double dax = -2 * ngyOld * sinCduZ - ng[0];
        double daxLim = limitsub(DAXMAX[ndxcdUw], dax);
        ng[0] = flagoodw ? -daxLim : 0;
        ng[0] += 2 * (-ng[1]) * sinCduZ; // -DELGMBX
        double omegaP = 2 * (-ng[0] + 2 * (-ng[1]) * sinCduZ);
        double omegaQ = 4 * (-ng[1] * cosCduX * 2 * cosCduZ + (-ng[2]) * sinCduX);
        double omegaR = 4 * (ng[1] * sinCduX * 2 * cosCduZ + (-ng[2]) * cosCduX);
        double[] omega = {omegaP, omegaQ, omegaR};
        String[] omegaS = {"OMEGAPD", "OMEGAQD", "OMEGARD"};
        String[] delCdu = {"DELCDUX", "DELCDUY", "DELCDUZ"};
        String[] delEr = {"DELPEROR", "DELQEROR", "DELREROR"};
        double[] jacc = env.oneJacc();
        for (int i = 0; i < 3; i++) {
            agc.set(omegaS[i], Agc.fromSigned((int) Math.round(omega[i] * 16384)));
            // CDUWXFR: angle interface, scaled by the ratio of DAP to CDUW interval
            agc.set(delCdu[i], Agc.fromSigned((int) Math.round(ng[i] * DT_DELT * 16384)));
            // Lag angle = OMEGA|OMEGA| / (2 ACCEL)
            double lag = omega[i] * Math.abs(omega[i]) / 16 / jacc[i];
            agc.set(delEr[i], Agc.fromSigned((int) Math.round(limitsub(DELERLIM, lag) * 16384)));
        }
        tcqcduw();
    }

    /** TCQCDUW: restore the user's EBANK and return (normal and abnormal return). */
    private void tcqcduw() {
        // EBANK restore and return through QCDUWUSR are implicit in the Java call return.
    }

    /** NOATTCNT / NOATTCNT +2: alarm 00402 (optional) and stop all rates, skipping autopilot commands. */
    public void noattcnt(boolean withAlarm) {
        if (withAlarm) {
            env.alarm(ALARM_NO_ATTITUDE_CONTROL);
        }
        env.stoprate();
        tcqcduw();
    }

    /** QUICTRIG for CDUSPOTX / CDUSPOTZ: sines and cosines used for body-rate transformation. */
    private void quictrig(int cduX, int cduZ) {
        double ax = cduAngle(cduX) * Math.PI;
        double az = cduAngle(cduZ) * Math.PI;
        sinCduX = Math.sin(ax);
        cosCduX = Math.cos(ax);
        sinCduZ = Math.sin(az);
        cosCduZ = Math.cos(az);
    }

    /**
     * FLTRSUB: filter one offset component. The increment is gain*(input - offset), limited by
     * DUNFVLIM; the total offset is limited by UNFVLIM.
     */
    private double fltrsub(double offset, double input) {
        double incr = GAINFLTR[ndxcdUw] * (input - offset);
        incr = limitsub(DUNFVLIM, incr);
        return limitsub(UNFVLIM, offset + incr);
    }

    // ---------------------------------------------------------------- single precision subroutines

    /**
     * LIMITSUB: limit value (L) to +-limit (A). Returns clamp(value, -|limit|, |limit|).
     */
    public static double limitsub(double limit, double value) {
        if (value > limit) {
            return limit;
        }
        if (value < -limit) {
            return -limit;
        }
        return value;
    }

    /** LIMITSUB on 15-bit one's complement words (A=limit, L=value). */
    public static int limitsub(int limit, int value) {
        int lim = Math.abs(Agc.toSigned(limit));
        int v = Agc.toSigned(value);
        return Agc.fromSigned(Math.max(-lim, Math.min(lim, v)));
    }

    /** ONESTO2S: convert a one's complement word to two's complement (negative values gain 1). */
    public static int onesTo2s(int a) {
        int v = Agc.toSigned(a);
        return v < 0 ? Agc.fromSigned(v + 1) : Agc.fromSigned(v);
    }

    /**
     * ARCTRGSP: angle (units of PI, range -1..+1) from sine and cosine values (SIN in A, COS in L).
     * The smaller of sin/cos is used for the arcsine to avoid loss of precision;
     * if the sine is zero the result is 0 or PI depending on the sign of the cosine.
     */
    public static double arctrgsp(double sin, double cos) {
        if (sin == 0) { // SINZERO
            return cos >= 0 ? 0 : 1.0;
        }
        if (Math.abs(sin) <= Math.abs(cos)) { // sin is smaller or equal
            double ang = sparcsin(sin);
            return cos >= 0 ? ang : (sin >= 0 ? 1.0 - ang : -1.0 - ang); // ANGLE = PI - ARCSIN(SIN)
        }
        // USECOS: cos is smaller: ANGLE = SIGN(SIN)(PI/2 - ARCSIN(COS))
        double ang = 0.5 - sparcsin(cos);
        return sin >= 0 ? ang : -ang;
    }

    /**
     * SPARCSIN: single precision arcsine returning an angle in units of PI (i.e. asin(x)/PI)
     * using an odd polynomial in x (coefficients DPL1..DPL9 at scale 2^-14).
     */
    public static double sparcsin(double x) {
        // The original evaluates DPL1 + x^2 (DPL3 + x^2 (DPL5 + ...)) in single precision;
        // the Java translation uses the exact function that polynomial approximates.
        return Math.asin(Math.max(-1.0, Math.min(1.0, x))) / Math.PI;
    }
}
