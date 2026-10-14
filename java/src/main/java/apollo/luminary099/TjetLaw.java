package apollo.luminary099;

import apollo.agc.Agc;

/**
 * TJET_LAW (Luminary 099): RCS jet-firing-time computation for the LM digital autopilot.
 * Designed by R. D. Goss and P. S. Weissman; coded 28 Feb 1968. Bank 17 at DAPS2, EBANK = TJP.
 *
 * TJETLAW is called as a subroutine when the LEM is not docked and the autopilot is in AUTOMATIC or
 * ATTITUDE-HOLD mode, to compute the jet firing time TJET for the axis indicated by AXISCTR
 * (-1 = P-axis, +0 = U-axis, +1 = V-axis). E and EDOT are the axis attitude error and error rate;
 * SENSETYP tells whether unbalanced couples are preferred. The sign of the required rotation is
 * carried as ROTSENSE and applied to TJET just before it is stored in TJP/TJU/TJV.
 * Simple algorithms RUFLAW1/2/3 are used when the error and/or error rate are large.
 *
 * Inputs: E, EDOT, AXISCTR, SENSETYP, TJP/-U/-V; 1/ACCONT tables starting at BLOCKTOP
 * (FIREDB, AXISDIST, COASTDB, 1/ACOAST, 1/ANET1, 1/ANET2, ACCFCTZ1, ACCFCTZ5, FLAT, ZONE3LIM, ACCSWU).
 * Outputs: TJP, TJU or TJV, NUMBERT (DAPTEMP5), FIREFCT (DAPTEMP3).
 *
 * Arithmetic is carried out on the erasable words interpreted as fractions of a 15-bit word
 * (scaling comments from the original are kept next to each step).
 */
public final class TjetLaw {
    private final Agc agc;

    // ---- constants for TJETLAW ----
    /** AXISDIFF(INDEX): registers between stored 1/ACCS parameters for the indexed axis and the U-axis. */
    public static final int[] AXISDIFF = {-16, 0, 16};
    /** SENSOR: ratio of TJET scaling in TJETLAW (4 sec) to T6 scaling (10.24 sec): OCT 14400. */
    public static final double SENSOR = 06400 / 16384.0;
    /** -3 degrees scaled at 45. */
    public static final double M3DEG = -0.06667;
    /** -.01125 sec^2 scaled at 8. */
    public static final double MP0112A8 = -0.00141;
    public static final double P1AT4 = 0.025;       // 0.1 s scaled at 4
    public static final double P1AT2 = 0.05;        // 0.1 s scaled at 2
    public static final double P0375AT4 = 0.00938;  // .0375 s scaled at 4
    public static final double MP025AT2 = -0.0125;  // -.025 s scaled at 2
    public static final double MP025AT4 = -0.00625; // -.025 s scaled at 4
    public static final double MP05AT2 = -0.025;
    public static final double MP15AT2 = -0.075;
    /** .00375 sec^2 scaled at 8 (2DEC .00375 B-3). */
    public static final double P00375A8 = 0.00375 / 8.0;
    /** -TJMAX: largest calculated time, .150 sec at 4. */
    public static final double MTJMAX = -0.0375;
    /** TJMIN: smallest allowable time, .020 sec at 4. */
    public static final double TJMIN = 0.005;
    public static final double MTJMIN = -0.005;
    /** RUFRATE: corresponds to target rate of 6.5 deg/s. */
    public static final double RUFRATE = 0.1444;

    private static final double BIT5 = 16 / 16384.0;
    private static final double BIT6 = 32 / 16384.0;
    private static final double BIT11 = 1024 / 16384.0;
    private static final double BIT12 = 2048 / 16384.0;
    private static final double NEG2 = -2 / 16384.0;
    private static final double NEGMAX = -16383 / 16384.0;
    private static final double M1_8 = -0.125;

    // ---- working registers (HOLDQ, ADRSDIF1/2, ROTSENSE, EDOTSQ, TTOAXIS, HH) ----
    private int adrsdif1;
    private int adrsdif2;
    private double rotsense;
    private double e;
    private double edot;
    private double edotsq;
    private double ttoaxis;
    private double hh;

    public TjetLaw(Agc agc) {
        this.agc = agc;
    }

    // ---- erasable access helpers ----
    private double sp(String sym) {
        return Agc.toSigned(agc.get(sym)) / 16384.0;
    }

    private double sp(String sym, int idx) {
        String key = idx == 0 ? sym : sym + (idx > 0 ? "+" : "") + idx;
        return Agc.toSigned(agc.get(key)) / 16384.0;
    }

    private void setSp(String sym, double v) {
        agc.set(sym, Agc.fromSigned((int) Math.round(v * 16384.0)));
    }

    /** TJETLAW entry. Computes TJET for the axis AXISCTR; result stored at TJP/TJU/TJV. */
    public void tjetlaw() {
        int axisctr = Agc.toSigned(agc.get("AXISCTR"));
        e = sp("E");
        edot = sp("EDOT");
        // SET INDEXERS TO CORRESPOND TO THE AXIS AND TO THE SIGN OF EDOT
        adrsdif1 = AXISDIFF[axisctr + 1];
        if (edot > 0) {
            adrsdif2 = adrsdif1;
            rotsense = SENSOR;               // positive EDOT: ROTSENSE positive
        } else {
            // NEGEDOT: complement E and EDOT, reverse sense, bump indexers by one
            e = -e;
            edot = -edot;
            adrsdif1 += 1;
            adrsdif2 = adrsdif1;
            rotsense = -SENSOR;
        }
        setSp("E", e);
        setSp("EDOT", edot);
        // TEST MAGNITUDE OF E (scaled at PI): >= PI/16 goes to the simplified routines
        if (e >= 1.0 / 16) {
            ruflaw2();
            return;
        }
        if (e <= -1.0 / 16) {
            ruflaw1();
            return;
        }
        // SCALEE: rescale to PI/4
        e = e * 4;
        setSp("E", e);
        // TEST MAGNITUDE OF EDOT (scaled PI/4 rad/sec): >= PI/32 rad/sec -> RUFLAW3
        if (Math.abs(edot) >= 1.0 / 8) {
            ruflaw3();
            return;
        }
        // *** FINELAW STARTS HERE ***
        edot = edot * 8;                     // SCALEDOT: EDOT scaled at PI/32 rad/sec
        setSp("EDOT", edot);
        edotsq = edot * edot / 4;            // scaled at PI^2/2^8 rad^2/sec^2
        agc.set("EDOTSQ", Agc.fromSigned((int) Math.round(edotsq * 16384)));
        // ERRTEST: does a big error (3 deg beyond the deadband) require maximum jets?
        double a = Math.abs(e) + M3DEG - sp("FIREDB", adrsdif1);
        boolean maxjets;
        if (a <= 0) {
            // SENSTEST: are unbalanced jets preferred?
            int sensetyp = Agc.toSigned(agc.get("SENSETYP"));
            maxjets = sensetyp == 0;
            if (!maxjets) {
                tjcalc(Math.abs(sensetyp) - 1);       // +0,1 for (U,V-axes) jet select
                return;
            }
        }
        // MAXJETS: increment address locator and set jet select switch to 4 (always for P-axis)
        adrsdif2 += 2;
        tjcalc(4);
    }

    /** TJCALC: beginning of the TJET calculations. */
    private void tjcalc(int numbert) {
        agc.set("NUMBERT", numbert);
        // FIREFCT = -E - .5(EDOTSQ)/ACC - DB at PI/4 radian
        double firefct = -edotsq * sp("1/ANET1", adrsdif2) + sp("FIREDB", adrsdif1) - e;
        setSp("FIREFCT", firefct);
        if (firefct <= 0) {
            zon123(firefct);
            return;
        }
        // ZONE4,5: test E + .5(EDOTSQ)/ACC + DB at PI/4 radian
        double f = sp("1/ACOAST", adrsdif1) * edotsq + e + sp("COASTDB", adrsdif1);
        if (f <= 0) {
            zone5(f);
            return;
        }
        zone4(firefct);
    }

    /**
     * ZONE4 is the coast region. However, if the jets are on and driving toward the axis within
     * +-(DB+FLAT) (drifting flight) or the usual target parabola (powered flight), they are kept on.
     */
    private void zone4(double firefct) {
        int axisctr = Agc.toSigned(agc.get("AXISCTR"));
        double tjetCur = sp("TJP", axisctr + 1);     // TJETU indexed by AXISCTR
        if (-tjetCur * rotsense <= 0) {
            coasttj();                               // jets not firing toward the desirable state
            return;
        }
        // JETSON: drifting (FLAT > 0) or powered/ullage?
        if (sp("FLAT") > 0) {
            // DRIFT/ON: can the target strip of the axis be reached from here?
            if (-2 * sp("FIREDB", adrsdif1) + firefct <= 0) {
                if (z123comp()) {
                    fulltime();
                    return;
                }
                zone23();
            } else {
                coasttj();
            }
            return;
        }
        // POWERED (or ULLAGE): can the target parabola be reached from this point?
        if (-firefct + sp("AXISDIST", adrsdif1) <= 0) {
            coasttj();
            return;
        }
        if (z123comp()) {
            fulltime();
            return;
        }
        zone1(firefct);
    }

    /** ZONE5: compute TJET by approximations (formulas 1 and 2) or fire full time. */
    private void zone5(double f) {
        if (rotsense < 0) {
            adrsdif2 -= 2;                           // access 1/ANET2 and ACCFCTZ5 from the lower registers
        }
        hh = f * sp("ACCFCTZ5", adrsdif2) * 4;       // double precision H scaled at 8 sec^2
        ttoaxis = sp("1/ANET2", adrsdif2) * edot;    // scaled at 4 sec
        // TEST WHETHER TJET GREATER THAN 50 MSEC: H - .05 TTOAXIS - .00125 > 0
        if (ttoaxis * MP05AT2 + hh + NEG2 <= 0) {
            // FORMULA1: TJET = H/(.025 + TTOAXIS) for TJET less than 50 msec
            double tjet = hh / (-MP025AT4 + ttoaxis) * 0.5;
            chkmintj(tjet);
            return;
        }
        // TEST WHETHER TJET GREATER THAN 150 MSEC: H - .15 TTOAXIS - .01125 > 0
        if (ttoaxis * MP15AT2 + hh + MP0112A8 <= 0) {
            // FORMULA2: TJET = (H + .00375)/(0.1 + TTOAXIS) for TJET greater than 50 msec
            hh += P00375A8;
            double tjet = hh / (ttoaxis + P1AT4) * 0.5;
            returntj(tjet);
            return;
        }
        fulltime();   // greater than 150 msec: 250 msec is enough to assure no skip next CSP
    }

    /** FULLTIME: 250 msec scaled at 4 sec. */
    private void fulltime() {
        returntj(BIT11);
    }

    /** COASTTJ: TJET = 0. */
    private void coasttj() {
        returntj(0);
    }

    /**
     * RETURNTJ: all branches end here with TJET (scaled at 4 sec). ROTSENSE applies the sign and
     * changes the scale to T6's; sets NUMBERT to 4 if 1/ACCS forced a max-jet calculation for this sense.
     */
    private void returntj(double tjet) {
        int axisctr = Agc.toSigned(agc.get("AXISCTR"));
        double signed = tjet * rotsense;
        String key = axisctr + 1 == 0 ? "TJP" : "TJP" + "+" + (axisctr + 1);   // TJP, TJU, TJV consecutive
        int word = Agc.fromSigned((int) Math.round(signed * 16384.0));
        agc.set(key, word);
        int accsw = Agc.toSigned(agc.get(axisctr == 0 ? "ACCSWU" : "ACCSWU" + (axisctr > 0 ? "+" : "") + axisctr));
        if ((long) Agc.toSigned(word) * accsw > 0) {
            agc.set("NUMBERT", 4);
        }
        // TC HOLDQ : return to the caller
    }

    /** Z123COMP: preparation for zones 1, 2, 3. Returns true if the caller must go to FULLTIME. */
    private boolean z123comp() {
        rotsense = -rotsense;                        // rescale TJET as TIME6 with proper sign
        ttoaxis = edot * sp("1/ANET1", adrsdif2);    // time-to-axis scaled at 4 sec
        return ttoaxis + MTJMAX > 0;                 // time to axis not less than 150 msec: fire jets
    }

    /** ZON1,2,3. */
    private void zon123(double firefct) {
        if (z123comp()) {
            fulltime();
            return;
        }
        // If the (negative) distance beyond the parabola is less than FLAT use special logic
        // to acquire the minimum impulse limit cycle (FLAT = 0 during powered flight or ullage).
        double a = firefct + sp("FLAT");
        if (a <= 0) {
            zone1(a);
        } else {
            zone23();
        }
    }

    /** ZONE2,3: fire for axis or, if close, fire minimum impulse. If on axis, coast. */
    private void zone23() {
        if (-sp("ZONE3LIM") + ttoaxis <= 0) {
            // ZONE3: one-jet minimum impulse (EDOT cannot be negative here)
            returntj(edot > 0 ? BIT6 : 0);
        } else {
            returntj(ttoaxis);                       // ZONE2: fire to axis
        }
    }

    /** ZONE1: TJET from H using formula 3 or the over-50-msec approximation. */
    private void zone1(double a) {
        hh = (a - sp("AXISDIST", adrsdif1)) * sp("ACCFCTZ1", adrsdif2) * 4;   // H scaled at 8 sec^2
        // Is .5(.150 - TTOAXIS)^2 - H negative (scaled at 8 sec^2)? Then need full time.
        double d = ttoaxis + MTJMAX;
        if (d * d - hh <= 0) {
            fulltime();
            return;
        }
        if (hh + NEG2 <= 0) {
            // FORMULA3: TJET = H/.025 + TTOAXIS for approximation over less than 50 msec
            chkmintj(hh / -MP025AT2 + ttoaxis);
        } else {
            // TJET = H/0.1 + TTOAXIS + .0375 for approximation over more than 50 msec
            returntj(hh / P1AT2 + ttoaxis + P0375AT4);
        }
    }

    /** CHKMINTJ: if computed jet time is less than TJMIN, TJET is set to zero. */
    private void chkmintj(double tjet) {
        if (tjet + MTJMIN <= 0) {
            coasttj();
        } else {
            returntj(tjet);
        }
    }

    // ------------------------------------------------------------------ ROUGHLAW

    /**
     * RUFLAW1: error more negative than PI/16 rad. Fire to a rate of 6.5 deg/sec (if jet time exceeds 20 msec).
     */
    private void ruflaw1() {
        edot -= RUFRATE;                             // target rate
        if (edot <= 0) {
            // SMALRATE: set NUMBERT and FIREFCT for maximum jets
            rufsetupNoReverse();
            adrsdif2 += rotsense > 0 ? 1 : (rotsense < 0 ? -1 : 0);   // point to proper 1/ANET sense
            ruflaw12(-edot);                         // (.1444 - EDOT) = desired rate change
            return;
        }
        rufsetup();                                  // reverse ROTSENSE and indicate max jets
        ruflaw12(edot);                              // desired rate change
    }

    /** RUFLAW12: TJET = (desired rate change)/(2-jet accel). */
    private void ruflaw12(double a) {
        double t = a * sp("1/ANET1", adrsdif2 + 2);
        t += M1_8;                                   // if TJET (scaled 32 sec) exceeds 4 sec, set to TJMAX
        if (t > 0 || t == 0) {
            fulltime();
            return;
        }
        t += BIT12;                                  // restore computed TJET
        chkmintj(t * 8);                             // rescaled TJET at 4 seconds
    }

    /** RUFLAW2: error more positive than PI/16 rad. Fire to an opposing rate of 6.5 deg/sec. */
    private void ruflaw2() {
        rufsetup();
        double a = RUFRATE + edot;                   // (.1444 + EDOT) = desired rate change
        if (a >= 1.0 || a <= -1.0) {                 // overflow: fire for full time
            fulltime();
            return;
        }
        ruflaw12(a);
    }

    /**
     * RUFLAW3: error rate greater than PI/32 rad/sec and error within bounds. Coast if below
     * FIREFCT switch curve, fire if above.
     */
    private void ruflaw3() {
        rufsetup();
        double a = (-sp("FIREDB", adrsdif1) + e) * BIT11;    // distance from switch curve
        double old = edot;
        edot = a;
        double sq = old * old;
        sq = sq * sp("1/ANET1", adrsdif1 + 2);
        if (sq + edot <= 0) {
            coasttj();                               // coast if below it
        } else {
            fulltime();                              // fire for full period if above it
        }
    }

    /** RUFSETUP: reverse ROTSENSE, require maximum (2) jets in U,V; suggest 4 jets in P-axis. */
    private void rufsetup() {
        rotsense = -rotsense;
        rufsetupNoReverse();
    }

    /** RUFSETUP +2. */
    private void rufsetupNoReverse() {
        agc.set("NUMBERT", 4);
        setSp("FIREFCT", NEGMAX);
    }
}
