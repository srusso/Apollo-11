package apollo.comanche055;

import apollo.agc.Agc;

/**
 * AUTOMATIC_MANEUVERS (Comanche055, pp 1025-1036): RCS DAP automatic attitude control for the
 * CSM (attitude hold, rate damping, minimum impulse, phase-plane jet-switching logic).
 *
 * Fractions are 14-bit scaled signed integers (AGC single precision); double-precision words
 * are held as hi/lo erasable pairs (symbol, symbol+1).
 */
public final class AutomaticManeuvers {
    private static final int BIT11 = 02000;
    private static final int BIT13 = 010000;
    private static final int BIT14 = 020000;
    private static final int OCT16000 = 016000;
    private static final int OCT01400 = 01400;
    private static final int POSMAX = 037777;
    private static final int NEGMAX = 040000;

    /** MINTAU table: +0, +14ms, -14ms, +0. */
    public static final int[] MINTAU = {0, 23, -23, 0};

    // Jet switching logic constants
    public static final double WLH_SLOP = .00463;     // WL+H/SLOPE = .83333 deg, $180
    public static final double WL_H_SLP = .00277;     // WL-H/SLOPE = .5 deg, $180
    public static final double WLH = .0011111111;     // WL+H = 0.5 deg/sec, $450
    public static final double WLMH = .0006666666;    // WL-H = 0.3 deg/sec
    public static final double WL = .0008888888;      // WL = 0.4 deg/sec
    public static final double SLOPE2 = .32;          // 0.8 deg/sec/deg

    private final Agc agc;

    public AutomaticManeuvers(Agc agc) {
        this.agc = agc;
    }

    private int s(String sym) {
        return Agc.toSigned(agc.get(sym));
    }

    private void w(String sym, int v) {
        agc.set(sym, Agc.fromSigned(v));
    }

    /** Fractional multiply hi word: (a*b)/2^14 */
    private static int mp(int a, int b) {
        return (int) (((long) a * b) >> 14);
    }

    /** Read a double-precision pair as a signed value in 2^-28 units. */
    private long dp(String sym) {
        return ((long) s(sym) << 14) + s(sym + "+1");
    }

    private void putDp(String sym, long v) {
        w(sym, (int) (v >> 14));
        w(sym + "+1", (int) (v & 0x3FFF));
    }

    private static long dpConst(double frac) {
        return Math.round(frac * (1L << 28));
    }

    /** AHFNOROT: attitude hold, no rotation commanded. */
    public void ahfnorot() {
        int chan31 = agc.get("CHAN31");
        if ((chan31 & BIT14) == 0) {                       // BZMF -> FREECONT (bit14 clear = free)
            freecont();
            return;
        }
        if ((agc.get("RCSFLAGS") & BIT14) == 0) {          // rate filter not initialised
            // TCF REINIT: recycle to initialise filter
            return;
        }
        if ((agc.get("CHAN31") & BIT13) == 0) {            // not AUTO -> HOLDFUNC
            holdfunc();
            return;
        }
        autocont();
    }

    /** AUTOCONT: HOLDFLAG +: GRABANG, otherwise ATTHOLD. */
    public void autocont() {
        if (s("HOLDFLAG") <= 0) {
            atthold();
        } else {
            grabang();
        }
    }

    /** FREECONT: minimum impulse control. */
    public void freecont() {
        agc.set("HOLDFLAG", 1);
        int l = agc.get("CHAN32");
        int chantemp = (~l) & agc.get("MANROT") & agc.get("CHANTEMP");
        agc.set("CHANTEMP", chantemp);
        // TC STICKCHK (sets RMANNDX, PMANNDX, YMANNDX)
        agc.set("TAU", MINTAU[agc.get("RMANNDX") & 3]);
        agc.set("TAU1", MINTAU[agc.get("PMANNDX") & 3]);
        agc.set("TAU2", MINTAU[agc.get("YMANNDX") & 3]);
        t6progm();
    }

    /** ATTHOLD: attitude errors AK = AMGB (CDU - THETAD) + BIAS. */
    public void atthold() {
        int errorX = Agc.toSigned(Agc.add(agc.get("CDUX"), ~agc.get("THETADX") & Agc.WORD_MASK));
        int t = Agc.toSigned(Agc.add(agc.get("CDUY"), ~agc.get("THETADY") & Agc.WORD_MASK));
        errorX += mp(t, s("AMGB1"));
        int errorY = mp(t, s("AMGB4"));
        int errorZ = mp(t, s("AMGB7"));
        t = Agc.toSigned(Agc.add(agc.get("CDUZ"), ~agc.get("THETADZ") & Agc.WORD_MASK));
        errorY += mp(t, s("AMGB5"));
        errorZ += mp(t, s("AMGB8"));
        if (s("HOLDFLAG") > 0) {                           // biases only during automatic maneuvers
            errorX += s("BIAS");
            errorY += s("BIAS1");
            errorZ += s("BIAS2");
        }
        w("ERRORX", errorX);
        w("ERRORY", errorY);
        w("ERRORZ", errorZ);
        jets();
    }

    /** HOLDFUNC: CCS HOLDFLAG: >0 falls to GRABANG (+1), 0 -> ATTHOLD. */
    public void holdfunc() {
        int hf = s("HOLDFLAG");
        if (hf > 0) {
            grabang();
        } else {
            atthold();
        }
    }

    /** GRABANG: zero WBODYs and biases; end damping when rate damping complete. */
    public void grabang() {
        for (String n : new String[] {"WBODY", "WBODY+1", "WBODY1", "WBODY1+1", "WBODY2", "WBODY2+1",
                "BIAS", "BIAS1", "BIAS2"}) {
            agc.set(n, 0);
        }
        if ((agc.get("RCSFLAGS") & OCT16000) == 0) {       // rate damping completed
            enddamp();
            return;
        }
        agc.set("ERRORX", 0);
        agc.set("ERRORY", 0);
        agc.set("ERRORZ", 0);
        jets();
    }

    /** ENDDAMP: HOLDFLAG = +0, pick up CDU angles as attitude hold references. */
    public void enddamp() {
        agc.set("HOLDFLAG", 0);
        agc.set("THETADX", agc.get("CDUX"));
        agc.set("THETADY", agc.get("CDUY"));
        agc.set("THETADZ", agc.get("CDUZ"));
        atthold();
    }

    /**
     * JETS: phase-plane switching logic for axes SPNDX = 2 (yaw), 1 (pitch), 0 (roll).
     * Computes in KMPAC the desired change in rate for each axis, then JTIME converts to jet time.
     */
    public void jets() {
        int t5temp = s("ADB") + 4;                         // ADB + AF (flat region .044 deg)
        w("T5TEMP", t5temp);
        for (int spndx = 2; spndx >= 0; spndx--) {
            agc.set("SPNDX", spndx);
            int dpndx = spndx * 2;
            agc.set("DPNDX", dpndx);
            long edot = dp("ADOT" + (dpndx == 0 ? "" : "+" + dpndx));   // DCA ADOT indexed
            if (s("HOLDFLAG") != 0) {                      // not in hold: EDOT = ADOT - WBODY
                edot -= dp("WBODY" + (dpndx == 0 ? "" : "+" + dpndx));
            }
            putDp("EDOT", edot);
            int aerr = s(spndx == 0 ? "ERRORX" : spndx == 1 ? "ERRORY" : "ERRORZ");
            w("AERR", aerr);
            long edotvel;
            int adbvel;
            int aerrvel;
            if (edot >= 0) {                               // POSVEL
                edotvel = edot;
                adbvel = t5temp;
                aerrvel = aerr;
            } else {                                       // NEGVEL
                edotvel = -edot;
                adbvel = -t5temp;
                aerrvel = -aerr;
            }
            putDp("EDOTVEL", edotvel);
            w("ADBVEL", adbvel);
            w("AERRVEL", aerrvel);
            int region = j6(edot, edotvel, aerr, aerrvel, adbvel, t5temp);
            if (region == 0) {
                continue; // DOJET+2 (do nothing; TAU was left unchanged for this axis)
            }
            jtime(spndx, region == 1);
        }
        t6prog();
    }

    /**
     * Region logic J6..J24. Returns 0 for "no action" (J23 with no forced firing),
     * otherwise stores desired change in rate in KMPAC and returns 1 (JTIME).
     */
    private int j6(long edot, long edotvel, int aerr, int aerrvel, int adbvel, int t5temp) {
        int adb = s("ADB");
        int slope = s("SLOPE");
        int wlhSlop = (int) Math.round(WLH_SLOP * 16384);
        int wlmhSlp = (int) Math.round(WL_H_SLP * 16384);
        if (aerrvel - adb + wlhSlop <= 0) {                // J6 -> J8 when AERRVEL - ADB + WLH/SLOP <= 0
            return j8(edot, edotvel, aerrvel, t5temp);
        }
        if (-t5temp + aerrvel > 0) {                       // J7
            if (-wlmhSlp - t5temp + aerrvel <= 0) {
                return j20(aerr, adbvel, edot);
            }
            return j21(edot);
        }
        long q = slope == 0 ? 0 : -edotvel / slope;        // DCS EDOTVEL; DV SLOPE
        if (q - aerrvel + adb <= 0) {
            return j18(edot);
        }
        return j23();
    }

    private int j8(long edot, long edotvel, int aerrvel, int t5temp) {
        long wtemp = -dpConst(WLH) + edotvel;              // EDOTVEL - WLH
        if (wtemp > 0 || wtemp == 0) {
            return j22(edot);
        }
        int slope = s("SLOPE");
        long v = (slope == 0 ? 0 : edotvel / slope) + t5temp + aerrvel;
        if (v > 0) {
            return j23();
        }
        wtemp = -dpConst(WLMH) + edotvel;
        if (wtemp >= 0) {
            return j23();
        }
        int a = aerrvel + t5temp + (int) Math.round(WL_H_SLP * 16384);
        if (a > 0) {
            return j24(edot);
        }
        return j22(edot);
    }

    /** J18: KMPAC = -EDOT. */
    private int j18(long edot) {
        putDp("KMPAC", -edot);
        return 1;
    }

    /** J20: KMPAC = SLOPE2*(ADBVEL - AERR) - EDOT. */
    private int j20(int aerr, int adbvel, long edot) {
        long k = (long) mp(adbvel - aerr, (int) Math.round(SLOPE2 * 16384));
        putDp("KMPAC", (k << 14) - edot);
        return 1;
    }

    /** J21: JP if EDOT >= 0 else JN. */
    private int j21(long edot) {
        return edot >= 0 ? jp(edot) : jn(edot);
    }

    /** JN: KMPAC = -EDOT + WL. */
    private int jn(long edot) {
        putDp("KMPAC", -edot + dpConst(WL));
        return 1;
    }

    /** JP: KMPAC = -EDOT - WL. */
    private int jp(long edot) {
        putDp("KMPAC", -edot - dpConst(WL));
        return 1;
    }

    /** J22: JN if EDOT > 0 else JP. */
    private int j22(long edot) {
        return edot > 0 ? jn(edot) : jp(edot);
    }

    /** J23: reset rate damping flag for the axis; force a firing if requested. */
    private int j23() {
        int spndx = agc.get("SPNDX");
        int bit = BIT13 >> spndx;                          // BIT13 roll, BIT12 pitch, BIT11 yaw (reset)
        agc.set("RCSFLAGS", agc.get("RCSFLAGS") & ~bit);
        int force = (OCT01400 >> spndx) & agc.get("RCSFLAGS");
        if (force == 0) {
            return 0;                                      // DOJET+2
        }
        return j18(dp("EDOT"));                            // force a firing
    }

    /** J24: KMPAC = SLOPE2*(-AERR - ADBVEL) - EDOT. */
    private int j24(long edot) {
        int adbvel = s("ADBVEL");
        int aerr = s("AERR");
        long k = (long) mp(-aerr - adbvel, (int) Math.round(SLOPE2 * 16384));
        putDp("KMPAC", (k << 14) - edot);
        return 1;
    }

    /**
     * JTIME: jet-on time TAU = J/M * delta W (scaled 16384/1600 sec).
     * J/M, J/M1, J/M2 are the S/C inertia/torque ratios for roll/pitch/yaw.
     */
    private void jtime(int spndx, boolean dummy) {
        int jm = s(spndx == 0 ? "J/M" : "J/M" + spndx);
        // SMALLMP twice: KMPAC (delta W, DP) * J/M * BIT11
        long k = dp("KMPAC");
        long prod = (k * jm) >> 14;
        prod = (prod * BIT11) >> 14;
        int tau;
        if (prod > 037777) {
            tau = POSMAX;                                  // overflow positive
        } else if (prod < -037777) {
            tau = NEGMAX;                                  // overflow negative
        } else {
            tau = Agc.fromSigned((int) prod);
        }
        dojet(spndx, tau);
    }

    /** DOJET: TAU (indexed by SPNDX) = result. */
    private void dojet(int spndx, int tau) {
        agc.set(spndx == 0 ? "TAU" : spndx == 1 ? "TAU1" : "TAU2", tau);
    }

    /** ZEROCMDS: TAU = TAU1 = TAU2 = 0. */
    public void zerocmds() {
        agc.set("TAU", 0);
        agc.set("TAU1", 0);
        agc.set("TAU2", 0);
        t6prog();
    }

    /** T6PROG (and T6PROGM): set T5LOC to JETSLECT (phase 3) and RESUME. */
    public void t6prog() {
        agc.set("T5LOC", 0); // 2CADR JETSLECT
        // TCF RESUME
    }

    public void t6progm() {
        t6prog();
    }
}
