package apollo.luminary099;

import apollo.agc.Agc;

/**
 * ASCENT_GUIDANCE.agc - LM powered ascent guidance (P12). ATMAG updates the thrust
 * acceleration estimate, ASCENT computes velocity-to-be-gained and the steering vector, with
 * engine cutoff (ENGOFF/CUTOFF/TERMASC) and the LOGSUB logarithm subroutine.
 *
 * The interpretive-language arithmetic is expressed on doubles holding the AGC fractions;
 * shift operations (SLn, SRn) are written as explicit scalings by 2^n so the original scaling
 * comments carry over.  Vectors are stored with {@link VecOps}.
 */
public class AscentGuidance {
    private final Agc agc;

    // constants (2DEC n B-k = n * 2^-k)
    public static final double SIN5DEG = 0.08716 / 4;     // 0.08716 B-2
    public static final double FPS40 = 0.12192 / 128;     // 40FPS: 0.12192 B-7
    public static final double SEC4_17 = 400.0 / (1 << 17);   // 4SEC(17)
    public static final double SEC2_17 = 200.0 / (1 << 17);   // 2SEC(17), also T2A
    public static final double T2A = SEC2_17;
    public static final double T3 = 1000.0 / (1 << 17);
    public static final double SEC6_18 = 600.0 / (1 << 18);
    public static final double SEC2_9 = 200.0 / (1 << 9);
    public static final double CS100 = 200.0 / (1 << 18); // 100CS EQUALS 2SEC(18)
    public static final double KT1 = 0.5;
    public static final double PRLIMIT = -0.0639;         // (B/TBUP)MIN = -.1 FT/SEC^3
    public static final double MINABDV = 0.0356 / 32;     // .0356 B-5: 10% bigger than gravity
    public static final double NEARONE = 1.0 - Math.pow(2, -28);
    public static final double CLOG2_32 = 0.0216608494;
    public static final int BIT3H = 4;                    // OCT 4
    public static final int BIT4H = 8;                    // OCT 10
    public static final int V06N63 = 663, V06N76 = 676, V06N33A = 633;
    public static final int V16N63 = 1663, V16N85C = 1685;
    /** LOGSUB polynomial coefficients (TC POLY, DEC 6): constant term first. */
    public static final double[] LOG_POLY = {
        0.0000000060, -0.0312514377, -0.0155686771, -0.0112502068,
        -0.0018545108, -0.0286607906, 0.0385598563, -0.0419361902};

    public AscentGuidance(Agc agc) {
        this.agc = agc;
    }

    private double dp(String s) { return VecOps.getDp(agc, s); }
    private void put(String s, double v) { VecOps.setDp(agc, s, v); }
    private double[] vec(String s) { return VecOps.getV(agc, s); }
    private boolean flag(String s) { return agc.get(s) != 0; }
    private void setFlag(String s, boolean v) { agc.set(s, v ? 1 : 0); }

    // ---------------------------------------------------------------- ATMAG
    /** ATMAG: ascent acceleration magnitude filter: updates 1/DV table, TBUP and AT. */
    public void atMag() {
        // TC PHASCHNG ; OCT 00035
        if (flag("FLRCS")) {                       // BON FLRCS ASCENT
            ascent();
            return;
        }
        double abdv = dp("ABDVCONV");
        if (abdv - MINABDV < 0) {                  // BMN ASCTERM4: no guidance this cycle
            ascTerm4();
            return;
        }
        setFlag("SURFFLAG", false);
        setFlag("RENDWFLG", false);
        double inv = BIT3H / abdv;                 // SLOAD BIT3H ; DDV ABDVCONV
        // shift the 1/DV history: 1/DV3 <- 1/DV2 <- 1/DV1 <- 1/DV0 <- new
        put("1/DV3", dp("1/DV2"));
        put("1/DV2", dp("1/DV1"));
        put("1/DV1", dp("1/DV0"));
        put("1/DV0", inv);
        double sum = dp("1/DV0") + dp("1/DV1") + dp("1/DV2") + dp("1/DV3");
        double x = sum * dp("VE") * SEC2_9 * 8;    // DMP DMP, SL3
        double tbup = x + dp("TBUP") / 2 - SEC6_18; // PDDL TBUP, SR1, DAD, DSU
        put("TBUP", tbup);
        put("AT", (dp("VE") / 2) / tbup);          // SR1 DDV TBUP -> AT
        ascent();
    }

    // ---------------------------------------------------------------- ASCENT
    /** ASCENT: main ascent-guidance cycle. */
    public void ascent() {
        double[] r = vec("R");
        double[] v = vec("V");
        double[] ur = vec("UNIT/R/");
        double[] z = vec("ZAXIS1");
        double rmag = VecOps.abs(r);
        put("/R/MAG", rmag);
        double zdot = VecOps.dot(z, v) * 2;        // ZDOT*2(-7)
        put("ZDOT", zdot);
        double[] laxis = VecOps.scale(VecOps.cross(z, ur), 2);   // LAXIS*2(-1)
        VecOps.setV(agc, "LAXIS", laxis);
        double ydot = VecOps.dot(laxis, v) * 2;
        put("YDOT", ydot);
        ycomp();
        double gr = VecOps.dot(VecOps.scale(vec("GDT1/2"), 1.0 / SEC2_9 * 0 + CS100), ur); // G.UR*2(9)
        double[] hr = VecOps.cross(ur, v);
        double h2r3 = VecOps.dot(hr, hr) / rmag;   // H(2)/R(3)*2(9)
        double geff = gr + h2r3 * 2;               // SL1, DAD: GEFF*2(10)
        put("GEFF", geff);
        double dzdot = dp("ZDOTD") - zdot;
        put("DZDOT", dzdot);
        double dydot = dp("YDOTD") - ydot;
        put("DYDOT", dydot);
        double drdot = dp("RDOTD") - dp("RDOT");
        put("DRDOT", drdot);
        // VG = (DRDOT)R + (DVDOT)L + (DZDOT)Z
        double[] vg = VecOps.scale(VecOps.add(VecOps.add(VecOps.scale(z, dzdot), VecOps.scale(laxis, dydot)),
                VecOps.scale(ur, drdot)), 2);
        double tgo = dp("TGO");
        vg = VecOps.sub(vg, VecOps.scale(VecOps.scale(ur, tgo * geff), 2)); // compensated for GEFF
        VecOps.setV(agc, "VGVECT", vg);
        VecOps.setV(agc, "VGBODY", VecOps.scale(VecOps.mxv(VecOps.getM(agc, "XNBPIP"), vg), 2)); // N85 display
        double vgmag = VecOps.abs(vg);
        if (flag("FLRCS")) {                       // use TGO = VG/AT with RCS
            put("TGO", vgmag / dp("AT/RCS"));
            ascTerm2();
            return;
        }
        mainEng(vgmag);
    }

    /** MAINENG: time-to-go with tail-off compensation and steering rate computation. */
    private void mainEng(double vgmag) {
        double vgve = vgmag / dp("VE");
        double tbup = dp("TBUP");
        double tgo = tbup * vgve * (NEARONE - KT1 * vgve) - dp("TTO"); // compensate for tailoff
        put("TGO", tgo);
        put("TTOGO", -tgo / 2048);                 // SR 11D ; DCOMP: TGO*2(-28) CS
        if (!flag("IDLEFLAG") && tgo - SEC4_17 < 0) {
            engOff();                              // BMN ENGOFF
            return;
        }
        t2Test(tgo);
    }

    /** T2TEST: below T2 use constant-acceleration (CMPONENT); otherwise compute B, D rates. */
    private void t2Test(double tgo) {
        double tbup = dp("TBUP");
        if (tgo - T2A < 0) {
            cmponent();
            return;
        }
        double l = logSub(1 - tgo / tbup) ;        // 1 - TGO/TBUP ; LOGSUB gives -L/32
        double minusL = l * 32;                    // SL 5: -L in PDL(0)
        double d12 = tbup - tgo / minusL;          // TBUP + TGO/L = D12*2(-17)
        if (!flag("FLPC")) {
            if (tgo - T3 >= 0) {
                rates(tgo, d12, minusL);
                return;
            }
            setFlag("FLPC", true);
        }
        put("PRATE", 0);                           // NORATES: B = 0
        put("YRATE", 0);                           // D = 0
        constant(d12, minusL);
    }

    private void rates(double tgo, double d12, double minusL) {
        double d21 = tgo - d12;                    // D21*2(-17)
        double e = (tgo - 2 * d21) * 8;            // E * 2(-13)
        double dr = tgo * dp("RDOT") + dp("/R/MAG") - dp("RCO"); // -DR *2(-24) as in listing
        double prate = ((d21 * dp("DRDOT") + (-dr)) * 4) / e / tgo; // B * 2(8)
        double tbup = dp("TBUP");
        if (prate >= 0) {                          // B>0 NOT PERMITTED
            prate = 0;
        } else if (prate / 16 / tbup - PRLIMIT < 0) { // CHKBMAG: (B/TAU) = 2(21) MAX
            prate = PRLIMIT * tbup * 16;           // B MAX
        }
        put("PRATE", prate);
        double dy = tgo * dp("YDOT") + dp("Y") - dp("YCO");  // Y + YDOT TGO - YCO
        double yrate = ((d21 * dp("DYDOT") - dy) * 4) / e / tgo;
        put("YRATE", yrate);
        constant(d12, minusL);
    }

    /** CONST: A and C constants of the linear acceleration profile. */
    private void constant(double d12, double minusL) {
        double prate = dp("PRATE");
        double yrate = dp("YRATE");
        double a = -dp("DRDOT") / minusL / 4 - d12 * prate; // (-DRDOT/L - D12 B) = A*2(-9)
        put("PCONS", a);
        double c = -dp("DYDOT") / minusL / 4 - d12 * yrate; // (-DYDOT/L - D12 D) = C*2(-9)
        put("YCONS", c);
        cmponent();
    }

    /** CMPONENT: radial (ATR) and out-of-plane (ATY) thrust-acceleration components, then steering. */
    private void cmponent() {
        double tbup = dp("TBUP");
        double ur[] = vec("UNIT/R/");
        double[] laxis = vec("LAXIS");
        double atr = ((CS100 * dp("PRATE") + dp("PCONS")) / tbup) * 2 - dp("GEFF"); // ATR*2(9)
        put("ATR", atr);
        double aty = ((CS100 * dp("YRATE") + dp("YCONS")) / tbup) * 2;              // ATY*2(9)
        put("ATY", aty);
        double[] ah = VecOps.scale(VecOps.add(VecOps.scale(laxis, aty), VecOps.scale(ur, atr)), 2); // AH*2(9)
        double ah2 = VecOps.dot(ah, ah);
        double at = dp("AT");
        double atp2 = at * at - ah2;               // ATP2*2(18)
        double atp3 = at * at - ah2;               // ATP3 (AT(2)KR(2) - AH(2))
        double atp;
        if (atp3 < 0) {                            // NO-ATP: scale AH so |AH| = AT, ATP = 0
            double kh = at / Math.sqrt(ah2);
            ah = VecOps.scale(ah, kh);
            atp = 0;
        } else {
            atp = Math.sqrt(atp2);                 // ATP*2(9)
            atp = dp("DZDOT") < 0 ? -atp : atp;    // SIGN DZDOT
        }
        put("ATP", atp);
        double[] unfc = VecOps.scale(VecOps.add(VecOps.scale(vec("ZAXIS1"), atp), ah), 2);
        VecOps.setV(agc, "UNFC/2", unfc);          // AT*2(0)
        if (flag("FLPI")) {
            // P12RET: continue in the P12 return path
            return;
        }
        if (flag("FLVR")) {
            checkAlt();
            return;
        }
        mainLine();
    }

    /** MAINLINE: set the desired pointing vectors; terminate when past TXO. */
    private void mainLine() {
        double[] ur = vec("UNIT/R/");
        VecOps.setV(agc, "UNWC/2", VecOps.scale(ur, -1));
        if (dp("TXO") - dp("PIPTIME") >= 0) {      // DSU PIPTIME BPL ASCTERM
            ascTerm();
            return;
        }
        if (flag("ROTFLAG")) {
            ang1Chek();
            return;
        }
        clrXFlag();
    }

    private void clrXFlag() {
        setFlag("NOR29FLG", false);                // start R29 in ascent phase
        setFlag("XOVINFLG", false);                // allow X-axis override
        ascTerm();
    }

    /** ASCTERM: compute CDU-W (FINDCDUW) unless RCS trimming, then display N63. */
    public void ascTerm() {
        if (!flag("FLRCS")) {
            // CALL FINDCDUW -2 (interpretive call)
        }
        ascTerm1();
    }

    private void ascTerm1() {
        if (flag("FLRCS") || flag("FLUNDFLG")) {   // bypass display in RCS trimming or engine failure
            ascTerm3();
            return;
        }
        // CAF V06N63* ; BANKCALL GODSPR
        ascTerm3();
    }

    public void ascTerm2() {
        ascTerm3();
    }

    /** ASCTERM3: TCF ENDOFJOB. */
    public void ascTerm3() {
        // TCF ENDOFJOB
    }

    /** ASCTERM4: no guidance this cycle; zero DAP commanded errors (ZATTEROR). */
    public void ascTerm4() {
        // INHINT ; IBNKCALL ZATTEROR
        ascTerm1();
    }

    /** CHECKALT: below 25K ft check Z-axis orientation. */
    private void checkAlt() {
        double h = dp("/R/MAG") - dp("/LAND/");
        if (h - dp("25KFT") < 0) {
            checkYaw();
            return;
        }
        exitVr();
    }

    private void exitVr() {
        setFlag("FLVR", false);
        if (flag("ROTFLAG")) {
            mainLine();
            return;
        }
        put("TXO", dp("PIPTIME") + dp("10SECS"));
        mainLine();
    }

    /** EXITVR1: clear ROTFLAG then EXITVR. */
    private void exitVr1() {
        setFlag("ROTFLAG", false);
        exitVr();
    }

    /** ANG1CHEK: check the thrust vector against body X axis limits (COSTHET1/COSTHET2). */
    private void ang1Chek() {
        double[] unfc = vec("UNFC/2");
        double[] xnb = VecOps.getV(agc, "XNBPIP");
        if (VecOps.dot(unfc, xnb) - dp("COSTHET1") >= 0
                || VecOps.dot(xnb, vec("UNIT/R/")) - dp("COSTHET2") < 0) {
            setFlag("ROTFLAG", false);             // OFFROT
            clrXFlag();
            return;
        }
        keepVr1();
    }

    /** CHECKYAW (also SETXFLAG): vertical-rise yaw check. */
    private void checkYaw() {
        setFlag("XOVINFLG", true);                 // prohibit X-axis override
        double[] los = VecOps.add(VecOps.scale(vec("LAXIS"), dp("ATY")), VecOps.scale(vec("ZAXIS1"), dp("ATP")));
        double[] u = VecOps.unit(los);
        VecOps.setV(agc, "LOSVEC", u);
        if (Math.abs(VecOps.dot(u, vec("YNBPIP"))) - SIN5DEG >= 0) {
            keepVr(u);
            return;
        }
        if (dp("RDOT") - FPS40 >= 0) {
            exitVr1();
            return;
        }
        keepVr(u);
    }

    private void keepVr(double[] los) {
        VecOps.setV(agc, "UNWC/2", los);
        keepVr1();
    }

    private void keepVr1() {
        VecOps.setV(agc, "UNFC/2", vec("UNIT/R/"));
        ascTerm();
    }

    /** ENGOFF: schedule engine shutoff at the computed time (ENGOFF1) and set IDLEFLAG. */
    private void engOff() {
        double dt = dp("TTOGO") + 0;               // TTOGO - (PIPTIME - now)
        int engoffDt = (int) Math.max(1, Math.round(dt * (1L << 28)) & 0x3FFF);
        agc.set("ENGOFFDT", engoffDt);
        // TWIDDLE ENGOFF1 ; PHASCHNG 47014 ... ; SET IDLEFLAG
        setFlag("IDLEFLAG", true);                 // disable delta-V monitor
        t2Test(dp("TGO"));
    }

    /** ENGOFF1: waitlist task: shut off the engine (ENGINOF2) and schedule CUTOFF. */
    public void engOff1() {
        // IBNKCALL ENGINOF2 ; FINDVAC PRIO17 CUTOFF ; PHASCHNG 07024 / 17000
        cutoff();
    }

    /** CUTOFF: set FLRCS; flash V16N63 and then continue to TERMASC or CUTOFF1. */
    public void cutoff() {
        setFlag("FLRCS", true);                    // UPFLAG FLRCS
        // CAF V16N63 ; BANKCALL GOFLASH: terminate -> TERMASC, proceed -> CUTOFF1, enter -> redisplay
        cutoff1();
    }

    /** CUTOFF1: zero attitude errors before reducing deadband. */
    public void cutoff1() {
        // IBNKCALL ZATTEROR ; IBNKCALL SETMINDB
        cutoff2();
    }

    /** CUTOFF2: flash V16N85C (VG body); proceed -> TERMASC. */
    public void cutoff2() {
        // PHASCHNG 04024 ; CAF V16N85C ; GOFLASH
        termAsc();
    }

    /** TERMASC: restore astronaut's deadband, disallow aborts and exit via GOTOPOOH. */
    public void termAsc() {
        // PHASCHNG 04024 ; IBNKCALL RESTORDB
        setFlag("LETABORT", false);                // DOWNFLAG LETABORT
        // TCF GOTOPOOH
    }

    /** YCOMP: Y = (UNIT/R/ . QAXIS) * 4 * RCO. */
    public void ycomp() {
        double y = VecOps.dot(vec("UNIT/R/"), vec("QAXIS")) * 4 * dp("RCO");
        put("Y", y);
    }

    // ---------------------------------------------------------------- LOGSUB
    /**
     * LOGSUB: input X in (0,1); returns -LOG(X) scaled 1/32 (the original computes it by
     * normalizing X and evaluating a 7th order polynomial plus the exponent times ln2/32).
     */
    public static double logSub(double x) {
        return -Math.log(x) / 32.0;
    }
}
