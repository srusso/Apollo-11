package apollo.luminary099;

import apollo.agc.Agc;

/**
 * Translation of P-AXIS_RCS_AUTOPILOT.agc (Luminary 1A build 099).
 *
 * <p>LM digital autopilot P-axis (roll, X body axis) RCS control: the 100 ms T5RUPT entry
 * (PAXIS), KALCMANU/rate-hold interface, rate derivation (state estimator with trap
 * accumulation), offset (AOS) estimation, gimbal-drive timers, Y-X translation, manual
 * minimum-impulse and rate-command modes, P-axis urgency/jet selection (PJETSLEC, SELECTP)
 * and the hand-off to the Q,R-axes autopilot (QRAXIS).
 *
 * <p>Conventions follow the sibling DAP files: 15-bit erasable registers are accessed by
 * name through {@link Agc}; DP quantities use name and name+1. Values read with {@link #g}
 * are raw one's-complement words, {@link #sg} converts them to signed ints. TC/TCF into
 * the middle of the original code become method calls; a "return to caller" is a Java
 * return. Since the original is a tangle of fall-through code, every label becomes a
 * method and each method ends by calling the method that the original falls into or
 * branches to.
 */
public class PAxisRcsAutopilot {
    private static final int BIT1 = 01, BIT3 = 04, BIT4 = 010, BIT7 = 0100, BIT9 = 0400,
            BIT12 = 04000, BIT13 = 010000, BIT14 = 020000, BIT15 = 040000;
    private static final int POSMAX = 037777;
    private static final int NEGMAX = 040000;

    // ---- constants from the source ----
    public static final int MINUS_OCT630 = 077147;      // -OCT630 (45 deg/sec limit, ones complement)
    public static final int ONE_TENTH_SEC = 01;         // 1/10SEC
    public static final int CYC40 = 050;                // 40CYC
    public static final int PQRBIT = 074777;            // PQRBIT: clears bits 10,11 of RCSFLAGS
    public static final int LINRATP = 46;               // LINRATP
    public static final int KAOS = 60;                  // KAOS (DEC 60)
    public static final int MINUS_160MST6 = -256;       // -160MST6
    public static final double MINUS_FOURDEG = -.08888; // -FOURDEG
    public static final int OCT07400 = 07400;           // 07400OCT
    public static final int PLUS_150MST6 = 240;         // +150MST6
    public static final int MINUS_136MST6 = -218;       // -136MST6 (also INDXYZ base)
    public static final int NORMSCL = 0266;             // NORMSCL
    public static final double MINUS_100MS = -.1;       // -100MS
    public static final double MS200 = .2;              // 200MS
    public static final double C25_32 = .78125;         // 25/32 (= PRIO31)
    public static final int BITS8_9 = 00600;            // BITS8,9
    public static final double ONE_40 = .025;           // 1/40
    /** MINTIMES: minimum jet times for +P / -P selection. */
    public static final int[] MINTIMES = {-22, 22};
    /** JETSALL: jet masks for the policy index (ROTINDEX): +P/-P, -Y,-Z,+Y,+Z,+V ... */
    public static final int[] JETSALL = {00252, 00125, 00140, 00006, 00220, 00011, 00151};
    /** TYPEP: jet policy types selected by NUMBERT. */
    public static final int[] TYPEP = {00146, 00226, 00231, 00151, 00132, 00245, 00377};
    /** INDXYZ: ROTINDEX values from the translation controller bits (index by CH31 bits 9-12). */
    public static final int[] INDXYZ = {-218, 4, 2, 07776, 5, 9, 10, 07776, 3, 8, 7,
        07776, 07776, 07776, 07776, 07776};

    private final Agc agc;

    public PAxisRcsAutopilot(Agc agc) {
        this.agc = agc;
    }

    // ---- small helpers --------------------------------------------------------------
    private int g(String s) { return agc.get(s); }
    private void s(String sym, int v) { agc.set(sym, v); }
    private int sg(String sym) { return Agc.toSigned(agc.get(sym)); }
    private void ss(String sym, int signed) { agc.set(sym, Agc.fromSigned(signed)); }
    private boolean chan31Low(int bit) { return (g("CHAN31") & bit) == 0; }   // inverted bits
    private static int clamp(int v) { return Math.max(-POSMAX, Math.min(POSMAX, v)); }

    /** MP: high-order word of the scaled product of two signed single-precision values. */
    private static int mpHi(int a, int b) { return (int) (((long) a * b) / 16384L); }

    /** MP low-order word (L). */
    private static int mpLo(int a, int b) { return (int) (((long) a * b) % 16384L); }

    /** 1STOTWOS: convert a ones-complement value to twos-complement form (negative values get +1). */
    public static int oneStoTwos(int a) {
        return a > 0 ? a : (a == 0 ? 0 : a + 1);
    }

    /** OVERSUB: A unchanged, or limited to POSMAX/NEGMAX if A overflowed. */
    public static int oversub(int a) {
        return clamp(a);
    }

    // =====================================================================
    // PAXIS: T5RUPT entry
    // =====================================================================

    /** PAXIS: T5RUPT entry every 100 ms; reads CDUs and processes the KALCMANU interface. */
    public void paxis() {
        s("TIME5", Agc.add(g("TIME5"), g("MS100")));         // allow synchronisation with other interrupts
        // interrupt lead-in (LXCH BANKRUPT / QXCH QRUPT) is handled by the interrupt layer
        if (sg("DAPZRUPT") > 0) {                            // DAP (JASK) still in progress: restart
            new AlarmAndAbort(agc).bailout(02000);
            return;
        }
        if (((~g("CHAN31")) & (BIT13 | BIT14)) == 0 || ((~g("IMODES33")) & 040) != 0) {
            new DapidlerProgram(agc).chekbits();             // TC CHEKBITS: DAP does not stay on
            return;
        }
        s("DAPTREG4", g("CDUX"));                            // read and store CDUs
        s("DAPTREG5", g("CDUY"));
        s("DAPTREG6", g("CDUZ"));
        kalcmanuInterface();
    }

    /**
     * KALCMANU-DAP and "RATE-HOLD"-DAP interface, executed every 100 ms: CDU desired =
     * CDU desired - (CDU change since last pass) in twos complement, then manual control
     * direct-rate counters are diminished.
     */
    public void kalcmanuInterface() {
        ss("CDUXD", oneStoTwos(sg("CDUXD") - sg("DELCDUX")));
        ss("CDUYD", oneStoTwos(sg("CDUYD") - sg("DELCDUY")));
        ss("CDUZD", oneStoTwos(sg("CDUZD") - sg("DELCDUZ")));
        dim("TCP");                                          // diminish manual control direct rate
        dim("TCQR");                                         // time counters
        paxfilt();                                           // proceeds to RATELOOP after SUPERJOB
    }

    private void dim(String sym) {
        int v = sg(sym);
        if (v > 0) {
            ss(sym, v - 1);
        } else if (v < 0) {
            ss(sym, v + 1);
        }
    }

    /**
     * SUBDIVDE: overflow protection routine giving POSMAX or NEGMAX if the divide would
     * overflow. Entry at "+3" ({@link #subdivdeEntry3}) has OMEGAU already loaded.
     * Result in A; also leaves a copy of the quotient in DAPTEMP5 (hi,lo).
     */
    public int subdivde(int a, int dapTemp3) {
        long prod = (long) a * dapTemp3;                     // MP DAPTEMP3 -> A,L
        long omegau = ((long) sg("OMEGAU") << 14) + sg("OMEGAU+1") + prod;   // DAS OMEGAU
        return subdivdeEntry3(omegau);
    }

    /** SUBDIVDE +3: divide OMEGAU (DP) by DAPTREG4 with overflow limiting. */
    public int subdivdeEntry3(long omegau) {
        int hi = (int) (omegau >> 14);
        int lo = (int) (omegau & 0x3FFF);
        s("DAPTEMP5", Agc.fromSigned(sg("OMEGAU")));         // DXCH DAPTEMP5 (remainder kept)
        s("DAPTEMP5+1", Agc.fromSigned(sg("OMEGAU+1")));
        ss("OMEGAU", hi);
        ss("OMEGAU+1", lo);
        if (hi != 0 && (hi > 0 ? hi + MINUS_OCT630_SIGNED() > 0 : -hi + MINUS_OCT630_SIGNED() > 0)) {
            return hi > 0 ? POSMAX : -POSMAX;                // 45 deg/sec limit
        }
        return divider();
    }

    private static int MINUS_OCT630_SIGNED() { return Agc.toSigned(MINUS_OCT630); }

    /** DIVIDER: divide OMEGAU (DP) by DAPTREG4. */
    public int divider() {
        long num = ((long) sg("OMEGAU") << 14) + sg("OMEGAU+1");
        int den = sg("DAPTREG4");
        if (den == 0) {
            return num >= 0 ? POSMAX : -POSMAX;
        }
        return clamp((int) (num / den));
    }

    // =====================================================================
    // BACKP: rate derivation
    // =====================================================================

    /**
     * BACKP: JETRATE = 1JACC*NO.PJETS*TJP, then the rate derivation: for each body axis
     * OMEGA = OMEGA + JETRATE + AOSTERM (+ TRAPED/NTRAPS if TRAPED is big). OMEGAP,Q,R are
     * body rates scaled at PI/4; TRAPEDP,Q,R are body angle errors from the predicted
     * angle (PI/40); NP(QR)TRAPS the number of accumulations of the angle error.
     */
    public void backp() {
        int a = mpHi(sg("DAPTEMP1"), sg("1JACC"));
        ss("JETRATE", a);

        // ---- X (P) axis ----
        int cdux = sg("DAPTREG4");
        int d = clamp(cdux - sg("OLDXFORP"));                // MSU OLDXFORP (scaled at PI)
        s("OLDXFORP", g("DAPTREG4"));
        ss("DAPTEMP1", d);
        ss("DAPTREG4", (int) (ONE_40 * 16384));              // CA 1/40
        ss("TRAPEDP", clamp(sg("TRAPEDP") + mpHi(-sg("JETRATE"), 1 << 13)));
        ss("TRAPEDQ", clamp(sg("TRAPEDQ") + mpHi(sg("JETRATEQ") + sg("AOSQTERM"), -(1 << 13))));
        ss("TRAPEDR", clamp(sg("TRAPEDR") + mpHi(sg("JETRATER") + sg("AOSRTERM"), -(1 << 13))));

        // ---- Y ----
        int cduy = sg("DAPTREG5");
        int dy = clamp(cduy - sg("OLDYFORP"));
        s("OLDYFORP", g("DAPTREG5"));
        ss("DAPTEMP2", dy);
        int om = mpHi(dy, sg("M11")) + sg("DAPTEMP1");       // M11 scaled at 1
        ss("OMEGAU", om);
        ss("OMEGAU+1", 0);
        int cdurate = subdivdeEntry3(((long) om) << 14);     // CDU rate at PI/4
        ss("TRAPEDP", oversub(sg("TRAPEDP") + cdurate - sg("OMEGAP")));
        ss("DXERROR", clamp(sg("DXERROR") + sg("DAPTEMP5")));
        ss("DXERROR+1", sg("DXERROR+1") + mpHi(-sg("PLAST"), (int) (ONE_40 * 16384)));   // manual X-attitude error (DP)

        // ---- Z ----
        int cduz = sg("DAPTREG6");
        int dz = clamp(cduz - sg("OLDZFORQ"));
        ss("DAPTEMP3", dz);
        s("OLDZFORQ", g("DAPTREG6"));
        ss("OMEGAU", mpHi(sg("M21"), sg("DAPTEMP2")));
        int q = subdivde(sg("M22"), sg("DAPTEMP3"));
        ss("TRAPEDQ", oversub(sg("TRAPEDQ") + q - sg("OMEGAQ")));
        ss("DYERROR", clamp(sg("DYERROR") + sg("DAPTEMP5")));
        ss("DYERROR+1", sg("DYERROR+1") + mpHi(-sg("QLAST"), (int) (ONE_40 * 16384)));   // manual Y-attitude error (DP)
        ss("OMEGAU", mpHi(sg("M31"), sg("DAPTEMP2")));
        int r = subdivde(sg("M32"), sg("DAPTEMP3"));
        ss("TRAPEDR", oversub(sg("TRAPEDR") + r - sg("OMEGAR")));   // TRAPEDS have all been computed
        ss("DZERROR", clamp(sg("DZERROR") + sg("DAPTEMP5")));
        ss("DZERROR+1", sg("DZERROR+1") + mpHi(-sg("RLAST"), (int) (ONE_40 * 16384)));   // manual Z-attitude error (DP)

        // pick up PAD loaded state estimator gains
        if ((g("DAPBOOLS") & g("CSMDOCKD")) == 0) {
            lmOnly();
        } else {                                             // docked
            s("DAPTREG4", g("DKOMEGAN"));
            s("DAPTREG4+1", g("DKOMEGAN+1"));
            s("DAPTREG6", g("DKTRAP"));
            pRate();
        }
    }

    /** LMONLY: undocked state estimator gains. */
    public void lmOnly() {
        s("DAPTREG4", g("LMOMEGAN"));
        s("DAPTREG4+1", g("LMOMEGAN+1"));
        s("DAPTREG6", g("LMTRAP"));
        pRate();
    }

    /** +5 / SMALPDIF / P-RATE: P-axis rate update with trap logic. */
    private void pRate() {
        int trapedp = sg("TRAPEDP");
        if (trapedp != 0) {
            int test = Math.abs(trapedp) + sg("DAPTREG6") * (trapedp > 0 ? 1 : -1);
            // TRAPSIZE > ABOUT 77001 (-1.4 DEG/SEC)
            if (trapedp > 0 && (trapedp + sg("DAPTREG6")) > 0) {
                int quot = sg("NPTRAPS") == 0 ? POSMAX : trapedp * 16384 / Math.max(1, sg("NPTRAPS")) / 16384;
                ss("TRAPEDP", 0);
                ss("OMEGAP", oversub(sg("OMEGAP") + quot));
                s("NPTRAPS", g("DAPTREG4"));                 // about 10 or 0 for docked or undocked
            } else if (trapedp < 0 && test > 0 && (trapedp + sg("DAPTREG6")) > 0) {
                ss("TRAPEDP", 0);
            }
        }
        ss("NPTRAPS", sg("NPTRAPS") + 1);                    // SMALPDIF
        ss("OMEGAP", oversub(sg("OMEGAP") + sg("JETRATE"))); // P-RATE
        qRate();
    }

    /** Q-axis rate update (CCS TRAPEDQ ... Q-RATE). */
    private void qRate() {
        int trapedq = sg("TRAPEDQ");
        if (trapedq > 0 && trapedq + sg("DAPTREG6") > 0) {
            int quot = trapedq / Math.max(1, sg("NQTRAPS"));
            ss("TRAPEDQ", 0);
            ss("DAPTEMP1", quot);                             // save for offset estimate
            ss("OMEGAQ", oversub(sg("OMEGAQ") + quot));
            int a = g("DAPTREG4");
            int old = g("NQTRAPS");
            s("NQTRAPS", a);
            int den = Agc.toSigned(old) + sg("DAPTREG5");     // KAOS > about 60
            int dt1 = sg("DAPTEMP1");
            s("DAPTEMP1", Agc.fromSigned(den));
            if (den != 0) {
                ss("AOSQ", sg("AOSQ") + (dt1 * 5) / den);
            }
        }
        ss("NQTRAPS", sg("NQTRAPS") + 1);                    // Q-RATE
        ss("OMEGAQ", oversub(sg("OMEGAQ") + sg("JETRATEQ") + sg("AOSQTERM")));
        rRate();
    }

    /** R-axis rate update (CCS TRAPEDR ... R-RATE). */
    private void rRate() {
        int trapedr = sg("TRAPEDR");
        if (trapedr > 0 && trapedr + sg("DAPTREG6") > 0) {
            int quot = trapedr / Math.max(1, sg("NRTRAPS"));
            ss("TRAPEDR", 0);
            ss("DAPTEMP2", quot);
            ss("OMEGAR", oversub(sg("OMEGAR") + quot));
            int a = g("DAPTREG4");
            int old = g("NRTRAPS");
            s("NRTRAPS", a);
            int den = Agc.toSigned(old) + sg("DAPTREG5");
            int dt2 = sg("DAPTEMP2");
            s("DAPTEMP2", Agc.fromSigned(den));
            if (den != 0) {
                ss("AOSR", sg("AOSR") + (dt2 * 5) / den);
            }
        }
        ss("NRTRAPS", sg("NRTRAPS") + 1);                    // R-RATE
        ss("OMEGAR", oversub(sg("OMEGAR") + sg("JETRATER") + sg("AOSRTERM")));
        offsetEstimator();                                   // END OF RATE DERIVATION
    }

    /**
     * Offset estimater. In powered flight AOSTASK is called every 2 seconds:
     * AOS = AOS + K*SUMRATE. Without the drift bit the offset values are zeroed.
     */
    public void offsetEstimator() {
        if ((~g("DAPBOOLS") & g("DRIFTBIT")) == 0) {         // drift bit is set: WORKTIME
            workTime();
            return;
        }
        s("ALPHAQ", 0);                                      // zero the offset acceleration values
        s("ALPHAR", 0);
        s("AOSQTERM", 0);
        s("AOSRTERM", 0);
        s("AOSQ", 0);
        s("AOSR", 0);
        pretimck();
    }

    /** WORKTIME: update AOSQ/AOSR from the Q and R acceleration derivatives. */
    public void workTime() {
        int calcode = 032;                                   // OCTAL 00032 is DECIMAL .1 AT 2(6)
        ss("AOSQ", sg("AOSQ") + mpHi(sg("QACCDOT"), calcode));
        s("ALPHAQ", g("AOSQ"));
        ss("AOSQTERM", mpHi(sg("AOSQ"), (int) (MS200 * 16384)));     // .2 AT 1
        ss("AOSR", sg("AOSR") + mpHi(sg("RACCDOT"), calcode));
        s("ALPHAR", g("AOSR"));
        ss("AOSRTERM", mpHi(sg("AOSR"), (int) (MS200 * 16384)));
        pretimck();
    }

    // =====================================================================
    // PAXFILT / SUPERJOB
    // =====================================================================

    /** PAXFILT: execute ACDT+C12 if needed, then set up the SUPERJOB (RATELOOP via DAPZRUPT). */
    public void paxfilt() {
        if ((g("CALLGMBL") & g("RCSFLAGS")) != 0) {
            new TrimGimbalControlSystem(agc).acdtC12();
        }
        // DXCH ARUPT/DAPARUPT, BRUPT, QRUPT/DAPBQRPT, ZRUPT/DAPZRUPT: save the interrupted
        // context; resume in the form of a JASK at SUPERJOB.
        s("DAPARUPT", g("ARUPT"));
        s("DAPBQRPT", g("QRUPT"));
        s("DAPZRUPT", g("ZRUPT"));
        superjob();
    }

    /** SUPERJOB: TCF RATELOOP (backp, the rate derivation, is the first leg). */
    public void superjob() {
        backp();
    }

    /** PRETIMCK: count down gimbal drive timers and turn off drives if required. */
    public void pretimck() {
        int qt = sg("QGIMTIMR");
        if (qt > 0) {
            ss("QGIMTIMR", qt - 1);                           // DECQTIMR
        } else if (qt <= 0 && qt != 0 || g("QGIMTIMR") == 077777) {
            turnoffQ();                                       // negative: drive should be ended
        }
        chkrtimr();
    }

    /** CHKRTIMR: same for the R axis (negative: inactive). */
    public void chkrtimr() {
        int rt = sg("RGIMTIMR");
        if (rt > 0) {
            ss("RGIMTIMR", rt - 1);                           // DECRTIMR
        } else if (rt == 0 && g("RGIMTIMR") == 077777) {
            turnoffR();
        }
        dim("PJETCTR");                                       // decrement docked jet inhibition counters
        dim("UJETCTR");
        dim("VJETCTR");
        if ((BIT12 & g("RCSFLAGS")) == 0) {
            skippaxs();
        } else {
            chkvisfz();
        }
    }

    /** TURNOFFQ: halt the Q gimbal drive. */
    public void turnoffQ() {
        s("NEGUQ", 0);
        s("QACCDOT", 0);
        s("CHAN12", g("CHAN12") & ~(01400));                  // QGIMBITS: bits 9 and 10 of channel 12
        s("QGIMTIMR", NEGMAX);
    }

    /** TURNOFFR: halt the R gimbal drive. */
    public void turnoffR() {
        s("NEGUR", 0);
        s("RACCDOT", 0);
        s("CHAN12", g("CHAN12") & ~(06000));                  // RGIMBITS (=PRIO6): bits 11 and 12
        s("RGIMTIMR", NEGMAX);
    }

    /** SKIPPAXS: set bit 12 of RCSFLAGS and go to QRAXIS or CTS. */
    public void skippaxs() {
        s("RCSFLAGS", g("RCSFLAGS") | BIT12);
        qraxis();
    }

    // =====================================================================
    // Y-X translation
    // =====================================================================

    /**
     * CHKVISFZ: Y-X translation. Input: bits 9-12 of CH31 (translation controller).
     * Output: NEXTP, the channel 6 code of jets for the desired translation. If the
     * desired policy has failures: diagonal translation uses the unfailed pair (alarm if
     * none); principal translation tries to tack with diagonal pairs (alarm if failed).
     */
    public void chkvisfz() {
        int in = (~g("CHAN31")) & OCT07400;
        if (in == 0) {
            tsnextp(0);
            return;
        }
        int idx = (in * BIT7) >> 14;                          // EXTEND; MP BIT7
        s("ROTINDEX", Agc.fromSigned(INDXYZ[idx & 017]));
        tryUorV();
    }

    /** TRYUORV: select Y/Z policy (6 jets) and verify that the choice has no failure. */
    public void tryUorV() {
        selectyz(6);
        int numbert = sg("NUMBERT");
        if (numbert == 6) {
            tsnextp(g("POLYTEMP"));
            return;
        }
        if (sg("ROTINDEX") - 5 <= 0) {                        // ALTERYZ
            alteryz();
            return;
        }
        if (4 - numbert <= 0) {
            tsnextp(g("POLYTEMP"));
            return;
        }
        abortyz();
    }

    /** ABORTYZ: alarm 2001, invert bit 1 of RCSFLAGS, NEXTP = 0. */
    public void abortyz() {
        new AlarmAndAbort(agc).alarm(02001, 0);
        s("RCSFLAGS", g("RCSFLAGS") ^ BIT1);
        tsnextp(0);
    }

    /** ALTERYZ: invert bit 1 of RCSFLAGS and try the alternate Y/Z policy. */
    public void alteryz() {
        s("RCSFLAGS", g("RCSFLAGS") ^ BIT1);
        s("ROTINDEX", g("ROTINDEX") + 4 + (g("RCSFLAGS") & BIT1));
        tryUorV();
    }

    /** TSNEXTP: store the jet code, then the state logic. */
    public void tsnextp(int nextp) {
        s("NEXTP", nextp);
        stateLogic();
    }

    /**
     * STATE LOGIC. Check in order (if on): LPDPHASE goes to PURGENCY; PULSES gives minimum
     * pulse logic; DETENT (bit 15 of CH31) gives rate command; otherwise PURGENCY.
     */
    public void stateLogic() {
        if ((g("CHAN31") & BIT13) == 0) {                     // bit 13 low: stick active (MANMODE)
            manmode();
            return;
        }
        if ((g("DAPBOOLS") & g("XOVINHIB")) != 0) {
            purgency();                                       // attitude steer during visibility phase
            return;
        }
        detentck();
    }

    /** MANMODE: pulse mode (minimum impulse) versus rate command. */
    public void manmode() {
        if ((g("PULSES") & g("DAPBOOLS")) == 0) {
            detentck();                                       // branch for rate command
            return;
        }
        s("PERROR", 0);
        // MINIMUM IMPULSE MODE
        s("CDUXD", g("CDUX"));
        if (sg("OLDPMIN") != 0) {
            checkp();
        } else {
            firep();
        }
    }

    /** FIREP: fire a minimum impulse if the translation controller commands one. */
    public void firep() {
        if (chan31Low(BIT3)) {
            xmin(10);                                         // +XMIN
        } else if (chan31Low(BIT4)) {
            xmin(-10);                                        // -XMIN
        } else {
            jetsoff();
        }
    }

    /** CHECKP: remember the rotation controller state for the next pass. */
    public void checkp() {
        s("OLDPMIN", (~g("CHAN31")) & 014);
        jetsoff();
    }

    /** +XMIN / -XMIN: anything less than 14 ms is corrected in the jet selection routine. */
    public void xmin(int ten) {
        ss("TJP", ten);
        s("OLDPMIN", 1);
        pjetslec(1);                                          // PJETSLEC -6
    }

    // =====================================================================
    // Manual rate command mode (R. F. Stengel)
    // =====================================================================

    /**
     * DETENTCK: manual rate command mode. Provides RCAH manual control through two control
     * laws: 1) direct rate and 2) pseudo-auto. Direct rate fires jets when the stick
     * position changes by a fixed number of increments in one DAP cycle (breakout .6 d/s
     * LM-only, .3 d/s docked) and nulls the rate error within the target deadband.
     * Pseudo-auto uses body-fixed rate and attitude errors with TJETLAW. Control switches
     * from direct rate to pseudo-auto when the target deadband is achieved or after 4 s.
     */
    public void detentck() {
        s("CH31TEMP", g("CHAN31"));
        if ((g("CHAN31") & BIT15) == 0) {                     // out of detent
            rhcmoved();
            return;
        }
        if ((g("OURRCBIT") & g("DAPBOOLS")) == 0) {           // not in rate command last pass
            purgency();
            return;
        }
        if ((g("RCSFLAGS") & BIT9) == 0) {                    // JUST IN DETENT?
            ruth();
            return;
        }
        if (chan31Low(BIT13)) {                               // attitude hold
            ratedamp();
            return;
        }
        s("RCSFLAGS", g("RCSFLAGS") & ~g("BITS9,11"));        // in AUTO: zero orbit (bit 11), just-in (bit 9)
        ratedamp();
    }

    /** RUTH: attitude hold; decide whether rate damping is finished. */
    public void ruth() {
        if ((g("RCSFLAGS") & g("PBIT")) != 0) {
            ratedamp();                                       // P-rate damping not finished
            return;
        }
        if ((g("RCSFLAGS") & g("QRBIT")) == 0) {
            ratedone();                                       // Q,R rate damping finished
            return;
        }
        ratedamp();
    }

    /** RATEDONE: manual command and damping completed in all axes. */
    public void ratedone() {
        s("DAPBOOLS", g("DAPBOOLS") & ~g("OURRCBIT"));
        // read CDUs into the CDU desired registers
        if (chan31Low(BIT13)) {
            new DapInterfaceSubroutines(agc).zatteror();      // via IBNKCALL
            purgency();
            return;
        }
        s("CDUXD", g("CDUX"));                                // (X-axis override)
        s("PERROR", 0);                                       // TS PERROR (after the +3 skip)
        justout();
    }

    /** JUSTOUT: first detection of out of detent; initialization of the first manual pass. */
    public void justout() {
        s("DAPBOOLS", g("DAPBOOLS") + g("OURRCBIT"));
        s("DXERROR", 0);
        s("DXERROR+1", 0);
        s("DYERROR", 0);
        s("DYERROR+1", 0);
        s("DZERROR", 0);
        s("DZERROR+1", 0);
        s("PLAST", 0);
        s("QLAST", 0);
        s("RLAST", 0);
        s("Q-RHCCTR", 0);
        s("R-RHCCTR", 0);
        s("RCSFLAGS", g("RCSFLAGS") & PQRBIT);                // bits 10 and 11 of RCSFLAGS are 0
        s("RCSFLAGS", g("RCSFLAGS") | BIT9);                  // set 'just-in' bit to 1
        zeroenbl();
        jetsoff();
    }

    /** ZEROENBL: zero the hand controller counters and enable them so they can be read next time. */
    public void zeroenbl() {
        s("SAVEHAND+1", g("R-RHCCTR"));
        s("SAVEHAND", g("Q-RHCCTR"));
        s("P-RHCCTR", 0);
        s("Q-RHCCTR", 0);
        s("R-RHCCTR", 0);
        s("CHAN13", g("CHAN13") | BITS8_9);                   // counters zeroed and enabled
    }

    /** RATEDAMP: zero the P hand controller count and compute the rate error. */
    public void ratedamp() {
        s("P-RHCCTR", 0);
        raterror();
    }

    /** RHCMOVED: P control; first manual pass if not yet in rate command. */
    public void rhcmoved() {
        if ((g("OURRCBIT") & g("DAPBOOLS")) == 0) {
            justout();                                        // JUSTOUT -1
            return;
        }
        raterror();
    }

    /** RATERROR: rate command; linear/quadratic stick scaling and direct rate decision. */
    public void raterror() {
        s("CDUXD", g("CDUX"));                                // FINDCDUW requires CDUXD=CDUX in X-axis override
        int p = sg("P-RHCCTR");
        int mag = Math.abs(p) * 2 * 2 + LINRATP;              // linear/quadratic controller scaling
        int a = mpLo(mag, p);                                 // MP P-RHCCTR; CA L
        a = mpHi(a, sg("STIKSENS"));
        int plast = sg("PLAST");
        ss("PLAST", a);
        int delta = plast == 0 ? -a : -plast + a;             // COM; AD PLAST (rate-command change)
        ss("DAPTEMP1", delta);
        zeroenbl();                                           // interval; zero and enable ACA counters
        ss("EDOTP", -plast + sg("OMEGAP"));
        if (Math.abs(delta) - 0 > 0 && Math.abs(delta) - Math.abs(sg("RATEDB")) > 0) {
            ss("TCP", CYC40);                                 // breakout exceeded: direct rate control
            pegi();
            return;
        }
        if ((g("RCSFLAGS") & g("PBIT")) != 0) {              // direct rate command last time?
            pegi();                                           // to pure rate command
            return;
        }
        s("E", g("DXERROR"));                                 // pseudo-auto control: X-attitude error (SP)
        s("PERROR", g("DXERROR"));                            // P-axis error for MODE1 FDAI display
        purgencyAfterCalc();
    }

    /** PEGI: direct rate control. */
    public void pegi() {
        s("CDUXD", g("CDUX"));
        s("DXERROR", 0);
        s("DXERROR+1", 0);
        s("PERROR", 0);                                       // zero P-axis error for FDAI display
        int edotp = sg("EDOTP");
        int abs = Math.abs(edotp);
        ss("ABSEDOTP", abs);
        boolean last = abs + sg("TARGETDB") <= 0 || sg("TCP") <= 0;   // within deadband, or > 4 s
        if (!last) {
            s("RCSFLAGS", g("RCSFLAGS") | g("PBIT"));         // bit 10 is 1
        } else {
            s("RCSFLAGS", g("RCSFLAGS") & ~g("PBIT"));        // LAST: bit 10 is 0
        }
        int a = -edotp;
        a = oversub(a + mpHi(-edotp, sg("1/ANETP")) );        // 1/2JTACC scaled at 2EXP(7)/PI
        int tjp = mpHi(a, (int) (C25_32 * 16384));            // 4.JET TIME
        ss("TJP", tjp);
        if (abs + MINUS_2JETLIM() <= 0) {
            s("NUMBERT_PENDING", 6);
            // CA SIX: jump into PJETSLEC -1 (two-jet) - four jets are not needed
            pjetslecWithNumbert(6);
            return;
        }
        ss("TJP", tjp + tjp);                                 // ADS TJP
        selectNumbertAndPjetslec();
    }

    private int MINUS_2JETLIM() { return sg("-2JETLIM"); }

    // =====================================================================
    // P-jet selection (rotation)
    // =====================================================================

    /**
     * Chooses 4 jets (NUMBERT = 5 if the B system is selected, else 4) from AORBSYST, then
     * PJETSLEC.
     * INPUT: NUMBERT 4,5,6 for which pair or 4 jets; TJP positive for +P rotation.
     * OUTPUT: channel 6, PJUMPADR for the P-axis skip, (JTLST call) for small TJP.
     * Order of policies tried in case of failure: +P 7,15 / 4,12 / 4,7 / 7,12 / 12,15 /
     * 4,15 / ALARM; -P 8,16 / 3,11 / 8,11 / 11,16 / 3,16 / 3,8 / ALARM.
     */
    public void selectNumbertAndPjetslec() {
        int n = ((g("AORBSYST") & g("DAPBOOLS")) != 0) ? 1 : 0;
        pjetslecWithNumbert(4 + n);
    }

    private void pjetslecWithNumbert(int numbert) {
        s("NUMBERT", numbert);
        pjetslec(0);
    }

    /** PJETSLEC: P-axis jet selection and write to channel 6; JTLST call for the remainder. */
    public void pjetslec(int fromMinimum) {
        int tjp = sg("TJP");
        int l = 1;                                            // CA ONE; TS L
        if (tjp == 0) {
            jetsoff();
            return;
        }
        if (tjp < 0) {
            l = 0;                                            // ZL: -P index 0
            tjp = -tjp;
        }
        ss("ABSTJ", tjp);
        s("ROTINDEX", l);
        selectp();
        int no = (sg("NUMBERT") == 6) ? 2 : -2;               // CS SIX; AD NUMBERT; BZF +2 else CS TWO
        ss("NO.PJETS", no + 4);
        writep(g("POLYTEMP"));
        int abstj = sg("ABSTJ");
        if (abstj - PLUS_150MST6 <= 0) {                      // CS ABSTJ; AD +150MST6 <= 0
            qraxis();
            return;
        }
        abstj = abstj - 218;                                  // AD -136MST6
        if (abstj <= 0) {
            ss("TJP", MINTIMES[sg("ROTINDEX") & 1]);          // min time, sign by direction
            // BZMF +5 branch (rest of PJETSLEC): skip the JTLST list
        } else {
            ss("ABSTJ", abstj);
            ss("TJP", MINTIMES[sg("ROTINDEX") & 1]);
            s("T6FURTHA", g("ABSTJ"));
            s("T6FURTHA+1", 0);
            new QRAxisRcsAutopilot(agc).jtlst();              // CADR JTLST
        }
        s("RCSFLAGS", g("RCSFLAGS") & ~BIT12);                // bit 12 set to 0
        altsyst();
        qraxis();
    }

    /** WRITEP: write the jet policy into channel 6 (P-axis jets) and record PJUMPADR. */
    public void writep(int policy) {
        s("CHAN6", policy);
    }

    /** ALTSYST: alternate P-axis jets (toggle AORBSYST in DAPBOOLS). */
    public void altsyst() {
        s("DAPBOOLS", g("DAPBOOLS") ^ g("AORBSYST"));
    }

    /** DKALT: docked alternate system. */
    public void dkalt() {
        altsyst();
    }

    /** JETSOFF: turn off P jets (WRITEP -1), zero TJP, go to QRAXIS. */
    public void jetsoff() {
        writep(0);
        s("TJP", 0);
        qraxis();
    }

    /**
     * CALCPERR: P-error calculation. PERROR / E = (CDUY - CDUYD) * M11 + (CDUX - CDUXD) +
     * DELPEROR (KALCMANU interface error); M13 = 1 identically implies null multiplication.
     */
    public int calcperr() {
        int d = sg("CDUY") - sg("CDUYD");
        int e = mpHi(clamp(d), sg("M11"));                    // scaled at PI radians
        d = sg("CDUX") - sg("CDUXD");
        e = e + d + sg("DELPEROR");                           // could be overflow
        ss("E", e);
        s("PERROR", g("E"));                                  // for eight-ball display
        return e;
    }

    /** P-axis urgency function calculation. */
    public void purgency() {
        calcperr();
        purgencyAfterCalc();
    }

    /** PURGENCY +4: common to LM DAP and SPS-backup mode. EDOTP = OMEGAP - OMEGAPD at PI/4. */
    public void purgencyAfterCalc() {
        ss("EDOTP", sg("OMEGAP") - sg("OMEGAPD"));
        ss("AXISCTR", -1);
        if ((g("DAPBOOLS") & g("CSMDOCKD")) != 0) {          // docked: DOCKED RCS LOGIC
            new SpsBackUpRcsControl(agc).spsrcs();
            if (sg("TJP") == 0) {
                dkalt();                                      // TJP zero: change AORBSYST
                return;
            }
            pjetslec(1);                                      // select AORBSYST and use two jets
            return;
        }
        s("SENSETYP", 0);                                     // HEADTJET
        new TjetLaw(agc).tjetlaw();
        if (-sg("FIREFCT") + (int) (MINUS_FOURDEG * 16384) <= 0) {
            pjetslec(1);
            return;
        }
        int tjp = sg("TJP");
        if (tjp == 0) {
            jetsoff();
            return;
        }
        if (Math.abs(tjp) + MINUS_160MST6 <= 0) {
            pjetslec(1);
            return;
        }
        pjetslecWithNumbert(6);                               // PJETSLEC -1
    }

    // =====================================================================
    // SELECTP: jet policy construction
    // =====================================================================

    /**
     * SELECTP: JET POLICY CONSTRUCTION SUBROUTINE. INPUT: ROTINDEX, NUMBERT. OUTPUT:
     * POLYTEMP (jet policy). Selects the subset of the desired jets which has no failure.
     */
    public void selectp() {
        selectpBody(6);
    }

    /** SELECTYZ: set NUMBERT = A and run SELECTP +1. */
    public void selectyz(int numbert) {
        s("NUMBERT", numbert);
        selectpBody(6);
    }

    private void selectpBody(int tempnum) {
        int numbert = sg("NUMBERT");
        int rot = sg("ROTINDEX");
        int policy = TYPEP[numbert & 7 % TYPEP.length] & JETSALL[rot & 7 % JETSALL.length];
        s("POLYTEMP", policy);
        if ((policy & g("CH6MASK")) == 0) {                   // no failed jets in the policy
            return;
        }
        if (tempnum > 1) {                                    // CCS TEMPNUM
            s("TEMPNUM", tempnum - 1);
            ss("NUMBERT", numbert - 1);                       // try the next policy
            selectpBody(tempnum - 1);
            return;
        }
        new AlarmAndAbort(agc).alarm(02003, 0);
        jetsoff();                                            // ********** TCF ALARMJET **********
    }

    // =====================================================================
    // QRAXIS: hand-off to the Q,R-axes RCS autopilot
    // =====================================================================

    /** QRAXIS: EDOTR = OMEGAR - OMEGARD, EDOTQ = OMEGAQ - OMEGAQD (limited), then CALLQERR via DTCB. */
    public void qraxis() {
        ss("EDOTR", oversub(sg("OMEGAR") - sg("OMEGARD")));
        ss("EDOTQ", oversub(sg("OMEGAQ") - sg("OMEGAQD")));
        new QRAxisRcsAutopilot(agc).callqerr();               // QERRCALL 2CADR CALLQERR (EBANK= AOSQ)
    }
}
