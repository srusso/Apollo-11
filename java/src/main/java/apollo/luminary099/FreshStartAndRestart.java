package apollo.luminary099;

import apollo.agc.Agc;

/**
 * FRESH_START_AND_RESTART.agc (Luminary 099, pp. 211-237).
 * SLAP1 (fresh start from PINBALL), GOPROG (hardware restart), STARTSUB/STARTSB1/STARTSB2
 * (initialisation common to both), MR.KLEAN/P00KLEAN/V37KLEAN, LIGHTSET, GOTOPOOH,
 * V37 (major mode change), V37XEQ, the V37 tables (FCADRMM1, PREMM1, DNLADMM1) and the
 * P00 state-vector integration entry (STATEUP / THISVINT).
 *
 * Channel writes are modelled as agc.set("CHANnn", ...) / {@link #wand}/{@link #wor}.
 */
public class FreshStartAndRestart {
    private final Agc agc;

    // ---- constants ----
    public static final int BIT4 = 0x8, BIT5 = 0x10, BIT6 = 0x20, BIT8 = 0x80, BIT10 = 0x200,
            BIT11 = 0x400, BIT12 = 0x800, BIT13 = 0x1000, BIT14 = 0x2000, BIT15 = 0x4000;
    public static final int BITS4_6 = 050;                 // BITS4&6
    public static final int OCT10000 = BIT13, OCT30000 = 030000, OCT7777 = 07777;
    /** STIKSTRT: 20 deg/s maximum companded rate, 0.825268. */
    public static final double STIKSTRT = 0.825268;
    public static final int RATESTRT = -218;
    public static final int BOOLSTRT = 021312;
    public static final int OCT_77001 = 077001;            // .14 deg scaled at 4.5 deg
    public static final int DEC_60 = 60;
    public static final int RSFLGBTS = 020100;
    public static final int MAXDB = 03434;                 // 5 deg attitude deadband, scaled at 45
    public static final int IFAILINH = 0435;
    public static final int OCT32001 = 032001, OCT27470 = 027470, OCT74160 = 074160, OCT30001 = 030001;
    public static final int NUMGRPS = 5;
    public static final int NEG_ELR = -022;                // -ERROR LIGHT RESET key code
    public static final int IM30INIF = 037411, IM30INIR = 037000;
    public static final int IM33INIT = 030;                // = PRIO16
    public static final int OCT_450 = 0450;                // 9,6,4
    public static final int RMODINIT = 0102;
    public static final int LTHVACA = 44;
    /** SWINIT: initial switch words (flagwords STATE..STATE+11). */
    public static final int[] SWINIT = {0, 0, 0, 02000 /* BIT 11 = NOR29FLG */, 0, 0, 0, 0100,
            0, 0, 0, 040000 /* BIT 15 = LRBYPASS */};
    public static final int OCT_500 = 0500, OCT_305 = 0305, OCT_26 = 026, P20REG = 0124, OCT_700 = 0700,
            OCT_37667 = 037667, OCT_40072 = 040072;
    public static final int DEC_70 = 70;
    public static final int ALARM_PHASE_TABLE = 01107;     // PTBAD
    public static final int ALARM_NO_MM_CHANGE = 01520;    // CANTROD

    /** FCADRMM1: starting jobs of each keyboard major mode, highest MM first. */
    public static final String[] FCADRMM1 = {"P79", "P78", "P76", "P75", "P74", "P73", "P72", "LANDJUNK",
            "P63LM", "P57", "PROG52", "P51", "P47LM", "P42LM", "P41LM", "P40LM", "P39", "P38", "P35", "P34",
            "P33", "P32", "P31", "P30", "PROG25", "PROG22", "PROG21", "PROG20", "P12LM", "P06"};
    /** PREMM1: PPPPPEEEMMMMMMM per MM: 5 priority bits, 3 E-bank bits, 7 major-mode bits. */
    public static final int[] PREMM1 = {027717, 027716, 027714, 027713, 027712, 027711, 027710, 027704,
            027677, 027271, 027264, 027263, 027657, 027652, 027651, 027650, 027647, 027646, 027643, 027642,
            027641, 027640, 027637, 027636, 027631, 027626, 027625, 027624, 027614, 027006};
    /** NOV37MM: number of entries in each table minus 1. */
    public static final int NOV37MM = 29;
    /** Downlist codes. */
    public static final int COSTALIN = 0, AGSUPDAT = 1, RENDEZVU = 2, ORBMANUV = 3, DESASCNT = 4, LUNRSALN = 5;
    public static final int DNLADP00 = 0;
    /** DNLADMM1 per mode (same order as FCADRMM1). */
    public static final int[] DNLADMM1 = {RENDEZVU, RENDEZVU, RENDEZVU, RENDEZVU, RENDEZVU, RENDEZVU, RENDEZVU,
            DESASCNT, DESASCNT, LUNRSALN, COSTALIN, COSTALIN, ORBMANUV, ORBMANUV, ORBMANUV, ORBMANUV,
            RENDEZVU, RENDEZVU, RENDEZVU, RENDEZVU, RENDEZVU, RENDEZVU, RENDEZVU, RENDEZVU, RENDEZVU,
            LUNRSALN, RENDEZVU, RENDEZVU, DESASCNT, COSTALIN};

    public FreshStartAndRestart(Agc agc) {
        this.agc = agc;
    }

    private static int s(int w) { return Agc.toSigned(w); }

    private void wand(String chan, int mask) { agc.set(chan, agc.get(chan) & mask); }

    private void wor(String chan, int mask) { agc.set(chan, agc.get(chan) | mask); }

    private void zero(String... syms) { for (String x : syms) { agc.set(x, 0); } }

    private void clearBits(String word, int bits) { agc.set(word, agc.get(word) & ~bits); }

    // ------------------------------------------------------------------ fresh start

    /** SLAP1: fresh start. Comes here from PINBALL. */
    public void slap1() {
        // INHINT
        startSub();                                     // does most of the work
        // STARTSW: TCF SKIPSIM (patch: TCF STARTSIM for simulation -> FINDVAC BIT14 SIM2CADR)
        skipSim();
    }

    /** SKIPSIM: turn off all DSPTAB+11D lamps except gimbal lock and no att on requested fresh start. */
    private void skipSim() {
        agc.set("DSPTAB+11", (agc.get("DSPTAB+11") & BITS4_6) + BIT15);
        agc.set("DUMPCNT", BIT12);                      // downlink erasable dump for one pass
        zero("ERCOUNT", "FAILREG", "FAILREG+1", "FAILREG+2", "REDOCTR");
        agc.set("DSRUPTSW", Agc.fromSigned(-030));      // CS PRIO12
        doFStart();
    }

    /** DOFSTART: ensure the engine is off. */
    public void doFStart() {
        wor("DSALMOUT", BIT14);                         // engine off
        agc.set("THRUST", Agc.fromSigned(-0));          // CS ZERO -> -0
        doFStrt1();
    }

    /** DOFSTRT1: fresh start without touching the engine - DAP, IMU, switch initialisation. */
    public void doFStrt1() {
        agc.set("RCSFLAGS", 4);                         // attitude error displays
        agc.set("RESTREG", 030);                        // PRIO30 super bank priority for displays
        zero("ABDELV", "NVSAVE", "EBANKTEM", "CH5MASK", "CH6MASK", "PVALVEST",
                "ERESTORE", "SMODE", "DNLSTCOD", "AGSWORD", "UPSVFLAG");
        agc.set("CHAN5", 0);                            // turn off RCS jets
        agc.set("CHAN6", 0);
        agc.set("CHAN12", 0);
        agc.set("CHAN13", 0);
        agc.set("CHAN14", 0);
        // IMU was in coarse align in gimbal lock? put it back into coarse align.
        if ((~agc.get("DSPTAB+11") & BITS4_6) != 0) {
            wor("CHAN12", BITS4_6);
        }
        mrKlean();
        agc.set("MODREG", Agc.fromSigned(-0));
        agc.set("IMODES30", IM30INIF);                  // fresh start IMU initialisation
        agc.set("DB", MAXDB);
        agc.set("RATEINDX", 4);                         // KALCMANU rate
        agc.set("DAPBOOLS", BOOLSTRT);
        agc.set("EBANK", 6);                            // EBANK6, EBANK=HIASCENT
        agc.set("STIKSENS", (int) Math.round(STIKSTRT * 16384));
        agc.set("-RATEDB", Agc.fromSigned(RATESTRT));
        agc.set("HIASCENT", agc.get("FULLAPS"));        // max ascent mass until pad load
        agc.set("DKTRAP", OCT_77001);                   // DAP filter gains: .14 deg
        agc.set("LMTRAP", OCT_77001);
        agc.set("DKKAOSN", DEC_60);                     // 6 sec gain for alpha
        agc.set("LMKAOSN", DEC_60);
        agc.set("LMOMEGAN", 0);                         // unity gain
        agc.set("DKOMEGAN", 10);                        // 1 sec gain for omega
        agc.set("DKDB", BIT8);                          // docked DB 1.4 deg; pad load may overwrite
        agc.set("IMODES33", IM33INIT + BIT6);           // DAP and error needles off until ICDU zero
        // Initialise switches only on fresh start (SWINIT -> STATE words).
        agc.set("STATE", SWINIT[0]);
        agc.set("STATE+1", SWINIT[1]);
        agc.set("STATE+2", SWINIT[2]);
        agc.set("STATE+3", (agc.get("STATE+3") & agc.get("REFSMBIT")) + SWINIT[3]); // keep REFSMFLG
        agc.set("STATE+4", SWINIT[4]);
        agc.set("STATE+5", SWINIT[5]);
        agc.set("STATE+6", SWINIT[6]);
        agc.set("STATE+7", SWINIT[7]);
        int keep8 = agc.get("SURFFBIT") + agc.get("CMOONBIT") + agc.get("LMOONBIT"); // SURFFLAG, CMOONFLG, LMOONFLG
        agc.set("STATE+8", (agc.get("STATE+8") & keep8) + SWINIT[8]);
        agc.set("STATE+9", SWINIT[9]);
        agc.set("STATE+10", (agc.get("STATE+10") & agc.get("APSFLBIT")) + SWINIT[10]); // keep APSFLAG
        agc.set("STATE+11", SWINIT[11]);
        endRstrt();
    }

    /** ENDRSTRT: POSTJUMP to DUMMYJOB+2 (picks up at RELINT; does not zero NEWJOB). */
    public void endRstrt() {
        // TC POSTJUMP; CADR DUMMYJOB +2
    }

    /** MR.KLEAN: clear all restart groups (-PHASE2 first) ; P00KLEAN: group 4; V37KLEAN: 1,3,5,6. */
    public void mrKlean() {
        // INHINT
        agc.set("-PHASE2", Agc.fromSigned(-0));
        agc.set("PHASE2", Agc.fromSigned(-0));
        p00Klean();
    }

    public void p00Klean() {
        agc.set("-PHASE4", Agc.fromSigned(-0));
        agc.set("PHASE4", Agc.fromSigned(-0));
        v37Klean();
    }

    public void v37Klean() {
        for (int g : new int[] {1, 3, 5, 6}) {
            agc.set("-PHASE" + g, Agc.fromSigned(-0));
            agc.set("PHASE" + g, Agc.fromSigned(-0));
        }
    }

    // ------------------------------------------------------------------ restart (GOPROG)

    /** GOPROG: from location 4000 (GOJAM) - restart any programs running at the time. */
    public void goProg() {
        agc.set("REDOCTR", Agc.add(agc.get("REDOCTR"), 1));
        // LXCH Q; EXTEND ROR SUPERBNK; DXCH RSBBQ: save Q and superbank.
        agc.set("RSBBQ", agc.get("Q"));
        if ((agc.get("DSPTAB+11") & BIT4) != 0) {
            wor("CHAN12", BIT6);                          // ISS was in coarse align: error counter enable
        }
        if (lightSet()) {
            return;                                      // fresh start forced (BUTTONS)
        }
        // ERASCHK recovery: ERESTORE must be +0 or a positive number below 2000 octal.
        int erestore = agc.get("ERESTORE");
        if ((erestore & ~03777) != 0 && erestore != 0) {
            nonAvKey3();                                 // E memory might be bad: fresh start
            return;
        }
        if (erestore != 0) {
            if (erestore != agc.get("SKEEP7")) {
                nonAvKey3();
                return;
            }
            agc.set("EBANK", agc.get("SKEEP4"));         // EBANK of memory under test
            // DCA SKEEP5 / DXCH 0(SKEEP7): E memory restored
            agc.set("ERESTORE", 0);
        }
        startSub();                                      // DORSTART
        setInfl();
    }

    /** SETINFL: clear INTFLBIT; keep alarm lamps; restore IMU inhibits; set engine state. */
    public void setInfl() {
        clearBits("FLGWRD10", agc.get("INTFLBIT"));
        agc.set("DSPTAB+11", (agc.get("DSPTAB+11") & OCT_450) + BIT15);
        agc.set("IMODES30", (agc.get("IMODES30") & IFAILINH) + IM30INIR);
        agc.set("DNLSTCOD", agc.get("AGSWORD"));
        wor("CHAN14", BIT4);                              // throttle counter / thrust drive on
        if ((~agc.get("FLAGWRD5") & agc.get("ENGONBIT")) != 0) {
            wor("DSALMOUT", BIT13);                       // engine on
        } else {
            wor("DSALMOUT", BIT14);                       // engine off
        }
        goProg3();
    }

    /** ENEMA: INHINT; STARTSB1; then GOPROG2A. */
    public void enema() {
        startSb1();
        goProg2a();
    }

    /** GOPROG2: STARTSB2 and continue. (Do not use GOPROG2 or ENEMA without consulting POOH people.) */
    public void goProg2() {
        startSb2();
        goProg2a();
    }

    private void goProg2a() {
        lightSet();
        clearBits("FLGWRD10", RSFLGBTS);                  // clear bits 7 and 14
        goProg3();
    }

    /** GOPROG3: verify phase table agreement, display MM, run active restart groups. */
    public void goProg3() {
        for (int g = NUMGRPS; g >= 0; g--) {
            int minus = agc.get("-PHASE" + (g + 1));
            int direct = agc.get("PHASE" + (g + 1));
            if (((~minus) ^ direct) != 0 && (minus ^ direct) != 0x7FFF) { // must be -0 for agreement
                ptBad();
                return;
            }
            if (g == 0) { break; }
        }
        agc.set("MPAC+6", 0);
        // TC MMDSPLAY: display major mode
        clearBits("FLAGWRD1", agc.get("DIDFLBIT"));       // force R10 to re-initialise
        clearBits("FLAGWRD1", agc.get("RODFLBIT"));       // P66 re-initialises
        clearBits("FLAGWRD0", agc.get("P21FLBIT"));       // P21 computes new base vectors
        for (int g = NUMGRPS; g >= 0; g--) {
            int phase = s(agc.get("PHASE" + (g + 1)));
            if (phase > 0) {
                agc.set("MPAC", phase + 1);               // abs of phase
                agc.set("MPAC+6", Agc.add(agc.get("MPAC+6"), 1)); // group demands present
                // CA RACTCADR; TC SWCALL -> RESTARTS
            }
            if (g == 0) { break; }
        }
        if (s(agc.get("MPAC+6")) > 0) {
            endRstrt();                                    // phase active
            return;
        }
        if ((agc.get("MODREG") & BIT15) == 0) {
            goToPooh();                                    // mode not -0
        } else {
            endRstrt();
        }
    }

    /** PTBAD: restart failure - alarm 1107 and fresh start without touching the engine. */
    private void ptBad() {
        // TC ALARM; OCT 1107
        doFStrt1();
    }

    /**
     * LIGHTSET: check for mark reject and error reset keys (2M and 5M on the main DSKY);
     * if both are pressed force a fresh start (NONAVKEY). Returns true when it took over.
     */
    public boolean lightSet() {
        if ((agc.get("NAVKEYIN") & BIT5) == 0) {
            return false;                                  // no mark reject
        }
        if (((agc.get("MNKEYIN") & 037) + NEG_ELR) != 0) {
            return false;                                  // NONAVKEY: TC Q
        }
        startSub();
        doFStart();
        return true;
    }

    /** NONAVKEY +3: fresh start but do not touch the engine. */
    private void nonAvKey3() {
        startSub();
        doFStrt1();
    }

    // ------------------------------------------------------------------ STARTSUB

    /** STARTSUB: initialisation common to fresh start and restart. */
    public void startSub() {
        agc.set("DNTMGOTO", agc.get("DNPHASE1"));          // restart current downlist
        agc.set("RADMODES", ((agc.get("CHAN33") & BIT6)) + RMODINIT);
        startSb1();
    }

    /** STARTSB1: TIME3/4/5 and DAP clocks. */
    public void startSb1() {
        agc.set("TIME3", 037777);                          // POSMAX
        agc.set("TIME4", Agc.add(037777, Agc.fromSigned(-2)));
        agc.set("TIME5", Agc.add(agc.get("TIME4"), Agc.fromSigned(-1)));
        agc.set("EBANK", 6);
        clearBits("RCSFLAGS", BIT13);                      // cause DAPIDLER to call 1/ACCS
        agc.set("T6NEXT", 037777);                         // disable TIME6
        wand("CHAN13", ~0x4000);
        agc.set("NXT6ADR", 0);
        agc.set("NEXTP", 0);
        clearBits("DAPBOOLS", agc.get("ACCSOKAY"));
        agc.set("T5ADR", agc.get("IDLEADR"));              // 2CADR DAPIDLER
        startSb2();
    }

    /** STARTSB2: software-restart initialisation (engine, IMU flags, waitlist, executive, DSKY). */
    public void startSb2() {
        wand("DSALMOUT", OCT30001);                        // do not disturb engine on/off, ISS warning
        clearBits("FLAGWRD3", agc.get("READRBIT"));        // clear READRFLG for R29
        int bits = ((~agc.get("FLAGWRD3") & agc.get("NR29FBIT")) != 0 ? BIT10 : 0) + OCT32001;
        agc.set("RADMODES", agc.get("RADMODES") & ~bits);
        wand("CHAN12", OCT27470);                          // do not disturb IMU flags
        clearBits("FLAGWRD5", agc.get("NORRMBIT"));        // enable R25
        clearBits("FLAGWRD5", agc.get("R77FLBIT"));        // clear R77FLAG
        wand("CHAN13", OCT74160);                          // telemetry flags, trap flags, T6 enable
        wor("CHAN13", BIT12);                              // re-enable RUPT10
        wand("CHAN14", BIT6);                              // gyro enable
        agc.set("EBANK", 3);                               // STARTEB = EBANK3
        for (int i = 0; i < 8; i++) {
            agc.set("LST1+" + i, Agc.fromSigned(-0x2000));  // NEG1/2 waitlist delta-Ts
        }
        for (int i = 0; i < 18; i += 2) {
            agc.set("LST2+" + i, ~agc.get("ENDTASK"));      // CS ENDTASK
            agc.set("LST2+" + (i + 1), ~agc.get("ENDTASK+1"));
        }
        for (int i = 0; i <= 84; i += 12) {
            agc.set("PRIORITY+" + i, 0);                    // all executive register sets available
        }
        agc.set("DSRUPTSW", 0);
        agc.set("NEWJOB", 0);                               // no active jobs
        agc.set("VAC1USE", agc.get("VAC1USE"));             // CAF VAC1ADRC: VAC areas available
        for (int i = 1; i <= 5; i++) {
            agc.set("VAC" + i + "USE", agc.get("VAC1USE") + (i - 1) * LTHVACA);
        }
        for (int i = 10; i >= 0; i--) {
            agc.set("DSPTAB+" + i, ~BIT12 & 0x7FFF);        // R1, R2, R3 blank
        }
        zero("DELAYLOC", "DELAYLOC+1", "DELAYLOC+2", "R1SAVE", "INLINK", "DSPCNT", "CADRSTOR", "REQRET",
                "CLPASS", "DSPLOCK", "MONSAVE", "MONSAVE1", "VERBREG", "NOUNREG", "DSPLIST", "MARKSTAT",
                "EXTVBACT", "IMUCADR", "OPTCADR", "RADCADR", "ATTCADR", "LGYRO", "FLAGWRD4");
        agc.set("NOUT", agc.get("NOUTCON"));
        agc.set("SAMPLIM", Agc.fromSigned(-1));
        agc.set("IMODES33", (agc.get("IMODES33") & BIT6) + IM33INIT);
        agc.set("SELFRET", agc.get("SELFCHK"));
        agc.set("DSPCOUNT", ~agc.get("VD1"));
    }

    // ------------------------------------------------------------------ GOTOPOOH / V37

    /** GOTOPOOH: flash V37N99 on the DSKY for a major-mode change request. */
    public void goToPooh() {
        agc.set("PHASE4", 033);                            // 4.33 spot for GOP00FIX
        agc.set("-PHASE4", ~033 & 0x7FFF);
        goP00Fix();
    }

    /** GOP00FIX: allow X-axis override, ULLAGFLG off, release mark system, flash V37N99. */
    public void goP00Fix() {
        agc.set("XOVINFLG", 0);
        agc.set("ULLAGFLG", 0);
        // CLEARMRK +2; CAF V37N99 (VN 3799); BANKCALL GOFLASH (all three returns recycle)
    }

    /** V37: major mode change. The new MM arrives in A. */
    public void v37(int mm) {
        agc.set("MMNUMBER", mm);                           // save major mode
        agc.set("RESTREG", 030);
        if ((agc.get("IMODES30") & BIT6) != 0) {
            cantRod();                                     // IMU being initialised
            return;
        }
        if (mm == DEC_70) { setup70(false); return; }
        if (mm == DEC_70 + 1) { setup70(true); return; }
        if (mm == 0) {
            isServon();                                    // P00: check servicer
            return;
        }
        if ((agc.get("FLAGWRD2") & agc.get("NODOBIT")) != 0) {
            cantRod();
            return;
        }
        checkTab(mm);
    }

    /** CANTROD: alarm 1520 - major mode change not permitted. */
    private void cantRod() {
        // TC ALARM; OCT 1520
        v37Bad();
    }

    /** V37BAD: release the display, return to last normal display (PINBRNCH). */
    public void v37Bad() {
        // TC RELDSP; POSTJUMP PINBRNCH
    }

    /** CHECKTAB / AGAINMM: search PREMM1 for the requested MM (highest MM first). */
    private void checkTab(int mm) {
        for (int idx = NOV37MM; idx >= 0; idx--) {
            // The table is searched from the first entry with decreasing MM.
        }
        int found = -1;
        for (int i = 0; i <= NOV37MM; i++) {
            if ((PREMM1[i] & 0177) == mm) {
                found = i;
                break;
            }
            if ((PREMM1[i] & 0177) < mm) {
                break;                                     // passed the MM
            }
        }
        if (found < 0) {
            v37NoNo();
            return;
        }
        agc.set("MINDEX", found);
        isServon();
    }

    /** V37NONO: requested MM doesn't exist - FALTON then V37BAD. */
    private void v37NoNo() {
        // TC FALTON (operator error light)
        v37Bad();
    }

    /** ISSERVON: if the servicer is running, stop it (AVEGFLAG down) and wait for V37RET. */
    public void isServon() {
        if ((agc.get("FLAGWRD7") & agc.get("V37FLBIT")) == 0) {
            canV37();                                      // servicer not going
            return;
        }
        agc.set("AVEGFLAG", 0);
        agc.set("OUTROUTE", agc.get("V37RETAD"));          // servicer returns to CANV37 via V37RET
    }

    /** V37RET: after the servicer ends - choose the 2.x spot depending on P20/P22 or P25. */
    public void v37Ret() {
        int a;
        if ((agc.get("FLAGWRD0") & agc.get("RNDVZBIT")) != 0) {
            a = OCT_40072;                                  // 2.7SPT
        } else if ((agc.get("FLAGWRD0") & agc.get("P25FLBIT")) != 0) {
            a = OCT_37667 + BIT5 + OCT_40072;               // 2.11SPT
        } else {
            a = OCT_37667 + OCT_40072;                      // 2.0SPT
        }
        // TC PHSCHNGA with a
        agc.set("PHSCHNGA", a);
    }

    /** CANV37: set up R00 and the phase change. */
    public void canV37() {
        agc.set("SUPERBNK", 0);
        agc.set("TEMPFLSH", agc.get("R00AD"));
        // PHASCHNG OCT 14
        r00();
    }

    /** R00: wait for integration to finish, clean up flags, then POOH or NOUVEAU. */
    public void r00() {
        // CALL INTSTALL (interpretive wait for integration)
        agc.set("3AXISFLG", 0);
        agc.set("FLGWRD11", agc.get("LRBYBIT"));
        agc.set("R04FLAG", 0);
        agc.set("MUNFLAG", 0);
        agc.set("XOVINFLG", 0);
        if (s(agc.get("MMNUMBER")) != 0) {
            nouveau();
        } else {
            pooh();
        }
    }

    /** POOH: release display, set P00 integration (STATEINT1 via group 2), clear flags. */
    public void pooh() {
        // TC RELDSP
        agc.set("PHSPRDT2", 05);                            // PRIO5
        // TC CLRADMOD
        clearBits("FLAGWRD2", agc.get("NODOBIT"));          // turn off NODOFLAG
        agc.set("PHASE2", 5);                               // restart for STATEINT1
        agc.set("-PHASE2", ~5 & 0x7FFF);
        clearBits("FLAGWRD0", OCT_700);                     // turn off P20, P25, IMU in use flags
        seudoP00(DNLADP00);
    }

    /** SEUDOP00: set the downlist code, engine off, ALLCOAST, clear TRACK/UPDATE flags. */
    public void seudoP00(int downlist) {
        agc.set("DNLSTCOD", downlist);
        agc.set("AGSWORD", downlist);
        // IBNKCALL ENGINOF1; IBNKCALL ALLCOAST (does RESTORDB)
        agc.set("EBANKTEM", ~0120 & 0x7FFF);
        clearBits("FLAGWRD1", 0120);                        // track and update flags
        v37Klean();                                         // kill groups 1,3,5,6
        if (s(agc.get("MMNUMBER")) != 0) {
            rendV00();
        } else {
            goMod();
        }
    }

    /** GOMOD: redundant clean except group 4; store MODREG and go to GOPROG2. */
    public void goMod() {
        p00Klean();
        agc.set("MODREG", agc.get("MMNUMBER"));
        goProg2();                                          // GOGOPROG
    }

    /** RENDV00: handle P20/P22/P25 versus the new major mode. */
    public void rendV00() {
        int mm = s(agc.get("MMNUMBER")), cur = s(agc.get("MODREG"));
        if (cur == OCT_26 - 0 || mm == OCT_26) {            // P22 (decimal 22 = OCT 26)
            reset22();
            return;
        }
        if (mm == 020 || mm == 025 - 0 && false) {
            rendN00();
            return;
        }
        if (mm == 020 || mm == 021 + 4) {                   // P20 or P25 (OCT 24 / 31)
            rendN00();
            return;
        }
        if ((agc.get("FLAGWRD0") & OCT_500) != 0) {
            p00Fizz();                                      // leave group 2 to pick up P20/P25
            return;
        }
        reset22();
    }

    /** RESET22: clear rendezvous, P25 and IMU in use flags; kill group 2. */
    private void reset22() {
        clearBits("FLAGWRD0", OCT_700);
        // TC CLRADMOD
        kill2();
    }

    private void kill2() {
        agc.set("-PHASE2", Agc.fromSigned(-0));
        agc.set("PHASE2", Agc.fromSigned(-0));
        p00Fizz();
    }

    /** P00FIZZ: restart point for V37XEQ. */
    private void p00Fizz() {
        agc.set("TEMPFLSH", agc.get("V37QCAD"));
        goProg2();
    }

    /** RENDN00: P20/P25 requested - keep it if the same program is already running. */
    private void rendN00() {
        int cur = s(agc.get("MODREG"));
        if (cur == 020 || cur == 025) {                     // OCT24 / OCT31 (MM 20 / 25)
            kill2();
            return;
        }
        int running = agc.get("FLAGWRD0") & OCT_500;
        int test = ~(running + agc.get("MMNUMBER")) & 0x7FFF;
        if (Agc.add(test, P20REG) == 0x7FFF || Agc.add(Agc.add(test, P20REG), OCT_305) == 0x7FFF) {
            statQuo();
            return;
        }
        kill2();
    }

    /** STATQUO: set TRACKFLAG and UPDATEFLAG (leave P20/P25 as is). */
    private void statQuo() {
        agc.set("FLAGWRD1", agc.get("FLAGWRD1") | 0120);
        goMod();
    }

    /** NOUVEAU: pick up a new program - reset IMUINUSE unless P20/P25 flag set. */
    public void nouveau() {
        if ((agc.get("FLAGWRD0") & OCT_500) == 0) {
            agc.set("IMUSE", 0);
        }
        // INDEX MINDEX / CAF DNLADMM1; INHINT
        seudoP00(DNLADMM1[s(agc.get("MINDEX"))]);
    }

    /** V37XEQ: start the selected program's job (priority, E-bank, fixed address from tables). */
    public void v37Xeq() {
        int idx = s(agc.get("MINDEX"));
        int premm = PREMM1[idx];
        agc.set("PHSPRDT3", premm);                         // MMTEMP
        int prio = (premm >> 10) & 037;
        agc.set("PHSPRDT4", prio);                          // preset group 4 restart priority
        agc.set("NEWPRIO", prio);                           // store priority for SPVAC
        int ebank = (premm >> 7) & 07;                      // bits 8,9,10 of MMTEMP
        // BBCON = EBANK bits + FCADR high bits ; GENADR portion = (FCADR & LOW10) + BIT11
        agc.set("BASETEMP", ebank);
        // TC SPVAC
        v37XeqC();
    }

    /** V37XEQC: after SPVAC, put the new MM in MODREG, release the display, end the job. */
    public void v37XeqC() {
        agc.set("MODREG", agc.get("PHSPRDT3") & 0177);      // NEWMODEA
        // TC RELDSP; TC ENDOFJOB
    }

    /** SETUP70 / SETUP71: go to P70 (Q = 0) or P71 (Q = 3 for the 2CADR offset). */
    private void setup70(boolean p71) {
        agc.set("Q", p71 ? 3 : 0);
        // DCA P70CADR; AD Q; DTCB
    }

    // ------------------------------------------------------------------ P00 state-vector integration

    /**
     * STATEUP: P00 integration. Extrapolates the CM state vector (and the 6x6 W matrix if the
     * LM is on the lunar surface and W is valid), then the LM state vector (and 9x9 W matrix),
     * with PRECIFLG controlling the 4-time-step logic, then clears NODOFLAG and ends.
     */
    public void stateUp() {
        agc.set("VINTFLAG", 1);
        if (agc.get("SURFFLAG") != 0 && agc.get("RENDWFLG") != 0) {
            agc.set("DIM0FLAG", 1);                          // 6x6 W-matrix
        }
        doInt();
    }

    /** DOINT: integrate the CM state, then (unless on the surface) the LM state. */
    private void doInt() {
        agc.set("PRECIFLG", 0);                              // engage 4-time-step logic (MODREG = 0)
        // CALL INTEGRV
        if (agc.get("SURFFLAG") != 0) {
            noInt();
            return;
        }
        agc.set("TDEC1", agc.get("TETCSM"));
        // CALL INTSTALL ; CLEAR VINTFLAG ; CALL SETIFLGS
        agc.set("VINTFLAG", 0);
        if (agc.get("RENDWFLG") != 0) {
            agc.set("DIM0FLAG", 1);
            agc.set("D6OR9FLG", 1);                          // 9x9 W-matrix
        }
        agc.set("PRECIFLG", 1);                              // disengage 4-time-step logic
        // CALL INTEGRV
        noInt();
    }

    /** NO-INT: CLRGO NODOFLAG ENDINT. */
    private void noInt() {
        agc.set("NODOFLAG", 0);
        // GOTO ENDINT
    }

    /** THISVINT: called by MIDTOAV1 and 2 - CLEAR VINTFLAG; RVQ. */
    public void thisVint() {
        agc.set("VINTFLAG", 0);
    }
}
