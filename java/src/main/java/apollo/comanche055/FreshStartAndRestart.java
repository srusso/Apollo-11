package apollo.comanche055;

import apollo.agc.Agc;

/**
 * FRESH_START_AND_RESTART.agc - SLAP1 / DOFSTART (fresh start), GOPROG (hardware restart),
 * ENEMA (software restart), GOPROG3, STARTSUB/STARTSB2, MR.KLEAN, GOTOPOOH, V37 (major mode change),
 * V37XEQ, INITSUB, VAC5STOR, the V37 PREMM1/FCADRMM1/DNLADMM1 tables and STATEUP (orbital integration
 * initialisation).
 *
 * <p>Hardware channels are modelled as erasable symbols CHAN5, CHAN6, CHAN12... and DSALMOUT (channel
 * 11). Named flag bits are erasable symbols (non-zero = set). Control transfers (POSTJUMP, TCF) become
 * method calls; TC ENDOFJOB-style exits call {@link Executive#endofjob()}.
 */
public final class FreshStartAndRestart {
    // -------- constants --------
    public static final int OCT6200 = 06200;          // CMOONFLG, LMOONFLG, SUFFLAG
    public static final int OCT7777 = 07777;
    public static final int IFAILINH = 0435;
    public static final int LTHVACA = 44;             // decimal length of a VAC area
    public static final int INTMASK = 020100;
    public static final int OCT77603 = 077603;
    public static final int OCT74777 = 074777;
    public static final int NUMGRPS = 5;
    public static final int NEG_ELR = -022;           // -ERROR LIGHT RESET key code
    public static final int IM30INIF = 037411;        // inhibits IMU fail for 5 sec and PIP ISSW
    public static final int IM30INIR = 037000;
    public static final int IM33INIT = 016000;        // PRIO16: no PIP or TM fail signals
    public static final int BITS_9_6_4 = 0450;        // 9,6,4: prog alarm, gimbal lock, no att
    public static final int OPTINITF = 0130;
    public static final int OPTINITR = 0430;
    /** SWINIT: initial STATE switch words (all zero except word 5 = 00200). */
    public static final int[] SWINIT = {0, 0, 0, 0, 0, 00200, 0, 0, 0, 0, 0, 0};

    /** FLAGTABL: bits cleared in FLAGWRD0..11 by INITSUB. */
    public static final int[] FLAGTABL = {
        0, 00040 /* IDLEFAIL */, 06000 /* P21FLAG, STEERSW */, 0, 0, 04140 /* V59FLAG, ENGONFLG, 3AXISFLG */,
        010000 /* STRULLSW */, 016000, 0, 042000 /* SWTOVER, V94FLAG */, 0, 0};

    /** Downlist selectors for DNLADMM1. */
    public static final int COSTALIN = 0;
    public static final int ENTRYUPD = 1;
    public static final int RENDEZVU = 2;
    public static final int POWERED = 3;
    public static final int P22DNLST = 4;
    public static final int DNLADP00 = 0;

    /** PREMM1: priority (bits 15-11), E-bank (bits 10-8), major mode (bits 7-1), highest MM first. */
    public static final int[] PREMM1 = {
        027117, // MM 79
        027116, // MM 78
        027115, // MM 77
        027714, // MM 76
        027113, // MM 75
        027112, // MM 74
        027111, // MM 73
        027110, // MM 72
        027476, // MM 62
        027475, // MM 61
        027266, // MM 54
        027265, // MM 53
        027264, // MM 52
        027263, // MM 51
        027657, // MM 47
        027451, // MM 41
        027450, // MM 40
        027047, // MM 39
        027046, // MM 38
        027645, // MM 37
        027043, // MM 35
        027042, // MM 34
        027041, // MM 33
        027040, // MM 32
        027637, // MM 31
        027636, // MM 30
        027227, // MM 23
        027226, // MM 22
        027025, // MM 21
        027424, // MM 20
        027021, // MM 17
        027006, // MM 06
        041201  // MM 01
    };

    /** NOV37MM: number of PREMM1 entries minus one. */
    public static final int NOV37MM = PREMM1.length - 1;

    /** FCADRMM1: starting job label for each PREMM1 entry (same order). */
    public static final String[] FCADRMM1 = {
        "P79", "P78", "P77", "P76", "P75", "P74", "P73", "P72", "P62", "P61", "P54", "P53", "PROG52", "P51",
        "P47CSM", "P41CSM", "P40CSM", "P39", "P38", "P37", "P35", "P34", "P33", "P32", "P31", "P30", "P23",
        "PROG22", "PROG21", "PROG20", "P17", "P06", "GTSCPSS1"};

    /** DNLADMM1: downlist code for each PREMM1 entry. */
    public static final int[] DNLADMM1 = {
        RENDEZVU, RENDEZVU, RENDEZVU, RENDEZVU, RENDEZVU, RENDEZVU, RENDEZVU, RENDEZVU, // P79..P72
        ENTRYUPD, POWERED,                                                              // P62, P61
        COSTALIN, COSTALIN, COSTALIN, COSTALIN,                                         // P54..P51
        POWERED, POWERED, POWERED,                                                      // P47, P41, P40
        RENDEZVU, RENDEZVU, RENDEZVU, RENDEZVU, RENDEZVU, RENDEZVU, RENDEZVU, RENDEZVU, // P39..P31
        RENDEZVU,                                                                       // P30
        RENDEZVU, P22DNLST, RENDEZVU, RENDEZVU, RENDEZVU,                               // P23..P17
        COSTALIN, COSTALIN};                                                            // P06, P01

    /** RMM: 800 km above the lunar surface, 2538.09E3 B-27 (m). */
    public static final double RMM = 2538.09E3;
    /** RME: 800 km above equatorial radius, 7178165 B-29 (m). */
    public static final double RME = 7178165.0;

    private static final int BIT7 = 0100;
    private static final int BIT13 = 010000;
    private static final int BIT14 = 020000;
    private static final int BIT15 = 040000;
    private static final int PRIO30 = 030000;
    private static final int PRIO12 = 012000;

    private final Agc agc;
    private final Executive executive;
    private final ServiceRoutines service;

    public FreshStartAndRestart(Agc agc) {
        this.agc = agc;
        this.executive = new Executive(agc);
        this.service = new ServiceRoutines(agc);
    }

    private boolean flag(String n) {
        return agc.get(n) != 0;
    }

    private void setFlag(String n, boolean on) {
        agc.set(n, on ? 1 : 0);
    }

    // ----------------------------------------------------------------------------------
    // Fresh start
    // ----------------------------------------------------------------------------------

    /** SLAP1: man-initiated fresh start (comes from PINBALL, VERB 36 ENTER). */
    public void slap1() {
        startsub();
        // STARTSW: TCF SKIPSIM (patch: TCF STARTSIM for simulation, which FINDVACs SIM2CADR at BIT14)
        // SKIPSIM:
        agc.set("DSPTAB+11D", ((agc.get("DSPTAB+11D") & BITS_4_6) + BIT15) & Agc.WORD_MASK); // requested fresh start
        agc.set("ERCOUNT", 0);
        agc.set("FAILREG", 0);
        agc.set("FAILREG+1", 0);
        agc.set("FAILREG+2", 0);
        agc.set("REDOCTR", 0);
        agc.set("DSRUPTSW", ~PRIO12 & Agc.WORD_MASK);
        dofstart();
    }

    private static final int BITS_4_6 = 050; // BITS4&6

    /** DOFSTART: machine-initiated fresh start. */
    public void dofstart() {
        agc.set("ERESTORE", 0);                 // MUST NOT BE REMOVED FROM DOFSTART
        agc.set("SMODE", 0);
        agc.set("UPSVFLAG", 0);                 // update state vector request flagword
        agc.set("CHAN5", 0);                    // turn off RCS jets
        agc.set("CHAN6", 0);
        agc.set("DSALMOUT", 0);                 // zero channel 11
        agc.set("CHAN12", 0);
        agc.set("CHAN13", 0);
        agc.set("CHAN14", 0);
        for (String s : new String[] {
            "WTOPTION", "DNLSTCOD", "NVSAVE", "EBANKTEM", "RATEINDX", "TRKMKCNT", "VHFCNT", "EXTVBACT"}) {
            agc.set(s, 0);
        }
        if ((~agc.get("DSPTAB+11D") & BITS_4_6 & Agc.WORD_MASK) != 0) {
            // the IMU was in coarse align in gimbal lock, so put it back into coarse align
            agc.set("CHAN12", agc.get("CHAN12") | BITS_4_6);
        }
        mrKlean();
        agc.set("MODREG", Agc.WORD_MASK);       // CS ZERO: -0
        agc.set("RESTREG", PRIO30);
        agc.set("IMODES30", IM30INIF);          // fresh start IMU initialization
        agc.set("OPTIND", Agc.fromSigned(-1));  // kill coarse optics
        agc.set("OPTMODES", OPTINITF);
        agc.set("IMODES33", IM33INIT);
        agc.set("T5LOC", 0 /* T5IDLER: T5IDLOC */);   // let T5 idle
        agc.set("STATE", SWINIT[0]);
        agc.set("FLAGWRD1", (agc.get("FLAGWRD1") & NOP01BIT) + SWINIT[1]);  // leave NODOP01 untouched
        agc.set("STATE+2", SWINIT[2]);
        agc.set("FLAGWRD3", (agc.get("FLAGWRD3") & BIT13) + SWINIT[3]);     // REFSMMAT flag
        agc.set("STATE+4", SWINIT[4]);
        agc.set("STATE+5", SWINIT[5]);
        agc.set("STATE+6", SWINIT[6]);
        agc.set("STATE+7", SWINIT[7]);
        agc.set("FLAGWRD8", (agc.get("FLAGWRD8") & OCT6200) + SWINIT[8]);   // CMOON, LMOON, SUFFLAG
        agc.set("STATE+9", SWINIT[9]);
        agc.set("STATE+10", SWINIT[10]);
        agc.set("STATE+11", SWINIT[11]);
        endrstrt();
    }

    private static final int NOP01BIT = 0; // mask value for NODOP01 flag bit (assembler-defined)

    /** ENDRSTRT: transfer control to the idle loop in DUMMYJOB +2 (does a RELINT). */
    public void endrstrt() {
        executive.dummyjob();
    }

    /** MR.KLEAN: clear the phase table (-PHASEn pairs set to -0). */
    public void mrKlean() {
        for (int g = 1; g <= 6; g++) {
            clearGroup(g);
        }
    }

    /** P00KLEAN: clears groups 4, 1 (and below). */
    public void p00klean() {
        clearGroup(4);
        clearGroup(1);
        v37klean();
    }

    /** V37KLEAN: clears groups 3, 5, 6. */
    public void v37klean() {
        clearGroup(3);
        clearGroup(5);
        clearGroup(6);
    }

    private void clearGroup(int g) {
        agc.set("-PHASE" + g, Agc.WORD_MASK);
        agc.set("-PHASE" + g + "+1", Agc.WORD_MASK);
    }

    // ----------------------------------------------------------------------------------
    // Hardware restart
    // ----------------------------------------------------------------------------------

    /** GOPROG: hardware restart entered from location 4000 (GOJAM). */
    public void goprog() {
        agc.set("REDOCTR", Agc.add(agc.get("REDOCTR"), 1));      // advance restart counter
        agc.set("RSBBQ", agc.get("Q"));                           // LXCH Q ; DXCH RSBBQ
        agc.set("RSBBQ+1", agc.get("SUPERBNK"));
        vac5stor();                                               // store erasables for debugging
        // test OSC FAIL bit to see if we have had a power transient
        boolean oscFail = (agc.get("CHAN33") & BIT15) != 0;
        if (!oscFail) {
            // BUTTONS path
        } else if ((agc.get("CHAN33") & BIT14) == 0) {
            // AGC warning on and OSC fail: assume restart loop -> fresh start
            startsub();
            dofstart();
            return;
        }
        if (lightset()) {                                         // BUTTONS
            return;
        }
        // ERASCHK check: ERESTORE must be +0 or +N less than 2000 octal
        int erestore = agc.get("ERESTORE");
        boolean doubt = (erestore & 0176000) != 0 && erestore != Agc.WORD_MASK;
        if (doubt) {
            startsub();
            dofstart();
            return;
        }
        if (erestore != 0 && erestore != Agc.WORD_MASK) {
            if (agc.get("ERESTORE") != agc.get("SKEEP7")) {      // not SKEEP7: doubt E memory
                startsub();
                dofstart();
                return;
            }
            agc.set("EBANK", agc.get("SKEEP4"));                  // E-bank of memory under test
            agc.set("E" + agc.get("SKEEP7"), agc.get("SKEEP5"));  // restore E memory (DXCH 0000)
            agc.set("E" + (agc.get("SKEEP7") + 1), agc.get("SKEEP6"));
            agc.set("ERESTORE", 0);
            startsub();                                           // initialization after erase restore
        }
        // ELRSKIP: restart autopilots by FLAGWRD6 bits 15,14
        int ap = (agc.get("FLAGWRD6") >> 13) & 3;                 // 00 T5IDLOC 01 REDORCS 10 REDOTVC 11 REDOSAT
        agc.set("T5LOC", ap);
        agc.set("RASFLAG", agc.get("RASFLAG") & ~INTFLBIT);
        agc.set("OPTMODES", (agc.get("OPTMODES") & OPTINITR) + BIT7);
        agc.set("IMODES33", (agc.get("IMODES33") & 040) + IM33INIT);
        int dsp = agc.get("DSPTAB+11D");                          // leave prog alarm, gimbal lock, no att
        agc.set("DSPTAB+11D", ((BITS_9_6_4 & dsp) + BIT15) & Agc.WORD_MASK);
        if ((dsp & 010) != 0 || true) {
            // IF NO ATT LAMP WAS ON, RETURN ISS TO COARSE ALIGN (SETCOARS) and wait 6 cs for CA+ECE
            if ((dsp & 010) != 0) {
                new ImuModeSwitchingRoutines(agc).setcoars();
                new Waitlist(agc).waitlist(6, () -> new ImuModeSwitchingRoutines(agc).caEce());
            }
        }
        agc.set("IMODES30", (IFAILINH & agc.get("IMODES30")) + IM30INIR);   // keep failure inhibits
        if ((agc.get("FLAGWRD5") & ENGONBIT) != 0) {
            agc.set("DSALMOUT", agc.get("DSALMOUT") | BIT13);    // turn engine on
        }
        goprog3();
    }

    private static final int INTFLBIT = 0100;
    private static final int ENGONBIT = 0100;

    /** ENEMA: software restart initiated by a major mode change. DO NOT CALL without consulting POOH people. */
    public void enema() {
        if (lightset()) {                                         // exit to DOFSTART on error reset + mark reject
            return;
        }
        startsb2();
        agc.set("RASFLAG", agc.get("RASFLAG") & ~INTMASK);        // reset integration bits
        if ((agc.get("FLAGWRD6") & 060000) != 0) {                // is TVC on
            new Waitlist(agc).waitlist(50, () -> new TvcExecutive(agc).tvcexec()); // call TVCEXEC killed in STARTSB2
        }
        goprog3();
    }

    /** GOPROG2 = ENEMA. */
    public void goprog2() {
        enema();
    }

    /**
     * GOPROG3: common to GOPROG and ENEMA. Verifies phase table, displays major mode, reschedules
     * active restart groups through RESTARTS, else alarm 1110 / GOTOPOOH.
     */
    public void goprog3() {
        for (int g = NUMGRPS; g >= 0; g--) {                      // PCLOOP: process all restart groups
            int phase = agc.get("PHASE" + (g + 1));
            int comp = agc.get("-PHASE" + (g + 1));
            if (((phase ^ comp) & Agc.WORD_MASK) != Agc.WORD_MASK) {
                ptbad();                                          // restart failure
                return;
            }
        }
        agc.set("MPAC+6", 0);
        new PhaseTableMaintenance(agc).mmdsplay();          // display major mode
        if ((agc.get("FLAGWRD6") & 060000) != 0) {                // RCS DAP running: do STOPRATE
            agc.set("EBANK", 06);
            new KalcmanuSteering(agc).stoprate();
            agc.set("EBANK", 03);
        }
        for (int g = NUMGRPS; g >= 0; g--) {                      // NXTRST
            int phase = Agc.toSigned(agc.get("PHASE" + (g + 1)));
            if (phase > 0) {                                      // PACTIVE
                agc.set("MPAC", phase + 1);                       // abs of phase
                agc.set("MPAC+6", agc.get("MPAC+6") + 1);         // group demands present
                new RestartsRoutine(agc).restarts(grp -> () -> { /* TODO: dispatch PHSNAMEn 2CADR (RACTCADR) for group grp */ });             // CA RACTCADR ; TC SWCALL
            }
        }
        if (agc.get("MPAC+6") != 0) {
            endrstrt();                                           // phase active
        } else if ((agc.get("MODREG") & BIT15) == 0) {            // mode is not -0
            gotopooh();
        } else {
            endrstrt();
        }
    }

    /** PTBAD: alarm 1107 (phase table failure) and fresh start. */
    private void ptbad() {
        new AlarmAndAbort(agc).alarm(01107);
        dofstart();
    }

    /**
     * LIGHTSET: DOFSTART if MARK REJECT and either ERROR LIGHT RESET button are depressed.
     * @return true if a fresh start was performed (caller must return)
     */
    private boolean lightset() {
        if ((agc.get("NAVKEYIN") & BIT7) == 0) {
            return false;                                         // NONAVKEY: no mark reject
        }
        boolean navReset = Agc.add(agc.get("NAVKEYIN") & 037, Agc.fromSigned(NEG_ELR)) == 0
            || (agc.get("NAVKEYIN") & 037) == 022;
        boolean mainReset = Agc.add(agc.get("MNKEYIN"), Agc.fromSigned(NEG_ELR)) == 0
            || agc.get("MNKEYIN") == 022;
        if (navReset || mainReset) {
            startsub();
            dofstart();
            return true;
        }
        return false;
    }

    /** STARTSUB: clear outbit channels 5 and 6; initialise TIME5, TIME4, TIME3; then STARTSB2. */
    public void startsub() {
        agc.set("DNTMGOTO", 0 /* DNPHASE1 */);   // restart current downlist
        agc.set("TIME3", Agc.WORD_MASK >> 1);      // 37777
        agc.set("TIME4", 037775);
        agc.set("TIME5", 037774);
        startsb2();
    }

    /** STARTSB2: common to STARTSUB and ENEMA. Re-initialises outbits, WAITLIST, EXECUTIVE, VACs, DSKY. */
    public void startsb2() {
        agc.set("DSALMOUT", agc.get("DSALMOUT") & OCT77603);      // uplink acty, temp caution, KR, flash, op error off
        agc.set("CHAN13", agc.get("CHAN13") & OCT74777);          // test alarms, standby enable off
        agc.set("FLAGWRD2", (agc.get("FLAGWRD2") & ~025000 & Agc.WORD_MASK) | SKIPVBIT);
        agc.set("EBANK", 03);                                     // E3 (LST1)
        for (int i = 0; i < 8; i++) {                             // initialise WAITLIST delta-Ts
            agc.set(DpWords.off("LST1", i), 037776 /* NEG1/2 */ & Agc.WORD_MASK);
        }
        for (int i = 0; i < 18; i += 2) {                         // replace all tasks with ENDTASK
            agc.set(DpWords.off("LST2", i), ~ENDTASK_HI & Agc.WORD_MASK);
            agc.set(DpWords.off("LST2", i + 1), ~ENDTASK_LO & Agc.WORD_MASK);
        }
        for (int i = 0; i < 7; i++) {                             // all executive register sets available
            agc.set(DpWords.off("PRIORITY", 12 * i), Agc.WORD_MASK);
        }
        agc.set("DSRUPTSW", Agc.WORD_MASK);
        agc.set("NEWJOB", Agc.WORD_MASK);                         // no active jobs
        int vac = agc.get("VAC1USE");                             // all VAC areas available
        for (int i = 1; i <= 5; i++) {
            agc.set("VAC" + i + "USE", vac + (i - 1) * LTHVACA);
        }
        for (int i = 10; i >= 0; i--) {                           // DSPOFF: blank DSKY registers
            agc.set("DSPTAB" + (i == 0 ? "" : "+" + i), ~04000 & Agc.WORD_MASK);
        }
        for (int i = 0; i < 4; i++) {
            agc.set("DELAYLOC" + i, 0);
        }
        for (String s : new String[] {
            "R1SAVE", "INLINK", "DSPCNT", "CADRSTOR", "REQRET", "CLPASS", "DSPLOCK", "MONSAVE", "MONSAVE1",
            "VERBREG", "NOUNREG", "DSPLIST", "MARKSTAT", "IMUCADR", "OPTCADR", "RADCADR", "ATTCADR", "LGYRO",
            "FLAGWRD4"}) {
            agc.set(s, 0);
        }
        agc.set("NOUT", NOUTCON);
        agc.set("EXTVBACT", agc.get("EXTVBACT") & BIT14);
        agc.set("SELFRET", 0 /* SELFCHK */);                      // self check go-to register
        agc.set("DSPCOUNT", ~VD1 & Agc.WORD_MASK);
    }

    private static final int SKIPVBIT = 0400;
    private static final int ENDTASK_HI = 0;
    private static final int ENDTASK_LO = 0;
    private static final int NOUTCON = 0;
    private static final int VD1 = 0;

    /** T5IDLOC: T5RUPT comes here every 163.84 s when nobody is using it. */
    public void t5idloc() {
        // CA L ; TCF NOQRSM +1
    }

    // ----------------------------------------------------------------------------------
    // GOTOPOOH
    // ----------------------------------------------------------------------------------

    /** GOTOPOOH: request major mode 00 by flashing V37N99. */
    public void gotopooh() {
        new PhaseTableMaintenance(agc).phaschng(014, 0);          // restart GOTOPOOH
        gop00fix();
    }

    /** GOP00FIX: display V37N99 flash and wait for response. */
    public void gop00fix() {
        initsub();
        new DisplayInterfaceRoutines(agc).clearmrk();
        int reply;
        do {
            reply = new DisplayInterfaceRoutines(agc).goflash(3799);
        } while (reply == 0 /* terminate/proceed/enter: all retry */ || true && reply < 0);
    }

    // ----------------------------------------------------------------------------------
    // V37 - major mode change
    // ----------------------------------------------------------------------------------

    /** V37: entered with new major mode number in A (via MMCHANG POSTJUMP CADR V37). */
    public void v37(int mm) {
        agc.set("MMNUMBER", mm);                                  // save major mode
        agc.set("RESTREG", PRIO30);                               // restart at PINBALL priority
        if ((agc.get("IMODES30") & 040) != 0) {                   // IMU being initialised
            cantr00();
            return;
        }
        boolean engineOn = (agc.get("DSALMOUT") & BIT13) != 0;
        boolean tvcOn = (agc.get("FLAGWRD6") & 060000) != 0;
        if (engineOn || tvcOn) {
            r00top00();                                           // set up for P00
        }
        isitp00();
    }

    /** R00TOP00: engine/TVC on: shut engine down, return to RCS DAP, mass properties, clear TVC. */
    private void r00top00() {
        agc.set("EBANK", 06);
        agc.set("MASSTMP", agc.get("CSMMASS"));
        ImuModeSwitchingRoutines dummy = null;
        new P40P47(agc).spsoff();
        new TvcMassprop(agc).massprop();
        new P40P47(agc).rcsdaponPlus1(037312 /* 3.1 SEC */);
        new P40P47(agc).tvczap();                          // disable TVC
        agc.set("MMNUMBER", 0);
        new ServiceRoutines(agc).delayjob(5);
        agc.set("CHAN5", 0);
        agc.set("CHAN6", 0);
    }

    /** ISITP00 */
    private void isitp00() {
        if (agc.get("MMNUMBER") == 0) {
            isserveron();                                         // P00: check servicer status
            return;
        }
        if ((agc.get("FLAGWRD2") & NODOBIT) != 0) {               // NODOFLAG set: not permitted
            cantr00();
            return;
        }
        checktab();
    }

    private static final int NODOBIT = 01;

    /** CANTR00: alarm 1520 (major mode change not permitted) and V37BAD. */
    private void cantr00() {
        new AlarmAndAbort(agc).alarm(01520);
        v37bad();
    }

    /** V37BAD: release the display and bring back the last normal display. */
    private void v37bad() {
        new PinballGameButtonsAndLights(agc).reldsp();
        new DisplayInterfaceRoutines(agc).pinbrnch();
    }

    /** CHECKTAB: find the requested major mode in PREMM1; index saved in MINDEX. */
    private void checktab() {
        int mm = agc.get("MMNUMBER");
        for (int i = NOV37MM; i >= 0; i--) {                      // AGAINMM
            int entryMm = PREMM1[i] & 0177;
            int diff = mm - entryMm;
            if (diff == 0) {
                agc.set("MINDEX", i);
                isserveron();
                return;
            }
            if (diff < 0) {
                break;                                            // passed MM
            }
        }
        // V37NONO: requested MM doesn't exist
        new PinballGameButtonsAndLights(agc).falton();
        v37bad();
    }

    /** ISSERVON: if V37 flag set (servicer going), clear AVEGFLAG and wait for servicer to hit CANV37. */
    private void isserveron() {
        if (!flag("V37FLAG")) {
            canv37();
            return;
        }
        setFlag("AVEGFLAG", false);                               // servicer will return to CANV37
        executive.endofjob();
    }

    /** CANV37: servicer finished; continue the major mode change. */
    public void canv37() {
        agc.set("TEMPFLSH", 0 /* R00AD = DUMMYAD */);
        new PhaseTableMaintenance(agc).phaschng(014, 0);
        // ROC: wait for integration to finish
        new IntegrationInitialization(agc).intstall();
        dummyad();
    }

    private static final int OCT1400 = 01400;
    private static final int OCT44571 = 044571;
    private static final int OCT600 = 0600;
    private static final int OCT01120 = 01120;
    private static final int OCT120 = 0120;
    private static final int BIT7_8 = 0300;

    /** DUMMYAD: restart point after integration; clear channel bits, INITSUB, kill flags, go POOH/NOUVEAU. */
    public void dummyad() {
        agc.set("DSALMOUT", agc.get("DSALMOUT") & ~OCT1400 & Agc.WORD_MASK);   // clear caution reset, test connector
        agc.set("CHAN12", agc.get("CHAN12") & OCT44571);          // clear optics/star tracker/TVC enable etc.
        agc.set("CHAN13", agc.get("CHAN13") & ~OCT600 & Agc.WORD_MASK);       // clear unused bits
        initsub();
        new DisplayInterfaceRoutines(agc).clearmrk();
        setFlag("STIKFLAG", false);
        new UpdateProgram(agc).upactoff();                               // turn off uplink activity light
        setFlag("VHFRFLAG", false);
        setFlag("R21MARK", false);
        if (agc.get("MMNUMBER") != 0) {
            nouveau();
        } else {
            pooh();
        }
    }

    /** POOH: request P00. */
    public void pooh() {
        new PinballGameButtonsAndLights(agc).reldsp();
        agc.set("PHSPRDT2", 05000);                               // variable restart register for P00 (PRIO5)
        agc.set("FLAGWRD2", agc.get("FLAGWRD2") & ~NODOBIT);      // turn off NODOFLAG
        agc.set("PHASE2", Agc.fromSigned(5));                     // 2.5 restart for STATEINT1
        agc.set("-PHASE2", Agc.fromSigned(~5 & Agc.WORD_MASK));
        agc.set("FLAGWRD0", agc.get("FLAGWRD0") & ~BIT7_8);       // reset IMUSE and kill P20 (RENDFLG)
        seudop00(DNLADP00);
    }

    /** SEUDOP00: set the downlist, turn off track/targ1/update flags and kill restart groups. */
    private void seudop00(int downlist) {
        agc.set("DNLSTCOD", downlist);
        agc.set("EBANKTEM", OCT01120);
        agc.set("FLAGWRD1", agc.get("FLAGWRD1") & ~OCT01120 & Agc.WORD_MASK);
        v37klean();                                               // GROUPKIL: groups 3, 5, 6
        if (agc.get("MMNUMBER") != 0) {
            rendv00();
            return;
        }
        p00klean();                                               // redundant except for group 4
        gomod();
    }

    private void gomod() {
        agc.set("MODREG", agc.get("MMNUMBER"));
        gogoprog();
    }

    private void gogoprog() {
        goprog2();                                                // POSTJUMP GOPROG2
    }

    /** RENDV00: new program is P20? */
    private void rendv00() {
        if (agc.get("MMNUMBER") == 020) {                         // RENDN00
            if (agc.get("MMNUMBER") == agc.get("MODREG")) {
                kill20();
                return;
            }
            if ((agc.get("FLAGWRD0") & RNDVZBIT) != 0) {
                statquo();                                        // P20 already running in background
                return;
            }
        }
        // P00FIZZ
        if ((agc.get("FLAGWRD0") & RNDVZBIT) != 0) {
            rev37();
        } else {
            kill20();
        }
    }

    private static final int RNDVZBIT = 0200;

    /** KILL20: kill groups 1 and 2. */
    private void kill20() {
        clearGroup(1);
        clearGroup(2);
        rev37();
    }

    /** REV37: set restart point V37XEQ +3 and enter GOPROG2. */
    private void rev37() {
        agc.set("TEMPFLSH", 03 /* V37QCAD */);
        gogoprog();
    }

    /** STATQUO: set TRACKFLAG and UPDATE flag; kill group 4. */
    private void statquo() {
        agc.set("FLAGWRD1", agc.get("FLAGWRD1") | OCT120);
        clearGroup(4);
        gomod();
    }

    /** NOUVEAU: pick up new program; reset IMUSE unless rendezvous is running. */
    private void nouveau() {
        if ((agc.get("FLAGWRD0") & RNDVZBIT) == 0) {
            setFlag("IMUSE", false);                              // bit 8 flag 0
        }
        seudop00(DNLADMM1[agc.get("MINDEX")]);                    // new downlist address
    }

    /** V37XEQ: schedule the new major mode job with SPVAC at the priority/E-bank from PREMM1. */
    public void v37xeq() {
        int idx = agc.get("MINDEX");
        int premm = PREMM1[idx];
        agc.set("MMTEMP", premm);                                 // priority bits 15-11
        int prio = (premm >> 1) & 037000;                         // CYR: shift right to bits 14-10
        agc.set("PHSPRDT4", prio);                                // preset group 4 restart priority
        agc.set("NEWPRIO", prio);
        int ebank = (premm >> 7) & 7;                             // EBANK bits 8-10
        agc.set("BASETEMP", idx);                                 // FCADR of the job (FCADRMM1[idx])
        executive.spvac(prio, ebank, idx);
        v37xeqc();
    }

    /** V37XEQC: placed in MODREG after SPVAC returns, release the display and end the job. */
    private void v37xeqc() {
        new PhaseTableMaintenance(agc).newmodea(agc.get("MMTEMP") & 0177);
        new PinballGameButtonsAndLights(agc).reldsp();
        executive.endofjob();
    }

    /** INITSUB: set deadband, clear flagword bits per FLAGTABL, set IMPULSW and OPTIND. */
    public void initsub() {
        agc.set("EBANK", 06);
        new KalcmanuSteering(agc).stoprate();
        if ((agc.get("FLAGWRD9") & MAXDBBIT) != 0) {
            new P40P47(agc).setmaxdb();       // max deadband selected
        } else {
            new P40P47(agc).setmindb();       // min deadband selected
        }
        for (int i = 11; i >= 0; i--) {                           // RAKE: clear flagword bits
            String w = "FLAGWRD" + i;
            agc.set(w, agc.get(w) & ~FLAGTABL[i] & Agc.WORD_MASK);
        }
        service.upflag("FLAGWRD2", 1);
        setFlag("IMPULSW", true);
        agc.set("OPTIND", Agc.fromSigned(-1));
    }

    private static final int MAXDBBIT = 02;

    // ----------------------------------------------------------------------------------
    // VAC5STOR - store erasables for debugging on a restart
    // ----------------------------------------------------------------------------------

    /** VAC5STOR: copy LOC, BBANK, PRIORITY of each core set, PHSNAME, PHASE tables into VAC5. */
    public void vac5stor() {
        int itemp2 = 0;
        for (int itemp1 = 0; ; itemp1 += 12, itemp2 += 3) {
            agc.set(DpWords.off("VAC5", itemp2), agc.get(DpWords.off("LOC", itemp1)));
            agc.set(DpWords.off("VAC5", itemp2 + 1), agc.get(DpWords.off("LOC", itemp1 + 1)));
            agc.set(DpWords.off("VAC5", itemp2 + 2), agc.get(DpWords.off("PRIORITY", itemp1)));
            if (itemp2 + 3 == 18) {                              // EIGHTEEN = OCT 22
                break;
            }
        }
        agc.set("EBANK", 03);                                     // PHSNAME registers are in EBANK3
        service.gentran(11, "PHSNAME1", "VAC5+21D");
        itemp2 = 0;
        for (int itemp1 = 0; itemp2 <= 5; itemp1 += 2, itemp2++) {
            agc.set(DpWords.off("VAC5", 33 + itemp2), agc.get(DpWords.off("PHASE1", itemp1)));
        }
        agc.set("VAC5+39D", agc.get("MPAC+3"));
        agc.set("VAC5+40D", agc.get("NEWLOC"));
        agc.set("VAC5+41D", agc.get("NEWLOC+1"));
        agc.set("VAC5+22D", agc.get("NEWJOB"));
        agc.set("VAC5+26D", agc.get("NEWPRIO"));
    }

    // ----------------------------------------------------------------------------------
    // Orbital integration (INTINIT bank)
    // ----------------------------------------------------------------------------------

    /**
     * STATEUP: extrapolate the CM state vector (and 6x6 W-matrix if valid for orbital navigation), then
     * the LM state vector (and W-matrix for rendezvous navigation). Interpretive routine; flag operands
     * are erasable flag symbols.
     */
    public void stateup() {
        IntegrationInitialization oi = new IntegrationInitialization(agc);
        setFlag("VINTFLAG", true);
        if (!flag("ORBWFLAG")) {
            setFlag("DIM0FLAG", true);
        }
        setFlag("PRECIFLG", false);
        oi.integrv();
        if (!flag("SURFFLAG")) {
            agc.set("TDEC1", agc.get("TETCSM"));
            agc.set("TDEC1+1", agc.get("TETCSM+1"));
            new IntegrationInitialization(agc).intstall();
            setFlag("VINTFLAG", false);
            oi.setiflgs();                                        // LM state vector + W matrix if valid
            if (flag("RENDWFLG")) {
                setFlag("DIM0FLAG", true);
            }
            setFlag("PRECIFLG", true);
            oi.integrv();
        }
        // STATEND
        setFlag("NODOFLAG", false);
        new IntegrationInitialization(agc).endint();
    }

    /** THISVINT: called by MIDTOAV1 and 2: set VINTFLAG and return. */
    public void thisvint() {
        setFlag("VINTFLAG", true);
    }
}
