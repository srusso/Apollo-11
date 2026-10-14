package apollo.luminary099;

import apollo.agc.Agc;

/**
 * Translation of P70-P71.agc (Luminary 1A build 099).
 *
 * P70 (DPS abort) and P71 (APS abort) programs, the R10/R11 landing-radar / LR-lights
 * task (R10,R11), the ascent-guidance initialisation (P70INIT, BOTHPOLY, UPTHROT) and
 * the abort-stage entry points (GOABORT, LEGAL?).
 *
 * Interpretive sections are translated as straight-line Java on double-precision
 * quantities held in erasable pairs (see {@link #dp}/{@link #setDp}), with the
 * original scaling noted in comments.
 */
public class P70P71 {
    private static final int BIT1 = 01;
    private static final int BIT4 = 010;
    private static final int BIT13 = 010000;
    private static final int OCTAL27 = 027;
    private static final int OCT37774 = 037774;
    private static final int DAPBITS = 00640;          // bits 6,8,9: ullage, drift, XOVINHIB
    private static final int MODE70 = 70;
    private static final int MODE71 = 71;
    private static final int DEC70 = 70;
    private static final int DEC71 = 71;

    // 2DEC constants
    public static final double TEN_SECS_CS = 1000;            // 10SECS
    public static final double HINJECT_M = 18288;             // 60,000 ft in metres, B-24
    public static final double TGO_A = 37000;                 // (TGO)A, B-17
    public static final double K_AT = .02;                    // scaling constant

    private final Agc agc;

    public P70P71(Agc agc) {
        this.agc = agc;
    }

    // ---- helpers ---------------------------------------------------------------
    /** Reads a DP pair (name, name+1) as a signed fraction of full scale. */
    private double dp(String sym) {
        long v = ((long) Agc.toSigned(agc.get(sym)) << 14) + Agc.toSigned(agc.get(sym + "+1"));
        return v / (double) (1L << 28);
    }

    /** Stores a signed fraction into a DP pair. */
    private void setDp(String sym, double value) {
        long v = Math.round(value * (1L << 28));
        agc.set(sym, Agc.fromSigned((int) (v >> 14)));
        agc.set(sym + "+1", Agc.fromSigned((int) (v & 037777)));
    }

    // ---- R10,R11 -----------------------------------------------------------------

    /** R10,R11: landing-radar-height task; runs every PIPA cycle while servicer is off. */
    public void r10R11() {
        if ((agc.get("FLAGWRD7") & agc.get("AVEGFBIT")) == 0) {
            // Servicer is running: let AVGEND take care of group 2.
            new ServiceRoutines(agc).taskover();
            return;
        }
        int pipctr = agc.get("PHSPRDT2");
        if (pipctr == 0) {
            new Lrhtask(agc).lrhtask();            // last pass
            return;
        }
        agc.set("LADQSAVE", pipctr);               // PIPCTR1 = LADQSAVE (decremented value)
        agc.set("A", 025);                          // OCT31
        new FreshStartAndRestart(agc).twiddle(() -> r10R11());
    }

    /** R10,R11A: flash the H and V lamps unless in lamp test, then proceed. */
    public void r10R11a() {
        if ((~agc.get("IMODES33") & BIT1) == 0) {
            lamp1011();                             // lamp test: leave LR lights alone
            return;
        }
        // FLASHH?
        if ((agc.get("FLGWRD11") & agc.get("HFLASH")) != 0) {
            new PinballGameButtonsAndLights(agc).flip(agc.get("HLITE"));
        }
        // FLASHV?
        if ((agc.get("FLGWRD11") & agc.get("VFLSHBIT")) != 0) {
            new PinballGameButtonsAndLights(agc).flip(agc.get("VLITE"));
        }
        lamp1011();
    }

    /** 10,11: check abort conditions then proceed to R10 (LANDISP). */
    public void lamp1011() {
        if ((agc.get("FLAGWRD9") & agc.get("LETABBIT")) == 0) {
            landisp();                              // no LETABORT
            return;
        }
        if (agc.get("MODREG") == DEC71) {           // P71NOW?
            landisp();
            return;
        }
        int ch30 = ~agc.get("CHAN30") & Agc.WORD_MASK;   // abort-stage discrete (inverted)
        agc.set("L", ch30);
        if ((ch30 & BIT4) != 0) {
            p71a();                                 // abort stage commanded
            return;
        }
        if (agc.get("MODREG") == DEC70) {           // P70NOW?
            landisp();
            return;
        }
        if ((ch30 & BIT1) != 0) {
            p70a();                                 // abort commanded
        } else {
            landisp();
        }
    }

    private void landisp() {
        new LandingAnalogDisplaysRoutines(agc).landisp();
    }

    // ---- P70 / P71 ---------------------------------------------------------------

    /** P70: DPS abort. */
    public void p70() {
        if (!legal()) return;
        p70a();
    }

    /** P70A: Q := 0 (DPS). */
    public void p70a() {
        contabrtEntry(0);
    }

    /** P71: APS abort. */
    public void p71() {
        if (!legal()) return;
        p71a();
    }

    /** P71A: Q := 2 (APS). */
    public void p71a() {
        contabrtEntry(2);
    }

    private void contabrtEntry(int q) {
        agc.set("Q", q);
        // INHINT; DCA CNTABTAD; DTCB -> run CONTABRT in the ABORTS1 bank
        contabrt();
    }

    /** CONTABRT: set up BRUPT to ABRTJASK and RESUME. */
    public void contabrt() {
        agc.set("BRUPT", 0); // ABRTJADR -> ABRTJASK
        abrtjask();         // RESUME lands on ABRTJASK
    }

    /** ABRTJASK: reinitialise everything for the abort program. */
    public void abrtjask() {
        int q = agc.get("Q");
        // 4.(Q+23)SPOT for GOABORT
        agc.set("L", OCTAL27 + q);
        agc.set("-PHASE4+1", agc.get("L"));
        agc.set("-PHASE4", ~agc.get("L") & Agc.WORD_MASK);
        int mode = (q == 0) ? MODE70 : MODE71;
        agc.set("MODREG", mode);
        agc.set("DISPDEX", mode);                   // insure DISPDEX is positive
        if (q != 0) {                               // set APSFLAG if P71
            agc.set("FLGWRD10", agc.get("FLGWRD10") | agc.get("APSFLBIT"));
        }
        agc.set("DAPBOOLS", agc.get("DAPBOOLS") & ~DAPBITS);   // ullage, drift, XOVINHIB off
        agc.set("FLAGWRD5", agc.get("FLAGWRD5") | agc.get("ENGONBIT"));   // set ENGONFLG
        // insure the engine is on, if armed
        int ds = agc.get("DSALMOUT") & ~agc.get("PRIO30") & Agc.WORD_MASK;
        agc.set("DSALMOUT", ds + BIT13);
        agc.set("FLGWRD11", agc.get("LRBYBIT"));    // terminate R12
        agc.set("FLAGWRD0", agc.get("FLAGWRD0") | agc.get("R10FLBIT"));   // suppress R10 outputs
        new RadarLeadingRoutines(agc).clradmod();    // CLRADMOD: RADMODES for R29
        agc.set("TEVENT", agc.get("TIME2"));         // TEVENT for downlink
        agc.set("TEVENT+1", agc.get("TIME1"));
        agc.set("AVGEXIT", agc.get("SERVEXIT"));     // SVEXITAD
        agc.set("-PHASE1", Agc.WORD_MASK); agc.set("-PHASE1+1", Agc.WORD_MASK);   // NEG0
        agc.set("-PHASE3", Agc.WORD_MASK); agc.set("-PHASE3+1", Agc.WORD_MASK);
        agc.set("-PHASE6", Agc.WORD_MASK); agc.set("-PHASE6+1", Agc.WORD_MASK);
        agc.set("L", 3);                             // 4.3SPOT for GOABORT
        agc.set("-PHASE4", ~3 & Agc.WORD_MASK);
        agc.set("-PHASE4+1", 3);
        agc.set("TIME5", OCT37774);                  // T5RUPT calls DAPIDLER in 40 ms
        new FreshStartAndRestart(agc).enema();       // POSTJUMP ENEMA
    }

    /** GOABORT: restart-group-4.3 entry, sets flags and branches to P70/P71 initialisation. */
    public void goabort() {
        new Interpreter(agc).callInitcduw();         // CALL INITCDUW
        agc.set("DVCNTR", 4);
        agc.set("WHICH", agc.get("WHICHADR"));
        downflag("FLRCS");
        downflag("FLUNDISP");
        downflag("IDLEFLAG");
        upflag("ACC4-2FL");                          // insure 4-jet translation capability
        if (agc.get("MODREG") == DEC70) {            // CHECKMM 70
            p70init();
        } else {
            p71ret();
        }
    }

    /** P70INIT: initialise DPS ascent abort guidance. */
    public void p70init() {
        tgocomp();
        double tgo4 = dp("MDOTDPS") * 16;                       // SL 4D
        double mass = dp("MASS");
        setDp("TBUP", mass / tgo4);                              // BDDV MASS ... STODL TBUP
        double kdv = dp("K(1/DV)");
        double invDv = (mass / 2) / kdv;                         // DDV, SR1
        setDp("1/DV1", invDv);
        setDp("1/DV2", invDv);
        setDp("1/DV3", invDv);
        setDp("AT", K_AT / invDv);                               // BDDV K(AT)
        setDp("TTO", -dp("DTDECAY") * 2048);                     // DCOMP, SL 11D
        setDp("VE", -dp("DPSVEX") / 4);                          // SLOAD DCOMP, SR2: DPS exhaust velocity
        agc.set("FLAP", 1);                                      // SET FLAP
        new Interpreter(agc).callCominit();                      // CALL COMMINIT
        bothpoly(0);                                             // AXC,1 0D : DPS coefficients
    }

    /** INJTARG: return here in P71, X1 selects APS coefficients (8D). */
    public void injtarg() {
        bothpoly(8);
    }

    /** BOTHPOLY: cubic polynomial in TGO for the target vertical velocity. */
    public void bothpoly(int x1) {
        double tgo = dp("TGO");
        double d = dp("ABTCOF+" + (x1 + 0));
        double c = dp("ABTCOF+" + (x1 + 2));
        double b = dp("ABTCOF+" + (x1 + 4));
        double a = dp("ABTCOF+" + (x1 + 6));
        double zdotd = a + tgo * (b + tgo * (c + tgo * d));     // A+TGO(B+TGO(C+TGO D))
        setDp("ZDOTD", zdotd);
        if (zdotd - dp("VMIN") < 0) {                            // too small: use the minimum
            setDp("ZDOTD", dp("VMIN"));
        }
        // UPRATE
        setDp("RDOTD", dp("ABTRDOT"));
        new Interpreter(agc).callYcomp();                        // compute Y
        double y = dp("Y");
        double yco;
        if (Math.abs(y) - dp("YLIM") < 0) {
            yco = dp("YCO");                                     // XR < .5 deg: leave YCO at 0
        } else {
            yco = Math.copySign(Math.abs(dp("DYMAX")), y);       // fix sign of deficit
        }
        setDp("YCO", yco);
        setDp("XRANGE", (yco - y) / 32);                         // SR 5D
        upthrot();
    }

    /** UPTHROT: set flags, throttle up and arrange guidance. */
    public void upthrot() {
        agc.set("FLVR", 1);                                      // SET FLVR
        upflag("ROTFLAG");
        throtup();
        new FreshStartAndRestart(agc).phaschng(04024);
        new Interpreter(agc).bankcallP40auto();                  // verify panel switches
        throtup();
        upthrot1();
    }

    /** UPTHROT1: set servicer to call ascent guidance (ATMAG). */
    public void upthrot1() {
        agc.set("AVGEXIT", agc.get("ATMAG"));
        grp4off();
    }

    /** GRP4OFF: terminate use of group 4. */
    public void grp4off() {
        new FreshStartAndRestart(agc).phaschng(00004);
        new Executive(agc).endofjob();
    }

    /** P71RET: P71 initialisation path. */
    public void p71ret() {
        downflag("LETABORT");
        agc.set("DVTHRUSH", agc.get("THRESH2"));                 // ascent DVMON threshold
        if (agc.get("FLAP") != 0) {                              // BON FLAP -> OLDTIME
            oldtime();
            return;
        }
        tgocomp();                                               // FLAP=0: TGO = T - TIG
        agc.set("QPRET", agc.get("INJTARG"));
        new Interpreter(agc).callP12init();                      // exits P12INIT to INJTARG
    }

    /** OLDTIME: if FLAP=1, TGO := 2*TGO. */
    public void oldtime() {
        setDp("TGO1", dp("TGO") * 2);                            // SL1 ... STCALL TGO1
        new Interpreter(agc).callP12init();
        new FreshStartAndRestart(agc).phaschng(04024);
        agc.set("TGO", agc.get("TGO1"));
        agc.set("TGO+1", agc.get("TGO1+1"));
        upthrot1();
    }

    /** LEGAL?: false (after alarm) if the desired program is in progress or aborts disabled. */
    public boolean legal() {
        boolean inProgress = agc.get("MMNUMBER") == agc.get("MODREG");
        boolean abortsEnabled = (agc.get("FLAGWRD9") & agc.get("LETABBIT")) != 0;
        boolean servicerOn = (agc.get("FLAGWRD7") & agc.get("AVEGFBIT")) != 0;
        if (!inProgress && abortsEnabled && servicerOn) {
            return true;
        }
        abortalm();
        return false;
    }

    /** ABORTALM: alarm, release display and re-enter PINBALL. */
    public void abortalm() {
        new Alarm(agc).falton();
        new PinballGameButtonsAndLights(agc).reldsp();
        new InterBankCommunication(agc).postjump(0, () -> new PinballGameButtonsAndLights(agc).pinbrnch());
    }

    /** TGOCOMP: TGO = (TIME - TIG) * 2^11 (RTB LOADTIME; DSU TIG; SL 11D). */
    public void tgocomp() {
        new RtbOpCodes(agc).loadtime();
        double t = dp("MPAC") - dp("TIG");
        setDp("TGO", t * 2048);
    }

    /** THROTUP: set thrust to maximum (BIT13) and enable throttle via channel 14 bit 4. */
    public void throtup() {
        agc.set("THRUST", BIT13);
        agc.set("CHAN14", agc.get("CHAN14") | BIT4);
    }

    // ---- flag helpers --------------------------------------------------------------
    private void upflag(String flag) {
        new FlagwordAssignments(agc).upflag(flag);
    }

    private void downflag(String flag) {
        new FlagwordAssignments(agc).downflag(flag);
    }

    /** ATMAGAD / ORBMANAD cells: referenced as 2CADR / ADRES constants elsewhere. */
    public static final String ATMAGAD = "ATMAG";
    public static final String ORBMANAD = "ORBMANUV";
    public static final String WHICHADR = "ABRTABLE";
}
