package apollo.luminary099;

import apollo.agc.Agc;

/**
 * Translation of PINBALL_GAME_BUTTONS_AND_LIGHTS.agc (Luminary 1A build 099).
 *
 * The Keyboard and Display (DSKY) program, "PINBALL". It operates under Executive control
 * and processes information exchanged between the AGC and the operator: DSKY keys (via
 * KEYRUPT1), the uplink (UPRUPT) and internal programs (via NVSUB). The language is a
 * VERB and NOUN pair, each a two character decimal number. Verbs are displays, loads,
 * monitors (displays updated once per second), special functions and extended verbs
 * (see EXTENDED_VERBS). Output goes to the DSPTAB buffer, which DSPOUT (T4RUPT) moves to
 * output channel 10.
 *
 * Input key codes: 0=20 oct (10000b), 1-7 = 1-7, 8,9 = 10,11, VERB=21, ERROR RESET=22,
 * KEY RELEASE=31, + =32, - =33, ENTER=34, CLEAR=36, NOUN=37.
 *
 * Display panel relay table (DSPTAB 0-10, entry 11 holds C relays): MD1=25 MD2=24 VD1=23
 * VD2=22 ND1=21 ND2=20 R1D1=16 ... R1D5=12, R2D1=11 ... R2D5=5, R3D1=4 ... R3D5=0 (octal
 * DSPCOUNT numbers). Channel 11 bits: KEY RELEASE light = bit 5, V/N flash = bit 6,
 * OPERATOR ERROR = bit 7.
 *
 * Modelling notes: AGC subroutine return addresses (Q/L/ENTRET/REQRET) are modelled
 * with Java calls and {@link Runnable} continuations; "TC ENDOFJOB" calls the Executive
 * and then the Java method simply returns. Erasable names (DSPCOUNT, DECBRNCH, VERBREG,
 * NOUNREG, XREG, YREG, ZREG, XREGLP, YREGLP, ZREGLP, MODREG, DSPLOCK, REQRET, LOADSTAT,
 * CLPASS, NOUT, NOUNCADR, MONSAVE, MONSAVE1, MONSAVE2, DSPTAB+0..13, CADRSTOR, NVQTEM,
 * NVBNKTEM, VERBSAVE, DSPLIST, EXTVBACT, DSPTEM1, DSPTEM2 and temporaries) are accessed
 * by name through {@link Agc}.
 */
public class PinballGameButtonsAndLights {
    // ---- bit constants (BITn = bit n counting from 1) -------------------------------------
    static final int BIT1 = 01, BIT2 = 02, BIT3 = 04, BIT4 = 010, BIT5 = 020, BIT6 = 040,
            BIT7 = 0100, BIT8 = 0200, BIT9 = 0400, BIT10 = 01000, BIT11 = 02000,
            BIT12 = 04000, BIT13 = 010000, BIT14 = 020000, BIT15 = 040000;
    static final int LOW5 = 037, MID5 = 01740, HI5 = 076000, LOW7 = 0177, LOW8 = 0377,
            LOW9 = 0777, LOW10 = 01777, LOW11 = 03777, MID7 = 037600;
    static final int POSMAX = 037777;
    static final int OCT1400 = 01400;

    // ---- constants -------------------------------------------------------------------------
    public static final int CHRPRIO = 030000;       // Executive priority of CHARIN
    public static final int VD1 = 023, ND1 = 021, MD1 = 025;   // packed display-number table
    public static final int R1D1 = 016, R2D1 = 011, R3D1 = 04;
    public static final int ELRCODE1 = 022;         // ERROR RESET key code
    public static final int LOWVERB = 28;           // lowest verb that avoids the noun test
    public static final int LST2CON = 40;           // first LIST2 (extended) verb
    public static final int BINCON = 10;
    public static final int MONDEL = 0144;          // 1 second monitor interval (centiseconds)
    public static final int BLANKCON = 04000;
    public static final int[] CRITCON = {022, 020, 012, 5, 0};
    public static final int[] INRELTAB = {4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, -1, 1, 1, 0, 0};
    public static final int[] SINBLANK = {014, 5, 4};
    public static final int[] DOUBLK = {013, 9, 3};
    public static final int BRNCHCON = 077774;
    public static final int[] SGNTAB = {5, 3, 0};   // -R1, -R2, -R3
    public static final int[] DSMSK = {037, 01740, 02000, 03740};
    public static final int DECON_NUM = 1;          // 2DEC 1 E-5 B14 = .16384 (see decon())
    public static final double DECON = 1e-5 * (1 << 14);
    public static final int[] DEGTAB = {05605, 03656, 016314, 031463};  // .18 and .45 (hi,lo)
    public static final int[] SECON2 = {01727, 01217};                  // .06
    public static final int[] MINCON2 = {00011, 032445};                // .0006
    public static final int[] MINCON1 = {02104, 010422};                // .066..66 + 2^-28
    public static final int[] HRCON1 = {02000 * 0 + 5243, 0};           // 2DEC .16384 (approx hi word)
    public static final int RNDCON = 062;                               // .5 s
    public static final int HISECON = 023147, HIMINCON = 023346;
    public static final int M_S_CON1 = 077753, M_S_CON2 = 041126;
    public static final int[] M_S_CON3 = {00025, 037016};               // 59m59.5s
    public static final int[] WHOLECON = {6, 03240};
    public static final int[] HRCON = {025, 037100};
    public static final int MINCON = 013560, MIN59 = 073, SEC5999 = 013557;
    public static final int FULLDSP = 05675, FULLDSP1 = 07675, TSTCON1 = 0175,
            TSTCON2 = 040674, TSTCON3 = 0115, SHOLTS = 0764;
    public static final int RFAILS2 = 0330, OCT55000 = 055000, C13_11_1 = 016001;
    public static final int ERCON = 0104, RFAILS = 0330, GL_NOATT = 050, NOTBIT12 = 073777;
    public static final int MONREF = 075377;
    public static final int VNDSPCON = 0244;                            // .01 rounded up
    /** Relay codes for digits 0-9 and blank (RELTAB lives in T4RUPT_PROGRAM). */
    public static final int[] RELTAB = {025, 03, 031, 033, 017, 036, 034, 023, 035, 037};

    private final Agc agc;

    /** ENTRET: continuation after the verb has done its work (TC ENDOFJOB, NVSUBEND, PASTEVB). */
    private Runnable entret = () -> { };
    /** The REQRET continuation: where ENTER (higher pass) resumes after data entry. */
    private Runnable reqretTarget = () -> { };

    public PinballGameButtonsAndLights(Agc agc) {
        this.agc = agc;
    }

    // ---- helpers ----------------------------------------------------------------------------
    private int g(String s) { return agc.get(s); }
    private void s(String sym, int v) { agc.set(sym, v); }
    private static int sgn(int w) { return Agc.toSigned(w); }
    private static int neg(int w) { return ~w & Agc.WORD_MASK; }
    private void endofjob() { new Executive(agc).endofjob(); }
    private void inrel(int v) { s("INREL", v); }
    private int inrel() { return g("INREL"); }

    /** Abort: used for CCSHOLE, ENDIDLE misuse etc. */
    private void poodoo(int code) { new AlarmAndAbort(agc).poodoo(code); }

    // ======================================================================================
    // CHARIN: keyboard character input (Executive job started by KEYRUPT1/UPRUPT)
    // ======================================================================================

    /** CHARIN: block display system, save old DSPLOCK, and fan out on the key code in MPAC. */
    public void charin() {
        int oldLock = g("DSPLOCK");
        s("DSPLOCK", 1);                              // make DSP system busy
        s("21/22REG", oldLock);                       // saved for error light reset
        if (sgn(g("CADRSTOR")) != 0 && g("MPAC") != ELRCODE1) {
            relDspOn();                               // all keys but ER turn on KR light
        }
        charin2();
    }

    /** CHARIN2: dispatch on key code. */
    public void charin2() {
        int key = g("MPAC");
        s("CHAR", key);
        switch (key) {
            case 0: charalrm(); break;
            case 1: case 2: case 3: case 4: case 5: case 6: case 7: num(); break;
            case 010: case 011: test89(); break;
            case 020: s("CHAR", 0); num(); break;     // "0" key (NUM -2)
            case 021: verb(); break;
            case 022: error(); break;
            case 031: vbreldsp(); break;
            case 032: posgn(); break;
            case 033: negsgn(); break;
            case 034: enterjmp(); break;
            case 036: clear(); break;
            case 037: noun(); break;
            default: charalrm();
        }
    }

    /** ENTERJMP: POSTJUMP to ENTER. */
    public void enterjmp() { enter(); }

    /** 89TEST: 8 and 9 are rejected for octal (DECBRNCH low bits = 0). */
    public void test89() {
        int dc = sgn(g("DSPCOUNT"));
        if (dc < 0 || g("DSPCOUNT") == Agc.WORD_MASK) { endofjob(); return; }   // data blocked
        if ((g("DECBRNCH") & 3) != 0) {
            num();                                    // decimal: 8 or 9 is OK
        } else {
            charalrm();                               // octal: reject
        }
    }

    // ======================================================================================
    // NUM: assemble octal 3 bits at a time; for decimal convert incoming word as a fraction
    // ======================================================================================

    /**
     * NUM: octal results are left in XREG/YREG/ZREG (or VERBREG/NOUNREG); hi part of decimal
     * in XREG..., low parts in XREGLP... DECBRNCH: 0 octal, 1 +dec, 2 -dec. DSPCOUNT negative
     * blocks further data.
     */
    public void num() {
        int dc = g("DSPCOUNT");
        if (sgn(dc) < 0 || dc == Agc.WORD_MASK) {
            if (sgn(dc) < 0 && dc != Agc.WORD_MASK) { /* -nz: block data in */ }
            endofjob();
            return;
        }
        getinrel();
        if (sgn(g("CLPASS")) >= 0) s("CLPASS", 0);          // + or +0 becomes +0
        int code = RELTAB[g("CHAR") & 7 > 9 ? 0 : relIndex(g("CHAR"))] & LOW5;
        s("CODE", code);
        s("COUNT", g("DSPCOUNT"));
        dspin();
        int br = g("DECBRNCH") & 3;
        String reg = regName(inrel());
        if (br == 0) {                                      // octal: shift left 3 and add digit
            int v = (g(reg) << 3) & Agc.WORD_MASK;
            endnmtst(v + g("CHAR"));
        } else {
            dectobin(reg);
        }
    }

    private static int relIndex(int key) {
        // key codes 1..7 -> digits 1..7, 010 -> 8, 011 -> 9, 0 -> 0
        return key & 017;
    }

    /** Name of the data register for INREL: 0 VERBREG, 1 NOUNREG, 2 XREG, 3 YREG, 4 ZREG. */
    private static String regName(int inrel) {
        switch (inrel) {
            case 0: return "VERBREG";
            case 1: return "NOUNREG";
            case 2: return "XREG";
            case 3: return "YREG";
            default: return "ZREG";
        }
    }

    /** DECTOBIN: decimal accumulation as a DP fraction in MPAC,MPAC+1 (10*sum + digit). */
    public void dectobin(String reg) {
        s("MPAC", g(reg));                                  // sum x 2^-14
        s("MPAC+1", 0);
        new Interpreter(agc).shortmp(10);                   // 10*sum x 2^-28 in MPAC, MPAC+1
        int lo = Agc.add(g("MPAC+1"), g("CHAR"));
        s("MPAC+1", lo);
        if (sgn(g("A")) != 0) {                             // overflow: must be the 5th char
            s("MPAC", Agc.add(g("MPAC"), g("A")));
            decend();
            return;
        }
        endnmtst(g("MPAC"));
    }

    /** ENDNMTST: store accumulated value; test whether the register is full. */
    public void endnmtst(int value) {
        s(regName(inrel()), value);
        if (neg(g("DSPCOUNT")) + CRITCON[Math.max(0, inrel() - 0)] == 0 || endCondition()) {
            endnum();
        } else {
            mornum();
        }
    }

    private boolean endCondition() {
        return (-sgn(g("DSPCOUNT")) + CRITCON[inrel()]) == 0;
    }

    /** ENDNUM: decimal needs DECEND; octal just blocks input. */
    public void endnum() {
        if ((g("DECBRNCH") & 3) != 0) decend(); else endall();
    }

    /** ENDALL: block NUMIN by placing DSPCOUNT negatively. */
    public void endall() {
        s("DSPCOUNT", neg(g("DSPCOUNT")));
        mornum1();
    }

    /** DECEND: scale decimal by 2^14/10^5 for R1,R2,R3 and apply sign. */
    public void decend() {
        if (inrel() <= 1) {                                 // VERBREG, NOUNREG: leave whole
            endall();
            return;
        }
        new Interpreter(agc).dmp(DECON);                    // (sum/10^5) x 2^-14 in MPAC..+2
        if ((g("DECBRNCH") & 3) == 2) {                     // - case: negate the DP value
            int hi = neg(g("MPAC+1")), lo = neg(g("MPAC+2"));
            s("MPAC+1", lo);
            s("MPAC+2", hi);
        }
        s(regName(inrel()) + "LP", g("MPAC+2"));            // XREGLP-2 +INREL
        s(regName(inrel()), g("MPAC+1"));
        endall();
    }

    /** MORNUM: decrement DSPCOUNT (more characters expected) and end of job. */
    public void mornum() {
        mornum1();
    }

    private void mornum1() {
        int dc = g("DSPCOUNT");
        s("DSPCOUNT", dc > 0 && dc < BIT14 ? dc - 1 : dc);   // CCS decrement
        endofjob();
    }

    /** GETINREL: INREL := INRELTAB[DSPCOUNT] (0 VERBREG, 1 NOUNREG, 2 XREG, 3 YREG, 4 ZREG). */
    public void getinrel() {
        int idx = g("DSPCOUNT");
        if (idx == 15) { new AlarmAndAbort(agc).ccshole(); return; }
        inrel(INRELTAB[Math.min(idx, 19)]);
    }

    // ======================================================================================
    // VERB / NOUN / sign / clear keys
    // ======================================================================================

    /** VERB key: clear VERBREG, DSPCOUNT = VD1. */
    public void verb() {
        s("VERBREG", 0);
        nvcom(VD1);
    }

    /** NOUN key: clear NOUNREG, DSPCOUNT = ND1. */
    public void noun() {
        s("NOUNREG", 0);
        nvcom(ND1);
    }

    /** NVCOM: blank the two characters, set DEC, REQRET and ENTRET for the verb/noun. */
    public void nvcom(int count) {
        s("DSPCOUNT", count);
        blank2();
        s("DECBRNCH", 1);                                    // decimal V/N code
        s("REQRET", 0);                                      // set for ENTPAS0
        entret = this::endofjob;                             // TC ENDOFJOB (ENDINST)
        endofjob();
    }

    /** - key. */
    public void negsgn() {
        if (!signtest()) return;
        minusOn();
        bothsgn(2);
    }

    /** BOTHSGN: set the decimal-complement bit for the register in DECBRNCH. */
    public void bothsgn(int add) {
        s("DECBRNCH", g("DECBRNCH") + add + (BIT7 >> 0) * 0);
        s("DECBRNCH", g("DECBRNCH") + signBit());
        fixclpas();
    }

    private int signBit() {
        return 0;     // INDEX INREL; AD BIT7 : the register bit is folded in by getinrel() caller
    }

    /** FIXCLPAS: if CLPASS is + or +0 make it +0 and end the job. */
    public void fixclpas() {
        if (sgn(g("CLPASS")) >= 0) s("CLPASS", 0);
        endofjob();
    }

    /** + key. */
    public void posgn() {
        if (!signtest()) return;
        plusOn();
        bothsgn(1);
    }

    /** +ON: show + sign for the register (INREL) and turn the - sign relay off. */
    public void plusOn() {
        getinrel();
        s("SGNOFF", SGNTAB[inrel() - 2]);
        s("SGNON", g("SGNOFF") + 1);
        sgncom();
    }

    /** -ON: show - sign for the register. */
    public void minusOn() {
        getinrel();
        s("SGNON", SGNTAB[inrel() - 2]);
        s("SGNOFF", g("SGNON") + 1);
        sgncom();
    }

    /** SGNCOM: turn sign relays off/on via 11DSPIN. */
    public void sgncom() {
        s("CODE", 0);
        dspin11(g("SGNOFF"));
        s("CODE", BIT11);
        dspin11(g("SGNON"));
    }

    /**
     * SIGNTEST: signs allowed only when DSPCOUNT = R1D1, R2D1 or R3D1 and only the first of
     * consecutive sign characters.
     * @return true if the sign is legal (otherwise ENDOFJOB has been called)
     */
    public boolean signtest() {
        if ((g("DECBRNCH") & 3) != 0) { endofjob(); return false; }
        int dc = g("DSPCOUNT");
        if (dc == R1D1 || dc == R2D1 || dc == R3D1) return true;
        endofjob();
        return false;
    }

    /**
     * CLEAR: blank the R1, R2 or R3 currently or last displayed; successive clears take care
     * of each Rx lower than the last until R1 is done. CLPASS: +0 pass 0 (can back up),
     * +nz high pass, -nz pass 0 cannot back up.
     */
    public void clear() {
        int dc = sgn(g("DSPCOUNT"));
        int idx = dc >= 0 ? dc : -dc - 1;
        inrel(INRELTAB[Math.min(idx, 19)]);
        int clpass = sgn(g("CLPASS"));
        if (clpass > 0) {                                    // CLPASHI
            int r = inrel() - 1;
            if (r < 0) r = 0;
            inrel(r);
            if (!legaltst()) return;
            s("REQRET", g("REQRET") + DOUBLK[2] * 0 - 0 + 0);
            s("REQRET", Agc.add(g("REQRET"), Agc.fromSigned(-3)));   // back up data requests
            s("MIXTEMP", inrel());
            s("VERBREG", Math.max(0, g("VERBREG") - 1));
            updatvb(g("VERBREG"));                           // redisplay verb
            inrel(g("MIXTEMP"));
            clear1();
            return;
        }
        if (!legaltst()) return;
        clear1();
    }

    /** CLEAR1: clear the register and bump CLPASS. */
    public void clear1() {
        clr5();
        if (sgn(g("CLPASS")) >= 0) s("CLPASS", g("CLPASS") + 1);
        endofjob();
    }

    /** CLR5: 5BLANK without its GETINREL. */
    public void clr5() { blank5(false); }

    /** LEGALTST: INREL must be >= 2. */
    public boolean legaltst() {
        if (inrel() >= 2) return true;
        endofjob();
        return false;
    }

    /**
     * 5BLANK: blank the 5 character display word in R1, R2 or R3 and zero XREG/YREG/ZREG.
     * Place the DSPCOUNT number of the register in DSPCOUNT first. DSPCOUNT is left at the
     * leftmost display number of the register just blanked.
     */
    public void blank5() { blank5(true); }

    private void blank5(boolean getinrel) {
        if (getinrel) getinrel();
        int r = inrel();
        s(regName(r), 0);
        s(regName(r) + "LP", 0);                             // XREGLP-2+INREL
        s("CODE", 0);
        s("DECBRNCH", g("DECBRNCH") & ~BIT7 & BRNCHCON);     // zero the DEC bit, low 2 bits
        s("COUNT", SINBLANK[r - 2]);                         // isolated char blanked separately
        dspin();
        s("DSPCOUNT", DOUBLK[r - 2]);
        blank2();
        s("DSPCOUNT", g("DSPCOUNT") - 2);
        blank2();
        s("DSPCOUNT", new int[] {R1D1, R2D1, R3D1}[r - 2]); // leftmost display number
    }

    /**
     * 2BLANK: blank two characters; the display number of the left character of the pair is
     * in DSPCOUNT and is left there.
     */
    public void blank2() {
        int idx = g("DSPCOUNT") >> 1;                        // SR
        int old = g("DSPTAB+" + idx);
        s("DSPTAB+" + idx, neg(BLANKCON));                   // stored negative = new data
        if (sgn(old) > 0) s("NOUT", g("NOUT") + 1);          // old + : one more activity
    }

    // ======================================================================================
    // ENTER
    // ======================================================================================

    /**
     * ENTER: pass 0 is the execute function; higher passes load data. The sign of REQRET
     * determines the pass: + for pass 0, - for higher passes.
     */
    public void enter() {
        s("CLPASS", 0);
        entret = this::endofjob;
        if (sgn(g("REQRET")) >= 0) { entpas0(); return; }
        entpashi();
    }

    /** ENTPASHI: higher pass enter (data). */
    public void entpashi() {
        if (neg(g("REQRET")) == 0 || isMmChangRequest()) { acceptwd(); return; }
        if ((g("DECBRNCH") & 3) != 0 && sgn(g("DSPCOUNT")) >= 0) {
            godspalm();                                      // < 5 char decimal
            return;
        }
        acceptwd();
    }

    private boolean isMmChangRequest() {
        return g("REQRET") == Agc.fromSigned(-g("MMCHANG_REQ"));
    }

    /** ACCEPTWD: 5 char in (or octal): set REQRET + and continue at the saved address. */
    public void acceptwd() {
        s("REQRET", neg(g("REQRET")));
        flashoff();
        reqretTarget.run();
    }

    /** ENTPAS0: execute the verb/noun (NVSUB enters here too). */
    public void entpas0() {
        s("DECBRNCH", 0);
        s("DSPCOUNT", neg(VD1));                             // block stray char
        testvb();
    }

    /** TESTVB: if VERB >= LOWVERB skip the noun test. */
    public void testvb() {
        s("VERBSAVE", neg(g("VERBREG")));                    // saved for recycle
        if (LOWVERB - g("VERBREG") <= 0) { verbfan(); return; }
        testnn();
    }

    /** TESTNN: read the noun tables and branch on normal/mixed noun type. */
    public void testnn() {
        new PinballNounTables(agc).lodnntab();               // NNADTEM, NNTYPTEM, MIXBR...
        if (g("MIXBR") != 0) { mixnoun(); return; }
        int nnad = sgn(g("NNADTEM"));
        if (nnad > 0) { verbfan(); return; }                 // normal noun
        if (nnad == 0 && g("NNADTEM") == 0) { godspalm(); return; }   // not in use
        if (g("NNADTEM") != Agc.WORD_MASK) { reqadd(); return; }       // MCTBS (negative)
        s("NOUNCADR", g("NOUNCADR") + 1);                    // -0: augment machine CADR
        setnadd();
        intmctbs2();
    }

    /** REQADD: machine CADR to be specified. */
    public void reqadd() {
        s("CLPASS", BIT15);                                  // pass 0 only
        if (entret != null && isInternalEntry()) {
            intmctbs();                                      // internal MCTBS
            return;
        }
        reqdatz();                                           // external MCTBS
        if (g("DECBRNCH") != 0) { almcycle(); return; }      // decimal used for MCTBS
        s("DSPCOUNT", neg(VD1));
        if (sgn(g("CADRSTOR")) == 0) flashon();
        useadd();
    }

    private boolean isInternalEntry() {
        return g("ENTEXIT_IS_ENDINST") == 0;
    }

    /** USEADD: ECADR from ZREG into NOUNCADR and continue with the noun tables. */
    public void useadd() {
        setncadr(g("ZREG"));
        new PinballNounTables(agc).lodnntab();
        verbfan();
    }

    /** INTMCTBS: internal MCTBS (CADR left in MPAC+2 by NVSUB). */
    public void intmctbs() {
        setncadr(g("MPAC+2"));
        intmctbs2();
    }

    private void intmctbs2() {
        if (g("VERBREG") == 5) { verbfan(); return; }        // don't display CADR if VB = 05
        s("DSPCOUNT", R3D1);
        dspoctwo(g("NOUNCADR"));
        verbfan();
    }

    /** VERBFAN: dispatch verb 0..39 through VERBTAB; verbs >= 40 are extended verbs. */
    public void verbfan() {
        int v = g("VERBREG");
        if (v - LST2CON >= 0) {                              // extended verb
            s("MPAC", v - LST2CON);
            reldsp();
            new ExtendedVerbs(agc).goextvb();                // POSTJUMP GOEXTVB with VB-40 in MPAC
            return;
        }
        vbfandir(v);
    }

    /** VBFANDIR: BANKJUMP through VERBTAB. */
    public void vbfandir(int v) {
        switch (v) {
            case 1: dspa(); break;
            case 2: dspb(); break;
            case 3: dspc(); break;
            case 4: dspab(); break;
            case 5: dspabc(); break;
            case 6: decdsp(); break;
            case 7: dspdpdec(); break;
            case 11: case 12: case 13: case 14: case 15: case 16: case 17: monitor(); break;
            case 21: aload(); break;
            case 22: bload(); break;
            case 23: cload(); break;
            case 24: abload(); break;
            case 25: abcload(); break;
            case 27: dspfmem(); break;
            case 30: vbrqexec(); break;
            case 31: vbrqwait(); break;
            case 32: vbreseq(); break;
            case 33: vbproc(); break;
            case 34: vbterm(); break;
            case 35: vbtstlts(); break;
            case 36: new FreshStartAndRestart(agc).slap1(); break;
            case 37: mmchang(); break;
            case 10: dspalarm(); break;
            default: godspalm();                              // 0, 8, 9, 18-20, 26, 28, 29, 38, 39 spare
        }
    }

    // ======================================================================================
    // MIXNOUN
    // ======================================================================================

    /**
     * MIXNOUN: NNADTAB holds an indirect address (IDADDREL in low 10 bits) of 3 consecutive
     * addresses in IDADDTAB. MIXNOUN fetches the data into MIXTEMP,+1,+2 and sets NOUNADD.
     */
    public void mixnoun() {
        int nn = g("NNADTEM");
        if (nn == 0) { godspalm(); return; }                 // +0 not in use
        if (LOW10 != 0 && 6 - g("VERBREG") < 0) { verbfan(); return; }   // only for display verbs
        for (int k = 2; k >= 0; k--) {
            s("DECOUNT", k);
            s("NOUNADD", mixtempAddr(k));
            s("NOUNTEM", g("IDAD" + (k + 1) + "TEM"));
            int rut = sfrutmix();
            int e = g("NOUNTEM");
            if (isDp(rut)) e += 1;                           // DP: minor part
            int addr = setebank(e & LOW11);
            s("MIXTEMP+" + k, g("E:" + addr));               // pick up C(ESUBK) / C(ESUBK+1)
        }
        verbfan();
    }

    private int mixtempAddr(int k) { return k; }

    /**
     * DPTEST: scale-factor routine numbers that deliver DP data are 4 (DP1OUT), 5 (DP2OUT),
     * 7 (DP3OUT), 10 (DP4OUT).
     */
    public boolean isDp(int rut) {
        return rut == 4 || rut == 5 || rut == 7 || rut == 10;
    }

    // ======================================================================================
    // Data request routines
    // ======================================================================================

    public void reqdatx() { reqcom(R1D1); }

    public void reqdaty() { reqcom(R2D1); }

    public void reqdatz() { reqcom(R3D1); }

    /** REQCOM: blank the register, set REQRET so ENTER (higher pass) returns, and flash. */
    public void reqcom(int count) {
        s("DSPCOUNT", count);
        s("REQRET", neg(1));                                  // CS Q : negative return indicator
        blank5();
        flashon();
        // ENDRQDAT: return through ENTEXIT (ends job until the operator keys ENTER)
        entexit();
    }

    /** UPDATNN: put NOUNREG into NOUNCADR path and display the noun. */
    public void updatnn(int noun) {
        s("NOUNREG", noun);
        new PinballNounTables(agc).lodnntab();
        if (sgn(g("NNADTEM")) > 0) {
            setncadr(g("NNADTEM"));                            // PUTADD: normal noun
        }
        s("DSPCOUNT", ND1);
        updat1(noun);
    }

    /** UPDATVB: put the verb into VERBREG and display it. */
    public void updatvb(int verb) {
        s("VERBREG", verb);
        s("DSPCOUNT", VD1);
        updat1(verb);
    }

    private void updat1(int value) {
        dspdecvn(value);                                       // GOVNUPDT
    }

    /** GOALMCYC: needed because BANKJUMP can't handle F/F. */
    public void goalmcyc() { almcycle(); }

    /** GODSPALM: POSTJUMP to DSPALARM. */
    public void godspalm() { dspalarm(); }

    // ======================================================================================
    // Display verbs
    // ======================================================================================

    /** DSPABC: octal display of components 1,2,3. */
    public void dspabc() {
        if (!comptest(2)) return;
        s("BUF+2", neg(g("E:" + g("NOUNADD") + "+2")));
        dspab();
    }

    /** DSPAB: octal display of components 1,2. */
    public void dspab() {
        if (!comptest(1)) return;
        s("BUF+1", neg(g("E:" + g("NOUNADD") + "+1")));
        dspa();
    }

    /** DSPA: component 1 (octal), also entry for DSPB/DSPC via DSPCOM1. */
    public void dspa() {
        if (!dectest()) return;
        tstfordp();
        s("BUF", neg(g("E:" + g("NOUNADD"))));
        dspcom2();
    }

    /** DSPB: component 2. */
    public void dspb() {
        if (!dcomptst(1)) return;
        s("BUF", neg(g("E:" + g("NOUNADD") + "+1")));
        dspcom2();
    }

    /** DSPC: component 3. */
    public void dspc() {
        if (!dcomptst(2)) return;
        s("BUF", neg(g("E:" + g("NOUNADD") + "+2")));
        dspcom2();
    }

    /** DSPCOM2/DSPCOM3: display BUF.. in R1,R2,R3 as 5 character octal words. */
    public void dspcom2() {
        int count = g("VERBREG") - 2;                         // A:-1 B:-0 C:+1 AB:+2 ABC:+3
        for (int i = Math.max(count, 0); ; i--) {
            if (count <= 0 && i <= 0 && count < 0) break;
            s("DISTEM", i);
            s("DSPCOUNT", new int[] {R1D1, R2D1, R3D1}[i]);
            dspoctwo(neg(g("BUF" + (i == 0 ? "" : "+" + i))));
            if (i == 0) break;
        }
        entexit();
    }

    /** COMPTEST: alarm if the verb's component number exceeds the noun's component count. */
    public boolean comptest(int verbComp) {
        return comptst1(verbComp);
    }

    private boolean comptst1(int verbComp) {
        int nounComp = (getcomp() >> 10) & 3;                  // LEFT5 then MASK THREE
        if (nounComp - verbComp >= 0) return true;
        godspalm();
        return false;
    }

    /** DCOMPTST: alarm if decimal-only bit is 1; otherwise regular COMPTEST. */
    public boolean dcomptst(int verbComp) {
        if (!dectest()) return false;
        return comptst1(verbComp);
    }

    /** DECTEST: alarm if the decimal-only bit (bit 4 of the component code) is 1. */
    public boolean dectest() {
        if ((getcomp() & BIT14) != 0) { godspalm(); return false; }
        return true;
    }

    /** DCTSTCYC: as DECTEST but alarm and recycle (load verbs). */
    public boolean dctstcyc() {
        if ((getcomp() & BIT14) != 0) { almcycle(); return false; }
        return true;
    }

    /** NOUNTEST: alarm if the no-load bit (bit 5 of the component code) is 1. */
    public boolean nountest() {
        if ((getcomp() & BIT15) != 0) { godspalm(); return false; }
        return true;
    }

    /** TSTFORDP: for DP data, display the minor part only (NOUNADD+1). */
    public void tstfordp() {
        if (g("NNADTEM") == Agc.fromSigned(-1)) { chandsp(); return; }   // channel to be specified
        if (g("MIXBR") != 0) return;                           // mixed case handled in MIXNOUN
        int rut = sfrutnor();
        if (isDp(rut)) s("NOUNADD", g("NOUNADD") + 1);
    }

    /** CHANDSP: display a channel (READ) value in octal. */
    public void chandsp() {
        int ch = g("NOUNCADR") & LOW9;
        s("BUF", neg(g("CHAN" + Integer.toOctalString(ch))));
        dspcom2();
    }

    /** GETCOMP: component code (hi 5 bits) of NNTYPTEM (normal) or NNADTEM (mixed). */
    public int getcomp() {
        return (g("MIXBR") == 0 ? g("NNTYPTEM") : g("NNADTEM")) & HI5;
    }

    /** DECDSP: decimal display (V06). */
    public void decdsp() {
        s("DECOUNT", (getcomp() >> 10) & 3);                   // component count - 1
        for (int i = g("DECOUNT"); i >= 0; i--) {              // DSPDCGET
            s("XREG" + (i == 0 ? "" : "+" + i), neg(g("E:" + (g("NOUNADD") + i))));
        }
        dsdcput(g("DECOUNT"));
    }

    /** DSPDCPUT: display each component with its scale factor routine and constant. */
    public void dsdcput(int decount) {
        for (int i = decount; i >= 0; i--) {
            s("DECOUNT", i);
            s("MPAC+1", 0);
            s("MPAC+2", 0);
            s("DSPCOUNT", new int[] {R1D1, R2D1, R3D1}[i]);
            s("MPAC", neg(g("XREG" + (i == 0 ? "" : "+" + i))));
            int sfcon = sfconum();
            s("SFTEMP1", sfcon);
            new PinballNounTables(agc).gtsfout();             // loads SFTEMP1, SFTEMP2
            int rut = g("MIXBR") == 0 ? sfrutnor() : sfrutmix();
            decdsp3(rut);
        }
        entexit();
    }

    /** DECDSP3: BANKJUMP through SFOUTABR to the scale-factor output routine. */
    public void decdsp3(int rut) {
        switch (rut) {
            case 0: predspal(); break;
            case 1: dspdcend(); break;
            case 2: degoutsf(); break;
            case 3: artoutsf(); break;
            case 4: dp1outsf(); break;
            case 5: dp2outsf(); break;
            case 6: lrposout(); break;
            case 7: dp3outsf(); break;
            case 8: hmsout(); break;
            case 9: msout(); break;
            case 10: dp2outsf(); break;
            case 11: arout1sf(); break;
            case 12: twoIntout(); break;
            case 13: threeSixtyCduo(); break;
            default: predspal();
        }
    }

    /** DSPDCEND: all SFOUT routines end here; display the decimal word. */
    public void dspdcend() {
        dspdecwd();
    }

    // ---- scale factor output routines ----------------------------------------------------------

    /** DEGOUTSF: scale by .18 the low 14 bits of the angle (add .18 for negative AGC range). */
    public void degoutsf() {
        s("MPAC+2", 0);                                         // index for full scale
        boolean negative = fixrange();
        if (negative) setaug();                                 // SFTEMP1,2 := augmenter
        degcom();
    }

    /** 360-CDUO: compute 360 - CDU angle then DEGOUTSF. */
    public void threeSixtyCduo() {
        threeSixtyCdu();
        degoutsf();
    }

    /** 360-CDU: MPAC := 360 degrees - angle (unless angle is 0 or 180). */
    public void threeSixtyCdu() {
        if ((g("MPAC") & POSMAX) != 0) {
            s("MPAC", Agc.add(neg(g("MPAC")), 1));
        }
    }

    /** LRPOSOUT: display 0..3 for channel 33 bits 7-6. */
    public void lrposout() {
        int v = neg((g("CHAN33") >> 5) & 03) & 3;               // bits 7-6 to bits 2-1, complemented
        s("MPAC", v);
        artoutsf();
    }

    /** SETAUG: load the DP augmenter constant into SFTEMP1,2. */
    public void setaug() {
        int i = g("MPAC+2");
        s("SFTEMP1", DEGTAB[2 * i]);
        s("SFTEMP2", DEGTAB[2 * i + 1]);
    }

    /** FIXRANGE: true (and sign masked out) if MPAC is negative. */
    public boolean fixrange() {
        if (sgn(g("MPAC")) >= 0) return false;
        s("MPAC", g("MPAC") & ~BIT15);
        return true;
    }

    /** DEGCOM: multiplier from DEGTAB times MPAC plus augmenter. */
    public void degcom() {
        int i = g("MPAC+2");
        new Interpreter(agc).shortmpDp(DEGTAB[2 * i], DEGTAB[2 * i + 1]);
        scoutend();
    }

    /** ARTOUTSF: arithmetic output, scale factor point at left of the DP constant. */
    public void artoutsf() {
        new Interpreter(agc).shortmpDp(g("SFTEMP1"), g("SFTEMP2"));
        scoutend();
    }

    /** SCOUTEND: POSTJUMP to DSPDCEND. */
    public void scoutend() { dspdcend(); }

    /** AROUT1SF: point between HI and LO parts; shift results left 14. */
    public void arout1sf() {
        new Interpreter(agc).shortmpDp(g("SFTEMP1"), g("SFTEMP2"));
        l14out();
    }

    /** DP1OUTSF: DP scale; result shifted by B14. */
    public void dp1outsf() {
        dpout();
        l14out();
    }

    /** L14/OUT: results shifted left 14 (MPAC+1,+2 become MPAC,MPAC+1). */
    public void l14out() {
        s("MPAC", g("MPAC+1"));
        s("MPAC+1", g("MPAC+2"));
        scoutend();
    }

    /** DP2OUTSF. */
    public void dp2outsf() {
        dpout();
        scoutend();
    }

    /** DP3OUTSF: shift left 7. */
    public void dp3outsf() {
        dpout();
        new Interpreter(agc).tpleftn(6);
        scoutend();
    }

    /** DPOUT: fresh DP data, force sign agreement, multiply by the DP scale factor. */
    public void dpout() {
        readlo();
        new Interpreter(agc).tpagree();
        new Interpreter(agc).dmpAt("SFTEMP1");
    }

    /**
     * 2INTOUT: display two contiguous SP positive integers as two positive decimals in
     * RxD1-RxD2 and RxD4-RxD5 (RxD3 blanked).
     */
    public void twoIntout() {
        blank5();
        plusOn();
        dspdecvn(g("MPAC"));
        s("DSPCOUNT", new int[] {R1D1, R2D1, R3D1}[g("DECOUNT")] - 3);
        readlo();
        dspdecvn(g("MPAC+1"));
        dspdcend();
    }

    /** READLO: fresh data for both HI and LO into MPAC,MPAC+1 (needed for time displays). */
    public void readlo() {
        int addr;
        if (g("MIXBR") == 0) {
            addr = g("NOUNADD");
        } else {
            addr = setebank(g("IDAD" + (g("DECOUNT") + 1) + "TEM") & LOW11);
        }
        s("MPAC", g("E:" + addr));
        s("MPAC+1", g("E:" + (addr + 1)));
        s("MPAC+2", 0);
    }

    /** HMSOUT: hours, minutes, seconds display (centiseconds in DP). */
    public void hmsout() {
        readlo();
        new Interpreter(agc).tpagree();
        sepsecnr();
        new Interpreter(agc).dmp(SECON2[0], SECON2[1]);          // x .06: centi-sec/10^5 mod 60
        s("DSPCOUNT", R3D1);
        dspdecwd();                                              // seconds mod 60
        sepmin();
        s("MPAC", MINCON2[0]);
        s("HITEMOUT", g("MPAC"));                                // whole hours
        new Interpreter(agc).prshrtmp(MINCON2[1], g("MPAC+1"));  // x .0006: minutes
        s("DSPCOUNT", R2D1);
        dspdecwd();
        new Interpreter(agc).prshrtmp(HRCON1[0], g("HITEMOUT")); // x .16384: hours
        s("DSPCOUNT", R1D1);
        dspdecwd();
        entexit();
    }

    /** M/SOUT: minutes/seconds display, limited to 59 minutes 59 seconds. */
    public void msout() {
        readlo();
        new Interpreter(agc).tpagree();
        long dp = ((long) sgn(g("MPAC")) << 14) + sgn(g("MPAC+1"));
        long limit = ((long) 037000 << 14) + 0;                 // 59m58.5s
        if (Math.abs(dp) >= ((long) -sgn(M_S_CON1 - 1) << 14)) {
            mslimit(dp);
        } else {
            msnorm();
        }
    }

    /** M/SLIMIT: force MPAC,MPAC+1 to +/- 59m59.5s and display 59m59s. */
    public void mslimit(long dp) {
        if (dp >= 0) {
            s("MPAC", M_S_CON3[0]);
            s("MPAC+1", M_S_CON3[1]);
        } else {
            s("MPAC", neg(M_S_CON3[0]));
            s("MPAC+1", neg(M_S_CON3[1]));
        }
        sepsecnr();
        msnorm1();
    }

    /** M/SNORM: normal minutes/seconds display. */
    public void msnorm() {
        sepsec();
        msnorm1();
    }

    private void msnorm1() {
        new Interpreter(agc).shortmp(HISECON);                   // x .6 + 2^-14: sec/100 mod 60
        s("DSPCOUNT", g("DSPCOUNT") - 3);
        dspdc2nr();                                              // seconds in D4D5
        s("CODE", 0);
        s("COUNT", R1D1 - 2);
        dspin();                                                 // blank middle char
        sepmin();
        new Interpreter(agc).mpHi(g("MPAC+1"), HIMINCON);        // x .6 + 2^-7: minutes
        s("DSPCOUNT", R1D1);
        dspdc2nr();
        dspdcend();
    }

    /** SEPSEC: round by +/- .5 s, then separate seconds. */
    public void sepsec() {
        long dp = ((long) sgn(g("MPAC")) << 14) + sgn(g("MPAC+1"));
        dp += dp >= 0 ? RNDCON : -RNDCON;
        s("MPAC", Agc.fromSigned((int) (dp >> 14)));
        s("MPAC+1", Agc.fromSigned((int) (dp & 037777)));
        sepsecnr();
    }

    /** SEPSECNR: leave fract sec/60 in MPAC,MPAC+1; whole minutes in bit 13 of LOTEMOUT. */
    public void sepsecnr() {
        new Interpreter(agc).dmp(0, 0);                          // x 2^12/6000 (SECON1)
        s("HITEMOUT", g("MPAC"));
        s("LOTEMOUT", g("MPAC+1"));
        new Interpreter(agc).tpsl1();
        new Interpreter(agc).tpsl1();                            // fract sec/60 in MPAC+1,+2
        s("MPAC", g("MPAC+1"));
        s("MPAC+1", g("MPAC+2"));
        s("MPAC+2", 0);
    }

    /** SEPMIN: whole minutes from LOTEMOUT; fract min/60 in MPAC+1; whole hours in MPAC. */
    public void sepmin() {
        int lo = g("LOTEMOUT");
        s("MPAC+1", (lo << 12) & Agc.WORD_MASK);
        s("MPAC", g("HITEMOUT"));
        new Interpreter(agc).dmp(MINCON1[0], MINCON1[1]);        // x 1/15
    }

    /** DSPDPDEC (V07): DP decimal display in R1,R2 (10 digits, sign in R1). */
    public void dspdpdec() {
        if (g("MIXBR") != 0) { dspalarm(); return; }
        s("MPAC", g("E:" + g("NOUNADD")));
        s("MPAC+1", g("E:" + (g("NOUNADD") + 1)));
        s("DSPCOUNT", R1D1);
        s("MPAC+2", 0);
        new Interpreter(agc).tpagree();
        dsp2dec();
        entexit();
    }

    // ======================================================================================
    // Load verbs
    // ======================================================================================

    /** ABCLOAD (V25): load components 1,2,3 into R1,R2,R3. */
    public void abcload() {
        if (!comptest(2) || !nountest()) return;
        updatvb(21); reqdatx();
        updatvb(22); reqdaty();
        updatvb(23); reqdatz();
        putxyz();
    }

    /** PUTXYZ: store the three loaded words; special handling for noun 7 (flag bits). */
    public void putxyz() {
        if (!alldcoc(6)) return;
        for (int k = 0; k < 3; k++) {
            int v = putcom(k);
            s("E:" + (g("NOUNADD") + k), v);
        }
        if (g("NOUNREG") != 7) { loadlv(); return; }
        // Noun 7: XREG = ECADR of flag word, YREG = bits, ZREG = 0 resets / non-zero sets bits
        setncadr(g("XREG"));
        int flagAddr = g("NOUNADD");
        int word = g("E:" + flagAddr);
        if (g("ZREG") != 0) {
            word |= g("YREG");                                  // set bits
        } else {
            word &= ~g("YREG");                                 // reset bits
        }
        s("E:" + flagAddr, word & Agc.WORD_MASK);
        loadlv();
    }

    /** ABLOAD (V24): components 1,2. */
    public void abload() {
        if (!comptest(1) || !nountest()) return;
        updatvb(21); reqdatx();
        updatvb(22); reqdaty();
        putxy();
    }

    /** PUTXY. */
    public void putxy() {
        if (!alldcoc(5)) return;
        for (int k = 0; k < 2; k++) {
            int v = putcom(k);
            s("E:" + (g("NOUNADD") + k), v);
        }
        loadlv();
    }

    /** ALOAD (V21). */
    public void aload() {
        reqdatx();
        int v = putcom(0);
        s("E:" + g("NOUNADD"), v);
        loadlv();
    }

    /** BLOAD (V22). */
    public void bload() {
        if (!comptest(1)) return;
        s("CLPASS", BIT15);                                      // pass 0 only
        reqdaty();
        int v = putcom(1);
        s("E:" + (g("NOUNADD") + 1), v);
        loadlv();
    }

    /** CLOAD (V23). */
    public void cload() {
        if (!comptest(2)) return;
        s("CLPASS", BIT15);
        reqdatz();
        int v = putcom(2);
        s("E:" + (g("NOUNADD") + 2), v);
        loadlv();
    }

    /** LOADLV: end of load: clear DECBRNCH, LOADSTAT := -0, release display, RECALTST. */
    public void loadlv() {
        s("DECBRNCH", 0);
        s("LOADSTAT", Agc.WORD_MASK);                            // -0: data in
        reldsp();
        s("DSPCOUNT", neg(VD1));                                 // block numeric chars, clears
        recaltst();
    }

    /**
     * ALLDC/OC: all data words loaded must be either all decimal or all octal (bits 3,4,5 of
     * DECBRNCH). {@code expected} is 6 for three components, 5 for two.
     */
    public boolean alldcoc(int expected) {
        int bits = (g("DECBRNCH") >> 2) & 7;                     // shifted right 2
        if (bits == 0) return true;                              // all octal
        if (bits == expected + 1 || bits + 1 == expected + 1 - 0) return true;
        almcycle();
        return false;
    }

    /** SFRUTNOR: scale factor routine number, normal case (MID5 of NNTYPTEM). */
    public int sfrutnor() {
        return (g("NNTYPTEM") & MID5) >> 5;
    }

    /** SFRUTMIX: scale factor routine number, mixed case (HI5/MID5/LOW5 of RUTMXTEM). */
    public int sfrutmix() {
        int k = g("DECOUNT");
        int mask = k == 0 ? LOW5 : k == 1 ? MID5 : HI5;
        int v = g("RUTMXTEM") & mask;
        return k == 0 ? v : k == 1 ? v >> 5 : v >> 10;
    }

    /** SFCONUM: 2 x (scale factor constant number). */
    public int sfconum() {
        if (g("MIXBR") == 0) return (g("NNTYPTEM") & LOW5) * 2;     // CONUMNOR
        int k = g("DECOUNT");
        int mask = k == 0 ? LOW5 : k == 1 ? MID5 : HI5;
        int v = g("NNTYPTEM") & mask;
        return (k == 0 ? v : k == 1 ? v >> 5 : v >> 10) * 2;
    }

    /**
     * PUTCOM: take the loaded word for component {@code k} (XREG/XREGLP...) and convert it
     * for storage: octal as is (DP minor part for DP scales), decimal through the
     * scale-factor input routines.
     * @return the word to store in the noun register
     */
    public int putcom(int k) {
        s("DECOUNT", k);
        s("MPAC+6", 0);
        String x = k == 0 ? "XREG" : k == 1 ? "YREG" : "ZREG";
        s("MPAC+1", g(x + "LP"));
        s("MPAC", g(x));
        int br = g("DECBRNCH") & 3;
        if (g("MIXBR") != 0) {
            setncadr(g("IDAD" + (k + 1) + "TEM") & LOW11);
            s("NOUNADD", g("NOUNADD") - k);
        } else {
            setnadd();
        }
        if (br != 0) {                                            // PUTDECSF
            return putdecsf();
        }
        if (!dctstcyc()) return 0;
        int rut = g("MIXBR") == 0 ? sfrutnor() : sfrutmix();
        if (isDp(rut)) {                                          // octal into DP: +0 major part
            s("E:" + g("NOUNADD"), 0);
            s("NOUNADD", g("NOUNADD") + 1);
        }
        return g("MPAC");
    }

    /** PUTDECSF: scale-factor input routine by SFINTABR. */
    public int putdecsf() {
        s("SFTEMP1", sfconum());
        new PinballNounTables(agc).gtsfin();                      // loads SFTEMP1, SFTEMP2
        int rut = g("MIXBR") == 0 ? sfrutnor() : sfrutmix();
        switch (rut) {
            case 0: almcycle(); return 0;
            case 1: return binround();
            case 2: case 13: return deginsf();
            case 3: return arthinsf();
            case 4: case 7: return dpinsf();
            case 5: return dpinsf2();
            case 8: return hmsin();
            case 10: return dpinsf4();
            case 11: return artin1sf();
            default: dspalarm(); return 0;                        // LRPOSOUT, M/S, 2INTOUT can't be loaded
        }
    }

    /** DEGINSF: decimal degrees; multiply by 5.555.. x 2^-3 and shift left 3 (360.00 overflows). */
    public int deginsf() {
        new Interpreter(agc).dmpConst(5.555555555 / 8);
        int lo = g("MPAC+1");
        s("MPAC+1", lo + (sgn(lo) != 0 ? BIT11 : -BIT11));         // round before shift
        new Interpreter(agc).tpsl1();
        new Interpreter(agc).tpsl1();
        testofuf();
        new Interpreter(agc).tpsl1();
        int mpac = sgn(g("MPAC"));
        if (mpac < 0) s("MPAC", neg(g("MPAC")));                  // use -magnitude + 1
        int of = sgn(g("MPAC+6"));
        if (of > 0) s("MPAC", neg(g("MPAC")) & POSMAX);           // overflow forces sign to 1
        if (of < 0 && sgn(g("MPAC")) < 0) s("MPAC", g("MPAC") & POSMAX);
        if ((g("MIXBR") == 0 ? sfrutnor() : sfrutmix()) != 2) {   // 360 - CDU degrees
            threeSixtyCdu();
        }
        return g("MPAC");
    }

    /** ARTHINSF: DP times SFTEMP1,2; results shifted left 14. */
    public int arthinsf() {
        new Interpreter(agc).dmpAt("SFTEMP1");
        int hi = g("MPAC+2");
        s("MPAC", g("MPAC+1"));
        s("MPAC+1", hi);
        if (g("MPAC") != 0) { almcycle(); return 0; }              // too large a load
        return binround();
    }

    /** BINROUND: round MPAC+1 into MPAC; test overflow/underflow. */
    public int binround() {
        tworound();
        testofuf();
        return g("MPAC");
    }

    /** ARTIN1SF. */
    public int artin1sf() {
        new Interpreter(agc).dmpAt("SFTEMP1");
        return binround();
    }

    /** DPINSF: DP load: major part in MPAC, minor to (E SUBK)+1 / E+1. */
    public int dpinsf() {
        new Interpreter(agc).dmpAt("SFTEMP1");
        s("MPAC+2", g("MPAC+2") * 2);
        tworound();
        testofuf();
        int minor = g("MPAC+1");
        int base = g("MIXBR") == 0 ? g("NOUNADD") : g("NOUNADD") + g("DECOUNT");
        s("E:" + (base + 1), minor);
        return g("MPAC");
    }

    /** DPINSF2: point between bits 7-8 of the hi part; shift left 7. */
    public int dpinsf2() {
        new Interpreter(agc).dmpAt("SFTEMP1");
        new Interpreter(agc).tpleftn(6);
        return dpinsf();
    }

    /** DPINSF4: point between bits 11-12; shift left 3. */
    public int dpinsf4() {
        new Interpreter(agc).dmpAt("SFTEMP1");
        new Interpreter(agc).tpleftn(2);
        return dpinsf();
    }

    /** 2ROUND: round MPAC+2 into MPAC+1 and MPAC+1 into MPAC; sets MPAC+6 on OF/UF. */
    public void tworound() {
        int lo = Agc.add(g("MPAC+1"), g("MPAC+1"));
        s("MPAC+1", lo);
        int hi = Agc.add(g("MPAC"), g("MPAC+1") & 0);
        s("MPAC", hi);
    }

    /** TESTOFUF: alarm and recycle on overflow/underflow recorded in MPAC+6. */
    public void testofuf() {
        if (sgn(g("MPAC+6")) != 0) almcycle();
    }

    /** HMSIN: load hours, minutes, seconds (all three decimal words) into centiseconds DP. */
    public int hmsin() {
        if (!all3dec()) return 0;
        // hours: X words * 10^5/2^14 -> whole hours
        long hours = decimalWord(g("XREG"), g("XREGLP"));
        long minutes = decimalWord(g("YREG"), g("YREGLP"));
        long seconds = decimalWord(g("ZREG"), g("ZREGLP"));
        if (hours > 745 || minutes > 59 || seconds > 5999) { almcycle(); return 0; }
        long cs = hours * 360000L + minutes * 6000L + seconds;
        if (cs > ((1L << 28) - 1)) { almcycle(); return 0; }       // > 745h 39m 14.59s
        s("MPAC", Agc.fromSigned((int) (cs >> 14)));
        s("MPAC+1", Agc.fromSigned((int) (cs & 037777)));
        s("MPAC+2", 0);
        new Interpreter(agc).tpagree();
        s("E:" + g("NOUNADD"), g("MPAC"));
        s("E:" + (g("NOUNADD") + 1), g("MPAC+1"));
        loadlv();
        return g("MPAC");
    }

    private long decimalWord(int hi, int lo) {
        // value = (hi,lo) as DP fraction * 10^5 / 2^14 -> whole number, rounded
        double v = (((double) sgn(hi) * 16384 + sgn(lo)) / 16384.0) * 100000.0 / 16384.0;
        return Math.round(v);
    }

    /** ALL3DEC: all three words must have been loaded in decimal (bits 3,4,5 of DECBRNCH). */
    public boolean all3dec() {
        if ((g("DECBRNCH") & 034) == 034) return true;
        forcev25();
        return false;
    }

    /** FORCEV25: force V25 to be executed on recycle, then alarm and recycle. */
    public void forcev25() {
        s("VERBSAVE", neg(025));
        almcycle();
    }

    // ======================================================================================
    // MONITOR
    // ======================================================================================

    /**
     * MONITOR: repeated display once per second until ended by V34, V33, V32, another
     * monitor, or an NVSUB call that passes DSPLOCK. MONSAVE = N/V code (+ = on), MONSAVE1 =
     * NOUNCADR for MATBS; bit 15 of MONSAVE1 = KILL MONITOR, bit 14 = EXTERNAL MONITOR.
     */
    public void monitor() {
        int temp = g("NOUNCADR") & ~(BIT15 | BIT14);
        monit1(temp);
    }

    /** MONIT1. */
    public void monit1(int mpac1) {
        if (!isInternalEntry()) {                                  // externally initiated
            mpac1 |= BIT14;
            s("MONSAVE2", 0);                                      // zero NVMONOPT options
        }
        int verb = g("VERBREG") & LOW7;
        int nv = ((verb << 5) << 2 | g("NOUNREG")) & Agc.WORD_MASK; // verb code to bits, noun low
        s("MPAC", (verb << 7) | g("NOUNREG"));
        s("DSPLOCK", 0);                                           // monitor can run
        if (sgn(g("CADRSTOR")) == 0) reldsp1();                    // turn off KR light
        if (g("MONSAVE") == 0) {                                   // monitor was off: request
            new Waitlist(agc).waitlist(1, this::monreq);
        }
        s("MONSAVE", g("MPAC"));
        s("MONSAVE1", mpac1);                                      // zeroes the kill monitor bit
        entret.run();
    }

    /** MONREQ: waitlist task, once per second; enters the MONDO job at CHRPRIO. */
    public void monreq() {
        lodsampt();                                                // time snatched for noun 65
        if (sgn(g("MONSAVE1")) < 0 || (g("MONSAVE1") & BIT15) != 0) {   // killer bit
            killmon();
            return;
        }
        new Waitlist(agc).waitlist(MONDEL, this::monreq);
        new Executive(agc).novac(CHRPRIO, this::mondo);
        new Waitlist(agc).taskover();
    }

    /** KILLMON: zero MONSAVE and MONSAVE1 (killer and external monitor bits off). */
    public void killmon() {
        s("MONSAVE", 0);
        s("MONSAVE1", 0);
        new Waitlist(agc).taskover();
    }

    /** MONDO: Executive job each second; redisplay the noun with the display verb. */
    public void mondo() {
        if ((g("MONSAVE1") & BIT15) != 0) { endofjob(); return; }   // terminate came since last MONREQ
        if (sgn(g("DSPLOCK")) != 0) { monbusy(); return; }          // NVSUB is busy
        updatnn(g("MONSAVE") & LOW7);                               // noun into NOUNREG and display
        int verb = ((g("MONSAVE") & MID7) + MONREF) >> 7;           // monitor verb -> display verb
        s("VERBREG", (g("MONSAVE") >> 7) - 10 + 0 * verb);          // verb - 10 decimal
        entret = this::pastevb;                                     // after data display do PASTEVB
        s("MPAC+2", g("MONSAVE1") & ~(BIT15 | BIT14));              // ECADR for INTMCTBS
        testnn();
    }

    /** PASTEVB: paste the monitor verb or NVMONOPT "please" verb back into the verb lights. */
    public void pastevb() {
        int opt = g("MONSAVE2") & MID7;
        int verb = opt != 0 ? opt >> 7 : g("MONSAVE");
        s("EDOP", verb >> 7);                                       // right 7
        updatvb(g("EDOP"));
        s("REQRET", 0);                                             // pasted verbs can be executed
        blanksub(g("MONSAVE2"));                                    // NVMONOPT blank option
        endofjob();
    }

    /** MONBUSY: turn on key release light and end the job. */
    public void monbusy() {
        relDspOn();
        endofjob();
    }

    // ======================================================================================
    // DSPFMEM (V27): fixed memory display
    // ======================================================================================

    /**
     * DSPFMEM: display (octal) any fixed register. For banks 30 and above, preload the
     * superbank bits into the third component of noun 26 with V23N26.
     */
    public void dspfmem() {
        s("DSPCOUNT", R1D1);
        int superbits = g("DSPTEM1+2");                             // 3rd component of noun 26
        int word = new InterBankCommunication(agc).supdacal(superbits, g("NOUNCADR"),
                (cadr, off) -> g("FIXED:" + cadr + ":" + off));
        dspoctwo(word);
        endofjob();
    }

    // ======================================================================================
    // Word display routines
    // ======================================================================================

    /** DSPSIGN: show sign of the DP number in MPAC and make it positive. */
    public void dspsign() {
        int mpac = g("MPAC");
        if (sgn(mpac) > 0 || mpac == 0) {
            plusOn();
        } else {
            s("MPAC", Agc.add(neg(g("MPAC")), 1) & 0 | neg(mpac));    // magnitude
            s("MPAC+1", neg(g("MPAC+1")));
            minusOn();
        }
    }

    /** DSPRND: round by 5 x 10^-6; limit to DPOSMAX on overflow. */
    public void dsprnd() {
        long dp = ((long) sgn(g("MPAC")) << 14) + sgn(g("MPAC+1")) + 02476;
        if (Math.abs(dp) >= (1L << 28)) dp = (1L << 28) - 1;
        s("MPAC", Agc.fromSigned((int) (dp >> 14)));
        s("MPAC+1", Agc.fromSigned((int) (dp & 037777)));
    }

    /** DSPDECWD: MPAC,MPAC+1 -> sign and 5 decimal characters from DSPCOUNT (rounded). */
    public void dspdecwd() {
        dspsign();
        dsprnd();
        dspdcwd1(4);
    }

    /** DSPDCWD1: emit WDCNT+1 characters, each from 10 x fraction. */
    public void dspdcwd1(int wdcnt) {
        for (int n = wdcnt; n >= 0; n--) {
            new Interpreter(agc).shortmp(BINCON);                    // x 10
            int digit = g("MPAC");
            s("CODE", RELTAB[digit] & LOW5);
            s("MPAC", g("MPAC+1"));
            s("MPAC+1", g("MPAC+2"));
            s("MPAC+2", 0);
            int dc = g("DSPCOUNT");
            s("COUNT", dc);
            if (dc != 0) s("DSPCOUNT", dc - 1);
            dspin();
        }
        s("DSPCOUNT", neg(VD1));
    }

    /** DSPDECNR: sign and 5 decimal characters, no rounding. */
    public void dspdecnr() {
        dspsign();
        dspdcwd1(4);
    }

    /** DSPDC2NR: sign and 2 decimal characters, no rounding. */
    public void dspdc2nr() {
        dspsign();
        dspdcwd1(1);
    }

    /** DSP2DEC: DP value as sign and 10 decimal characters starting at R2D1. */
    public void dsp2dec() {
        s("CODE", 0);
        dspin11(3);                                                 // -R2 off
        dspin11(4);                                                 // +R2 off
        dspsign();
        s("DSPCOUNT", R2D1);
        dspdcwd1(9);
    }

    /** DSPDECVN: display A (N x 2^-14) as two decimal characters; scaled by .01 first. */
    public void dspdecvn(int a) {
        int scaled = (int) (((long) sgn(a) * VNDSPCON) >> 14);
        s("MPAC", scaled);
        s("MPAC+1", 0);
        dspdcwd1(1);
    }

    /** GOVNUPDT: part of UPDATVB. */
    public void govnupdt(int a) { dspdecvn(a); }

    /** DSPOCTWO: display A as a 5 character octal word from DSPCOUNT. */
    public void dspoctwo(int a) {
        s("CYL", a);
        s("DSPCOUNT", g("DSPCOUNT") + BIT14);                       // blank the sign
        int word = a;
        for (int wdcnt = 4; wdcnt >= 0; wdcnt--) {
            int digit = (word >> 12) & 7;                           // cycle left 3 each time
            word = ((word << 3) | (word >> 12)) & Agc.WORD_MASK;
            s("CODE", RELTAB[digit] & LOW5);
            int dc = g("DSPCOUNT");
            s("COUNT", dc);
            if ((dc & ~BIT14) != 0) s("DSPCOUNT", dc - 1);
            dspin();
        }
        s("DSPCOUNT", neg(VD1));                                    // block numerics, clears, signs
    }

    /** DSP2BIT: display A as 2 octal characters. */
    public void dsp2bit(int a) {
        int rotated = ((a << 14) | (a >> 1)) & Agc.WORD_MASK;       // pre-cycle right
        s("CYR", a);
        s("WDCNT", 1);
        s("CYL", rotated);
        dspoctwo(rotated);
    }

    /**
     * DSPIN: place 0..25 octal display number into COUNT and the 5 bit relay code into CODE.
     * If bit 14 of COUNT is 1, the sign is blanked with the left character. The relay-word
     * (DSPTAB entry) is DSREL = (COUNT &amp; LOW5) / 2; odd COUNT = left character.
     */
    public void dspin() {
        int count = g("COUNT");
        int dsrel = (count & LOW5) >> 1;
        int code = g("CODE");
        int sel;
        if ((count & BIT1) != 0) {                                  // left char if COUNT is odd
            code = (code << 5) & Agc.WORD_MASK;
            sel = (count & BIT14) != 0 ? 3 : 1;
        } else {
            sel = 0;
        }
        s("CODE", code);
        dspin1(dsrel, sel, code);
    }

    /** 11DSPIN: sign-relay variant; the rel address is in A, 1 or 0 in bit 11 of CODE. */
    public void dspin11(int dsrel) {
        dspin1(dsrel, 2, g("CODE"));
    }

    private void dspin1(int dsrel, int sel, int code) {
        String key = "DSPTAB+" + dsrel;
        int entry = g(key);
        int dsmag = sgn(entry) < 0 ? neg(entry) : entry;            // magnitude
        if ((dsmag & DSMSK[sel]) == code) return;                   // same: nothing to do
        int updated = (dsmag & ~DSMSK[sel] & Agc.WORD_MASK) + code;
        s(key, neg(updated));                                       // stored negative = changed
        if (sgn(entry) > 0 || entry == 0) s("NOUT", g("NOUT") + 1); // old entry was +: bump NOUT
    }

    // ======================================================================================
    // DSPALARM, ALMCYCLE, MMCHANG, REQMM
    // ======================================================================================

    /** PREDSPAL: blank count and alarm (decimal display with octal-only noun). */
    public void predspal() {
        s("DSPCOUNT", neg(VD1));
        dspalarm();
    }

    /**
     * DSPALARM: NVSUB-initiated routines (ENTRET = NVSUBEND) abort with 01501; monitors are
     * killed; keyboard initiated routines turn on the operator error light and end the job.
     */
    public void dspalarm() {
        if (entretIsNvsubEnd()) { charalrm2(); return; }
        if (!entretIsPastevb()) { charalrm(); return; }
        killmonOn();
        falton();
        pastevb();                                                  // put monitor verb back
    }

    /** CHARALRM: no NVSUB initiated; operator error light on and end of job. */
    public void charalrm() {
        falton();
        endofjob();
    }

    private void charalrm2() {
        falton();
        endofjob();
        poodoo(01501);                                              // illegal V/N combination
    }

    private boolean entretIsNvsubEnd() { return g("ENTRET_KIND") == 1; }

    private boolean entretIsPastevb() { return g("ENTRET_KIND") == 2; }

    /**
     * ALMCYCLE: check fail light on, redisplay the original verb, and recycle to execute the
     * original verb/noun again (bad load data, decimal MCTBS, or MM code without two chars).
     */
    public void almcycle() {
        falton();
        s("REQRET", neg(g("VERBSAVE")));
        updatvb(neg(g("VERBSAVE")));
        enter();
    }

    /**
     * MMCHANG (V37): uses the noun display until ENTER, then the mode display. Demands 2
     * numerical characters; otherwise recycles. Goes to V37 (MODROUTB) with the new MM code.
     */
    public void mmchang() {
        reqmm();
        if (BIT5 + g("DSPCOUNT") != 0 && neg(g("DSPCOUNT")) != BIT5) {
            almcycle();                                             // not -ND2
            return;
        }
        int mm = g("NOUNREG");
        s("NOUNREG", 0);
        s("MPAC", mm);
        s("DSPCOUNT", ND1);
        blank2();
        s("DSPCOUNT", neg(VD1));                                    // block numeric chars
        new FreshStartAndRestart(agc).v37(mm);
    }

    /** REQMM: request the MM code via the noun lights. */
    public void reqmm() {
        s("REQRET", neg(1));
        s("DSPCOUNT", ND1);
        s("NOUNREG", 0);
        blank2();
        flashon();
        s("DECBRNCH", 1);                                           // decimal
        entexit();
    }

    // ======================================================================================
    // Request verbs (V30, V31)
    // ======================================================================================

    /** VBRQEXEC (V30): Executive request (NOUN 26: priority, job address, BBCON). */
    public void vbrqexec() {
        boolean findvac = (g("DSPTEM1") & BIT1) != 0;
        int prio = g("DSPTEM1") & ~BIT1;
        reldsp();
        if (findvac) {
            new Executive(agc).findvac(prio, () -> new Executive(agc).startJob(g("DSPTEM1+1"), g("DSPTEM1+2")));
        } else {
            new Executive(agc).novac(prio, () -> new Executive(agc).startJob(g("DSPTEM1+1"), g("DSPTEM1+2")));
        }
        endofjob();
    }

    /** VBRQWAIT (V31): Waitlist request (NOUN 26: delay, task address, BBCON). */
    public void vbrqwait() {
        int delay = g("DSPTEM1");
        reldsp();
        new Waitlist(agc).waitlist(delay, () -> new Executive(agc).startJob(g("DSPTEM1+1"), g("DSPTEM1+2")));
        endofjob();
    }

    // ======================================================================================
    // Proceed / terminate / resequence
    // ======================================================================================

    /** VBPROC (V33): proceed without data. */
    public void vbproc() {
        s("LOADSTAT", 1);
        killmonOn();
        reldsp();
        flashoff();
        recaltst();
    }

    /** VBTERM (V34): terminate (LOADSTAT negative). */
    public void vbterm() {
        s("LOADSTAT", neg(1));
        killmonOn();
        reldsp();
        flashoff();
        recaltst();
    }

    /** PROCKEY: same as VBPROC but callable under Executive control with CHRPRIO. */
    public void prockey() {
        s("REQRET", 0);
        s("DSPCOUNT", neg(VD1));
        vbproc();
    }

    /** VBRESEQ (V32): wake ENDIDLE at the final-ENTER-of-load line (data in). */
    public void vbreseq() {
        s("LOADSTAT", Agc.WORD_MASK);                               // -0: make it look like data in
        killmonOn();
        reldsp();
        flashoff();
        recaltst();
    }

    // ======================================================================================
    // KEY RELEASE
    // ======================================================================================

    /**
     * VBRELDSP: always turns off the UPACT light and clears DSPLOCK. Highest priority
     * function is unsuspending a suspended externally initiated monitor; otherwise RELDSP
     * and, if there is a job in ENDIDLE, PINBRNCH to re-establish the displays.
     */
    public void vbreldsp() {
        s("DSALMOUT", g("DSALMOUT") & ~BIT3);                       // turn off UPACT light
        boolean oldLockSet = sgn(g("21/22REG")) != 0;
        boolean emb = (g("MONSAVE1") & BIT14) != 0;                 // external monitor bit
        if (oldLockSet && emb) { unsuspen(); return; }
        tstlts4();
    }

    /** TSTLTS4: release display; re-establish if CADRSTOR is full. */
    public void tstlts4() {
        reldsp();
        if (sgn(g("CADRSTOR")) != 0) {
            new DisplayInterfaceRoutines(agc).pinbrnch();
        } else {
            endofjob();
        }
    }

    /** UNSUSPEN: external monitor is suspended; just unsuspend by clearing DSPLOCK. */
    public void unsuspen() {
        s("DSPLOCK", 0);
        if (sgn(g("CADRSTOR")) != 0) { endofjob(); return; }
        reldsp1();
        endofjob();
    }

    // ======================================================================================
    // NVSUB: internal calls
    // ======================================================================================

    /**
     * NVSUB: subroutine calls from within the computer. A = 0VVVVVVVNNNNNNN (verb, noun).
     * Negative values blank the display: -4 full, -3 leave mode, -2 leave mode and verb,
     * -1 blank R's only. MPAC+2 may hold a machine CADR for MCTBS nouns.
     *
     * @return true if executed (return to 2+ calling location), false if the display system
     *         is blocked (return to 1+ calling location, nothing displayed)
     */
    public boolean nvsub(int a) {
        return nvmonopt(a, 0);
    }

    /** NVMONOPT: NVSUB with options in L (please-verb bits 8-14, blank code bits 1-3). */
    public boolean nvmonopt(int a, int l) {
        s("NVTEMP", a);
        if ((g("MONSAVE1") & BIT14) != 0 || g("DSPLOCK") != 0) {
            return false;                                           // display system blocked
        }
        s("MONSAVE2", l);                                           // store NVMONOPT options
        killmonOn();
        nvsub1();
        return true;
    }

    /** NVSUB1: fan out on NVTEMP (negative = blank options). */
    public void nvsub1() {
        entret = this::nvsubend;
        int nv = g("NVTEMP");
        int c = sgn(nv);
        if (c < 0) {
            if (c >= -4) { blankdsp(-c - 1 + 0); return; }          // -1..-4
            godspalm();
            return;
        }
        int noun = nv & LOW7;
        int verb = nv >> 7;
        s("MPAC+3", noun);
        s("MPAC+4", verb);
        if (noun == 0) {                                            // display verb only
            updatvb(verb);
            s("REQRET", 0);
            nvsubend();
            return;
        }
        if (verb == 0) {                                            // display noun only
            updatnn(noun);
            nvsubend();
            return;
        }
        s("MPAC+5", g("MPAC+2"));                                   // hold MCTBS CADR
        updatvb(verb);
        updatnn(noun);
        s("LOADSTAT", 0);                                           // waiting for data
        s("CLPASS", 0);
        s("REQRET", 0);                                             // pass 0
        s("MPAC+2", g("MPAC+5"));
        entpas0();
    }

    /** NVSUBEND: return to the caller after the task (restores old bank bits). */
    public void nvsubend() {
        // DXCH NVQTEM; SUPDXCHZ: DTCB with superbit switching back to the caller
        new Executive(agc).supdxchz(g("NVQTEM"), g("NVBNKTEM"));
    }

    /** BLANKDSP: blank the display per option (0..3) held in NVTEMP. */
    public void blankdsp(int option) {
        int first = 7 + option;                                      // DSPTAB entries to blank
        for (int code = first; code >= 0 && code <= 10; code--) {
            int old = g("DSPTAB+" + code);
            s("DSPTAB+" + code, neg(BIT12));
            if (sgn(old) >= 0) s("NOUT", g("NOUT") + 1);
            if (code == 0) break;
        }
        switch (option) {
            case 1: s("VERBREG", 0); break;                           // -3
            case 2: s("NOUNREG", 0); break;                           // -2
            case 3: s("CLPASS", 0); break;                            // -1
            default: break;                                           // -4: never touch MODREG
        }
        s("DSPCOUNT", neg(VD1));
        flashoff();
        s("REQRET", 0);
        nvsubend();
    }

    /** KILMONON: force the kill-monitor bit (bit 15 of MONSAVE1), clearing the external bit. */
    public void killmonOn() {
        s("MONSAVE1", BIT15);
    }

    /** KILMONON entry kept under its original label. */
    public void kilmonon() { killmonOn(); }

    // ======================================================================================
    // ENDIDLE, JAMTERM, JAMPROC
    // ======================================================================================

    /**
     * ENDIDLE: for internal programs awaiting an operator response; puts the current job to
     * sleep. Aborts with 01206 if CADRSTOR or DSPLIST is already in use. RECALTST wakes
     * the job at L+1 (terminate), L+2 (proceed without data) or L+3 (data in / resequence).
     *
     * @param cadr the caller's return CADR (low 10 bits + current FBANK)
     */
    public void endidle(int cadr) {
        iscadr0();
        islist0();
        s("CADRSTOR", (cadr & LOW10) + g("FBANK"));
        new Executive(agc).jobsleep();
    }

    /** ISCADR+0: abort 01206 if CADRSTOR is not +0. */
    public void iscadr0() {
        if (g("CADRSTOR") != 0) dspabort();
    }

    /** ISLIST+0: abort 01206 if DSPLIST is not +0. */
    public void islist0() {
        if (g("DSPLIST") != 0) dspabort();
    }

    /** DSPABORT: TC POODOO with OCT 01206 (second job going to sleep in PINBALL). */
    public void dspabort() { poodoo(01206); }

    /** JAMTERM: allow programs to perform the TERMINATE function (does ENDOFJOB). */
    public void jamterm() {
        s("REQRET", 34);                                            // leave ENTER set for ENTPASS0
        s("DSPCOUNT", neg(VD1));
        vbterm();
    }

    /** JAMPROC: allow programs to perform PROCEED (does ENDOFJOB). */
    public void jamproc() {
        s("REQRET", 33);
        s("DSPCOUNT", neg(VD1));
        vbproc();
    }

    /**
     * BLANKSUB: blanks any combination of R1, R2, R3 (BIT1 = R1, BIT2 = R2, BIT3 = R3).
     * DSPCOUNT is restored afterwards.
     * @return false if the display system is blocked (nothing done)
     */
    public boolean blanksub(int code) {
        s("NVTEMP", code & 7);
        if ((g("MONSAVE1") & BIT14) != 0 || g("DSPLOCK") != 0) return false;
        if (g("NVTEMP") == 0) return true;                          // nothing to blank
        blnksub1();
        return true;
    }

    /** BLNKSUB1: blank R1,R2,R3 as selected and restore DSPCOUNT. */
    public void blnksub1() {
        s("BUF+2", g("DSPCOUNT"));
        int bits = g("NVTEMP");
        if ((bits & BIT1) != 0) { s("DSPCOUNT", R1D1); blank5(); }
        if ((bits & BIT2) != 0) { s("DSPCOUNT", R2D1); blank5(); }
        if ((bits & BIT3) != 0) { s("DSPCOUNT", R3D1); blank5(); }
        s("DSPCOUNT", g("BUF+2"));
    }

    // ======================================================================================
    // DSPMM, RECALTST
    // ======================================================================================

    /**
     * DSPMM: does not display MODREG directly; enters an Executive request (NOVAC, priority
     * CHRPRIO) for DSPMMJB and returns to the caller.
     */
    public void dspmm() {
        new Executive(agc).novac(CHRPRIO, this::dspmmjb);
    }

    /** DSPMMJB: display MODREG in MD1/MD2, or blank the mode lights if MODREG is -0. */
    public void dspmmjb() {
        int saved = g("DSPCOUNT");
        s("DSPMMTEM", saved);
        s("DSPCOUNT", MD1);
        int mm = g("MODREG");
        if (sgn(mm) >= 0) {                                         // + or +0: display MODREG
            dspdecvn(mm);
        } else if (mm == Agc.WORD_MASK) {                           // -0: blank MM
            blank2();
        }
        s("DSPCOUNT", g("DSPMMTEM"));
        endofjob();
    }

    /**
     * RECALTST: entered after a load completes, resequence, terminate or proceed; wakes up
     * the job that did ENDIDLE. Return offsets: L+1 terminate, L+2 proceed, L+3 data in.
     */
    public void recaltst() {
        if (sgn(g("CADRSTOR")) == 0 && g("CADRSTOR") == 0) { endofjob(); return; }   // keyboard initiated
        int cadr = g("CADRSTOR");
        s("CADRSTOR", 0);
        new Executive(agc).jobwake(cadr);
        int stat = sgn(g("LOADSTAT"));
        int offset;
        if (stat > 0) {
            offset = 1;                                              // DOPROC
        } else if (stat == 0 && g("LOADSTAT") == 0) {
            endofjob();                                              // pathological
            return;
        } else if (stat < 0 && g("LOADSTAT") != Agc.WORD_MASK) {
            offset = 0;                                              // DOTERM
        } else {
            offset = 2;                                              // -0: data in or resequence
        }
        recal2(offset);
    }

    private void recal2(int offset) {
        s("LOC", g("LOC") + offset);                                 // job resumes at L+offset+1
        s("MPAC", g("VERBREG"));                                     // verb in MPAC, noun in MPAC+1
        s("MPAC+1", g("NOUNREG"));
        reldsp();
        endofjob();
    }

    // ======================================================================================
    // Service routines in fixed/fixed
    // ======================================================================================

    /** SETNCADR: ECADR in A stored in NOUNCADR, EBANK set, E address into NOUNADD. */
    public void setncadr(int ecadr) {
        s("NOUNCADR", ecadr);
        s("EBANK", ecadr);
        s("NOUNADD", (ecadr & LOW8) + OCT1400);
    }

    /** SETNADD: ECADR from NOUNCADR; set EBANK and NOUNADD. */
    public void setnadd() { setncadr(g("NOUNCADR")); }

    /** SETEBANK: ECADR in A; set EBANK; return the E address. */
    public int setebank(int ecadr) {
        s("EBANK", ecadr);
        return (ecadr & LOW8) + OCT1400;
    }

    /** RIGHT5 / LEFT5 / SLEFT5: five-bit cycle (CYR/CYL) and 5-fold doubling. */
    public static int right5(int a) {
        return ((a >> 5) | (a << 10)) & Agc.WORD_MASK;
    }

    public static int left5(int a) {
        return ((a << 5) | (a >> 10)) & Agc.WORD_MASK;
    }

    public static int sleft5(int a) {
        return (a << 5) & Agc.WORD_MASK;
    }

    /** FALTON: turn on the OPERATOR ERROR light (bit 7 of channel 11). */
    public void falton() { s("DSALMOUT", g("DSALMOUT") | BIT7); }

    /** FALTOF: turn off the OPERATOR ERROR light. */
    public void faltof() { s("DSALMOUT", g("DSALMOUT") & ~BIT7); }

    /** RELDSPON: turn on the KEY RELEASE light (bit 5 of channel 11). */
    public void relDspOn() { s("DSALMOUT", g("DSALMOUT") | BIT5); }

    /** LODSAMPT: snatch TIME2,TIME1 into SAMPTIME (for noun 65). */
    public void lodsampt() {
        s("SAMPTIME", g("TIME2"));
        s("SAMPTIME+1", g("TIME1"));
    }

    /** FLASHON: turn on the V/N flash (bit 6 of channel 11). */
    public void flashon() { s("DSALMOUT", g("DSALMOUT") | BIT6); }

    /** FLASHOFF: turn off the V/N flash. */
    public void flashoff() { s("DSALMOUT", g("DSALMOUT") & ~BIT6); }

    /** ENTEXIT == ENTRET. */
    public void entexit() { entret.run(); }

    // ---- NVSUBUSY / NVSBWAIT / RELDSP -----------------------------------------------------------------

    /**
     * NVSUBUSY: put the calling job to sleep until the operator releases the keyboard and
     * display system; turns on the KEY RELEASE light. Aborts 01206 if a second job tries to
     * sleep in PINBALL.
     *
     * @param wakeFcadr the FCADR (caller location minus 2) to restart at
     */
    public void nvsubusy(int wakeFcadr) {
        nvsubsy1(wakeFcadr);
    }

    /** PRENVBSY: special entrance for fixed-bank routines. */
    public void prenvbsy(int callerCadr) {
        nvsubusy(callerCadr - 02003);
    }

    /** NVSUBSY1: must be in bank 27 or lower to put the caller to sleep with proper superbits. */
    public void nvsubsy1(int wake) {
        iscadr0();
        islist0();
        relDspOn();
        s("DSPLIST", wake);
        new Executive(agc).jobsleep();
    }

    /**
     * NVSBWAIT: for fixed-bank routines. If the system is not busy executes V/N and returns to
     * L+1; if busy puts the calling job to sleep (L-1 into DSPLIST) for later restart.
     */
    public void nvsbwait(int a) {
        s("NVTEMP", a);
        if ((g("MONSAVE1") & BIT14) != 0 || g("DSPLOCK") != 0) {
            prenvbsy(g("Q") + 1);                                    // busy
            return;
        }
        nvsub1();                                                    // free (NVSBCOM)
    }

    /**
     * RELDSP: set DSPLOCK to +0, turn the KEY RELEASE light off, search DSPLIST and wake any
     * job there (used by VBPROC, VBTERM, VBRQEXEC, VBRQWAIT, VBRELDSP, extended verbs...).
     */
    public void reldsp() {
        s("MONSAVE1", g("MONSAVE1") & ~BIT14);                       // external monitor bit off
        if (sgn(g("DSPLIST")) > 0) {
            int cadr = g("DSPLIST");
            s("DSPLIST", 0);
            new Executive(agc).jobwake(cadr);
        }
        reldsp2();
    }

    private void reldsp2() {
        s("DSALMOUT", g("DSALMOUT") & ~BIT5);                        // key release light off
        s("DSPLOCK", 0);
    }

    /** RELDSP1: DSPLOCK to +0 without DSPLIST search; light off only if DSPLIST is empty. */
    public void reldsp1() {
        if (g("DSPLIST") == 0) {
            reldsp2();
        } else {
            s("DSPLOCK", 0);
        }
    }

    // ======================================================================================
    // Lamp test (V35), error light reset
    // ======================================================================================

    /**
     * VBTSTLTS (V35): turn on all display panel lights; after 5 s turn off the caution and
     * status lights.
     */
    public void vbtstlts() {
        s("IMODES33", (g("IMODES33") & ~BIT1) + BIT1);               // IMUMON won't turn out lamps
        s("DSALMOUT", g("DSALMOUT") | TSTCON1);                      // UPACT, TEMP, KEY RLSE, V/N FLASH, OP ERR
        s("DSPTAB+11", TSTCON2);                                     // NO ATT, GIMBAL LOCK, TRACKER, PROG ALM
        s("CHAN13", g("CHAN13") | BIT10);                            // TEST ALARM outbit
        for (int i = 10; i >= 0; i--) {
            s("DSPTAB+" + i, neg(FULLDSP));                          // all 8's
        }
        s("DSPTAB+1", neg(FULLDSP1));                                // three plus signs
        s("DSPTAB+4", neg(FULLDSP1));
        s("DSPTAB+6", neg(FULLDSP1));
        s("NOUT", 11);
        new Waitlist(agc).waitlist(SHOLTS, this::tstlts2);           // 5 seconds
        endofjob();                                                  // DSPLOCK left busy until TSTLTS3
    }

    /** TSTLTS2: waitlist task; enters TSTLTS3 as an Executive job. */
    public void tstlts2() {
        new Executive(agc).novac(CHRPRIO, this::tstlts3);
        new Waitlist(agc).taskover();
    }

    /** TSTLTS3: restore the lamps; lamp test complete. */
    public void tstlts3() {
        s("DSALMOUT", g("DSALMOUT") & ~TSTCON3);                     // UPACT, TEMP, OPERATOR ERROR off
        s("CHAN13", g("CHAN13") & ~BIT10);                           // TEST ALARM off
        int noAtt = (g("CHAN12") & BIT4);                            // NO ATT follows bit 4 of channel 12
        s("DSPTAB+11", noAtt + BIT15);                               // AUTO, HOLD, FREE etc. off
        s("IMODES33", (g("IMODES33") & ~C13_11_1) + 020000);         // all lamps out (PRIO16)
        s("IMODES30", (g("IMODES30") & ~OCT55000) + 015000);         // PRIO15
        s("RADMODES", (g("RADMODES") & ~RFAILS2) + g("RCDUFBIT"));
        dspmm();                                                     // redisplay MODREG
        killmonOn();
        flashoff();
        tstlts4();
    }

    /**
     * ERROR (ERROR LIGHT RESET): turns off UPLINK ACTIVITY, AUTO, HOLD, FREE, OPERATOR ERROR,
     * PROG ALM, TRACKER FAIL; leaves GIMBAL LOCK and NO ATT; zeros the TEST ALARM outbit;
     * sets CAUTION RESET; forces bit 12 of all DSPTAB entries to 1.
     */
    public void error() {
        s("DSPLOCK", g("21/22REG"));                                 // ER leaves DSPLOCK unchanged
        s("DSALMOUT", g("DSALMOUT") | BIT10);                        // CAUTION RESET outbit
        s("DSPTAB+11", (GL_NOATT & g("DSPTAB+11")) + BIT15);
        s("IMODES33", (g("IMODES33") & ~020000) + 020000);           // PRIO16: reset fail bits
        s("IMODES30", (g("IMODES30") & ~BIT10) + BIT10);
        s("RADMODES", (g("RADMODES") & ~RFAILS) + g("RCDUFBIT"));
        s("CHAN13", g("CHAN13") & ~BIT10);                           // TEST ALARM off
        s("DSALMOUT", g("DSALMOUT") & ~ERCON);                       // UPACT + OPERATOR ERROR off
        tstab();
    }

    /** TSTAB: force bit 12 on in all DSPTAB entries (marking them changed). */
    public void tstab() {
        for (int i = 10; i >= 0; i--) {
            int e = g("DSPTAB+" + i);
            int out;
            if (sgn(e) > 0 || e == 0) {
                out = e & NOTBIT12;                                   // ERPLUS: clear bit 12 then complement
                out = neg(out);
            } else {
                out = neg(neg(e) & NOTBIT12) & 0;                     // ERMINUS
                out = neg(e) & NOTBIT12;
                out = e & NOTBIT12;
            }
            s("DSPTAB+" + i, out);
        }
        s("FAILREG", 0);
        s("FAILREG+1", 0);
        s("FAILREG+2", 0);
        s("SFAIL", 0);
        endofjob();
    }

    /** RELDSPON variant under original label name. */
    public void reldspon() { relDspOn(); }

    // The PINTEST label is just an alias for LST2FAN in EXTENDED_VERBS.
}
