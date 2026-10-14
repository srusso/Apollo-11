package apollo.comanche055;

import apollo.agc.Agc;

/**
 * PINBALL_GAME_BUTTONS_AND_LIGHTS.agc (Comanche 055, pages 307-389).
 *
 * KEYBOARD AND DISPLAY PROGRAM (PINBALL). Operates under Executive control and processes
 * information exchanged between the AGC and the operator. Inputs are the keyboard, internal
 * programs (through NVSUB) and the uplink. A key press (KEYRUPT/UPRUPT) schedules CHARIN;
 * the communication language is a VERB/NOUN pair of two-digit decimal numbers.
 *
 * Exits: most paths end with ENDOFJOB (kept as {@link #endOfJob()}); extended verbs go to
 * the extended-verb fan, VERB 37 goes to V37, and internal callers return through NVSUB's
 * ENTRET ({@link #nvsubEnd()}). Alarm exit: operator error light (keyboard) or ABORT 01501
 * (NVSUB initiated); a second job trying to sleep in PINBALL aborts with 01206.
 *
 * Modelling notes: erasables are accessed by name through agc.get/set. AGC return addresses
 * held in erasable (REQRET, ENTRET) are modelled by Runnable fields; REQRET's numeric value
 * keeps its sign meaning (+0 pass 0 / execute, negative = awaiting data). DP/TP arithmetic on
 * MPAC, MPAC+1, MPAC+2 is done with Java longs (see the DP helpers at the end).
 *
 * Input codes: 0=10000, 1..9, VERB=10001, ERROR RESET=10010, KEY RELEASE=11001, +=11010,
 * -=11011, ENTER=11100, CLEAR=11110, NOUN=11111.
 *
 * Display DSPCOUNT numbers (octal): MD1 25, MD2 24, VD1 23, VD2 22, ND1 21, ND2 20,
 * R1D1..R1D5 16..12, R2D1..R2D5 11..5, R3D1..R3D5 4..0. DSPTAB[0..10] hold relay words
 * 1..11 (DSPTAB+11 = C relays).
 */
public final class PinballGameButtonsAndLights {
    // ---- constants (page 378-379 packed tables and misc) ----
    public static final int R1D1 = 14;       // octal 16
    public static final int R2D1 = 9;
    public static final int R3D1 = 4;
    public static final int LOW5 = 037;
    public static final int MID5 = 01740;
    public static final int HI5 = 076000;
    public static final int VD1 = 19;         // octal 23
    public static final int ND1 = 17;         // octal 21
    public static final int MD1 = 21;         // octal 25
    public static final int LOW11 = 03777;
    public static final int LOW8 = 0377;
    public static final int LOW7 = 0177;
    public static final int MID7 = 037600;
    public static final int BINCON = 10;
    public static final int CHRPRIO = 030000;
    public static final int ELRCODE1 = 022;
    public static final int LOWVERB = 28;     // lower verb that avoids noun test
    public static final int LST2CON = 40;     // first LIST2 (extended) verb
    public static final int BLANKCON = 04000;
    public static final int BRNCHCON = 077774;
    public static final int MONDEL = 0144;    // 1 second monitor interval
    public static final int NOTBIT12 = 073777;
    public static final int GL_NOATT = 050;
    public static final int ERCON = 0104;
    public static final int OCT1400 = 01400;
    /** CRITCON: DSPCOUNT values at which numeric input is complete, per INREL 0..4. */
    public static final int[] CRITCON = {18, 16, 10, 5, 0};
    /** INRELTAB: input register selector for each DSPCOUNT 0..19 (-1 = CCSHOLE). */
    public static final int[] INRELTAB = {4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, -1, 1, 1, 0, 0};
    /** SGNTAB: DSPTAB indices for sign relay words of R1..R3 (octal 5, 3, 0). */
    public static final int[] SGNTAB = {5, 3, 0};
    public static final int[] SINBLANK = {14, 5, 4};
    public static final int[] DOUBLK = {13, 9, 3};
    public static final int[] DSMSK = {037, 01740, 02000, 03740};
    public static final int VBSP1LD = 21, VBSP2LD = 22, VBSP3LD = 23;
    public static final int DEC34 = 34, DEC33 = 33, OCT31 = 025;

    private final Agc agc;
    private final PinballNounTables nouns;

    /** ENTRET: where ENTER/execute-completed routines exit (default TC ENDOFJOB). */
    private Runnable entret;
    /** The Runnable forms of ENDINST (TC ENDOFJOB) and NVSUBEND (TC NVSUBEND). */
    private final Runnable endInst = this::endOfJob;
    private final Runnable nvsubEndInst = this::nvsubEnd;
    /** REQRET continuation: where the ENTER of a data load resumes. */
    private Runnable reqretTarget;
    /** DECRET: continuation of PUTCOM. */
    private Runnable monBack;
    /** History of data-request continuations of the current load (for CLEAR back up). */
    private final java.util.ArrayList<Runnable> reqHist = new java.util.ArrayList<>();

    public PinballGameButtonsAndLights(Agc agc) {
        this.agc = agc;
        this.nouns = new PinballNounTables(agc);
        this.entret = endInst;
        this.monBack = this::pastevb;
    }

    // ---------------------------------------------------------------- small helpers

    private int g(String s) { return agc.get(s); }
    private void s(String sym, int v) { agc.set(sym, v); }
    private int sg(String sym) { return Agc.toSigned(agc.get(sym)); }

    /** CCS classification: 0 = positive, 1 = +0, 2 = negative, 3 = -0. */
    private static int ccs(int word) {
        word &= Agc.WORD_MASK;
        if (word == 0) return 1;
        if (word == Agc.WORD_MASK) return 3;
        return (word & Agc.SIGN_BIT) != 0 ? 2 : 0;
    }

    private static int neg(int v) { return Agc.fromSigned(-Agc.toSigned(v)); }

    private void endOfJob() { new Executive(agc).endofjob(); }

    private String regName(int inrel) {
        return new String[] {"VERBREG", "NOUNREG", "XREG", "YREG", "ZREG"}[inrel];
    }

    private String regLp(int inrel) {
        return new String[] {"XREGLP-2", "XREGLP-1", "XREGLP", "YREGLP", "ZREGLP"}[inrel];
    }

    /** Erasable word at E-address {@code addr} (EBANK already folded in); MIXTEMP area = 0x1000+k. */
    private int rd(int addr) {
        return agc.get(memKey(addr));
    }

    private void wr(int addr, int v) {
        agc.set(memKey(addr), v);
    }

    private static final int MIXAD = 0x1000;

    private String memKey(int addr) {
        if (addr >= MIXAD) {
            int k = addr - MIXAD;
            return k == 0 ? "MIXTEMP" : "MIXTEMP+" + k;
        }
        return "E" + Integer.toOctalString(addr);
    }

    // ------------------------------------------------------------ executive section

    /**
     * CHARIN: entered through the Executive (priority CHRPRIO) with the 5 bit key code in MPAC.
     * Blocks the display system (saving old DSPLOCK in 21/22REG for error light reset), turns
     * on the key release light if CADRSTOR is full, and dispatches on the key code.
     */
    public void charin() {
        int old = g("DSPLOCK");
        s("DSPLOCK", 1);
        s("21/22REG", old);
        if (ccs(g("CADRSTOR")) != 1) {                // CADRSTOR full
            if (g("MPAC") != ELRCODE1) {              // all keys except ER turn on KR lite
                relDspOn();
            }
        }
        int key = g("MPAC");
        s("CHAR", key);
        switch (key) {
            case 1: case 2: case 3: case 4: case 5: case 6: case 7: num(); break;
            case 010: case 011: test89(); break;
            case 020: s("CHAR", 0); num(); break;     // NUM -2: key 0
            case 021: verb(); break;
            case 022: error(); break;
            case 031: vbReldsp(); break;
            case 032: posgn(); break;
            case 033: negsgn(); break;
            case 034: enterJmp(); break;
            case 036: clear(); break;
            case 037: noun(); break;
            default: charAlrm(); break;
        }
    }

    private void enterJmp() { enter(); }

    /** 89TEST: 8 and 9 are accepted only for decimal (DECBRNCH nonzero). */
    public void test89() {
        switch (ccs(g("DSPCOUNT"))) {
            case 0: case 1: break;
            default: endOfJob(); return;              // data blocked
        }
        if (ccs(g("DECBRNCH") & 3) == 0) num(); else charAlrm();
    }

    /**
     * NUM assembles octal 3 bits at a time; for decimal converts the incoming digit keeping DP
     * results. Octal results are left in XREG/YREG/ZREG, hi part of decimal in XREG.. and low
     * parts in XREGLP... DECBRNCH is +0 for octal, +1 for +decimal, +2 for -decimal. If DSPCOUNT
     * is left negative no more data is accepted.
     */
    public void num() {
        int c = ccs(g("DSPCOUNT"));
        if (c >= 2) { endOfJob(); return; }
        getInrel();
        if (ccs(g("CLPASS")) <= 1) s("CLPASS", 0);   // if + or +0 make +0
        int ch = g("CHAR");
        s("CODE", T4ruptProgram.RELTAB[ch] & LOW5);
        s("COUNT", g("DSPCOUNT"));
        dspin();
        int inrel = g("INREL");
        int mode = g("DECBRNCH") & 3;
        if (mode == 0) {                              // octal
            int v = g(regName(inrel));
            endNmTst(((v << 3) & Agc.WORD_MASK) + ch);
            return;
        }
        // DECTOBIN: sum kept as hi (reg LP slot) * 2^14 + lo (reg slot), x10 + digit
        long sum = ((long) g(regLp(inrel)) << 14) + g(regName(inrel));
        sum = sum * 10 + ch;
        s(regLp(inrel), (int) (sum >> 14));
        endNmTst((int) (sum & 0x3FFF));
    }

    private void endNmTst(int lo) {
        int inrel = g("INREL");
        s(regName(inrel), lo);
        if (g("DSPCOUNT") == CRITCON[inrel]) {        // ENDNUM
            if ((g("DECBRNCH") & 3) != 0) decEnd(); else endAll();
        } else {
            morNum();
        }
    }

    private void endAll() {                           // block NUMIN by placing DSPCOUNT negatively
        s("DSPCOUNT", neg(g("DSPCOUNT")));
        endOfJob();
    }

    /** DECEND: for R1..R3 convert the integer to fraction (sum/10^5) in XREG (hi) / XREGLP. */
    private void decEnd() {
        int inrel = g("INREL");
        if (inrel <= 1) {                             // VERBREG, NOUNREG: leave whole
            s(regLp(inrel), 0);
            endAll();
            return;
        }
        long sum = ((long) g(regLp(inrel)) << 14) + g(regName(inrel));
        long v = (sum << 28) / 100000L;               // x DECON = 2EXP14/10EXP5
        int hi = (int) ((v >> 14) & 0x3FFF);
        int lo = (int) (v & 0x3FFF);
        if ((g("DECBRNCH") & 3) == 2) {               // - case: complement both
            hi = Agc.fromSigned(-hi);
            lo = Agc.fromSigned(-lo);
        }
        s(regLp(inrel), lo);
        s(regName(inrel), hi);
        endAll();
    }

    private void morNum() {                           // decrement DSPCOUNT
        s("DSPCOUNT", Agc.toSigned(g("DSPCOUNT")) - 1);
        endOfJob();
    }

    /** GETINREL: INREL = register for current DSPCOUNT (0 VERBREG, 1 NOUNREG, 2 X, 3 Y, 4 Z). */
    public void getInrel() {
        int v = INRELTAB[g("DSPCOUNT")];
        if (v < 0) { ccsHole(); return; }
        s("INREL", v);
    }

    private void ccsHole() { new AlarmAndAbort(agc).poodoo(0); }

    /** VERB key: clear verb, blank VD1/VD2 and start accepting the verb code. */
    public void verb() {
        s("VERBREG", 0);
        nvcom(VD1);
    }

    /** NOUN key. */
    public void noun() {
        s("NOUNREG", 0);
        nvcom(ND1);
    }

    private void nvcom(int dspcount) {
        s("DSPCOUNT", dspcount);
        blank2();
        s("DECBRNCH", 1);                             // set for decimal V/N code
        s("REQRET", 0);                               // set for ENTPAS0
        reqretTarget = null;
        entret = endInst;                             // DSPALARM before first ENTPAS0 must end job
        endOfJob();
    }

    /** NEGSGN: '-' key. */
    public void negsgn() {
        if (!signTest()) return;
        minusOn();
        bothSgn(2);
    }

    /** POSGN: '+' key. */
    public void posgn() {
        if (!signTest()) return;
        plusOn();
        bothSgn(1);
    }

    private void bothSgn(int a) {
        int inrel = g("INREL");
        // set DEC COMP bit (bit 5 for R1, bit 4 for R2, bit 3 for R3) and sign code in DECBRNCH
        s("DECBRNCH", g("DECBRNCH") + a + (1 << (6 - inrel)));
        fixClPass();
    }

    private void fixClPass() {
        if (ccs(g("CLPASS")) <= 1) {                  // if + or +0 make +0
            s("CLPASS", 0);
            return;
        }
        endOfJob();
    }

    /** +ON: turn on + sign of the register being loaded. */
    public void plusOn() {
        getInrel();
        int off = SGNTAB[g("INREL") - 2];
        s("SGNOFF", off);
        s("SGNON", off + 1);
        sgnCom();
    }

    /** -ON: turn on - sign of the register being loaded. */
    public void minusOn() {
        getInrel();
        int on = SGNTAB[g("INREL") - 2];
        s("SGNON", on);
        s("SGNOFF", on + 1);
        sgnCom();
    }

    private void sgnCom() {
        s("CODE", 0);
        dspin11(g("SGNOFF"));
        s("CODE", 02000);                             // BIT11
        dspin11(g("SGNON"));
    }

    /**
     * SIGNTEST: allows +,- only when DSPCOUNT = R1D1, R2D1 or R3D1 and only for the first of
     * consecutive sign characters. Returns true if the sign is legal (else ends the job).
     */
    private boolean signTest() {
        if ((g("DECBRNCH") & 3) != 0) { endOfJob(); return false; }
        int d = g("DSPCOUNT");
        if (d == R1D1 || d == R2D1 || d == R3D1) return true;
        endOfJob();
        return false;
    }

    public void clear() {
        int idx = Math.abs(sg("DSPCOUNT"));
        int inrel = INRELTAB[Math.min(idx, 19)];
        s("INREL", inrel);
        if (ccs(g("CLPASS")) == 0) {                  // CLPASHI
            int ir = Math.max(inrel - 1, 0);          // CCS INREL ; TS INREL
            s("INREL", ir);
            if (!legalTest(ir)) return;
            s("REQRET", Agc.add(g("REQRET"), 3));     // +3 to - number backs data requests
            if (reqHist.size() >= 2) {                // back up to the previous request
                reqHist.remove(reqHist.size() - 1);
                reqretTarget = reqHist.get(reqHist.size() - 1);
            }
            int saved = g("INREL");
            s("VERBREG", g("VERBREG") - 1);           // decrement verb and re-display
            updatvb(g("VERBREG"));
            s("INREL", saved);
        } else if (!legalTest(inrel)) {
            return;
        }
        clr5();
        if (ccs(g("CLPASS")) <= 1) s("CLPASS", g("CLPASS") + 1);   // only if + or +0
        endOfJob();                                   // set for higher pass
    }

    private void clr5() { blank5(); }

    /** LEGALTST: legal if INREL >= 2; otherwise ENDOFJOB. */
    private boolean legalTest(int inrel) {
        if (inrel >= 2) return true;
        endOfJob();
        return false;
    }

    /**
     * 5BLANK blanks the 5 character display word in R1, R2 or R3 and zeroes XREG, YREG or ZREG.
     * Leaves DSPCOUNT at the left-most display number of the register just blanked.
     */
    public void blank5() {
        getInrel();
        int inrel = g("INREL");
        s(regName(inrel), 0);
        s(regLp(inrel), 0);
        s("CODE", 0);
        s("DECBRNCH", g("DECBRNCH") & ~(1 << (6 - inrel)) & BRNCHCON);   // zero dec comp bit + low 2 bits
        s("COUNT", SINBLANK[inrel - 2]);              // isolated char blanked separately
        dspin();
        s("DSPCOUNT", DOUBLK[inrel - 2]);
        blank2();
        s("DSPCOUNT", Agc.add(g("DSPCOUNT"), neg(2)));
        blank2();
        s("DSPCOUNT", new int[] {R1D1, R2D1, R3D1}[inrel - 2]);
    }

    /** 5BLANK-1 entry (BLANKSUB): DSPCOUNT given in A. */
    public void blank5(int dspcount) {
        s("DSPCOUNT", dspcount);
        blank5();
    }

    /** 2BLANK blanks two characters; DSPCOUNT holds the display number of the left one. */
    public void blank2() {
        int idx = g("DSPCOUNT");
        int old = g("DSPTAB" + idx);
        s("DSPTAB" + idx, neg(BLANKCON));             // CS BLANKCON : changed marker
        if (ccs(old) == 0) {                          // old contents +  -> one more word to output
            s("NOUT", g("NOUT") + 1);
        }
    }

    // ---------------------------------------------------------------- ENTER

    /**
     * ENTER pass 0 is the execute function; higher passes load data. The sign of REQRET
     * determines the pass (+ pass 0, - higher). MCTBS nouns want an ECADR loaded.
     */
    public void enter() {
        s("CLPASS", 0);
        entret = endInst;
        if (ccs(g("REQRET")) <= 1) {                  // + or +0 : pass 0
            entpas0();
            return;
        }
        // ENTPASHI
        if (reqretTarget == mmchangRef) {             // MM code requires 2 chars
            acceptWd();
            return;
        }
        if ((g("DECBRNCH") & 3) != 0) {               // decimal: 5 chars needed
            if (ccs(g("DSPCOUNT")) <= 1) { godspalm(); return; }
        }
        acceptWd();
    }

    private final Runnable mmchangRef = this::mmchang1;

    private void acceptWd() {
        s("REQRET", neg(g("REQRET")));                // set REQRET +
        flashOff();
        Runnable r = reqretTarget;
        if (r != null) r.run();
    }

    /**
     * ENTPAS0: NOUN VERB SUB enters here. Clears DECBRNCH, blocks further numeric
     * characters, saves the verb for recycle and goes to the verb fan (noun test for verbs
     * below LOWVERB).
     */
    public void entpas0() {
        reqHist.clear();
        s("DECBRNCH", 0);
        s("DSPCOUNT", neg(VD1));
        testVb();
    }

    private void testVb() {
        int verb = g("VERBREG");
        s("VERBSAVE", neg(verb));
        if (verb >= LOWVERB) { verbFan(); return; }
        testNn();
    }

    /** TESTNN: read noun tables; handle normal / mixed / machine CADR to be specified nouns. */
    public void testNn() {
        nouns.lodnntab();
        if (g("MIXBR") == 2) {                        // mixed
            mixNoun();
            return;
        }
        switch (ccs(g("NNADTEM"))) {                  // normal
            case 0: setNcadr(g("NNADTEM")); verbFan(); return;   // + : ECADR in table
            case 1: godspalm(); return;               // +0 : not in use
            case 2: reqAdd(); return;                 // - : machine CADR to be specified
            default:                                  // -0 : augment machine CADR
                s("NOUNCADR", g("NOUNCADR") + 1);
                setNadd();
                intMctbs2();
        }
    }

    private void reqAdd() {
        s("CLPASS", 040000);                          // pass 0 only
        if (entret != endInst) {                      // internal machine CADR to be specified
            intMctbs();
            return;
        }
        reqDatZ(this::reqAddCont);                    // external MCTBS: ask for CADR in R3
    }

    private void reqAddCont() {
        if (ccs(g("DECBRNCH")) != 1) {                // decimal used for MCTBS: alarm + recycle
            almCycle();
            return;
        }
        s("DSPCOUNT", neg(VD1));                      // octal ok; block numeric char
        if (ccs(g("CADRSTOR")) == 1) { /* EXTERNAL MCTBS leaves flash on if ENDIDLE != +0 */ } else {
            flashOn();
        }
        setNcadr(g("ZREG"));
        nouns.lodnntab();
        verbFan();
    }

    /** INTMCTBS: internal machine CADR to be specified (NVSUB left CADR in MPAC+2). */
    private void intMctbs() {
        setNcadr(g("MPAC+2"));
        intMctbs2();
    }

    private void intMctbs2() {
        if (g("VERBREG") != 5) {                      // display CADR unless VB = 05
            s("DSPCOUNT", R3D1);
            dspOctWo(g("NOUNCADR"));
        }
        verbFan();
    }

    /** VERBFAN: extended verbs (>= 40) go to GOEXTVB, others through VERBTAB. */
    public void verbFan() {
        int verb = g("VERBREG");
        if (verb >= LST2CON) {
            s("MPAC", verb - LST2CON);
            relDsp();
            new ExtendedVerbs(agc).goextvb();         // VERB-40 in MPAC
            return;
        }
        switch (verb) {
            case 1: dspa(); break;                    // DISPLAY OCT COMP 1 (R1)
            case 2: dspb(); break;                    // COMP 2
            case 3: dspc(); break;                    // COMP 3
            case 4: dspab(); break;                   // COMP 1,2
            case 5: dspabc(); break;                  // COMP 1,2,3
            case 6: decdsp(); break;                  // DECIMAL DISPLAY
            case 7: dspdpdec(); break;                // DP DECIMAL DISPLAY (R1,R2)
            case 11: case 12: case 13: case 14: case 15: case 16: case 17:
                monitor(); break;                     // MONITOR VERBS
            case 21: aload(); break;                  // LOAD COMP 1
            case 22: bload(); break;
            case 23: cload(); break;
            case 24: abload(); break;
            case 25: abcload(); break;
            case 27: dspfmem(); break;                // FIXED MEMORY DISPLAY
            case 30: vbrqexec(); break;               // REQUEST EXECUTIVE
            case 31: vbrqwait(); break;               // REQUEST WAITLIST
            case 32: vbreseq(); break;                // RESEQUENCE
            case 33: vbproc(); break;                 // PROCEED WITHOUT DATA
            case 34: vbterm(); break;                 // TERMINATE
            case 35: vbtstlts(); break;               // TEST LIGHTS
            case 36: new FreshStartAndRestart(agc).slap1(); break;   // FRESH START
            case 37: mmchang(); break;                // CHANGE MAJOR MODE
            case 10: dspAlarm(); break;               // VB10 spare
            default: godspalm(); break;               // VB00, 08, 09, 18-20, 26, 28, 29, 38, 39 illegal
        }
    }

    // ------------------------------------------------------------ mixed noun support

    /**
     * MIXNOUN gets the data of a mixed noun and stores it in MIXTEMP, +1, +2, setting NOUNADD
     * for MIXTEMP. NNADTAB holds IDADDREL (low 10 bits) and the component code (high 5 bits).
     */
    public void mixNoun() {
        int c = ccs(g("NNADTEM"));
        if (c == 1) { godspalm(); return; }           // +0 not in use
        if (g("VERBREG") > 6) { verbFan(); return; }  // avoid MIXNOUN swap if VB is not a display
        int k = 2;
        while (true) {
            s("DECOUNT", k);
            s("NOUNADD", MIXAD);
            s("NOUNTEM", g(k == 0 ? "IDAD1TEM" : k == 1 ? "IDAD2TEM" : "IDAD3TEM"));
            int rut = sfRutMix();                     // SF routine number
            if (dpTest(rut)) s("NOUNTEM", g("NOUNTEM") + 1);   // DP: minor part only
            int eadr = setEbank(g("NOUNTEM") & LOW11);
            wr(MIXAD + k, rd(eadr));
            if (k == 0) break;
            k--;
        }
        verbFan();
    }

    /** DPTEST: true if the scale factor routine number (in A) is a DP type. */
    public boolean dpTest(int sfRoutine) {
        switch (sfRoutine) {
            case 4: case 5: case 7: case 10: case 13: return true;   // DP1, DP2, DP3, DP4, DPFRAC
            default: return false;
        }
    }

    private void reqDatX(Runnable cont) { reqCom(R1D1, cont); }
    private void reqDatY(Runnable cont) { reqCom(R2D1, cont); }
    private void reqDatZ(Runnable cont) { reqCom(R3D1, cont); }

    /**
     * REQDATX/Y/Z: blank the register, flash, and wait for ENTER; {@code cont} is the code
     * which follows the TC REQDATx in the original (resumed by ENTER pass hi through REQRET).
     */
    private void reqCom(int dspcount, Runnable cont) {
        s("DSPCOUNT", dspcount);
        reqretTarget = cont;
        reqHist.add(cont);
        s("REQRET", Agc.fromSigned(-1));              // CS Q : negative return address
        blank5();
        flashOn();
        entret.run();                                 // ENDRQDAT: TC ENTEXIT
    }

    public void updatnn(int noun) {
        s("NOUNREG", noun);
        nouns.lodnntab();
        if (ccs(g("NNADTEM")) == 0) setNcadr(g("NNADTEM"));   // normal: ECADR into NOUNCADR/NOUNADD
        s("DSPCOUNT", ND1);
        dspDecVn(noun);
    }

    /** UPDATVB: VERBREG in A. Displays the verb. */
    public void updatvb(int verb) {
        s("VERBREG", verb);
        s("DSPCOUNT", VD1);
        dspDecVn(verb);
    }

    private void godspalm() { dspAlarm(); }

    // ------------------------------------------------------------- display verbs

    private void dspabc() {
        if (!compTest(-2)) return;
        s("BUF+2", neg(rd(g("NOUNADD") + 2)));
        dspab();
    }

    private void dspab() {
        if (!compTest(-1)) return;
        s("BUF+1", neg(rd(g("NOUNADD") + 1)));
        dspa();
    }



    private void dspa() {
        if (!decTest()) return;
        dspa1();
    }

    private void dspa1() {
        if (tstForDp()) return;                       // channel nouns were displayed already
        s("BUF", neg(rd(g("NOUNADD"))));
        dspCom2();
    }

    private void dspb() {
        if (!dCompTst(-1)) return;
        s("BUF", neg(rd(g("NOUNADD") + 1)));
        dspCom2();
    }

    private void dspc() {
        if (!dCompTst(-2)) return;
        s("BUF", neg(rd(g("NOUNADD") + 2)));
        dspCom2();
    }

    private void dspCom2() {
        int verb = g("VERBREG");
        int top = verb >= 4 ? verb - 3 : 0;           // A B C show in R1; AB in R1,R2; ABC in R1..R3
        for (int i = top; i >= 0; i--) {
            dspCom3(i);
        }
        entret.run();
    }

    private void dspCom3(int i) {
        int[] rs = {R1D1, R2D1, R3D1};
        s("DISTEM", i);
        s("DSPCOUNT", rs[i]);
        dspOctWo(neg(g(i == 0 ? "BUF" : "BUF+" + i)));
    }

    /** COMPTEST alarms if the verb component number exceeds the noun's highest component. */
    private boolean compTest(int negVerbComp) {
        s("SFTEMP1", negVerbComp);
        int noun = (getComp() >> 10) & 3;
        if (noun - (-negVerbComp) < 0) { godspalm(); return false; }
        return true;
    }

    /** DCOMPTST alarms if decimal only bit set; otherwise does COMPTEST. */
    private boolean dCompTst(int negVerbComp) {
        s("SFTEMP1", negVerbComp);
        if (!decTest()) return false;
        return compTest(negVerbComp);
    }

    /** DECTEST: alarms if the decimal-only bit (bit 4 of component code number) is set. */
    private boolean decTest() {
        if ((getComp() & 020000) != 0) { godspalm(); return false; }
        return true;
    }

    /** DCTSTCYC: like DECTEST but alarms and recycles (used by load verbs). */
    private boolean dcTstCyc() {
        if ((getComp() & 020000) != 0) { almCycle(); return false; }
        return true;
    }

    private boolean nounTest() {
        if ((getComp() & 040000) != 0) { godspalm(); return false; }
        return true;
    }

    private boolean tstForDp() {
        if (g("NNADTEM") == Agc.WORD_MASK - 1) {      // -1: channel to be specified
            chanDsp();
            return true;
        }
        if (g("MIXBR") != 2 && dpTest(sfRutNor())) {
            s("NOUNADD", g("NOUNADD") + 1);           // DP: E+1 for the minor part
        }
        return false;
    }

    private void chanDsp() {
        int ch = g("NOUNCADR") & 0x1FF;
        s("BUF", agc.get("CHAN" + Integer.toOctalString(ch)) ^ Agc.WORD_MASK);
        dspCom2();
    }

    /** GETCOMP: component code number (HI5 of NNTYPTEM for normal, NNADTEM for mixed). */
    private int getComp() {
        int w = g(g("MIXBR") == 2 ? "NNADTEM" : "NNTYPTEM");
        return w & HI5;
    }

    /**
     * DECDSP: decimal display. Picks up the data of up to three components and converts each
     * with the scale factor constant/routine given by the noun tables, then DSPDECWD.
     */
    public void decdsp() {
        int comp = (getComp() >> 10) & 3;
        s("DECOUNT", comp);
        for (int i = comp; i >= 0; i--) {             // DSPDCGET
            s(i == 0 ? "XREG" : i == 1 ? "YREG" : "ZREG", rd(g("NOUNADD") + i));
        }
        for (int i = comp; i >= 0; i--) {             // DSPDCPUT
            s("DECOUNT", i);
            s("MPAC+1", 0);
            s("MPAC+2", 0);
            s("DSPCOUNT", new int[] {R1D1, R2D1, R3D1}[i]);
            s("MPAC", g(i == 0 ? "XREG" : i == 1 ? "YREG" : "ZREG"));
            int twiceSf = sfConum() * 2;
            s("SFTEMP1", twiceSf);
            nouns.gtsfout(twiceSf);                   // loads SFTEMP1, SFTEMP2
            int rut = g("MIXBR") == 2 ? sfRutMix() : sfRutNor();
            sfOut(rut);
            if (i == 0) break;
        }
        entret.run();
    }

    /** DECDSP3: SFOUTABR dispatch of display scale factor routines. */
    private void sfOut(int rut) {
        switch (rut) {
            case 0: preDspAl(); break;                // alarm if decimal display of octal-only noun
            case 1: dspDcEnd(); break;                // straight fractional
            case 2: degOutSf(); break;
            case 3: artOutSf(); break;
            case 4: dp1OutSf(); break;
            case 5: dp2OutSf(); break;
            case 6: opDegOut(); break;
            case 7: dp3OutSf(); break;
            case 8: hmsOut(); break;
            case 9: msOut(); break;
            case 10: dp2OutSf(); break;
            case 11: arOut1Sf(); break;
            case 12: twoIntOut(); break;
            case 13: dpFracOt(); break;
            default: godspalm(); break;
        }
    }

    /** DSPDCEND: all SFOUT routines end here; displays MPAC,+1 by DSPDECWD. */
    private void dspDcEnd() {
        dspDecWd();
    }

    private void preDspAl() {
        s("DSPCOUNT", neg(VD1));
        dspAlarm();
    }

    // --------------------------------------------------------- SF routine selection

    /** SFRUTNOR: SF routine number for a normal noun (MID5 of NNTYPTEM). */
    private int sfRutNor() {
        return (g("NNTYPTEM") & MID5) >> 5;
    }

    /** SFRUTMIX: SF routine number for component DECOUNT of a mixed noun (RUTMXTEM). */
    private int sfRutMix() {
        int k = g("DECOUNT");
        int w = g("RUTMXTEM");
        switch (k) {
            case 0: return w & LOW5;
            case 1: return (w & MID5) >> 5;
            default: return (w & HI5) >> 10;
        }
    }

    /** SFCONUM: SF constant number (low 5 bits for normal; by component for mixed). */
    private int sfConum() {
        int w = g("NNTYPTEM");
        if (g("MIXBR") != 2) return w & LOW5;
        switch (g("DECOUNT")) {
            case 0: return w & LOW5;
            case 1: return (w & MID5) >> 5;
            default: return (w & HI5) >> 10;
        }
    }

    // ------------------------------------------------------- scale factor out routines

    public static final int[] DEGTAB = {05605, 03656, 016314, 031463};
    public static final int OPT_BIAS_20 = 016040;
    public static final int SECON2_HI = 01727, SECON2_LO = 01217;
    public static final int MINCON2_HI = 011, MINCON2_LO = 032445;
    public static final int MINCON1_HI = 02104, MINCON1_LO = 010422;
    public static final int HISECON = 023147, HIMINCON = 023346;
    public static final int M_S_CON1 = 077753, M_S_CON2 = 041126;
    public static final int M_S_CON3_HI = 025, M_S_CON3_LO = 037016;
    public static final int RNDCON_LO = 050;          // .5 sec in centiseconds
    public static final long HRCON1 = Math.round(0.16384 * (1L << 28));

    public void degOutSf() {
        s("MPAC+2", 0);                               // index for full scale
        degCom(fixRange());
    }

    public void opDegOut() {
        int m = g("MPAC");
        int sum = Agc.toSigned(ccs(m) == 3 ? 0 : m) + OPT_BIAS_20;
        // positive: add the 20 degree bias and correct for possible overflow into bit 15
        int v = sum > 037777 ? (sum & 037777) | 040000 : Agc.add(m, OPT_BIAS_20);
        s("MPAC", v);
        s("MPAC+2", 2);                               // multiplier .45
        degCom(fixRange());
    }

    /** FIXRANGE: if MPAC is negative mask out the sign bit and return true (augmenter needed). */
    private boolean fixRange() {
        int m = g("MPAC");
        if (ccs(m) <= 1) return false;
        s("MPAC", m & ~040000 & Agc.WORD_MASK);
        return true;
    }

    private void degCom(boolean augment) {
        int idx = g("MPAC+2") * 2;
        long mult = ((long) DEGTAB[idx] << 14) | DEGTAB[idx + 1];
        s("MPAC+1", 0);
        shortMp(g("MPAC"), mult);                     // adjusted angle x multiplier
        long aug = augment ? mult : (((long) g("SFTEMP1") << 14) | g("SFTEMP2"));
        setTp(getTp() + (aug << 14));                 // DAS MPAC : add augmenter
        dspDcEnd();
    }

    /** ARTOUTSF: arithmetic SF; assumes point at left of DP SF constant. */
    public void artOutSf() {
        long sf = ((long) g("SFTEMP1") << 14) | g("SFTEMP2");
        shortMp(g("MPAC"), sf);
        dspDcEnd();
    }

    /** AROUT1SF: point between hi and lo of SF constant; results shifted left 14. */
    public void arOut1Sf() {
        artOutSf();
        l14Out();
    }

    /** DP1OUTSF: scale MPAC,+1 by DP SF in SFTEMP1,2 then scale result by B14. */
    public void dp1OutSf() {
        dpOut();
        l14Out();
    }

    private void l14Out() {
        int t2 = g("MPAC+2"), t1 = g("MPAC+1");
        s("MPAC", t1);
        s("MPAC+1", t2);
        s("MPAC+2", 0);
        dspDcEnd();
    }

    /** DP2OUTSF: scale MPAC, MPAC+1 by DP SF. */
    public void dp2OutSf() {
        dpOut();
        dspDcEnd();
    }

    /** DP3OUTSF: as DP2OUTSF with result shifted left 7 (round MPAC+2 into MPAC+1). */
    public void dp3OutSf() {
        dpOut();
        tpLeftN(7);
        dspDcEnd();
    }

    /** DPOUT: fresh data for hi and lo (READLO), make DP agree, multiply by SFTEMP1,2. */
    private void dpOut() {
        readLo();
        tpAgree();
        dmp(((long) g("SFTEMP1") << 14) | g("SFTEMP2"));
    }

    public void twoIntOut() {
        int[] base = {R1D1, R2D1, R3D1};
        blank5();                                     // blank RXD3
        plusOn();                                     // turn on + sign
        dspDecVn(g("MPAC"));                          // display 1st integer (like verb and noun)
        s("DSPCOUNT", base[g("DECOUNT")] - 3);        // RXD4
        readLo();                                     // get 2nd integer
        dspDecVn(g("MPAC+1"));
        dspDcEnd2();
    }

    private void dspDcEnd2() {                        // DSPDCEND +2 : continue with next component
    }

    /** DPFRACOT: DP fraction to MPAC,+1. */
    public void dpFracOt() {
        readLo();
        dspDcEnd();
    }

    /**
     * READLO picks up fresh data for both hi and lo (needed for time display) into MPAC, +1;
     * zeroes MPAC+2 without forcing TPAGREE.
     */
    public void readLo() {
        int addr;
        if (g("MIXBR") != 2) {
            addr = g("NOUNADD");
        } else {
            int k = g("DECOUNT");
            addr = setEbank(g(k == 0 ? "IDAD1TEM" : k == 1 ? "IDAD2TEM" : "IDAD3TEM") & LOW11);
        }
        s("MPAC", rd(addr));
        s("MPAC+1", rd(addr + 1));
        s("MPAC+2", 0);
    }

    // ----------------------------------------------------------- time displays

    public void hmsOut() {
        readLo();
        tpAgree();
        sepSecNr();                                   // fract sec/60 in MPAC,+1; whole min saved
        dmp(((long) SECON2_HI << 14) | SECON2_LO);    // x .06 : centi-sec/10EXP5 mod 60
        s("DSPCOUNT", R3D1);
        dspDecWd();                                   // display seconds mod 60
        sepMin();                                     // fract min/60 in MPAC+1, hours in MPAC
        s("HITEMOUT", g("MPAC"));                     // save whole hours
        shortMp(g("MPAC+1"), ((long) MINCON2_HI << 14) | MINCON2_LO);   // x .0006
        s("DSPCOUNT", R2D1);
        dspDecWd();                                   // display minutes mod 60
        shortMp(g("HITEMOUT"), HRCON1);               // whole hours x .16384
        s("DSPCOUNT", R1D1);
        dspDecWd();
        entret.run();
    }

    public void msOut() {
        readLo();
        tpAgree();
        long dp = getDp();
        long limit = 359950;                          // 59M 59.5S in centiseconds
        if (Math.abs(dp) >= 359850) {                 // G/ 59M58.5S : force to +/- 59M59.5S
            setDp(dp < 0 ? -limit : limit);
            sepSecNr();                               // M/SNORM+1: no rounding
        } else {
            sepSec();                                 // M/SNORM : round by .5 sec
        }
        msNorm();
    }

    private void msNorm() {
        int[] base = {R1D1, R2D1, R3D1};
        int decount = g("DECOUNT");
        shortMp(HISECON);                             // x .6 + 2EXP-14 : sec/100 mod 60
        s("DSPCOUNT", base[decount] - 3);
        dspDc2Nr();                                   // seconds in D4D5
        s("CODE", 0);
        s("COUNT", base[decount] - 2);
        dspin();                                      // blank middle char
        sepMin();
        setDp((long) g("MPAC+1") * HIMINCON);         // fract min/60 x (.6 + 2EXP-7) : min/100 mod 60
        s("MPAC+2", 0);
        s("DSPCOUNT", base[decount]);
        dspDc2Nr();                                   // minutes in D1D2
        dspDcEnd2();
    }

    private void sepSec() {
        long dp = getDp();
        setDp(dp >= 0 ? dp + RNDCON_LO : dp - RNDCON_LO);   // round by +/- .5 sec (50 cs)
        sepSecNr();
    }

    private void sepSecNr() {
        long dp = getDp();
        long cs = Math.abs(dp);
        long min = cs / 6000;                         // whole minutes (x 2EXP12/6000 in the original)
        long frac = ((cs % 6000) << 28) / 6000;       // fract sec/60 as DP fraction
        s("HITEMOUT", (int) ((min >> 14) & 0x3FFF));
        s("LOTEMOUT", (int) (min & 0x3FFF));
        setDp(dp < 0 ? -frac : frac);
        s("MPAC+2", 0);
    }

    private void sepMin() {
        long min = ((long) g("HITEMOUT") << 14) | g("LOTEMOUT");
        s("MPAC", (int) (min / 60));                  // whole hours
        s("MPAC+1", (int) (((min % 60) << 14) / 60)); // fract min/60 (single precision)
        s("MPAC+2", 0);
    }

    /**
     * DSPDPDEC: display a DP word as 10 decimal digits in R1 and R2 (sign in R1). Not for mixed
     * nouns.
     */
    public void dspdpdec() {
        if (g("MIXBR") == 2) { dspAlarm(); return; }
        s("MPAC", rd(g("NOUNADD")));
        s("MPAC+1", rd(g("NOUNADD") + 1));
        s("DSPCOUNT", R1D1);
        s("MPAC+2", 0);
        tpAgree();
        dsp2Dec();
        entret.run();
    }

    // -------------------------------------------------------------- load verbs

    /**
     * LOAD VERBS: if an alarm condition is detected during EXECUTE the check fail light is lit
     * and the job ends. If detected during ENTER of data, check fail is lit and it recycles to
     * the EXECUTE of the original load verb (ALMCYCLE). Recycle causes: decimal MCTBS, mixture
     * of octal/decimal, octal into decimal-only noun, decimal into octal-only noun, data too
     * large for scale, fewer than 3 words for HMS.
     */
    public void abcload() {
        if (!compTest(-2) || !nounTest()) return;
        updatvb(VBSP1LD);
        reqDatX(() -> {
            updatvb(VBSP2LD);
            reqDatY(() -> {
                updatvb(VBSP3LD);
                reqDatZ(this::putXyz);
            });
        });
    }

    /** PUTXYZ: all three words loaded; check all DEC or all OCT, then store them. */
    private void putXyz() {
        if (!allDcOc(-6)) return;
        nouns.lodnntab();
        for (int k = 0; k < 3; k++) {
            if (!putCom(k)) return;
            wr(g("NOUNADD") + k, putVal);
        }
        if (g("NOUNREG") != 7) { loadLv(); return; }
        // NOUN 7: ZREG zero resets, nonzero sets bits YREG in the flagword at ECADR XREG
        setNcadr(g("XREG"));
        int addr = g("NOUNADD");
        if (g("ZREG") == 0) {
            wr(addr, rd(addr) & ~g("YREG") & Agc.WORD_MASK);       // reset bits
        } else {
            wr(addr, rd(addr) | g("YREG"));                       // set bits
        }
        loadLv();
    }

    /** ABLOAD: load components 1 and 2. */
    public void abload() {
        if (!compTest(-1) || !nounTest()) return;
        updatvb(VBSP1LD);
        reqDatX(() -> {
            updatvb(VBSP2LD);
            reqDatY(this::putXy);
        });
    }

    private void putXy() {
        if (!allDcOc(-5)) return;
        nouns.lodnntab();
        for (int k = 0; k < 2; k++) {
            if (!putCom(k)) return;
            wr(g("NOUNADD") + k, putVal);
        }
        loadLv();
    }

    /** ALOAD: load component 1. */
    public void aload() {
        reqDatX(() -> {
            nouns.lodnntab();
            if (!putCom(0)) return;
            wr(g("NOUNADD"), putVal);
            loadLv();
        });
    }

    /** BLOAD: load component 2 (pass 0 only). */
    public void bload() {
        if (!compTest(-1)) return;
        s("CLPASS", 040000);
        reqDatY(() -> {
            nouns.lodnntab();
            if (!putCom(1)) return;
            wr(g("NOUNADD") + 1, putVal);
            loadLv();
        });
    }

    /** CLOAD: load component 3 (pass 0 only). */
    public void cload() {
        if (!compTest(-2)) return;
        s("CLPASS", 040000);
        reqDatZ(() -> {
            nouns.lodnntab();
            if (!putCom(2)) return;
            wr(g("NOUNADD") + 2, putVal);
            loadLv();
        });
    }

    /** LOADLV: completes a load; releases the display and goes to RECALTST. */
    public void loadLv() {
        s("DECBRNCH", 0);
        s("LOADSTAT", Agc.WORD_MASK);                 // -0 : data in
        relDsp();
        s("DSPCOUNT", neg(VD1));                      // block numeric chars, clears after load
        recaltst();
    }

    private boolean allDcOc(int expected) {
        int bits = (g("DECBRNCH") >> 2) & 7;          // DEC COMP bits R1,R2,R3 (bits 5,4,3)
        if (bits == 0) return true;                   // all octal
        int need = expected == -6 ? 7 : 6;            // all required are DEC: 3 or 2 components
        if (bits != need) { almCycle(); return false; }
        return true;
    }

    private int putVal;

    /**
     * PUTCOM: converts and checks component {@code k} (0..2) in XREG/XREGLP; leaves in MPAC the
     * value to be stored at NOUNADD+k and sets NOUNADD for the component. Returns false if an
     * alarm/recycle has already terminated the load.
     */
    private boolean putCom(int k) {
        s("DECOUNT", k);
        s("MPAC+6", 0);
        String lp = new String[] {"XREGLP", "YREGLP", "ZREGLP"}[k];
        String hi = new String[] {"XREG", "YREG", "ZREG"}[k];
        s("MPAC+1", g(lp));
        s(lp, 0);
        s("MPAC", g(hi));
        s(hi, 0);
        boolean decimal = g("DECBRNCH") != 0;
        int rut;
        if (g("MIXBR") == 2) {                        // mixed noun: component K address
            int ecadr = g(k == 0 ? "IDAD1TEM" : k == 1 ? "IDAD2TEM" : "IDAD3TEM") & LOW11;
            setNcadr(ecadr);
            s("NOUNADD", g("NOUNADD") - k);           // (E SUBK) - K
            if (decimal) return putDecSf();
            if (!dcTstCyc()) return false;
            rut = sfRutMix();
            if (dpTest(rut)) putDpCom();              // octal into DP: major part zero
            putVal = g("MPAC");
            return true;
        }
        setNadd();
        if (decimal) return putDecSf();
        if (!dcTstCyc()) return false;
        rut = sfRutNor();
        if (dpTest(rut)) {
            s("DECOUNT", 0);
            putDpCom();
            putVal = g("MPAC");
            return true;
        }
        if (g("NNADTEM") == Agc.WORD_MASK - 1) {      // -1 : channel to be specified
            chanLoad();
            return false;
        }
        putVal = g("MPAC");
        return true;
    }

    /** PUTDPCOM: E+1 into NOUNADD for the minor part; zero the major part. */
    private void putDpCom() {
        s("NOUNADD", g("NOUNADD") + 1);
        int e1 = g("NOUNADD") + g("DECOUNT");
        wr(e1 - 1, 0);
    }

    /** CHANLOAD: load a channel (but never channel 7, the superbank register). */
    private void chanLoad() {
        if (g("NOUNCADR") != 7) {
            int ch = g("NOUNCADR") & 0x1FF;
            agc.set("CHAN" + Integer.toOctalString(ch), g("MPAC"));
        }
        loadLv();
    }

    /** PUTDECSF: decimal load through the noun's scale factor routine (GTSFIN loads SFTEMP). */
    private boolean putDecSf() {
        int twiceSf = sfConum() * 2;
        nouns.gtsfin(twiceSf);
        int rut = g("MIXBR") == 2 ? sfRutMix() : sfRutNor();
        boolean ok;
        switch (rut) {
            case 0: almCycle(); return false;         // dec load with octal-only noun
            case 1: ok = binRound(); break;
            case 2: ok = degInSf(); break;
            case 3: ok = arthInSf(); break;
            case 4: ok = dpInSf(); break;
            case 5: ok = dpInSf2(); break;
            case 6: ok = optDegIn(); break;
            case 7: ok = dpInSf(); break;             // same as ARITHDP1
            case 8: ok = hmsIn(); break;
            case 9: dspAlarm(); return false;         // MIN/SEC can't be loaded
            case 10: ok = dpInSf4(); break;
            case 11: ok = artIn1Sf(); break;
            case 12: dspAlarm(); return false;        // 2INTOUT can't be loaded
            case 13: ok = dpFracIn(); break;
            default: godspalm(); return false;
        }
        if (ok) putVal = g("MPAC");
        return ok;
    }

    // ---- SF in routines (results in MPAC, MPAC+1; scale factors in SFTEMP1, SFTEMP2) ----

    public static final long DEGCON1 = Math.round(5.555555555 / 8 * (1L << 28));
    public static final long DEGCON2 = Math.round(2.222222222 / 4 * (1L << 28));
    public static final int NEG_DOT2 = 0xFFFF & ~06250 & Agc.WORD_MASK;   // -.197753906 bias scaled

    /** DEGINSF applies 1000/180 * 5.55555(10) = 5.43434(8). */
    private boolean degInSf() {
        dmp(DEGCON1);
        round(0x0800);                                // BIT11 rounding before shift left 3
        return degInSf2();
    }

    private boolean degInSf2() {
        long t = getTp();
        t <<= 2;                                      // LEFT 2 (TPSL1 twice)
        if (Math.abs(t) >= (1L << 42)) { almCycle(); return false; }   // TESTOFUF
        t <<= 1;                                      // LEFT 3
        boolean of = Math.abs(t) >= (1L << 42);
        long mag = Math.abs(t) & ((1L << 42) - 1);
        setTp(t < 0 ? -mag : mag);
        if (of) {                                     // SGNTO1: force sign to 1
            s("MPAC", g("MPAC") | 040000);
        } else if (t < 0 && g("MPAC") == 0) {         // UF: force sign to 0 except -180
            s("MPAC", 037777 & g("MPAC"));
        }
        return true;                                  // ENDSCALE -> PUTCOM2
    }

    /** ARTHINSF scales MPAC,+1 by SFTEMP1,2 (point between hi and lo); result shifted left 14. */
    private boolean arthInSf() {
        dmp(((long) g("SFTEMP1") << 14) | g("SFTEMP2"));
        int t0 = g("MPAC+2"), t1 = g("MPAC+1"), t2 = g("MPAC");
        s("MPAC", t1);
        s("MPAC+1", t0);
        s("MPAC+2", 0);
        if (t2 != 0) { almCycle(); return false; }    // too large a load
        return binRound();
    }

    /** BINROUND: round MPAC+1 into MPAC (2ROUND) and test for overflow. */
    private boolean binRound() {
        round(0);
        if (g("MPAC+6") != 0) { almCycle(); return false; }
        return true;
    }

    private boolean artIn1Sf() {
        dmp(((long) g("SFTEMP1") << 14) | g("SFTEMP2"));
        return binRound();
    }

    /** OPTDEGIN: optics scaling; rejects negative input; subtract 20 deg bias; x 100/45. */
    private boolean optDegIn() {
        if (ccs(g("MPAC")) >= 2) { almCycle(); return false; }
        s("MPAC", Agc.add(g("MPAC"), NEG_DOT2));
        dmp(DEGCON2);
        round(0x0400);                                // BIT12 rounding as in DEGINSF
        return degInSf2();
    }

    /** DPINSF: scale by SF, store low part in E+1, doubling MPAC+2 into rounding. */
    private boolean dpInSf() {
        dmp(((long) g("SFTEMP1") << 14) | g("SFTEMP2"));
        return dpInEnd();
    }

    private boolean dpInEnd() {
        s("MPAC+2", (g("MPAC+2") << 1) & 0x3FFF);
        round(0);
        if (g("MPAC+6") != 0) { almCycle(); return false; }
        return dpFracIn();
    }

    /** DPFRACIN: store the low part (MPAC+1) at (E SUBK)+1 or E+1. */
    private boolean dpFracIn() {
        int off = g("MIXBR") == 2 ? g("DECOUNT") : 0;
        wr(g("NOUNADD") + off + 1, g("MPAC+1"));
        return true;
    }

    /** DPINSF2: SF point between bits 7-8 of hi part; results shifted left 7. */
    private boolean dpInSf2() {
        dmp(((long) g("SFTEMP1") << 14) | g("SFTEMP2"));
        tpLeftN(7);
        return dpInEnd();
    }

    /** DPINSF4: SF point between bits 11-12; results shifted left 3. */
    private boolean dpInSf4() {
        dmp(((long) g("SFTEMP1") << 14) | g("SFTEMP2"));
        tpLeftN(3);
        return dpInEnd();
    }

    /** 2ROUND: round MPAC+2 into MPAC+1 (and carry into MPAC); MPAC+6 flags overflow. */
    private void round(int bias) {
        long t = getTp();
        t += (t >= 0 ? 1 : -1) * ((long) bias << 14);
        long half = (t >= 0 ? 1L : -1L) * (1L << 13);
        t += half;
        t -= t % (1L << 14);                          // drop MPAC+2
        s("MPAC+6", Math.abs(t) >= (1L << 42) ? (t > 0 ? 1 : -1) : 0);
        setTp(t);
    }

    /** TPLEFTN: shift MPAC,+1,+2 left n bits setting MPAC+6 for OF/UF. */
    private void tpLeftN(int n) {
        for (int i = 0; i < n; i++) tpSl1();
    }

    /**
     * HMSIN: hours, minutes, seconds loaded as three decimal words become centiseconds in
     * a DP word at NOUNADD. Alarms (recycle) if not all three words were loaded in decimal or
     * if the total exceeds 745 hr 39 min 14.59 sec.
     */
    private boolean hmsIn() {
        if (!all3Dec()) return false;
        long hours = whole(g("MPAC"), g("MPAC+1"));
        if (hours < 0 || hours > 745) { almCycle(); return false; }
        long cs = hours * 360000L;
        long min = whole(g("YREG"), g("YREGLP"));
        if (min < 0 || min > 59) { almCycle(); return false; }
        cs += min * 6000L;
        long sec = whole(g("ZREG"), g("ZREGLP"));     // centiseconds
        if (sec < 0 || sec > 5999) { almCycle(); return false; }
        cs += sec;
        if (cs >= (1L << 28)) { almCycle(); return false; }
        setDp(cs);
        wr(g("NOUNADD"), g("MPAC"));
        wr(g("NOUNADD") + 1, g("MPAC+1"));
        loadLv();
        return false;                                 // load completes here (POSTJUMP LOADLV)
    }

    /** Whole number (x 10^5 fraction, rounded) from a DP fraction pair, as in WHOLECON/RND/TST. */
    private long whole(int hi, int lo) {
        long dp = ((long) hi << 14) | lo;
        return (dp * 100000L + (1L << 27)) >> 28;
    }

    /** ALL3DEC: all 3 words must have been loaded in decimal (DECBRNCH bits 3,4,5). */
    private boolean all3Dec() {
        if (((g("DECBRNCH") >> 2) & 7) != 7) {
            s("VERBSAVE", neg(OCT31));                // force verb 25 on recycle
            almCycle();
            return false;
        }
        return true;
    }

    // ------------------------------------------------------------------ monitor

    /**
     * MONITOR allows other keyboard activity. It is ended by VERB TERMINATE, PROCEED WITHOUT
     * DATA, RESEQUENCE, another monitor, or any NVSUB call passing the DSPLOCK. Suspended (not
     * ended) by any keyboard action except ERROR LIGHT RESET; resumes at KEY RELEASE. Saves the
     * noun and display verb in MONSAVE and NOUNCADR (MCTBS) in MONSAVE1; bit 15 of MONSAVE1 is
     * the KILL MONITOR bit, bit 14 the EXTERNAL MONITOR bit. MONSAVE +0 means monitor off.
     */
    public void monitor() {
        monit1(g("NOUNCADR") & ~060000 & Agc.WORD_MASK);
    }

    private void monit1(int monsave1) {
        int temp = monsave1;
        if (entret == endInst) {                      // externally initiated monitor
            temp |= 040000;                           // external monitor bit
            s("MONSAVE2", 0);                         // zero NVMONOPT options
        }
        int vn = ((g("VERBREG") & LOW7) << 7) | g("NOUNREG");
        s("DSPLOCK", 0);                              // so monitor can run
        if (ccs(g("CADRSTOR")) == 1) relDsp1();       // turn off KR lite if CADRSTOR/DSPLIST empty
        if (ccs(g("MONSAVE")) != 0) {                 // MONSAVE was 0: request MONREQ
            new Waitlist(agc).waitlist(1, this::monreq);
        }
        s("MONSAVE", vn);
        s("MONSAVE1", temp);
        entret.run();
    }

    /** MONREQ: waitlist task (once per second); schedules MONDO under the Executive. */
    public void monreq() {
        lodsampt();                                   // time snatched for noun 65
        if (ccs(g("MONSAVE1")) >= 2) { killMon(); return; }
        new Waitlist(agc).waitlist(MONDEL, this::monreq);
        new Executive(agc).novac(CHRPRIO, this::mondo);
        new Waitlist(agc).taskover();
    }

    /** KILLMON: zero MONSAVE and MONSAVE1 (turns off kill and external monitor bits). */
    public void killMon() {
        s("MONSAVE", 0);
        s("MONSAVE1", 0);
        new Waitlist(agc).taskover();
    }

    /** MONDO: Executive job which re-displays the monitored data. */
    public void mondo() {
        if (ccs(g("MONSAVE1")) >= 2) { endOfJob(); return; }   // terminate came since last MONREQ
        if (ccs(g("DSPLOCK")) != 1) { monBusy(); return; }     // NVSUB is busy
        updatnn(g("MONSAVE") & LOW7);                 // noun into NOUNREG and display it
        int verb = ((g("MONSAVE") & MID7) >> 7) - 10;  // monitor verb -> display verb (MONREF)
        s("VERBREG", verb);
        entret = monBack;                             // return to PASTEVB after data display
        s("MPAC+2", g("MONSAVE1") & ~060000 & Agc.WORD_MASK);
        testNn();                                     // TESTNN (may display MCTBS CADR)
    }

    /** PASTEVB: paste monitor (or NVMONOPT *please* verb) into the verb lights. */
    public void pastevb() {
        int opt = g("MONSAVE2") & MID7;
        int verb = opt != 0 ? opt >> 7 : (g("MONSAVE") & MID7) >> 7;
        updatvb(verb);
        s("REQRET", 0);                               // pasted verbs can be executed by operator
        blankSub(g("MONSAVE2"));                      // NVMONOPT blank option
        endOfJob();
    }

    private void monBusy() {
        relDspOn();
        endOfJob();
    }

    /**
     * DSPFMEM displays (in octal) any fixed register; used with MCTBS noun. The FCADR is punched
     * in; for banks 30 and above the third component of noun 26 must hold the superbank bits.
     * Summary: banks L/E 27 V27N01E(FCADR)E; banks G/E 30 V23N26E(SUPERBITS)E V27N01E(FCADR)E.
     */
    public void dspfmem() {
        s("DSPCOUNT", R1D1);
        int super_ = g("DSPTEM1+2");
        int fcadr = g("NOUNCADR");
        int word = new InterBankCommunication(agc).supdacal(fcadr | super_, fcadr,
                a -> agc.get("F" + Integer.toOctalString(a)));
        dspOctWo(word);
        endOfJob();
    }

    // -------------------------------------------------------- word display routines

    /** DSPSIGN: puts the sign of MPAC,+1 on the display; leaves magnitude. */
    private void dspSign() {
        int m = g("MPAC");
        int c = ccs(m);
        if (c <= 1) {
            plusOn();
        } else {
            s("MPAC", Agc.toSigned(m) < 0 ? -Agc.toSigned(m) : 0);   // |MPAC| (-0 -> +0)
            minusOn();
            s("MPAC+1", neg(g("MPAC+1")));
        }
    }

    /** DSPRND: round by 5 EXP-6; on overflow use +max. */
    private void dspRnd() {
        long dp = getDp() + 02476;
        if (dp >= (1L << 28)) dp = (1L << 28) - 1;
        setDp(dp);
    }

    /** DSPDECWD: MPAC,+1 as a sign and 5 decimal characters at DSPCOUNT; rounds by 5 EXP-6. */
    public void dspDecWd() {
        dspSign();
        dspRnd();
        dspDcWd1(4);
    }

    /** DSPDECNR: as DSPDECWD without rounding. */
    public void dspDecNr() {
        dspSign();
        dspDcWd1(4);
    }

    /** DSPDC2NR: sign and 2 decimal characters, no rounding. */
    public void dspDc2Nr() {
        dspSign();
        dspDcWd1(1);
    }

    /** DSPDCWD1: emit WDCNT+1 digits of the DP fraction in MPAC,+1 starting at DSPCOUNT. */
    private void dspDcWd1(int wdcnt) {
        for (int n = wdcnt; n >= 0; n--) {
            s("WDCNT", n);
            shortMp(BINCON);                          // MPAC = digit, MPAC+1,+2 remainder
            int digit = g("MPAC") & 0xF;
            s("CODE", T4ruptProgram.RELTAB[digit] & LOW5);
            s("MPAC", g("MPAC+1"));
            s("MPAC+1", g("MPAC+2"));
            s("MPAC+2", 0);
            int d = g("DSPCOUNT");
            s("COUNT", d);
            s("DSPCOUNT", ccs(d) == 0 ? d - 1 : d);   // decrement DSPCOUNT except at +0
            dspin();
        }
        s("DSPCOUNT", neg(VD1));
    }

    /** DSP2DEC: MPAC,+1 as sign and 10 decimal characters starting at DSPCOUNT. */
    public void dsp2Dec() {
        s("CODE", 0);
        dspin11(3);                                   // -R2 off
        dspin11(4);                                   // +R2 off
        dspSign();
        dspDcWd1(R2D1);                               // 10 characters
    }

    /**
     * DSPDECVN displays N (x 2EXP-14) as a 2 character decimal beginning at DSPCOUNT; scaled
     * to N/100 before display conversion.
     */
    public void dspDecVn(int a) {
        s("MPAC", (a * 0244) & 0x3FFF);               // x .01 (VNDSPCON), result from L
        s("MPAC+1", 0);
        s("MPAC+2", 0);
        dspDcWd1(1);                                  // no sign, no round, 2 characters
    }

    /** DSPOCTWD: C(A) as a 5 character octal starting at DSPCOUNT (signs blanked). */
    public void dspOctWo(int a) {
        s("DSPCOUNT", g("DSPCOUNT") + 040000);        // blank signs
        int v = a & Agc.WORD_MASK;
        for (int n = 4; n >= 0; n--) {
            int digit = (v >> (3 * n)) & 7;
            s("CODE", T4ruptProgram.RELTAB[digit] & LOW5);
            int d = g("DSPCOUNT");
            s("COUNT", d);
            s("DSPCOUNT", ccs(d) == 0 ? d - 1 : d);
            dspin();
        }
        s("DSPCOUNT", neg(VD1));                      // DSPLV: block numeric chars after display
    }

    /** DSP2BIT: C(A) as a 2 character octal (pre-cycled right, then 5 char logic). */
    public void dsp2Bit(int a) {
        int v = ((a >> 1) | ((a & 1) << 14)) & Agc.WORD_MASK;
        s("DSPCOUNT", g("DSPCOUNT") + 040000);
        for (int n = 1; n >= 0; n--) {
            int digit = (v >> (3 * n)) & 7;
            s("CODE", T4ruptProgram.RELTAB[digit] & LOW5);
            int d = g("DSPCOUNT");
            s("COUNT", d);
            s("DSPCOUNT", ccs(d) == 0 ? d - 1 : d);
            dspin();
        }
        s("DSPCOUNT", neg(VD1));
    }

    /**
     * DSPIN: COUNT holds 0..25 octal (display number, bit 14 = blank sign with left char), CODE
     * the 5 bit relay code. Puts the code into the proper half of DSPTAB[COUNT/2]; NOUT counts
     * the changed words for DSPOUT in T4RUPT.
     */
    public void dspin() {
        int count = g("COUNT");
        int dsrel = (count & LOW5) >> 1;
        s("DSREL", dsrel);
        int sel;
        if ((count & 1) != 0) {                       // left character if COUNT is odd
            s("CODE", (g("CODE") << 5) & Agc.WORD_MASK);
            sel = (count & 040000) != 0 ? 3 : 1;      // 3 also blanks the sign
        } else {
            sel = 0;
        }
        dspin1(dsrel, sel);
    }

    /** DSPIN1: DSMSK[sel] mask selection of the DSPTAB entry. */
    private void dspin1(int dsrel, int sel) {
        int old = g("DSPTAB" + dsrel);
        int mag = ccs(old) >= 2 ? neg(old) : old;     // magnitude of the entry
        int code = g("CODE");
        if ((mag & DSMSK[sel]) == code) return;       // same: nothing to do
        int val = (mag & ~DSMSK[sel] & Agc.WORD_MASK) + code;
        s("DSPTAB" + dsrel, neg(val));                // stored complemented (changed marker)
        if (ccs(old) == 0) s("NOUT", g("NOUT") + 1);  // old entry was +: one more to output
    }

    /** 11DSPIN: DSPIN for the 11 bit sign relays; DSPTAB index in A, bit 11 of CODE. */
    public void dspin11(int dsrel) {
        s("DSREL", dsrel);
        s("COUNT", 2);
        dspin1(dsrel, 2);
    }

    // ------------------------------------------------------- alarms and mode change

    /** PREDSPAL / DSPALARM entry used by internal alarm conditions. */
    public void dspAlarm() {
        if (entret == nvsubEndInst) {                 // NVSUB initiated: abort 01501
            new AlarmAndAbort(agc).poodoo(01501);
            return;
        }
        if (entret == monBack) {                      // alarm in a monitor: kill it
            kilMonOn();
            faltOn();
            pastevb();                                // put monitor verb back in VERBREG
            return;
        }
        charAlrm();
    }

    /** CHARALRM: keyboard initiated alarm: turn on operator error light and end job. */
    public void charAlrm() {
        faltOn();
        endOfJob();
    }

    /**
     * ALMCYCLE turns on the check fail light, redisplays the original verb and recycles to
     * execute the original verb/noun combination (used for bad data during loads and by MCTBS).
     */
    public void almCycle() {
        faltOn();
        int verb = neg(g("VERBSAVE"));
        s("REQRET", verb);                            // set for ENTPAS0
        updatvb(verb);
        enter();                                      // ENDALM
    }

    /**
     * MMCHANG uses the noun display until ENTER, then mode display. Goes to V37 with the new
     * major mode in A. Demands two numerical characters be punched in (else recycles).
     */
    public void mmchang() {
        reqMm(mmchangRef);
    }

    private void mmchang1() {
        if (g("DSPCOUNT") != neg(016)) {              // DSPCOUNT must be -ND2
            almCycle();
            return;
        }
        int mm = g("NOUNREG");
        s("NOUNREG", 0);
        s("DSPCOUNT", ND1);
        blank2();
        s("DSPCOUNT", neg(VD1));                      // block numeric chars
        new FreshStartAndRestart(agc).v37(mm);        // MODROUTB
    }

    private void reqMm(Runnable cont) {
        reqretTarget = cont;
        s("REQRET", Agc.fromSigned(-1));
        s("DSPCOUNT", ND1);
        s("NOUNREG", 0);
        blank2();
        flashOn();
        s("DECBRNCH", 1);                             // set for decimal
        entret.run();
    }

    /**
     * VBRQEXEC enters an Executive request for any address with any priority; ends with
     * ENDOFJOB. Assumes noun 26 was preloaded: comp 1 priority (bits 10-14) with bit 1 = 0 NOVAC
     * / 1 FINDVAC, comp 2 job address (12 bit), comp 3 BBCON.
     */
    public void vbrqexec() {
        int prio = g("DSPTEM1") & ~1 & Agc.WORD_MASK;
        boolean findvac = (g("DSPTEM1") & 1) != 0;
        relDsp();
        if (findvac) {
            new Executive(agc).findvac(prio, g("DSPTEM1+1"), g("DSPTEM1+2"));
        } else {
            new Executive(agc).novac(prio, g("DSPTEM1+1"), g("DSPTEM1+2"));
        }
        endOfJob();
    }

    /**
     * VBRQWAIT enters a Waitlist request for any address with any delay (noun 26: comp 1 delay,
     * comp 2 task address, comp 3 BBCON); ends with ENDOFJOB. The display system is released.
     */
    public void vbrqwait() {
        relDsp();
        int delay = g("DSPTEM1");
        agc.set("WAITLIST_TASK_CADR", g("DSPTEM1+1"));
        agc.set("WAITLIST_TASK_BBCON", g("DSPTEM1+2"));
        new Waitlist(agc).waitlist(delay, () -> { });
        endOfJob();
    }

    /** VBPROC: proceed without data (LOADSTAT = +1). */
    public void vbproc() {
        s("LOADSTAT", 1);
        vbprocCom();
    }

    /** VBTERM: terminate (LOADSTAT = -1). */
    public void vbterm() {
        s("LOADSTAT", Agc.fromSigned(-1));
        vbprocCom();
    }

    /** VBRESEQ: wakes ENDIDLE at the same line as final ENTER of a load (L+3). */
    public void vbreseq() {
        s("LOADSTAT", Agc.WORD_MASK);                 // make it look like data in
        vbprocCom();
    }

    private void vbprocCom() {
        kilMonOn();                                   // turn on kill monitor bit
        relDsp();
        flashOff();
        recaltst();                                   // any recall from ENDIDLE?
    }

    /** PROCKEY performs VBPROC under the Executive with CHRPRIO. */
    public void prockey() {
        s("REQRET", 0);
        s("DSPCOUNT", neg(VD1));
        vbproc();
    }

    /**
     * KEY RELEASE: always turns off the UPACT light and clears DSPLOCK. Highest priority is
     * un-suspending an externally initiated monitor; otherwise RELDSP, and if a job is in
     * ENDIDLE control goes to PINBRNCH to re-execute the NVSUB calls preceding ENDIDLE.
     */
    public void vbReldsp() {
        s("DSALMOUT", g("DSALMOUT") & ~04 & Agc.WORD_MASK);       // UPACT light off
        boolean oldLockSet = ccs(g("21/22REG")) != 1;
        boolean emb = (g("MONSAVE1") & 040000) != 0;
        if (oldLockSet && emb) {                      // unsuspend external monitor
            s("DSPLOCK", 0);
            if (ccs(g("CADRSTOR")) == 1) relDsp1();
            endOfJob();
            return;
        }
        tstLts4();
    }

    private void tstLts4() {
        relDsp();
        if (ccs(g("CADRSTOR")) != 1) {
            new DisplayInterfaceRoutines(agc).pinbrnch();   // re-establish displays
            return;
        }
        endOfJob();
    }

    // -------------------------------------------------------------------- NVSUB

    /**
     * NVSUB: used for subroutine calls from within the computer. A holds 0VVVVVVVNNNNNNN (7-bit
     * verb and noun). Negative codes blank the display: -4 full, -3 leave mode, -2 leave mode
     * and verb, -1 R's only. MPAC+2 may carry a CADR for MCTBS nouns.
     * Returns true if executed (original L+2), false if the display system is blocked (L+1,
     * nothing displayed). Aborts with 01501 on a display program alarm.
     */
    public boolean nvsub(int a) {
        return nvmonopt(a, 0);
    }

    /**
     * NVMONOPT: like NVSUB but with options in L for a monitor VN: bits 8-14 a *please* verb to
     * paste after each monitor display, bits 1-3 a BLANKSUB code for the data.
     */
    public boolean nvmonopt(int a, int l) {
        s("NVTEMP", a);
        if (nvBusy()) return false;
        nvsbcom(l);
        return true;
    }

    private boolean nvBusy() {
        return ((g("MONSAVE1") & 040000) + g("DSPLOCK")) != 0;
    }

    private void nvsbcom(int l) {
        s("MONSAVE2", l);
        kilMonOn();
        nvsub1();
    }

    /** NVSBWAIT: executes V/N if the system is free; if busy, puts the caller to sleep. */
    public boolean nvsbwait(int a, int callerCadr) {
        s("NVTEMP", a);
        if (nvBusy()) {
            nvsubUsy(callerCadr - 1);
            return false;
        }
        nvsbcom(0);
        return true;
    }

    /** NVSUBEND: restores the caller's bank and returns (here: control simply returns). */
    public void nvsubEnd() {
    }

    /** NVSUB1 (bank 41): do the work of NVSUB. */
    public void nvsub1() {
        entret = nvsubEndInst;
        switch (ccs(g("NVTEMP"))) {
            case 0: break;                            // normal call
            case 2: blankDsp(-Agc.toSigned(g("NVTEMP")) - 1); return;
            default: godspalm(); return;
        }
        int nvtemp = g("NVTEMP");
        int noun = nvtemp & LOW7;
        int verb = nvtemp >> 7;
        int mach = g("MPAC+2");
        if (noun == 0) {                              // display verb only
            updatvb(verb);
            s("REQRET", 0);
            nvsubEnd();
            return;
        }
        if (verb == 0) {                              // display noun only
            updatnn(noun);
            nvsubEnd();
            return;
        }
        updatvb(verb);
        updatnn(noun);
        s("LOADSTAT", 0);
        s("CLPASS", 0);
        s("REQRET", 0);
        s("MPAC+2", mach);                            // CADR for INTMCTBS
        entpas0();
    }

    /** BLANKDSP: blank the display per option A (0 R's, 1 + noun, 2 + verb, 3 + mode). */
    private void blankDsp(int a) {
        for (int code = a + 7; code >= 0; code--) {
            int old = g("DSPTAB" + code);
            s("DSPTAB" + code, neg(0x800));           // CS BIT12
            if (ccs(old) == 0) s("NOUT", g("NOUT") + 1);
        }
        int opt = Agc.toSigned(g("NVTEMP"));
        if (opt <= -3) { s("VERBREG", 0); }
        if (opt <= -2) { s("NOUNREG", 0); }
        if (opt <= -1) { s("CLPASS", 0); }
        if (opt == -4) { /* never touch MODREG */ }
        s("DSPCOUNT", neg(VD1));
        flashOff();                                   // protect against invisible flash
        s("REQRET", 0);
        nvsubEnd();
    }

    /** KILMONON: force bit 15 of MONSAVE1 (kill monitor), clearing the external monitor bit. */
    public void kilMonOn() {
        s("MONSAVE1", 040000);
    }

    /**
     * ENDIDLE: for internal programs waiting for an operator response. Puts the job to sleep;
     * RECALTST wakes it at L+1 (terminate), L+2 (proceed) or L+3 (data in/resequence). Aborts
     * with 01206 if a second job tries to sleep in PINBALL (CADRSTOR or DSPLIST not +0).
     */
    public void endIdle(int callerCadr) {
        isCadr0();
        isList0();
        s("CADRSTOR", (callerCadr & 01777) + 0);      // same strategy as MAKECADR
        new Executive(agc).jobsleep(callerCadr);
    }

    private void isCadr0() {
        if (ccs(g("CADRSTOR")) != 1) dspAbort();
    }

    private void isList0() {
        if (ccs(g("DSPLIST")) != 1) dspAbort();
    }

    private void dspAbort() {
        new AlarmAndAbort(agc).poodoo(01206);
    }

    /** JAMTERM lets programs perform the TERMINATE function (does ENDOFJOB). */
    public void jamterm() {
        s("REQRET", DEC34);                           // leave ENTER set for ENTPAS0
        s("DSPCOUNT", neg(VD1));
        vbterm();
    }

    /** JAMPROC lets programs perform the PROCEED/PROCEED WITHOUT DATA function. */
    public void jamproc() {
        s("REQRET", DEC33);
        s("DSPCOUNT", neg(VD1));
        vbproc();
    }

    /**
     * BLANKSUB blanks any combination of R1, R2, R3 (BIT1 R1, BIT2 R2, BIT3 R3). Returns true for
     * the 2+ return (executed or nothing to blank) and false for 1+ return (display blocked).
     * DSPCOUNT is restored.
     */
    public boolean blankSub(int code) {
        s("NVTEMP", code & 7);
        if (nvBusy()) return false;
        if (g("NVTEMP") == 0) return true;
        int saved = g("DSPCOUNT");
        if ((g("NVTEMP") & 1) != 0) blank5(R1D1);
        if ((g("NVTEMP") & 2) != 0) blank5(R2D1);
        if ((g("NVTEMP") & 4) != 0) blank5(R3D1);
        s("DSPCOUNT", saved);
        return true;
    }

    /**
     * DSPMM does not display MODREG directly: it puts an Executive request (priority 30000) for
     * DSPMMJB. MODREG -0 blanks the mode lights.
     */
    public void dspmm() {
        new Executive(agc).novac(CHRPRIO, this::dspmmjb);
    }

    /** DSPMMJB: the job which does the display of MODREG. */
    public void dspmmjb() {
        int saved = g("DSPCOUNT");
        s("DSPCOUNT", MD1);
        switch (ccs(g("MODREG"))) {
            case 0: case 1: dspDecVn(g("MODREG")); break;   // display MODREG
            case 3: blank2(); break;                  // -0 : blank MM
            default: break;                           // -NZ : do nothing
        }
        s("DSPCOUNT", saved);
        endOfJob();
    }

    /**
     * RECALTST: entered after data is loaded, RESEQUENCE, TERMINATE or PROCEED. Wakes the job
     * which did ENDIDLE (L+1 terminate, L+2 proceed, L+3 data in).
     */
    public void recaltst() {
        if (ccs(g("CADRSTOR")) == 1) { endOfJob(); return; }   // keyboard initiated
        int cadr = g("CADRSTOR");
        s("CADRSTOR", 0);
        new Executive(agc).jobwake(cadr);
        int add;
        switch (ccs(g("LOADSTAT"))) {
            case 0: add = 1; break;                   // proceed without data
            case 2: add = 0; break;                   // terminate
            case 3: add = 2; break;                   // data in or resequence
            default: endOfJob(); return;              // pathological
        }
        s("LOC", g("LOC") + add);                     // job resumes at L+1 + add
        s("MPAC", g("VERBREG"));                      // verb and noun at time of response
        s("MPAC+1", g("NOUNREG"));
        relDsp();
        endOfJob();
    }

    // ------------------------------------------------- fixed-fixed service routines

    /** SETNCADR: ECADR in A -> NOUNCADR, set EBANK, derive E ADRES into NOUNADD. */
    public void setNcadr(int ecadr) {
        s("NOUNCADR", ecadr);
        s("EBANK", ecadr);
        s("NOUNADD", (ecadr & LOW8) + OCT1400);
    }

    /** SETNADD: ECADR from NOUNCADR. */
    public void setNadd() {
        setNcadr(g("NOUNCADR"));
    }

    /** SETEBANK: ECADR in A: set EBANK and return E ADRES. */
    public int setEbank(int ecadr) {
        s("EBANK", ecadr);
        return (ecadr & LOW8) + OCT1400;
    }

    public static int right5(int a) { return ((a >> 5) | ((a & LOW5) << 10)) & Agc.WORD_MASK; }
    public static int left5(int a) { return ((a << 5) | (a >> 10)) & Agc.WORD_MASK; }

    /** FALTON: operator error light on (bit 7 of channel 11). */
    public void faltOn() { s("DSALMOUT", g("DSALMOUT") | 0100); }

    /** FALTOF: operator error light off. */
    public void faltOf() { s("DSALMOUT", g("DSALMOUT") & ~0100 & Agc.WORD_MASK); }

    /** RELDSPON: key release light on (bit 5 of channel 11). */
    public void relDspOn() { s("DSALMOUT", g("DSALMOUT") | 020); }

    /** FLASHON: V/N flash on (bit 6 of channel 11). */
    public void flashOn() { s("DSALMOUT", g("DSALMOUT") | 040); }

    /** FLASHOFF: V/N flash off. */
    public void flashOff() { s("DSALMOUT", g("DSALMOUT") & ~040 & Agc.WORD_MASK); }

    /** LODSAMPT: snatch TIME2, TIME1 into SAMPTIME. */
    public void lodsampt() {
        s("SAMPTIME", g("TIME2"));
        s("SAMPTIME+1", g("TIME1"));
    }

    /**
     * RELDSP: set DSPLOCK to +0, turn the key release light off, clear the external monitor
     * bit and wake any job in DSPLIST.
     */
    public void relDsp() {
        s("MONSAVE1", g("MONSAVE1") & ~040000 & Agc.WORD_MASK);
        if (ccs(g("DSPLIST")) != 1) {
            int cadr = g("DSPLIST");
            s("DSPLIST", 0);
            new Executive(agc).jobwake(cadr);
        }
        relDsp2();
    }

    private void relDsp2() {
        s("DSALMOUT", g("DSALMOUT") & ~020 & Agc.WORD_MASK);   // key release light off
        s("DSPLOCK", 0);
    }

    /**
     * RELDSP1: set DSPLOCK to +0 with no DSPLIST search. Key release light off only if DSPLIST
     * is empty.
     */
    public void relDsp1() {
        if (ccs(g("DSPLIST")) == 1) {
            relDsp2();
            return;
        }
        s("DSPLOCK", 0);
    }

    /**
     * NVSUBUSY puts the calling job to sleep until the operator releases the keyboard and
     * display system (also turns on key release). Aborts 01206 for a second sleeper.
     */
    public void nvsubUsy(int wakeCadr) {
        isCadr0();
        isList0();
        relDspOn();
        s("DSPLIST", wakeCadr);
        new Executive(agc).jobsleep(wakeCadr);
    }

    // -------------------------------------------------------------- test lights

    public static final int FULLDSP = 05675, FULLDSP1 = 07675, TSTCON1 = 0175, TSTCON2 = 040650;
    public static final int TSTCON3 = 0115, SHOLTS = 0764;

    /**
     * VBTSTLTS turns on all display panel lights. After 5 seconds it turns off the caution and
     * status lights. DSPLOCK is left busy until TSTLTS3 so the test is seen.
     */
    public void vbtstlts() {
        if (!new ExtendedVerbs(agc).chkpooh()) return;
        s("IMODES33", g("IMODES33") | 1);             // IMUMON won't turn out lamps
        s("DSALMOUT", g("DSALMOUT") | TSTCON1);       // UPLINK ACTIVITY, TEMP, KEY RLSE, FLASH, OPR ERR
        s("DSPTAB11", TSTCON2);                       // NO ATT, GIMBAL LOCK, TRACKER, PROG ALM
        s("CHAN13", g("CHAN13") | 01000);             // test alarm outbit
        for (int i = 10; i >= 0; i--) s("DSPTAB" + i, neg(FULLDSP));   // all 8's
        for (int i : new int[] {1, 4, 6}) s("DSPTAB" + i, neg(FULLDSP1));   // 3 plus signs
        s("NOUT", 11);
        new Waitlist(agc).waitlist(SHOLTS, this::tstlts2);
        endOfJob();
    }

    /** TSTLTS2: waitlist task: schedule TSTLTS3. */
    public void tstlts2() {
        new Executive(agc).novac(CHRPRIO, this::tstlts3);
        new Waitlist(agc).taskover();
    }

    /** TSTLTS3: executive job which restores the lights after the 5 second test. */
    public void tstlts3() {
        s("DSALMOUT", g("DSALMOUT") & ~TSTCON3 & Agc.WORD_MASK);
        s("CHAN13", g("CHAN13") & ~01000 & Agc.WORD_MASK);
        s("DSPTAB11", 040000 + (g("CHAN12") & 010));  // NO ATT follows bit 4 of channel 12
        s("IMODES33", (g("IMODES33") & ~016001 & Agc.WORD_MASK) + 020000);
        s("IMODES30", (g("IMODES30") & ~055000 & Agc.WORD_MASK) + 015000);
        s("OPTMODES", g("OPTMODES") | 0100);
        dspmm();                                      // redisplay MODREG
        kilMonOn();
        flashOff();
        tstLts4();
    }

    public void error() {
        s("DSPLOCK", g("21/22REG"));                  // restore original DSPLOCK
        s("DSALMOUT", g("DSALMOUT") | 02000);         // turn on CAUTION RESET outbit (bit 10 chan 11)
        s("DSPTAB11", (g("DSPTAB11") & GL_NOATT) + 040000);   // keep gimbal lock and no att
        s("IMODES33", g("IMODES33") | 016000);        // reset fail bits so a persisting failure alarms again
        s("IMODES30", g("IMODES30") | 01000);
        s("OPTMODES", g("OPTMODES") | 0100);
        s("CHAN13", g("CHAN13") & ~01000 & Agc.WORD_MASK);   // turn off TEST ALARM outbit
        s("DSALMOUT", g("DSALMOUT") & ~ERCON & Agc.WORD_MASK);   // uplink activity, operator error off
        for (int i = 10; i >= 0; i--) {               // TSTAB: force bit 12 of all DSPTAB entries to 1
            int w = g("DSPTAB" + i);
            boolean plus = ccs(w) <= 1;
            int mag = plus ? w : neg(w);
            mag |= ~NOTBIT12 & Agc.WORD_MASK;         // bit 12 forced to 1
            s("DSPTAB" + i, plus ? mag : neg(mag));
        }
        s("FAILREG", 0);
        s("FAILREG+1", 0);
        s("FAILREG+2", 0);
        s("SFAIL", 0);
        endOfJob();
    }

    // ---------------------------------------------------- DP / TP arithmetic helpers

    private static long mag(int hi, int lo) {
        return ((long) (hi & 0x3FFF) << 14) | (lo & 0x3FFF);
    }

    /** DP value of (MPAC, MPAC+1) as a signed 28 bit integer (sign from MPAC, -0 aware). */
    private long getDp() {
        int hi = g("MPAC"), lo = g("MPAC+1");
        boolean negative = (hi & Agc.SIGN_BIT) != 0 || (hi == 0 && (lo & Agc.SIGN_BIT) != 0);
        if (negative) {
            return -mag((~hi) & 0x3FFF, (~lo) & 0x3FFF);
        }
        return mag(hi, lo);
    }

    private void setDp(long v) {
        boolean negative = v < 0;
        long m = Math.abs(v) & ((1L << 28) - 1);
        int hi = (int) (m >> 14), lo = (int) (m & 0x3FFF);
        if (negative) { hi = Agc.fromSigned(-hi); lo = Agc.fromSigned(-lo); }
        s("MPAC", hi);
        s("MPAC+1", lo);
    }

    /** Signed value of (MPAC, MPAC+1, MPAC+2) as a 42-bit integer (MPAC weighs 2^28). */
    private long getTp() {
        int w0 = g("MPAC"), w1 = g("MPAC+1"), w2 = g("MPAC+2");
        boolean negative = (w0 & Agc.SIGN_BIT) != 0;
        if (negative) {
            w0 = (~w0) & Agc.WORD_MASK; w1 = (~w1) & 0x3FFF; w2 = (~w2) & 0x3FFF;
        }
        long m = ((long) (w0 & Agc.WORD_MASK) << 28) | ((long) (w1 & 0x3FFF) << 14) | (w2 & 0x3FFF);
        return negative ? -m : m;
    }

    private void setTp(long v) {
        boolean negative = v < 0;
        long m = Math.abs(v);
        int w0 = (int) ((m >> 28) & 0x3FFF), w1 = (int) ((m >> 14) & 0x3FFF), w2 = (int) (m & 0x3FFF);
        if (negative) { w0 = Agc.fromSigned(-w0); w1 = Agc.fromSigned(-w1); w2 = Agc.fromSigned(-w2); }
        s("MPAC", w0);
        s("MPAC+1", w1);
        s("MPAC+2", w2);
    }

    /** TPAGREE: make MPAC, MPAC+1 (and +2) agree in sign. */
    private void tpAgree() {
        setTp(getTp());
    }

    /** DMP: (MPAC, MPAC+1) x DP operand -> MPAC, +1, +2 (top 42 bits of the 56 bit product). */
    private void dmp(long operand) {
        long a = getDp();
        long prod = a * operand;                      // up to 56 bits
        setTp(prod >> 14);
    }

    /** SHORTMP: (MPAC, MPAC+1) x single precision A (integer) -> MPAC, +1, +2. */
    private void shortMp(int a) {
        setTp(getDp() * Agc.toSigned(a));
    }

    /** SHORTMP with explicit factors: A x DP constant. */
    private void shortMp(int a, long dp) {
        setTp(Agc.toSigned(a) * dp);
    }

    private long shortMpVal(int a, long dp) {
        return Agc.toSigned(a) * dp;
    }

    /** TPSL1: shift MPAC, +1, +2 left 1; MPAC+6 = +/-1 on OF/UF. */
    private void tpSl1() {
        long t = getTp() << 1;
        if (Math.abs(t) >= (1L << 42)) {
            s("MPAC+6", t > 0 ? 1 : Agc.fromSigned(-1));
            t = t > 0 ? t - (1L << 42) : t + (1L << 42);
        }
        setTp(t);
    }

    // ---- lower-case label aliases for callers in other files ----
    public void reldsp() { relDsp(); }
    public void reldsp1() { relDsp1(); }
    public void reldspon() { relDspOn(); }
    public void falton() { faltOn(); }
    public void faltof() { faltOf(); }
    public void flashon() { flashOn(); }
    public void flashoff() { flashOff(); }
    public void kilmonon() { kilMonOn(); }
    public void endidle(int callerCadr) { endIdle(callerCadr); }
    public void nvsubusy(int wakeCadr) { nvsubUsy(wakeCadr); }
    public boolean blanksub(int code) { return blankSub(code); }
    public void setncadr(int ecadr) { setNcadr(ecadr); }
    public void setnadd() { setNadd(); }
    public int setebank(int ecadr) { return setEbank(ecadr); }
    public void dspdecvn(int a) { dspDecVn(a); }
    public void dspoctwo(int a) { dspOctWo(a); }
    public void dsp2bit(int a, boolean unused) { dsp2Bit(a); }
    public void dspdecwd() { dspDecWd(); }
    public void dspdecnr() { dspDecNr(); }
    public void dspdc2nr() { dspDc2Nr(); }
    public void dsp2dec() { dsp2Dec(); }
    public void dspalarm() { dspAlarm(); }
    public void almcycle() { almCycle(); }
    public void blank2Alias() { blank2(); }
    public void vbreldsp() { vbReldsp(); }
    public void charalrm() { charAlrm(); }
    public void getinrel() { getInrel(); }
}
