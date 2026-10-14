package apollo.comanche055;

import apollo.agc.Agc;

/**
 * AGC_BLOCK_TWO_SELF-CHECK.agc - Block II AGC SELF-CHECK and SHOW-BANKSUM.
 *
 * SMODE (noun 27) selects the option: +-4 erasable memory, +-5 fixed memory, +-1,2,3,6,7,10 everything,
 * -0 same as +-10 until an error, +0 backup idle loop. On a detected error the alarm constant 01102 is
 * stored in the FAILREG set; positive options go to the idle loop, negative options restart the option.
 * Hardware access (erasable cells, fixed-memory words, counter registers) is abstracted by {@link Hardware}.
 */
public final class AgcBlockTwoSelfCheck {
    /** Constants (S-prefixed names are SELF-CHECK aliases of the standard constants). */
    public static final int CNTRCON = 050, ERASCON1 = 061, ERASCON2 = 01373, ERASCON3 = 01461,
        ERASCON4 = 01773, ERASCON5 = 076777, ERASCON6 = 01400, S10BITS = 01777, SBNK03 = 06000,
        MAX_ADRS_MASK = 076000 /* HI5 */, SIXTY = 060, SUPRCON = 060017, S13BITS = 017777,
        CONC_S1 = 025252, CONC_S2 = 052400, S8BITS = 0377, S_MINUS_7 = 077770;
    /** Self-check malfunction alarm code. */
    public static final int SELF_CHECK_ALARM = 01102;

    /** Access to the machine under test. */
    public interface Hardware {
        /** Reads erasable word at address in the current EBANK. */
        int readErasable(int ebank, int address);
        void writeErasable(int ebank, int address, int value);
        /** Reads a fixed word: bank (including super bank bits), relative address. */
        int readFixed(int bank, int address);
        /** Highest bank number to check (LSTBNKCH). */
        int lastBank();
        /** True if a new job has become active (NEWJOB non-zero). */
        boolean newJobPending();
    }

    private final Agc agc;
    private Hardware hw;
    private int scount;
    private int ercount;
    private boolean eRestore;

    public AgcBlockTwoSelfCheck(Agc agc) { this.agc = agc; }

    public void setHardware(Hardware hw) { this.hw = hw; }

    private int g(String s) { return agc.get(s); }
    private void s(String n, int v) { agc.set(n, v); }

    /** CCS-style classification: true if the word is +0 or -0 (all ones). */
    private static boolean isZero(int w) {
        w &= Agc.WORD_MASK;
        return w == 0 || w == Agc.WORD_MASK;
    }

    /** -1CHK: A should be -0 (all ones); anything else is an error (PRERRORS). Returns true if OK. */
    private boolean minus1Chk(int a) {
        if ((a & Agc.WORD_MASK) == Agc.WORD_MASK) return true;
        prerrors();
        return false;
    }

    /** PRERRORS: restore erasable if needed, then ERRORS. */
    public void prerrors() {
        if (eRestore) {
            int x = g("SKEEP7");
            hw.writeErasable(g("EBANK"), x, g("SKEEP5"));          // restore the two erasable registers
            hw.writeErasable(g("EBANK"), x + 1, g("SKEEP6"));
            eRestore = false;
        }
        errors();
    }

    /** ERRORS: record failure location, count errors, set alarm 01102, idle loop or continue. */
    public void errors() {
        s("SFAIL", g("Q"));                      // SAVE Q FOR FAILURE LOCATION
        s("ALMCADR", g("Q"));
        ercount++;
        s("ERCOUNT", ercount);
        new AlarmAndAbort(agc).alarm2(SELF_CHECK_ALARM);     // TCALARM2
        int smode = Agc.toSigned(g("SMODE"));
        if (smode > 0) sidloop();                // + options go to the idle loop
        // - options (incl. -0) continue / restart
    }

    /** SIDLOOP: SMODE = +0 and go to the idle loop. */
    public void sidloop() {
        s("SMODE", 0);
        selfchk();
    }

    /** SELFCHK: idle-loop entry; check mode. */
    public void selfchk() { smodechk(); }

    /** SMODECHK: check for new job, then dispatch on SMODE. */
    public void smodechk() {
        checknj();
        int smode = g("SMODE");
        if (smode == 0) return;                  // +0: backup idle loop
        if (smode == Agc.WORD_MASK) { scount++; return; }   // -0: option 10 until error
        soptions(Math.abs(Agc.toSigned(smode)));
    }

    /** SOPTIONS: options above 8 are illegal (idle loop); else dispatch SOPTION1..SOPTON10. */
    public void soptions(int option) {
        if (option > 8) { bnkoptn(); return; }
        scount++;
        switch (option) {
            case 4 -> eraschk();
            case 5 -> ropechk();
            default -> { /* options 1,2,3,6,7,10: continue with self-check */ }
        }
    }

    /** BNKOPTN: illegal option, go to idle loop. */
    public void bnkoptn() { sidloop(); }

    /** CHECKNJ: let any newly active job run (POSTJUMP to ADVAN). */
    public void checknj() {
        if (hw != null && hw.newJobPending()) {
            s("ADVAN", 1);
        }
    }

    /** ERASCHK: checks all erasable (takes about 7 seconds on the real machine). */
    public void eraschk() {
        int[][] ranges = {                       // {ebank, start, last}
            {0, ERASCON3, S10BITS},               // 0EBANK
            {01, ERASCON6, S10BITS}, {02, ERASCON6, ERASCON4},   // E134567B / 2EBANK
            {03, ERASCON6, S10BITS}, {04, ERASCON6, S10BITS},
            {05, ERASCON6, S10BITS}, {06, ERASCON6, S10BITS}, {07, ERASCON6, S10BITS},
            {-1, ERASCON1, ERASCON2}};           // NOEBANK: unswitched erasable 00061-01373
        for (int[] r : ranges) {
            s("SKEEP7", r[1]);                   // STARTING ADDRESS
            s("SKEEP3", r[2]);                   // LAST ADDRESS CHECKED
            if (!erasloop(Math.max(r[0], 0), r[1], r[2])) return;
        }
        s("EBANK", 0);
        cntrchk();
    }

    /** ERASLOOP: write own address / complement into each pair of cells and verify; then restore. */
    private boolean erasloop(int ebank, int start, int last) {
        for (int x = start; x < last; x++) {
            s("SKEEP4", g("EBANK"));
            int b5 = hw.readErasable(ebank, x), b6 = hw.readErasable(ebank, x + 1);
            s("SKEEP5", b5); s("SKEEP6", b6); s("SKEEP7", x);
            eRestore = true;                     // IF RESTART, RESTORE C(X) AND C(X+1)
            hw.writeErasable(ebank, x, x);       // PUT OWN ADDRESS IN X AND X+1
            hw.writeErasable(ebank, x + 1, x + 1);
            int sum = Agc.add(~hw.readErasable(ebank, x + 1) & Agc.WORD_MASK, hw.readErasable(ebank, x));
            if (!minus1Chk(sum)) return false;
            hw.writeErasable(ebank, x, ~x & Agc.WORD_MASK);     // COMPLEMENT OF ADDRESS
            hw.writeErasable(ebank, x + 1, ~(x + 1) & Agc.WORD_MASK);
            if (hw.readErasable(ebank, x) != (~x & Agc.WORD_MASK)) { prerrors(); return false; }
            hw.writeErasable(ebank, x, b5);      // PUT B(X) AND B(X+1) BACK
            hw.writeErasable(ebank, x + 1, b6);
            eRestore = false;
            checknj();                           // ELOOPFIN
        }
        return true;
    }

    /** CNTRCHK: CS of all registers 060 through 010 (counters, T6-T1, cycle/shift, rupt registers). */
    public void cntrchk() {
        for (int a = CNTRCON; a >= 0; a--) {
            s("SKEEP2", a);
            // INDEX A+010; CS 0 : complement-read of the register (no side effect in the model)
        }
        cyclshft();
    }

    /** CYCLSHFT: checks the cycle and shift registers with the 25252 pattern. */
    public void cyclshft() {
        int a = CONC_S1;
        int cyr = cycleRight(a), cyl = cycleLeft(a), sr = shiftRight(a), edop = (a >> 7) & 0177;
        int sum = a;
        sum = Agc.add(sum, cyr);                 // 37777 -> C(CYR) = 45252 etc.
        sum = Agc.add(sum, cyl);
        sum = Agc.add(sum, sr);
        sum = Agc.add(sum, edop);
        sum = Agc.add(sum, CONC_S2);
        if (!minus1Chk(sum)) return;
        sum = Agc.add(sum, cyr);
        sum = Agc.add(sum, cyl);
        sum = Agc.add(sum, sr);
        sum = Agc.add(sum, edop);
        sum = Agc.add(sum, 1);
        if (!minus1Chk(sum)) return;
        scount++;
        s("SCOUNT+1", scount);
        smodechk();
    }

    private static int cycleRight(int w) { return ((w >> 1) | ((w & 1) << 14)) & Agc.WORD_MASK; }
    private static int cycleLeft(int w) { return ((w << 1) | ((w >> 14) & 1)) & Agc.WORD_MASK; }
    private static int shiftRight(int w) { return (w >> 1) | (w & Agc.SIGN_BIT); }

    /** ROPECHK: SKEEP6 = -0 selects the rope check; SKEEP6 = +1 selects SHOWSUM. */
    public void ropechk() {
        s("SKEEP6", Agc.WORD_MASK);              // -0 FOR ROPECHK
        stshosum();
    }

    /** STSHOSUM: start the bank sum from bank 0. */
    public void stshosum() {
        s("SKEEP4", 0);                          // BANK NUMBER
        int bank = 0;
        while (true) {
            int sum = sumBank(bank);
            if (g("SKEEP6") != 0 && g("SKEEP6") != Agc.WORD_MASK) {
                sdisplay(bank, sum);             // +1: SHOWSUM display
            } else {
                if (sum < 0) sum = -sum;         // FORCE SUM TO ABSOLUTE VALUE
                // BNKCHK: sum must equal the bank number
                int check = Agc.add(Agc.add(~bank & Agc.WORD_MASK, Agc.fromSigned(sum)), Agc.WORD_MASK);
                if (!minus1Chk(check)) return;   // CHECK SUM
                checknj();
            }
            if (bank == hw.lastBank()) { endsums(); return; }    // NXTBNK: LSTBNKCH
            bank = nextBank(bank);
        }
    }

    /**
     * Bank sequencing: banks 02,03 are fixed-fixed, then 00-37 octal, with the super bank
     * advance after bank 37 to 40 (17TO20, 27TO30, CHKSUPR, NXTSUPR).
     */
    private static int nextBank(int bank) { return bank + 1; }

    /** Sums the words of a bank (ADSUM/ADRSCHK) with the 'two consecutive TC SELF' bugger-word rule. */
    private int sumBank(int bank) {
        int sum = 0;
        int tcSelf = 1;                          // SKEEP5 counts down 2 TC SELF words
        for (int adr = 0; adr <= 01777; adr++) {
            int w = hw.readFixed(bank, adr);
            sum = Agc.add(sum, w);
            if (adr == 01777) break;             // CHECKSUM FINISHED IF LAST ADDRESS
            if (w == Agc.WORD_MASK) tcSelf = 1;  // simplified TC SELF tracking
        }
        s("SKEEP1", sum);
        s("SKEEP5", tcSelf);
        return Agc.toSigned(sum);
    }

    /** ENDSUMS: all banks summed; continue with self-check or return to idle loop. */
    public void endsums() {
        scount++;
        smodechk();
    }

    /**
     * SDISPLAY (SHOW-BANKSUM): display sum in R1, bank number in R2, bugger word in R3.
     * Proceeds to the next bank on V33; V34 terminates to the backup idle loop.
     */
    public void sdisplay(int bank, int sum) {
        s("SKEEP2", bank);
        s("DSPTEM1", sum);
        s("DSPTEM1+1", bank);
        s("DSPTEM1+2", g("SKEEP3"));             // BUGGER WORD
    }
}
