package apollo.luminary099;

import apollo.agc.Agc;

/**
 * AGC_BLOCK_TWO_SELF_CHECK.agc (Luminary 1A build 099, pages 1284-1293). Mod 1 by Gauntt, 20 Dec 1967.
 *
 * SELF-CHECK runs as a zero priority job in the backup idle loop; SHOW-BANKSUM runs as a regular
 * executive job (V91 / V33). The option in SMODE (noun 27) selects: +-4 erasable memory, +-5 fixed
 * memory, +-1,2,3,6,7,10 everything in options 4 and 5, -0 same as +-10 until an error is detected,
 * +0 no check (backup idle loop). On error ALARM2 with code 01102 is raised; + options enter the
 * backup idle loop, - options restart the option, -0 proceeds after the failing line.
 *
 * Erasable cells: SMODE, SFAIL, ERCOUNT, ERESTORE, SKEEP1..SKEEP7, SCOUNT, SELFRET, LSTBNKCH.
 * Memory under test is modelled by the simulated erasable store ({@link Agc}, symbols "E<addr>")
 * and by a pluggable {@link RopeAccess} for fixed memory.
 */
public final class AgcBlockTwoSelfCheck {
    // Constants (SBITn = BITn etc.)
    public static final int CNTRCON = 050;
    public static final int ERASCON1 = 00061;
    public static final int ERASCON2 = 01373;
    public static final int ERASCON6 = 01400;
    public static final int ERASCON3 = 01461;
    public static final int ERASCON4 = 01773;
    public static final int ERASCON5 = 076777;
    public static final int S10BITS = 01777;
    public static final int SBNK03 = 06000;
    public static final int MAXADRS = 01777;
    public static final int SIXTY = 060;
    public static final int SUPRCON = 060017;
    public static final int S13BITS = 017777;
    public static final int CONC_S1 = 025252; // used in CYCLSHFT
    public static final int CONC_S2 = 052400; // used in CYCLSHFT
    public static final int S8BITS = 00377;
    public static final int HI5 = 076000;
    /** SELFADRS: SELFCHK return address (put in SELFRET going to SHOWSUM, in SKEEP1 coming back). */
    public static final String SELFADRS = "SELFCHK";
    public static final int ALARM_CODE = 01102;

    /** Access to fixed memory words for ROPECHK/SHOWSUM. */
    public interface RopeAccess {
        /** Word at (bank, address) in 15-bit one's-complement form. */
        int read(int bank, int address);
    }

    private final Agc agc;
    private RopeAccess rope = (b, a) -> 0;
    private int lastBank = 043; // LSTBNKCH: BBCON* constant, last bank to be checked

    public AgcBlockTwoSelfCheck(Agc agc) {
        this.agc = agc;
    }

    public void setRope(RopeAccess rope) {
        this.rope = rope;
    }

    /** Thrown to unwind to the idle loop when an option ends. */
    private static final class IdleLoop extends RuntimeException {
        IdleLoop() { super(null, null, false, false); }
    }

    private int g(String s) { return agc.get(s); }
    private void s(String sym, int v) { agc.set(sym, v); }

    // ---------------------------------------------------------------- error handling

    /** -1CHK: A must be -0 (77777) or +0 after one's complement check; otherwise PRERRORS. */
    private void minusOneCheck(int a) {
        // CCS A: positive -> error; +0 -> second CCS; -0 / <0 ... Accept only +-0 pattern.
        int v = Agc.toSigned(a);
        if (v > 0) { prerrors(); return; }
        if (a == 0 || a == Agc.WORD_MASK) {
            // CCS then TCF PRERRORS: +0 -> error, then CCS A again: -0 -> return
            if (a == 0) { prerrors(); return; }
            return;
        }
        // negative values: CCS A gives |A|-1 ; error unless magnitude is exactly 1 -> second CCS tests
        if (-v - 1 > 0) { prerrors(); return; }
    }

    /** PRERRORS: restore erasable if needed, then ERRORS. */
    private void prerrors() {
        if (g("ERESTORE") != 0) {
            // DCA SKEEP5 ; INDEX SKEEP7 ; DXCH 0000 : restore the two erasable registers.
            int x = g("SKEEP7");
            s("E" + x, g("SKEEP6"));
            s("E" + (x + 1), g("SKEEP5"));
            s("ERESTORE", 0);
        }
        errors();
    }

    /** ERRORS: inhibit, record failure location, count error, raise alarm 01102. */
    private void errors() {
        s("SFAIL", g("Q"));
        s("ALMCADR", g("Q"));
        s("ERCOUNT", g("ERCOUNT") + 1);
        s("ALARM2", ALARM_CODE); // TC ALARM2 ; OCT 01102 (self-check malfunction indicator)
        int smode = Agc.toSigned(g("SMODE"));
        if (smode > 0) {
            sidLoop(); // + options enter backup idle loop
        } else if (smode == 0 && g("SMODE") == 0) {
            sidLoop();
        }
        // negative options: restart option; -0: continue with self-check after TC SFAIL
    }

    /** SIDLOOP: set SMODE to +0 and go to idle loop. */
    private void sidLoop() {
        s("SMODE", 0);
        selfChk();
    }

    // ---------------------------------------------------------------- mode dispatch

    /** SELFCHK: the backup idle loop entry. */
    public void selfChk() {
        smodeChk();
    }

    /** SMODECHK: check for new job, then dispatch on SMODE. */
    public void smodeChk() {
        checkNj();
        int smode = g("SMODE");
        if (smode == 0) {
            return; // +0: backup idle loop (SMODECHK +2)
        }
        if (smode == Agc.WORD_MASK) { // -0: same as option 10
            sOptions(010);
        } else {
            sOptions(Math.abs(Agc.toSigned(smode)));
        }
        s("SCOUNT", g("SCOUNT") + 1);
    }

    /** SOPTIONS: options 1-10 dispatch (above 010 = BNKOPTN, illegal: idle loop). */
    private void sOptions(int option) {
        if (option > 010) {
            sidLoop(); // BNKOPTN
            return;
        }
        s("SCOUNT", g("SCOUNT") + 1);
        switch (option) {
            case 4: eraschk(); break; // SOPTION4
            case 5: ropeChk(); break; // SOPTION5
            default: break; // SOPTION1,2,3,6,7,10: TC SKEEP1 (continue with self-check)
        }
    }

    /** CHECKNJ: check whether any jobs have become active (TC POSTJUMP, CADR ADVAN). */
    private void checkNj() {
        s("SELFRET", g("Q"));
        // POSTJUMP to ADVAN lets the Executive run other jobs if NEWJOB is nonzero.
    }

    // ---------------------------------------------------------------- erasable check

    /**
     * ERASCHK: check erasable memory pairwise (takes about 7 seconds). SKEEP2 controls checking of
     * non-switchable erasable with bank numbers in EB: banks 0, 1,3,4,5,6,7, 2, then the no-ebank
     * region 00061-01373. For each pair (X, X+1) the original contents are stored, the cells are
     * written with their own address then the complement and checked via one's-complement
     * addition (-1CHK), then restored. Interrupts are inhibited around each pair.
     */
    public void eraschk() {
        s("SKEEP2", 1);
        // 0EBANK: bank 0, region 01461-01777
        eraseRegion(0, ERASCON3, S10BITS);
        // E134567B: banks 1,3,4,5,6,7 : 01400-01777 (bank 2 handled with 2EBANK limit 01773)
        for (int bank = 1; bank <= 7; bank++) {
            int last = bank == 2 ? ERASCON4 : S10BITS;
            s("EBANK", bank << 8);
            eraseRegion(bank, ERASCON6, last);
        }
        s("SKEEP2", 0);
        // NOEBANK: 00061 - 01373 in fixed erasable (no bank register)
        eraseRegion(0, ERASCON1, ERASCON2);
        s("EBANK", ERASCON6);
        cntrChk();
    }

    /** ERASLOOP over [start, last] in one bank. */
    private void eraseRegion(int bank, int start, int last) {
        for (int x = start; x < last; x++) {
            s("SKEEP7", x);
            s("SKEEP3", last);
            s("SKEEP4", g("EBANK"));
            String a = "E" + bank + "_" + x;
            String b = "E" + bank + "_" + (x + 1);
            int bx = g(a);
            int bx1 = g(b);
            s("SKEEP5", bx);
            s("SKEEP6", bx1);
            s("ERESTORE", x == 0 ? 1 : x); // if restart occurs, restore C(X), C(X+1)
            // put own address in X and X+1
            s(a, x);
            s(b, x);
            // CS X+1 ; AD X : x + ~x = -0 expected
            int sum = Agc.add(~g(b) & Agc.WORD_MASK, g(a));
            if (sum != Agc.WORD_MASK && sum != 0) {
                prerrors();
                continue;
            }
            // complement of address into both
            s(a, ~x & Agc.WORD_MASK);
            s(b, ~x & Agc.WORD_MASK);
            sum = Agc.add(~g(a) & Agc.WORD_MASK, g(b));
            if (sum != Agc.WORD_MASK && sum != 0) {
                prerrors();
                continue;
            }
            s(a, bx); // put B(X), B(X+1) back
            s(b, bx1);
            s("ERESTORE", 0);
            // ELOOPFIN: RELINT, CHECKNJ, restore EBANK
            checkNj();
        }
    }

    /**
     * CNTRCHK: performs a CS of all registers from octal 60 through octal 10 (counters, T6-1,
     * cycle and shift, RUPT registers), then CYCLSHFT.
     */
    private void cntrChk() {
        for (int r = CNTRCON; r > 0; r -= 8) {
            s("SKEEP2", r);
            // INDEX A ; CS 0000 : complement of register (x + 010)
        }
        cyclShft();
    }

    /** CYCLSHFT: checks the cycle (CYR, CYL) and shift (SR) registers and EDOP. */
    private void cyclShft() {
        int a = CONC_S1; // 25252
        int cyr = a, cyl = a, sr = a, edop = a;
        cyr = (cyr >> 1) | ((cyr & 1) << 14); // CYR  12525
        cyl = ((cyl << 1) & Agc.WORD_MASK) | ((cyl >> 14) & 1); // CYL 52524
        sr = (sr >> 1) | (sr & 0x4000); // SR   12525 (sign-extended shift)
        edop = (edop >> 7) & 0177; // EDOP 00125
        a = Agc.add(a, cyr);
        a = Agc.add(a, cyl);
        a = Agc.add(a, sr);
        a = Agc.add(a, edop);
        a = Agc.add(a, CONC_S2);
        minusOneCheck(a);
        a = Agc.add(a, cyr);
        a = Agc.add(a, cyl);
        a = Agc.add(a, sr);
        a = Agc.add(a, edop);
        a = Agc.add(a, 1);
        minusOneCheck(a);
        s("SCOUNT+1", g("SCOUNT+1") + 1);
        smodeChk();
    }

    // ---------------------------------------------------------------- fixed memory (rope) check

    /**
     * ROPECHK: checksum every fixed-memory bank. A bank is good when its sum equals plus or minus
     * the bank number (the "bugger word" in the last location of each bank forces this); two
     * consecutive TC SELF words (-0 checks) end a bank early. SKEEP6 = -0 for ROPECHK,
     * +1 for SHOWSUM (display of each bank sum).
     */
    public void ropeChk() {
        s("SKEEP6", Agc.WORD_MASK); // -0 for ROPECHK
        stShowSum();
    }

    /** STSHOWSUM: begin summing at bank 0. */
    public void stShowSum() {
        s("SKEEP4", 0);
        for (int bank = 0; bank <= lastBank; bank++) {
            s("SKEEP4", bank);
            int sum = bankSum(bank);
            if (g("SKEEP6") != Agc.WORD_MASK) {
                sDisplay(bank, sum);
                continue;
            }
            // SOPT/BNKCHK: force sum to absolute value, then compare against -bank - 1 +... expects -0.
            int abs = Math.abs(Agc.toSigned(sum));
            int check = Agc.add(Agc.add(~bank & Agc.WORD_MASK, Agc.fromSigned(abs)), Agc.WORD_MASK);
            minusOneCheck(check);
        }
        // ENDSUMS: finished all banks.
    }

    /** COMADRS / FXADRS / ADSUM / ADRSCHK: sum the words of one bank with end-around carry. */
    private int bankSum(int bank) {
        int sum = 0;
        int tcSelfRun = 1;
        // bank 2 and 3 are fixed-fixed (04000-07777), others common-fixed (02000-03777 window)
        int start = (bank == 2 || bank == 3) ? (bank == 2 ? 04000 : SBNK03) : 02000;
        for (int rel = 0; rel < 02000; rel++) {
            int addr = start + rel;
            int word = rope.read(bank, addr);
            sum = Agc.add(sum, word);
            // ADRSCHK: two consecutive TC SELF words (-0 CCS trick) end the checksum early
            int self = (addr & 03777) | 0; // TC to own address is the word whose value equals the address
            if (word == self && rel < 01777) {
                tcSelfRun--;
                if (tcSelfRun < 0) {
                    break;
                }
            } else {
                tcSelfRun = 1;
            }
            if ((addr & S10BITS) == MAXADRS) {
                break; // last relative address of bank
            }
            s("SKEEP3", addr + 1);
            checkNj();
        }
        s("SKEEP1", sum);
        return sum;
    }

    /**
     * SDISPLAY (SHOWSUM): display bank sum in R1, bank number in R2, bugger word in R3 (V91 first
     * bank, V33 proceed, V34 terminate -> backup idle loop).
     */
    private void sDisplay(int bank, int sum) {
        s("DSPR1", sum);
        s("DSPR2", bank);
        s("DSPR3", g("SKEEP3"));
    }

    /** NXTBNK helper: the next bank number accounting for super-bank hops (37-&gt;40, 27-&gt;30). */
    public static int nextBank(int bank) {
        return bank + 1; // bank numbers here are logical; BANKn/SUPERBANK arithmetic is in the rope map
    }
}
