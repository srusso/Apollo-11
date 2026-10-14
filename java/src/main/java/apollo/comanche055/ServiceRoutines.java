package apollo.comanche055;

import apollo.agc.Agc;

/**
 * SERVICE_ROUTINES.agc - flag set/clear, DELAYJOB, GENTRAN, B5OFF, tracker fail light.
 */
public final class ServiceRoutines {
    private static final int OCT77770 = 0x7FF8; // 077770
    private static final int OCT7 = 7;
    private static final int OCT40200 = 040200;
    private static final int BIT5 = 020;
    private static final int BIT7 = 0100;
    private static final int BIT15 = 040000;
    private static final int DELAYNUM = 6; // number of DELAYLOC slots minus one (ccs-loop start)

    private final Agc agc;

    public ServiceRoutines(Agc agc) {
        this.agc = agc;
    }

    /**
     * UPENT2: set a bit in a flagword. {@code code} holds the flagword index in its low 3 bits and the
     * bit mask in the remaining bits (the value of the named flag constant).
     */
    public void upent2(int code) {
        int word = code & OCT7;                 // which flagword
        int bit = code & OCT77770;              // bit information
        String sym = "FLAGWRD" + word;          // INHINT region
        agc.set("ITEMP1", bit);
        agc.set(sym, agc.get(sym) | bit);
    }

    /** DOWNENT2: clear a bit in a flagword (same encoding as {@link #upent2}). */
    public void downent2(int code) {
        int word = code & OCT7;
        int bit = code & OCT77770;
        String sym = "FLAGWRD" + word;
        agc.set(sym, agc.get(sym) & ~bit);
    }

    /**
     * UPFLAG: general flag setter. Returns the flagword value after the operation (A on return).
     * @param flagwordSymbol erasable name of the register holding the bit
     * @param bit            bit number 1-15
     */
    public int upflag(String flagwordSymbol, int bit) {
        return comflag(flagwordSymbol, bit, true);
    }

    /** DOWNFLAG: general flag clearer. */
    public int downflag(String flagwordSymbol, int bit) {
        return comflag(flagwordSymbol, bit, false);
    }

    /** COMFLAG: common tail of UPFLAG/DOWNFLAG (DEBIT decodes the bit; ROR/MASK applies it). */
    private int comflag(String sym, int bit, boolean set) {
        int mask = 1 << (bit - 1);
        int value = agc.get(sym);
        value = set ? (value | mask) : (value & ~mask);
        agc.set(sym, value);
        return agc.get(sym);
    }

    /**
     * DELAYJOB: delay the calling job by dt centiseconds (WAITLIST task wakes it with JOBWAKE).
     * Bails out with alarm 01104 if no DELAYLOC is free.
     */
    public void delayjob(int dt) {
        int slot = DELAYNUM;
        while (slot >= 0 && agc.get("DELAYLOC" + slot) != 0) { // DELLOOP via CCS
            slot--;
        }
        if (slot < 0) {
            new AlarmAndAbort(agc).bailout(01104);
            return;
        }
        // OK2DELAY: set up waitlist call of WAKER after dt centiseconds, then TCGETCAD stores the
        // caller's cadr in DELAYLOC and JOBSLEEPs.
        agc.set("RUPTREG1", slot);
        agc.set("DELAYLOC" + slot, 1); // marks the slot busy (caller's FCADR in the original)
        new Waitlist(agc).dly(dt, () -> waker(agc.get("RUPTREG1")));
        new Executive(agc).jobsleep();
    }

    /** WAKER: waitlist task freeing a DELAYLOC slot and waking the sleeping job. */
    public void waker(int slot) {
        agc.set("DELAYLOC" + slot, 0);
        new Executive(agc).jobwake(slot);
        // TC TASKOVER
    }

    /**
     * GENTRAN: block transfer of n words from {@code src} to {@code dst} (symbolic array bases).
     * Words are copied from the highest address downward, as in the original.
     */
    public void gentran(int nMinus1, String srcBase, String dstBase) {
        for (int i = nMinus1; i >= 0; i--) {
            agc.set(dstBase + "+" + i, agc.get(srcBase + "+" + i));
        }
    }

    /** B5OFF: zero bit 5 of EXTVBACT (set by TESTXACT), then end the job. */
    public void b5off() {
        agc.set("EXTVBACT", agc.get("EXTVBACT") & ~BIT5);
        new Executive(agc).endofjob();
    }

    /** TRFAILOF: turn off the tracker fail light (DSPTAB+11D) but keep OCDU FAIL if it was on. */
    public void trfailof() {
        int dsp = agc.get("DSPTAB+11D") & ~OCT40200;
        agc.set("DSPTAB+11D", Agc.add(dsp, BIT15));
        int opt = agc.get("OPTMODES");
        agc.set("OPTMODES", opt | BIT7);
        // REQ: RELINT, return
    }

    /** TRFAILON: turn on the tracker fail light. */
    public void trfailon() {
        int dsp = agc.get("DSPTAB+11D");
        agc.set("DSPTAB+11D", dsp | OCT40200);
    }
}
