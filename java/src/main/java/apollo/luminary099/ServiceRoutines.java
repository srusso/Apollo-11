package apollo.luminary099;

import apollo.agc.Agc;

/**
 * SERVICE_ROUTINES.agc - UPENT2/DOWNENT2 (flag word 4 bits), UPFLAG/DOWNFLAG (general
 * flag set/clear), DELAYJOB, GENTRAN (block transfer) and B5OFF.
 */
public class ServiceRoutines {
    private final Agc agc;

    public static final int OCT77770 = 0x7FF8;
    public static final int OCT7 = 7;
    /** Number of DELAYLOC slots (DELAYNUM). */
    public static final int DELAYNUM = 4;
    /** Alarm code used by DELAYJOB when no slot is free (OCT 1104). */
    public static final int DELAY_ALARM = 0x244; // octal 1104

    private final int[] delayLoc = new int[DELAYNUM + 1];

    public ServiceRoutines(Agc agc) {
        this.agc = agc;
    }

    /** UPENT2: set low 3 bits of FLAGWRD4 from the given value (INHINT section). */
    public void upEnt2(int a) {
        int l = a & OCT77770;
        int fw = agc.get("FLAGWRD4");
        agc.set("FLAGWRD4", fw | l);
        // JOIN: RELINT; TCF Q+1
    }

    /** DOWNENT2: clear flag bits of FLAGWRD4 per the mask. */
    public void downEnt2(int a) {
        int mask = ~(a & OCT77770) & Agc.WORD_MASK;
        agc.set("FLAGWRD4", agc.get("FLAGWRD4") & mask);
    }

    /**
     * UPFLAG: set a named bit. The bit is described by flagWord symbol name and bit number
     * (1..15); the original encoded this in the ADRES word following the TC.
     * @return the current flag word setting.
     */
    public int upFlag(String flagWord, int bit) {
        int w = agc.get(flagWord) | (1 << (bit - 1));
        agc.set(flagWord, w); // COMFLAG
        return w;
    }

    /** DOWNFLAG: clear a named bit. @return the current flag word setting. */
    public int downFlag(String flagWord, int bit) {
        int w = agc.get(flagWord) & ~(1 << (bit - 1)) & Agc.WORD_MASK;
        agc.set(flagWord, w);
        return w;
    }

    /**
     * DELAYJOB: delay the calling job dt centiseconds using the waitlist. A free DELAYLOC slot
     * records the caller; WAKER later frees the slot and wakes the job.
     * @param callerCadr identifier of the sleeping job's return address
     */
    public void delayJob(int dt, int callerCadr) {
        int slot = -1;
        for (int i = DELAYNUM; i >= 0; i--) { // DELLOOP
            if (delayLoc[i] == 0) { slot = i; break; }
        }
        if (slot < 0) {
            agc.set("BUF2", dt);
            bailout(DELAY_ALARM); // TC BAILOUT1 ; OCT 1104
            return;
        }
        // OK2DELAY: waitlist call DLY2 with WAKER task; TCGETCAD saves caller then JOBSLEEP
        delayLoc[slot] = callerCadr == 0 ? 1 : callerCadr;
        agc.set("DELAYTASK" + slot, dt);
    }

    /** WAKER: task fired by waitlist; frees the DELAYLOC slot and wakes the job (JOBWAKE). */
    public void waker(int slot) {
        delayLoc[slot] = 0; // XCH DELAYLOC
        // TC JOBWAKE ; TC TASKOVER
    }

    private void bailout(int code) {
        agc.set("BAILOUT", code);
    }

    /**
     * GENTRAN: block transfer of n consecutive words from a source symbol block to a destination
     * block (copies from the end down to the start, as in the original).
     */
    public void genTran(int n, String[] source, String[] dest) {
        for (int i = n - 1; i >= 0; i--) {
            agc.set(dest[i], agc.get(source[i]));
        }
    }

    /** Convenience overload copying n words from erasable block sym+0..n-1 to dsym+0..n-1. */
    public void genTran(int n, String sourcePrefix, String destPrefix) {
        for (int i = n - 1; i >= 0; i--) {
            agc.set(destPrefix + "+" + i, agc.get(sourcePrefix + "+" + i));
        }
    }

    /** B5OFF: clear bit 5 of EXTVBACT (set by TESTXACT), then ENDOFJOB. */
    public void b5Off() {
        agc.set("EXTVBACT", agc.get("EXTVBACT") & ~0x10 & Agc.WORD_MASK);
        // TC ENDOFJOB
    }
}
