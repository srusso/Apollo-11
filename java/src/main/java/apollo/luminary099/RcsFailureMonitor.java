package apollo.luminary099;

import apollo.agc.Agc;

/**
 * RCS_FAILURE_MONITOR.agc (Luminary 1A build 099, pages 190-192).
 * Author J S Miller. Attached to T4RUPT, entered every 480 ms. Examines the low 8 bits of
 * channel 32 (RCS isolation-valve closure bits) and, when they differ from PVALVEST, updates
 * the DAP jet-inhibit masks CH5MASK/CH6MASK and PVALVEST. At most one change (the highest
 * numbered bit) is accepted per entry. Exit is always RCSMONEX (= RESUME).
 */
public final class RcsFailureMonitor {
    private static final int LOW8 = 0x00FF;

    /** CH5 jet bit corresponding to CH32 bit 8..1 (5FAILTAB). */
    public static final int[] FAIL_TAB_5 = {0x20, 0x10, 0x40, 0x80, 0x08, 0x01, 0x04, 0x02};
    /** CH6 jet bit corresponding to CH32 bit 8..1 (6FAILTAB). */
    public static final int[] FAIL_TAB_6 = {0x08, 0x10, 0x04, 0x80, 0x01, 0x02, 0x20, 0x40};

    private final Agc agc;

    public RcsFailureMonitor(Agc agc) {
        this.agc = agc;
    }

    /** RCSMONIT / RCSMON. */
    public void rcsmon() {
        int c = ~agc.get("CHAN32") & Agc.WORD_MASK & LOW8; // inverted channel 32, low 8 bits
        int pvalvest = agc.get("PVALVEST");
        // P' C + P C' : bits that differ
        int diff = ((~pvalvest & c) | (~c & pvalvest)) & Agc.WORD_MASK;
        if (diff == 0) {
            rcsmonex();
            return;
        }
        // Find highest-numbered bit (bit 8 down to 1) needing action.
        int idx = 0; // index into tables: 0 => bit 8 ... 7 => bit 1
        int bit = 0;
        for (int i = 0; i < 8; i++) {
            int mask = 0x80 >> i; // BIT8 first
            if ((diff & mask) != 0) {
                idx = i;
                bit = mask;
                break;
            }
        }
        if ((pvalvest & bit) != 0) {
            vopened(idx, bit);
        } else {
            // The valve has just been closed: set inhibit bits.
            agc.set("CH5MASK", agc.get("CH5MASK") | FAIL_TAB_5[idx]);
            agc.set("CH6MASK", agc.get("CH6MASK") | FAIL_TAB_6[idx]);
            agc.set("PVALVEST", pvalvest | bit); // record action taken
            oneAccFix();
        }
    }

    /** VOPENED: a valve has just been opened; remove inhibit bits. */
    private void vopened(int idx, int bit) {
        agc.set("CH5MASK", agc.get("CH5MASK") & ~FAIL_TAB_5[idx]);
        agc.set("CH6MASK", agc.get("CH6MASK") & ~FAIL_TAB_6[idx]);
        agc.set("PVALVEST", agc.get("PVALVEST") & ~bit);
        oneAccFix();
    }

    /** 1/ACCFIX: set up job 1/ACCJOB at priority 27 (NOVAC) and exit. */
    private void oneAccFix() {
        // CAF PRIO27 ; TC NOVAC ; 2CADR 1/ACCJOB (EBANK= AOSQ)
        agc.set("NOVAC_REQUEST_1/ACCJOB", 27);
        rcsmonex();
    }

    /** RCSMONEX = RESUME. */
    private void rcsmonex() {
        // TCF RESUME: return from interrupt (handled by the executive).
    }
}
