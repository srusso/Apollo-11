package apollo.luminary099;

import apollo.agc.Agc;

/**
 * T6-RUPT_PROGRAMS.agc - T6JOBCHK and DOT6RUPT: the LM DAP's exclusive control of TIME6
 * for reaction-control jet thrust timing. Also WRITEP/WRITEU/WRITEV output-channel writers.
 *
 * Conventions: only the DAP loads TIME6; TIME6 = POSMAX means inactive; -0 means a T6RUPT is
 * pending or in process; positive 1..240 means counting down.
 */
public class T6RuptPrograms {
    private final Agc agc;

    public static final int POSMAX = 0x3FFF;
    public static final int NEG_ZERO = 0x7FFF;
    /** 00314OCT: mask of the U/V-axis jet bits in channel 5. */
    public static final int OCT_00314 = 0x00C;
    public static final int BIT15 = 0x4000;
    public static final int PRIO37 = 0x37; // placeholder for PRIO37 check constant (assembler-defined)

    public T6RuptPrograms(Agc agc) {
        this.agc = agc;
    }

    /** T6JOBCHK: check TIME6 for a waiting T6RUPT; returns normally if clock is counting down. */
    public void t6JobChk() {
        int t6 = agc.get("TIME6");
        if (t6 == NEG_ZERO) {
            t6Job();
        } else if (t6 == 0) {
            // TC CCSHOLE: +0 should never be found
            throw new IllegalStateException("CCSHOLE");
        }
        // else TC Q: clock counting down
    }

    /** T6JOB: reached only when C(TIME6) = -0, i.e. a T6RUPT must be processed. */
    public void t6Job() {
        // disable clock (WAND CHAN13 with POSMAX clears bit 15)
        agc.set("CHAN13", agc.get("CHAN13") & POSMAX);
        // DXCH T6FURTHA ; DXCH T6NEXT ; LXCH NXT6ADR : shift the T6 task list down
        int furtherA = agc.get("T6FURTHA");
        int next = agc.get("T6NEXT");
        int nextAdr = agc.get("NXT6ADR");
        agc.set("T6NEXT", furtherA);
        agc.set("T6FURTHA", agc.get("T6FURTHA+1"));
        int newTime = nextAdr == 0 ? POSMAX : nextAdr;
        agc.set("TIME6", newTime);
        agc.set("NXT6ADR", agc.get("T6NEXT+1"));
        if (newTime != POSMAX) {
            enableT6();
        }
        goCh56(next);
    }

    /** ENABLET6: set bit 15 of channel 13. */
    private void enableT6() {
        agc.set("CHAN13", agc.get("CHAN13") | BIT15);
    }

    /** GOCH56: dispatch to WRITEP / WRITEU / WRITEV based on L (task index). */
    private void goCh56(int index) {
        switch (index) {
            case 0: writeP(); break;
            case 1: writeU(); break;
            default: writeV(); break;
        }
    }

    /** WRITEP: write NEXTP to channel 6 (pitch/roll jet control). */
    public void writeP() {
        agc.set("CHAN6", agc.get("NEXTP"));
    }

    /** WRITEU: replace U-axis jet bits of channel 5 with NEXTU. */
    public void writeU() {
        writeChan5(agc.get("NEXTU"));
    }

    /** WRITEV: replace V-axis jet bits of channel 5 with NEXTV. */
    public void writeV() {
        writeChan5(agc.get("NEXTV"));
    }

    private void writeChan5(int value) {
        int keep = agc.get("CHAN5") & ~OCT_00314 & Agc.WORD_MASK;
        agc.set("CHAN5", keep + (value & Agc.WORD_MASK));
    }

    /** DOT6RUPT: T6 interrupt service: call T6JOBCHK, then RESUME. */
    public void doT6Rupt() {
        t6JobChk();
        // TCF RESUME
    }
}
