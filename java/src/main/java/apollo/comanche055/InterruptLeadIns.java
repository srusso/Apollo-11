package apollo.comanche055;

import apollo.agc.Agc;

/**
 * INTERRUPT_LEAD_INS.agc: the interrupt vector block at 4000 (bank 02, COUNT 02/RUPTS).
 * Each vector saves A/L (DXCH ARUPT), loads the interrupt routine's bank/EBANK (BBCON)
 * and transfers to the handler. Here each vector is a method that dispatches to the
 * handler class by naming convention.
 */
public final class InterruptLeadIns {
    private final Agc agc;

    /** Interrupt vector indices (4000 + 4*n). */
    public enum Vector { T6RUPT, T5RUPT, T3RUPT, T4RUPT, KEYRUPT1, KEYRUPT2, UPRUPT, DOWNRUPT, RADAR_RUPT, HAND_CONTROL_RUPT }

    // BBCON/EBANK tags (bank register values used by each handler). Symbolic: EBANK symbol.
    public static final String GOBB_EBANK = "LST1";        // GOBB BBCON GOPROG (restart uses E0,E3)
    public static final String T3RPTBB_EBANK = "LST1";
    public static final String KEYRPTBB_EBANK = "KEYTEMP1";
    public static final String MKRUPTBB_EBANK = "MRKBUF1";
    public static final String UPRPTBB_EBANK = KEYRPTBB_EBANK; // UPRPTBB = KEYRPTBB
    public static final String DWNRPTBB_EBANK = "DNTMBUFF";
    public static final String RDRPTBB_EBANK = "DATATEST";
    public static final String HCRUPTBB_EBANK = "TIME1";   // NOT USED
    public static final String T4RPTBB_EBANK = "DSRUPTSW";
    public static final String T5RPTBB_EBANK = "TIME1";

    private int arupt, lrupt, bbank;

    public InterruptLeadIns(Agc agc) {
        this.agc = agc;
    }

    /** Address 4000: INHINT; CAF GOBB; XCH BBANK; TCF GOPROG (power-up / restart). */
    public void start() {
        bbank = 0;
        new Goprog(agc).goprog();
    }

    /** Vector 4004: T6RUPT. DXCH ARUPT; DCA T6LOC; DTCB (jump through T6LOC). */
    public void t6rupt() {
        saveAL();
        new Object() { void dtcb() { /* jump through T6LOC 2CADR */ } }.dtcb();
    }

    /** Vector 4010: T5RUPT. CS TIME5; AD .5SEC; TCF T5RUPT. */
    public void t5rupt() {
        saveAL();
        int a = Agc.add(~agc.get("TIME5") & Agc.WORD_MASK, agc.get(".5SEC"));
        t5ruptBody(a);
    }

    /** T5RUPT: if A <= 0 (no overflow of the clock) resume at NOQBRSM, else DTCB through T5LOC. */
    private void t5ruptBody(int a) {
        int s = Agc.toSigned(a);
        if (s <= 0) {
            new Resume(agc).noqbrsm();
        } else {
            // EXTEND; DCA T5LOC; DTCB -- jump through the 2CADR in T5LOC
        }
    }

    /** Vector 4014: T3RUPT. */
    public void t3rupt() { saveAL(); new T3Rupt(agc).t3rupt(); }

    /** Vector 4020: T4RUPT. */
    public void t4rupt() { saveAL(); new T4Rupt(agc).t4rupt(); }

    /** Vector 4024: KEYRUPT1. */
    public void keyrupt1() { saveAL(); new KeyRupt(agc).keyrupt1(); }

    /** Vector 4030: KEYRUPT2 (mark). */
    public void keyrupt2() { saveAL(); new KeyRupt(agc).markrupt(); }

    /** Vector 4034: UPRUPT. */
    public void uprupt() { saveAL(); new KeyRupt(agc).uprupt(); }

    /** Vector 4040: DOWNRUPT. */
    public void downrupt() { saveAL(); new Downrupt(agc).dodowntm(); }

    /** Vector 4044: RADAR RUPT. */
    public void radarRupt() { saveAL(); new RadarRupt(agc).vhfread(); }

    /** Vector 4050: HAND CONTROL RUPT (NOT USED): TCF RESUME +3. */
    public void handControlRupt() { saveAL(); new Resume(agc).resume3(); }

    /** DXCH ARUPT: save A and L in the interrupt save registers. */
    private void saveAL() {
        arupt = agc.get("A");
        lrupt = agc.get("L");
    }

    public int arupt() { return arupt; }
    public int lrupt() { return lrupt; }
    public int bbank() { return bbank; }

    // Placeholder neighbours (resolved elsewhere in the translation).
    private static final class Goprog { Goprog(Agc a) {} void goprog() {} }
    private static final class Resume { Resume(Agc a) {} void noqbrsm() {} void resume3() {} }
    private static final class T3Rupt { T3Rupt(Agc a) {} void t3rupt() {} }
    private static final class T4Rupt { T4Rupt(Agc a) {} void t4rupt() {} }
    private static final class KeyRupt { KeyRupt(Agc a) {} void keyrupt1() {} void markrupt() {} void uprupt() {} }
    private static final class Downrupt { Downrupt(Agc a) {} void dodowntm() {} }
    private static final class RadarRupt { RadarRupt(Agc a) {} void vhfread() {} }
}
