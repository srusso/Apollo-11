package apollo.comanche055;

import apollo.agc.Agc;

/**
 * SYSTEM_TEST_STANDARD_LEAD_INS.agc (Comanche 055, pp. 420-422).
 * Programs easing erasable-memory programs (E/BKCALL, E/CALL, E/JOBWAK) and system-test lead-ins.
 */
public class SystemTestStandardLeadIns {
    private final Agc agc;

    public SystemTestStandardLeadIns(Agc agc) {
        this.agc = agc;
    }

    /** Return-slot selector for {@link #eBkcall}. */
    public enum Return { TERMINATE_OR_BAD, PROCEED_OR_GOOD, ENTER_OR_RECYCLE }

    /** E/BKCALL: bank-call from/to erasable. A and L preserved. EBUF2 holds the saved DP return. */
    public Return eBkcall(java.util.function.Supplier<Return> routine) {
        // DXCH BUF2 / DXCH EBUF2; INCR EBUF2 (double CADR); form BBCON from EBANK; call routine via SWCALL.
        agc.set("EBUF2", Agc.add(agc.get("EBUF2"), 1));
        Return r = routine.get();
        // TC +4 (terminate/bad), TC +2 (proceed/good), INCR EBUF2 twice (enter/recycle)
        if (r == Return.ENTER_OR_RECYCLE) {
            agc.set("EBUF2", Agc.add(agc.get("EBUF2"), 2));
        }
        eSwitch();
        return r;
    }

    /** E/SWITCH: DXCH EBUF2; DTCB (return to saved caller). */
    public void eSwitch() {
        // exchange EBUF2 with A,L and perform double-TC bank return
    }

    /** E/CALL: call a fixed-memory interpretive subroutine from erasable; returns in interpretive. */
    public void eCall(Runnable interpretiveRoutine) {
        // LXCH LOC ... DXCH EBUF2 ; CALL EBUF2 ; EXIT ; LXCH EBUF2+1 ; TCF INTPRET+2
        interpretiveRoutine.run();
    }

    /** E/JOBWAK: wake an erasable-memory job; call in interrupt or with INHINT. ADRES in A. */
    public void eJobwak(int wakeAdr) {
        // TC JOBWAKE; CS BIT11; NDX LOCCTR; ADS LOC (knock fixed-memory bit out of ADRES); TC RUPTREG3
        int loc = agc.get("LOC");
        agc.set("LOC", Agc.add(loc, Agc.fromSigned(-0x400)));
    }

    /** COMPVER: must be at 33,2000. */
    public void compver() { /* TC GCOMPVER */ }

    /** GTSCPSS1: must be at 33,2001. */
    public void gtscpss1() { /* TC GTSCPSS */ }

    /** REDO: display MM 07, falls into IMUTEST. */
    public void redo() {
        // TC NEWMODEX ; MM 07
        // FALL INTO IMUTEST
    }
}
