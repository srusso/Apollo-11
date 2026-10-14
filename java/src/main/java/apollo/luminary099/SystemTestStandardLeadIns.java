package apollo.luminary099;

import java.util.function.Supplier;

import apollo.agc.Agc;

/**
 * SYSTEM_TEST_STANDARD_LEAD_INS.agc (Luminary 099, pp. 370-372).
 * Programs easing erasable-memory programs: E/BKCALL (bank calls from and to erasable),
 * E/CALL (calling fixed-memory interpretive subroutines from erasable) and E/JOBWAK
 * (waking erasable-memory jobs from interrupt).
 */
public class SystemTestStandardLeadIns {
    private final Agc agc;

    public SystemTestStandardLeadIns(Agc agc) {
        this.agc = agc;
    }

    /** Which of the three return slots following the double CADR is taken. */
    public enum Return { TERMINATE_OR_BAD_STALL, PROCEED_OR_GOOD_STALL, ENTER_OR_RECYCLE }

    /**
     * E/BKCALL: bank call from and returning to erasable. A and L are preserved.
     * Uses EBUF2 and EBUF2+1 (unswitched, unshared by interrupts) to hold the DP return.
     */
    public Return eBkcall(Supplier<Return> routine) {
        // DXCH BUF2 / DXCH EBUF2: save A,L and the DP return.
        // INCR EBUF2: return +1 because of the double CADR.
        agc.set("EBUF2", Agc.add(agc.get("EBUF2"), 1));
        // CA BBANK / MASK LOW10 / ADS EBUF2+1: form BBCON from the current EBANK.
        agc.set("EBUF2+1", Agc.add(agc.get("EBUF2+1"), agc.get("BBANK") & 0x3FF));
        // NDX EBUF2 / CA 0 -1 / TC SWCALL: go to the routine, Q set to SWRETURN.
        Return r = routine.get();
        if (r == Return.ENTER_OR_RECYCLE) {
            // INCR EBUF2 twice for the enter/recycle return.
            agc.set("EBUF2", Agc.add(agc.get("EBUF2"), 2));
        }
        eSwitch();
        return r;
    }

    /** E/SWITCH: DXCH EBUF2; DTCB - return to the saved caller. */
    public void eSwitch() {
        // Exchange the saved DP return with A,L and perform the double TC (bank switch return).
    }

    /**
     * E/CALL: call a fixed-memory interpretive subroutine from erasable and return to
     * erasable. The CADR follows the RTB; the subroutine must leave via RVQ or equivalent.
     */
    public void eCall(Runnable interpretiveRoutine) {
        // LXCH LOC / CA of CADR / INCR L twice: CADR in A, return address in L.
        // DXCH EBUF2: store CADR and return.
        interpretiveRoutine.run(); // CALL EBUF2 (indirect)
        // EXIT; LXCH EBUF2+1; TCF INTPRET +2: pick up the return and set LOC.
    }

    /**
     * E/JOBWAK: wake an erasable-memory job. Must be called in interrupt or with interrupts
     * inhibited. The address of the sleeping job arrives in A.
     */
    public void eJobwak(int jobAddress) {
        // TC JOBWAKE
        // CS BIT11 / NDX LOCCTR / ADS LOC: knock the fixed-memory bit out of the address.
        agc.set("LOC", Agc.add(agc.get("LOC"), Agc.fromSigned(-0x400)));
        // TC RUPTREG3: return.
    }
}
