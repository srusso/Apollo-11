package apollo.luminary099;

import apollo.agc.Agc;

/**
 * P76 - Target delta V program (Luminary 099). Lets the astronaut enter a delta V (VG) and TIG,
 * then updates the state vector by applying the impulsive delta V (in REFCOSYS) and re-integrating.
 */
public class P76 {
    private final Agc agc;

    /** Flash displays: V06 N84 (delta V of other vehicle), V06 N33 (TIG). */
    public static final String V06N84 = "0684";
    public static final String V06N33 = "0633";

    public P76(Agc agc) {
        this.agc = agc;
    }

    /** Result of a keyboard flash: terminate, proceed, or enter-data (repeat). */
    public enum FlashResult { TERMINATE, PROCEED, ENTER }

    /** Hook for GOFLASH via BANKCALL; supplied by the surrounding system. */
    public java.util.function.Function<String, FlashResult> goFlash = code -> FlashResult.PROCEED;

    /** P76: main entry. */
    public void p76() {
        upFlag("TRACKFLG");                 // TC UPFLAG / ADRES TRACKFLG: set track flag
        // VLOAD DELVLVC ; STORE DELVOV
        agc.set("DELVOV", agc.get("DELVLVC"));
        FlashResult r;
        do {                                // flash last delta V, repeat on data entry
            r = goFlash.apply(V06N84);
            if (r == FlashResult.TERMINATE) { endP76(); return; }
        } while (r == FlashResult.ENTER);
        do {                                // flash TIG
            r = goFlash.apply(V06N33);
            if (r == FlashResult.TERMINATE) { endP76(); return; }
        } while (r == FlashResult.ENTER);
        // DLOAD TIG ; STCALL TDEC1 ; OTHPREC
        agc.set("TDEC1", agc.get("TIG"));
        othPrec();
        compMat();
    }

    /** COMPMAT: build REFCOSYS matrix from U(-R), U(VxR), U((RxV)xR), apply delta V, integrate. */
    public void compMat() {
        // 24D = unit(-RATT); 18D = unit(unit(-R) x VATT); 12D = unit(18D x 24D)
        // V(MPAC) = DELVOV * [12D matrix] (VSL1); VATT + that -> V(PD6)
        // INTSTALL; P76SUB1 (select sphere of influence, X2 = 0 or 2)
        int x2 = p76Sub1();
        // VCV = V(PD6) >> 0,2 ; RCV = RATT >> 0,2 ; TET = TIG ; clear INTYPFLG ; TDEC1 = TETTHIS
        agc.set("TET", agc.get("TIG"));
        agc.set("TDEC1", agc.get("TETTHIS"));
        intOThis();
    }

    /** INTOTHIS: integrate the other vehicle's state to TDEC1, then rectify and finish. */
    public void intOThis() {
        // CALL INTEGRVS ; CALL INTSTALL
        // RRECT = RATT1 ; RCV = RRECT ; TET = TAT ; VATT1 -> MINIRECT
        agc.set("RRECT", agc.get("RATT1"));
        agc.set("TET", agc.get("TAT"));
        // CALL MINIRECT ; PHASCHNG OCT 04024 ; set REINTFLG ; CALL ATOPOTH
        upFlag("REINTFLG");
        // SSP QPRET OUT ; BANKCALL INTWAKE1 (permit orbital integration use)
        out();
    }

    /** OUT: falls through to ENDP76. */
    public void out() {
        endP76();
    }

    /** ENDP76: clear radar mark counter and go to P00 (GOTOPOOH). */
    public void endP76() {
        agc.set("MARKCTR", 0);
        // TCF GOTOPOOH
    }

    /** P76SUB1: returns X2 = 2 if the Moon is the sphere of influence (MOONFLAG or CMOONFLG), else 0 and clears MOONFLAG. */
    public int p76Sub1() {
        if (agc.get("MOONFLAG") != 0 || agc.get("CMOONFLG") != 0) {
            agc.set("MOONFLAG", 1);
            return 2;
        }
        agc.set("MOONFLAG", 0);
        return 0;
    }

    private void othPrec() { /* CALL OTHPREC: other-vehicle precision integration */ }

    private void upFlag(String flag) { agc.set(flag, 1); }
}
