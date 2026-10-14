package apollo.comanche055;

import apollo.agc.Agc;

/**
 * P76 - TARGET DELTA V PROGRAM (Comanche055, pp 511-513).
 *
 * Flashes V06N84 (delta V, LV) and V06N33 (TIG) for the astronaut, integrates the other
 * vehicle state vector to TIG, adds DELTA V (transformed LV -> REF COSYS) and then
 * integrates it to the state vector of this vehicle (INTEGRVS). Exits via ENDOFJOB.
 */
public final class P76 {
    public static final int V06N84 = 684;
    public static final int V06N33 = 633;

    private final Agc agc;

    public P76(Agc agc) {
        this.agc = agc;
    }

    /** P76: entry via V37E76E. */
    public void p76() {
        agc.set("TRACKFLG", 1);                           // TC UPFLAG / ADRES TRACKFLG
        if (!flash(V06N84)) { endp76(); return; }         // flash last delta V; TERMINATE -> ENDP76
        if (!flash(V06N33)) { endp76(); return; }         // flash last TIG
        // Interpretive: D(MPAC) = TIG (CSEC B28); TDEC1 = TIG; call OTHPREC
        agc.set("TDEC1", agc.get("TIG"));
        // STCALL TDEC1 / OTHPREC
        othprec();
        compmat();
    }

    /** Flash a verb/noun pair and wait; true on PROCEED, repeats on ENTER (store data), false on TERMINATE. */
    private boolean flash(int vn) {
        return true; // BANKCALL GOFLASH: TERMINATE -> false; PROCEED -> true; ENTER -> repeat flashing
    }

    private void othprec() {
        // CSMPREC or LEMPREC depending on which vehicle is "other" (placeholder for OTHPREC in INTEGRATION_INITIALIZATION)
        new IntegrationInitialization(agc).othprec();
    }

    /**
     * COMPMAT: build the LV-to-REF transformation and update the other vehicle state.
     * Uses RATT, VATT, DELVOV; scratch vectors 24D, 18D, 12D, 6 (PUSH list).
     */
    public void compmat() {
        double[] ratt = vec("RATT");
        double[] vatt = vec("VATT");
        double[] delvov = vec("DELVOV");
        double[] u24 = unit(neg(ratt));                       // U(-R)
        double[] u18 = unit(cross(u24, vatt));                // U(-R) x V = U(V x R)
        double[] u12 = cross(u18, u24);                       // U(V x R) x U(-R)
        // V(MPAC) = DELTA V in REFCOSYS = delvov * [12D 18D 24D], VSL1
        double[] dvRef = {
            2 * (delvov[0] * u12[0] + delvov[1] * u18[0] + delvov[2] * u24[0]),
            2 * (delvov[0] * u12[1] + delvov[1] * u18[1] + delvov[2] * u24[1]),
            2 * (delvov[0] * u12[2] + delvov[1] * u18[2] + delvov[2] * u24[2])};
        double[] v6 = {vatt[0] + dvRef[0], vatt[1] + dvRef[1], vatt[2] + dvRef[2]};
        callIntstall();                                       // prevent orbital integ from interfering
        p76sub1();
        // VCV = v6 >> (0,2); RCV = RATT >> (0,2); TET = TIG
        store("VCV", v6);
        store("RCV", ratt);
        agc.set("TET", agc.get("TIG"));
        agc.set("INTYPFLG", 0);                               // CLEAR INTYPFLG
        intothis();
    }

    /** INTOTHIS: TDEC1 = TETTHIS, integrate with INTEGRVS, update other vehicle. */
    public void intothis() {
        agc.set("TDEC1", agc.get("TETTHIS"));
        new IntegrationInitialization(agc).integrvs();
        callIntstall();
        p76sub1();                                            // set/clear MOONFLAG
        agc.set("RRECT", agc.get("RATT1"));
        agc.set("RCV", agc.get("RATT1"));
        agc.set("TET", agc.get("TAT"));
        // VATT1 -> MINIRECT (rectify other vehicle state)
        new IntegrationInitialization(agc).minirect();
        // EXIT; TC PHASCHNG; OCT 04024
        agc.set("REINTFLG", 1);                               // UPFLAG REINTFLG
        // CALL ATOPOTH (store state in other-vehicle erasables), then INTWAKE1 via BANKCALL
        new IntegrationInitialization(agc).atopoth();
        new IntegrationInitialization(agc).intwake1();
        endp76();
    }

    /** ENDP76: clear RR tracking mark counter and invalidate mark buffer; go to POOH. */
    public void endp76() {
        agc.set("MARKCTR", 0);
        agc.set("VHFCNT", 0);
        agc.set("MRKBUF2", Agc.fromSigned(-1));               // NEGONE
        // TCF GOTOPOOH
    }

    /** P76SUB1: clear MOONFLAG, then set it if X2 != 0 (X2 = 2 means set). */
    public void p76sub1() {
        agc.set("MOONFLAG", 0);
        if (agc.get("X2") != 0) {
            agc.set("MOONFLAG", 1);
        }
    }

    private void callIntstall() {
        // CALL INTSTALL
    }

    private double[] vec(String name) {
        return new double[] {agc.get(name), agc.get(name + "+2"), agc.get(name + "+4")};
    }

    private void store(String name, double[] v) {
        agc.set(name, (int) v[0]);
        agc.set(name + "+2", (int) v[1]);
        agc.set(name + "+4", (int) v[2]);
    }

    private static double[] neg(double[] a) {
        return new double[] {-a[0], -a[1], -a[2]};
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }

    private static double[] unit(double[] a) {
        double n = Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        return n == 0 ? a : new double[] {a[0] / n, a[1] / n, a[2] / n};
    }
}
