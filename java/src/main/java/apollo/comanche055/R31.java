package apollo.comanche055;

import apollo.agc.Agc;

/**
 * R31 / R34 - Rendezvous parameter display routines (Comanche055, pp 505-510).
 *
 * R31CALL starts a displayed-noun job (V83) that repeatedly extrapolates CM and LM state
 * vectors (STATEXTP), computes range (RANGE), range rate (RRATE) and angle (RTHETA)
 * and displays them via V16N54 (R31) or V16N53 (R34).
 */
public final class R31 {
    public static final int V16N54 = 1654;
    public static final int V16N53 = 1653;
    private static final int BIT12 = 02000;
    private static final int BIT5 = 020;
    private static final int BIT4 = 010;

    private final Agc agc;

    public R31(Agc agc) {
        this.agc = agc;
    }

    /** R31CALL: start V83CALL job at priority PRIO3 (FINDVAC), then loop display. */
    public void r31call() {
        // CAF PRIO3; TC FINDVAC; 2CADR V83CALL  (schedules v83call())
        dspdelay();
    }

    /** DSPDELAY: wait 1 sec until the display request (EXTVBACT bit 12) is active. */
    public void dspdelay() {
        do {
            // CAF 1SEC; BANKCALL DELAYJOB
        } while ((agc.get("EXTVBACT") & BIT12) == 0);
        dispn5x();
    }

    /** DISPN5X: choose noun 54 (R31) or noun 53 (R34) per R31FLAG (FLAGWRD9 bit 4) and display with GOMARKF. */
    public void dispn5x() {
        int vn = ((agc.get("FLAGWRD9") & BIT4) != 0) ? V16N54 : V16N53;
        // BANKCALL GOMARKF(vn): returns TERMINATE -> B5OFF, PROCEED -> B5OFF, else repeat
        // TC B5OFF twice => exit; ENTER/other => TCF DISPN5X
    }

    /** V83: integrate state vectors starting at HAVEBASE. */
    public void v83() {
        haveBase();
    }

    /** V83CALL: extrapolate state vectors starting at STATEXTP. */
    public void v83call() {
        statextp();
    }

    /**
     * COMPDISP: compute RANGE, RRATE, RTHETA from RATT/VATT (LM) and RONE/VONE (CM).
     */
    public void compdisp() {
        double[] r = sub(vec("RATT"), vec("RONE"));            // RATT - RONE
        double range = abs(r);
        agc.set("RANGE", (int) range);                         // METERS B-29
        double[] los = unit(r);
        double[] dv = sub(vec("VATT"), vec("VONE"));
        double rrate = 2 * dot(dv, los);                       // SL1; range rate M/CS B-7
        agc.set("RRATE", (int) rrate);
        // CDUTRIG initialises for *NBSM*
        r34los();                                              // NOTE: PDL must = 0
    }

    /** R34ANG: compute RTHETA, the angle between UR and the plane of the thrusting axis. */
    public void r34ang() {
        double[] ur = unit(vec("RONE"));
        double[] axis = vec("THISAXIS");                       // UNITX for CM, UNITZ for LM
        boolean r31 = (agc.get("FLAGWRD9") & BIT4) != 0;       // R31FLAG: ON = R31 THETA, OFF = R34 PHI
        double[] v = r31 ? axis : vec("12D");                  // R31: THISAXIS, R34: 12D (from SXTNB)
        double[] uxorz = mxv(vec("REFSMMAT"), nbsm(v));        // *NBSM*, then VXM REFSMMAT
        double[] proj = scale(vproj(ur, uxorz), 4);            // VPROJ, VSL2
        double[] up = unit(sub(uxorz, proj));                  // BVSU, UNIT: UP/2
        double[] cr = unit(cross(unit(cross(vec("RONE"), vec("VONE"))), vec("RONE")));
        double sign = Math.signum(dot(cr, uxorz));
        double rtheta = Math.acos(clamp(dot(scale(up, 2), uxorz) * sign));
        if (dot(vec("RONE"), uxorz) < 0) {
            rtheta = 1 - rtheta;                               // DPPOSMAX - RTHETA, between 0 and 1 rev
        }
        agc.set("RTHETA", (int) rtheta);
        // EXIT: test if display request answered
        if ((agc.get("EXTVBACT") & BIT5) == 0) {
            return;                                            // BZF ENDEXT, die
        }
        agc.set("EXTVBACT", agc.get("EXTVBACT") | BIT12);      // CS EXTVBACT; MASK BIT12; ADS
        v83();
    }

    /**
     * STATEXTP: initial precision extrapolation of the LM state vector to present time (or PIPTIME
     * if AVG is on, V37FLAG set), saved as base vector; CM likewise (CSMPREC).
     */
    public void statextp() {
        boolean avgOn = agc.get("V37FLAG") != 0;
        if (avgOn) {
            getrvn();
        } else {
            agc.set("BASETIME", agc.get("LOADTIME"));          // RTB LOADTIME
        }
        agc.set("TDEC1", agc.get("BASETIME"));
        new IntegrationInitialization(agc).lemprec();
        copy("RATT1", "BASEOTP");
        copy("VATT1", "BASEOTV");
        if (avgOn) {
            compdisp();                                        // RONE(RN) VONE(VN) with RATT/VATT (LEMPREC)
            return;
        }
        agc.set("TDEC1", agc.get("TAT"));
        new IntegrationInitialization(agc).csmprec();
        copy("RATT1", "BASETHP");
        copy("VATT1", "BASETHV");
        haveBase();
    }

    /** HAVEBASE: subsequent integrations; integrate CM from base vector to current time. */
    public void haveBase() {
        if (agc.get("V37FLAG") != 0) {
            getrvn5();
            return;
        }
        agc.set("TDEC1", agc.get("LOADTIME"));
        // CALL INTSTALL
        agc.set("MOONFLAG", 0);                                // CLEAR MOONFLAG
        copy("BASETHP", "RCV");
        copy("BASETHV", "VCV");
        agc.set("TDEC1", agc.get("BASETIME"));
        if (agc.get("MOONTHIS") != 0) {
            agc.set("MOONFLAG", 1);
        }
        agc.set("INTYPFLG", 0);                                // conic unless LM down
        if (agc.get("SURFFLAG") != 0) {
            agc.set("INTYPFLG", 1);                            // precision if LM down
        }
        agc.set("TET", agc.get("TDEC1"));
        new IntegrationInitialization(agc).integrvs();         // integration at last
        copy("RATT", "RONE");
        copy("VATT", "VONE");
        agc.set("TDEC1", agc.get("TAT"));
        if (agc.get("SURFFLAG") != 0) {
            getrvn6();                                         // LEMPREC if LM down
            return;
        }
        // CALL INTSTALL; SET INTYPFLG (conic if not down)
        agc.set("INTYPFLG", 1);
        othint();
    }

    /** OTHINT: entered if AVG on, to integrate LM from base other-vehicle vector. */
    public void othint() {
        agc.set("MOONFLAG", 0);
        copy("BASEOTP", "RCV");
        copy("BASEOTV", "VCV");
        agc.set("TDEC1", agc.get("BASETIME"));
        if (agc.get("MOONTHIS") != 0) {
            agc.set("MOONFLAG", 1);
        }
        agc.set("TET", agc.get("TDEC1"));
        new IntegrationInitialization(agc).integrvs();
        compdisp();
    }

    /** GETRVN5: AVG on. */
    public void getrvn5() {
        getrvn();
        if (agc.get("SURFFLAG") != 0) {
            getrvn6();                                         // LM down, LEMPREC
            return;
        }
        // CALL INTSTALL
        agc.set("INTYPFLG", 0);
        othint();
    }

    /** GETRVN6: precise LM extrapolation then display computation. */
    public void getrvn6() {
        agc.set("TDEC1", agc.get("TDEC1"));
        new IntegrationInitialization(agc).lemprec();
        compdisp();
    }

    /** GETRVN: AVG on, RONE = RN, VONE = VN, use PIPTIME. */
    public void getrvn() {
        copy("RN", "RONE");
        copy("VN", "VONE");
        agc.set("MPAC", agc.get("PIPTIME"));
    }

    /** R34LOS (bank R34): save CDUS/CDUT in the pushlist, call SXTNB, continue at R34ANG. */
    public void r34los() {
        agc.set("9D", agc.get("CDUS"));
        agc.set("11D", agc.get("CDUT"));
        agc.set("X1", Agc.fromSigned(-(agc.get("FIXLOC") + 6)));
        // CALL SXTNB (sextant navigation base); STCALL 12D -> R34ANG
        r34ang();
    }

    // --- small vector helpers (floating-point stand-ins for the interpreter) ---
    private void copy(String from, String to) {
        for (int i = 0; i < 6; i += 2) {
            agc.set(i == 0 ? to : to + "+" + i, agc.get(i == 0 ? from : from + "+" + i));
        }
    }

    private double[] vec(String n) {
        return new double[] {agc.get(n), agc.get(n + "+2"), agc.get(n + "+4")};
    }

    private static double clamp(double x) {
        return Math.max(-1, Math.min(1, x));
    }

    private static double[] sub(double[] a, double[] b) {
        return new double[] {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    private static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    private static double abs(double[] a) {
        return Math.sqrt(dot(a, a));
    }

    private static double[] scale(double[] a, double k) {
        return new double[] {a[0] * k, a[1] * k, a[2] * k};
    }

    private static double[] unit(double[] a) {
        double n = abs(a);
        return n == 0 ? a : scale(a, 1 / n);
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }

    private static double[] vproj(double[] a, double[] u) {
        return scale(u, dot(a, u));
    }

    private static double[] mxv(double[] m, double[] v) {
        return v; // VXM REFSMMAT (matrix is held in erasable; identity placeholder)
    }

    private static double[] nbsm(double[] v) {
        return v; // *NBSM*: navigation base to stable member transform (supplied by CDU angles)
    }
}
