package apollo.luminary099;

import apollo.agc.Agc;

/**
 * R31.agc - Rendezvous Navigation Routine R31 (V83 "RENDEZVOUS PARAMETER DISPLAY").
 * Extrapolates the LM and CSM state vectors to a common time and displays range (RANGE),
 * range rate (RRATE) and theta (RTHETA) with Noun 54 (V16N54).
 */
public class R31 {
    private final Agc agc;

    /** V16N54: verb 16 noun 54 display. */
    public static final int V16N54 = 1654;
    public static final int PRIO3 = 3;
    public static final int PRIO5 = 5;
    public static final int PRIO22 = 22;

    public R31(Agc agc) {
        this.agc = agc;
    }

    private boolean flag(String name) {
        return agc.get(name) != 0;
    }

    /** R31CALL: start the V83 job (FINDVAC V83CALL), then wait for the display bit. */
    public void r31Call() {
        // CAF PRIO3 ; TC FINDVAC ; 2CADR V83CALL  -> schedule v83Call()
        // DSPDELAY: FIXDELAY 100 until EXTVBACT bit 12 set
        while ((agc.get("EXTVBACT") & 0x800) == 0) {
            // TC FIXDELAY ; DEC 100  (1 second delay)
        }
        // CAF PRIO5 ; TC NOVAC ; 2CADR DISPN5X
        dispN5X();
        // TCF TASKOVER
    }

    /** DISPN5X: flash display V16N54 until terminated; B5OFF between responses. */
    public void dispN5X() {
        // CAF V16N54 ; BANKCALL GOMARKF  (returns on terminate/proceed/enter)
        ServiceRoutines sr = new ServiceRoutines(agc);
        sr.b5Off(); // TC B5OFF (terminate)
        sr.b5Off(); // TC B5OFF (proceed)
        // TCF DISPN5X (enter: redisplay)
    }

    /** V83CALL: decide which base vectors are needed. */
    public void v83Call() {
        if (flag("AVEGFLAG")) {          // AVERAGE G ON
            if (flag("MUNFLAG")) {       // MUNG?: MUNFLAG set -> CSM base not needed
                getRvn();
            } else {
                oneBase();
            }
            return;
        }
        if (flag("SURFFLAG")) {          // on surface: bypass LEMPREC
            oneBase();
            return;
        }
        // Extrapolate both state vectors
        agc.set("TDEC1", loadTime());
        // CALL LEMPREC: precision base vector for LM
        lemPrec();
        VecOps.setV(agc, "BASETHP", VecOps.getV(agc, "RATT1"));
        VecOps.setV(agc, "BASETHV", VecOps.getV(agc, "VATT1"));
        docmBase(agc.get("TAT"));
    }

    /** ONEBASE: get the CSM base vector only. */
    private void oneBase() {
        docmBase(loadTime());
    }

    /** DOCMBASE: precision base vector for CM. */
    private void docmBase(int time) {
        agc.set("BASETIME", time);
        agc.set("TDEC1", time);
        csmPrec();
        VecOps.setV(agc, "BASEOTP", VecOps.getV(agc, "RATT1"));
        VecOps.setV(agc, "BASEOTV", VecOps.getV(agc, "VATT1"));
        // EXIT; falls into REV83
        rev83();
    }

    /** REV83: compute the extrapolated vectors, display; restart if V83 is answered with Enter. */
    public void rev83() {
        do {
            if (flag("AVEGFLAG")) {
                getRvn();                       // use RN, VN
            } else if (flag("SURFFLAG")) {
                r31Surf();                      // on surface: use LEMPREC
            } else {
                // conic extrapolation for both vehicles
                agc.set("TDEC1", loadTime());
                intStall();
                agc.set("MOONFLAG", flag("MOONTHIS") ? 1 : 0);
                VecOps.setV(agc, "RCV", VecOps.getV(agc, "BASETHP"));
                VecOps.setV(agc, "VCV", VecOps.getV(agc, "BASETHV"));
                agc.set("INTYPFLG", 1); // conic
                agc.set("TET", agc.get("BASETIME"));
                integrvs();
                othConic();
            }
            // check whether terminated (bit 5 of EXTVBACT)
            if ((agc.get("EXTVBACT") & 0x10) == 0) {
                return; // ENDEXT
            }
            agc.set("EXTVBACT", agc.get("EXTVBACT") | 0x800); // set bit 12; start again
        } while (true);
    }

    /** OTHCONIC: conic extrapolation of the other vehicle. */
    private void othConic() {
        VecOps.setV(agc, "RONE", VecOps.getV(agc, "RATT"));
        VecOps.setV(agc, "VONE", VecOps.getV(agc, "VATT"));
        intStall();
        agc.set("INTYPFLG", 1);
        othInt(agc.get("TAT"));
    }

    /** OTHINT: precision (or conic) integration of the other vehicle, then COMPDISP. */
    private void othInt(int tdec) {
        agc.set("TDEC1", tdec);
        agc.set("MOONFLAG", flag("MOONTHIS") ? 1 : 0);
        VecOps.setV(agc, "RCV", VecOps.getV(agc, "BASEOTP"));
        VecOps.setV(agc, "VCV", VecOps.getV(agc, "BASEOTV"));
        agc.set("TET", agc.get("BASETIME"));
        integrvs();
        compDisp(VecOps.getV(agc, "RATT"), VecOps.getV(agc, "VATT"));
    }

    /**
     * COMPDISP: compute RANGE (scaled 2^29 m), RRATE (scaled 2^7 m/cs) and RTHETA, the angle
     * between the LOS projection and the unit Z of the stable member.
     */
    private void compDisp(double[] rcm, double[] vcm) {
        double[] rone = VecOps.getV(agc, "RONE");
        double[] vone = VecOps.getV(agc, "VONE");
        double[] los = VecOps.sub(rcm, rone);
        double range = VecOps.abs(los);
        agc.set("RANGE", 0);
        VecOps.setDp(agc, "RANGE", range);          // scaled 2(29) m
        double[] uLos = VecOps.unit(los);
        double rrate = VecOps.dot(VecOps.sub(vcm, vone), uLos);
        VecOps.setDp(agc, "RRATE", rrate);          // scaled 2(7) m/cs
        double[] ur = VecOps.unit(rone);
        double[] uz = VecOps.vxm(new double[] {0, 0, 1}, VecOps.getM(agc, "REFSMMAT")); // CDU*NBSM / REFSMMAT
        // UNIT(P) = UNIT(UZ - (UZ) PROJ (UR))
        double k = VecOps.dot(uz, ur);
        double[] p = VecOps.unit(VecOps.sub(uz, VecOps.scale(ur, k)));
        double[] rxv = VecOps.cross(ur, vone);
        double sign = VecOps.dot(VecOps.cross(rxv, ur), p) >= 0 ? 1 : -1;
        double rtheta = Math.acos(Math.max(-1, Math.min(1, VecOps.dot(p, uz) * sign))) / (2 * Math.PI);
        if (VecOps.dot(ur, uz) < 0) {
            rtheta = 1 - rtheta;                    // DPPOSMAX - RTHETA
        }
        VecOps.setDp(agc, "RTHETA", rtheta);
    }

    /** GETRVN: use the LM state vector (RN, VN) and CSM (R(CSM), V(CSM)) at PIPTIME. */
    private void getRvn() {
        // CA PRIO22 ; TC PRIOCHNG  (inhibit servicer)
        VecOps.setV(agc, "RONE", VecOps.getV(agc, "RN"));
        VecOps.setV(agc, "VONE", VecOps.getV(agc, "VN"));
        double[] rcsm = VecOps.getV(agc, "R(CSM)");
        double[] vcsm = VecOps.getV(agc, "V(CSM)");
        // CA PRIO3 ; TC PRIOCHNG
        if (!flag("MUNFLAG")) {
            getRvn2();                // CM delta precision extrapolation
            return;
        }
        double[][] ref = VecOps.getM(agc, "REFSMMAT");
        // VXM VSR4 (R scaled), VXM VSL1 (V rescaled)
        double[] r = VecOps.scale(VecOps.vxm(rcsm, ref), 1.0 / 16);
        double[] v = VecOps.scale(VecOps.vxm(vcsm, ref), 2.0);
        compDisp(r, v);
    }

    /** GETRVN2: precision extrapolation for the other vehicle. */
    private void getRvn2() {
        intStall();
        agc.set("INTYPFLG", 0);
        othInt(agc.get("PIPTIME"));
    }

    /** R31SURF: LM on surface; precision integration uses the planetary inertial orientation. */
    private void r31Surf() {
        agc.set("TDEC1", loadTime());
        lemPrec();
        othConic();
    }

    // ---- external routines (other files) ----
    private int loadTime() { return agc.get("TIME2"); }              // RTB LOADTIME
    private void lemPrec() { new Integration(agc).lemPrec(); }
    private void csmPrec() { new Integration(agc).csmPrec(); }
    private void intStall() { new Integration(agc).intStall(); }
    private void integrvs() { new Integration(agc).integrvs(); }

    /** Thin forwarding stubs to the orbital integration routines (separate source files). */
    private static final class Integration {
        private final Agc agc;
        Integration(Agc agc) { this.agc = agc; }
        void lemPrec() { new IntegrationInitialization(agc).lemprec(); }
        void csmPrec() { new IntegrationInitialization(agc).csmprec(); }
        void intStall() { new IntegrationInitialization(agc).intstall(); }
        void integrvs() { new IntegrationInitialization(agc).integrvs(); }
    }
}
