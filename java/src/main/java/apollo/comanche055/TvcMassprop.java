package apollo.comanche055;

import apollo.agc.Agc;

/**
 * TVCMASSPROP (Comanche 055): computes TVC/RCS DAP mass properties (moments of inertia
 * IXX0..IXX2 and WEIGHT/G) from CSM/LEM mass, using piecewise-linear break-point tables.
 * Erasable symbols accessed through {@link Agc}: DAPDATR1, LEMMASS, CSMMASS, VARST0..,
 * IXX, IXX+1, IXX+2, WEIGHT/G, PHI333, PSI333, TEMP333.
 */
public final class TvcMassprop {
    private static final int BIT13 = 010000;
    private static final int BIT14 = 020000;

    // Table values (physical value; original scale in comment).
    /** NOLEMVAL: break-point values with no LEM attached. */
    static final double[] NOLEMVAL = {
        25445.0,       // B-20
        87450.0,       // B-20
        .30715,        // B-2
        1.22877e-5,    // B+12
        1.6096,        // B-6
        1.54,          // B-6
        7.77177,       // B-6
        3.46458e-5,    // B+12
    };
    /** INTVALUE: intercepts for LEM attached. */
    static final double[] INTVALUE = {
        26850.0, 127518.0, .54059, .153964e-4, -.742923, 1.5398, 9.68, .647625e-4,
        -27228.0, -.206476,
    };
    /** SLOPEVAL: slopes versus LEM mass. */
    static final double[] SLOPEVAL = {
        1.96307, 27.5774, 2.3548e-5, 2.1777e-9, 1.044e-3, 0, 2.21068e-3, 1.5166e-9,
        -1.284, 2e-5,
    };
    /** NEGBPW: CSM break-point weight, -15402.17 B-16. */
    static final double NEGBPW = -15402.17;
    /** DXITFIX: -1.88275E-5 B+12 (double precision). */
    static final double DXITFIX = -1.88275e-5;

    private final Agc agc;
    private final double[] varst = new double[10 + 8]; // VARST0 .. (slopes at VARST5.. area)

    public TvcMassprop(Agc agc) {
        this.agc = agc;
    }

    /** MASSPROP: entry. Builds VARST0..9 then computes IXX0..2 and WEIGHT/G. */
    public void massprop() {
        boolean lemAttached = (agc.get("DAPDATR1") & BIT13) != 0; // LEMTEST
        double lemMass = agc.get("LEMMASS");
        double[] v = new double[10];
        for (int i = 9; i >= 0; i--) { // ten passes, PHI333 = 9..0
            if (!lemAttached) {
                v[i] = i < NOLEMVAL.length ? NOLEMVAL[i] : 0; // LEMNO
            } else {
                // LEMYES: VARST = 2*(2*LEMMASS*SLOPEVAL) + INTVALUE
                v[i] = lemMass * SLOPEVAL[i] + INTVALUE[i];
            }
        }
        // DXTEST: if descent stage on (DAPDATR1 positive/non-negative), fix up VARST0+1, VARST0+7.
        if (Agc.toSigned(agc.get("DAPDATR1")) > 0) {
            v[1] += v[8]; // DXCH VARST0+8D ; DAS VARST0+1
            v[7] += DXITFIX; // ADS VARST0+7
        }
        System.arraycopy(v, 0, varst, 0, 10);     // keep VARST0.. for later FIXCW calls (GAINCHNG)
        fixcw(v);
    }

    /** FIXCW entry from GAINCHNG: recompute IXX from the VARST table built by MASSPROP. */
    public void fixcw() {
        fixcw(java.util.Arrays.copyOf(varst, 10));
    }

    /** FIXCW: computation phase. Three passes, selects slope set by sign of delta CSM weight. */
    private void fixcw(double[] v) {
        double csmMass = agc.get("CSMMASS");
        double temp333 = 2 * (csmMass + NEGBPW); // DOUBLE
        int phi = 2;
        if (temp333 <= 0) {
            phi = -2; // NEG2: use the other slope set
        }
        int psi = 2;
        double[] out = new double[3];
        while (true) { // PEGGY
            double slope = v[5 + phi + 0];
            double val = 2 * slope * temp333 + v[psi];
            out[psi] = val;
            agc.set(psi == 0 ? "IXX" : psi == 1 ? "IXX+1" : "IXX+2", (int) val);
            if (psi == 0) {
                break;
            }
            psi--; // BOKKEP2
            phi--;
        }
        int w = (agc.get("DAPDATR1") & BIT14) != 0 ? agc.get("LEMMASS") : 0;
        agc.set("WEIGHT/G", Agc.add(w, agc.get("CSMMASS"))); // scaled B+16 kg
        // ENDMASSP: TC Q
    }
}
