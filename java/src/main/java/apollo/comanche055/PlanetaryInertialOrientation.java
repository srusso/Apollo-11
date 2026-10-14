package apollo.comanche055;

import apollo.agc.Agc;

/**
 * PLANETARY_INERTIAL_ORIENTATION (Comanche 055): conversion between planet-fixed (RP) and
 * reference-inertial (R) coordinates for Earth and Moon (RP-TO-R, R-TO-RP), using the
 * M matrix built by EARTHMX / MOONMX and the libration vector L (EARTHL / 504LM).
 *
 * The AGC interpretive code works in scaled fixed-point; here vectors are double[3],
 * the matrix M is double[3][3] (row-major, MMATRIX), angles are in revolutions (B0)
 * converted to radians for sin/cos, and times are in centiseconds.
 */
public final class PlanetaryInertialOrientation {
    // Moon orientation constants. Rates are in revs/centisecond.
    static final double NODDOT = -.457335121e-2 * Math.pow(2, -28); // REVS/CSEC B+28
    static final double FDOT = .570863327 * Math.pow(2, -27);       // REVS/CSEC B+27
    static final double BDOT = -3.07500686e-8 * Math.pow(2, -28);   // REVS/CSEC B+28
    static final double NODIO = .986209434;  // REVS B0 (6.19653663041 rad)
    static final double FSUBO = .829090536;  // REVS B0 (5.20932947829 rad)
    static final double BSUBO = .0651201393; // REVS B0 (0.40916190299 rad)
    static final double WEARTH = .973561595 * Math.pow(2, -23);     // REVS/CSEC B+23 (7.29211494E-5 rad/s)
    /** COS(5521.5 SEC) B-1, scaled value; the physical value is twice this. */
    static final double COSI = .99964173;
    /** SIN(5521.5 SEC) B-1. */
    static final double SINI = .02676579;

    private final Agc agc;
    /** MMATRIX: result of EARTHMX / MOONMX. */
    private double[][] mmatrix = new double[3][3];
    /** 504LM: moon libration vector LM (erasable, radians). */
    private double[] lm = new double[3];
    /** 504RPR: the R or RP vector. */
    private double[] rpr = new double[3];
    /** AZO, TIMSUBO, AXO, AYO erasable-held parameters (revs, csec, radians). */
    private double azo;
    private double timsubo;
    private double axo;
    private double ayo;

    public PlanetaryInertialOrientation(Agc agc) {
        this.agc = agc;
    }

    public double[] lmVector() { return lm.clone(); }
    public double[][] mmatrix() { return mmatrix; }
    public void setLm(double[] v) { lm = v.clone(); }
    public void setEarthParams(double azo, double axo, double ayo) {
        this.azo = azo; this.axo = axo; this.ayo = ayo;
    }
    public void setTimsubo(double csec) { this.timsubo = csec; }

    /**
     * RP-TO-R: convert planet-fixed vector rp at time t (csec) to inertial R.
     * R = M^T (RP + LP x RP).
     * @param moon true for the moon (B-register nonzero), false for Earth (BHIZ RPTORA)
     */
    public double[] rpToR(double[] rp, double t, boolean moon) {
        double[] lp;
        if (moon) {
            mmatrix = moonmx(t);
            lp = lm.clone();                     // 504LM
        } else {                                  // RPTORA
            mmatrix = earthmx(t);
            lp = vxm(earthl(), mmatrix);          // LP = M(T) * L
        }
        rpr = rp.clone();
        double[] sum = vad(cross(lp, rp), rp);    // RPTORB
        double[] r = vxm(sum, mmatrix);
        return r;                                 // RPRPXXXX: reset pushdown, exit via RPREXIT
    }

    /**
     * R-TO-RP: convert inertial R at time t (csec) to planet-fixed RP.
     * RP = M (R - L x R).
     */
    public double[] rToRp(double[] r, double t, boolean moon) {
        double[] l;
        if (moon) {
            mmatrix = moonmx(t);
            l = vxm(lm, mmatrix);                 // L = MT(T)*LP
        } else {                                  // RTORPA
            mmatrix = earthmx(t);
            l = earthl();                         // L = (-AX,-AY,0)
        }
        double[] diff = vsu(r, cross(l, r));      // RTORPB
        return mxv(mmatrix, diff);                // RPRPXXXX
    }

    /** NEWANGLE: X = X0 + XDOT*(T+T0), in revolutions modulo 1. */
    double newangle(double x0, double xdot, double t) {
        double x = x0 + xdot * (t + timsubo);
        return x - Math.floor(x);
    }

    /** MOONMX: builds the moon M matrix from B, F and NODI angles. */
    double[][] moonmx(double t) {
        double bRevs = newangle(BSUBO, BDOT, t);
        double cob = Math.cos(2 * Math.PI * bRevs);   // COB
        double sob = Math.sin(2 * Math.PI * bRevs);   // SOB
        double f = 2 * Math.PI * newangle(FSUBO, FDOT, t);   // 504F
        double nodi = 2 * Math.PI * newangle(NODIO, NODDOT, t);
        double c = Math.cos(nodi);
        double s = Math.sin(nodi);                    // SINNODI
        double[] a = {c, s * cob, s * sob};           // AVECTR
        double[] b = {-s, c * cob, c * sob};          // BVECTR
        double[] cv = {0, -sob, cob};                 // CVECTR
        double cosI = 2 * COSI;
        double sinI = 2 * SINI;
        double[] m2 = vad(vxsc(b, sinI), vxsc(cv, cosI));   // M2 = B*SINI + C*COSI
        double[] d = vsu(vxsc(b, cosI), vxsc(cv, sinI));    // DVECTR = B*COSI - C*SINI
        double[] m1 = vsu(vxsc(a, Math.sin(f)), vxsc(d, Math.cos(f)));  // M1 = A sinF - D cosF
        double[] m0 = vxsc(vad(vxsc(a, Math.cos(f)), vxsc(d, Math.sin(f))), -1); // M0 = -(A cosF + D sinF)
        return new double[][] {m0, m1, m2};
    }

    /** EARTHMX: rotation about Z by AZ = AZO + WEARTH*(T+T0). */
    double[][] earthmx(double t) {
        double az = 2 * Math.PI * newangle(azo, WEARTH, t);   // 504AZ
        return new double[][] {
            {Math.cos(az), Math.sin(az), 0},
            {-Math.sin(az), Math.cos(az), 0},
            {0, 0, 1},
        };
    }

    /** EARTHL: L = (-AXO, -AYO, 0) (504LPL). */
    double[] earthl() {
        return new double[] {-axo, -ayo, 0};
    }

    // --- vector helpers matching interpretive operations ---
    static double[] cross(double[] a, double[] b) {  // VXV
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }
    static double[] vad(double[] a, double[] b) { return new double[] {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
    static double[] vsu(double[] a, double[] b) { return new double[] {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
    static double[] vxsc(double[] a, double k) { return new double[] {a[0] * k, a[1] * k, a[2] * k}; }
    /** MXV: M * v. */
    static double[] mxv(double[][] m, double[] v) {
        double[] r = new double[3];
        for (int i = 0; i < 3; i++) r[i] = m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2];
        return r;
    }
    /** VXM: v * M (= M transpose times v). */
    static double[] vxm(double[] v, double[][] m) {
        double[] r = new double[3];
        for (int j = 0; j < 3; j++) r[j] = v[0] * m[0][j] + v[1] * m[1][j] + v[2] * m[2][j];
        return r;
    }
}
