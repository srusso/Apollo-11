package apollo.luminary099;

import apollo.agc.Agc;

/**
 * Planetary inertial orientation (Luminary 099, $$/LUROT): conversion between the reference (basic) coordinate
 * system and Earth-fixed or Moon-fixed planetary coordinates.
 *
 * The original interpretive-language routines work in fixed-point scaled values (B-29 metres for Earth, B-27 for the
 * Moon, matrices at B-1, angles in revolutions). This translation uses doubles in natural units: positions in
 * metres, angles in revolutions (unless noted), time in centiseconds, matrices unscaled.
 */
public class PlanetaryInertialOrientation {
    private final Agc agc;

    // Launch data (erasable in the original)
    /** BSUBO, BDOT: moon node-related angle B (revs) and rate (revs/csec). */
    public double bSubO, bDot;
    /** FSUBO, FDOT. */
    public double fSubO, fDot;
    /** NODIO, NODDOT. */
    public double nodIO, nodDot;
    /** COSI, SINI: cos/sin of angle between mean lunar equator and the ecliptic (1 deg 32.1 min). */
    public double cosI, sinI;
    /** AZO (revs), WEARTH (revs/csec). */
    public double azO, wEarth;
    /** TIMSUBO: time offset in csec (triple precision in the original). */
    public double timSubO;
    /** 504LM: libration vector of the moon in moon-fixed coordinates (radians). */
    public double[] lm = new double[3];
    /** AXO, AYO: Earth L-vector components. */
    public double axO, ayO;

    /** MMATRIX: last computed M matrix, rows M0, M1, M2. */
    public double[][] mMatrix = new double[3][3];

    public PlanetaryInertialOrientation(Agc agc) {
        this.agc = agc;
    }

    /**
     * RP-TO-R: R = MT(T) * (RP + LP x RP).
     *
     * @param moon true for Moon (MPAC non-zero), false for Earth
     * @param rp   vector in planetary coordinates
     * @param time time, csec
     * @return vector in the reference system
     */
    public double[] rpToR(boolean moon, double[] rp, double time) {
        double[] lp;
        if (moon) {
            moonMx(time);                       // LP = LM for moon
            lp = lm.clone();
        } else {
            earthMx(time);                      // RPTORA: Earth computations
            lp = mxv(mMatrix, earthL());        // LP = M(T)*L
        }
        double[] sum = vad(cross(lp, rp), rp);  // RPTORB
        return vxm(sum, mMatrix);               // MT * (RP + LP x RP)
    }

    /**
     * R-TO-RP: RP = M(T) * (R - L x R).
     *
     * @param moon true for Moon, false for Earth
     */
    public double[] rToRp(boolean moon, double[] r, double time) {
        double[] l;
        if (moon) {
            moonMx(time);
            l = vxm(lm, mMatrix);               // L = MT(T)*LP
        } else {
            earthMx(time);                      // RTORPA
            l = earthL();                       // L = (-AX,-AY,0)
        }
        double[] diff = vsub(r, cross(l, r));   // RTORPB
        return mxv(mMatrix, diff);              // RPRPXXXX
    }

    /** MOONMX: compute the Moon M matrix (rows M0, M1, M2) into {@link #mMatrix}. */
    public void moonMx(double time) {
        double b = newAngle(bSubO, bDot, time);
        double cob = Math.cos(2 * Math.PI * b);
        double sob = Math.sin(2 * Math.PI * b);
        double f = newAngle(fSubO, fDot, time);             // 504F
        double nodI = newAngle(nodIO, nodDot, time);
        double cn = Math.cos(2 * Math.PI * nodI), sn = Math.sin(2 * Math.PI * nodI);

        double[] a = {cn, sn * cob, sn * sob};              // AVECTR
        double[] bv = {-sn, cn * cob, cn * sob};            // BVECTR
        double[] c = {0, -sob, cob};                        // CVECTR
        double[] m2 = vad(vxsc(bv, sinI), vxsc(c, cosI));   // M2 = B*SINI + C*COSI
        double[] d = vsub(vxsc(bv, cosI), vxsc(c, sinI));   // DVECTR
        double cf = Math.cos(2 * Math.PI * f), sf = Math.sin(2 * Math.PI * f);
        double[] m1 = vsub(vxsc(a, sf), vxsc(d, cf));       // M1 = A*SINF - D*COSF
        double[] m0 = vxsc(vad(vxsc(a, cf), vxsc(d, sf)), -1); // M0 = -(A*COSF + D*SINF)
        mMatrix = new double[][] {m0, m1, m2};
    }

    /**
     * NEWANGLE: X = X0 + XDOT*(T + T0), revolutions.
     *
     * @param x0   initial value (revs)
     * @param xdot rate (revs/csec)
     * @param time time T (csec)
     */
    public double newAngle(double x0, double xdot, double time) {
        return x0 + xdot * (time + timSubO);
    }

    /** EARTHMX: compute the Earth M matrix (rotation about Z by AZ). */
    public void earthMx(double time) {
        double az = 2 * Math.PI * newAngle(azO, wEarth, time);  // 504AZ
        double c = Math.cos(az), s = Math.sin(az);
        mMatrix = new double[][] {{c, s, 0}, {-s, c, 0}, {0, 0, 1}};
    }

    /** EARTHL: L vector for Earth = (-AX, -AY, 0) radians. */
    public double[] earthL() {
        return new double[] {-axO, -ayO, 0};
    }

    // --- vector helpers (interpreter VXV, VAD, VSU, VXSC, MXV, VXM) ---
    private static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }
    private static double[] vad(double[] a, double[] b) {
        return new double[] {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
    }
    private static double[] vsub(double[] a, double[] b) {
        return new double[] {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }
    private static double[] vxsc(double[] a, double k) {
        return new double[] {a[0] * k, a[1] * k, a[2] * k};
    }
    /** Matrix times vector. */
    private static double[] mxv(double[][] m, double[] v) {
        double[] r = new double[3];
        for (int i = 0; i < 3; i++) r[i] = m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2];
        return r;
    }
    /** Vector times matrix (= transpose(matrix) times vector). */
    private static double[] vxm(double[] v, double[][] m) {
        double[] r = new double[3];
        for (int j = 0; j < 3; j++) r[j] = v[0] * m[0][j] + v[1] * m[1][j] + v[2] * m[2][j];
        return r;
    }
}
