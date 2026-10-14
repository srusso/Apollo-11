package apollo.luminary099;

import apollo.agc.Agc;

/**
 * S-BAND_ANTENNA_FOR_LM (Luminary 099): routine R05, computes S-band antenna pitch and yaw
 * angles from the LM state vector, the Earth/Moon direction and the current IMU orientation.
 * The AGC interpretive code is rendered using double-precision vector arithmetic.
 */
public class SBandAntennaForLm {
    /** 1/SQRT(2). */
    public static final double ONE_OVER_SQRT2 = 0.7071067815;
    /** VN 0651: display of antenna angles. */
    public static final int V06N51 = 0651;

    private final Agc agc;
    /** Collaborator hooks, supplied by the surrounding system (kept abstract here). */
    public interface Env {
        void r02Both();
        double loadTime();
        void lemConic(double tdec1);
        int moonSphereFlag();
        double[] ratt();
        double tat();
        double[] lunPos();
        double[] vMoon();
        double remDist();
        double[][] refsmmat();
        double[][] smnb(double pitch, double yaw);
        boolean overflowCleared();
    }
    private Env env;

    public SBandAntennaForLm(Agc agc) {
        this.agc = agc;
    }

    public void setEnv(Env env) {
        this.env = env;
    }

    private static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    private static double[] unit(double[] a) {
        double n = Math.sqrt(dot(a, a));
        return new double[] {a[0] / n, a[1] / n, a[2] / n};
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]};
    }

    private static double[] sub(double[] a, double[] b) {
        return new double[] {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    /** SBANDANT: R05 entry. Returns {pitchAng, yawAng} (revolutions), or null on SBANDEX overflow exit. */
    public double[] sbandant() {
        env.r02Both(); // CHECK IF IMU IS ON AND ALIGNED
        double tdec1 = env.loadTime();
        env.lemConic(tdec1); // CONIC INTEGRATION TO TDEC1
        double[] ur;
        double[] rlm = env.ratt();
        if (env.moonSphereFlag() == 0) {
            // CONV4: UE = -UNIT(RATT), Earth sphere
            ur = env.ratt();
        } else {
            // CONV3: UE = UNIT vector earth-to-moon; (REM)(UEM) + RL, Moon sphere
            double[] uem = env.lunPos();
            double[] vm = env.vMoon();
            double rem = env.remDist();
            double[] sum = new double[3];
            for (int i = 0; i < 3; i++) {
                sum[i] = 2 * vm[i] * rem + rlm[i];
            }
            ur = sum;
        }
        // CONV5: UR = -UNIT(...), transform by REFSMMAT to stable member
        ur = unit(ur);
        for (int i = 0; i < 3; i++) {
            ur[i] = -ur[i];
        }
        double[][] ref = env.refsmmat();
        double[] urSm = new double[3];
        for (int i = 0; i < 3; i++) {
            urSm[i] = 2 * dot(ref[i], ur);
        }
        double pitch = 0;
        double yaw = 0; // ZERO OUT ANGLES
        // *SMNB*: pre-multiply by NBSA matrix
        double[][] nb = env.smnb(pitch, yaw);
        double[] r = new double[3];
        for (int i = 0; i < 3; i++) {
            r[i] = 2 * dot(nb[i], urSm);
        }
        // Rotate by 45 degrees about the Y axis (1OVSQRT2 terms)
        double x = r[0];
        double y = r[1];
        double z = r[2];
        double newZ = (z - x) * ONE_OVER_SQRT2;
        double newX = (z + x) * ONE_OVER_SQRT2;
        double[] rb = {newX, y, newZ};
        double[] urp = unit(rb);
        // Projection of R onto LM XZ plane
        double[] hiUnitY = {0, 0.5, 0};
        double[] hiUnitZ = {0, 0, 0.5};
        double[] hiUnitX = {0.5, 0, 0};
        double[] proj = new double[3];
        double yc = dot(urp, hiUnitY) * 2;
        for (int i = 0; i < 3; i++) {
            proj[i] = urp[i] - yc * hiUnitY[i] * 2;
        }
        double[] unitProj = unit(proj);
        double[] xvec = cross(new double[] {0, 0, 1}, unitProj);
        for (int i = 0; i < 3; i++) {
            xvec[i] = -xvec[i];
        }
        double sgn = Math.signum(dot(xvec, new double[] {0, 1, 0}));
        double abv = Math.sqrt(dot(xvec, xvec));
        pitch = Math.asin(sgn * abv) / (2 * Math.PI); // ASIN, REV B0
        if (dot(unitProj, new double[] {0, 0, 1}) <= 0) {
            // -90 TO +90 not satisfied: PITCHANG = HIDPHALF - PITCHANG
            pitch = 0.5 - pitch;
        }
        // NOADJUST: Z = UR x URP
        double[] zvec = cross(ur, urp);
        double ang = pitch * 2 * Math.PI;
        double[] t1 = {0, 0, Math.sin(ang)};
        double[] t2 = {Math.cos(ang), 0, 0};
        double[] w = sub(t2, t1); // (UX COS ALPHA) - (UZ SIN ALPHA)
        double yawS = dot(w, zvec);
        double yawAbv = Math.sqrt(dot(w, w));
        yaw = Math.asin(Math.signum(yawS) * yawAbv) / (2 * Math.PI);
        agc.set("PITCHANG", Agc.fromSigned((int) Math.round(pitch * 16384)));
        agc.set("YAWANG", Agc.fromSigned((int) Math.round(yaw * 16384)));
        return new double[] {pitch, yaw};
    }

    /**
     * SBANDEX: if bit 5 of EXTVBACT is still set, display V06N51 (via GOMARKFR) and, on proceed,
     * loop back to SBANDANT +2 to keep displaying the angles.
     */
    public boolean sbandex() {
        if ((agc.get("EXTVBACT") & 020) == 0) {
            return false; // ENDEXT
        }
        // PRIOCHNG PRIO5; GOMARKFR displaying V06N51
        // Terminate/Proceed -> B5OFF; Recycle -> ENDOFJOB; otherwise BLANKET R3, PRIO4, SBANDANT +2.
        return true;
    }
}
