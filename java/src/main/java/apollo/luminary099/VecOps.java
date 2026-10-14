package apollo.luminary099;

import apollo.agc.Agc;

/**
 * Small helper for the translations in this package: double-precision (28-bit) erasable
 * values, 3-vectors stored as three DP values, and the AGC multiply (MP) split into major/minor
 * parts. DP values are kept as signed longs equal to major*16384 + minor.
 */
public final class VecOps {
    private VecOps() {}

    public static final long HALF_RANGE = 16384L; // 2^14, one word of a DP value

    /** Reads the DP signed integer stored in sym (major) and sym+1 (minor). */
    public static long getDpRaw(Agc agc, String sym) {
        long hi = Agc.toSigned(agc.get(sym));
        long lo = Agc.toSigned(agc.get(sym + "+1"));
        return hi * HALF_RANGE + lo;
    }

    /** Stores a DP signed integer into sym, sym+1. */
    public static void setDpRaw(Agc agc, String sym, long value) {
        agc.set(sym, Agc.fromSigned((int) (value / HALF_RANGE)));
        agc.set(sym + "+1", Agc.fromSigned((int) (value % HALF_RANGE)));
    }

    /** Reads a DP value as a fraction in (-1,1) (full scale = 2^28). */
    public static double getDp(Agc agc, String sym) {
        return getDpRaw(agc, sym) / (double) (HALF_RANGE * HALF_RANGE);
    }

    public static void setDp(Agc agc, String sym, double fraction) {
        setDpRaw(agc, sym, Math.round(fraction * HALF_RANGE * HALF_RANGE));
    }

    /** Symbol name for vector component i (0..2) of a vector based at sym (2 words each). */
    public static String comp(String sym, int i) {
        return i == 0 ? sym : sym + "+" + (2 * i);
    }

    public static double[] getV(Agc agc, String sym) {
        return new double[] {getDp(agc, comp(sym, 0)), getDp(agc, comp(sym, 1)), getDp(agc, comp(sym, 2))};
    }

    public static void setV(Agc agc, String sym, double[] v) {
        for (int i = 0; i < 3; i++) {
            setDp(agc, comp(sym, i), v[i]);
        }
    }

    /** AGC MP: product of two signed words; returns {major, minor}. */
    public static int[] mp(int a, int b) {
        long p = (long) a * (long) b;
        return new int[] {(int) (p / HALF_RANGE), (int) (p % HALF_RANGE)};
    }

    /** DAS: adds major*2^14+minor to the DP value at sym. */
    public static void das(Agc agc, String sym, int major, int minor) {
        setDpRaw(agc, sym, getDpRaw(agc, sym) + (long) major * HALF_RANGE + minor);
    }

    public static double[] add(double[] a, double[] b) {
        return new double[] {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
    }

    public static double[] sub(double[] a, double[] b) {
        return new double[] {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    public static double[] scale(double[] a, double k) {
        return new double[] {a[0] * k, a[1] * k, a[2] * k};
    }

    public static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    public static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }

    public static double abs(double[] a) {
        return Math.sqrt(dot(a, a));
    }

    public static double[] unit(double[] a) {
        double m = abs(a);
        return m == 0 ? new double[3] : scale(a, 1.0 / m);
    }

    /** Matrix (rows) times vector. */
    public static double[] mxv(double[][] m, double[] v) {
        return new double[] {dot(m[0], v), dot(m[1], v), dot(m[2], v)};
    }

    /** Vector times matrix (VXM): v * M = M^T v. */
    public static double[] vxm(double[] v, double[][] m) {
        return new double[] {
            v[0] * m[0][0] + v[1] * m[1][0] + v[2] * m[2][0],
            v[0] * m[0][1] + v[1] * m[1][1] + v[2] * m[2][1],
            v[0] * m[0][2] + v[1] * m[1][2] + v[2] * m[2][2]};
    }

    /** Reads a 3x3 matrix stored row-wise as nine DP values (18 words) at sym. */
    public static double[][] getM(Agc agc, String sym) {
        double[][] m = new double[3][3];
        for (int i = 0; i < 9; i++) {
            m[i / 3][i % 3] = getDp(agc, i == 0 ? sym : sym + "+" + (2 * i));
        }
        return m;
    }
}
