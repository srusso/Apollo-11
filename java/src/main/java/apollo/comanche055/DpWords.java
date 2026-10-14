package apollo.comanche055;

import apollo.agc.Agc;

/**
 * Helper for the Comanche055 translations (shared by R30, SERVICER207, REENTRY_CONTROL, P61-P67,
 * FRESH_START_AND_RESTART): double-precision and triple-precision erasable quantities.
 *
 * A double-precision AGC quantity occupies two consecutive erasable words (major, minor). Here it is
 * addressed as {@code SYMBOL} and {@code SYMBOL+1}, and read/written as a fraction in [-1, 1)
 * (value = raw / 2^28), i.e. in the AGC's own scaled units. Vector quantities are three
 * double-precision words at SYMBOL, SYMBOL+2, SYMBOL+4.
 */
final class DpWords {
    private static final double SCALE = 268435456.0; // 2^28

    private DpWords() {
    }

    static String off(String sym, int n) {
        return n == 0 ? sym : sym + "+" + n;
    }

    /** Raw 28-bit signed value of a DP pair. */
    static long getRaw(Agc agc, String sym) {
        return ((long) Agc.toSigned(agc.get(sym)) << 14) + Agc.toSigned(agc.get(off(sym, 1)));
    }

    static void setRaw(Agc agc, String sym, long raw) {
        long hi = raw >> 14;
        long lo = raw - (hi << 14);
        agc.set(sym, Agc.fromSigned((int) hi));
        agc.set(off(sym, 1), Agc.fromSigned((int) lo));
    }

    static double get(Agc agc, String sym) {
        return getRaw(agc, sym) / SCALE;
    }

    static void set(Agc agc, String sym, double value) {
        setRaw(agc, sym, Math.round(value * SCALE));
    }

    static double[] getVec(Agc agc, String sym) {
        return new double[] {get(agc, sym), get(agc, off(sym, 2)), get(agc, off(sym, 4))};
    }

    static void setVec(Agc agc, String sym, double[] v) {
        set(agc, sym, v[0]);
        set(agc, off(sym, 2), v[1]);
        set(agc, off(sym, 4), v[2]);
    }

    /** Adds {@code delta} (in raw DP units, e.g. centiseconds) to a DP pair (the DAS instruction). */
    static void das(Agc agc, String sym, long delta) {
        setRaw(agc, sym, getRaw(agc, sym) + delta);
    }

    static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    static double abval(double[] a) {
        return Math.sqrt(dot(a, a));
    }

    static double[] unit(double[] a) {
        double m = abval(a);
        return m == 0 ? a.clone() : new double[] {a[0] / m, a[1] / m, a[2] / m};
    }

    static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }

    static double[] scale(double[] a, double k) {
        return new double[] {a[0] * k, a[1] * k, a[2] * k};
    }

    static double[] add(double[] a, double[] b) {
        return new double[] {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
    }

    static double[] sub(double[] a, double[] b) {
        return new double[] {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    /** Test bit {@code n} (1-15) of an erasable word. */
    static boolean bit(Agc agc, String sym, int n) {
        return (agc.get(sym) & (1 << (n - 1))) != 0;
    }

    static void setBit(Agc agc, String sym, int n, boolean on) {
        int w = agc.get(sym);
        agc.set(sym, on ? (w | (1 << (n - 1))) : (w & ~(1 << (n - 1))));
    }
}
