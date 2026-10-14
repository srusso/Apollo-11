package apollo.luminary099;

import apollo.agc.Agc;

/**
 * SINGLE_PRECISION_SUBROUTINES.agc (Luminary 1A build 099, page 1102).
 * Single precision sine and cosine; arguments scaled at pi (revolutions/2), result scaled at 1.
 * Coefficients C1/2, C3/2, C5/2 and the LIMITS table are defined elsewhere (CONSTANTS).
 */
public final class SinglePrecisionSubroutines {
    private final Agc agc;

    public SinglePrecisionSubroutines(Agc agc) {
        this.agc = agc;
    }

    /** Fractional word helper: value in (-1,1) to 15-bit word scaled at 1 (2^14). */
    private static double toFrac(int word) {
        return Agc.toSigned(word) / 16384.0;
    }

    private static int toWord(double f) {
        return Agc.fromSigned((int) Math.round(f * 16384.0));
    }

    /** SPCOS: cos(pi*x) = sin(pi*(x + 1/2)). Returns the result in TEMK and as return value. */
    public int spcos(int a) {
        return spsin(Agc.add(a, 0x2000)); // AD HALF
    }

    /**
     * SPSIN: sin(pi*x) with x in A scaled at 1 (full word = pi). Uses the polynomial
     * sin = 2*x*(C1/2 + x^2*(C3/2 + x^2*C5/2)) (label POLLEY), with the argument first
     * folded into +/- 1/2 (SPT).
     */
    public int spsin(int a) {
        agc.set("TEMK", a);
        int temk = Agc.toSigned(a);
        // SPT: DOUBLE; if no overflow the argument is already within +/- 1/2.
        int doubled = temk * 2;
        if (Math.abs(doubled) > 0x3FFF) {
            // overflow: reflect about 90 degrees using LIMITS (+/-1 at 90 degrees)
            int limits = temk >= 0 ? 0x3FFF : -0x3FFF;
            int reflected = limits - temk; // INDEX TEMK / AD LIMITS / COM / AD TEMK
            if (reflected == 0) {
                return arg90(temk);
            }
            temk = reflected;
        }
        return polley(temk);
    }

    /** POLLEY: polynomial evaluation. */
    private int polley(int temkSigned) {
        double x = temkSigned / 16384.0;
        double sq = x * x;
        agc.set("SQ", toWord(sq));
        double c5 = toFrac(agc.get("C5/2"));
        double c3 = toFrac(agc.get("C3/2"));
        double c1 = toFrac(agc.get("C1/2"));
        double r = (c5 * sq + c3) * sq + c1;
        r = 2.0 * r * x; // MP TEMK then DDOUBL
        int w = toWord(r);
        agc.set("TEMK", w);
        return w;
    }

    /** ARG90: exactly 90 degrees: result is +/- 1 (CS LIMITS indexed by A). */
    private int arg90(int temkSigned) {
        int w = Agc.fromSigned(temkSigned >= 0 ? -0x3FFF : 0x3FFF);
        return w;
    }
}
