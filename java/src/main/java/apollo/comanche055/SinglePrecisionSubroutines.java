package apollo.comanche055;

import apollo.agc.Agc;

/**
 * SINGLE_PRECISION_SUBROUTINES: single-precision sine/cosine (SPSIN, SPCOS).
 * Arguments are scaled at pi (word value 1.0 == pi radians, i.e. 0x4000 units per pi);
 * the result is scaled at 1. The polynomial coefficients C5/2, C3/2, C1/2 and the LIMITS
 * table are defined elsewhere in the program; here the polynomial is evaluated numerically.
 */
public class SinglePrecisionSubroutines {
    private static final double UNIT = 16384.0;
    private final Agc agc;

    public SinglePrecisionSubroutines(Agc agc) {
        this.agc = agc;
    }

    /** SPCOS: AD HALF then fall into SPSIN (cos x = sin(x + pi/2)). Returns result, also left in TEMK. */
    public int spcos(int a) {
        return spsin(Agc.add(a, 0x2000));
    }

    /** SPSIN: sine of A (scaled at pi). Result left in TEMK and returned. */
    public int spsin(int a) {
        agc.set("TEMK", a);
        int v = Agc.toSigned(a);
        // SPT: DOUBLE; no overflow -> POLLEY directly.
        // Overflow (|arg| > pi/2): reflect through LIMITS (+-MAX) -> argument folded into range.
        // Exactly +-pi/2 (ARG90): result is +-1 from the LIMITS table.
        if (Math.abs(v) == 0x2000) {
            return arg90(a);
        }
        double x = v / UNIT;
        if (Math.abs(v) > 0x2000) {
            x = (v > 0 ? 1.0 : -1.0) - x; // sin(pi - x) = sin(x)
        }
        agc.set("TEMK", Agc.fromSigned((int) Math.round(x * UNIT)));
        return polley();
    }

    /** POLLEY: odd polynomial sin(pi*x) on TEMK; SQ holds TEMK squared. */
    public int polley() {
        double x = Agc.toSigned(agc.get("TEMK")) / UNIT;
        agc.set("SQ", Agc.fromSigned((int) Math.round(x * x * UNIT)));
        int out = Agc.fromSigned((int) Math.round(Math.sin(Math.PI * x) * (UNIT - 1)));
        agc.set("TEMK", out);
        return out;
    }

    /** ARG90: result is +MAX or -MAX (LIMITS table indexed by sign of A). */
    public int arg90(int a) {
        int out = Agc.toSigned(a) > 0 ? 0x3FFF : Agc.fromSigned(-0x3FFF);
        agc.set("TEMK", out);
        return out;
    }
}
