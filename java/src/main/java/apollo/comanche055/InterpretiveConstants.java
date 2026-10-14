package apollo.comanche055;

import apollo.agc.Agc;

/**
 * INTERPRETIVE_CONSTANTS.agc (Comanche055): constants used by the interpreter
 * (INTPRET1 / INTPRET2 fixed-memory banks). Double-precision values are given as doubles
 * (fractions of full scale); vectors as double[3].
 */
public final class InterpretiveConstants {
    private final Agc agc;

    public InterpretiveConstants(Agc agc) {
        this.agc = agc;
    }

    // ---- INTPRET1 (COUNT 23/ICONS) ----
    /** DP1/4TH: 2DEC .25 */
    public static final double DP1_4TH = 0.25;
    /** UNITZ: 2DEC 0 */
    public static final double UNITZ = 0.0;
    /** UNITY: 2DEC 0 */
    public static final double UNITY = 0.0;
    /** UNITX: 2DEC .5 (also DPHALF) */
    public static final double UNITX = 0.5;
    /** ZEROVECS: three zero double-precision words. */
    public static final double[] ZEROVECS = {0.0, 0.0, 0.0};
    /** DPHALF = UNITX */
    public static final double DPHALF = UNITX;
    /** DPPOSMAX: OCT 37777 / OCT 37777 (largest positive double-precision value). */
    public static final int[] DPPOSMAX = {0x3FFF, 0x3FFF};

    // ---- INTPRET2 (COUNT 14/ICONS) ----
    /** ZUNIT: 2DEC 0 */
    public static final double ZUNIT = 0.0;
    /** YUNIT: 2DEC 0 */
    public static final double YUNIT = 0.0;
    /** XUNIT: 2DEC .5 */
    public static final double XUNIT = 0.5;
    /** ZEROVEC: three zero words. */
    public static final double[] ZEROVEC = {0.0, 0.0, 0.0};
    /** OCT 77777 (-0); the -0, -6, -12 constants must remain in this order. */
    public static final int MINUS_ZERO = 0x7FFF;
    /** DEC-6 */
    public static final int DEC_MINUS_6 = -6;
    /** DEC-12 */
    public static final int DEC_MINUS_12 = -12;
    /** LODPMAX: 2OCT 3777737777 (adjacent to LODPMAX1 and must be the same, for integration). */
    public static final int[] LODPMAX = {0x3FFF, 0x3FFF};
    /** LODPMAX1: 2OCT 3777737777 */
    public static final int[] LODPMAX1 = {0x3FFF, 0x3FFF};
    /** ZERODP = ZEROVEC */
    public static final double[] ZERODP = ZEROVEC;
    /** HALFDP = XUNIT */
    public static final double HALFDP = XUNIT;
}
