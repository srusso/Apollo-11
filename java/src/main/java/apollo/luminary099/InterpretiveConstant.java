package apollo.luminary099;

/**
 * INTERPRETIVE_CONSTANT.agc - constants used by the interpreter (banks INTPRET1 / INTPRET2).
 *
 * Double-precision values are given as the two 15-bit words {high, low}; vectors as three DP words.
 */
public final class InterpretiveConstant {
    private InterpretiveConstant() { }

    // ---- INTPRET1 ----
    /** DP1/4TH: 2DEC .25 */
    public static final int[] DP1_4TH = {010000, 0};
    /** UNITZ: 2DEC 0 */
    public static final int[] UNITZ = {0, 0};
    /** UNITY: 2DEC 0 */
    public static final int[] UNITY = {0, 0};
    /** UNITX: 2DEC .5 (also DPHALF) */
    public static final int[] UNITX = {020000, 0};
    /** ZEROVECS: three DP zeros. */
    public static final int[] ZEROVECS = {0, 0, 0, 0, 0, 0};
    /** DPHALF = UNITX */
    public static final int[] DPHALF = UNITX;
    /** DPPOSMAX: OCT 37777 / OCT 37777 */
    public static final int[] DPPOSMAX = {037777, 037777};

    // ---- INTPRET2 ----
    /** ZUNIT: 2DEC 0 */
    public static final int[] ZUNIT = {0, 0};
    /** YUNIT: 2DEC 0 */
    public static final int[] YUNIT = {0, 0};
    /** XUNIT: 2DEC .5 */
    public static final int[] XUNIT = {020000, 0};
    /** ZEROVEC: three DP zeros. */
    public static final int[] ZEROVEC = {0, 0, 0, 0, 0, 0};
    /** Unlabelled OCT 77777 following ZEROVEC (minus zero). */
    public static final int MINUS_ZERO = 077777;
    /** DFC-6: DEC -6 */
    public static final int DFC_MINUS_6 = -6;
    /** DFC-12: DEC -12 */
    public static final int DFC_MINUS_12 = -12;
    /** LODPMAX: 2OCT 3777737777 */
    public static final int[] LODPMAX = {037777, 037777};
    /** LODPMAX1: 2OCT 3777737777 */
    public static final int[] LODPMAX1 = {037777, 037777};
    /** ZERODP = ZEROVEC */
    public static final int[] ZERODP = ZEROVEC;
    /** HALFDP = XUNIT */
    public static final int[] HALFDP = XUNIT;
}
