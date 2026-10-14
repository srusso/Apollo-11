package apollo.luminary099;

import apollo.agc.Agc;

/**
 * Translation of POWERED_FLIGHT_SUBROUTINES.agc (Luminary 1A build 099).
 *
 * CDU trigonometry (CDUTRIG, CD*TR*G(S), QUICTRIG), the AX*SR*T axis rotation used for
 * body/stable-member transformations (TRG*SMNB, TRG*NBSM, CDU*SMNB, CDU*NBSM, *SMNB*,
 * *NBSM*), and FLESHPOT / CALCSMSC, which computes the body-to-stable-member matrix XNB.
 *
 * Angles are held as 15-bit 2's complement words in CDUSPOT (order Y Z X at +0, +2, +4);
 * sines and cosines (half-unit, double precision) go to SINCDU and COSCDU (6 words each,
 * Y Z X). Vectors arrive in VBUF as three DP words (x, y, z) at VBUF, VBUF+2, VBUF+4.
 */
public class PoweredFlightSubroutines {
    private final Agc agc;

    public PoweredFlightSubroutines(Agc agc) {
        this.agc = agc;
    }

    /** Hook for external routines (CDULOGIC, SPSIN, SPCOS, ...); default does nothing. */
    protected void extern(String routine, Object... args) {
    }

    // ---- helpers ----

    private double dp(String sym) {
        long v = ((long) Agc.toSigned(agc.get(sym)) << 14) + Agc.toSigned(agc.get(sym + "+1"));
        return v / (double) (1L << 28);
    }

    private void setDp(String sym, double value) {
        long v = Math.round(value * (1L << 28));
        agc.set(sym, Agc.fromSigned((int) (v >> 14)));
        agc.set(sym + "+1", Agc.fromSigned((int) (v & 037777)));
    }

    private static String at(String base, int off) {
        return off == 0 ? base : base + "+" + off;
    }

    private static double dpFromWord(int w2sComp) {
        return Agc.toSigned(w2sComp) / (double) (1 << 14);
    }

    /** Converts a 15-bit 2's complement CDU word to an angle in revolutions (-0.5..0.5). */
    private static double cduRevs(int word) {
        int w = word & Agc.WORD_MASK;
        if ((w & Agc.SIGN_BIT) != 0) {
            w -= 0x8000;
        }
        return w / (double) (1 << 15);
    }

    // ---- CDUTRIG family ----

    /** CDUTRIG: sines and cosines of the current CDU angles (interpretive entry, RVQ). */
    public void cdutrig() {
        cdutrigs();
    }

    /** CD*TR*G: like CD*TR*GS but callable from interpretive. */
    public void cdTrG() {
        cdTrGs();
    }

    /** CDUTRIGS: load CDUX/Y/Z into CDUSPOT (Y Z X) then compute. */
    public void cdutrigs() {
        agc.set("CDUSPOT+4", agc.get("CDUX"));
        agc.set("CDUSPOT", agc.get("CDUY"));
        agc.set("CDUSPOT+2", agc.get("CDUZ"));
        cdTrGs();
    }

    /**
     * CD*TR*GS: for each of the three angles at CDUSPOT, store the 1's complement version back
     * in CDUSPOT (DP) and the DP half-sine and half-cosine in SINCDU / COSCDU.
     */
    public void cdTrGs() {
        for (int tem3 = 4; tem3 >= 0; tem3 -= 2) {       // TR*GL**P: index 4, 2, 0
            double rev = cduRevs(agc.get(at("CDUSPOT", tem3)));
            extern("CDULOGIC");                           // 2's -> 1's complement in MPAC
            agc.set(at("CDUSPOT", tem3), Agc.fromSigned((int) Math.round(rev * (1 << 14))));
            double angle = rev * 2 * Math.PI;
            setDp(at("COSCDU", 3 * tem3), Math.cos(angle) / 2);   // DP half-unit cosine
            setDp(at("SINCDU", 3 * tem3), Math.sin(angle) / 2);   // DP half-unit sine
        }
    }

    // ---- QUICTRIG ----

    /**
     * QUICTRIG: fast sine/cosine of the three 2's complement angles at CDUSPOT, +2, +4 stored
     * single precision (scaled to match interpreter outputs) in SINCDU / COSCDU. Intended for
     * guidance cycle use (4.1 ms vs 40 ms for CD*TR*GS).
     */
    public void quictrig() {
        for (int i = 4; i >= 0; i -= 2) {
            double angle = cduRevs(agc.get(at("CDUSPOT", i))) * 2 * Math.PI;
            setDp(at("SINCDU", 3 * i), Math.sin(angle) / 2);
            setDp(at("COSCDU", 3 * i), Math.cos(angle) / 2);
        }
    }

    // ---- interface routines ----

    /** CDU*SMNB: SM to NB using current CDU angles; vector in MPAC, result in MPAC. */
    public void cduSmnb() {
        cdutrigs();
        cMmN1();
    }

    /** TRG*SMNB: SM to NB using angles at CDUSPOT. */
    public void trgSmnb() {
        cdTrGs();
        cMmN1();
    }

    private void cMmN1() {
        mpacVbuf();
        axSrT(-3);                                        // CS THREE: SM to NB
        vbufMpac();
    }

    /** CDU*NBSM: NB to SM using current CDU angles. */
    public void cduNbsm() {
        cdutrigs();
        cMmN3();
    }

    /** TRG*NBSM: NB to SM using angles at CDUSPOT. */
    public void trgNbsm() {
        cdTrGs();
        cMmN3();
    }

    private void cMmN3() {
        mpacVbuf();
        axSrT(3);                                         // CA THREE: NB to SM
        vbufMpac();
    }

    /** *SMNB*: like TRG*SMNB but uses SINCDU / COSCDU already computed. */
    public void starSmnbStar() {
        cMmN1();
    }

    /** *NBSM*: like TRG*NBSM but uses SINCDU / COSCDU already computed. */
    public void starNbsmStar() {
        cMmN3();
    }

    /** MPACVBUF: copy the interpretive vector in MPAC (+0, +3, +5) to VBUF (+0, +2, +4). */
    private void mpacVbuf() {
        for (int i = 0; i < 3; i++) {
            int src = i == 0 ? 0 : (i == 1 ? 3 : 5);
            agc.set(at("VBUF", 2 * i), agc.get(at("MPAC", src)));
            agc.set(at("VBUF", 2 * i + 1), agc.get(at("MPAC", src + 1)));
        }
    }

    private void vbufMpac() {
        for (int i = 0; i < 3; i++) {
            int dst = i == 0 ? 0 : (i == 1 ? 3 : 5);
            agc.set(at("MPAC", dst), agc.get(at("VBUF", 2 * i)));
            agc.set(at("MPAC", dst + 1), agc.get(at("VBUF", 2 * i + 1)));
        }
    }

    // ---- AX*SR*T ----

    /** INDEXI: the "index of indexes" used by AX*SR*T (don't touch). */
    private static final int[] INDEXI = {4, 2, 0, 4};

    /**
     * AX*SR*T: rotate the vector in VBUF about the three axes in turn. Pass +3 for the NB to
     * SM transformation, -3 for SM to NB. Uses SINCDU / COSCDU (order Y Z X). Safe only for
     * vectors of magnitude less than unity.
     *
     * Each pass rotates two components using sin/cos of one angle: component pair (a, b)
     * becomes (a cos - b sin, b cos + a sin) with the sign of the sine reversed for the
     * opposite direction. The passes run +3,+2,+1 (NBSM) or -3,-2,-1 (SMNB), i.e. axes in
     * opposite orders.
     */
    public void axSrT(int dexdex) {
        agc.set("DEXDEX", Agc.fromSigned(dexdex));
        int d = dexdex;
        int sign = d > 0 ? 1 : -1;
        int mag = Math.abs(d);
        while (mag > 0) {                                   // R*TL**P
            int idx = sign > 0 ? 3 - mag : mag - 1;          // +3->0 -3->2, +2->1, +1->2 / -1->0
            int dex1 = INDEXI[idx];                          // DEX1 offsets into the SINCDU table
            int dex2 = INDEXI[idx + 1];
            agc.set("DEXI", INDEXI[idx]);
            double s = dp(at("SINCDU", dex1)) * 2;   // half-unit back to unit
            double c = dp(at("COSCDU", dex1)) * 2;
            double a = dp(at("VBUF", dex1));                   // component 1
            double b = dp(at("VBUF", dex2));                   // component 2
            // For NBSM the second component's sine product is subtracted, for SMNB added.
            double na = a * c + sign * b * s;
            double nb = b * c - sign * a * s;
            setDp(at("VBUF", dex1), na);
            setDp(at("VBUF", dex2), nb);
            mag--;                                           // DIM DEXDEX
        }
        agc.set("DEXDEX", 0);
    }

    // ---- FLESHPOT / CALCSMSC ----

    /** CALCSMSC: compute XNB into the ECADR XNB (via BANKCALL FLESHPOT -1), return RVQ. */
    public void calcsmsc() {
        fleshpot("XNB");
    }

    /**
     * FLESHPOT: compute the body to stable member transformation matrix XNB (the 18-word
     * matrix at the location given by ECADR) from SINCDU/COSCDU (X Y Z angles):
     *
     * <pre>
     *   row 0: cosY cosZ,       sinZ,         -sinY cosZ
     *   row 1: -cosX cosY sinZ + sinX sinY,  cosX cosZ,  sinY cosX sinZ + sinX cosY
     *   row 2: sinX cosY sinZ + cosX sinY,  -sinX cosZ,  -sinY sinX sinZ + cosX cosY
     * </pre>
     * (the original stores the first three terms of each row from the intermediate products,
     * with the third row's cross-product terms from the VXV entry at word offsets 6-12.)
     */
    public void fleshpot(String xnb) {
        double cx = 2 * dp("COSCDUX");
        double cy = 2 * dp("COSCDUY");
        double cz = 2 * dp("COSCDUZ");
        double sx = 2 * dp("SINCDUX");
        double sy = 2 * dp("SINCDUY");
        double sz = 2 * dp("SINCDUZ");
        double[][] m = new double[3][3];
        m[0][0] = cy * cz;                                   // = COSY COSZ   (offset 0)
        m[0][1] = sz;                                        // = SINZ        (offset 2)
        m[0][2] = -sy * cz;                                  // = -SINY COSZ  (offset 4)
        m[2][0] = -sy * cx + sx * sz * cy;                   // = -SINY COSX + SINX SINZ COSY (14)
        m[2][1] = -sx * cz;                                  // = -SINX COSZ  (16)
        m[2][2] = cx * cy - sx * sy * sz;                    // = COSX COSY - SINX SINY SINZ (20)
        // Rows 1 (offsets 6-12) are the cross product of the other rows (VXV).
        m[1][0] = m[2][1] * m[0][2] - m[2][2] * m[0][1];
        m[1][1] = m[2][2] * m[0][0] - m[2][0] * m[0][2];
        m[1][2] = m[2][0] * m[0][1] - m[2][1] * m[0][0];
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                setDp(at(xnb, 6 * r + 2 * c), m[r][c] / 2);   // half-unit matrix
            }
        }
    }
}
