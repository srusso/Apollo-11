package apollo.comanche055;

import apollo.agc.Agc;

/**
 * POWERED_FLIGHT_SUBROUTINES.agc (Comanche 055).
 * CDU trig (CDUTRIG, CD*TR*G, QUICTRIG), SM/NB vector transforms (NBSM, CDU*SMNB, CDU*NBSM, *SMNB*, *NBSM*,
 * AX*SR*T) and SMCDURES (small-angle gimbal-increment resolution).
 *
 * Angles are held as one's-complement fractions of a revolution (CDU 15-bit words); sines and cosines are
 * doubles scaled to +/-1 (the AGC stores them as DP words). Axis ordering in CDUSPOT/SINCDU/COSCDU is
 * Y (index 0), Z (index 1), X (index 2) corresponding to word offsets 0, 2, 4.
 */
public class PoweredFlightSubroutines {
    private final Agc agc;

    /** CDUSPOT: 1's complement CDU angles in turns (Y,Z,X). */
    public final double[] cduspot = new double[3];
    public final double[] sincdu = new double[3];
    public final double[] coscdu = new double[3];
    /** VBUF: vector being transformed (components X,Y,Z). */
    public final double[] vbuf = new double[3];

    /** INDEXI: constants for AX*SR*T. Do not touch. */
    public static final int[] INDEXI = {4, 2, 0, 4};

    public PoweredFlightSubroutines(Agc agc) {
        this.agc = agc;
    }

    /** CDUTRIG: sines/cosines of the current CDU angles, via interpretive return. */
    public void cdutrig() {
        cdutrigs();
    }

    /** CD*TR*G: trig of the angles already in CDUSPOT. */
    public void cdTrG() {
        cdTrGs();
    }

    private static double angleOf(int cduWord) {
        return Agc.toSigned(cduWord) / 16384.0 * 0.5; // fraction of a revolution
    }

    /** CDUTRIGS: load CDUY, CDUZ, CDUX into CDUSPOT then fall into CD*TR*GS. */
    public void cdutrigs() {
        cduspot[0] = angleOf(agc.get("CDUY"));
        cduspot[1] = angleOf(agc.get("CDUZ"));
        cduspot[2] = angleOf(agc.get("CDUX"));
        cdTrGs();
    }

    /** CD*TR*GS: for each of the three angles compute COSCDU and SINCDU (CDULOGIC, COSINE, SINE+1). */
    public void cdTrGs() {
        for (int i = 2; i >= 0; i--) {
            double theta = cduspot[i] * 2 * Math.PI;
            coscdu[i] = Math.cos(theta);
            sincdu[i] = Math.sin(theta);
        }
    }

    /** QUICTRIG: single-precision fast trig (INHINT, uses SPSIN/SPCOS, scaled by BIT14). */
    public void quictrig() {
        for (int i = 2; i >= 0; i--) {
            double theta = cduspot[i] * 2 * Math.PI;
            sincdu[i] = Math.sin(theta) / 2;
            coscdu[i] = Math.cos(theta) / 2;
        }
    }

    /** NBSM: transform the vector in 32D from nav base to stable member using the angles at (S1). */
    public double[] nbsm(double[] angles, double[] vec32d) {
        System.arraycopy(angles, 0, cduspot, 0, 3);
        return trgNbsm(vec32d);
    }

    /** CDU*SMNB: current CDU angles, SM to NB. */
    public double[] cduSmnb(double[] v) {
        cdutrigs();
        return commonSmnb(v);
    }

    /** TRG*SMNB: SM to NB with trig already computed. */
    public double[] trgSmnb(double[] v) {
        cdTrGs();
        return commonSmnb(v);
    }

    /** C*MM*N1: SM to NB transform (signal = -3). */
    private double[] commonSmnb(double[] v) {
        System.arraycopy(v, 0, vbuf, 0, 3);
        return axSrT(-3);
    }

    /** CDU*NBSM: current CDU angles, NB to SM. */
    public double[] cduNbsm(double[] v) {
        cdutrigs();
        return commonNbsm(v);
    }

    /** TRG*NBSM: NB to SM with trig already computed. */
    public double[] trgNbsm(double[] v) {
        cdTrGs();
        return commonNbsm(v);
    }

    /** C*MM*N3: NB to SM transform (signal = +3). */
    private double[] commonNbsm(double[] v) {
        System.arraycopy(v, 0, vbuf, 0, 3);
        return axSrT(3);
    }

    /** *SMNB*: SM to NB with trig and VBUF set. */
    public double[] starSmnbStar() {
        return axSrT(-3);
    }

    /** *NBSM*: NB to SM with trig and VBUF set. */
    public double[] starNbsmStar() {
        return axSrT(3);
    }

    /**
     * AX*SR*T: apply the three axis rotations to VBUF. DEXDEX = +3 (NBSM) or -3 (SMNB); the loop R*TL**P
     * runs three times, picking index pairs from INDEXI. Each pass rotates a pair of vector components
     * (DEX1, DEX2) by sine/cosine of one CDU angle (the NBSM sign is reversed relative to SMNB).
     */
    public double[] axSrT(int dexdex) {
        int signal = dexdex;
        while (true) {
            // CCS DEXDEX; CS DEXDEX; AD THREE: +3->0, +2->1, +1->2, -1->0, -2->1, -3->2
            int idx = signal > 0 ? 3 - signal : 3 + signal;
            if (signal < 0) {
                idx = -signal - 1 == 2 ? 0 : 3 + signal - 0;
                idx = Math.min(Math.max(idx, 0), 2);
            }
            int dex1 = INDEXI[idx];
            int dex2 = INDEXI[idx + 1];
            int p = dex1 / 2;
            int q = dex2 / 2;
            int angle = 3 - p - q; // remaining axis slot selects the gimbal angle for this pass
            // NBSM and SMNB differ by the sign of the sine term
            double sgn = signal > 0 ? 1.0 : -1.0;
            double s = sincdu[Math.floorMod(angle, 3)];
            double c = coscdu[Math.floorMod(angle, 3)];
            double vp = vbuf[p];
            double vq = vbuf[q];
            vbuf[q] = 2 * (c * vq / 2 - sgn * s * vp / 2);
            vbuf[p] = 2 * (c * vp / 2 + sgn * s * vq / 2);
            // DIM DEXDEX: decrement magnitude preserving sign
            signal = signal > 0 ? signal - 1 : signal + 1;
            // TSTPOINT: CCS DEXDEX - only the branching function is used; loop until zero
            if (signal == 0) {
                return vbuf.clone();
            }
        }
    }

    /**
     * SMCDURES: resolve small stable-member angle increments DTHETASM into CDU increments DCDU,
     * using SINCDUY/COSCDUY/SINCDUZ/COSCDUZ.
     */
    public double[] smcdures(double[] dthetasm, double sincduy, double coscduy, double sincduz, double coscduz) {
        double[] dcdu = new double[3];
        // DCDU = (DTHETASM*COSCDUY - DTHETASM+4*SINCDUY) / COSCDUZ
        dcdu[0] = (dthetasm[0] * coscduy - dthetasm[2] * sincduy) / coscduz;
        // DCDU+2 = DTHETASM+2 - SINCDUZ * DCDU (scaled by SL1)
        dcdu[1] = dthetasm[1] - 2 * sincduz * dcdu[0];
        // DCDU+4 = 2*(DTHETASM*SINCDUY + DTHETASM+4*COSCDUY)
        dcdu[2] = 2 * (dthetasm[0] * sincduy + dthetasm[2] * coscduy);
        return dcdu;
    }
}
