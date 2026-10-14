package apollo.comanche055;

import apollo.agc.Agc;

/**
 * MEASUREMENT_INCORPORATION.agc (Comanche 055).
 * INCORP1 computes Z = B*W partitions, the scalar GAMMA and OMEGA vectors and DELTAX from one measurement
 * (BVECTOR, VARIANCE, DELTAQ). INCORP2 then updates the 6 or 9 dimensional W matrix in phases (FAZA, FAZB, FAZC),
 * incorporates the state-vector correction DELTAX (FAZAB) and restarts via phase changes.
 *
 * Interpretive vector arithmetic is done in doubles; the AGC scaling shifts (SL, SR, NORM) are not reproduced
 * numerically, only noted where they define the data flow. The W matrix is modelled as 9x9 in 3x3 blocks,
 * Z/OMEGA/DELTAX as three 3-vectors (ZI, Z2, Z3).
 */
public class MeasurementIncorporation {
    private final Agc agc;

    /** W: 9x9 (or 6x6 when DMENFLG set) covariance factor. */
    public final double[][] w = new double[9][9];
    /** BVECTOR: three 3-vectors (B(0), B(1), B(2)). */
    public final double[][] bvector = new double[3][3];
    /** ZI, Z2, Z3. */
    public final double[][] zi = new double[3][3];
    public final double[][] omega = new double[3][3];
    public final double[][] omegam = new double[3][3];
    public final double[][] deltax = new double[3][3];
    public final double[][] holdw = new double[3][9];
    public double variance, deltaq, gamma, tripa, tempvar, normzi;
    public double[] x789 = new double[3];
    public double[] tx789 = new double[3];
    public boolean dmenflg; // 9 vs 6 dimensional (ON = 6 dim: Z3 zeroed)
    public boolean vehupflg;
    public boolean moonthis;
    public int wixa, wixb, zixa, zixb;

    /** 54DD, 6DD, 12DD constants. */
    public static final int C54 = 54, C6 = -6, C12 = 12;
    public static final double[] ZEROVECS = {0, 0, 0};

    public MeasurementIncorporation(Agc agc) {
        this.agc = agc;
    }

    private double[] mxv(double[] v, int rowBlock, int colBlock) {
        double[] r = new double[3];
        for (int c = 0; c < 3; c++) {
            for (int k = 0; k < 3; k++) {
                r[c] += v[k] * w[rowBlock * 3 + k][colBlock * 3 + c];
            }
        }
        return r;
    }

    private static double[] vad(double[] a, double[] b) {
        return new double[] {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
    }

    private static double[] vxsc(double[] a, double s) {
        return new double[] {a[0] * s, a[1] * s, a[2] * s};
    }

    private static double vsq(double[] a) {
        return a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
    }

    /** INCORP1: Z123 loop, GAMMA, DELTAQ/A, OMEGA1..3, DELTAX. */
    public void incorp1() {
        // Z123: Z_j = B(0)*W + B(1)*W' + B(2)*W'' for j = 1..3
        for (int j = 0; j < 3; j++) {
            double[] z = mxv(bvector[0], 0, j);
            z = vad(z, mxv(bvector[1], 1, j));
            z = vad(z, mxv(bvector[2], 2, j));
            zi[j] = z;
        }
        if (dmenflg) {
            zi[2] = ZEROVECS.clone(); // INCOR1: 6-dimensional case zeros Z3
        }
        // INCOR1A: TRIPA = |Z1|^2 + |Z2|^2 + |Z3|^2 + VARIANCE
        tripa = vsq(zi[0]) + vsq(zi[1]) + vsq(zi[2]) + variance;
        // INCOR1B/1C: normalise VARIANCE (shift left while no overflow); GAMMA = 1/4 / (A + sqrt(A*V))
        tempvar = variance;
        double a = tripa;
        gamma = 0.25 / (a + Math.sqrt(a * tempvar));
        double normA = a; // NORM TRIPA -> X1
        double dqOverA = deltaq / normA; // DELTAQ/A pushed to PD
        newzcomp();
        // INCOR2: OMEGA_j = Z1*W + Z2*W' + Z3*W'' (block row products)
        for (int j = 0; j < 3; j++) {
            double[] o = mxv(zi[0], 0, j);
            o = vad(o, mxv(zi[1], 1, j));
            o = vad(o, mxv(zi[2], 2, j));
            omega[j] = o;
        }
        if (dmenflg) {
            omega[2] = ZEROVECS.clone(); // INCOR2AB
        }
        // INCOR3: DELTAX_j = OMEGA_j * (DELTAQ/A), shifted
        for (int j = 0; j < 3; j++) {
            deltax[j] = vxsc(omega[j], dqOverA);
        }
        // VSL3 on DELTAX+6 then GOTO EGRESS
        deltax[1] = vxsc(deltax[1], 8);
    }

    /** NEWZCOMP / NEWZCMP1: find the largest |Zi| and normalise Z (shift) -- scale only, no value change. */
    private void newzcomp() {
        double m = Math.sqrt(vsq(zi[0]));
        m = Math.max(m, Math.sqrt(vsq(zi[1])));
        m = Math.max(m, Math.sqrt(vsq(zi[2])));
        normzi = m; // NORMZI = largest ABVAL; shift count saved in NORMZI+1 and folded into NORMGAM
    }

    /** INCORP2: GAMMA*OMEGA, then phased W update (FAZA..FAZC) and state incorporation (FAZAB). */
    public void incorp2() {
        // INTSTALL
        for (int j = 0; j < 3; j++) {
            omegam[j] = vxsc(omega[j], gamma);
        }
        wixa = wixb = C54;
        zixa = zixb = 0;
        // FAZA: PHASCHNG 04022; UPFLAG REINTFLG
        fazaLoop();
        fazc();
    }

    /** FAZA/FAZA1/FAZB/FAZB1: update the W matrix partitions from OMEGAM and Z, 3 rows per pass. */
    private void fazaLoop() {
        int passes = dmenflg ? 6 : 9;
        for (int row = 0; row < passes; row += 1) {
            // FAZA1: W(row-block) += -Z(row) (x) OMEGAM_j
            int i = row % 3;
            int zj = row / 3;
            double zComp = zi[zj][i];
            for (int j = 0; j < 3; j++) {
                for (int c = 0; c < 3; c++) {
                    int rb = row; // row within W
                    if (rb < 9 && j * 3 + c < 9) {
                        w[rb][j * 3 + c] -= zComp * omegam[j][c];
                    }
                }
            }
            // FAZB: GRP2PC; FAZB1 transfers HOLDW into W permanently (modelled as in-place update)
            wixb = wixa - 6;
            zixb = zixa - 2;
        }
    }

    /** FAZC: state vector incorporation phase 3 (X789), then FAZAB position/velocity increments. */
    private void fazc() {
        // GRP2PC; TX789 = X789 + DELTAX+12D
        tx789 = vad(x789, deltax[2]);
        if (vehupflg) {
            docsm();
        } else {
            movepLem();
        }
        fazab();
    }

    /** DOCSM: MOVEPCSM then FAZAB. */
    private void docsm() {
        // RTB MOVEPCSM
    }

    private void movepLem() {
        // RTB MOVEPLEM
    }

    /** FAZAB: add DELTAX to TDELTAV/TNUV (or RCV/VCV after overflow, then RECTIFY). */
    private void fazab() {
        // BOVB TCDANZIG ; scale shifts: 7 (moon) / 7 (earth) for position, 4 for velocity
        // VLOAD DELTAX; VSR* -7,2; VAD TDELTAV ; ... overflow -> RCV += DELTAX, VCV += DELTAX+6; RECTIFY
        // FAZAB3: GRP2PC; CALL SVDWN1/SVDWN2 (downlink state vector); FAZAB4: GRP2PC
        if (!dmenflg) {
            x789 = tx789.clone(); // 9-dimensional: X789 = TX789
        }
        // FAZAB5: EGRESS via POSTJUMP INTWAKE
    }

    /** DOCSM1: MOVEACSM, SVDWN1, then FAZAB4. */
    public void docsm1() {
        // RTB MOVEACSM ; CALL SVDWN1 ; GOTO FAZAB4
    }
}
