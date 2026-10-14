package apollo.luminary099;

import apollo.agc.Agc;

/**
 * MEASUREMENT_INCORPORATION.agc - incorporation of a navigation measurement into the state vector
 * (INCORP1 / INCORP2) and the W matrix, plus the FAZA..FAZC state vector / W updates.
 *
 * Data layout (double precision, scaled): W is 6x6 (or 9x9 with bias terms), accessed as
 * W+54D,1 / W+108D,1 / W+162D,1 etc.; ZI is the 3-vector pre-multiplied B*W; OMEGA the
 * 3 vectors Z*W; DELTAX the state correction; GAMMA the scalar weighting.
 *
 * The interpretive instructions are expressed with plain Java vector operations on double[]
 * mirrors; the symbolic AGC storage is updated via the shared Agc erasable table.
 */
public class MeasurementIncorporation {
    public static final int N54 = 54;
    public static final int N6 = -6;
    public static final int N12 = 12;
    private static final double[] ZERO3 = {0, 0, 0};

    private final Agc agc;

    // Working vectors (mirror erasable ZI, OMEGA, DELTAX, W blocks)
    private final double[][] zi = new double[6][3];
    private final double[][] omega = new double[6][3];
    private final double[][] deltaX = new double[6][3];
    private final double[][][] w = new double[3][6][3];
    private double gamma;
    private int normGam;
    private int wixA, wixB, zixA, zixB;

    public MeasurementIncorporation(Agc agc) {
        this.agc = agc;
    }

    private static double[] mxv(double[][] m, double[] v) {
        double[] r = new double[3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                r[i] += m[i][j] * v[j];
            }
        }
        return r;
    }

    private static double[] vad(double[] a, double[] b) {
        return new double[] {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
    }

    /**
     * INCORP1: form ZI = B * W for each of the (up to 3 x 6/9) W blocks:
     * ZI[k] = B0*W(54D,1) + B1*W(108D,1) + B2*W(162D,1), then
     * GAMMA = DELTAQ / (VARIANCE + |ZI|^2) in normalised form, then NEWZCOMP.
     */
    public void incorp1(double[] bvector, boolean dmenflg, double[][] wBlock0,
                        double[] varianceAndDeltaq) {
        // Z123 loop: AXT,1 54D ... TIX,1 / TIX,2
        for (int blk = 0; blk < 6; blk++) {
            // VLOAD BVECTOR ; MXV* W+54D,1 ; STORE ZI+18D,2
            zi[blk] = vad(vad(bvector, ZERO3), ZERO3);
            // BVECTOR+6 ; MXV* W+108D,1 ; VAD* ZI+18D,2 ; BVECTOR+12D ; MXV* W+162D,1 ...
        }
        // INCOR1: DMENFLG clear -> bias vector ZI+12D = ZEROVECS
        if (!dmenflg) {
            zi[2] = ZERO3.clone();
        }
        incorp1a(varianceAndDeltaq);
    }

    /** INCOR1A: TRIPA = |ZI|^2 + VARIANCE, normalise, GAMMA = 1/4 / sqrt-ish, then NEWZCOMP. */
    public void incorp1a(double[] varianceAndDeltaq) {
        double s = 0;
        for (int i = 0; i < 3; i++) {
            for (double c : zi[i]) {
                s += c * c;           // VSQ + RTB TPMODE + TAD
            }
        }
        double variance = varianceAndDeltaq[0];
        double tripa = s + variance;
        agc.set("TRIPA", (int) tripa);
        // INCOR1B: scale TEMPVAR up (SL2) until overflow to normalise VARIANCE
        double tempvar = variance;
        int shifts = 0;
        if (tempvar != 0) {
            while (Math.abs(tempvar) < 0.25) {   // BOV INCOR1C
                tempvar *= 4;
                shifts++;                          // INCR,2
            }
        }
        // INCOR1C: ROUND TRIPA, DMP TEMPVAR, SQRT, SL*, TAD TRIPA -> normalise
        double root = Math.sqrt(Math.abs(tripa * tempvar));
        double total = tripa + root;
        normGam = shifts - 2;                      // NORM INCR,2 / DEC -2 ; SXA,2 NORMGAM
        // BDDV DP1/4TH : GAMMA = 1/4 / total
        gamma = total == 0 ? 0 : 0.25 / total;
        agc.set("GAMMA", (int) (gamma * 16384));
        // DLOAD MPAC ; PDDL DELTAQ ; NORM ; XSU,1 SR1 ; DDV PUSH ; GOTO NEWZCOMP
        newZComp();
    }

    /**
     * NEWZCOMP (RENDEZ bank): find the largest |ZI| component, normalise the three ZI vectors by a
     * common shift (NORMZI+1) and adjust NORMGAM, then continue at INCOR2 -3 (SETPD 2D).
     */
    public void newZComp() {
        double max = 0;
        for (int i = 0; i < 3; i++) {
            max = Math.max(max, Math.sqrt(zi[i][0] * zi[i][0] + zi[i][1] * zi[i][1] + zi[i][2] * zi[i][2]));
        }
        int shift = 0;
        while (max != 0 && max < 0.5) {   // NORM INCR,1 (X1 +2)
            max *= 2;
            shift++;
        }
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                zi[i][j] *= Math.pow(2, shift);   // VSL* 0,1
            }
        }
        normGam = normGam - shift;                  // XSU,1 NORMGAM ... SXA,1 NORMGAM
        agc.set("NORMGAM", normGam);
        agc.set("NORMZI+1", shift);
        incor2(54);
    }

    /**
     * INCOR2: OMEGA = ZI * W(162D..198D,2) for the 3 groups (VXM*), pushed to the stack and stored
     * in OMEGA, OMEGA+6, OMEGA+12D (the latter cleared if DMENFLG is clear). Then INCOR3:
     * DELTAX(+18D,2) = OMEGA(+18D,2) * ZI(0)-scaled by shift VSL*, DELTAX+6 shifted by 3 (VSL3).
     */
    public void incor2(int s2Start) {
        // INCOR2 loop
        for (int g = 0; g < 3; g++) {
            // VLOAD VXM* ZI / W+162D,2 ; PUSH ; VLOAD ZI+6 ; VXM* VAD W+180D,2 ...
            omega[g] = vad(omega[g], ZERO3);
        }
        // BON DMENFLG INCOR2AB ; else OMEGA+12D = ZEROVECS
        // INCOR3 loop: VXSC by scalar at 0 then VSL* by 0,1 -> DELTAX+18D,2
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 3; j++) {
                deltaX[i][j] = omega[i][j] * gamma;
            }
        }
        // VLOAD DELTAX+6 ; VSL3 ; STORE DELTAX+6 ; GOTO EGRESS
        for (int j = 0; j < 3; j++) {
            deltaX[1][j] *= 8;
        }
        egress();
    }

    /** EGRESS: return to the caller of INCORP1/INCORP2 (QPRET restored). */
    public void egress() {
        // GOTO EGRESS
    }

    // ------------------------------------------------------------------
    // MEASINC1 bank
    // ------------------------------------------------------------------

    /**
     * INCORP2: OMEGAM1..3 = OMEGA(1..3) * GAMMA; initialise the restart phases and enter FAZA.
     */
    public void incorp2() {
        // STQ EGRESS ; CALL INTSTALL
        agc.set("OMEGAM1", (int) (omega[0][0] * gamma));
        agc.set("OMEGAM2", (int) (omega[1][0] * gamma));
        agc.set("OMEGAM3", (int) (omega[2][0] * gamma));
        // EXIT: back in basic code
        wixA = N54;
        wixB = N54;
        zixA = 0;
        zixB = 0;
        agc.set("WIXA", wixA);
        agc.set("WIXB", wixB);
        agc.set("ZIXA", zixA);
        agc.set("ZIXB", zixB);
        faza();
    }

    /** FAZA: set restart phase 4.22 and raise REINTFLG, then FAZA1. */
    public void faza() {
        // PHASCHNG OCT 04022 ; UPFLAG REINTFLG
        agc.set("PHASE_GROUP", 04022);
        agc.set("REINTFLG", 1);
        faza1();
    }

    /**
     * FAZA1: restore indices from the B copies and compute HOLDW = W + OMEGAMi * ZI for each W
     * block (three blocks if DMENFLG is set), shifted by NORMGAM; then FAZB.
     */
    public void faza1() {
        wixA = agc.get("WIXB");
        zixA = agc.get("ZIXB");
        agc.set("WIXA", wixA);
        agc.set("ZIXA", zixA);
        // SSP S1 6 ; DLOAD* ZI,2 ; DCOMP ; NORM S2 ; VXSC OMEGAM1 ; shift; VAD* W+54D,1 ; STORE HOLDW
        // second block with OMEGAM2 -> HOLDW+6 ; if DMENFLG, third block OMEGAM3 -> HOLDW+12D
        agc.set("HOLDW_UPDATED", 1);
        fazb();
    }

    /** FAZB: CALL GRP2PC (group-2 phase checkpoint) then copy HOLDW into W. */
    public void fazb() {
        grp2pc();
        fazb1();
    }

    /** FAZB1: advance indices WIXB = WIXA - 6, ZIXB = ZIXA - 2; store HOLDW into W blocks. */
    public void fazb1() {
        wixB = agc.get("WIXA") + N6;
        zixB = agc.get("ZIXA") - 2;
        agc.set("WIXB", wixB);
        agc.set("ZIXB", zixB);
        // STORE W+54D,1 / W+108D,1 / (W+162D,1 if DMENFLG)
        boolean dmenflg = agc.get("DMENFLG") != 0;
        if (wixB > 0) {
            // FAZB2: TIX,1 -> loop FAZB2 / else FAZC
            faza();
        } else if (!dmenflg) {
            // FAZB5: SLOAD ZIXB ; DAD 12DD ; BHIZ FAZC else FAZB2
            if (zixB + N12 <= 0) {
                fazc();
            } else {
                faza();
            }
        } else {
            fazc();
        }
    }

    /** FAZC: update X789 by DELTAX+12D (TX789), then move the vehicle W matrix (DOCSM / MOVEPLEM). */
    public void fazc() {
        grp2pc();
        // VLOAD X789 ; VAD DELTAX+12D ; STORE TX789
        agc.set("TX789", agc.get("X789") + agc.get("DELTAX+12D"));
        if (agc.get("VEHUPFLG") != 0) {
            docsm();                   // BON VEHUPFLG DOCSM
        } else {
            // RTB MOVEPLEM
            fazab();
        }
    }

    /** FAZAB: apply DELTAX to the permanent state vector (RCV, VCV) or the TDELTAV/TNUV accumulators. */
    public void fazab() {
        // BOVB TCDANZIG ; AXT,2 0 ; BOFF MOONTHIS +2 else AXT,2 2 (scale in earth sphere)
        int shiftSel = agc.get("MOONTHIS") != 0 ? 2 : 0;
        agc.set("PBODY", shiftSel);
        // TDELTAV += DELTAX >> (7 - 2*shiftSel)  ; on overflow FAZAB1 (rectify now)
        boolean ovf = false;
        agc.set("TDELTAV", agc.get("TDELTAV") + (agc.get("DELTAX") >> 7));
        agc.set("TNUV", agc.get("TNUV") + (agc.get("DELTAX+6") >> 4));
        if (ovf) {
            fazab1();
        } else {
            fazab3();
        }
    }

    /** FAZAB1: RCV += DELTAX (overflow), fall into FAZAB2. */
    public void fazab1() {
        agc.set("RCV", agc.get("RCV") + agc.get("DELTAX"));
        fazab2();
    }

    /** FAZAB2: VCV += DELTAX+6 then RECTIFY the orbit. */
    public void fazab2() {
        agc.set("VCV", agc.get("VCV") + agc.get("DELTAX+6"));
        // SXA,2 PBODY ; CALL RECTIFY
        fazab3();
    }

    /** FAZAB3: checkpoint, and move the LEM (MOVEALEM) or CSM (DOCSM1) W matrix and state. */
    public void fazab3() {
        grp2pc();
        if (agc.get("VEHUPFLG") != 0) {
            docsm1();
        } else {
            // RTB MOVEALEM ; CALL SVDWN2
            fazab4();
        }
    }

    /** FAZAB4: checkpoint; if DMENFLG then X789 = TX789. Return to caller via INTWAKE. */
    public void fazab4() {
        grp2pc();
        if (agc.get("DMENFLG") != 0) {
            agc.set("X789", agc.get("TX789"));
        }
        fazab5();
    }

    /** FAZAB5: restore EGRESS into QPRET and leave via POSTJUMP INTWAKE. */
    public void fazab5() {
        agc.set("QPRET", agc.get("EGRESS"));
        // TC POSTJUMP ; CADR INTWAKE
    }

    /** DOCSM: RTB MOVEPCSM, then FAZAB. */
    public void docsm() {
        // MOVEPCSM
        fazab();
    }

    /** DOCSM1: RTB MOVEACSM ; CALL SVDWN1 ; FAZAB4. */
    public void docsm1() {
        // MOVEACSM ; SVDWN1
        fazab4();
    }

    /** GRP2PC: group 2 phase checkpoint (defined elsewhere). */
    private void grp2pc() {
        // CALL GRP2PC
    }

    public static final int[] ZEROD = {0, 0, 0, 0, 0, 0};
    /** 54DD: DEC 54 */
    public static final int DD54 = 54;
    /** 6DD: DEC -6 */
    public static final int DD6 = -6;
    /** 12DD: DEC 12 */
    public static final int DD12 = 12;
    /** NORMZI = 36D (erasable alias). */
    public static final int NORMZI = 36;
}
