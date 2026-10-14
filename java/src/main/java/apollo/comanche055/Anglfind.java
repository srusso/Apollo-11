package apollo.comanche055;

import apollo.agc.Agc;

/**
 * ANGLFIND.agc (Comanche055): KALCMAN3 and its subroutines for the automatic maneuver
 * (KALCMANU) angle finding: direction cosine matrices from CDU angles, the single rotation
 * axis and angle between two attitudes, the rotation matrix DEL, and the inverse
 * (DCM to CDU angles).
 *
 * <p>Interpretive-language vector/matrix work is done here with Java doubles.
 * Matrices are 3x3 stored row-major as double[9] (element n corresponds to the AGC double-precision
 * word offset 2n from the matrix base). Angles are in revolutions (CDU angle scaled 2 pi); the
 * AGC scale factors ($1, $2, $4, ...) are therefore not applied: values are true values.
 * CDU angle vectors are {X, Y, Z} = {OGA (phi), IGA (theta), MGA (psi)}.
 */
public final class Anglfind {
    private final Agc agc;

    // Constants
    /** MINANG: minimum maneuver angle, .00069375 revolutions (0.25 degrees). */
    public static final double MINANG = 0.00069375;
    /** MAXANG: .472222 revolutions (170 degrees). */
    public static final double MAXANG = 0.472222;
    /** Gimbal lock constants: D = MGA of gimbal lock = 60 degrees, NGL = buffer angle = 2 degrees. */
    public static final double SD = 0.433015;      // sin(D)               $2
    public static final double K3S1 = 0.86603;     // sin(D)               $2
    public static final double K4 = -0.25;         // -cos(D)              $2
    public static final double K4SQ = 0.125;       // cos(D)cos(D)         $2
    public static final double SNGLCD = 0.008725;  // sin(NGL)cos(D)       $2
    public static final double CNGL = 0.499695;    // cos(NGL)             $2

    // Working storage (erasable in the original; BCDU etc.)
    /** BCDU: initial S/C CDU angles (revolutions). */
    public double[] bcdu = new double[3];
    /** CPHI: final (commanded) CDU angles (revolutions). */
    public double[] cphi = new double[3];
    public double[] mis = new double[9];
    public double[] mfs = new double[9];
    public double[] tmis = new double[9];
    public double[] mfi = new double[9];
    public double[] tmfi = new double[9];
    public double[] mfisym = new double[9];
    public double[] cofskew = new double[3];
    public double[] cof = new double[3];
    public double[] del = new double[9];
    public double cam;
    public double am;

    private static final java.util.Map<Agc, Anglfind> INSTANCES = new java.util.WeakHashMap<>();

    /** Shared per-AGC instance so the working storage (MIS, MFS, DEL, ...) is common to all callers. */
    public static synchronized Anglfind of(Agc agc) {
        return INSTANCES.computeIfAbsent(agc, Anglfind::new);
    }

    public Anglfind(Agc agc) {
        this.agc = agc;
    }

    /** READCDUK: loads the current CDU angles (X=CDUX, Y=CDUY, Z=CDUZ) in revolutions. */
    public double[] readcduk() {
        // INHINT; CA CDUZ; TS MPAC+2; DCA CDUX; RELINT
        double[] v = new double[3];
        v[0] = cduToRev(agc.get("CDUX"));
        v[1] = cduToRev(agc.get("CDUY"));
        v[2] = cduToRev(agc.get("CDUZ"));
        return v;
    }

    /** Converts a 15-bit CDU counter to signed revolutions (-0.5 .. 0.5). */
    private static double cduToRev(int word) {
        int w = word & Agc.WORD_MASK;
        double rev = w / 32768.0;
        return rev >= 0.5 ? rev - 1.0 : rev;
    }

    /**
     * KALCMAN3: compute the rotation axis COF and angle AM from the current attitude to CPHI,
     * then continue to LOCSKIRT (gimbal lock check) in KALCMANU_STEERING.
     */
    public void kalcman3() {
        bcdu = readcduk();                       // RTB READCDUK; STORE BCDU
        mis = cdutodcm(bcdu);                    // MIS: initial S/C axes to stable member axes
        mfs = cdutodcm(cphi);                    // MFS: final S/C axes to stable member axes
        secad();
    }

    /** SECAD: MIS and MFS arrays calculated. */
    public void secad() {
        tmis = transpos(mis);                    // TMIS = transpose(MIS), scaled 2
        mfi = mxm3(tmis, mfs);                   // MFI = TMIS MFS (scaled 4)
        tmfi = transpos(mfi);                    // TMFI = transpose(MFI)

        // CALCULATE COFSKEW AND MFISYM
        cofskew = new double[] {
            tmfi[1] - mfi[1],
            mfi[2] - tmfi[2],
            tmfi[5] - mfi[5]
        };                                       // COF scaled 2/sin(AM); equals MFISKEW

        // CALCULATE AM AND PROCEED ACCORDING TO ITS MAGNITUDE
        cam = (mfi[0] + mfi[4] + mfi[8] - 1.0) / 2.0;   // CAM = (MFI0+MFI4+MFI8-1)/2
        am = Math.acos(Math.max(-1.0, Math.min(1.0, cam))) / (2.0 * Math.PI);   // AM = ARCCOS(CAM), revolutions
        if (am - MINANG < 0) {
            // MANEUVER LESS THAN 0.25 DEG: go directly into attitude hold about commanded angles
            agc.set("HOLDFLAG", Agc.fromSigned(-1));   // CS ONE; TS HOLDFLAG
            // INHINT; TC LOADCDUD
            new KalcmanuSteering(agc).loadcdud();
            nogo();                                    // TCF NOGO (NOGO stops any rate, set up for a good return)
            return;
        }
        checkmax();
    }

    /** CHECKMAX: AM versus MAXANG decides whether the axis is found from the skew part or the symmetric part. */
    public void checkmax() {
        if (am - MAXANG < 0) {
            cof = unit(cofskew);                 // COF is the maneuver axis
            new GimbalLockAvoidance(agc).locskirt(cof, am);   // see if maneuver goes through gimbal lock
        } else {
            altcalc();
        }
    }

    /** ALTCALC: if AM is greater than 170 degrees. */
    public void altcalc() {
        for (int i = 0; i < 9; i++) {
            mfisym[i] = (mfi[i] + tmfi[i]) / 2.0;   // MFISYM = (MFI + TMFI)/2, scaled 4
        }
        // CALCULATE COF
        double denom = 1.0 - cam;
        double cofz = Math.sqrt(Math.max(0, (mfisym[8] - cam)) / denom);
        double cofy = Math.sqrt(Math.max(0, (mfisym[4] - cam)) / denom);
        double cofx = Math.sqrt(Math.max(0, (mfisym[0] - cam)) / denom);
        cof = unit(new double[] {cofx, cofy, cofz});
        cofmaxgo();
    }

    /** COFMAXGO: determine largest COF component and adjust signs accordingly. */
    public void cofmaxgo() {
        if (cof[0] - cof[1] < 0) {               // COFY greater than COFX
            comp12();
            return;
        }
        if (cof[0] - cof[2] < 0) {               // COFZ greater than COFX or COFY
            method3();
        } else {
            method1();                           // COFX greater than COFY or COFZ
        }
    }

    private void comp12() {
        if (cof[1] - cof[2] < 0) {
            method3();                           // COFZ greater than COFY or COFX
        } else {
            method2();
        }
    }

    /** METHOD2: COFY is the maximum. */
    public void method2() {
        if (cofskew[1] < 0) {                    // UY
            negate(cof);
        }
        if (mfisym[1] < 0) {                     // sign of UX opposite to UY
            cof[0] = -cof[0];
        }
        if (mfisym[5] < 0) {                     // sign of UZ opposite to UY
            cof[2] = -cof[2];
        }
        new GimbalLockAvoidance(agc).locskirt(cof, am);
    }

    /** METHOD1: COFX is the maximum. */
    public void method1() {
        if (cofskew[0] < 0) {                    // UX
            negate(cof);
        }
        if (mfisym[1] < 0) {                     // sign of UY opposite to UX
            cof[1] = -cof[1];
        }
        if (mfisym[2] < 0) {                     // sign of UZ opposite to UX
            cof[2] = -cof[2];
        }
        new GimbalLockAvoidance(agc).locskirt(cof, am);
    }

    /** METHOD3: COFZ is the maximum. */
    public void method3() {
        if (cofskew[2] < 0) {                    // UZ
            negate(cof);
        }
        if (mfisym[2] < 0) {                     // sign of UX opposite to UZ
            cof[0] = -cof[0];
        }
        if (mfisym[5] < 0) {                     // sign of UY opposite to UZ
            cof[1] = -cof[1];
        }
        new GimbalLockAvoidance(agc).locskirt(cof, am);
    }

    // ---------------------------------------------------------------------------------------
    // MATRIX OPERATIONS
    // ---------------------------------------------------------------------------------------

    /** MXM3: multiplies two 3x3 matrices (m1 * m2); result is the product matrix. */
    public static double[] mxm3(double[] m1, double[] m2) {
        double[] r = new double[9];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                double s = 0;
                for (int k = 0; k < 3; k++) {
                    s += m1[3 * i + k] * m2[3 * k + j];
                }
                r[3 * i + j] = s;
            }
        }
        return r;
    }

    /** TRANSPOS / TRNSPSPD: transposes a 3x3 matrix. */
    public static double[] transpos(double[] m) {
        double[] r = new double[9];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                r[3 * j + i] = m[3 * i + j];
            }
        }
        return r;
    }

    // ---------------------------------------------------------------------------------------
    // CDUTODCM
    // ---------------------------------------------------------------------------------------

    /**
     * CDUTODCM: direction cosine matrix relating S/C axes to stable member axes from the
     * three CDU angles {phi=OGA, theta=IGA, psi=MGA} (revolutions).
     */
    public static double[] cdutodcm(double[] cdu) {
        double sphi = Math.sin(2 * Math.PI * cdu[0]);
        double cphi = Math.cos(2 * Math.PI * cdu[0]);
        double sth = Math.sin(2 * Math.PI * cdu[1]);
        double cth = Math.cos(2 * Math.PI * cdu[1]);
        double spsi = Math.sin(2 * Math.PI * cdu[2]);
        double cpsi = Math.cos(2 * Math.PI * cdu[2]);
        double[] c = new double[9];
        c[0] = cth * cpsi;
        c[1] = -cth * spsi * cphi + sth * sphi;
        c[2] = cth * spsi * sphi + sth * cphi;
        c[3] = spsi;
        c[4] = cpsi * cphi;
        c[5] = -cpsi * sphi;
        c[6] = -sth * cpsi;
        c[7] = sth * spsi * cphi + cth * sphi;
        c[8] = -sth * spsi * sphi + cth * cphi;
        return c;
    }

    // ---------------------------------------------------------------------------------------
    // DELCOMP
    // ---------------------------------------------------------------------------------------

    /**
     * DELCOMP: DEL = (IDMATRIX)cos(A) + U U^T (1-cos(A)) + U x sin(A), scaled 1, where U (COF) is
     * the unit rotation axis and A (angle, revolutions) is the rotation angle.
     * Result left in {@link #del}.
     */
    public void delcomp(double angleRev) {
        double a = 2 * Math.PI * angleRev;
        double s = Math.sin(a);
        double c = Math.cos(a);
        double oneMinusC = 1.0 - c;
        // diagonal components
        del[0] = cof[0] * cof[0] * oneMinusC + c;   // UX UX (1-COS(A)) + COS(A)
        del[4] = cof[1] * cof[1] * oneMinusC + c;   // UY UY (1-COS(A)) + COS(A)
        del[8] = cof[2] * cof[2] * oneMinusC + c;   // UZ UZ (1-COS(A)) + COS(A)
        // off diagonal terms
        double xy = cof[0] * cof[1] * oneMinusC;
        double uzs = cof[2] * s;
        del[3] = xy + uzs;
        del[1] = xy - uzs;
        double xz = cof[0] * cof[2] * oneMinusC;
        double uys = cof[1] * s;
        del[2] = xz + uys;      // UX UZ (1-COS(A)) + UY SIN(A)
        del[6] = xz - uys;      // UX UZ (1-COS(A)) - UY SIN(A)
        double yz = cof[1] * cof[2] * oneMinusC;
        double uxs = cof[0] * s;
        del[7] = yz + uxs;      // UY UZ (1-COS(A)) + UX SIN(A)
        del[5] = yz - uxs;      // UY UZ (1-COS(A)) - UX SIN(A)
    }

    // ---------------------------------------------------------------------------------------
    // DCMTOCDU
    // ---------------------------------------------------------------------------------------

    /**
     * DCMTOCDU: direction cosine matrix to CDU angle routine. Returns {phi, theta, psi} in revolutions
     * (CDU angles scaled 2 pi). cos(MGA) is left in S1 (field {@link #s1}).
     *
     * <pre>
     * C0 = cos(THETA)cos(PSI)                     C1 = -cos(THETA)sin(PSI)cos(PHI)+sin(THETA)sin(PHI)
     * C2 = cos(THETA)sin(PSI)sin(PHI)+sin(THETA)cos(PHI)   C3 = sin(PSI)
     * C4 = cos(PSI)cos(PHI)                       C5 = -cos(PSI)sin(PHI)
     * C6 = -sin(THETA)cos(PSI)                    C7 = sin(THETA)sin(PSI)cos(PHI)+cos(THETA)sin(PHI)
     * C8 = -sin(THETA)sin(PSI)sin(PHI)+cos(THETA)cos(PHI)
     * where PHI = OGA, THETA = IGA, PSI = MGA
     * </pre>
     */
    public double[] dcmtocdu(double[] c) {
        double psi = Math.asin(clamp(c[3])) / (2 * Math.PI);          // ARCSIN C3
        s1 = Math.cos(2 * Math.PI * psi);                             // COS(MGA)
        double theta = Math.asin(clamp(-c[6] / s1)) / (2 * Math.PI);
        if (c[0] < 0) {                                               // check sign of cos(THETA) for quadrant
            theta = (-theta >= 0) ? -theta - 0.5 : -theta + 0.5;
        }
        double phi = Math.asin(clamp(-c[5] / s1)) / (2 * Math.PI);
        if (c[4] < 0) {
            phi = (-phi >= 0) ? -phi - 0.5 : -phi + 0.5;
        }
        return new double[] {phi, theta, psi};                        // VDEF
    }

    /** S1: COS(MGA) left by DCMTOCDU. */
    public double s1;

    // ---------------------------------------------------------------------------------------
    // NOGO
    // ---------------------------------------------------------------------------------------

    /** NOGOM2: accessed by a BZMF NOGO -2. */
    public void nogom2() {
        // INHINT; TC BANKCALL; CADR ZEROEROR
        new KalcmanuSteering(agc).zeroerror();
        nogo();
    }

    /** NOGO: routine for terminating automatic maneuvers. */
    public void nogo() {
        // INHINT
        new KalcmanuSteering(agc).stoprate();      // terminate maneuver
        // CAF TWO; TC WAITLIST; 2CADR ENDMANU : all returns now made via GOODEND
        new Waitlist(agc).waitlist(2, new KalcmanuSteering(agc)::endmanu);
        // TCF ENDOFJOB
    }

    // ---------------------------------------------------------------------------------------
    // helpers
    // ---------------------------------------------------------------------------------------
    private static double clamp(double v) {
        return Math.max(-1.0, Math.min(1.0, v));
    }

    private static void negate(double[] v) {
        for (int i = 0; i < v.length; i++) {
            v[i] = -v[i];
        }
    }

    private static double[] unit(double[] v) {
        double n = Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (n == 0) {
            return v.clone();
        }
        return new double[] {v[0] / n, v[1] / n, v[2] / n};
    }
}
