package apollo.luminary099;

import apollo.agc.Agc;

/**
 * ATTITUDE_MANEUVER_ROUTINE.agc - KALCMANU: computes the single-axis (coefficient-of-rotation)
 * maneuver from the present attitude (CDU angles, matrix MIS) to the desired attitude (CPHI/CTHETA/CPSI,
 * matrix MFS). Result: unit axis COF, total angle AM, and the incremental rotation matrix KEL.
 *
 * Matrices are 3x3 stored row-wise in 9 doubles (original: 9 DP words, row stride 6). Angles are
 * fractions of a revolution (AGC CDU units / 2PI), vectors are double[3].
 */
public class AttitudeManeuverRoutine {
    /** MINANG: 2DEC 0.00069375 (revolutions) - below this the maneuver is skipped. */
    public static final double MINANG = 0.00069375;
    /** MAXANG: 2DEC 0.472222222 - above this use the alternate (symmetric) axis computation. */
    public static final double MAXANG = 0.472222222;
    public static final double SD = 0.433015;
    public static final double K3S1 = 0.86603;
    public static final double K4 = -0.25;
    public static final double K4SQ = 0.125;
    public static final double SNGLCD = 0.008725;
    public static final double CNGL = 0.499695;
    /** LOCKANGL: DEC .388889 (gimbal lock limit, ~70 deg of 180). */
    public static final double LOCKANGL = 0.388889;
    public static final double DP1_4TH = 0.25;
    public static final double DPHALF = 0.5;

    /** Alarm code for gimbal lock. */
    public static final int ALARM_GIMBAL_LOCK = 0401;

    private final Agc agc;

    /** MIS: present-to-stable-member matrix; MFS: desired matrix; MFI, TMIS, TMFI, MFISYM, KEL. */
    public double[] mis = new double[9];
    public double[] mfs = new double[9];
    public double[] tmis = new double[9];
    public double[] mfi = new double[9];
    public double[] tmfi = new double[9];
    public double[] mfisym = new double[9];
    public double[] kel = new double[9];
    public double[] cofskew = new double[3];
    public double[] cof = new double[3];
    public double cam;
    public double am;

    public AttitudeManeuverRoutine(Agc agc) {
        this.agc = agc;
    }

    // ------------------------------------------------------------------
    // Matrix helpers
    // ------------------------------------------------------------------

    /** MXM3: c = a * b (3x3 matrices, row-major). */
    public static double[] mxm3(double[] a, double[] b) {
        double[] c = new double[9];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                double s = 0;
                for (int k = 0; k < 3; k++) {
                    s += a[3 * i + k] * b[3 * k + j];
                }
                c[3 * i + j] = s;
            }
        }
        return c;
    }

    /** TRANSPOS / TRNSPSPD: transpose of a 3x3 matrix. */
    public static double[] transpose(double[] a) {
        double[] t = new double[9];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                t[3 * j + i] = a[3 * i + j];
            }
        }
        return t;
    }

    private static double[] unit(double[] v) {
        double n = Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        return n == 0 ? v.clone() : new double[] {v[0] / n, v[1] / n, v[2] / n};
    }

    /** CDU word (15-bit one's complement, 2PI = 2^14 counts... full scale 1 rev = 2^15) -> revolutions. */
    private static double cduRevs(int word) {
        return Agc.toSigned(word) / 16384.0 / 2.0 * 2.0;   // CDULOGIC: scaled fraction of 2PI
    }

    private static final double TWO_PI = 2 * Math.PI;

    // ------------------------------------------------------------------
    // KALCMAN3
    // ------------------------------------------------------------------

    /**
     * KALCMAN3: read the CDUs (BCDU), check middle gimbal angle against LOCKANGL, build MIS from the
     * current gimbal angles and MFS from the desired angles (CPHI,CTHETA,CPSI), form MFI = MIS^T*MFS
     * then compute the rotation axis COF and angle AM.
     */
    public void kalcMan3() {
        // RTB READCDUK ; STORE BCDU
        readCduK();
        double cpsiAbs = Math.abs(cduRevs(agc.get("CPSI")));
        if (cpsiAbs - LOCKANGL >= 0) {          // DSU LOCKANGL ; BPL TOOBADF
            tooBadF();
            return;
        }
        mis = cduToDcm(new int[] {agc.get("BCDU"), agc.get("BCDU+1"), agc.get("BCDU+2")});
        mfs = cduToDcm(new int[] {agc.get("CPHI"), agc.get("CTHETA"), agc.get("CPSI")});
        secAd();
    }

    /** SECAD: form TMIS = transpose(MIS), MFI = TMIS * MFS, TMFI = transpose(MFI), COFSKEW, CAM, AM. */
    public void secAd() {
        tmis = transpose(mis);
        mfi = mxm3(tmis, mfs);
        tmfi = transpose(mfi);
        // COFSKEW = skew terms: (TMFI[1,2]-MFI[1,2], MFI[2,0]-TMFI[2,0], TMFI[2,1]-MFI[2,1]) using layout offsets +2,+4,+10D
        cofskew = new double[] {
            tmfi[1] - mfi[1] == 0 ? 0 : tmfi[1] - mfi[1],
            mfi[2] - tmfi[2],
            tmfi[5] - mfi[5]
        };
        // CAM = (MFI[0] + MFI[8] - 1/4 + MFI[4]) ... AGC: (MFI + MFI+16D) - 1/4 + MFI+8D, scaled 1/4
        cam = (mfi[0] + mfi[8] - DP1_4TH + mfi[4]) ;
        am = Math.acos(Math.max(-1, Math.min(1, cam * 2 / 1.0 - 1))) / TWO_PI * 2;
        if (am - MINANG < 0) {
            // TLOAD CPHI ; STCALL CDUXD ; TOOBADI : maneuver too small -> just set CDUXD
            agc.set("CDUXD", agc.get("CPHI"));
            tooBadI();
            return;
        }
        checkMax();
    }

    /** CHECKMAX: if AM > MAXANG use ALTCALC (symmetric axis), else COF = unit(COFSKEW) then LOCSKIRT. */
    public void checkMax() {
        if (am - MAXANG >= 0) {
            altCalc();
            return;
        }
        cof = unit(cofskew);
        locSkirt();
    }

    /**
     * ALTCALC: near 180 degrees the skew vector vanishes; instead use the symmetric part
     * MFISYM = (MFI + TMFI)/2 and compute COF components as sqrt((MFISYM_ii - cos(AM)) / (1 - cos(AM)))
     * (with CAM/2 scaling), then fix signs per COFMAXGO / METHOD1-3.
     */
    public void altCalc() {
        for (int i = 0; i < 9; i++) {
            mfisym[i] = (mfi[i] + tmfi[i]) / 2.0;
        }
        double half = cam / 2.0;
        double denom = DPHALF - cam;
        double[] c = new double[3];
        int[] diag = {8, 4, 0};      // order of components generated: MFISYM+16D, +8D, +0
        for (int k = 0; k < 3; k++) {
            double q = denom == 0 ? 0 : (mfisym[diag[k]] - half) / denom;
            c[k] = Math.sqrt(Math.max(q, 0));
        }
        // VDEF assembles (z,y,x) order -> COF = (x, y, z)
        cof = unit(new double[] {c[2], c[1], c[0]});
        cofMaxGo();
    }

    /** COFMAXGO: choose the largest component of COF and fix the signs by METHOD1/2/3. */
    public void cofMaxGo() {
        if (cof[0] - cof[1] < 0) {          // COF < COF+2 ?
            if (cof[1] - cof[2] < 0) {
                method3();
            } else {
                method2();                  // COMP12 path
            }
            return;
        }
        if (cof[0] - cof[2] < 0) {
            method3();
        } else {
            method1();
        }
    }

    /** METHOD2: y component largest. */
    public void method2() {
        if (cofskew[1] < 0) {               // U2POS test
            cof = new double[] {-cof[0], -cof[1], -cof[2]};
        }
        if (mfisym[1] >= 0) {               // OKU21
            // keep
        } else {
            cof[0] = -cof[0];
        }
        if (mfisym[5] < 0) {
            cof[2] = -cof[2];
        }
        locSkirt();
    }

    /** METHOD1: x component largest. */
    public void method1() {
        if (cofskew[0] < 0) {
            cof = new double[] {-cof[0], -cof[1], -cof[2]};
        }
        if (mfisym[1] < 0) {
            cof[1] = -cof[1];
        }
        if (mfisym[2] < 0) {
            cof[2] = -cof[2];
        }
        locSkirt();
    }

    /** METHOD3: z component largest. */
    public void method3() {
        if (cofskew[2] < 0) {
            cof = new double[] {-cof[0], -cof[1], -cof[2]};
        }
        if (mfisym[2] < 0) {
            cof[0] = -cof[0];
        }
        if (mfisym[5] < 0) {
            cof[1] = -cof[1];
        }
        locSkirt();
    }

    /** LOCSKIRT: continues in the main KALCMANU routine (outside this file, rate/time computation). */
    public void locSkirt() {
        // GOTO LOCSKIRT in KALCMANU_STEERING / the main maneuver file
    }

    // ------------------------------------------------------------------
    // Support routines
    // ------------------------------------------------------------------

    /** READCDUK: read CDUZ, CDUX (DCA CDUX gives CDUY,CDUX pairs) into BCDU(+0..2). */
    public void readCduK() {
        agc.set("MPAC+2", agc.get("CDUZ"));
        agc.set("MPAC", agc.get("CDUX"));
        agc.set("MPAC+1", agc.get("CDUX+1"));
        agc.set("BCDU", agc.get("MPAC"));
        agc.set("BCDU+1", agc.get("MPAC+1"));
        agc.set("BCDU+2", agc.get("MPAC+2"));
    }

    /**
     * CDUTODCM: direction cosine matrix from three gimbal angles (outer, inner, middle =
     * A=CDUX, B=CDUY... in AGC order [10D,1] [12D,1] [14D,1]) using the standard
     * sin/cos formulas stored at 0..16D.
     */
    public static double[] cduToDcm(int[] cdu) {
        // cdu[0]=OGA (X), cdu[1]=IGA (Y), cdu[2]=MGA (Z); converted to radians
        double so = Math.sin(Agc.toSigned(cdu[0]) * Math.PI / 16384.0);
        double co = Math.cos(Agc.toSigned(cdu[0]) * Math.PI / 16384.0);
        double si = Math.sin(Agc.toSigned(cdu[1]) * Math.PI / 16384.0);
        double ci = Math.cos(Agc.toSigned(cdu[1]) * Math.PI / 16384.0);
        double sm = Math.sin(Agc.toSigned(cdu[2]) * Math.PI / 16384.0);
        double cm = Math.cos(Agc.toSigned(cdu[2]) * Math.PI / 16384.0);
        // AGC layout (rows): 
        //  [ cos(Y) cos(Z),   -cos(Y) sin(Z) ... ] arranged exactly as in the original stores:
        double[] m = new double[9];
        m[0] = ci * cm;                         // STORE 0,2
        m[1] = -co * sm * ci + so * si;         // 2,2 : -(cos O sin M cos I) + ...  (BDSU)
        m[2] = so * sm * ci + co * si;          // 4,2
        m[3] = sm;                              // 6,2 : STORE 6,2 = 8D
        m[4] = co * cm;                         // 8D,2
        m[5] = -so * cm;                        // 10D,2
        m[6] = -si * cm;                        // 12D,2
        m[7] = co * sm * si + so * ci;          // 14D,2
        m[8] = -so * sm * si + co * ci;         // 16D,2
        return m;
    }

    /**
     * DELCOMP: given the angle in the pushdown (half-angle increment) compute the incremental
     * rotation matrix KEL about the unit axis COF:
     * KEL = cos*I + (1-cos)*COF*COF^T + sin*skew(COF)  (AGC scaling /4 in various places).
     */
    public void delComp(double angle) {
        double s = Math.sin(angle);
        double c = Math.cos(angle);
        double d = 1 - c;
        double x = cof[0];
        double y = cof[1];
        double z = cof[2];
        kel[0] = c + d * x * x;
        kel[1] = d * x * y + s * z;      // KEL+2 : BDSU
        kel[2] = d * x * z - s * y;      // KEL+4
        kel[3] = d * x * y - s * z;      // KEL+6
        kel[4] = c + d * y * y;          // KEL+8D
        kel[5] = d * y * z + s * x;      // KEL+10D
        kel[6] = d * x * z + s * y;      // KEL+12D
        kel[7] = d * y * z - s * x;      // KEL+14D
        kel[8] = c + d * z * z;          // KEL+16D
    }

    /**
     * DCMTOCDU: convert direction cosine matrix (rows in MIS-style layout) to CDU angles
     * (outer, inner, middle) = (PHI-like, THETA-like, PSI-like) in revolutions; includes the
     * +/- half-revolution correction ("SUHALFA"/"SUHALFAP") when cos(middle) &lt; 0 quadrant.
     *
     * @return {outer, inner, middle} gimbal angles in radians/(2PI) (revolutions)
     */
    public static double[] dcmToCdu(double[] m) {
        double sinMiddle = m[3];
        double middle = Math.asin(Math.max(-1, Math.min(1, sinMiddle)));
        double cosMiddle = Math.cos(middle);
        double inner = Math.asin(Math.max(-1, Math.min(1, -m[6] / cosMiddle)));
        if (m[0] < 0) {                     // OKTHETA: quadrant fix
            inner = inner >= 0 ? Math.PI - inner : -Math.PI - inner;
        }
        double outer = Math.asin(Math.max(-1, Math.min(1, -m[5] / cosMiddle)));
        if (m[4] < 0) {                     // OKPHI
            outer = outer >= 0 ? Math.PI - outer : -Math.PI - outer;
        }
        return new double[] {outer / TWO_PI, inner / TWO_PI, middle / TWO_PI};
    }

    // ------------------------------------------------------------------
    // Error exits
    // ------------------------------------------------------------------

    /** TOOBADF: gimbal lock - ALARM 00401, ZATTEROR, then NOGO. */
    public void tooBadF() {
        new AlarmAndAbort(agc).alarm(ALARM_GIMBAL_LOCK, agc.get("Q"));
        // BANKCALL ZATTEROR (zero attitude error)
        agc.set("ZATTEROR_CALLED", 1);
        noGo();
    }

    /** NOGO: STOPRATE (stop commanded rates), schedule MANUSTOP-like GOODMANU after 2 ticks, end job. */
    public void noGo() {
        agc.set("STOPRATE_CALLED", 1);
        // INHINT ; WAITLIST (2 ticks) GOODMANU ; ENDOFJOB
        agc.set("WAITLIST_GOODMANU", 2);
    }

    /** TOOBADI: no maneuver required -> NOGO. */
    public void tooBadI() {
        noGo();
    }
}
