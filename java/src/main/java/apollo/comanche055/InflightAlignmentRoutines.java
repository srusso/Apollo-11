package apollo.comanche055;

import apollo.agc.Agc;

/**
 * INFLIGHT_ALIGNMENT_ROUTINES.agc - CALCGTA (gyro torque angles), ARCTRIG, CALCGA (CDU driving
 * angles) and AXISGEN (coordinate-system generation from two star pairs).
 *
 * Angles are fractions of a revolution (range -0.5..+0.5); vectors are half-unit (|v| = 0.5)
 * where the original says so. SINTH/COSTH in the original carry SIN/4, COS/4; here true values are used.
 */
public final class InflightAlignmentRoutines {
    /** QTSN45: sin(45)/4 */
    public static final double QTSN45 = .1768;
    /** .166...: 60 degrees in revolutions. */
    public static final double SIXTY_DEG = .1666666667;
    /** Alarm code for gimbal lock. */
    public static final int GIMBAL_LOCK_ALARM = 00401;

    private final Agc agc;

    /** Desired stable member axes in present SM coordinates (XDC, YDC, ZDC). */
    public double[] xdc = new double[3], ydc = new double[3], zdc = new double[3];
    /** Navigation base and desired SM axes in a common frame (XNB.., XSM..). */
    public double[] xnb = new double[3], ynb = new double[3], znb = new double[3];
    public double[] xsm = new double[3], ysm = new double[3], zsm = new double[3];
    /** Gyro torque angles / gimbal angles (fractions of rev). */
    public double igc, mgc, ogc, theta;
    /** THETAD, THETAD+1, THETAD+2. */
    public double[] thetad = new double[3];
    public boolean cphiflag, glokfail;
    /** STARAD: three 3-vectors (STAR1/XA, STAR2, ...), STARAD, STARAD+6, STARAD+12D. */
    public double[][] starad = new double[3][3];
    /** STAR1 and STAR2 referred to system B (locations 6 and 12D of the VAC area). */
    public double[] star1b = new double[3], star2b = new double[3];

    public InflightAlignmentRoutines(Agc agc) { this.agc = agc; }

    private static double[] cross(double[] a, double[] b) {
        return new double[]{a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]};
    }
    private static double dot(double[] a, double[] b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
    private static double[] unit(double[] v) {
        double n = Math.sqrt(dot(v, v));
        return n == 0 ? v.clone() : new double[]{v[0]/n, v[1]/n, v[2]/n};
    }

    /**
     * CALCGTA: gyro torque angles IGC (Y gyro), MGC (Z gyro), OGC (X gyro) to bring the stable
     * member to the orientation XDC, YDC, ZDC (half-unit vectors).
     */
    public void calcgta() {
        // ZP = UNIT(-XD3 0 XD1)
        double[] zp = unit(new double[]{-xdc[2], 0, xdc[0]});
        igc = arctrig(zp[0], zp[2]);                       // SIN(IGC)=ZP1, COS(IGC)=ZP3
        double sinMg = xdc[1] * 2;                         // SIN(MGC) = XD2 (half-unit -> true)
        double cosMg = (zp[2] * xdc[0] - zp[0] * xdc[2]) * 2; // COS(MGC) = ZP3*XD1 - ZP1*XD3
        mgc = arctrig(sinMg, cosMg);
        double cosOg = dot(zp, zdc) * 2;                   // COS(OGC) = ZP . ZDC
        double sinOg = dot(zp, ydc) * 2;                   // SIN(OGC) = ZP . YDC
        ogc = arctrig(sinOg, cosOg);
    }

    /**
     * ARCTRIG: angle (fraction of rev, -.5..+.5) from its sine and cosine.
     * Uses arccos for angles in (45,135) or (-135,-45), arcsin otherwise (TRIG1/TRIG2).
     */
    public double arctrig(double sin, double cos) {
        double r;
        if (Math.abs(sin) >= Math.sqrt(0.5)) {             // ABS(SIN) - SIN(45) >= 0
            r = Math.acos(clamp(cos)) / (2 * Math.PI);     // X = ARCCOS(COS) with SIGN(SIN)
            if (sin < 0) r = -r;
        } else {                                           // TRIG1
            r = Math.asin(clamp(sin)) / (2 * Math.PI);     // X = ARCSIN(SIN)
            if (cos < 0) {                                 // TRIG2: (135,-135)
                r = (sin < 0 ? -0.5 : 0.5) - r;
            }
        }
        theta = r;
        return r;
    }

    private static double clamp(double x) { return Math.max(-1, Math.min(1, x)); }

    /**
     * CALCGA: CDU driving angles (THETAD..THETAD+2) required to bring the stable member into
     * the orientation XSM,YSM,ZSM given navigation base axes XNB,YNB,ZNB.
     */
    public void calcga() {
        double[] mga = unit(cross(xnb, ysm));              // MGA = UNIT(OGA X IGA), OGA=XNB, IGA=YSM
        ogc = arctrig(dot(mga, ynb) * 2, dot(mga, znb) * 2);   // SIN(OG)=MGA.YNB, COS(OG)=MGA.ZNB
        double cosMg = dot(ysm, cross(mga, xnb)) * 2 * 2;      // COS(MG) = IGA . (MGA X OGA)
        double sinMg = dot(ysm, xnb) * 2;                      // SIN(MG) = IGA . OGA
        mgc = arctrig(sinMg, cosMg / 2);
        if (Math.abs(mgc) - SIXTY_DEG >= 0) {                  // ABS(MG) > 60 deg
            gimlock1(mga);
            return;
        }
        calcga1(mga);
    }

    /** CALCGA1: inner gimbal angle, then store THETAD (OGC, IGC, MGC order per V1STO2S). */
    public void calcga1(double[] mga) {
        igc = arctrig(dot(xsm, mga) * 2, dot(zsm, mga) * 2);   // SIN(IG)=XSM.MGA, COS(IG)=ZSM.MGA
        // RTB V1STO2S; BONCLR CPHIFLAG: THETAD = (OGC IGC MGC) as 2's complement
        thetad[0] = ogc;
        thetad[1] = igc;
        thetad[2] = mgc;
        cphiflag = false;
    }

    /** GIMLOCK1: ALARM 00401, UPFLAG GLOKFAIL, then continue at CALCGA1. */
    public void gimlock1(double[] mga) {
        AlarmAndAbort alarm = new AlarmAndAbort(agc);
        alarm.alarm(GIMBAL_LOCK_ALARM);
        glokfail = true;                                       // TC UPFLAG ; ADRES GLOKFAIL
        calcga1(mga);
    }

    /**
     * AXISGEN: given two stars referred to system A (starad[0], starad[1]) and to system B
     * (star1b, star2b), computes coordinate system A referred to B as three half-unit vectors
     * left in xdc/ydc/zdc and starad[0..2].
     */
    public void axisgen() {
        double[][] a = triad(starad[0], starad[1]);            // UA, VA, WA
        double[][] b = triad(star1b, star2b);                  // UB, VB, WB
        double[][] out = {xdc, ydc, zdc};
        for (int i = 0; i < 3; i++) {                          // AXISGEN2 loop over X, Y, Z
            double[] v = new double[3];
            for (int k = 0; k < 3; k++) {
                v[k] = a[0][i] * b[0][k] + a[1][i] * b[1][k] + a[2][i] * b[2][k];  // L + J + K
            }
            double[] u = unit(v);
            for (int k = 0; k < 3; k++) out[i][k] = u[k] * 0.5;     // STORE half-unit
        }
        for (int i = 0; i < 3; i++) starad[i] = out[i].clone();
    }

    /** AXISGEN1: builds (U = S1, V = UNIT(S1 X S2), W = U X V). */
    private double[][] triad(double[] s1, double[] s2) {
        double[] u = s1.clone();
        double[] v = unit(cross(s1, s2));
        double[] w = cross(u, v);
        return new double[][]{u, v, w};
    }
}
