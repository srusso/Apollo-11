package apollo.luminary099;

import apollo.agc.Agc;

/**
 * INFLIGHT_ALIGNMENT_ROUTINES (Luminary 099), bank 22 at INFLIGHT.
 *
 * CALCGTA (gyro torque angles), ARCTRIG, CALCGA (CDU driving angles), AXISGEN.
 * Vectors are half-unit 3-vectors held as double[3]; angles are fractions of a revolution.
 */
public final class InflightAlignmentRoutines {
    private final Agc agc;

    /** 2DEC .1768 : sin(45 deg)/4. */
    public static final double QTSN45 = 0.1768;
    /** 2DEC .1666666667 (label .166...) : 60 degrees expressed in revolutions. */
    public static final double SIXTY_DEG = 0.1666666667;

    // Working storage (SINTH, COSTH, THETA are erasable DP in the original)
    public double sinth;
    public double costh;
    public double theta;

    // Outputs of CALCGTA: gyro torque angles for Y, Z, X gyros (IGC, MGC, OGC), revolutions
    public double igc;
    public double mgc;
    public double ogc;

    // CALCGA output: THETAD, THETAD+1, THETAD+2 (CDU driving angles, 1's complement SP)
    public int[] thetad = new int[3];

    public InflightAlignmentRoutines(Agc agc) {
        this.agc = agc;
    }

    private static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }

    private static double[] unit(double[] a) {
        double n = Math.sqrt(dot(a, a));
        return n == 0 ? a.clone() : new double[] {a[0] / n, a[1] / n, a[2] / n};
    }

    /**
     * CALCGTA: gyro torque angles to bring the stable member to the desired orientation.
     * Inputs: desired stable member coordinates xdc, ydc, zdc (half-unit vectors) referred to present SM.
     * Outputs: IGC (Y gyro), MGC (Z gyro), OGC (X gyro), fractions of a revolution.
     */
    public void calcgta(double[] xdc, double[] ydc, double[] zdc) {
        // ZP = UNIT(-XD3 0 XD1)
        double[] zprime = unit(new double[] {-xdc[2], 0.0, xdc[0]});
        // SIN(IGC) = ZP1 ; COS(IGC) = ZP3 (scaled by 1/4 via SR1 on half-units)
        sinth = zprime[0] / 2;
        costh = zprime[2] / 2;
        arctrig();
        igc = theta; // Y gyro torquing angle
        // SIN(MGC) = XD2 ; COS(MGC) = (ZP3)(XD1) - (ZP1)(XD3)
        sinth = xdc[1] / 2;
        costh = zprime[2] * xdc[0] - zprime[0] * xdc[2];
        arctrig();
        mgc = theta; // Z gyro torquing angle
        // COS(OGC) = ZP . ZDC ; SIN(OGC) = ZP . YDC
        costh = dot(zprime, zdc);
        sinth = dot(zprime, ydc);
        arctrig();
        ogc = theta; // X gyro torquing angle
    }

    /**
     * ARCTRIG: angle (-0.5..+0.5 rev) from SIN/4 (SINTH) and COS/4 (COSTH); result in THETA.
     */
    public double arctrig() {
        if (Math.abs(sinth) - QTSN45 < 0) {
            // TRIG1: (-45,45) or (135,-135)
            theta = Math.asin(Math.max(-1, Math.min(1, sinth * 4))) / (2 * Math.PI);
            if (costh < 0) {
                // TRIG2: (135,-135): .5 with sign(SIN) - arcsin(SIN)
                theta = Math.copySign(0.5, sinth) - theta;
            }
        } else {
            // (45,135) or (-135,-45): X = ARCCOS(COS) with sign(SIN)
            theta = Math.copySign(Math.acos(Math.max(-1, Math.min(1, costh * 4))) / (2 * Math.PI), sinth);
        }
        return theta;
    }

    /**
     * CALCGA: CDU driving angles that bring the stable member into the desired orientation.
     * Inputs: navigation base axes xnb,ynb,znb and desired SM axes xsm,ysm,zsm (any common frame).
     * Outputs: OGC, MGC, IGC and THETAD (1's complement words).
     */
    public void calcga(double[] xnb, double[] ynb, double[] znb, double[] xsm, double[] ysm, double[] zsm) {
        // PD0 = UNIT(OGA X IGA) = MGA, OGA = XNB, IGA = YSM
        double[] mga = unit(cross(xnb, ysm));
        costh = dot(mga, znb);   // COS(OG)
        sinth = dot(mga, ynb);   // SIN(OG)
        arctrig();
        ogc = theta;
        // COS(MG) = IGA . (MGA X OGA) ; SIN(MG) = IGA . OGA
        costh = dot(ysm, cross(mga, xnb)) * 2;
        sinth = dot(ysm, xnb);
        arctrig();
        mgc = theta;
        if (Math.abs(mgc) - SIXTY_DEG >= 0) {
            gimlock1(); // MG angle greater than 60 degrees
        }
        calcga1(mga, xsm, zsm);
    }

    /** CALCGA1: inner gimbal angle. */
    private void calcga1(double[] mga, double[] xsm, double[] zsm) {
        costh = dot(zsm, mga); // COS(IG)
        sinth = dot(xsm, mga); // SIN(IG)
        arctrig();
        igc = theta;
        // V1STO2S converts (IGC, MGC, OGC) to 1's complement; S2 return
        thetad[0] = Agc.fromSigned((int) Math.round(igc * 16384));
        thetad[1] = Agc.fromSigned((int) Math.round(mgc * 16384));
        thetad[2] = Agc.fromSigned((int) Math.round(ogc * 16384));
        for (int i = 0; i < 3; i++) {
            agc.set(i == 0 ? "THETAD" : "THETAD+" + i, thetad[i]);
        }
    }

    /** GIMLOCK1: alarm 00401 and set the GLOKFAIL flag, then continue at CALCGA1. */
    private void gimlock1() {
        new AlarmAndAbort(agc).alarm(0401, 0); // OCT 00401 (caller CADR not modelled)
        agc.set("GLOKFAIL", 1);              // TC UPFLAG ; ADRES GLOKFAIL
    }

    /**
     * AXISGEN: coordinates of one coordinate system referred to another, from two stars.
     * a1,a2 = STAR1/STAR2 in system A (STARAD, STARAD+6); b1,b2 = same stars in system B (VAC+0, VAC+6).
     * Returns {XDC, YDC, ZDC}: system A referred to system B (also left in STARAD, +6, +12D).
     */
    public double[][] axisgen(double[] a1, double[] a2, double[] b1, double[] b2) {
        // AXISGEN1: build orthonormal triad (U = S1, V = UNIT(S1 x S2), W = U x V) in each system
        double[][] a = triad(a1, a2);
        double[][] b = triad(b1, b2);
        // AXISGEN2: each output axis = UNIT of sum over basis of (A basis vector)(component of B basis)
        double[][] out = new double[3][3];
        for (int k = 0; k < 3; k++) {
            double[] r = new double[3];
            for (int j = 0; j < 3; j++) {
                for (int i = 0; i < 3; i++) {
                    r[i] += a[j][i] * b[j][k];
                }
            }
            out[k] = unit(r);
        }
        return out;
    }

    private static double[][] triad(double[] s1, double[] s2) {
        double[] u = s1;
        double[] v = unit(cross(s1, s2));
        double[] w = cross(u, v);
        return new double[][] {u, v, w};
    }
}
