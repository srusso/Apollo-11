package apollo.luminary099;

import apollo.agc.Agc;

/**
 * Time of free fall (TFF) subroutines (Luminary 099, $$/TFF): TFFCONIC, TFFRP/RA, CALCTFF/CALCTPER and T(X).
 *
 * The original works in scaled fixed point with explicit normalisation counts (X1 = -NR, X2 = -NA) and uses the
 * push-down list as scratch storage (the PDL offsets are noted below). This translation works in doubles and
 * natural units: metres, metres/centisecond, 1/sqrt(mu) in sqrt(cs^2/m^3), flight time in centiseconds, so the
 * Earth/Moon scale levels (B-29/B-27 etc.) are not needed: the caller supplies consistent units.
 * Exceptional outcomes returned as POSMAX in the original (flight time = NEARONE) are returned as {@link #TFF_POSMAX}.
 *
 * Usage: set {@link #rOne}, {@link #vOne}, call {@link #tffConic(double)} with 1/sqrt(mu), then
 * {@link #calcTff(double)} or {@link #calcTper(double)} and/or {@link #tffRpRa()}.
 */
public class TimeOfFreeFall {
    private final Agc agc;

    /** Value returned in place of NEARONE/POSMAX when no valid result exists. */
    public static final double TFF_POSMAX = Double.MAX_VALUE;

    // Constants (TOF-FF1 bank 32)
    /** PI/16 2DEC 3.141592653 B-4 : pi. */
    public static final double PI = 3.141592653;
    /** LIM(-22) = 1.0 - B(-22) ; DP(-22) = B(-22) : indeterminacy threshold for the Z denominator. */
    public static final double DP_M22 = Math.scalb(1.0, -22);
    /** R300K: 6464778 B-29 metres. */
    public static final double R300K = 6464778;
    /** NEARONE. */
    public static final double NEARONE = 0.999999999;

    /** T(X) polynomial coefficients, ascending powers. */
    private static final double[] T_COEFFS = {
        3.333333333e-1, -1.999819135e-1, 1.418148467e-1, -1.01310997e-1, 5.609004986e-2, -1.536156925e-2
    };

    // User supplied state vector (RONE, VONE)
    public double[] rOne = new double[3];
    public double[] vOne = new double[3];

    // Conic parameters left by tffConic (push-list resident in the original)
    public double rMag1;       // RMAG1, present radius
    public double tffNp;       // TFFNP, semi-latus rectum LCP
    public double tffRtMu;     // TFF/RTMU, 1/sqrt(mu)
    public double tffVsq;      // TFFVSQ, -(V^2/mu)
    public double tffAlfa;     // TFFALFA, 1/semi-major axis (signed)
    public double tffRtAlf;    // TFFRTALF, sqrt(|alfa|)
    public double tffOneAlf;   // TFF1/ALF, signed semi-major axis (0 if alfa tiny/zero)
    public double[] vOnePrime = new double[3]; // VONE', V/sqrt(mu)

    // Outputs of tffRpRa
    public double rPer, rApo;

    // Outputs left for entry display routines by calcTff
    public double tffX, tffTem, tffQ1, tffDelQ, rTerm;

    public TimeOfFreeFall(Agc agc) {
        this.agc = agc;
    }

    /**
     * TFFCONIC: enter with 1/sqrt(mu) (stored in TFF/RTMU), then fall through to {@link #tffConMu()}.
     */
    public void tffConic(double oneOverRtMu) {
        tffRtMu = oneOverRtMu;
        tffConMu();
    }

    /**
     * TFFCONMU: compute RMAG, VONE', LCP = |r x v'|^2, ALFA = 2/r - v'.v', sqrt|alfa| and 1/alfa.
     */
    public void tffConMu() {
        rMag1 = norm(rOne);
        double[] ur = scale(rOne, 1.0 / rMag1);
        vOnePrime = scale(vOne, tffRtMu);                       // VN/SQRT(MU)
        double[] h = cross(ur, scale(vOnePrime, rMag1));        // (UR x VN') * R
        tffNp = dot(h, h);                                      // LCP = H.H/MU
        double vsq = dot(vOnePrime, vOnePrime);
        tffVsq = -vsq;                                          // -(V SQ / MU)
        tffAlfa = 2.0 / rMag1 - vsq;                            // ALFA = 2/RN - VN.VN/MU
        tffRtAlf = Math.sqrt(Math.abs(tffAlfa));
        // BZE: if alfa is zero/too small, 1/ALFA is set to 0 to show a small alfa
        tffOneAlf = (tffAlfa == 0.0) ? 0.0 : 1.0 / tffAlfa;
    }

    /**
     * TFFRP/RA: perigee radius RP = P/(1+E), apogee radius RA = (1+E)/ALFA, with E^2 = 1 - P*ALFA.
     * RA = POSMAX when alfa <= 0 (parabola/hyperbola) or the value overflows.
     */
    public void tffRpRa() {
        double e = Math.sqrt(Math.abs(1.0 - tffNp * tffAlfa));  // ABS protects SQRT near e = 0
        rPer = tffNp / (1.0 + e);
        if (tffOneAlf == 0.0) {                                 // MAXRA: ALFA = 0
            rApo = TFF_POSMAX;
            return;
        }
        double ra = (1.0 + e) * tffOneAlf;
        rApo = (ra < 0) ? TFF_POSMAX : ra;                      // negative: not meaningful for hyperbola
    }

    /** CALCTFF: time of free fall from present position to radius rTerm on the inbound side; returns TFF (cs). */
    public double calcTff(double rTermIn) {
        return calcTffOrTper(rTermIn, false);
    }

    /** CALCTPER: time to perigee (enter with perigee radius); returns TFF (cs). */
    public double calcTper(double rPerigee) {
        return calcTffOrTper(rPerigee, true);
    }

    private double calcTffOrTper(double rTermIn, boolean tper) {
        rTerm = rTermIn;                                        // RTERM
        double term = rTerm * (2.0 - tffAlfa * rTerm);          // RTERM(2 - ALFA RTERM)
        double arg = tper ? 0.0 : term - tffNp;                 // for TPER, Q2 = 0
        if (arg < 0) {
            return TFF_POSMAX;                                  // MAXTFF1: no conic to RTERM
        }
        double q2 = -Math.sqrt(arg);                            // RT is on the inbound side
        tffTem = q2;
        tffQ1 = dot(vOnePrime, rOne);                           // Q1 = RN.VN/SQRT(MU)
        double num, den;
        if (tffQ1 < 0) {                                        // INBOUND
            num = rTerm - rMag1;
            den = q2 + tffQ1;
        } else {                                                // OUTBOUND
            num = q2 - tffQ1;
            den = (2.0 - tffAlfa * rTerm) - tffAlfa * rMag1;    // 2 - ALFA(RTERM + RN)
        }
        tffX = num;
        // SAVEDEN: indeterminacy test
        if (Math.abs(den) < DP_M22) {
            if (tffAlfa > 0) {                                  // TFFEL1: assume 90 degrees, D/N = 0
                return tffEll1(0.0);
            }
            return 0.0;                                         // DUMPTFF1: return TFF = 0
        }
        // C(MPAC) = NUM*SQRT(ALFA)/|DEN|; overflow (>= 1) selects the ellipse DELE>=90 equations
        if (Math.abs(num * tffRtAlf / Math.abs(den)) >= 1.0) {
            return tffEll(num, den);
        }
        double z = num / den;                                   // general conic equation
        double zsq = z * z;
        tffTem = tffNp * zsq * Math.signum(z == 0 ? 1 : z);     // P Z Z SGN(SDELF) for entry display
        tffX = tffAlfa * zsq;                                   // X = ALFA Z Z
        double tSqrtMu = (rTerm + rMag1 - 2.0 * zsq * tOfX(tffX)) * z; // TFF SQRT(MU)
        if (tSqrtMu < 0) {
            if (tffQ1 < 0) {
                return endTff(tSqrtMu);                         // NEGTFF: gone past
            }
            if (tffOneAlf <= 0) {
                return endTff(tSqrtMu);                         // ALFA not positive: TFF negative
            }
            // CORRECT FOR ORBITAL PERIOD: add 2 PI / (ALFA SQRT(ALFA))
            tSqrtMu += 2.0 * PI * tffOneAlf / tffRtAlf;
        }
        return endTff(tSqrtMu);
    }

    /** ENDTFF: TFF = TFF SQRT(MU) / SQRT(MU) (cs); POSMAX on overflow. */
    private double endTff(double tSqrtMu) {
        double tff = tSqrtMu * tffRtMu;
        return Double.isInfinite(tff) || Double.isNaN(tff) ? TFF_POSMAX : tff;
    }

    /** TFFELL: ellipse with |DELE| >= 90 degrees. */
    private double tffEll(double num, double den) {
        return tffEll1(den / num);                              // D/N = 1/Z
    }

    /** TFFEL1: enter with D/N = 1/Z. */
    private double tffEll1(double invZ) {
        tffDelQ = tffTem - tffQ1;                               // Q2 - Q1
        double a = tffOneAlf * invZ;                            // 1/(ALFA Z)
        double xx = a * invZ;                                   // X = 1/(ALFA Z Z)
        tffX = xx;
        double term = 2.0 * (xx * tOfX(xx) - 1.0) * a;          // 2(X T(X) - 1)/(Z ALFA)
        double sgn = tffQ1 + rMag1 * invZ;                      // sign of SDELF
        tffTem = Math.copySign(tffNp * tffOneAlf, sgn);         // P/ALFA with sign (arg for TFF/TRIG)
        double inner = PI * Math.sqrt(tffOneAlf) + term - tffDelQ;
        return endTff(tffOneAlf * inner);                       // (PI/SQRT(ALFA) - Q2 + Q1 + term)/ALFA
    }

    /**
     * T(X): polynomial approximation of 1/3 - X/5 + X^2/7 - X^3/9 ... (max deviation 2E-5, valid for -0.08 to 1).
     */
    public static double tOfX(double x) {
        double r = 0;
        for (int i = T_COEFFS.length - 1; i >= 0; i--) {
            r = r * x + T_COEFFS[i];
        }
        return r;
    }

    // --- vector helpers ---
    private static double dot(double[] a, double[] b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
    private static double norm(double[] a) { return Math.sqrt(dot(a, a)); }
    private static double[] scale(double[] a, double k) { return new double[] {a[0] * k, a[1] * k, a[2] * k}; }
    private static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }
}
