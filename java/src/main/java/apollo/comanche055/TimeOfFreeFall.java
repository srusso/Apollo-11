package apollo.comanche055;

import apollo.agc.Agc;

/**
 * TIME_OF_FREE_FALL (Comanche055, pp 1373-1388): TFF subroutines TFFCONIC, TFFRP/RA,
 * CALCTPER/CALCTFF and the T(X) polynomial.
 *
 * The original works in scaled fixed-point interpretive format (with NR/NA normalisation
 * counts kept in X1/X2). This translation computes with unscaled SI doubles: positions in
 * metres, velocities in metres/centisecond, 1/SQRT(MU) in sqrt(cs^2/m^3), times in centiseconds.
 * The comments retain the original scale notes (E = earth origin, M = moon origin).
 * Results are also deposited in the corresponding erasable symbols via the Agc helper.
 *
 * Push-list layout (original): TFFDELQ=10D, RMAG1=12D, TFFQ1=14D (=RPER, CDELF/2),
 * NRTERM=16D (=RAPO), RTERM=18D, TFFVSQ=20D, TFF1/ALF=22D, TFFRTALF=24D, TFFALFA=26D,
 * TFFNP=28D, TFF/RTMU=30D, NRMAG=32D, TFFX=34D, TFFTEM=36D; TFFSW = bit 1 of 119D.
 */
public final class TimeOfFreeFall {
    /** 1/RTMU: modified earth MU, 1/SQRT(MU) at scale B17. */
    public static final double RTMU_INV = .5005750271E-5;
    public static final double PI_16 = 3.141592653;      // PI/16 constant (B-4 scaled)
    public static final double LIM_22 = 1.0 - Math.pow(2, -22);
    public static final double DP_22 = Math.pow(2, -22);
    public static final double DP2_3 = Math.pow(2, -3);
    public static final double DP2_4 = Math.pow(2, -4);
    public static final double RPAD1 = 6373338.0;        // RPAD (earth pad radius), m
    public static final double R300K = 6464778.0;        // m
    public static final double NEARONE = .999999999;
    /** T(X) polynomial coefficients (4th degree in X, N-1 = 4). */
    public static final double[] T_COEFFS = {
        3.333333333E-1, -1.999819135E-1, 1.418148467E-1, -1.01310997E-1, 5.609004986E-2, -1.536156925E-2};

    // Erasable (push list) quantities
    public double tffDelq;      // Q2-Q1
    public double rmag1;        // |RN|
    public double tffQ1;        // R.V / SQRT(MU)
    public double nrterm;
    public double rterm;
    public double tffVsq;       // -(V^2/MU)
    public double tff1Alf;      // semi major axis (signed 1/ALFA); 0 if ALFA small
    public double tffRtAlf;     // sqrt(|ALFA|)
    public double tffAlfa;      // ALFA (1/m)
    public double tffNp;        // semi latus rectum
    public double tffRtMu;      // 1/SQRT(MU)
    public double nrmag;
    public double tffX;
    public double tffTem;
    public double rper;
    public double rapo;
    public double[] vonePrime = new double[3];   // VONE' = VN/SQRT(MU)
    public boolean tffsw;       // false = CALCTFF, true = CALCTPER

    private final Agc agc;

    public TimeOfFreeFall(Agc agc) {
        this.agc = agc;
    }

    /** TFFCONIC: enter with 1/SQRT(MU) in MPAC; save it in TFF/RTMU then TFFCONMU. */
    public void tffconic(double rtMuInv, double[] rone, double[] vone) {
        tffRtMu = rtMuInv;                                  // E:(17) M:(14)
        tffconmu(rone, vone);
    }

    /** TFFCONMU: compute conic parameters (ALFA, LCP, RMAG...) with TFF/RTMU already loaded. */
    public void tffconmu(double[] rone, double[] vone) {
        double rmag = norm(rone);
        rmag1 = rmag;                                       // E:(-29) M:(-27)
        nrmag = rmag;                                       // normalised by NR in original
        for (int i = 0; i < 3; i++) {
            vonePrime[i] = vone[i] * tffRtMu;               // VN/SQRT(MU) E:(10) M:(9)
        }
        double[] ur = {rone[0] / rmag, rone[1] / rmag, rone[2] / rmag};
        double[] h = cross(scale(ur, rmag), vonePrime);     // H = RN x VN / sqrt(MU)
        tffNp = dot(h, h);                                  // LCP = H.H / MU  semi latus rectum
        double v2 = dot(vonePrime, vonePrime);
        tffVsq = -v2;                                       // -(V SQ/MU)  E:(20) M:(18)
        tffAlfa = 2 / rmag - v2;                            // ALFA = 2/RN - VN.VN/MU
        tffRtAlf = Math.sqrt(Math.abs(tffAlfa));            // SQRT(ABS(ALFA))
        double alfa2 = tffAlfa * tffAlfa;
        // SIGN / BZE: 1/ALFA set to 0 to show small ALFA
        tff1Alf = (tffAlfa == 0) ? 0 : 1 / tffAlfa;
        // (dumpcnic: RVQ)
    }

    /** TFFRP/RA: perigee and apogee radii for a general conic (RPER, RAPO). */
    public void tffrpRa() {
        double e2 = Math.abs(1 - tffAlfa * tffNp);          // E SQ = 1 - P ALFA
        double e = Math.sqrt(e2);
        double onePlusE = 1 + e;
        rper = tffNp / onePlusE;                            // RP = P/(1+E)
        if (tff1Alf == 0) {                                 // ALFA = 0: MAXRA
            rapo = NEARONE;
            return;
        }
        double ra = onePlusE * tff1Alf;                     // RA = (1+E)/ALFA
        if (Double.isInfinite(ra) || ra < 0) {
            rapo = NEARONE;                                 // MAXRA: set POSMAX as a tag
        } else {
            rapo = ra;
        }
    }

    /** CALCTPER: time to perigee. Enter with RPER in MPAC. Sets TFFSW. */
    public double calctper(double rper, double[] rone) {
        tffsw = true;
        return calctffCommon(rper, rone);
    }

    /** CALCTFF: free-fall time to RTERM. Enter with RTERM (E:(-29) M:(-27)) in MPAC. Returns TFF in cs. */
    public double calctff(double rterm, double[] rone) {
        tffsw = false;
        return calctffCommon(rterm, rone);
    }

    private double calctffCommon(double rtermIn, double[] rone) {
        rterm = rtermIn;
        nrterm = rtermIn;
        double den0 = 2 - tffAlfa * rterm;                  // (2 - ALFA RTERM)
        double rad = rterm * den0 - tffNp;                  // RTERM(2-ALFA RTERM) - LCP
        double q2sq = tffsw ? 0 : rad;                      // TPER: Q2 = 0
        if (q2sq < 0) {
            return maxtff1();                               // no free fall conic to RTERM from here
        }
        double q2 = -Math.sqrt(q2sq);                       // RT is on the inbound side
        tffTem = q2;
        tffQ1 = dot(vonePrime, rone);                       // Q1 = RN.VN/SQRT(MU), save for GONEPAST
        double num;
        double den;
        if (tffQ1 >= 0 || true) {
            if (q2 - tffQ1 <= 0 && tffQ1 < 0) {
                // INBOUND: NUM = RTERM - RN, DEN = Q2 + Q1
                num = rterm - rmag1;
                den = q2 + tffQ1;
            } else {
                // OUTBOUND: NUM = Q2 - Q1, DEN = 2 - ALFA(RTERM+RN)
                num = q2 - tffQ1;
                den = den0 - tffAlfa * rmag1;
            }
        }
        tffX = num;
        double absDen = Math.abs(den);
        if (absDen < DP_22) {
            // SAVEDEN: DEN indeterminate; set DEN = 0 (Z indeterminate at DELE/2 = 0 and 90)
            return tffAlfa >= 0 ? tffel1(0) : 0;            // DUMPTFF1: return TFF = 0
        }
        double z = num / den;                               // Z = NUM / DEN
        if (Math.abs(tffAlfa * z * z) >= 1.0) {
            return tffell(num, den);                        // ELLIPSE with DEL ECC ANOM >= 90 deg
        }
        tffTem = tffNp * z * z * Math.signum(z);            // LCP Z Z SGN(SDELF)
        double zsq = z * z;
        tffX = tffAlfa * zsq;                               // X = ALFA Z Z
        double tffSqrtMu = (rterm + rmag1 - 2 * zsq * tx(tffX)) * z;   // TFF SQRT(MU)
        if (tffSqrtMu < 0) {                                // BPL ENDTFF
            // GONEPAST test: negative
            if (tffQ1 >= 0 || tff1Alf < 0) {
                return endtff(tffSqrtMu);                   // NEGTFF
            }
            tffSqrtMu += 2 * Math.PI * Math.abs(tff1Alf) / tffRtAlf; // correct for orbital period
        }
        return endtff(tffSqrtMu);
    }

    /** ENDTFF: TFF = TFF SQRT(MU) * TFF/RTMU... overflow gives POSMAX (MAXTFF). */
    private double endtff(double tffSqrtMu) {
        double tff = tffSqrtMu * tffRtMu;
        if (Double.isNaN(tff) || Math.abs(tff) >= 1.0 / DP2_3 * 1e12) {
            return maxtff();
        }
        agc.set("TFFX", Agc.fromSigned((int) (tffX * 16384) & 0x3FFF));
        return tff;
    }

    /** MAXTFF1 / MAXTFF: no conic to RTERM; return POSMAX (NEARONE). */
    private double maxtff1() {
        return maxtff();
    }

    private double maxtff() {
        return NEARONE;
    }

    /** TFFEL1: ellipse with DEL ECC ANOM >= 90 or <= -90. D/N passed in (0 if indeterminate). */
    private double tffel1(double dOverN) {
        tffDelq = tffTem - tffQ1;                          // Q2 - Q1
        tffTem = dOverN;
        double alfaZ = tff1Alf * dOverN;                   // 1/ALFA Z... (as in original)
        tffX = tffAlfa == 0 ? 0 : (alfaZ == 0 ? 0 : 1 / (tffAlfa * dOverN * dOverN));
        double xTx = tffX * tx(tffX);
        return endtff(0 + xTx * 0);                        // returns 0 when D/N = 0 (DUMPTFF1)
    }

    /** TFFELL: time of flight for ellipse when DEL(ECCENTRIC ANOM) >= 90 or <= -90. */
    private double tffell(double num, double den) {
        double dOverN = den / num;                          // D/N (N/D in PDL)
        tffDelq = tffTem - tffQ1;                           // Q2 - Q1
        double alfaZ = tff1Alf * dOverN;                    // 1/ALFA Z
        double oneOverZ = dOverN;
        tffX = 1.0 / (tffAlfa * (1 / (dOverN * dOverN))) ; // X = 1/ALFA Z Z
        double xtx = tffX * tx(Math.min(tffX, 1.0));
        double term = 2 * (xtx - 1) / (1 / alfaZ);          // 2(X T(X) - 1)/Z ALFA
        double sgn = tffQ1 + rmag1 * oneOverZ;              // (Q1 + R/Z) = sign of SDELF
        tffTem = tffNp * tff1Alf * Math.signum(sgn);        // P/ALFA with sign (arg for TFF/TRIG)
        double pi = Math.PI * Math.sqrt(Math.abs(tff1Alf));
        double tffSqrtMu = (pi + term - tffDelq) * tff1Alf; // (PI/SQRT(ALFA) - Q2 + Q1 + ...)/ALFA
        return endtff(tffSqrtMu);
    }

    /**
     * T(X): polynomial approximation to 1/3 - X/5 + X^2/7 - X^3/9 ...,
     * fitted over (-.08, +1) with max deviation 2E-5. Original: TC POLY, N-1 = 4.
     */
    public static double tx(double x) {
        double r = 0;
        for (int i = T_COEFFS.length - 1; i >= 0; i--) {
            r = r * x + T_COEFFS[i];
        }
        return r;
    }

    private static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    private static double norm(double[] a) {
        return Math.sqrt(dot(a, a));
    }

    private static double[] scale(double[] a, double k) {
        return new double[] {a[0] * k, a[1] * k, a[2] * k};
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }
}
