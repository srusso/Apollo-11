package apollo.comanche055;

import apollo.agc.Agc;

/**
 * CSM_GEOMETRY.agc - sextant/navigation-base geometry (SXTNB, CALCSXA, SXTANG),
 * state vector deviation summing (SVDWN1/2), natural log (LOG) and Earth rotator (EARROT1/2).
 *
 * The interpretive-language code is expressed with doubles (angles in revolutions, vectors as
 * double[3]); the scaling comments from the original (B-n) are retained where relevant.
 */
public final class CsmGeometry {
    /** 10DEGS-: DEC 3600 (CDU counts correction for 19.775 degree offset). */
    public static final int TEN_DEGS_MINUS = 3600;
    /** 270DEG: shaft 270 degrees, 2's complement DP. */
    public static final int[] DEG270 = {060000, 0};
    /** 20DEGS-: DEC -07199, -00000. */
    public static final int[] DEGS20_MINUS = {-7199, 0};
    /** 20DEG-: DEC 3600, 0. */
    public static final int[] DEG20_MINUS = {3600, 0};
    /** 1/WIE: 2DEC 8616410 (centiseconds per revolution of Earth rate). */
    public static final double ONE_OVER_WIE = 8616410;
    /** CLOG2/32: 2DEC .0216608494 */
    public static final double CLOG2_32 = .0216608494;
    /** NEARLY1: 2DEC .999999999 */
    public static final double NEARLY1 = .999999999;

    /** NB2NB1 matrix (rows), scaled B-1. */
    public static final double[][] NB2NB1 = {
        {0.8431756920, 0, -0.5376381241},
        {0, 0.5, 0},
        {0.5376381241, 0, 0.8431756920}};
    /** NB1NB2 matrix (rows), scaled B-1. */
    public static final double[][] NB1NB2 = {
        {0.8431756920, 0, 0.5376381241},
        {0, 0.5, 0},
        {-0.5376381241, 0, 0.8431756920}};
    /** ZERINFLT / HALFNFLT constants. */
    public static final double ZERINFLT = 0, HALFNFLT = 0.5;

    private static final double TWO_PI = 2 * Math.PI;

    private final Agc agc;

    /** Interpretive working variables (erasable symbols of the same names). */
    public double[] star = new double[3], starm = new double[3];
    public double[] xnb = new double[3], ynb = new double[3], znb = new double[3];
    public double[] xnb1 = new double[3], ynb1 = new double[3], znb1 = new double[3];
    public double[] rtinit = new double[3], rteast = new double[3], rtnorm = new double[3], rt = new double[3];
    public double[] unitw = {0, 0, 1};
    public double sac, pac, dtearot;
    public boolean cultflag;
    public double[] tdeltav = new double[3], rcv = new double[3], vcv = new double[3];
    public double[] rn = new double[3], vn = new double[3], rOther = new double[3], vOther = new double[3];
    public double[] tnuv = new double[3];
    public double tet, pipTime;
    public boolean avemidsw;
    /** X2 index: 2 if state vector scaled B27/B5 else 0 (B29/B7). */
    public int x2;

    public CsmGeometry(Agc agc) { this.agc = agc; }

    private static double[] mxv(double[][] m, double[] v) {
        double[] r = new double[3];
        for (int i = 0; i < 3; i++) r[i] = m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2];
        return r;
    }
    private static double[] scale(double[] v, double k) { return new double[]{v[0]*k, v[1]*k, v[2]*k}; }
    private static double[] add(double[] a, double[] b) { return new double[]{a[0]+b[0], a[1]+b[1], a[2]+b[2]}; }
    private static double dot(double[] a, double[] b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
    private static double[] cross(double[] a, double[] b) {
        return new double[]{a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]};
    }
    private static double[] unit(double[] v) {
        double n = Math.sqrt(dot(v, v));
        return n == 0 ? v : scale(v, 1 / n);
    }

    /**
     * SXTNB: converts sextant shaft and trunnion angles (revolutions) into a half-unit line-of-sight
     * vector in navigation-base coordinates. Result stored at 32D (returned and in starm).
     */
    public double[] sxtnb(double shaftRev, double trunnionRev) {
        double ta = sxtlogic(trunnionRev);             // TRUNNION = TA (corrected for 19.775 deg offset)
        double sa = shaftRev * TWO_PI;
        double sinTa = Math.sin(ta * TWO_PI);          // SIN(TA)
        double cosSa = Math.cos(sa), sinSa = Math.sin(sa);
        double cosTa = Math.cos(ta * TWO_PI);
        starm[0] = cosSa * sinTa;                      // COS(SA)SIN(TA)
        starm[1] = sinSa * sinTa;                      // SIN(SA)SIN(TA)
        starm[2] = cosTa;
        double[] r = scale(mxv(NB1NB2, starm), 2);     // MXV NB1NB2 ; VSL1
        System.arraycopy(r, 0, starm, 0, 3);
        return r;                                      // STORE 32D ; RVQ
    }

    /** SXTLOGIC: correct trunnion for 19.775 degree offset, scaled by QUARTER. */
    public double sxtlogic(double trunnionRev) {
        // CAF 10DEGS-; ADS MPAC; CAF QUARTER; TC SHORTMP (multiply by 1/4); TC DANZIG
        // 10DEGS- = 3600 counts; modelled here as an offset in revolutions (approximate scaling).
        return trunnionRev + TEN_DEGS_MINUS / 131072.0;
    }

    /**
     * CALCSXA: computes sextant shaft and trunnion angles required to point the optics along STAR.
     * Inputs: star (present stable member coordinates), CDU sines/cosines (via smnb).
     * Outputs: sac, pac.
     */
    public void calcsxa(double[][] smnbMatrix) {
        star = scale(mxv(NB2NB1, mxv(smnbMatrix, star)), 2);   // CALL *SMNB* ; MXV NB2NB1 ; VSL1
        xnb1 = new double[]{1, 0, 0};                           // HIUNITX
        ynb1 = new double[]{0, 1, 0};                           // HIUNITY
        znb1 = new double[]{0, 0, 1};                           // HIUNITZ
        sxtang1();
    }

    /** SXTANG: same, for NB axes xnb, ynb, znb in the same frame as star. */
    public void sxtang() {
        xnb1 = scale(mxv(NB2NB1, xnb), 2);
        ynb1 = scale(mxv(NB2NB1, ynb), 2);
        znb1 = scale(mxv(NB2NB1, znb), 2);
        sxtang1();
    }

    /** SXTANG1: core of the shaft and trunnion angle computation. */
    public void sxtang1() {
        double[] c = cross(znb1, star);
        double n = Math.sqrt(dot(c, c));
        if (n == 0) { znbEqS1(); return; }                      // ZNB=S1: S along ZNB
        double[] pda = unit(c);                                 // PDA = UNIT(ZNB X S)
        double sinth = -dot(pda, xnb1);                         // SIN(SA) = PDA . -XNB
        double costh = dot(pda, ynb1);                          // COS(SA) = PDA . YNB
        double sa = Math.atan2(sinth, costh);                   // ARCTRIG
        if (sa < 0) sa += TWO_PI;
        sac = sa / TWO_PI;                                      // 1STO2S -> SAC
        double ct = dot(star, znb1) * 2;                        // DOT SL1
        double trunnion = Math.acos(Math.max(-1, Math.min(1, ct))) / TWO_PI;
        // trunnion angle negative cannot occur with acos; > 90 deg handled below
        if (trunnion > 0.25) { sxtalarm(); return; }
        pac = trunnion - DEG20_MINUS[0] / 131072.0;             // DSU 20DEG-
        cultflag = false;                                       // CLRGO CULTFLAG
    }

    /** SXTALARM: trunnion out of range; set CULTFLAG (alarm raised later by MPI). */
    public void sxtalarm() {
        cultflag = true;                                        // SETGO CULTFLAG
    }

    /** ZNB=S1: star along the Z navigation-base axis; SAC = 270 deg, PAC = 20DEGS-. */
    public void znbEqS1() {
        sac = 0.75;                                             // 270DEG
        pac = DEGS20_MINUS[0] / 16384.0;                        // 20DEGS-
        cultflag = false;                                       // CLRGO CULTFLAG
    }

    /**
     * SVDWN1: compute actual CSM(LM) state vector from conic R,V plus deviations.
     * Returns without action if AVEMIDSW is set (BOF/RVQ).
     */
    public void svdwn1() {
        if (avemidsw) return;                                   // BOF RVQ AVEMIDSW
        double sh = Math.pow(2, x2);
        rn = scale(add(scale(tdeltav, Math.pow(2, -7 + x2 * 0)), rcv), sh);
        vn = scale(add(scale(tnuv, Math.pow(2, -4)), vcv), sh);
        pipTime = tet;
    }

    /** SVDWN2: same for the other vehicle; result to R-OTHER, V-OTHER. */
    public void svdwn2() {
        double sh = Math.pow(2, x2);
        rOther = scale(add(scale(tdeltav, Math.pow(2, -7)), rcv), sh);
        vOther = scale(add(scale(tnuv, Math.pow(2, -4)), vcv), sh);
    }

    /**
     * LOG: natural log of the DP value (returns -LOG as in the original, i.e. positive for arg &lt; 1).
     * Shifts the argument into [.5,1), uses a polynomial for the mantissa and adds shiftcount*CLOG2/32.
     */
    public double log(double arg) {
        int n = 0;
        double x = arg;
        while (x < 0.5) { x *= 2; n++; }                         // NORM
        double y = NEARLY1 - x;                                  // BDSU NEARLY1
        double poly = 0 + 0.031335467 * y + 0.0130145859 * y * y + 0.0215738898 * y * y * y; // POLY
        return poly + n * CLOG2_32 ;                             // + shift count * CLOG2/32
    }

    /** EARROT1: first call; resolves RTINIT into east/normal components, then falls to EARROT2. */
    public void earrot1(double dt) {
        dtearot = dt;
        rteast = cross(unitw, rtinit);                           // UNITW X RTINIT
        rtnorm = cross(rteast, unitw);
        earrot2();
    }

    /** EARROT2: rotates the initial target vector about UNITW through WIE*DTEAROT. */
    public void earrot2() {
        double wt = dtearot / ONE_OVER_WIE * TWO_PI;
        if (Math.abs(dtearot / ONE_OVER_WIE) >= 1) { overaday(); return; }   // BOV OVERADAY
        double[] v = add(add(scale(rtnorm, Math.cos(wt) - 1), scale(rteast, Math.sin(wt))), rtinit);
        rt = unit(v);                                            // keep RT 'unit'
    }

    /** OVERADAY: DTEAROT more than one day; reduce modulo 1/WIE and retry. */
    public void overaday() {
        dtearot = dtearot - Math.signum(dtearot) * ONE_OVER_WIE;
        earrot2();
    }
}
