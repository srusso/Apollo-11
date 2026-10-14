package apollo.luminary099;

import apollo.agc.Agc;

/**
 * CONIC_SUBROUTINES.agc - KEPLER, LAMBERT, TIMETHET (time-theta), TIMERAD (time-radius) and
 * APSIDES, plus the common lower-rank routines PARAM, GEOM, GETX, DELTIME, NEWSTATE, ITERATOR
 * and CHECKCTR (Colossus 88 / Luminary 099, Krause and Robertson).
 *
 * The inverse-square-force problem is solved with universal variables: x (sqrt(meters)),
 * xi = alpha*x^2, and the Stumpff functions S(xi), C(xi). The AGC code evaluates S and C by
 * polynomial (TC POLY, coefficient tables kept below as {@link #S_POLY}/{@link #C_POLY});
 * this translation uses the equivalent closed forms/series in double precision. Quantities are
 * kept in physical units (meters, meters/centisecond, centiseconds, central-body MU in
 * m^3/cs^2) rather than the AGC power-of-2 scaling; the original scale factors are noted in
 * comments. Index X1 = -2D (Earth) or -10D (Moon) selects the MUTABLE row.
 * Vectors are double[3]. Interpretive switches are boolean fields.
 */
public final class ConicSubroutines {
    private final Agc agc;

    // ---- constants (CONICS1 bank) ----
    public static final double D1_8 = 1.0 / 8, D1_128 = 1.0 / 128, D1_64 = 1.0 / 64, D1_4 = 1.0 / 4,
            D1_16 = 1.0 / 16, D1_32 = 1.0 / 32, D1_1024 = 1.0 / 1024, D1_256 = 1.0 / 256;
    public static final double DP9_10 = .9;
    public static final double KEPZERO = 0.0;
    public static final double MINUS_50SC = -50.0;           // -50 B-12
    public static final double TWO_PI_SC = 6.28318530;       // 2PISC (B-6)
    public static final double BEE19 = Math.pow(2, -19), BEE22 = Math.pow(2, -22), ONEBIT = Math.pow(2, -28);
    public static final double COGUPLIM = .999511597, COGLOLIM = -.999511597;
    /** ABTCONIC alarm. */
    public static final int ALARM_CONIC = 00607;

    /** DELTIME polynomial coefficients for S(XI) and C(XI) (kept for reference). */
    public static final double[] S_POLY = {.083333334, -.266666684, .406349155, -.361198675, .210153242,
            -.086221951, .026268812, -.006163316, .001177342, -.000199055};
    public static final double[] C_POLY = {.031250001, -.166666719, .355555413, -.406347410, .288962094,
            -.140117894, .049247387, -.013081923, .002806389, -.000529414};
    /** GETX series coefficients for the polynomial in R1A * (1/W)^2. */
    public static final double[] GETX_POLY = {.5, -.166666770, .100000392, -.071401086, .055503292,
            -.047264098, .040694204};

    /** MUTABLE table: {MU, ...} in m^3/cs^2 for the Earth (row -2D) and the Moon (row -10D). */
    public static final double MU_EARTH = 3.986032e14 / 1.0e4;
    public static final double MU_MOON = 4.902778e12 / 1.0e4;

    // ---- erasable ----
    /** Central body: true = Moon (X1 = -10D), false = Earth (X1 = -2D). */
    public boolean moon = false;
    public double mu() { return moon ? MU_MOON : MU_EARTH; }
    private double rootMu() { return Math.sqrt(mu()); }

    // KEPLER
    public double[] rrect = new double[3], vrect = new double[3], rcv = new double[3], vcv = new double[3];
    public double tau, xkepnew, tc, xprev, tmodulo, alpha, xmax, xmin, x, xi, sxi, xsqcxi, t, r1;
    public double kepc1, kepc2, epsilont, delx, delt;
    public double[] urrect = new double[3];
    public int iterctr = 20;
    // LAMBERT
    public double[] r1vec = new double[3], r2vec = new double[3], vvec = new double[3], vtarget = new double[3];
    public double[] un = new double[3], ur1 = new double[3], u2 = new double[3];
    public double tdesired, coga, cogamax, cogamin, dcoga, p, r1a, snth, csth, oneMinusCsth, csthRho, magvec2;
    public double geomsgn = .5, vtargtag, tprev, terrlamb, epsilonl, tweekit;
    public boolean guessw, normsw, solnsw, rvsw, cogaflag, infinflg, apsesw, sw360, slopesw, ordersw;
    // TIME-THETA / TIME-RADIUS / APSIDES
    public double[] rvec = new double[3];
    public double rdesired, sgnrdot, cosf, ecc, apoapse, periapse;
    public double[] newrvec = new double[3], newvvec = new double[3];
    // ITERATOR
    public double indep, delindep, dep, depreV, deldep, max, min;
    /** First/second guess: max/min of the independent variable. */
    private double ovfind;

    public ConicSubroutines(Agc agc) { this.agc = agc; }

    // ---- small vector helpers ----
    private static double dot(double[] a, double[] b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
    private static double abval(double[] a) { return Math.sqrt(dot(a, a)); }
    private static double[] unit(double[] a) { double n = abval(a); return new double[]{a[0]/n, a[1]/n, a[2]/n}; }
    private static double[] cross(double[] a, double[] b) {
        return new double[]{a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]};
    }
    private static double[] lin(double ka, double[] a, double kb, double[] b) {
        return new double[]{ka*a[0]+kb*b[0], ka*a[1]+kb*b[1], ka*a[2]+kb*b[2]};
    }

    // =====================================================================
    // KEPLER
    // =====================================================================

    /**
     * KEPLERN: update RRECT, VRECT through TAU centiseconds along the conic -> RCV, VCV.
     * Backdating (negative TAU) and |TAU| greater than a period are handled by modulo-ing.
     * XKEPNEW is the guess of the root X; XPREV, TC the previous convergence point.
     * (Called via CSMCONIC / LEMCONIC in the original which also run KEPPREP.)
     */
    public void kepler() {
        double rm = mu(), root = rootMu();
        r1 = abval(rrect);
        urrect = unit(rrect);
        kepc1 = dot(rrect, vrect) / root;                              // C1 = R.V/ROOTMU
        kepc2 = dot(vrect, vrect) * r1 / rm - 1;                       // C2 = R V.V/MU - 1
        alpha = (1 - kepc2) / r1;                                      // ALPHA = (1-C2)/R1
        // Maximum X depends on type of conic
        xmax = alpha >= 0 ? TWO_PI_SC_REAL() / Math.sqrt(alpha) : Math.sqrt(-50.0 / alpha);
        // Remove whole periods (ellipse only)
        tmodulo = 0;
        if (alpha > 0) {
            double period = TWO_PI_SC_REAL() / (root * Math.pow(alpha, 1.5));
            while (Math.abs(tau) >= period) {                          // PERIODCH
                double sub = Math.copySign(period, tau);
                tau -= sub; tmodulo += sub;
            }
        }
        // MODDONE: validate the guess X, else use half of XMAX with the sign of TAU
        x = xkepnew;
        if (x == 0 || (x < 0) != (tau < 0) || Math.abs(x) >= xmax) x = Math.copySign(xmax / 2, tau); // BADX
        // STORBNDS
        if (tau >= 0) { xmin = 0; } else { xmin = -xmax; xmax = 0; }
        epsilont = Math.abs(tau) * BEE22;
        delx = x - xprev;
        for (int iter = 0; iter < 200; iter++) {                       // KEPLOOP
            deltime();
            delt = tau - t;                                            // DELT = DELINDEP
            if (Math.abs(delt) <= Math.max(epsilont, 1e-12 * Math.abs(tau))) break;   // KEPCONVG
            double rr = radiusAtX();
            double trial = delt * root / rr;                           // Newton: dt/dx = r/sqrt(mu)
            double xn = x + trial;
            // t(x) is increasing in x: too long (delt<0) -> X is a new upper bound, else a lower bound
            if (delt < 0) xmax = x; else xmin = x;
            if (xn <= xmin || xn >= xmax) xn = (xmin + xmax) / 2;       // keep inside bounds
            delx = xn - x;
            x = xn;
            tc = t;
            if (delx == 0) break;
            iterctr--;
            if (iterctr <= 0) break;                                    // CHECKCTR
        }
        kepconvg();
    }
    private static double TWO_PI_SC_REAL() { return 2 * Math.PI; }

    private double radiusAtX() {
        return xsqcxi + kepc1 * x * (1 - xi * sxi) + r1 * (1 - xi * cOf(xi));
    }

    /** KEPCONVG: compute terminal RCV and VCV from the converged X (f and g series). */
    private void kepconvg() {
        double root = rootMu();
        double c = cOf(xi), s = sxi;
        xsqcxi = x * x * c;
        double rc = xsqcxi + kepc1 * x * (1 - xi * s) + r1 * (1 - xi * c);   // RCNORM
        double f = 1 - xsqcxi / r1;
        double g = (t * 0 + tau) - x * x * x * s / root;
        rcv = lin(f, rrect, g, vrect);
        double fdot = root / (rc * r1) * (xi * x * s - x);
        double gdot = 1 - xsqcxi / rc;
        vcv = lin(fdot, rrect, gdot, vrect);
        tc = t;
        xprev = x;
    }

    /** Stumpff C(z) = (1-cos sqrt z)/z (z is XI). */
    private static double cOf(double z) {
        if (Math.abs(z) < 1e-6) return .5 - z / 24;
        return z > 0 ? (1 - Math.cos(Math.sqrt(z))) / z : (Math.cosh(Math.sqrt(-z)) - 1) / -z;
    }
    /** Stumpff S(z) = (sqrt z - sin sqrt z)/z^1.5. */
    private static double sOf(double z) {
        if (Math.abs(z) < 1e-6) return 1.0 / 6 - z / 120;
        if (z > 0) { double q = Math.sqrt(z); return (q - Math.sin(q)) / (q * q * q); }
        double q = Math.sqrt(-z); return (Math.sinh(q) - q) / (q * q * q);
    }

    /**
     * DELTIME: given X (XI = ALPHA X^2), compute S(XI), X^2 C(XI) and the time
     * T = (X (C2 X^2 S + R1) + C1 X^2 C) / ROOTMU. Result in T (centiseconds, +28).
     */
    public double deltime() {
        xi = alpha * x * x;
        sxi = sOf(xi);
        xsqcxi = x * x * cOf(xi);
        t = (x * (kepc2 * x * x * sxi + r1) + kepc1 * xsqcxi) / rootMu();
        return t;
    }

    /**
     * NEWSTATE: propagate (RVEC/UR1, VVEC) to the new state through X: returns R2 in newrvec and
     * V2 in newvvec; R2 magnitude into MAGVEC2. LAMENTER entry is {@link #lamenter}.
     */
    public void newstate() {
        double root = rootMu();
        double f = 1 - xsqcxi / r1;
        double g = t - x * x * x * sxi / root;
        double[] r0 = lin(r1, ur1, 0, ur1);
        newrvec = lin(f, r0, g, vvec);
        magvec2 = abval(newrvec);
        lamenter(magvec2);
        // V2 = (sqrt(mu)/(R2 R1)) (XI x S - x) R1vec + (1 - x^2C/R2) V1
        double[] v1 = vvec;
        newvvec = lin(root / (magvec2 * r1) * (xi * x * sxi - x), r0, 1 - xsqcxi / magvec2, v1);
    }

    /** LAMENTER: entry with R2 known (used by LAMBERT for the target velocity). */
    public void lamenter(double r2) { magvec2 = r2; }

    // =====================================================================
    // ITERATOR / CHECKCTR
    // =====================================================================

    /**
     * ITERATOR: secant/bracketing step on the independent variable. Updates DELINDEP; keeps
     * MIN and MAX bounds. FIRSTIME (SLOPESW set) uses TWEEKIT to start from the bounds.
     */
    public void iterator() {
        double trial;
        if (slopesw) {                                                // FIRSTIME
            slopesw = false;
            trial = Math.copySign(Math.abs(max - min) * tweekit, deldep);
            sgncheck(trial);
            return;
        }
        double dd = dep - depreV;
        trial = dd == 0 ? 0 : delindep * deldep / dd;                  // secant step
        if (!ordersw) trial = Math.abs(trial) * Math.signum(deldep);   // 2nd deriv. sign ignored
        sgncheck(trial);
    }
    private void sgncheck(double trial) {
        if (trial >= 0) {                                              // POSDEL
            if (!ordersw) min = indep;                                 // move MIN bound in
            if (indep + trial > max) trial = (max - indep) * DP9_10;   // MODPSDEL
        } else {
            if (!ordersw) max = indep;                                 // move MAX bound in
            if (indep + trial < min) trial = (min - indep) * DP9_10;   // MODNGDEL
        }
        delindep = trial;                                              // NEWDEL
    }

    /** CHECKCTR: decrement ITERCTR; returns the new count (zero or less means exceeded). */
    public int checkctr() { iterctr--; return iterctr; }

    // =====================================================================
    // PARAM / GEOM / GETX
    // =====================================================================

    /** PARAM: from RVEC, VVEC compute R1, R1A (=R1 V^2/MU...), P, COGA, UR1, U2, UN. */
    public void param(double[] r, double[] v) {
        normsw = false; cogaflag = false;
        geomsgn = .5;                                                  // gamma always < 180 deg
        geom(v, r);
        double sngaSin = snth;                                         // 36D = SIN GAMMA
        double csga = csth;
        coga = csga / sngaSin;                                         // COGA = cot(gamma)
        double v1sqOverMu = magvec2 * magvec2 * r1 / mu();
        r1a = 2 - v1sqOverMu;                                          // R1A = 2 - R1 V1^2/MU
        p = v1sqOverMu * sngaSin * sngaSin;                            // P = R1 V1^2 sin^2/MU
    }

    /**
     * GEOM: given V2VEC (MPAC) and R1VEC compute UR1, U2, R1, MAGVEC2, CSTH, SNTH and UN.
     * With NORMSW set UN is an input; GEOMSGN selects the sign of SNTH (and hence the
     * angle > 180 deg case).
     */
    public void geom(double[] v2, double[] r1v) {
        magvec2 = abval(v2);
        u2 = unit(v2);
        r1 = abval(r1v);
        ur1 = unit(r1v);
        csth = dot(ur1, u2);
        double[] c = cross(ur1, u2);
        if (normsw) {                                                  // HAVENORM
            snth = Math.copySign(abval(c), geomsgn);
        } else {
            double m = abval(c);
            un = m == 0 ? colinearNormal() : unit(c);                  // COLINEAR: halve and retry
            snth = Math.copySign(m, geomsgn);
            if (geomsgn < 0) un = lin(-1, un, 0, un);
        }
    }
    private double[] colinearNormal() { return new double[]{0, 0, 1}; }

    /**
     * GETX: from P, R1A, COGA, CSTH, SNTH determine X (and XI, KEPC1, KEPC2) for the transfer
     * through the true anomaly difference. INFINFLG set if closure through infinity is needed;
     * SW360 handles the near 360 degree case.
     */
    public void getx() {
        infinflg = false;
        double th = Math.atan2(snth, csth);
        if (th <= 0) th += 2 * Math.PI;                                // 0..2pi; sign convention via GEOMSGN
        double ec = p - 1, es = p * coga;                              // e cos f0, e sin f0
        double e2 = ec * ec + es * es;
        double f0 = Math.atan2(es, ec);
        double xx;
        if (Math.abs(r1a) < 1e-12) {                                   // parabola
            double d = Math.tan((f0 + th) / 2) - Math.tan(f0 / 2);
            xx = Math.sqrt(p * r1) * d;
            xi = 0;
        } else if (r1a > 0) {                                          // ellipse: x = sqrt(a) dE
            double a = r1 / r1a;
            double e = Math.sqrt(e2);
            double e0 = Math.atan2(Math.sqrt(1 - Math.min(e2, 1)) * Math.sin(f0), e + Math.cos(f0));
            double f1 = f0 + th;
            double e1 = Math.atan2(Math.sqrt(1 - Math.min(e2, 1)) * Math.sin(f1), e + Math.cos(f1));
            double de = e1 - e0;
            while (de < 0) de += 2 * Math.PI;
            if (th < Math.PI && de > Math.PI * 1.5) de -= 2 * Math.PI;
            xx = Math.sqrt(a) * de;
            xi = r1a / r1 * xx * xx;
        } else {                                                       // hyperbola: x = sqrt(-a) dH
            double a = r1 / r1a;                                       // negative
            double e = Math.sqrt(e2);
            double f1 = f0 + th;
            if (e * Math.cos(f1) + 1 <= 0 && (1 + e * Math.cos(f1)) <= 0) { infinflg = true; return; } // INFINITY
            double h0 = 2 * atanh(Math.sqrt((e - 1) / (e + 1)) * Math.tan(f0 / 2));
            double h1 = 2 * atanh(Math.sqrt((e - 1) / (e + 1)) * Math.tan(f1 / 2));
            xx = Math.sqrt(-a) * (h1 - h0);
            xi = r1a / r1 * xx * xx;
        }
        x = xx;
        alpha = r1a / r1;
        kepc1 = Math.sqrt(p * r1) * coga;                              // C1 = sqrt(P R1/ ...) COGA
        kepc2 = 1 - r1a;                                               // C2 = 1 - R1A
        t = 0;
    }
    private static double atanh(double v) { return .5 * Math.log((1 + v) / (1 - v)); }

    private void infinity() { infinflg = true; ovfind = 0; }

    // =====================================================================
    // LAMBERT
    // =====================================================================

    /**
     * LAMBERT: velocity VVEC at R1VEC to reach R2VEC in TDESIRED centiseconds (and VTARGET at
     * R2VEC if VTARGTAG is zero). Iterates on COGA (cotangent of flight-path angle from the
     * vertical) using GETX/DELTIME and the bracketing ITERATOR. SOLNSW is set when no solution.
     */
    public void lambert() {
        solnsw = false;
        epsilonl = tdesired * BEE19;
        slopesw = true;
        // GEOM with V2 := R2VEC
        geom(r2vec, r1vec);
        double r2 = magvec2;
        double r1OverR2 = r1 / r2;
        oneMinusCsth = 1 - csth;
        if (Math.abs(oneMinusCsth) < 1e-12) { solnsw = true; return; }  // 360LAMB: CSTH = 1
        double rho = r1OverR2;
        csthRho = csth - rho;
        // COGAMAX = SQRT(2 R1/(R2 (1-CSTH))) + SNTH/(1-CSTH), limited (scaled B+5) to COGUPLIM
        cogamax = Math.min(COGUPLIM * 32, snth / oneMinusCsth + Math.sqrt(2 * r1OverR2 / oneMinusCsth));
        cogamax = Math.min(cogamax, 32 * COGUPLIM);
        cogamin = geomsgn < 0 ? COGLOLIM * 32 : (csthRho / snth);
        if (cogamin < COGLOLIM * 32) cogamin = COGLOLIM * 32;
        if (!guessw) { /* use input COGA */ } else {
            tweekit = .5;
            coga = (cogamin + cogamax) / 2;                            // NOGUESS
            dcoga = coga;
        }
        iterctr = Math.max(iterctr, 20);
        for (;;) {                                                      // LAMBLOOP
            p = oneMinusCsth / (snth * coga - csthRho);
            if (p <= 0) { negp(); continue_or_fail(); if (solnsw) return; continue; }
            r1a = 2 - p * (1 + coga * coga);
            getx();
            if (infinflg) { negp(); continue_or_fail(); if (solnsw) return; continue; }
            alpha = r1a / r1;
            kepc1 = coga * Math.sqrt(p * r1);
            kepc2 = 1 - r1a;
            tprev = t;
            deltime();
            terrlamb = tdesired - t;
            if (Math.abs(terrlamb) <= Math.abs(epsilonl) + ONEBIT * tdesired) { initv(); return; }
            if (checkctr() <= 0) { sufficiencyCheck(); return; }
            // ITERATOR on COGA
            indep = coga; dep = t; deldep = terrlamb;
            min = cogamin; max = cogamax;
            double dt = deldep;
            double slope = (t - tprev) == 0 ? 1 : (t - tprev) / (delindep == 0 ? 1 : delindep);
            double step = Math.abs(slope) > 1e-12 && !slopesw ? dt / slope : (terrlamb > 0 ? -1 : 1) * Math.abs(cogamax - cogamin) / 4;
            slopesw = false;
            double newc = coga + step;
            if (newc <= cogamin || newc >= cogamax) newc = (cogamin + cogamax) / 2;
            if (terrlamb > 0) cogamax = Math.max(coga, cogamin); else cogamin = Math.min(coga, cogamax);
            delindep = newc - coga;
            coga = newc;
            if (delindep == 0) { sufficiencyCheck(); return; }
        }
    }
    private int failures;
    private void continue_or_fail() {
        if (++failures > 100) { solnsw = true; failures = 0; }
    }

    /** NEGP: impossible trajectory (bounds inaccurate) - try new COGA, LOENERGY or HIENERGY. */
    private void negp() {
        if (dcoga >= 0) hienergy(); else loenergy();
    }
    /** HIENERGY: high energy trajectory (overflow of P or R1A or XI > 50): new lower bound. */
    private void hienergy() { cogamin = coga; commonlm(); }
    /** LOENERGY: low energy trajectory (overflow of time): new upper bound. */
    private void loenergy() { cogamax = coga; commonlm(); }
    /** COMMONLM: use DCOGA/2 as decrement and restart LAMBLOOP. */
    private void commonlm() {
        dcoga /= 2;
        if (dcoga == 0) { solnsw = true; return; }                     // SUFFCHEK
        coga = coga - dcoga;
    }

    /** SUFFCHEK: sufficient accuracy? If |TERRLAMB| small enough go to INITV, else SOLNSW. */
    private void sufficiencyCheck() {
        if (Math.abs(terrlamb) - tdesired * Math.pow(2, -17) - ONEBIT < 0) initv();
        else solnsw = true;
    }

    /** INITV: initial velocity VVEC = VTAN (UN x UR1) + VRAD UR1; optionally VTARGET via LAMENTER. */
    private void initv() {
        double vtan = Math.sqrt(mu() * p / r1);                        // sqrt(MU P / R1)
        double[] vv = lin(vtan * coga, ur1, vtan, cross(un, ur1));
        vvec = vv;
        if (vtargtag == 0) targetv();
    }
    /** TARGETV: velocity at R2 using NEWSTATE/LAMENTER. */
    private void targetv() {
        // XI, X, S(XI) are those of the last DELTIME
        double root = rootMu();
        double[] r0 = lin(r1, ur1, 0, ur1);
        double f = 1 - xsqcxi / r1, g = t - x * x * x * sxi / root;
        double[] r2v = lin(f, r0, g, vvec);
        double rr = abval(r2v);
        vtarget = lin(root / (rr * r1) * (xi * x * sxi - x), r0, 1 - xsqcxi / rr, vvec);
    }

    // =====================================================================
    // TIME-THETA
    // =====================================================================

    /**
     * TIMETHET: from RVEC, VVEC and the true-anomaly difference (SNTH, CSTH), compute the
     * transfer time T (and, if RVSW is clear, the updated NEWRVEC, NEWVVEC). Aborts via
     * ABTCONIC if COGAFLAG or INFINFLG is set.
     */
    public double timethet() {
        ovfind = 0;
        double snSave = snth, csSave = csth;
        param(rvec, vvec);
        // PARAM clobbers SNTH/CSTH with the flight path angle values; restore the desired ones
        snth = snSave; csth = csSave;
        double[] v1 = vvec;
        vvec = v1;
        if (cogaflag) abtconic();
        getx();
        commnout();
        return t;
    }

    /** COMMNOUT: INFINFLG -> abort; otherwise DELTIME, then NEWSTATE if RVSW clear. */
    private void commnout() {
        if (infinflg) abtconic();
        cogaflag = false;
        alpha = r1a / r1;
        kepc1 = coga * Math.sqrt(p * r1); kepc2 = 1 - r1a;
        deltime();
        if (!rvsw) newstate();
    }

    /** COGAOVFL: set COGAFLAG and abort. */
    private void cogaovfl() { cogaflag = true; abtconic(); }

    // =====================================================================
    // TIME-RADIUS
    // =====================================================================

    /**
     * TIMERAD: time to reach RDESIRED (radial velocity sign SGNRDOT) from RVEC, VVEC. APSESW
     * is set when RDESIRED is beyond apocenter/pericenter (that apse is then returned);
     * SOLNSW if the conic is so close to a circle that the point is ambiguous (ecc less than 2^-18).
     */
    public double timerad() {
        param(rvec, vvec);
        // eccentricity vector magnitude: e^2 = (P-1)^2 + (P COGA)^2
        double ec = p - 1, es = p * coga;
        double e = Math.sqrt(ec * ec + es * es);
        ecc = e;
        if (e < Math.pow(2, -18)) { solnsw = true; abtconic(); }       // CIRCULAR
        solnsw = false;
        double rd = rdesired;
        cosf = (p * r1 / rd - 1) / e;                                   // COSF at the desired radius
        apsesw = false;
        if (cosf > 1 || cosf < -1) {                                    // BADR2
            cosf = Math.copySign(1, cosf); apsesw = true;
        }
        double sinf = Math.copySign(Math.sqrt(Math.max(0, 1 - cosf * cosf)), sgnrdot);
        double f0 = Math.atan2(es, ec);
        double f1 = Math.atan2(sinf, cosf);
        double th = f1 - f0;
        while (th < 0) th += 2 * Math.PI;
        csth = Math.cos(th); snth = Math.sin(th);                       // TERMNVEC
        getx();
        commnout();
        return t;
    }

    // =====================================================================
    // APSIDES
    // =====================================================================

    /**
     * APSIDES: radius of apocenter (returned) and pericenter from RVEC, VVEC; ECC set.
     * When the apocenter is beyond scaling (or the orbit not closed) the maximum position
     * (DPPOSMAX, LDPOSMAX) is returned.
     */
    public void apsides() {
        param(rvec, vvec);
        double ec = p - 1, es = p * coga;
        ecc = Math.sqrt(ec * ec + es * es);                             // GETECC
        periapse = p * r1 / (1 + ecc);                                  // RP
        apoapse = ecc >= 1 ? Double.MAX_VALUE : p * r1 / (1 - ecc);     // RA
        if (apoapse < 0 || apoapse > (moon ? 134217727.5 : 536870910.0)) {
            apoapse = moon ? 134217727.5 : 536870910.0;                 // INFINAPO -> LDPOSMAX
        }
    }

    /** ABTCONIC: abort through POODOO with alarm code 00607. */
    public void abtconic() {
        throw new IllegalStateException("POODOO alarm " + Integer.toOctalString(ALARM_CONIC));
    }
}
