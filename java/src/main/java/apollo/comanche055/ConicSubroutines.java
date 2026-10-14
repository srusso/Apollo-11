package apollo.comanche055;

import apollo.agc.Agc;

/**
 * CONIC_SUBROUTINES (Comanche 055 / Colossus 2A): KEPLER, LAMBERT, TIME-THETA, TIME-RADIUS and
 * APSIDES, together with their support routines (PARAM, GEOM, GETX, DELTIME, ITERATOR, NEWSTATE /
 * LAMENTER) and the MUTABLE table of gravitational constants.
 *
 * <p>The original is interpretive code on scaled double-precision fixed-point numbers. Here all
 * quantities are carried as Java doubles in physical units (metres, metres/centisecond,
 * centiseconds) and the scale factors of the original are given in the comments ("+29 EARTH /
 * +27 MOON" etc). The central body is chosen by {@link #setBody}: X1 = -2D (Earth) or -10D (Moon)
 * indexes the MUTABLE table. Interpretive switches (INFINFLG, COGAFLAG, SOLNSW, APSESW, 360SW,
 * SLOPESW, ORDERSW, NORMSW, GUESSW, RVSW) live in the erasable symbol table via agc.get/set.
 * Aborts go to POODOO with alarm 00607 (ABTCONIC).
 *
 * <p>The conic routines cannot interrupt each other (they share erasable beyond the pushlist).
 */
public final class ConicSubroutines {
    // ---- constants (CONICS1 bank) ----
    /** D1/4 ... D1/1024 are binary fractions used only for scale alignment; kept for reference. */
    static final double D1_8 = 1.0 / 8, D1_128 = 1.0 / 128, D1_64 = 1.0 / 64, D1_4 = 1.0 / 4,
        D1_16 = 1.0 / 16, D1_32 = 1.0 / 32, D1_1024 = 1.0 / 1024, D1_256 = 1.0 / 256;
    /** DP9/10: factor used to pull a trial step back inside the bounds. */
    static final double DP9_10 = .9;
    /** -50SC: -50.0 B-12 (limit on alpha x^2 for hyperbolic orbits). */
    static final double MINUS_50 = -50.0;
    /** 2PISC: 2 pi (B-6 scaled). */
    static final double TWO_PI = 6.28318530;
    /** BEE19 = 2^-19, BEE22 = 2^-22: convergence tolerance factors. */
    static final double BEE19 = Math.pow(2, -19);
    static final double BEE22 = Math.pow(2, -22);
    /** ONEBIT: 1.0 B-28 (one bit of a time word). */
    static final double ONEBIT = 1.0;
    static final double COGUPLIM = .999511597 * 32;  // +5 scaled upper limit of COGA
    static final double COGLOLIM = -.999511597 * 32; // lower limit of COGA
    /** Time word limit (T is scaled B+28 centiseconds): overflow beyond this. */
    static final double TIME_MAX = Math.pow(2, 28);
    /** POSMAX of DP position, +29 Earth / +27 Moon, in metres. */
    static final double[] DPPOSMAX = {Math.pow(2, 29), Math.pow(2, 27)};

    /**
     * MUTABLE: {MU, 1/MU, SQRT(MU), 1/SQRT(MU)} per body; index 0 = Earth (X1 = -2D),
     * index 1 = Moon (X1 = -10D).
     */
    static final double[][] MUTABLE = {
        {3.986032e10, .25087606e-10, 1.99650495e5, .50087529e-5},
        {4.902778e8, .203966e-8, 2.21422176e4, .45162595e-4}};

    /** Result of NEWSTATE / TIMETHET / TIMERAD: transfer time and (optionally) the new state. */
    public static final class Result {
        public double t;       // T, +28 centiseconds
        public double[] r;     // terminal position (0D-5D, +29 or +27)
        public double[] v;     // terminal velocity (MPAC, +7 or +5)
    }

    private final Agc agc;
    private int body;               // 0 = Earth, 1 = Moon
    private int x1;                 // the X1 value itself (-2 or -10)

    // KEPLER erasables
    public double[] rrect = new double[3], vrect = new double[3];
    public double tau, xkepnew, tc, xprev;
    public double[] rcv, vcv;
    double alpha, xmax, xmin, x, xi, sxi, xsqc, t, r1, kepc1, kepc2, delx, delt, epsilont, rcnorm;
    double[] urrect = new double[3];
    double mu, invMu, rootMu, invRootMu;
    int iterctr;

    // LAMBERT / GEOM / PARAM erasables
    public double[] r1vec = new double[3], r2vec = new double[3];
    public double tdesired, geomsgn, coga, vtargtag;
    public double[] un = new double[3], vvec, vtarget;
    double[] rvec = new double[3];
    double[] u2 = new double[3], ur1 = new double[3];
    double magvec2, snth, csth, oneMinusCsth, p, r1a, cstRho, epsilonl, tprev, terrlamb;
    double cogamax, cogamin, dcoga, tweekit;
    double ecc, rdesired, sgnrdot, cosf;
    public double diffalt;

    // ITERATOR erasables
    double indep, delindep, dep, deprev, deldep, min, max;

    public ConicSubroutines(Agc agc) {
        this.agc = agc;
        setBody(0);
    }

    /** Sets X1 to -2D (Earth, body 0) or -10D (Moon, body 1) and loads MU constants. */
    public void setBody(int body) {
        this.body = body;
        this.x1 = body == 0 ? -2 : -10;
        double[] m = MUTABLE[body];
        mu = m[0];
        invMu = m[1];
        rootMu = m[2];
        invRootMu = m[3];
    }

    private boolean flag(String name) { return agc.get(name) != 0; }
    private void setFlag(String name) { agc.set(name, 1); }
    private void clearFlag(String name) { agc.set(name, 0); }

    // ---- small vector helpers ----
    private static double dot(double[] a, double[] b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
    private static double abval(double[] a) { return Math.sqrt(dot(a, a)); }
    private static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }
    private static double[] scale(double[] a, double k) { return new double[] {a[0] * k, a[1] * k, a[2] * k}; }
    private static double[] add(double[] a, double[] b) { return new double[] {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
    private static double[] sub(double[] a, double[] b) { return new double[] {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
    private static double[] unit(double[] a) {
        double n = abval(a);
        return n == 0 ? new double[3] : scale(a, 1.0 / n);
    }
    private static double sign(double magnitude, double sgn) { return sgn < 0 ? -Math.abs(magnitude) : Math.abs(magnitude); }

    // ================= KEPLER =================

    /**
     * KEPLERN / KEPLER: given RRECT, VRECT, TAU (centiseconds, may be negative or exceed a period),
     * XKEPNEW (guess of x), TC, XPREV, solves Kepler's equation and returns RCV, VCV in
     * {@link #rcv}/{@link #vcv}; TC and XPREV hold the converged time and x. Must be reached via
     * CSMCONIC / LEMCONIC (no interrupts); KEPRTN is the return of this method.
     */
    public void kepler() {
        clearFlag("OVFIND");
        // C1 = R.V/ROOTMU, C2 = R V.V/MU - 1, ALPHA = (1-C2)/R1
        urrect = unit(rrect);
        iterctr = 20;
        r1 = abval(rrect);
        kepc1 = dot(rrect, vrect) * invRootMu;
        kepc2 = dot(vrect, vrect) * invMu * r1 - 1;
        alpha = (1 - kepc2) / r1;
        // maximum X depends on the type of conic
        if (alpha > 0) {
            xmax = TWO_PI / Math.sqrt(alpha);        // 1REV: X for one period
        } else {
            xmax = Math.sqrt(Math.abs(MINUS_50 / alpha)); // SQRT(-50SC/ALPHA), hyperbolic limit
        }
        // PERIOD = XMAX / (ROOTMU ALPHA); remove whole periods from TAU (TMODULO)
        if (alpha > 0) {
            double period = xmax / (rootMu * alpha);
            while (true) { // PERIODCH
                double d = Math.abs(tau) - period;
                if (d < 0) {
                    break; // MODDONE
                }
                tau = sign(d, tau);
            }
        }
        // MODDONE: sanity-check the guess of X
        x = xkepnew;
        if (tau == 0 || x == 0 || (x < 0) != (tau < 0) || Math.abs(x) >= xmax) {
            x = sign(xmax / 2, tau); // BADX
        }
        // STORBNDS
        if (tau >= 0) {
            xmin = 0;
        } else {
            xmin = -xmax;
            xmax = 0;
        }
        // DXCOMP
        epsilont = Math.abs(tau * BEE22);
        delx = x - xprev; // XDIFF
        while (true) { // KEPLOOP
            double xsq = x * x;
            xi = alpha * xsq;
            boolean timeOverflow = deltime(xi, xsq);
            if (timeOverflow) {
                // TIMEOVFL: X was too big
                if (x < 0) {
                    xmin = x; // NEGTOVFL
                } else {
                    xmax = x;
                }
                delx = delx / 2; // CMNTOVFL
                if (delx == 0) {
                    return; // KEPRTN
                }
                x = x - delx;
                t = tc;
                if (branchCounter()) {
                    kepConverged();
                    return;
                }
                continue;
            }
            delt = tau - t;
            if (epsilont - Math.abs(delt) >= 0) {
                kepConverged();
                return;
            }
            // secant step: trial DELX = DELX * DELT / (T - TC)
            double trial = delx * delt / (t - tc);
            double newdelx;
            if (trial >= 0) { // POSDELX
                xmin = x;
                if (xmax - x - trial < 0) {
                    newdelx = DP9_10 * (xmax - x); // PDXCHNGE
                } else {
                    newdelx = trial;
                }
            } else {
                xmax = x; // move max bound in
                if (xmin - x - trial >= 0) {
                    newdelx = DP9_10 * (xmin - x); // NDXCHNGE
                } else {
                    newdelx = trial;
                }
            }
            delx = newdelx; // NEWDELX
            if (delx == 0) {
                kepConverged();
                return;
            }
            x = delx + x;
            tc = t;
            if (branchCounter()) { // BRNCHCTR
                kepConverged();
                return;
            }
        }
    }

    /** BRNCHCTR: RTB CHECKCTR / BHIZ KEPCONVG; true when the iteration limit is reached. */
    private boolean branchCounter() {
        return checkctr() <= 0;
    }

    /** KEPCONVG: build RCV and VCV, TC = T, XPREV = X. */
    private void kepConverged() {
        // RCV = (R1 - XSQC) URRECT + (T - X^3 S(XI)/ROOTMU) VRECT
        double g = t - x * x * x * sxi * invRootMu;
        rcv = add(scale(urrect, r1 - xsqc), scale(vrect, g));
        rcnorm = abval(rcv);
        // VCV = URRECT (XI S(XI) - 1) X ROOTMU/RCNORM + VRECT (1 - XSQC/RCNORM)
        double fdot = (xi * sxi - 1) * rootMu * x / rcnorm;
        vcv = add(scale(urrect, fdot), scale(vrect, 1 - xsqc / rcnorm));
        tc = t;
        xprev = x;
    }

    /**
     * DELTIME: from XI and XSQ computes S(XI), C(XI), XSQC(XI) = XSQ C(XI) and the time
     * T = (KEPC1 XSQC + X (KEPC2 XSQ S + R1)) / ROOTMU. Returns true on time overflow.
     */
    boolean deltime(double xiArg, double xsq) {
        double z = xiArg / 64.0; // XI is scaled +6
        sxi = 2 * poly(S_COEF, z);             // S(XI), first polynomial (scaled B-1)
        double c = 16 * poly(C_COEF, z);       // C(XI), second polynomial (scaled B-4)
        xsqc = xsq * c;
        double term1 = kepc1 * xsqc;
        double term2 = kepc2 * xsq * sxi;
        // T = (KEPC1 XSQ C + X (KEPC2 XSQ S + R1)) / ROOTMU
        t = (term1 + x * (term2 + r1)) / rootMu;
        return Math.abs(t) >= TIME_MAX || Double.isNaN(t) || Double.isInfinite(t);
    }

    /** First POLY table of DELTIME (S(XI)): DEC 8 then ten 2DEC coefficients. */
    static final double[] S_COEF = {.083333334, -.266666684, .406349155, -.361198675, .210153242,
        -.086221951, .026268812, -.006163316, .001177342, -.000199055};
    /** Second POLY table of DELTIME (C(XI)). */
    static final double[] C_COEF = {.031250001, -.166666719, .355555413, -.406347410, .288962094,
        -.140117894, .049247387, -.013081923, .002806389, -.000529414};

    private static double poly(double[] c, double z) {
        double r = 0;
        for (int i = c.length - 1; i >= 0; i--) {
            r = r * z + c[i];
        }
        return r;
    }

    /**
     * CHECKCTR: decrement ITERCTR; at zero the iteration limit is exceeded (original goes to
     * DANZIG for the CSM). Returns the new count.
     */
    int checkctr() {
        iterctr = iterctr - 1;
        if (iterctr < 0) {
            danzig();
        }
        return iterctr;
    }

    private void danzig() {
        // TC DANZIG: iteration count went negative
        new AlarmAndAbort(agc).poodoo(0430);
    }

    // ================= ITERATOR =================

    /**
     * ITERATOR: general secant/bounded iteration. INDEP is the independent variable, DEP the
     * dependent variable, DELDEP the error in DEP, MIN/MAX the bounds. Computes the next
     * DELINDEP (also returned). SLOPESW is set on entry for the first pass; ORDERSW selects
     * second-order (min-finding) use.
     */
    double iterator() {
        double trial;
        if (flag("SLOPESW")) {
            clearFlag("SLOPESW"); // BONCLR: first time through
            // FIRSTIME: DELINDEP = SIGN(DELDEP) (MIN-MAX)*TWEEKIT
            trial = sign(max * tweekit - min * tweekit, deldep);
        } else {
            trial = delindep * deldep / (dep - deprev);
            if (!flag("ORDERSW")) {
                // keep sign as is
            } else {
                trial = sign(trial, deldep); // in case 2nd derivative changed sign
            }
        }
        // SGNCHECK
        if (trial >= 0) {
            if (!flag("ORDERSW")) {
                min = indep; // if not 2nd order can move MIN bound in
            }
            if (max - indep - trial < 0) { // MODPSDEL
                trial = DP9_10 * (max - indep);
            }
        } else {
            if (!flag("ORDERSW")) {
                max = indep; // can move MAX bound in
            }
            if (min - indep - trial >= 0) { // MODNGDEL
                trial = DP9_10 * (min - indep);
            }
        }
        delindep = trial; // NEWDEL
        return delindep;
    }

    // ================= NEWSTATE / LAMENTER =================

    /**
     * NEWSTATE: given X, XI, S(XI), XSQC, T, R1, UR1, VVEC computes the terminal position
     * R2VEC = (R1 - XSQC) UR1 + (T - X^3 S/ROOTMU) VVEC and velocity via LAMENTER.
     */
    Result newstate() {
        double[] r2v = add(scale(ur1, r1 - xsqc), scale(vvec, t - x * x * x * sxi * invRootMu));
        Result res = new Result();
        res.t = t;
        res.r = r2v;
        res.v = lamenter(abval(r2v));
        return res;
    }

    /**
     * LAMENTER: V2VEC = UR1 (XI S(XI) - 1) X ROOTMU/R2 + VVEC (1 - XSQC/R2). R2 = |R2VEC|
     * (also left in MAGVEC2).
     */
    double[] lamenter(double r2) {
        double fdot = (xi * sxi - 1) * rootMu * x / r2;
        return add(scale(ur1, fdot), scale(vvec, 1 - xsqc / r2));
    }

    // ================= TIME-THETA =================

    /**
     * TIMETHET: time to traverse the true anomaly difference given by SNTH/CSTH, starting from
     * RVEC/VVEC. If RVSW is set only T is output; otherwise the new state is also returned.
     * Aborts through POODOO 00607 if COGAFLAG or INFINFLG results.
     */
    public Result timethet(double[] rv, double[] vv) {
        rvec = rv;
        vvec = vv;
        param(rv, vv);
        if (flag("OVFIND")) { // BOV COGAOVFL
            cogaovfl();
        }
        getx(p);
        return commnout();
    }

    /** COMMNOUT: common exit of TIMETHET / TIMERAD. */
    private Result commnout() {
        if (flag("INFINFLG")) {
            abtconic();
        }
        clearFlag("COGAFLAG");
        deltime(xi, x * x);
        Result res;
        if (flag("RVSW")) {
            res = new Result();
            res.t = t;
        } else {
            res = newstate();
        }
        return res;
    }

    private void cogaovfl() {
        setFlag("COGAFLAG");
        abtconic();
    }

    /** ABTCONIC: TC POODOO, OCT 00607. */
    private void abtconic() {
        new AlarmAndAbort(agc).poodoo(0607);
        throw new IllegalStateException("ABTCONIC: POODOO 00607");
    }

    // ================= PARAM / GEOM / GETX =================

    /**
     * PARAM: with MPAC = V1VEC and 0D = R1VEC computes COGA, 1/MU, R1A and P from the
     * geometry of the state (flight-path angle GAMMA always below 180 deg).
     */
    void param(double[] r1v, double[] v1v) {
        clearFlag("NORMSW");
        clearFlag("COGAFLAG");
        geomsgn = 1.0; // SSP GEOMSGN 37777
        double sngamma = geom(v1v, r1v);        // SNGA, CSGA (+1) in csth
        double csga = csth;
        coga = csga / sngamma;                  // COGA (+5)
        double v1sq = magvec2 * magvec2;
        double r1v2OverMu = v1sq * invMu * r1;  // R1 V1SQ/MU (+6)
        r1a = 2 - r1v2OverMu;                   // R1A (+6)
        p = r1v2OverMu * sngamma * sngamma;     // P (+4)
    }

    /**
     * GEOM: MPAC = V2VEC, 0D = R1VEC. Sets U2, MAGVEC2, UR1, CSTH, R1, UN and returns SNTH.
     * If NORMSW is set UN is an input; otherwise UN = UNIT(UR1 x U2 * sign GEOMSGN). SNTH is
     * signed by GEOMSGN.
     */
    double geom(double[] v2v, double[] r1v) {
        u2 = unit(v2v);
        magvec2 = abval(v2v);
        ur1 = unit(r1v);
        csth = dot(ur1, u2);
        r1 = abval(r1v);
        double[] c = cross(ur1, u2);
        if (flag("NORMSW")) { // HAVENORM
            snth = sign(abval(c), geomsgn);
            return snth;
        }
        double[] c2 = geomsgn < 0 ? scale(c, -1) : c;
        if (abval(c2) == 0) {
            // COLINEAR: VSR1 and retry UNIT; UN stays zero
            un = new double[3];
        } else {
            un = unit(c2);
        }
        snth = sign(abval(c), geomsgn);
        return snth;
    }

    /**
     * GETX: assumes P (+4) given. Using CSTH, SNTH, COGA, R1A computes X, XI, KEPC1 and KEPC2.
     * Sets INFINFLG when closure through infinity is required (no solution). Uses the
     * continued-fraction loop W -> W + SQRT(W^2 + R1A) (three passes) and the POLYCOEF series;
     * near 360 degrees (360SW) the alternative inverse form (INVRSEQN) is used.
     */
    void getx(double pArg) {
        p = pArg;
        clearFlag("360SW");
        double sqrtP = Math.sqrt(p);
        double tanHalf;
        boolean near360 = false;
        if (snth == 0) {
            near360 = true;
            tanHalf = 0;
        } else {
            tanHalf = (1 - csth) / snth;
            if (Math.abs(tanHalf) > 16) { // BOV 360CHECK
                near360 = true;
            }
        }
        double w = (tanHalf - coga) * sqrtP;
        if (!near360 && Math.abs(w) > 32) { // second BOV 360CHECK
            near360 = true;
        }
        double u; // 1/W (+2) or the result of the inverse sequence
        if (near360) {
            setFlag("360SW"); // 360CHECK
            // INVRSEQN: 1/W = SNTH / (SQRT(P) ((1 + CSTH) - SNTH COGA))
            double denom = sqrtP * ((1 + csth) - snth * coga);
            double u0 = denom == 0 ? Double.MAX_VALUE : Math.abs(snth / denom);
            double g = 1.0; // D1/16 start
            for (int i = 0; i < 3; i++) { // 1/WLOOP
                double s = g * g + r1a * u0 * u0;
                if (s < 0) {
                    infinity();
                    return;
                }
                g = g + Math.sqrt(s);
            }
            u = u0 / g;
        } else {
            double g = w;
            for (int i = 0; i < 3; i++) { // WLOOP
                double s = g * g + r1a;
                if (s < 0) {
                    infinity();
                    return;
                }
                g = g + Math.sqrt(s);
            }
            if (g == 0) {
                infinity();
                return;
            }
            u = 1.0 / g;
        }
        // POLYCOEF
        if (u < 0) {
            infinity();
            return;
        }
        double z = r1a * u * u;
        double f = poly(POLYCOEF, z);
        double h = u * f; // PUSH: 0D
        if (flag("360SW")) { // TRUE360X
            if (r1a < 0) {
                infinity();
                return;
            }
            h = TWO_PI / Math.sqrt(r1a) - h;
        }
        // XCOMMON
        xi = r1a * h * h;                 // XI (+6)
        x = Math.sqrt(r1) * h;            // X (+17 Earth / +16 Moon)
        kepc1 = Math.sqrt(p * r1) * coga; // KEPC1
        kepc2 = 1 - r1a;                  // KEPC2
        clearFlag("INFINFLG");
    }

    /** POLYCOEF series, DEC 5 then seven 2DEC coefficients. */
    static final double[] POLYCOEF = {.5, -.166666770, .100000392, -.071401086, .055503292,
        -.047264098, .040694204};

    /** INFINITY: no solution, closure through infinity required. */
    private void infinity() {
        setFlag("INFINFLG");
    }

    // ================= LAMBERT =================

    /**
     * LAMBERT: velocity VVEC at R1VEC required to reach R2VEC in TDESIRED. Inputs: R1VEC, R2VEC,
     * TDESIRED, GEOMSGN (+/-), GUESSW (no COGA guess), COGA, NORMSW/UN, VTARGTAG (0 = also
     * compute VTARGET). Returns VVEC; SOLNSW is set (see {@code agc.get("SOLNSW")}) if the problem
     * cannot be solved (caller then branches to LAMABORT).
     */
    public double[] lambert() {
        iterctr = 20;
        epsilonl = tdesired * BEE19;
        setFlag("SLOPESW");
        // GEOM with MPAC = R2VEC, 0D = R1VEC
        geom(r2vec, r1vec);
        double r2 = magvec2;
        double rho = r1 / r2; // R1/R2 (+7)
        oneMinusCsth = 1 - csth;
        if (Math.abs(oneMinusCsth) < 1.0 / 32768) { // ROUND BZE 360LAMB
            setFlag("SOLNSW"); // LAMBERT cannot handle CSTH = 1
            return null;
        }
        // COGAMAX = SQRT(2 R1/(R2 (1-CSTH))) + SNTH/(1-CSTH)
        cogamax = Math.sqrt(2 * r1 / (r2 * oneMinusCsth)) + snth / oneMinusCsth;
        if (Double.isNaN(cogamax) || cogamax > COGUPLIM) {
            cogamax = COGUPLIM; // UPLIM
        }
        cstRho = csth - rho; // CSTH-RHO
        if (geomsgn < 0 || snth == 0) {
            cogamin = COGLOLIM; // LOLIM
        } else {
            cogamin = cstRho / snth; // MINCOGA
            if (Double.isNaN(cogamin) || cogamin < COGLOLIM) {
                cogamin = COGLOLIM;
            }
        }
        if (flag("GUESSW")) {
            // NOGUESS
            tweekit = .5;
            coga = (cogamin + cogamax) / 2;
            dcoga = coga;
        } else {
            tweekit = 1; // 00001 (tweek unit)
        }
        return lamloop();
    }

    /** LAMBLOOP and the exits of LAMBERT (NEGP, HIENERGY, LOENERGY, SUFFCHEK, INITV). */
    private double[] lamloop() {
        while (true) {
            // LAMBLOOP
            double denom = snth * coga - cstRho;
            boolean bad = denom <= 0;
            if (!bad) {
                p = oneMinusCsth / denom;                  // P (+4)
                r1a = 2 - p * (1 + coga * coga);           // R1A (+6)
                if (Math.abs(p) >= 16 || Math.abs(r1a) >= 32) {
                    // HIENERGY: overflow of P or R1A
                    if (hienergy()) {
                        return sufficiencyCheck();
                    }
                    continue;
                }
                getx(p);
                tprev = t; // previous T
                if (flag("INFINFLG")) {
                    bad = true;
                } else {
                    boolean ovfl = deltime(xi, x * x);
                    if (ovfl) { // BIGTIME
                        t = tprev;
                        if (loenergy()) {
                            return sufficiencyCheck();
                        }
                        continue;
                    }
                    terrlamb = tdesired - t;
                    if (epsilonl - Math.abs(terrlamb) >= 0) {
                        return initv();
                    }
                    if (checkctr() <= 0) {
                        return sufficiencyCheck();
                    }
                    deldep = terrlamb;
                    dep = t;
                    deprev = tprev;
                    indep = coga;
                    min = cogamin;
                    max = cogamax;
                    delindep = dcoga;
                    double step = iterator();
                    dcoga = delindep;
                    if (step == 0) {
                        return sufficiencyCheck();
                    }
                    coga = coga + step;
                    continue;
                }
            }
            // NEGP: impossible trajectory due to inaccurate bound calculation, try new COGA
            boolean done = dcoga >= 0 ? loenergy() : hienergy();
            if (done) {
                return sufficiencyCheck();
            }
        }
    }

    /** HIENERGY: COGA becomes the new lower bound; then COMMONLM. Returns true to stop. */
    private boolean hienergy() {
        cogamin = coga;
        return commonlm();
    }

    /** LOENERGY: COGA becomes the new upper bound; then COMMONLM. Returns true to stop. */
    private boolean loenergy() {
        cogamax = coga;
        return commonlm();
    }

    /** COMMONLM: use DCOGA/2 as decrement; stop (SUFFCHEK) when it underflows. */
    private boolean commonlm() {
        dcoga = dcoga / 2;
        if (dcoga == 0 || Math.abs(dcoga) < Math.pow(2, -28) * 32) {
            return true;
        }
        coga = coga - dcoga;
        return false;
    }

    /** SUFFCHEK: accept the result if the time error is small enough, else SOLNSW. */
    private double[] sufficiencyCheck() {
        double v = Math.abs(terrlamb) - ONEBIT; // DSU ONEBIT
        if (v >= 0 || Math.abs(terrlamb) <= Math.abs(tdesired) * D1_4) {
            return initv();
        }
        setFlag("SOLNSW");
        return null;
    }

    /** INITV: build VVEC = VTAN (COGA UR1 + UN x UR1); optionally VTARGET via LAMENTER. */
    private double[] initv() {
        double vtan = Math.sqrt(p / r1) * rootMu; // tangential speed
        vvec = scale(add(scale(ur1, coga), cross(un, ur1)), vtan);
        clearFlag("SOLNSW");
        if (vtargtag == 0) { // TARGETV
            vtarget = lamenter(magvec2);
        }
        return vvec;
    }

    // ================= TIME-RADIUS =================

    /**
     * TIMERAD: time for the conic through RVEC/VVEC to reach radius RDESIRED, with radial
     * velocity sign SGNRDOT. If RVSW is set only T is output, else the new state is returned too.
     * APSESW is set when RDESIRED lies beyond an apsis; SOLNSW when the conic is circular.
     */
    public Result timerad(double[] rv, double[] vv) {
        rvec = rv;
        vvec = vv;
        param(rv, vv);
        if (flag("OVFIND")) {
            cogaovfl();
        }
        // eccentricity vector (+3): (1 - R1A) UR1 - COGA SQRT(P (2 - R1A)) U2
        double k = coga * Math.sqrt(Math.max(0, p * (2 - r1a)));
        double[] eccv = sub(scale(ur1, 1 - r1a), scale(u2, k));
        ecc = abval(eccv);
        if (ecc == 0) { // CIRCULAR
            setFlag("SOLNSW");
            abtconic();
        }
        double[] uecc = scale(eccv, 1.0 / ecc);
        double cf = (r1 * p / rdesired - 1) / ecc; // COSF
        cosf = cf;
        double sinf;
        clearFlag("APSESW");
        if (Double.isNaN(cf) || Double.isInfinite(cf) || 1 - cf * cf < 0) {
            // BADR2: beyond an apsis; use the apsis solution
            cosf = sign(1.0, cf);
            sinf = 0;
            setFlag("APSESW");
        } else {
            sinf = sign(Math.sqrt(1 - cf * cf), sgnrdot);
        }
        // TERMNVEC: U2 = COSF UECC + SINF (UN x UECC)
        double[] u2n = add(scale(uecc, cosf), scale(cross(un, uecc), sinf));
        csth = Math.max(-1, Math.min(1, dot(u2n, ur1)));      // CSTH (+1)
        snth = dot(cross(ur1, u2n), un);                      // SNTH (+1)
        getx(p);
        clearFlag("SOLNSW");
        return commnout();
    }

    // ================= APSIDES =================

    /**
     * APSIDES: radius of apocenter (returned) and pericenter (written to {@code periapse[0]})
     * and the eccentricity ECC (+3) for the conic through RVEC/VVEC. If the apocenter exceeds
     * the position scaling, DPPOSMAX is returned.
     */
    public double[] periapse = new double[1];

    public double apsides(double[] rv, double[] vv) {
        rvec = rv;
        vvec = vv;
        param(rv, vv);
        if (flag("OVFIND")) {
            clearFlag("OVFIND"); // GETECC
        }
        // GETECC: ECC = SQRT(1 - R1A P)
        double e2 = 1 - r1a * p;
        ecc = Math.sqrt(Math.max(0, e2));
        double rp = r1 * p / (1 + ecc);           // 0D = RP (+29/+27)
        periapse[0] = rp;
        if (r1a <= 0) {
            return DPPOSMAX[body]; // INFINAPO
        }
        double ra = 2 * r1 / r1a - rp;            // 2A - RP
        if (ra < 0 || ra >= DPPOSMAX[body]) {
            return DPPOSMAX[body];                // INFINAPO: LDPOSMAX
        }
        return ra;
    }
}
