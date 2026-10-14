package apollo.luminary099;

import apollo.agc.Agc;

/**
 * AOSTASK and AOSJOB (Luminary 099, $$/DAPAO): the 1/ACCS job. It computes, from the current LM mass and
 * configuration, the inverse jet accelerations (1JACC), the Q/R-axis coupling coefficients, the acceleration
 * derivative terms (ACCDOTQ/R, QACCDOT/RACCDOT) and, from the acceleration offset (AOS) estimates, the
 * U/V/P-axis jet-switching deadbands and 1/ANET, ACCFUN tables used by the RCS DAP.
 *
 * Numeric model: every AGC single-precision word is a fraction in [-1,1) (word/16384). The helpers {@link #f(String)}
 * and {@link #setF(String, double)} convert to/from the one's-complement word held in {@link Agc}. Scale factors
 * quoted in the original comments (PI/2, 2^7/PI, ...) are therefore folded into the constants, exactly as in the
 * fixed-point code, and are noted in the comments where they matter.
 */
public class AostaskAndAosjob {
    private final Agc agc;

    // Fraction constants from the fixed-fixed pool
    private static final double ONE_LSB = 1.0 / 16384.0;
    private static final double POSMAX = 16383.0 / 16384.0;
    private static final double NEGMAX = -POSMAX;
    private static final double BIT14 = 0.5, BIT13 = 0.25, BIT12 = 0.125, BIT9 = 256.0 / 16384.0, BIT8 = 128.0 / 16384.0;
    private static final double BIT10 = 512.0 / 16384.0;

    // Local constants
    /** -.03R/S2: OCT 77377 = minus 2^-6; AMIN at PI/2 is its negative. */
    public static final double MINUS_AMIN = -BIT9;
    public static final double AMIN = BIT9;
    /** 1/.03 = POSMAX (2^7/PI scaling). */
    public static final double ONE_OVER_AMIN = POSMAX;
    /** .0125RS = BIT8: AMIN/2. */
    public static final double HALF_AMIN = BIT8;
    /** ZONE3MAX: 17.5 ms (35 ms for 1 jet) at 4 seconds. */
    public static final double ZONE3MAX = 0.004375;
    /** FLATVAL: 0.8 at PI/4 rad. */
    public static final double FLATVAL = 0.01778;
    /** DGBF: .3 scaled at 1/2. */
    public static final double DGBF = 0.6;
    /** 0.35356 (.70711 scaled at 2), .7071, -.7071. */
    public static final double C_0_35356 = 0.35356, C_P_7071 = 0.70711, C_M_7071 = -0.70711;
    /** -EPSMAX: epsilon is limited to a maximum of .42265. */
    public static final double MINUS_EPSMAX = -0.42265;
    /** -.1875 PI/2 rad/sec^2 scaled at PI/2. */
    public static final double MINUS_0_1875 = -0.18750;
    /** GFACTM: 979.24/2.20462 at B+15 (OCT 337). */
    public static final double GFACTM = 0337 / 16384.0;
    /** TORQCONS: 0.51443 B-14, corresponds to 500 lb-ft. */
    public static final double TORQCONS = 0.51443 / 16384.0;
    /** 1JACCCON: OCT 167 scaled at PI/4 x 2^16 rad/sec^2-kg. */
    public static final double ONE_JACC_CON = 0167 / 16384.0;

    /** Gimbal-motion bits in channel 12, pairs: Q (non-zero test, positive-direction test), R. */
    public static final int[] GIMBLBTS = {01400, 01000, 06000, 04000};
    /** Jet-failure masks against CH5MASK: -U, -V, +U, +V. */
    public static final int[] MINUS_UMASK = {00110, 00022};
    public static final int[] PLUS_UMASK = {00204, 00041};

    /**
     * INERCONA (2DEC): inverse-inertia numerators A. Index = (JX/2)+1: L, then descent P,Q,R, then ascent P,Q,R.
     */
    public static final double[] INERCONA = {
        +.0410511917, +.0059347674, +.0014979264, +.0010451889, +.0065443852, +.0035784354, +.0056946631
    };
    /** INERCONB: B coefficients, index (JX/2)+1 with L first (L B desc, then P,Q,R descent, then P,Q,R ascent). */
    public static final double[] INERCONB = {
        +.155044, +.002989, +.018791, +.021345, +.000032, +.162862, +.009312
    };
    /** INERCONC: C coefficients, same indexing as INERCONB. */
    public static final double[] INERCONC = {
        -.025233, +.008721, -.068163, -.066027, -.006923, +.002588, -.023608
    };

    /** COEFF: inertia (C,F,B,A,E,D) then CG (C,F,B,A,E,D) coefficients for the docked computation. */
    public static final double[] COEFF = {
        .19518, -.00529, -.17670, -.03709, .06974, .02569,
        .20096, .13564, .75704, -.37142, -.63117, .41179
    };

    public AostaskAndAosjob(Agc agc) {
        this.agc = agc;
    }

    // ---- helpers ----
    private double f(String sym) { return Agc.toSigned(agc.get(sym)) / 16384.0; }
    private void setF(String sym, double v) {
        double c = Math.max(NEGMAX, Math.min(POSMAX, v));
        agc.set(sym, Agc.fromSigned((int) Math.round(c * 16384.0)));
    }
    private boolean bit(String sym, String mask) { return (agc.get(sym) & agc.get(mask)) != 0; }

    /**
     * DVOVSUB: dividend / divisor with overflow protection. POSMAX for a zero divisor; if |divisor| <= |dividend|
     * the quotient would not fit, so the signed maximum is returned.
     */
    public static double dvOvSub(double dividend, double divisor) {
        if (divisor == 0.0) return POSMAX;
        if (Math.abs(divisor) <= Math.abs(dividend)) {
            return ((dividend > 0) == (divisor > 0)) ? POSMAX : NEGMAX;
        }
        return dividend / divisor;
    }

    /** INVERT: invert -input at PI/2 giving BIT9/input (2^7/PI scaling). */
    public static double invert(double input) {
        return BIT9 / input;
    }

    // ---- entry points ----

    /** 1/ACCSET: entry from fresh start/restart coding. Nulls the offset estimates then falls into 1/ACCJOB. */
    public void oneAccSet() {
        agc.set("AOSQ", 0);
        agc.set("AOSR", 0);
        agc.set("ALPHAQ", 0);
        agc.set("ALPHAR", 0);
        oneAccJob();
    }

    /** 1/ACCJOB: run 1/ACCS via BANKCALL (skipping its EBANK setting) then ENDOFJOB. */
    public void oneAccJob() {
        oneAccs();
        // TC ENDOFJOB
    }

    /** Scratch (MPAC) values carried between the 1/ACCS sections. */
    private int mpac;          // JX base index
    private boolean docked;    // DOCKTEMP
    private double lemMass;
    private double epsilon;
    private double coeffQ, coeffR;
    private double oneJaccU, oneJaccV;

    /** 1/ACCS: compute the inverse acceleration/jet data. */
    public void oneAccs() {
        // EBANK set to 6; return address saved in ACCRETRN
        docked = (agc.get("DAPBOOLS") & agc.get("CSMDOCKD")) != 0;
        lemMass = docked ? f("MASS") - f("CSMMASS") : f("MASS");
        // INHINT
        boolean ascent = (agc.get("FLGWRD10") & agc.get("APSFLBIT")) != 0;   // staged or on lunar surface
        if (ascent) {
            setF("-2JETLIM", NEGMAX);           // always 2 jets for P-axis rate command
            mpac = 012;                         // initialise index at 12
            double hi = f("HIASCENT") - lemMass;
            if (hi <= 0) massFix(hi);           // mass too high: catch staging
            else {
                double lo = f("LOASCENT") - lemMass;
                if (lo > 0) massFix(lo);        // mass too low: limits the MASSMON decrementing
            }
        } else {                                 // DPSFLITE
            setF("-2JETLIM", -BIT10);           // four jets for P-axis rate errors > 1.4 deg/sec (scaled at 45)
            mpac = 6;                           // initialise index at 6
            double hi = f("HIDESCNT") - lemMass;
            if (hi <= 0) massFix(hi);
            else {
                double lo = f("LODESCNT") + f("HIASCENT") - lemMass;
                if (lo > 0) massFix(lo);
            }
        }
        fMass();
    }

    /** MASSFIX: store the violated limit as LEMMASS and correct total MASS (low-order word zeroed). */
    private void massFix(double delta) {
        lemMass += delta;
        double total = docked ? lemMass + f("CSMMASS") : lemMass;
        setF("MASS", total);
        agc.set("MASS+1", 0);
    }

    /** F(MASS): RELINT then branch to the docked or undocked computation. */
    private void fMass() {
        if (docked) {
            dockedCase();
            return;
        }
        // J = 2,1,0 for 1JACCR, 1JACCQ, 1JACC (P); JX = 10,8,6 (ascent) or 4,2,0 (descent)
        int jx = mpac - 2;
        for (int j = 2; j >= 0; j--, jx -= 2) {
            stctr1(jx, j);
        }
        mpac = jx + 2;                          // last JX used (6 ascent / 0 descent)
        commEqs();
    }

    /** STCTR1: 1JACC(J) = A(JX)/(MASS + C(JX)) + B(JX). */
    private void stctr1(int jx, int j) {
        int idx = jx / 2 + 1;
        double denom = lemMass + INERCONC[idx];
        double v = INERCONA[idx] / denom + INERCONB[idx];
        String[] names = {"1JACC", "1JACCQ", "1JACCR"};
        if (j >= 0) setF(names[j], v);
        else setF("L,PVT-CG", v);               // 1JACC(-1) = L,PVT-CG at 8 feet
    }

    /** COMMEQS: Q/R coupling coefficients and the U,V inverse accelerations. */
    private void commEqs() {
        double q = f("1JACCQ"), r = f("1JACCR");
        if (q - r > 0) {                        // IR greater than IQ
            epsilon = (q - r) / q;              // 1 - IQ/IR
            if (epsilon + MINUS_EPSMAX > 0) epsilon = -MINUS_EPSMAX;   // limited to .42265
            coeffR = epsilon * C_0_35356 + C_P_7071;        // .707(1 + .5 eps)
            coeffQ = (epsilon - POSMAX) * coeffR;           // -.707(1+.5eps)(1-eps)
        } else {                                // BIGIQ: epsilon = 1 - IR/IQ, -epsilon is computed
            double negEps = (q - r) / r;
            if (-negEps + MINUS_EPSMAX > 0) negEps = MINUS_EPSMAX;
            coeffQ = negEps * C_0_35356 + C_M_7071;         // -.707(1 + .5 eps)
            coeffR = (-negEps + NEGMAX) * coeffQ;           // .707(1+.5eps)(1-eps)
        }
        jaccUv();
    }

    /** JACCUV: 1JACCU = 1JACCV = (-COEFFQ*1JACCQ + COEFFR*1JACCR)/2 (scaled PI/2). */
    private void jaccUv() {
        oneJaccU = (-coeffQ * f("1JACCQ") + coeffR * f("1JACCR")) * BIT14;
        oneJaccV = oneJaccU;
        setF("1JACCU", oneJaccU);
        setF("1JACCV", oneJaccV);
        if (mpac > 0) {                         // ascent
            agc.set("ALLOWGTS", 0);
            agc.set("INGTS", 0);                // 1/ACCONT -1: zero INGTS in ascent
            oneAccOnt();
            return;
        }
        // Descent: compute L,PVT-CG too (JX = -2, J = -1), then LRESC
        stctr1(-2, -1);
        lResc();
    }

    /** LRESC: jerk terms for descent (ACCDOTR/Q) from delta V, mass and the CG/pivot distance. */
    private void lResc() {
        double x = dvOvSub(f("ABDELV") * f("MASS"), GFACTM);
        x = x * f("L,PVT-CG");                  // scaled at 8 feet
        double accDotR = dvOvSub(x * f("1JACCR"), f("TORKJET1"));
        double accDotQ = dvOvSub(x * f("1JACCQ"), f("TORKJET1"));
        setF("ACCDOTR", accDotR);
        spsCont(accDotQ, accDotR);
    }

    /** SPSCONT: K, KSQ terms and signed jerk terms QACCDOT/RACCDOT, then dock/GTS tests. */
    private void spsCont(double accDotQ, double accDotR) {
        setF("ACCDOTQ", accDotQ);
        setF("ACCDOTR", accDotR);
        setF("KQ", accDotQ * DGBF);             // .3 ACCDOTQ scaled at PI/2(8)
        setF("KRDAP", accDotR * DGBF);
        int chan12 = agc.get("CHAN12");         // gimbal drive bits 9-12
        double[] mag = {accDotQ, accDotR};
        String[] out = {"QACCDOT", "RACCDOT"};
        for (int idx = 2; idx >= 0; idx -= 2) { // LOOP3: R first (index 2) then Q (index 0)
            double v;
            if ((chan12 & GIMBLBTS[idx]) == 0) v = 0;                 // no gimbal motion
            else if ((chan12 & GIMBLBTS[idx + 1]) == 0) v = mag[idx / 2];   // negative rotation: positive
            else v = -mag[idx / 2];                                   // positive rotation: negative
            setF(out[idx / 2], v);
        }
        boolean usable = (agc.get("DAPBOOLS") & agc.get("USEQRJTS")) == 0;
        if (!usable || agc.get("T5ADR") != agc.get("PAXISADR")) {
            downGts();
            return;
        }
        if (agc.get("INGTS") == 0) {
            // IBNKCALL TIMEGMBL: null offset and find ALLOWGTS
        }
        dockTest();
    }

    /** DOWNGTS: zero switches when USEQRJTS bit is up or the DAP is off. */
    private void downGts() {
        agc.set("ALLOWGTS", 0);
        agc.set("INGTS", 0);
        dockTest();
    }

    /** DOCKTEST: bypass 1/ACCONT when docked. */
    private void dockTest() {
        if (docked) oneAccRet();
        else oneAccOnt();
    }

    /** DOCKED: separate computation of inverse inertias/CG for the docked configuration (CSM + LEM). */
    private void dockedCase() {
        double[] masses = {lemMass, f("CSMMASS")};      // MASSCTR 0 = LEM, 1 = CSM
        double[] acc = new double[2];                   // [0] inertia term, [1] CG term
        for (int pass = 0; pass < 2; pass++) {
            int coefCtr = pass == 0 ? 1 : 7;            // 1 = inertia coefficients, 7 = CG coefficients
            int base = coefCtr - 1;                      // COEFF - 1 = C
            double x = masses[1], y = masses[0];         // X = CSMMASS, Y = LEMMASS
            double mp = COEFF[base] * y * x + COEFF[base + 1];     // C X Y + F
            for (int massCtr = 1; massCtr >= 0; massCtr--) {
                int cc = coefCtr - (massCtr == 1 ? 0 : 1);
                double m = masses[massCtr];
                int a = massCtr == 1 ? base + 3 : base + 2;        // COEFF+2: A (CSM) or B (LEM)
                int e = massCtr == 1 ? base + 5 : base + 4;        // COEFF+4: D (CSM) or E (LEM)
                mp += (COEFF[a] * m + COEFF[e]) * m;                // (A X + D) X + (B Y + E) Y
            }
            acc[pass] = mp;
        }
        double inertia = acc[0];                         // x 2^38 kg-cm^2
        double cgTerm = acc[1];                          // CG x DELDOT = x 4 PI rad-cm/sec
        double oneJacc = dvOvSub(ONE_JACC_CON, f("MASS"));
        setF("1JACC", oneJacc);                          // scaled at PI/4
        for (String s : new String[] {"1/ANETP", "1/ANET2+1", "1/ANET2+2", "1/ANET2+17", "1/ANET2+18"}) {
            setF(s, POSMAX);                             // inverse jet accelerations -> accel of 1.4 d/ss
        }
        double q = TORQCONS / inertia;
        setF("1JACCQ", q);
        setF("1JACCR", q);
        coeffQ = C_M_7071;                               // chosen to make U and V axes orthogonal
        coeffR = C_P_7071;
        setF("COEFFQ", coeffQ);
        setF("COEFFR", coeffR);
        double accDotR = dvOvSub(f("MASS") * cgTerm * f("ABDELV"), inertia);
        spsCont(q == 0 ? 0 : accDotR * 0 + 0, accDotR);  // ACCDOTR stored, then SPSCONT with ACCDOTQ=ACCDOTR*.. below
    }

    /**
     * 1/ACCONT: initialise deadband values and compute the U/V (and P) axis data.
     * (Entry "1/ACCONT -1" zeroes INGTS in ascent first; done by the caller.)
     */
    private double dbVal2, dbVal3;
    private double drifter, flatEmp, z3Tem;
    private double[] atem = new double[10];     // 1/ATEM1 buffer: [0..9], see layout below
    private double[] acostt = new double[2];    // 1/ACOSTT, +1
    private double[] dbb = new double[4];       // DBB1, DBB2, DBB4, DBB3 in memory order
    private double[] udb = new double[4];
    private double[] axdstem = new double[2], uaxdist = new double[2];
    private int accSw;

    public void oneAccOnt() {
        double db = f("DB");
        double l = db * BIT13;                  // 0.25 DB
        dbVal3 = l + l;                         // 0.50 DB
        dbVal2 = l - db;                        // -.75 DB
        setF("DBVAL2", dbVal2);
        setF("DBVAL3", dbVal3);
        getAosUv();
    }

    /** GETAOSUV: AOSU, AOSV from AOSQ, AOSR (via ROT-TOUV), drift/GTS flags, P-axis data, then U and V axes. */
    private void getAosUv() {
        double[] uv = rotToUv(f("AOSQ"), f("AOSR"));
        double aosU = uv[0], aosV = uv[1];
        setF("AOSU", aosU);
        setF("AOSV", aosV);
        drifter = (agc.get("DAPBOOLS") & agc.get("DRIFTBIT")) != 0 ? 1 : 0;
        setF("DRIFTER", drifter);
        flatEmp = (drifter + agc.get("ALLOWGTS") > 0) ? FLATVAL : 0;   // drift or GTS near
        if (flatEmp != 0) {
            z3Tem = drifter > 0 ? ZONE3MAX : 0;
        }
        // DOPAXIS
        double oneJacc = f("1JACC");
        double funTem = oneJacc + BIT9;                 // 1 + ANET/ACOAST at 2^6
        double inv = invert(oneJacc);                   // 1/ANETP at 2^7/PI
        setF("1/ANETP", inv);
        setF("1/ANETP+1", inv);
        double pAccFun = (-BIT9 * inv) / funTem;        // -1/(ANET + ANET^2/ACOAST)
        setF("PACCFUN", pAccFun);
        setF("PACCFUN+1", pAccFun);
        setF("1/ACOSTP", ONE_OVER_AMIN);                // no AOS for P axis: ACOAST = AMIN
        setF("1/ACOSTP+1", ONE_OVER_AMIN);
        if (drifter > 0) { aosU = 0; aosV = 0; setF("AOSU", 0); setF("AOSV", 0); }
        for (int uvAxis = 0; uvAxis < 2; uvAxis++) {
            bothAxes(uvAxis, uvAxis == 0 ? aosU : aosV, uvAxis == 0 ? oneJaccU : oneJaccV);
        }
        finishAxes();
    }

    /** ROT-TOUV: rotates (AOSQ, AOSR) into (AOSU, AOSV). Provided by another bank; override if needed. */
    protected double[] rotToUv(double q, double r) {
        return new double[] {q, r};
    }

    /** BOTHAXES: code common to U and V axes (uv = 0 for U, 1 for V). */
    private void bothAxes(int uv, double aos, double oneJaccUv) {
        int signAos = aos < 0 ? 1 : 0;
        double absAos = Math.abs(aos);
        int minusSign = -signAos;               // -SIGNAOS used as an index
        double db = f("DB");
        dbb[0] = db; dbb[1] = db;               // DB1, DB2 = DBVAL1 (= DB)
        dbb[2] = 0; dbb[3] = 0;
        atem = new double[10];
        accSw = 0;
        if (absAos + MINUS_AMIN <= 0) {         // NOTMUCH: ABS(AOS) less than AMIN
            acostt[0] = ONE_OVER_AMIN;
            acostt[1] = ONE_OVER_AMIN;
            if (flatEmp == 0) {
                if (HALF_AMIN + (absAos - AMIN) <= 0) {   // NOAOS: less than AMIN/2
                    dbb[3] = db; dbb[2] = db;
                } else {                        // SOMEAOS: AMIN/2 < |AOS| < AMIN
                    dbb[3 + minusSign] = dbVal3;
                    dbb[2 + signAos] = dbVal3 + dbVal3;
                }
            }
        } else {                                // BIGAOS
            if (flatEmp == 0) {
                dbb[1 + minusSign] += db;       // DB2(1) = 2 DB
                dbb[2 + signAos] = dbb[1 + minusSign];
                double a;
                if (MINUS_0_1875 + absAos <= 0) a = -dbVal3;   // -.5 DB
                else a = ((-4 * absAos + BIT14) * 2) * db;     // (1 - 8 ABSAOS) DB
                if (MINUS_0_1875 + absAos <= 0) a = (1 - 8 * absAos) * db;
                else a = -dbVal3;
                dbb[0 + signAos] = a;
                dbb[3 + minusSign] = dbVal2;    // DB3(4) = -.75 DB
            }
            // SKIPDB1
            double inv = invert(absAos * BIT12 + absAos);       // 1/ABS(AOS), (9/8) ABSAOS
            acostt[1 - signAos] = inv;          // 1/ACOASTPOS(NEG)
            acostt[signAos] = ONE_OVER_AMIN;
            double sum = absAos + oneJaccUv + oneJaccUv + BIT9;
            if (sum >= 1.0) {                   // overflow: rescale to PI
                double anetRaw = absAos * BIT14 + oneJaccUv;
                double term = anetRaw + BIT8;
                double oneAnet = invert(anetRaw) * BIT14;
                double accFun = (-BIT8 * oneAnet) / term;
                acctHere(signAos, accFun, oneAnet, absAos, oneJaccUv, uv, aos);
                return;
            }
        }
        // SKIPDB2: ANETPOS(NEG) max = 2 JACC + ABS(AOS)
        double anet = absAos + oneJaccUv + oneJaccUv;
        double[] r = do1NetPlus(anet, anet);
        acctHere(signAos, r[1], r[0], absAos, oneJaccUv, uv, aos);
    }

    /** DO1/NET+: returns {1/ANET, ACCFUN} for raw ANET and term (= ANET/ACOAST at 2^6). */
    private double[] do1NetPlus(double anetRaw, double term) {
        double anetVar = term + BIT9;           // 1 + ANET/ACOAST at 2^6
        double oneAnet = invert(anetRaw);       // 1/ANET at 2^7/PI
        double accFun = (-BIT9 * oneAnet) / anetVar;
        return new double[] {oneAnet, accFun};
    }

    /** ACCTHERE and the rest of the per-axis computation (min/negative side, failures, deadbands, buffers). */
    private void acctHere(int signAos, double accFun, double oneAnet, double absAos, double oneJaccUv, int uv, double aos) {
        int minusSign = -signAos;
        atem[9 + minusSign] = accFun;           // Z5TEM+2 (max, positive-side)
        atem[3 + minusSign] = oneAnet;          // 1/ATEM2+2
        // minimum value on this side
        double sum = absAos + oneJaccUv + BIT9;
        double anet = sum >= 1.0 ? POSMAX + MINUS_AMIN : sum + MINUS_AMIN;   // truncate to PI/2 on overflow
        double[] r = do1NetPlus(anet, anet);
        atem[7 + minusSign] = r[0] == 0 ? 0 : r[1];     // Z5TEM
        atem[1 + minusSign] = r[0];             // 1/ATEM2
        // negative(positive) side, maximum jets
        double a = -absAos + oneJaccUv + oneJaccUv;
        double[] n = oneAnetMinus(a, signAos);
        atem[8 + signAos] = n[1];               // Z1TEM+2
        double savedAccFun = n[1];
        atem[2 + signAos] = n[0];               // 1/ATEM1+2
        // minimum jets, negative side
        double anetMin = -absAos + oneJaccUv;
        double[] m;
        if (anetMin + MINUS_AMIN <= 0) {        // FIXMIN
            accSw = signAos > 0 ? 1 : -1;       // AOS neg: +1; AOS pos: -1
            int idx = accSw + uv;
            boolean failed = (agc.get("CH5MASK") & ((idx <= 0 ? MINUS_UMASK : PLUS_UMASK)[idx <= 0 ? idx + 1 - 1 + (idx < 0 ? 0 : 0) : idx - 2 < 0 ? 0 : idx - 2])) != 0;
            if (failed) {
                m = oneNetMin(AMIN, signAos);   // jet failure: use AMIN
            } else {
                m = new double[] {n[0], savedAccFun};   // store max values for min jets
            }
        } else {
            m = oneNetMin(anetMin, signAos);
        }
        // STMIN-
        atem[6 + signAos] = m[1];               // Z1TEM
        atem[0 + signAos] = m[0];               // 1/ATEM1
        if ((agc.get("CH5MASK") & PLUS_UMASK[uv]) != 0) {   // +U (+V) jet failure
            atem[3] = atem[1];                  // 1/ATEM2+2 = 1/ATEM2
            atem[9] = atem[7];                  // Z5TEM+2 = Z5TEM
        }
        if ((agc.get("CH5MASK") & MINUS_UMASK[uv]) != 0) {  // -U (-V) jet failure
            atem[2] = atem[0];                  // 1/ATEM1+2 = 1/ATEM1
            atem[8] = atem[6];                  // Z1TEM+2 = Z1TEM
        }
        // DBFUN
        axdstem[0] = dbb[0] - dbb[3] + flatEmp;     // AXISDIST: DB1 - DB3 + FLAT
        axdstem[1] = dbb[1] - dbb[2] + flatEmp;     // DB2 - DB4 + FLAT
        String prefix = uv == 0 ? "1/ANET1" : "1/ANET1+16";
        if (uv == 0) {
            agc.set("ACCSWU", accSw & Agc.WORD_MASK);
        } else {
            agc.set("ACCSWV", accSw & Agc.WORD_MASK);
        }
        for (int i = 0; i < 10; i++) {          // GENTRAN: 10 words to the real place
            setF(prefix + "+" + i, atem[i]);
        }
        if (uv == 0) {
            udb = dbb.clone();                  // save U DBs for later storing
            uaxdist = axdstem.clone();
        } else {
            vdb = dbb.clone();
            vaxdist = axdstem.clone();
        }
    }
    private double[] vdb = new double[4], vaxdist = new double[2];

    /** 1/ANET-: returns {1/ANET, ACCFUN} for the opposite-sign maximum jets. */
    private double[] oneAnetMinus(double a, int signAos) {
        accSw = 0;
        double anet = a;
        if (anet + MINUS_AMIN <= 0) anet = AMIN;      // NETNEG: fake it as AMIN
        return oneNetMin(anet, signAos);
    }

    /** 1/NETMIN: term = ANET * 1/ACOASTPOS(NEG) then DO1/NET+. */
    private double[] oneNetMin(double anet, int signAos) {
        double term = anet * acostt[1 - signAos];
        return do1NetPlus(anet, term);
    }

    /** After both axes: P-axis deadbands, drift/GTS deadbands or normal storing, then 1/ACCRET. */
    private void finishAxes() {
        // STORV already stored V data; swap FLATEMP/Z3TEM into FLAT/ZONE3LIM
        setF("FLAT", flatEmp);
        setF("ZONE3LIM", z3Tem);
        double db = f("DB");
        setF("PDB1", db);
        setF("PDB2", db);
        setF("PDB3", db + flatEmp);
        setF("PDB4", db + flatEmp);
        setF("PAXDIST", 0);
        setF("PAXDIST+1", 0);
        if (flatEmp > 0) {                      // DRFDB: drift or GTS -- compute deadbands
            for (int off : new int[] {0, 1, 16, 17}) {
                setF("FIREDB+" + off, db);
                setF("COASTDB+" + off, db + flatEmp);
                setF("AXISDIST+" + off, 0);
            }
        } else {
            // U deadbands: FIREDB <- UDB1,UDB2; COASTDB <- UDB4,UDB3; AXISDIST <- UAXDIST
            setF("FIREDB", udb[0]);     setF("FIREDB+1", udb[1]);
            setF("COASTDB", udb[2]);    setF("COASTDB+1", udb[3]);
            setF("AXISDIST", uaxdist[0]); setF("AXISDIST+1", uaxdist[1]);
            // V deadbands
            setF("FIREDB+16", vdb[0]);  setF("FIREDB+17", vdb[1]);
            setF("COASTDB+16", vdb[2]); setF("COASTDB+17", vdb[3]);
            setF("AXISDIST+16", vaxdist[0]); setF("AXISDIST+17", vaxdist[1]);
        }
        oneAccRet();
    }

    /** 1/ACCRET: set ACCSOKAY in DAPBOOLS to indicate data good, return to caller (BANKJUMP ACCRETRN). */
    public void oneAccRet() {
        agc.set("DAPBOOLS", agc.get("DAPBOOLS") | agc.get("ACCSOKAY"));
    }
}
