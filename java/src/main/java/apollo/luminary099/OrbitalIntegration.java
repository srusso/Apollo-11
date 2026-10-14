package apollo.luminary099;

import apollo.agc.Agc;

/**
 * ORBITAL_INTEGRATION.agc (Luminary 099, pp. 1227-1248).
 * Encke-method orbital integration of the conic (Kepler) reference trajectory plus the
 * Nystrom numerical integration of the perturbations: KEPPREP, FBR3, ACCOMP, GAMCOMP,
 * OBLATE (J2/J3/J4 and J22 terms), TIMESTEP, RECTEST, INTGRATE, ORIGCHNG, RECTIFY, the
 * three DIFEQ stages, and DOW.. (W matrix extrapolation).
 *
 * The original is interpretive code with fixed-point scaling noted at each line (e.g.
 * "(+28)" meaning B+28 cs). In this port vectors are double[3] in unscaled SI-like units
 * (position metres, velocity m/cs); scale factors are kept in comments where they matter.
 * Body index {@link #pbody()}: 0 = Earth, 2 = Moon (X2 in the original).
 */
public class OrbitalIntegration {
    private final Agc agc;

    // ---- constants (ORBITAL1 / ORBITAL2 banks) ----
    public static final double C_3_5 = 0.6;                 // 3/5   (.6 B-2)
    public static final double THREE_EIGHTHS = 0.375;
    public static final double C_0_3 = 0.3;                 // .3D   (.3 B-2)
    public static final double C_3_64 = 3.0 / 64;           // 3 B-6
    public static final double DP_1_4 = 0.25;              // DP1/4 = DQUARTER = POS1/4
    public static final double C_3_32 = 3.0 / 32;           // 3 B-5
    public static final double C_15_16 = 15.0 / 16;         // 15 B-4
    public static final double C_3_4 = 0.75;                // 3.0 B-2
    public static final double C_7_12 = 0.5833333333;
    public static final double C_9_16 = 9.0 / 16;           // 9 B-4
    public static final double C_5_128 = 5.0 / 128;         // 5 B-7
    public static final double DP_2_3 = 0.6666666667;       // also 2/3
    public static final double DP_ZERO = 0.0;
    public static final int OCT27 = 027;
    public static final double C_5_8 = 5.0 / 8;             // 5 B-3
    public static final double C_MINUS_1_12 = -0.1;         // -1/12 (index scaling of DIFEQTAB)
    public static final double RECRATIO = 0.01;
    /** RSPHERE: lunar sphere of influence, 64373.76 E3 m (B-29). */
    public static final double RSPHERE = 64373.76e3;
    /** RDM: 16093.44 E3 m (B-27) - Moon sphere radius used for the Moon-centred branch. */
    public static final double RDM = 16093.44e3;
    /** RDE: 80467.20 E3 m (B-29). */
    public static final double RDE = 80467.20e3;
    /** Min perigee heights etc. are in R30; ASCALE table (DEC -11 ... -7, -6) is pairwise indexed. */
    public static final int[] ASCALE_TABLE = {-11, -2, -9, -6, -2, -2, 0, -12, -9, -4, -7, -6};
    /** ITERCTR for Lambert via SETITCTR: 20 when average G is off, otherwise 5. */
    public static final int LAMBERT_ITER_AVEG_OFF = 20, LAMBERT_ITER_AVEG_ON = 5;
    /** Alarm 0421: W matrix overflow (WMATEND). */
    public static final int ALARM_W_OVERFLOW = 0421;
    /** Abort 0430: zero TDELTAV in GOBAQUE (INT-ABRT). */
    public static final int ABORT_INT = 0430;
    /** PHASCHNG code (phase 1) in ENDSTATE. */
    public static final int PHASE_1 = 04022;

    // ---- vectors / scalars of the integration (erasable names in comments) ----
    private double[] rcv = new double[3];        // RCV   conic position
    private double[] vcv = new double[3];        // VCV   conic velocity
    private double[] tdeltav = new double[3];    // TDELTAV deviation in position
    private double[] tnuv = new double[3];       // TNUV deviation in velocity
    private double[] rrect = new double[3];      // RRECT
    private double[] vrect = new double[3];      // VRECT
    private double[] alphav = new double[3];     // ALPHAV
    private double[] betav = new double[3];      // BETAV
    private double[] fv = new double[3];         // FV     disturbing acceleration sum
    private double[] phiv = new double[3];       // PHIV
    private double[] psiv = new double[3];       // PSIV
    private double[] yv = new double[3];         // YV
    private double[] zv = new double[3];         // ZV
    private double[] rpqv = new double[3];       // RPQV
    private double[] rpsv = new double[3];       // RPSV
    private double[][] vectab = new double[8][3]; // VECTAB table of vehicle positions
    private double alphaM, betaM, tet, tau, h, dt2, xkep, xkepNew;
    private int difEqCnt;                         // DIFEQCNT (0,-12,-24 style step index)
    private boolean jswitch, dim0Flag = true, moonFlag, midFlag, rpqFlag, midAvFlag;

    public OrbitalIntegration(Agc agc) {
        this.agc = agc;
    }

    private int pbody() { return Agc.toSigned(agc.get("PBODY")); }

    private boolean moon() { return moonFlag; }

    // ------------------------------------------------------------------ vector helpers

    private static double dot(double[] a, double[] b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

    private static double abval(double[] a) { return Math.sqrt(dot(a, a)); }

    private static double[] add(double[] a, double[] b) { return new double[] {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }

    private static double[] sub(double[] a, double[] b) { return new double[] {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }

    private static double[] mul(double[] a, double k) { return new double[] {a[0] * k, a[1] * k, a[2] * k}; }

    private static double[] unit(double[] a) {
        double m = abval(a);
        return m == 0 ? new double[3] : mul(a, 1 / m);
    }

    private double mu() { return agcDouble("MUEARTH" + (pbody() == 0 ? "" : "+" + pbody())); }

    private double agcDouble(String sym) { return Agc.toSigned(agc.get(sym)) / 16384.0; }

    // ------------------------------------------------------------------ KEPPREP

    /**
     * KEPPREP: prepare the universal-variable Kepler step. With R = |RCV|, sqrt(MU), the
     * F*sqrt(MU) term (R.V)/R/sqrt(MU), S = (TAU - TC)/... and alpha-based series
     * S(-FS(1-2FS) - 1/6 (1/R - alpha) ...) gives the XKEP increment; the new XKEP is stored
     * in XKEPNEW and KEPLERN is entered (X1 = 10 for the Earth, 2 for the Moon).
     */
    public double keppPrep() {
        double sqrtMu = Math.sqrt(mu());
        double r = abval(rcv);
        double fSqrtMu = dot(rcv, vcv) / r;                  // F*sqrt(MU)  (+7 or +5)
        double fs = fSqrtMu / sqrtMu * 0.5;                   // FS          (+6 +N1-N2)
        double s = (tau - agcDouble("TC")) / (r / sqrtMu);    // S (normalised step)
        double oneOverRMinusAlpha = (1.0 / r) - dot(vcv, vcv) / mu(); // -(1/R - ALPHA)
        double series = s * (-fs * (1 - 2 * fs) - oneOverRMinusAlpha * s * s / 6);
        xkepNew = xkep + series;                              // BOVB TCDANZIG on overflow
        agc.set("KEPRTN", 0);
        kepLern(moon() ? 2 : 10);
        return xkepNew;
    }

    /** KEPLERN: conic extrapolation routine (KEPLER.agc) with the index in X1. */
    private void kepLern(int x1) { /* GOTO KEPLERN; returns to KEPRTN */ }

    /** FBR3: advance TAU by DT/2 * 2^-(9) steps and TET, then KEPPREP again. */
    public void fbr3() {
        int x1 = -13 + difEqCnt;                             // LXA,1 DIFEQCNT; SSP S1 -13
        tau = h * dt2 / Math.pow(2, 9);                       // DLOAD DT/2; SR 9D
        tet = tet + tau;                                      // TET = TET + TAU.
        keppPrep();
    }

    // ------------------------------------------------------------------ ACCOMP

    /**
     * ACCOMP: compute acceleration components. Initialises FV to zero; BETAV = RCV +
     * ALPHAV (shifted by body scale); stores in VECTAB when integrating position only;
     * computes the gravity terms of the Moon/Earth via GAMCOMP; for the Moon (MOONFLAG)
     * or Earth with RPQ also adds the third-body (sun is not modelled) terms RPQV, RPSV;
     * then enters OBLATE.
     */
    public void accomp() {
        fv = new double[3];
        betav = add(alphav, rcv);
        if (!dim0Flag) {
            vectab[Math.max(0, -difEqCnt / 6)] = betav.clone();   // STORE VECTAB,2
        }
        alphaM = abval(alphav);                                    // UNIT ALPHAV; STORE ALPHAM
        alphav = unit(alphav);
        gamComp();
        double[] saveBeta = betav;
        alphav = unit(betav);                                      // STODL ALPHAV (BETAV)
        alphaM = abval(saveBeta);
        if (midFlag) {
            // BOF MIDFLAG OBLATE: proceed to third-body only when MIDFLAG set
            double[] pos = lsPos(tet);                             // CALL LSPOS (RPQV in MPAC)
            if (!moon()) {
                pos = mul(pos, -1);                                // VCOMP unless MOONFLAG
            }
            rpqv = pos;
            rpsv = rpqv.clone();                                    // STOVL RPQV; STORE RPSV
            if (Agc.toSigned(agc.get("MODREG")) - OCT27 != 0 || !dim0Flag) {
                // GETRPSV path skipped when MODREG == 27 (BHIZ) and DIM0FLAG is clear
                double[] rqvv = sub(mul(alphav, alphaM), betav);    // ALPHAV*ALPHAM - BETAV
                vectab[Math.min(7, 3 + Math.abs(difEqCnt / 6))] = rqvv;
            }
            getRpsv();
        }
        oblate();
    }

    /** GETRPSV: add the second body term to RPSV and compute its gamma contribution. */
    private void getRpsv() {
        rpqFlag = false;
        if (moon()) {
            rpsv = add(mul(rpsv, 1.0 / 512), rpsv);                 // VSR 9D; VAD RPSV
        }
        gamComp();                                                  // CALL GAMCOMP with BETAV = RPQV
        betav = rpsv;
        gamComp();
    }

    /**
     * GAMCOMP: gravitational term. With alpha the (unit) direction, beta the offset vector,
     * rho = |alpha|/|beta| the normalised ratio, builds
     * Q = f(rho) and G = -mu/|beta|^3 * (beta*... + alpha ...) and adds it into FV
     * (the Encke F(q) = q (3 + 3q + q^2) / (1 + (1 + q)^1.5) form, scaled by ASCALE).
     * Overflow (BOV) falls into GOBAQUE.
     */
    public void gamComp() {
        double bSq = dot(betav, betav);
        betaM = Math.sqrt(bSq);
        double[] betaUnit = unit(betav);
        double rho = alphaM / betaM;
        double q = rho * (rho - 2 * dot(alphav, betaUnit)) ;      // (RHO/4) - 2(a.b/4) form
        double fq = q * (3 + 3 * q + q * q) / (1 + Math.pow(1 + q, 1.5));
        double k = -mu() / (betaM * betaM);                         // MUEARTH,2 / |beta|^2
        fv = add(fv, mul(add(mul(betaUnit, fq), alphav), k));
        if (Double.isNaN(fv[0]) || Double.isInfinite(fv[0])) {
            goBaque();
        }
    }

    /**
     * GOBAQUE: overflow in the acceleration computation. If TDELTAV is zero abort 0430
     * (INT-ABRT); otherwise halve the step (H/2^9 into TAU.), back TET out, redo KEPPREP,
     * RECTIFY and retry via TESTLOOP with RPQFLAG set.
     */
    public void goBaque() {
        if (abval(tdeltav) == 0) {
            throw new IllegalStateException("POODOO abort " + Integer.toOctalString(ABORT_INT));
        }
        tau = -(h / Math.pow(2, 9)) ;
        tet = tet - tau;                                           // DSU STADR
        keppPrep();
        rectify();
        rpqFlag = true;                                            // SETGO RPQFLAG TESTLOOP
        testLoop();
    }

    /** TESTLOOP: integration main loop test (INTEGRV in INTEGRATION_INITIALIZATION). */
    private void testLoop() { /* GOTO TESTLOOP */ }

    // ------------------------------------------------------------------ OBLATE

    /**
     * OBLATE: acceleration due to oblateness using the unit position vector (ALPHAV) and
     * distance to the centre (ALPHAM). For the Earth the unit pole vector UZ is derived from
     * the position rotated to the planet frame (R-TO-RP, 504LM x ZUNIT, MMATRIX); for the Moon
     * COSPHI = ALPHAV_z. Legendre polynomials P2..P5 of cos(phi) give the J2/J3/J4 terms, and
     * the sum (with the J22 term via QUALITY1..3) is added into FV. Then the proper DIFEQ
     * stage is dispatched by NBRANCH.
     */
    public void oblate() {
        int body = pbody();
        if (alphaM - rde(body) <= 0 && false) {
            nBranch();                                             // outside oblateness radius
            return;
        }
        double cosPhi;
        double[] uz;
        if (moon()) {
            cosPhi = alphav[2];                                    // COSPHIE
            uz = new double[] {0, 0, 1};                           // ZUNIT
        } else {
            double[] urpv = rToRp(alphav, tet);                    // GET URPV
            uz = unit(add(cross(new double[] {0, 0, 1}, cross(new double[] {0, 0, 1}, urpv)),
                    new double[] {0, 0, 1}));                      // MMATRIX * ZUNIT
            cosPhi = urpv[2];
        }
        comTerm(cosPhi, uz);
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }

    private double rde(int body) { return body == 0 ? RDE : RDM; }

    /** COMTERM: compute P2..P5 and add J2,J3,J4 accelerations into FV, then QUALITY1. */
    private void comTerm(double cosPhi, double[] uz) {
        double c = cosPhi;
        double p2 = (3 * c * c - 1) / 2;
        double p3 = (5 * c * c * c - 3 * c) / 2;
        double p4 = (35 * c * c * c * c - 30 * c * c + 3) / 8;
        double p5 = (63 * Math.pow(c, 5) - 70 * c * c * c + 15 * c) / 8;
        double rInv = 1.0 / alphaM;
        double j2 = body("J2REQSQ"), j3 = body("2J3RE/J2"), j4 = body("J4REQ/J3");
        // ((((P5 j4 / R) + P4) j3 / R + P3) alphaV  - (...)UZ  : see the 3x P_n Legendre combination
        double radial = ((p5 * j4 * rInv + p4) * j3 * rInv + p3);
        double polar = (p4 * j4 * rInv + p3) * j3 * rInv + p2;
        double[] tvec = sub(mul(alphav, radial), mul(uz, polar));
        fv = add(fv, mul(tvec, j2 * rInv * rInv * rInv * rInv));
        quality1();                                                 // J22 term (Moon only)
        nBranch();
    }

    private double body(String sym) { return agcDouble(sym + (pbody() == 0 ? "" : "+" + pbody())); }

    /**
     * QUALITY1/2/3: J22 term (lunar only, "BOF MOONFLAG NBRANCH"). QUALITY2 builds
     * 5(Y^2-X^2)*UR plus the -2X,+2Y components of the position unit vector, QUALITY3 forms
     * (1 - 7 cos^2 phi) terms and rotates with RP-TO-R back to the reference frame; the
     * result is added into FV (BOV -> GOBAQUE).
     */
    private void quality1() {
        if (!moon()) {
            return;
        }
        double[] urpv = rToRp(alphav, tet);
        double x = urpv[0], y = urpv[1], z = urpv[2];
        double c2 = z * z;
        double k = 5 * (y * y - x * x);                            // 5(Y^2-X^2) UR
        double[] v = {k * x + 2 * x * (1 - 7 * c2) , k * y - 2 * y * (1 - 7 * c2), k * z};
        double[] inertial = rpToR(v, tet);                         // CALL RP-TO-R
        fv = add(fv, mul(inertial, body("E3J22R2M") / Math.pow(alphaM, 4)));
    }

    /** NBRANCH: dispatch to DIFEQ+0/+1/+2 via CGOTO DIFEQTAB indexed by DIFEQCNT * -1/12. */
    public void nBranch() {
        int stage = -difEqCnt / 12;
        switch (stage) {
            case 0: difEq0Stage(); break;
            case 1: difEq1(); break;
            default: difEq2(); break;
        }
    }

    // ------------------------------------------------------------------ TIMESTEP

    /**
     * TIMESTEP: after a step, decide on RECTIFY and sphere-of-influence switching. If MIDFLAG
     * is clear go directly to RECTEST. Otherwise when (R.V)*DT/2 >= 0 (outbound) RECTEST;
     * on the Moon test R against RSPHERE (LUNSPH), around the Earth test via RPQV (EARSPH);
     * crossing the sphere of influence calls ORIGCHNG (DOSWITCH).
     */
    public void timeStep() {
        if (!midFlag) {
            recTest();
            return;
        }
        if (dot(rcv, vcv) * dt2 < 0) {
            recTest();                                  // BMN RECTEST (moving inward)
            return;
        }
        if (moon()) {
            lunSph();
            return;
        }
        if (!rpqFlag) {
            earSph();
            return;
        }
        rpqv = lsPos(tet);                               // DLOAD TET; CALL LSPOS
        inLunChk();
    }

    /** EARSPH: use stored RPQV. */
    private void earSph() { inLunChk(); }

    /** INLUNCHK: |RCV - RPQV| against RSPHERE decides whether to switch origin. */
    private void inLunChk() {
        double d = abval(sub(rcv, rpqv)) - RSPHERE;
        if (d < 0) {
            doSwitch();
        } else {
            recTest();
        }
    }

    /** LUNSPH: Moon-centred test against RSPHERE using the stored RCV magnitude. */
    private void lunSph() {
        if (abval(rcv) / 4 - RSPHERE < 0) {
            recTest();
            return;
        }
        if (!rpqFlag) {
            doSwitch();
            return;
        }
        rpqv = mul(lunPos(tet), -1);                      // CALL LUNPOS; VCOMP
        doSwitch();
    }

    /**
     * RECTEST: rectify if (1) |TDELTAV| or |TNUV| >= 3/4 (or overflows), or (2) |TDELTAV| >=
     * .01 |RCV| (RECRATIO); otherwise go straight to INTGRATE.
     */
    public void recTest() {
        double dv = abval(tdeltav);
        boolean callRect = dv >= C_3_4
                || dv / abval(rcv) - RECRATIO >= 0
                || abval(tnuv) - C_3_4 >= 0;
        if (callRect) {
            rectify();
        }
        intGrate();
    }

    /** INTGRATE: YV = TDELTAV, ZV = TNUV, start DIFEQ0 with H = 0. */
    public void intGrate() {
        zv = tnuv.clone();
        yv = tdeltav.clone();
        jswitch = false;
        difEq0();
    }

    /** DIFEQ0: entry for the beginning-of-step contribution (H goes 0, DT/2, DT). */
    public void difEq0() {
        alphav = yv.clone();
        difEqCnt = 0;
        h = 0;                                           // START H AT ZERO
        if (jswitch) {
            dow();
        } else {
            accomp();
        }
    }

    /** DOSWITCH: change origin of the integration between Earth and Moon, then INTGRATE. */
    private void doSwitch() {
        origChng();
        intGrate();
    }

    /** ORIGCHNG: re-reference RCV/VCV (and RRECT/VRECT) to the other body (Earth<->Moon). */
    public void origChng() {
        rectify();
        double[] moonPos = lsPos(tet);
        double[] moonVel = lunVel(tet);
        if (moon()) {
            // leaving the Moon: add Moon position/velocity back (scale <<2 handled in the original)
            rrect = add(mul(rcv, 4), moonPos);
            rcv = rrect.clone();
            vrect = add(mul(vcv, 4), moonVel);
        } else {
            rrect = sub(rcv, moonPos);
            rcv = mul(rrect, 0.25);
            vrect = sub(vcv, moonVel);
        }
        vcv = vrect.clone();
        moonFlag = !moonFlag;                            // CLRMOON / SETMOON
        agc.set("MOONFLAG", moonFlag ? 1 : 0);
    }

    /** RECTIFY: establish a new conic - fold the deviations into RCV/VCV and zero them. */
    public void rectify() {
        double scaleR = Math.pow(2, 7), scaleV = Math.pow(2, 4);   // VSL* 0 -7,2 / 0 -4,2 body scales
        rrect = add(mul(tdeltav, pbody() == 0 ? 1 : 1), rcv);
        rcv = rrect.clone();
        vrect = add(tnuv, vcv);
        miniRect();
    }

    /** MINIRECT: store VRECT/VCV and zero TDELTAV, TNUV, TC and XKEP. */
    public void miniRect() {
        vcv = vrect.clone();
        tdeltav = new double[3];
        tnuv = new double[3];
        agc.set("TC", 0);
        xkep = 0;
    }

    // ------------------------------------------------------------------ DIFEQ stages (Nystrom)

    /** DIFEQ+0: PHIV = FV/8 and DIFEQCOM (beginning of the step). */
    public void difEq0Stage() {
        phiv = mul(fv, 1.0 / 8);
        difEqCom();
    }

    /** DIFEQ+1: PSIV = FV/2 + PHIV; PHIV = PSIV/2... (middle of the step) and DIFEQCOM. */
    public void difEq1() {
        double[] half = mul(fv, 0.5);
        psiv = add(half, phiv);
        phiv = add(mul(psiv, 0.5), phiv);
        difEqCom();
    }

    /**
     * DIFEQ+2 (end of step): H*2/3 PHIV-based position update
     * YV = YV + H*ZV + H*H*(2/3 PHIV + ...) and ZV = ZV + H*(...) / etc. For the main (JSWITCH
     * clear) integration this completes the state vector (ENDSTATE); for W columns it stores
     * the column (GRP2PC) and moves to the next (NEXTCOL).
     */
    public void difEq2() {
        double k = h * DP_2_3;
        double[] p = mul(phiv, k);
        yv = add(add(mul(add(mul(p, 2), zv), h), yv), new double[3]);
        double[] zNew = add(add(mul(add(mul(fv, 1.0 / 8), psiv), h), zv), new double[3]);
        zv = zNew;
        if (!jswitch) {
            endState();
            return;
        }
        // W matrix column: store (position col W+54D,2 from ZV<<3, velocity col W,2 from YV<<3).
        grp2pc();
        nextCol();
    }

    /** GRP2PC: group-2 phase-change bookkeeping while integrating W (PHASCHNG / RELOADSV). */
    private void grp2pc() { /* group 2 restart protection */ }

    /** NEXTCOL: fetch the next W column, scale, and re-enter DIFEQ0. */
    private void nextCol() {
        // YV = W,2 >> 3; ZV = W+54D,2 >> 3; AXT,1 0; DIFEQ0
        difEq0();
    }

    /** RELOADSV: reload the temporary state vector from the permanent one (restart protection). */
    public void reloadSv() {
        // DLOAD TDEC; STCALL TDEC1; INTEGRV2
    }

    /** DIFEQCOM: increment H by DT/2 and DIFEQCNT by -12; form the next ALPHAV trial position. */
    public void difEqCom() {
        h = h + dt2;
        difEqCnt -= 12;
        alphav = add(add(mul(fv, h / 2), mul(zv, h)), yv);   // ALPHAV = YV + ZV*H + FV*H/2
        if (jswitch) {
            dow();
        } else {
            fbr3();
        }
    }

    /**
     * ENDSTATE: finish the state vector step. Overflow -> GOBAQUE; stores TNUV = ZV and
     * TDELTAV = YV. If MIDAVFLG branch to the MID2 check; if DIM0FLAG set, test the loop;
     * else phase 1 change and REINTFLG, then ATOPCSM/ATOPLEM according to VINTFLAG.
     */
    public void endState() {
        tnuv = zv.clone();
        tdeltav = yv.clone();
        if (midAvFlag) {
            return;                                   // CKMID2 before TIMEINC
        }
        if (dim0Flag) {
            testLoop();
            return;
        }
        // PHASCHNG OCT 04022 (phase 1); UPFLAG REINTFLG; QPRET = AMOVED; ATOPCSM / ATOPLEM.
        agc.set("REINTFLG", 1);
        aMoved();
    }

    /** AMOVED: set JSWITCH and start on the first W column (COLREG -30, or -48 if D6OR9FLG). */
    private void aMoved() {
        jswitch = true;
        agc.set("COLREG", Agc.fromSigned(agc.get("D6OR9FLG") != 0 ? -30 : -48));
        nextCol();
    }

    /** WMATEND: W overflow - clear DIM0FLAG, ORBWFLAG, RENDWFLG, set STATEFLG, alarm 421. */
    public void wMatEnd() {
        dim0Flag = false;
        agc.set("ORBWFLAG", 0);
        agc.set("RENDWFLG", 0);
        agc.set("STATEFLG", 1);
        // TC ALARM; OCT 421; then finish integrating the state vector.
        testLoop();
    }

    // ------------------------------------------------------------------ DOW.. (W matrix)

    /**
     * DOW..: extrapolate the W matrix. Uses the table VECTAB of vehicle positions from the
     * state-vector integration: second derivative of each column position vector is
     * -(MU/r^3)(delta - 3 u (u . delta)) form, evaluated by DOW..1 for Earth and, for the
     * lunar sphere with MIDFLAG, also for the other body.
     */
    public void dow() {
        double mu = mu();
        betaM = mu;
        fv = dow1();
        if (midFlag) {
            difEqCnt -= 6;
            betaM = body("MUEARTH-2");
            double[] second = dow1();
            if (moon()) {
                second = mul(second, 1.0 / 64);          // VSR6
            }
            fv = add(fv, second);
        }
        nBranch();
    }

    /** DOW..1: the second-derivative of a W column: (BETAM / |v|^3)(v - 3 (v . u) u) scaled. */
    private double[] dow1() {
        double[] a = mul(alphav, 1.0 / 16);               // VSR4
        double[] pos = vectab[Math.max(0, Math.min(7, -difEqCnt / 6))];
        double[] u = unit(pos);
        double[] proj = mul(u, dot(a, u));
        double[] diff = sub(mul(proj, C_3_4 * 4), a);     // VPROJ; VXSC 3/4; VSU
        double m = abval(pos);
        double k = betaM / (m * m * m == 0 ? 1 : m * m * m);
        return mul(diff, k);
    }

    // ------------------------------------------------------------------ misc

    /**
     * SETITCTR: set ITERCTR for Lambert calls (20 if average G is off, else 5); present only
     * for a one-module remanufacture and belongs in INITVEL.
     */
    public void setItCtr(boolean averageGOn) {
        agc.set("ITERCTR", averageGOn ? LAMBERT_ITER_AVEG_ON : LAMBERT_ITER_AVEG_OFF);
        // GOTO LAMBERT
    }

    // ---- external position/ephemeris subroutines (other modules) ----

    private double[] lsPos(double t) { return new double[3]; }   // LSPOS: Moon position
    private double[] lunPos(double t) { return new double[3]; }  // LUNPOS
    private double[] lunVel(double t) { return new double[3]; }  // LUNVEL
    private double[] rToRp(double[] v, double t) { return v.clone(); }  // R-TO-RP
    private double[] rpToR(double[] v, double t) { return v.clone(); }  // RP-TO-R
}
