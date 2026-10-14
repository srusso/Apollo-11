package apollo.comanche055;

import apollo.agc.Agc;

/**
 * ORBITAL_INTEGRATION (Comanche 055): Encke's method orbit integrator.
 *
 * The AGC interpretive code works with scaled fixed-point vectors; here vectors are
 * double[3] in SI-ish AGC units (metres, centiseconds), so the many SL/SR scaling shifts of
 * the original disappear. The structure is preserved:
 * <ul>
 * <li>KEPPREP / FBR3: prepare Kepler solution (conic) for the TAU. interval</li>
 * <li>ACCOMP / GAMCOMP / OBLATE: acceleration (deviation from conic) components</li>
 * <li>TIMESTEP / RECTEST / CHKSWTCH / ORIGCHNG / RECTIFY: rectification and origin change</li>
 * <li>DIFEQ+0, DIFEQ+1, DIFEQ+2, DIFEQCOM: Nystrom integration of the deviation (YV,ZV)</li>
 * <li>NEXTCOL / ENDSTATE / DOW..: the W (covariance transition) matrix extrapolation</li>
 * </ul>
 * Flags (MOONFLAG, MIDFLAG, DIM0FLAG, JSWITCH, RPQFLAG, D6OR9FLG, VINTFLAG, MIDAVFLG,
 * ORBWFLAG, RENDWFLG, STATEFLG, REINTFLG) are kept in {@link Agc} as 0/1 words.
 */
public final class OrbitalIntegration {
    // ---- constants (ORBITAL1 / ORBITAL2 banks) ----
    static final double THREE_8 = .375;
    static final double DQUARTER = .25;       // DP1/4
    static final double DP2_3 = .6666666667;  // DP2/3
    static final double SEVEN_12 = .5833333333;
    static final double HALFDP = .5;
    static final double THREE_4 = 3.0 / 4;    // 3/4 scaled B-2 in the original
    static final double NEG_1_12 = -.1;       // -1/12
    /** ASCALE: scaling shift table for GAMCOMP. */
    static final int[] ASCALE = {-11, -2, -9, -6, -2, -2, 0, -12, -9, -4, -7, -6};
    /** Gravitational constants, index PBODY/2: 0 = Earth, 1 = Moon (MUEARTH, MUM). Sun at -2. */
    static final double MU_SUN = 1.32715445e16;   // B-54
    static final double MU_MOON = 4.9027780e8;    // B-30
    static final double MU_EARTH = 3.986032e10;   // B-36
    /** Oblateness constants: 0 = Earth, 1 = Moon. */
    static final double J4REQ_J3_EARTH = .4991607391e7;
    static final double J4REQ_J3_MOON = -176236.02;
    static final double TWO_J3RE_J2_EARTH = -.1355426363e5;
    static final double TWO_J3RE_J2_MOON = .3067493316e18;  // moon J2 constant
    static final double J2REQSQ_EARTH = 1.75501139e21;
    static final double J2REQSQ_MOON = 9.20479048e16;       // 3J22R2MU (moon)
    static final double RECRATIO = .01;
    static final double RSPHERE = 64373.76e3;   // sphere of influence radius (m)
    static final double RDM = 16093.44e3;       // moon oblateness cutoff
    static final double RDE = 80467.20e3;       // earth oblateness cutoff

    private static final java.util.Map<Agc, OrbitalIntegration> INSTANCES = new java.util.WeakHashMap<>();

    /** Shared per-AGC instance, so the Java-held integration state (RCV, VCV, TC, ...) is common to all callers. */
    public static synchronized OrbitalIntegration of(Agc agc) {
        return INSTANCES.computeIfAbsent(agc, OrbitalIntegration::new);
    }

    private final Agc agc;
    private final PlanetaryInertialOrientation orient;

    // ---- state (erasable) ----
    double[] rcv = new double[3], vcv = new double[3];       // conic position/velocity
    double[] rrect = new double[3], vrect = new double[3];   // rectification vectors
    double[] tdeltav = new double[3], tnuv = new double[3];  // deviation pos / vel
    double[] yv = new double[3], zv = new double[3];         // integration pos/vel dev
    double[] fv = new double[3];                             // acceleration sum
    double[] phiv = new double[3], psiv = new double[3];
    double[] alphav = new double[3], betav = new double[3];
    double alpham, betam;
    double[] rpqv = new double[3], rpsv = new double[3];     // third-body positions
    double tet, tau, tc, h, dtHalf, xkep, xkepnew;
    int pbody;            // 0 = Earth, 2 = Moon
    int difeqcnt;         // 0, 12, 24 in original (stage 0..2 here)
    int colreg;           // W matrix column being integrated
    double[][] vectab = new double[6][3]; // table of positions for W integration
    double[][] w = new double[9][3];      // W matrix positions (columns)
    double[][] wv = new double[9][3];     // W matrix velocities

    public OrbitalIntegration(Agc agc) {
        this.agc = agc;
        this.orient = new PlanetaryInertialOrientation(agc);
    }

    private boolean flag(String f) { return agc.get(f) != 0; }
    private void set(String f) { agc.set(f, 1); }
    private void clear(String f) { agc.set(f, 0); }
    private double mu() { return pbody == 2 ? MU_MOON : MU_EARTH; }

    // ---- vector helpers ----
    private static double dot(double[] a, double[] b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
    private static double abval(double[] a) { return Math.sqrt(dot(a, a)); }
    private static double[] add(double[] a, double[] b) { return PlanetaryInertialOrientation.vad(a, b); }
    private static double[] sub(double[] a, double[] b) { return PlanetaryInertialOrientation.vsu(a, b); }
    private static double[] sc(double[] a, double k) { return PlanetaryInertialOrientation.vxsc(a, k); }
    private static double[] unit(double[] a) { double m = abval(a); return m == 0 ? new double[3] : sc(a, 1 / m); }
    private static boolean bad(double[] a) {
        return !Double.isFinite(a[0]) || !Double.isFinite(a[1]) || !Double.isFinite(a[2]);
    }

    // ================= Kepler preparation =================

    /**
     * KEPPREP: compute the new estimate XKEPNEW of the Kepler universal variable for
     * time interval TAU. - TC, then jump to KEPLERN (iterating on the conic).
     */
    public void kepprep() {
        double sqmu = Math.sqrt(mu());
        double r = abval(rcv);
        double[] u = unit(rcv);
        double udotv = dot(u, vcv);          // F*SQRT(MU)
        double dt = tau - tc;
        double s = dt / r;                   // S
        double fs = udotv * s / 1.0;         // FS (dimensionless)
        double ssqOverMu = s * s / mu();     // SSQ/MU
        double alphaTerm = dot(vcv, vcv) / mu() - 1 / r; // (1/R - ALPHA)
        // Series: S*(-FS(1-2FS) - (1/6)(S^2/MU)(1/R - ALPHA) R ...) scaled by SQRT(MU)
        double series = -fs * (1 - 2 * fs) - (1.0 / 6.0) * ssqOverMu * alphaTerm * r;
        xkepnew = xkep + s * sqmu * series;
        // STQ KEPRTN ; AXC,1 10 (Earth) or 2 (Moon, MOONFLAG set) ; GOTO KEPLERN
        agc.set("KEPRTN", 0);
        agc.set("KEPLERN_N", flag("MOONFLAG") ? 2 : 10);
        ConicSubroutines conic = new ConicSubroutines(agc);   // KEPLERN
        conic.setBody(flag("MOONFLAG") ? 1 : 0);
        conic.rrect = rrect.clone();
        conic.vrect = vrect.clone();
        conic.tau = tau;
        conic.xkepnew = xkepnew;
        conic.tc = tc;
        conic.xprev = xkep;
        conic.kepler();
        rcv = conic.rcv;
        vcv = conic.vcv;
        tc = conic.tc;
        xkep = conic.xprev;
    }

    /** FBR3: advance TAU by DT and TET, then prepare Kepler solution (final stage). */
    public void fbr3() {
        int cnt = difeqcnt;                 // SSP -13 ... TIX
        double dtFull = dtHalf * 2;         // DLOAD DT/2 ; SR 9D
        tau += dtFull;                      // PUSH DAD TC ; STODL TAU.
        tet += dtFull;                      // DAD TET ; STCALL TET
        kepprep();
    }

    // ================= Acceleration components =================

    /** ACCOMP: compute acceleration components (FV) at the current trial position. */
    public void accomp() {
        fv = new double[3];
        betav = add(sc(alphav, 1.0), rcv);               // R = deviation + conic
        if (!flag("DIM0FLAG")) {                          // BOF DIM0FLAG +5
            vectab[difeqcnt / 12 % vectab.length] = betav.clone();   // STORE VECTAB,2
        }
        double[] dev = alphav.clone();
        alphav = unit(dev);                               // ALPHAV = UNIT(deviation)
        alpham = abval(dev);
        gamcomp();                                         // Encke deviation acceleration
        alphav = unit(betav);                              // (BETAV set unit by GAMCOMP)
        alpham = betam;
        if (!flag("MIDFLAG")) {                            // BOF MIDFLAG OBLATE
            oblate();
            return;
        }
        // third-body perturbations
        double[][] pos = lspos(tet);                        // CALL LSPOS: Sun and Moon positions
        rpqv = flag("MOONFLAG") ? sc(pos[0], -1) : pos[0];
        rpsv = pos[1];
        if (!flag("DIM0FLAG")) {
            vectab[(difeqcnt / 12 + 3) % vectab.length] = sub(sc(alphav, alpham), rpqv); // STORE VECTAB+6,2
        }
        // Moon (or Earth) as disturbing body
        fv = add(fv, thirdBody(mu() == MU_EARTH ? MU_MOON : MU_EARTH, rpqv));
        clear("RPQFLAG");
        if (!flag("MOONFLAG")) {
            fv = add(fv, thirdBody(MU_SUN, rpsv));         // VSR 9D ; VAD RPSV
        }
        oblate();
    }

    /** Direct plus indirect acceleration from a disturbing body at position rb. */
    private double[] thirdBody(double muB, double[] rb) {
        double[] r = sc(alphav, alpham);
        double[] d = sub(rb, r);
        double dm = abval(d);
        double rbm = abval(rb);
        return sc(sub(sc(d, 1 / (dm * dm * dm)), sc(rb, 1 / (rbm * rbm * rbm))), muB);
    }

    /**
     * GAMCOMP: Encke acceleration due to departure from the conic. Input: ALPHAV, ALPHAM =
     * deviation unit/magnitude, BETAV = full position. Output: FV += -mu/R^3 * (delta - f(q) R);
     * BETAV, BETAM are left as unit(R), |R|. Overflow raises GOBAQUE.
     */
    public void gamcomp() {
        double[] delta = sc(alphav, alpham);
        double[] rr = betav.clone();
        double rm = abval(rr);
        double q = dot(delta, sub(delta, sc(rr, 2))) / (rm * rm);
        double fq = q * (3 + 3 * q + q * q) / (1 + Math.pow(1 + q, 1.5));
        double[] a = sc(add(delta, sc(rr, fq)), -mu() / (rm * rm * rm));
        fv = add(fv, a);
        betav = unit(rr);
        betam = rm;
        if (bad(fv)) {
            gobaque();
        }
    }

    /** OBLATE: acceleration due to oblateness of the primary; then dispatch DIFEQ stage via NBRANCH. */
    public void oblate() {
        double r = alpham;
        double cutoff = pbody == 2 ? RDM : RDE;
        if (r - cutoff >= 0) {                   // BPL NBRANCH: too far, no oblateness
            nbranch();
            return;
        }
        double[] u = alphav;
        double[] uz;
        double[] urpv = null;
        double cosphi;
        if (!flag("MOONFLAG")) {                 // COSPHIE
            uz = new double[] {0, 0, 1};
            cosphi = u[2];
        } else {
            urpv = orient.rToRp(u, tet, true);   // CALL R-TO-RP
            double[] zunit = {0, 0, 1};
            double[] lm = orient.lmVector();
            uz = unit(PlanetaryInertialOrientation.vxm(
                add(PlanetaryInertialOrientation.cross(lm, zunit), zunit), orient.mmatrix()));
            cosphi = dot(u, uz);
        }
        // zonal harmonics: a_n = mu J_n Re^n / r^(n+2) [ (n+1) Pn u - Pn'(z)(uz - z u) ]
        double z = cosphi;
        boolean moon = flag("MOONFLAG");
        double c2 = moon ? TWO_J3RE_J2_MOON : J2REQSQ_EARTH;
        double[] a = zonal(2, c2, z, u, uz, r);
        if (!moon) {
            double c3 = J2REQSQ_EARTH * TWO_J3RE_J2_EARTH / 2;
            double c4 = c3 * J4REQ_J3_EARTH;
            a = add(a, zonal(3, c3, z, u, uz, r));
            a = add(a, zonal(4, c4, z, u, uz, r));
        }
        fv = add(fv, a);
        if (moon) {
            quality3(urpv, r);
        }
        if (bad(fv)) {
            gobaque();
            return;
        }
        nbranch();
    }

    private static double[] zonal(int n, double c, double z, double[] u, double[] uz, double r) {
        double p, dp;
        switch (n) {
            case 2 -> { p = (3 * z * z - 1) / 2; dp = 3 * z; }
            case 3 -> { p = (5 * z * z * z - 3 * z) / 2; dp = (15 * z * z - 3) / 2; }
            default -> { p = (35 * Math.pow(z, 4) - 30 * z * z + 3) / 8; dp = (140 * z * z * z - 60 * z) / 8; }
        }
        double[] t = sub(sc(u, (n + 1) * p), sc(sub(uz, sc(u, z)), dp));
        return sc(t, c / Math.pow(r, n + 2));
    }

    /**
     * QUALITY3: lunar J22 term, evaluated in moon-fixed coordinates and rotated back
     * with RP-TO-R. Scale: 3 J22 R^2 MU / r^4.
     */
    private void quality3(double[] urpv, double r) {
        double k = J2REQSQ_MOON / Math.pow(r, 4);
        double w22 = urpv[0] * urpv[0] - urpv[1] * urpv[1];
        double[] a = {2 * urpv[0] - 5 * w22 * urpv[0], -2 * urpv[1] - 5 * w22 * urpv[1], -5 * w22 * urpv[2]};
        a = sc(a, k);
        a = orient.rpToR(a, tet, true);
        fv = add(fv, a);
    }

    /** NBRANCH: dispatch DIFEQ+0 / +1 / +2 (CGOTO DIFEQTAB indexed by DIFEQCNT * -1/12). */
    private void nbranch() {
        switch (difeqcnt) {
            case 0 -> difeq0Stage();
            case 1 -> difeq1();
            default -> difeq2();
        }
    }

    // ================= time step, rectification =================

    /** TIMESTEP: decide origin change / rectification then integrate. */
    public void timestep() {
        if (flag("MIDFLAG")) {                    // BOF MIDFLAG RECTEST
            double d = chkswtch();                // CALL CHKSWTCH
            if (d < 0) {
                doswitch();                       // BMN DOSWITCH
                return;
            }
        }
        rectest();
    }

    /** RECTEST: rectify when deviation too big (3/4 scaled limit or 1% of |RCV|). */
    private void rectest() {
        double limit = THREE_4;
        double tdv = abval(tdeltav);
        boolean rect = Double.isInfinite(tdv) || tdv >= limit * 1e7   // 1) 3/4 of full-scale
            || tdv / abval(rcv) >= RECRATIO                           // 2) 1% of |RCV|
            || abval(tnuv) >= limit * 1e4;
        if (rect) {
            rectify();                            // CALLRECT
        }
        intgrate();
    }

    /** DOSWITCH: change origin then integrate. */
    private void doswitch() {
        origchng();
        intgrate();
    }

    /** INTGRATE: start the Nystrom integration of the deviation. */
    private void intgrate() {
        zv = tnuv.clone();
        yv = tdeltav.clone();
        clear("JSWITCH");
        difeq0();
    }

    /** DIFEQ0: initialise H, DIFEQCNT and branch to ACCOMP (or DOW.. when JSWITCH). */
    private void difeq0() {
        alphav = yv.clone();
        difeqcnt = 0;
        h = 0;
        if (flag("JSWITCH")) {
            dow();
        } else {
            accomp();
        }
    }

    /**
     * CHKSWTCH: returns |R| - RSPHERE-like distance sign: negative means switch origin.
     * Gets the Moon's position (LUNPOS) if RPQFLAG not set.
     */
    public double chkswtch() {
        if (!flag("RPQFLAG")) {
            double[] pos = lunpos(tet);           // GET MOON POSITION
            rpqv = flag("MOONFLAG") ? sc(pos, -1) : pos;
        }
        double[] r = add(sc(tdeltav, 1.0), rcv);  // R = deviation + conic
        if (flag("MOONFLAG")) {                    // INSIDE moon sphere
            return abval(r) - RSPHERE / 4;      // negative when inside: switch to Earth
        }
        return abval(sub(r, rpqv)) - RSPHERE;      // OUTSIDE
    }

    /** ORIGCHNG: switch origin between Earth and Moon, rectifying first. */
    public void origchng() {
        rectify();
        double[] r = sub(rcv, rpqv);
        rrect = r;
        rcv = r;
        double[] vm = lunvel(tet);                 // CALL LUNVEL
        if (flag("MOONFLAG")) {
            vm = sc(vm, -1);
        }
        vrect = sub(vcv, vm);
        vcv = vrect.clone();
        if (flag("MOONFLAG")) {
            clrmoon();
        } else {
            setmoon();
        }
    }

    /** RECTIFY: establish a new conic from current deviation. */
    public void rectify() {
        rrect = add(tdeltav, rcv);
        rcv = rrect.clone();
        vrect = add(tnuv, vcv);
        minirect();
    }

    /** RECTIFY13D: store VRECT into VCV; zero TDELTAV, TNUV, TC and XKEP. */
    public void rectify13d() {
        minirect();
    }

    /** Zero the temporary position and velocity deviations (TDELTAV, TNUV). */
    public void tnuv() {
        tdeltav = new double[3];
        tnuv = new double[3];
    }

    /** MINIRECT: store new conic velocity, zero deviation, TC and XKEP. */
    public void minirect() {
        vcv = vrect.clone();
        tdeltav = new double[3];
        tnuv = new double[3];
        tc = 0;
        xkep = 0;
    }

    // ================= Nystrom integration =================

    /** DIFEQ+0: beginning of timestep contribution. */
    private void difeq0Stage() {
        phiv = sc(fv, 1.0 / 8);
        difeqcom();
    }

    /** DIFEQ+1 (DIFEQ+12): middle of timestep contribution. */
    private void difeq1() {
        psiv = add(sc(fv, .5), phiv);
        phiv = add(sc(fv, .25), phiv);
        difeqcom();
    }

    /** DIFEQ+2 (DIFEQ+24): end of timestep; update YV and ZV; handle W matrix columns. */
    private void difeq2() {
        double twoThirdsH = h * DP2_3;
        yv = add(sc(add(zv, sc(phiv, 2 * twoThirdsH)), h), yv);
        zv = add(zv, sc(add(psiv, sc(fv, 1.0 / 8)), 2 * twoThirdsH));
        if (!flag("JSWITCH")) {
            endstate();
            return;
        }
        grp2pc();
        // store W column results (position scaled by 8)
        wv[colreg] = zv.clone();
        w[colreg] = yv.clone();
        if (bad(yv) || bad(zv)) {
            wmatend();
            return;
        }
        grp2pc();
        colreg += 1;                               // INCR,2 6 (next column)
        if (colreg >= (flag("D6OR9FLG") ? 9 : 6)) {
            // TIX,2 RELOADSV: exhausted columns -> reload state vector
            reloadsv();
            return;
        }
        nextcol();
    }

    /** NEXTCOL: set up integration for next W column (position then velocity). */
    private void nextcol() {
        grp2pc();
        yv = w[colreg].clone();
        zv = wv[colreg].clone();
        difeq0();
    }

    /** DIFEQCOM: advance H by DT/2, update DIFEQCNT, form trial position ALPHAV. */
    private void difeqcom() {
        h += dtHalf;
        difeqcnt += 1;
        alphav = add(sc(add(zv, sc(fv, h / 2)), h), yv);
        if (flag("JSWITCH")) {
            dow();
        } else {
            fbr3();
        }
    }

    /** ENDSTATE: finish state vector integration step. */
    private void endstate() {
        if (bad(zv) || bad(yv)) {
            gobaque();
            return;
        }
        tnuv = zv.clone();
        tdeltav = yv.clone();
        if (flag("MIDAVFLG")) {
            new IntegrationInitialization(agc).ckmid2();    // check for MID2 before TIMEINC
            return;
        }
        if (!flag("DIM0FLAG")) {
            new IntegrationInitialization(agc).testloop();
            return;
        }
        // PHASCHNG 04022 (phase 1) ; REINTFLG set (phase change between INTSTALL and INTWAKE)
        agc.set("PHASE", 04022);
        set("REINTFLG");
        // afterwards: start W integration (AMOVED), return to ATOPCSM or ATOPLEM
        colreg = flag("D6OR9FLG") ? 0 : 0;
        set("JSWITCH");
        nextcol();
    }

    /** RELOADSV: restore temporary state vector from permanent one and restart at INTEGRV2. */
    private void reloadsv() {
        agc.set("TDEC1", agc.get("TDEC"));
        new IntegrationInitialization(agc).integrv2();
    }

    /** WMATEND: W overflow; invalidate W (alarm 421) and finish state vector. */
    private void wmatend() {
        clear("DIM0FLAG");
        clear("ORBWFLAG");
        clear("RENDWFLG");
        set("STATEFLG");
        new AlarmAndAbort(agc).alarm(0421);
        new IntegrationInitialization(agc).testloop();
    }

    /** GOBAQUE: overflow; if no deviation abort, otherwise rectify and restart integration. */
    public void gobaque() {
        if (abval(tdeltav) == 0) {
            new AlarmAndAbort(agc).poodoo(0430);   // INT-ABRT
            return;
        }
        tau = tet - h * 2;                          // BDSU TC
        kepprepAfterBackup();
    }

    private void kepprepAfterBackup() {
        kepprep();
        rectify();
        set("RPQFLAG");
        new IntegrationInitialization(agc).testloop();
    }

    // ================= W matrix extrapolation =================

    /**
     * DOW..: second derivative of each W column position vector (gravity gradient), using the
     * VECTAB of vehicle positions saved during state vector integration.
     */
    public void dow() {
        betam = mu();
        fv = dow1(vectab[Math.min(difeqcnt, vectab.length - 1)], betam);
        if (!flag("MIDFLAG")) {
            nbranch();
            return;
        }
        double mub = pbody == 2 ? MU_EARTH : MU_MOON;     // DEC -6 index: other body's mu
        betam = mub;
        double[] second = dow1(vectab[Math.min(difeqcnt + 3, vectab.length - 1)], betam);
        if (!flag("MOONFLAG")) {
            second = sc(second, 1.0 / 64);                 // VSR6
            fv = add(fv, second);
        }
        nbranch();
    }

    /** DOW..1: gravity-gradient acceleration of an offset vector ALPHAV from position pos. */
    private double[] dow1(double[] pos, double mu) {
        double[] d = sc(alphav, 1.0 / 16);               // VSR4
        double[] u = unit(pos);
        double rm = abval(pos);
        // delta_a = (mu/r^3) (3 (d.u) u - d)
        return sc(sub(sc(u, 3 * dot(d, u)), d), mu / (rm * rm * rm));
    }

    // ================= neighbour stubs (names per convention) =================

    /** LSPOS: {Moon, Sun} positions. */
    private double[][] lspos(double t) {
        LunarAndSolarEphemeridesSubroutines.Result r = new LunarAndSolarEphemeridesSubroutines(agc).lspos(t);
        return new double[][] {r.moon, r.sun};
    }
    private double[] lunpos(double t) { return new LunarAndSolarEphemeridesSubroutines(agc).lunpos(t); }
    private double[] lunvel(double t) { return new LunarAndSolarEphemeridesSubroutines(agc).lunvel(t); }
    private void clrmoon() { new IntegrationInitialization(agc).clrmoon(); }
    private void setmoon() { new IntegrationInitialization(agc).setmoon(); }
    private void grp2pc() { new IntegrationInitialization(agc).grp2pc(); }
}
