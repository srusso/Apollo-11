package apollo.luminary099;

import apollo.agc.Agc;

/**
 * P34-35_P74-75.agc (Luminary 099): Transfer Phase Initiation (TPI) programs P34/P74 and
 * Rendezvous Mid-Course programs P35/P75, together with their interpretive support routines
 * S33/34.1, S34/35.1 .. S34/35.5, LOMAT, INTINT/GOINT, VN1645, DISPLAYE, P3XORP7X, VNPOOH,
 * INITVEL, MIDGIM (GET+MGA / GET.LVC), SELECTMU, PERIAPO/PERIAPO1, SETRAD, PRECSET,
 * LEMSTORE/CSMSTORE (ACTIVE/PASSIVE), VECSHIFT, SHIFTR1 and the R36 out-of-plane rendezvous
 * routine.
 *
 * <p>Interpretive code is written with doubles holding the stored fraction of full scale (as
 * {@link VecOps} does); the AGC scale shifts (SL/SR/VSL/VSR) are kept as multiplications by
 * powers of two. The interpretive push-down list is replaced by Java locals. State lives in
 * erasable symbols accessed through agc.get/set (flags: non-zero = set).
 */
public class P3435P7475 {
    // ---- display codes ----
    public static final int V06N37 = 637;
    public static final int V06N55 = 655;
    public static final int V06N58 = 658;
    public static final int V06N59 = 659;
    public static final int V06N81 = 681;
    public static final int V16N45 = 1645;
    public static final int V06N16N = 616;
    public static final int V06N90N = 690;

    // ---- constants ----
    /** TWOPI: 2DEC 6.283185307 B-4. */
    public static final double TWOPI = 6.283185307 / 16.0;
    /** MAX250: 2DEC 25 E3 (seconds*100 = 250 s limit on TPI time step). */
    public static final double MAX250 = 25e3;
    public static final double THIRD = .333333333;
    /** ELEPS: elevation angle tolerance. */
    public static final double ELEPS = .27777777e-3;
    /** DP-.01: -.01 for MGA display (OCT 77777 / 61337). */
    public static final double DP_MINUS_01 = -.01;
    public static final double EPSFOUR = .0416666666;
    /** 130DEG: 2DEC .3611111111 (130 degrees in revolutions). */
    public static final double DEG130 = .3611111111;
    public static final double DPPOSMAX = 0.99999999;
    public static final double DPHALF = 0.5;
    /** HALFREV: 2DEC 1 B-1. */
    public static final double HALFREV = 0.5;
    /** RPAD: standard radius of Pad 37-B, 6373338 m B-29. */
    public static final double RPAD = 6373338.0 / (1L << 29);
    public static final int EARTH = -1;      // X1 primary body indicator
    public static final int MOON = -10;

    private final Agc agc;

    public P3435P7475(Agc agc) {
        this.agc = agc;
    }

    // =====================================================================
    // helpers (erasable access)
    // =====================================================================
    private boolean flag(String name) {
        return agc.get(name) != 0;
    }

    private void flag(String name, boolean on) {
        agc.set(name, on ? 1 : 0);
    }

    private double dp(String sym) {
        return VecOps.getDp(agc, sym);
    }

    private void dp(String sym, double v) {
        VecOps.setDp(agc, sym, v);
    }

    private double[] vec(String sym) {
        return VecOps.getV(agc, sym);
    }

    private void vec(String sym, double[] v) {
        VecOps.setV(agc, sym, v);
    }

    private static double shift(double v, int n) {
        return n >= 0 ? v * (1L << n) : v / (1L << -n);
    }

    private static double[] shift(double[] v, int n) {
        return VecOps.scale(v, n >= 0 ? (double) (1L << n) : 1.0 / (1L << -n));
    }

    /** DSU/DAD style sign function: SIGN of a with magnitude of v. */
    private static double sign(double mag, double signSource) {
        return signSource < 0 ? -Math.abs(mag) : Math.abs(mag);
    }

    /** Display verb/noun via GOFLASH; 0 = terminate, 1 = proceed, 2 = recycle (V32). */
    private int flash(int vn) {
        return new PinballGameButtonsAndLights(agc).goflash(String.valueOf(vn));
    }

    private void gotopooh() {
        new FreshStartAndRestart(agc).goToPooh();
    }

    /** Exception-free "return to caller" marker for GOTOPOOH exits in non-void callers. */
    private boolean terminated;

    // =====================================================================
    // P34 / P74 : TRANSFER PHASE INITIATION
    // =====================================================================

    /** P34: this vehicle is the active vehicle. */
    public void p34() {
        new P32P35P72P75(agc).avflaga();
        p34p74a();
    }

    /** P74: other vehicle is the active vehicle. */
    public void p74() {
        new P32P35P72P75(agc).avflagp();
        p34p74a();
    }

    /**
     * P34/P74A: set update/track flags, ask for TTPI (V06N37), set CENTANG = 130 deg, NN = 0,
     * display ELEV/CENTANG, and decide whether TTPI (ETPIFLAG clear) or ELEV (set) is computed.
     */
    public void p34p74a() {
        while (true) {
            terminated = false;
            new P32P35P72P75(agc).p20flgon();       // SET UPDATFLG, TRACKFLG
            vnPooh(V06N37);                       // TTPI
            if (terminated) {
                return;
            }
            dp("CENTANG", DEG130);                // DCA 130DEG ; DXCH CENTANG
            agc.set("NN", 0);                     // CAF P30ZERO ; TS NN
            displayE();                           // ELEV and CENTANG
            if (terminated) {
                return;
            }
            flag("ETPIFLAG", false);
            dp("TIG", dp("TTPI"));
            if (dp("ELEV") != 0) {
                flag("ETPIFLAG", true);           // BZE skips the SET when ELEV == 0
            }
            selectmu();                           // P34/P74B CALL SELECTMU
            int r = p34p74c();
            if (r == 1) {
                return;                           // finished via GOTO P34/P74C loop exit
            }
            // r == 0: PROCEED after the 611 alarm -> restart at P34/P74A
        }
    }

    /**
     * P34/P74C ... INTLOOP ... P34/P74E: the iteration loop on the TPI time (or elevation).
     * Returns 0 if the caller should restart at P34/P74A, 1 otherwise.
     */
    private int p34p74c() {
        while (true) {
            // P34/P74C: ITSWICH set, NOMTPI = 0 unless ETPIFLAG; SWCHSET stores NOMTPI
            double[] zero = new double[3];
            flag("ITSWICH", true);
            double nomtpi = 0;
            if (flag("ETPIFLAG")) {
                flag("ITSWICH", false);
            }
            dp("NOMTPI", nomtpi);
            // INTLOOP
            boolean again;
            do {
                again = false;
                dp("TDEC1", dp("TTPI") + dp("NOMTPI"));
                precset();
                double mpac = s3334p1();          // BZE EXIT SWCHCLR
                if (mpac != 0) {
                    new AlarmAndAbort(agc).alarm(0611, 0);
                    int r = flash(509);          // V05N09
                    if (r == 0) {
                        gotopooh();
                        return 1;
                    }
                    if (r == 1) {
                        return 0;                 // PROCEED -> P34/P74A
                    }
                    again = true;                 // V32 recycle: TC -7 (re-run INTLOOP)
                    continue;
                }
                // SWCHCLR
                if (flag("ITSWICH")) {            // BONCLR ITSWICH, INTLOOP
                    flag("ITSWICH", false);
                    again = true;
                }
            } while (again);
            if (flag("ETPIFLAG")) {
                // P34/P74D: display TTPI
                vnPooh(V06N37);
            } else {
                displayE();                       // ELEV and CENTANG
            }
            if (terminated) {
                return 1;
            }
            // P34/P74E
            double centang = dp("CENTANG");
            dp("CSTH", Math.cos(centang * 2 * Math.PI));
            dp("SNTH", Math.sin(centang * 2 * Math.PI));
            int x2 = agc.get("RTX2");
            vec("RVEC", shift(vec("RPASS3"), -x2));
            flag("RVSW", true);
            vec("VVEC", shift(vec("VPASS3"), -x2));
            timethet();
            dp("INTIME", dp("TTPI"));             // FOR INITVEL
            dp("TPASS4", dp("TTPI") + dp("T"));   // RENDEZVOUS TIME
            s3435p2();
            dp("DELVTPI", VecOps.abs(vec("DELVEET3")));
            dp("DELVTPF", VecOps.abs(VecOps.sub(vec("VPASS4"), vec("VTPRIME"))));
            // PERIAPO1 on (RACT3, VIPRIME)
            double[] apsides = periapo1(vec("RACT3"), vec("VIPRIME"));
            dp("POSTTPI", shiftr1(apsides[3]));
            dp("TIG", dp("TTPI"));
            vnPooh(V06N58);
            if (terminated) {
                return 1;
            }
            s3435p5();
            vn1645();
            if (terminated) {
                return 1;
            }
            // GOTO P34/P74C
        }
    }

    // =====================================================================
    // P35 / P75 : RENDEZVOUS MID-COURSE
    // =====================================================================
    /** ATIGINC / PTIGINC: DP increments (centiseconds) to the mid-course TIG. */
    public void p35() {
        new P32P35P72P75(agc).avflaga();
        dp("KT", dp("ATIGINC"));
        p35p75a();
    }

    public void p75() {
        new P32P35P72P75(agc).avflagp();
        dp("KT", dp("PTIGINC"));
        p35p75a();
    }

    private void p35p75a() {
        new P32P35P72P75(agc).p20flgon();           // SET UPDATFLG, TRACKFLG
        selectmu();
        terminated = false;
        while (!terminated) {
            // P35/P75B
            double tstrt = loadTime();
            dp("TSTRT", tstrt);
            double tig = tstrt + dp("KT");
            dp("TIG", tig);
            dp("INTIME", tig);                    // FOR INITVEL
            dp("TDEC1", tig);
            precset();                            // ADVANCE BOTH VEHICLES
            s3435p1();                            // NORM AND LOS FOR TRANSFORM
            s3435p2();                            // DELTA V (LV)
            s3435p5();
            vn1645();
        }
    }

    // =====================================================================
    // S33/34.1  -- iterate TPI time (or ELEV) to meet the elevation angle
    // =====================================================================
    /**
     * S33/34.1: advance both vehicles by trial TPI-time increments NOMTPI until the line of
     * sight elevation (computed in ELCALC) equals ELEV (ITSWICH set) or compute ELEV from the
     * given TTPI. Returns 0 on normal exit, non-zero if the iteration count was exceeded.
     */
    public double s3334p1() {
        agc.set("TITER", 040000);
        dp("SECMAX", MAX250);
        vec("RAPREC", vec("RACT3"));
        vec("VAPREC", vec("VACT3"));
        vec("RPPREC", vec("RPASS3"));
        vec("VPPREC", vec("VPASS3"));
        while (true) {
            // ELCALC
            s3435p1();
            double[] ra = vec("RACT3");
            double[] unrm = vec("UNRM");
            double[] ulos = vec("ULOS");
            double[] hor = VecOps.cross(unrm, ra);               // (RA x VA) x RA direction
            double[] ura = VecOps.unit(ra);
            // component of ULOS in the local horizontal plane (VPROJ/BVSU), unitised = UP
            double[] up = VecOps.unit(VecOps.sub(ulos, shift(VecOps.scale(ura, VecOps.dot(ulos, ura)), 0)));
            // angle between ULOS and its horizontal projection, signed by ULOS . RA (above/below)
            double cosEa = Math.max(-1, Math.min(1, VecOps.dot(up, ulos)));
            double ea = Math.acos(cosEa) / (2 * Math.PI);          // revolutions
            if (VecOps.dot(ulos, ra) < 0) {
                ea = -ea;
            }
            double delelo;
            double delel;
            if (!flag("ITSWICH")) {
                // ELEX: ELEV is output
                dp("TTPI", dp("TTPI") + dp("NOMTPI"));
                if (!flag("ETPIFLAG")) {
                    dp("ELEV", ea);
                }
                dp("TIMEX", 0);
                return 0;                                         // DLOAD ZEROVECS ; GOTO NORMEX
            }
            delelo = dp("DELEL");
            dp("DELELO", delelo);
            delel = ea - dp("ELEV");
            dp("DELEL", delel);
            if (Math.abs(delel) - ELEPS < 0) {                     // BMN TIMEX : converged
                return s3334Timex();
            }
            // FIGTIME
            int titer = Agc.toSigned(agc.get("TITER")) >> 1;
            if (titer == 0) {
                return 1;                                          // TOO MANY ITERATIONS
            }
            agc.set("TITER", Agc.fromSigned(titer));
            double[] rp = vec("RPASS3");
            double rpMag = VecOps.abs(rp);
            double raMag = VecOps.abs(ra);
            double diff = rpMag - raMag;                           // 30D: RP - RA magnitudes
            double elev = dp("ELEV");
            if (sign(DPHALF - elev, diff) < 0) {                   // DSU ELEV ; SIGN 30D ; BMN NORMEX
                return 0;
            }
            double cosArg = -Math.cos(Math.PI * 2 * (DPHALF - elev)) * rpMag / raMag; // 28D
            if (DPHALF - Math.abs(cosArg) < 0) {
                return 0;                                          // BMN NORMEX
            }
            double[] hA = VecOps.unit(VecOps.cross(unrm, ra));
            double wa = VecOps.dot(vec("VACT3"), hA) * rpMag;
            double[] hP = VecOps.unit(VecOps.cross(VecOps.cross(rp, vec("VPASS3")), rp));
            double wp = VecOps.dot(vec("VPASS3"), hP) * raMag;
            double wdiff = wa - wp;                                // normalised WA - WP (12D)
            double alpha = Math.acos(Math.max(-1, Math.min(1,
                VecOps.dot(VecOps.unit(ra), VecOps.unit(rp))))) / (2 * Math.PI);
            alpha = sign(alpha, VecOps.dot(VecOps.cross(ra, rp), unrm)) - elev + DPHALF; // ALPHA PI
            double beta = Math.acos(Math.max(-1, Math.min(1, cosArg))) / (2 * Math.PI);
            beta = sign(DPHALF - beta, diff);
            double dt = (alpha + beta) * TWOPI / (wdiff == 0 ? 1e-9 : wdiff);
            // limit by SECMAX
            double secmax = dp("SECMAX");
            if (Math.abs(dt) - secmax >= 0) {
                dt = sign(secmax, dt);
            }
            boolean firstIter = flag("FIRSTITER");
            if (!firstIter) {
                // first pass: TITER = 37777 and use dt as-is (STORDELT)
                agc.set("TITER", 037777);
                flag("FIRSTITER", true);
            } else if (delel * delelo < 0) {
                // crossed over solution: DT = (-SIGN(DTO)//DT//)/2 ; SECMAX *= THIRD
                dp("SECMAX", secmax * THIRD);
                dt = -sign(Math.abs(dt) / 2, dp("DELTEEO"));
            } else if (Math.abs(delel) - Math.abs(delelo) < 0) {   // NEXTES: converging
                dt = sign(Math.abs(dt), dp("DELTEEO"));            // RESIGN
            } else {
                // REVERS: wrong direction
                double back = -dp("DELTEEO");
                dp("DELTEEO", back / 2);
                dt = back + back / 2;
            }
            dp("DELTEEO", dt);                                     // STORDELT
            dp("NOMTPI", dp("NOMTPI") + dt);                       // ADTIME
            goInt(vec("RAPREC"), vec("VAPREC"));
            active();                                              // store new RACT3 VACT3
            goInt(vec("RPPREC"), vec("VPPREC"));
            passive();                                             // store new RPASS3 VPASS3
        }
    }

    private double s3334Timex() {
        dp("TIMEX", 0);
        return 0;
    }

    // =====================================================================
    // S34/35.1, S34/35.2, S34/35.3, S34/35.4, S34/35.5
    // =====================================================================

    /** S34/35.1: unit line of sight (ULOS) and unit normal (UNRM) from active/passive state. */
    public void s3435p1() {
        double[] ulos = VecOps.unit(VecOps.sub(vec("RPASS3"), vec("RACT3")));
        vec("ULOS", ulos);
        vec("UNRM", VecOps.unit(VecOps.cross(vec("RACT3"), vec("VACT3"))));
    }

    /**
     * S34/35.2: advance the passive vehicle to rendezvous time, get required velocity from
     * Lambert (INITVEL), compute ACTCENT, DELLT4 and DELVLVC.
     */
    public void s3435p2() {
        double[] vp3 = vec("VPASS3");
        double[] rp3 = vec("RPASS3");
        double intime = dp("INTIME");
        double tpass4 = dp("TPASS4");
        // CONIC when NN == 0 (push DPPOSMAX/ZEROVECS), otherwise PRECISION with NN offsets
        double precision = Agc.toSigned(agc.get("NN")) == 0 ? 0 : 1;
        double[] rtarg = intInt(rp3, vp3, intime, tpass4, TWOPI, precision);   // S3435.23
        s343525(rtarg);
    }

    /** S3435.25: continuation of S34/35.2 after INTINT - store RTARG/VPASS4, then ACTCENT, DELLT4, DELVLVC. */
    public void s343525(double[] rtarg) {
        double intime = dp("INTIME");
        double tpass4 = dp("TPASS4");
        vec("RTARG", rtarg);
        vec("VPASS4", vec("VATT"));
        // PHI = PI + (ACOS(UNIT RA . UNIT RP) - PI) SIGN(RA*RP . U)
        double[] ura = VecOps.unit(vec("RACT3"));
        double[] urp = VecOps.unit(rtarg);
        double cross = VecOps.dot(VecOps.cross(ura, urp), vec("UNRM"));
        double ang = Math.acos(Math.max(-1, Math.min(1, VecOps.dot(ura, urp)))) / (2 * Math.PI);
        double actcent = cross < 0 ? ang : DPPOSMAX;   // BPL NOPIE ; DAD DPPOSMAX (reasonable two pi)
        dp("ACTCENT", actcent);
        dp("DELLT4", tpass4 - intime);
        // initial inputs for INITVEL
        dp("INITVEL0", agc.get("NN"));        // number of offsets / iterations
        dp("INITVEL2", EPSFOUR);              // angle to 180 deg when rotation starts
        vec("RINIT", vec("RACT3"));
        vec("VINIT", vec("VACT3"));
        initvel();
        lomat();
        double[][] m = lomatrix;
        vec("DELVLVC", shift(VecOps.mxv(m, vec("DELVEET3")), 1));
    }

    /** S34/35.3: new DELVEET3 from astronaut-loaded DELVLVC; new target vector. */
    public void s3435p3() {
        lomat();
        double[] dv = shift(VecOps.vxm(vec("DELVLVC"), lomatrix), 1);
        vec("DELVEET3", dv);                                   // save for transform
        double[] vreq = VecOps.add(vec("VACT3"), dv);          // new V required
        double[] rtarg = intInt(vec("RACT3"), vreq, dp("TIG"), dp("TPASS4"), DPPOSMAX, 0);
        vec("RTARG", rtarg);
        novrwrt();
    }

    /** S34/35.4: no astronaut overwrite; compute DVLOS from DELVEET3. */
    public void s3435p4() {
        novrwrt();
    }

    /** NOVRWRT: DVLOS = DELVEET3 in the (ULOS, UNRM x ULOS ...) line-of-sight frame. */
    private void novrwrt() {
        double[] ulos = vec("ULOS");
        double[] unrm = vec("UNRM");
        double[] y = VecOps.unit(VecOps.scale(VecOps.cross(ulos, unrm), -1.0)); // UNIT(-(ULOS x UNRM))... per VCOMP
        double[] z = shift(VecOps.cross(y, ulos), 1);
        double[][] m = {z, y, ulos};
        vec("DVLOS", shift(VecOps.mxv(m, vec("DELVEET3")), 1));
    }

    /**
     * S34/35.5: allow the astronaut to modify DELVLVC (V06N81), set NTARGFLG on change, then
     * display DVLOS (V06N59).
     */
    public void s3435p5() {
        if (flag("FINALFLG")) {
            flag("NTARGFLG", false);
            double[] dv = vec("DELVLVC");
            vec("GDT/2", dv);
            while (true) {
                int r = flash(V06N81);
                if (r == 0) {
                    gotopooh();
                    terminated = true;
                    return;
                }
                if (r == 1) {
                    break;                                // PRO
                }
                // LOAD (recycle): FLAGON +5 -> redisplay
            }
            agc.set("EBANK", 07);
            // compare the 6 words of DELVLVC against the saved GDT/2 copy
            boolean changed = false;
            double[] now = vec("DELVLVC");
            double[] sav = vec("GDT/2");
            for (int i = 0; i < 3; i++) {
                if (now[i] != sav[i]) {
                    changed = true;
                }
            }
            if (changed) {
                flag("NTARGFLG", true);
            }
            if (flag("NTARGFLG")) {
                s3435p3();
            }
            // NOCHG
            flag("XDELVFLG", false);
            vec("DELVSIN", vec("DELVEET3"));
        } else {
            flag("UPDATFLG", true);
        }
        // FLAGOFF
        s3435p4();
        vnPooh(V06N59);
    }

    // =====================================================================
    // LOMAT, GOINT, INTINT
    // =====================================================================
    private double[][] lomatrix = new double[3][3];

    /** LOMAT: local-vertical transformation matrix (rows Z*-Y, Y, Z) in lomatrix. */
    public void lomat() {
        double[] y = VecOps.scale(vec("UNRM"), -1);
        double[] z = VecOps.scale(VecOps.unit(vec("RACT3")), -1);
        double[] x = shift(VecOps.cross(y, z), 1);       // Z*-Y ... 0D
        lomatrix = new double[][] {x, y, z};
    }

    /** GOINT: INTINT with the time increment NOMTPI and conic mode. */
    private void goInt(double[] r, double[] v) {
        // PDDL ZEROVECS ; PDDL NOMTPI ; PUSH PUSH -> INTINT(r, v, tIn = NOMTPI-relative, ...)
        double[] rr = intInt(r, v, dp("NOMTPI"), 0, 0, 0);
        vec("RATT", rr);
    }

    /**
     * INTINT: integrate (r, v) from time t0 to t1 with Encke (INTEGRVS). The conic / precision
     * selection comes from {@code precision} (INTYPFLG set when non-zero); MOONFLAG is set
     * unless CMOONFLG is on. Result r is returned (also left in RATT/VATT).
     */
    public double[] intInt(double[] r, double[] v, double t0, double t1, double dtOrTwopi,
            double precision) {
        new IntegrationInitialization(agc).intstall();
        flag("INTYPFLG", false);
        if (precision != 0) {
            flag("INTYPFLG", true);
        }
        dp("TDEC1", t1);
        flag("MOONFLAG", true);
        int x2 = agc.get("RTX2");
        if (flag("CMOONFLG")) {
            flag("MOONFLAG", false);
        }
        dp("TET", t0);
        vec("RCV", shift(r, -x2));
        vec("VCV", shift(v, -x2));
        new IntegrationInitialization(agc).integrvs();
        return vec("RATT");
    }

    // =====================================================================
    // VN1645, DISPLAYE, P3XORP7X, VNPOOH
    // =====================================================================

    /** VN1645: set MGA, update TTOGO task and flash V16N45 (TRKMKCNT, TTOGO, +MGA). */
    public void vn1645() {
        dp("+MGA", DP_MINUS_01);
        boolean getMga = false;
        if (flag("FINALFLG")) {
            dp("+MGA", DP_MINUS_01 + DP_MINUS_01);       // MGA = -.02
            if (flag("REFSMFLG")) {
                // P3XORP7X : P3X computes MGA, P7X skips
                if (p3xorp7x()) {
                    getMga = true;
                }
            }
        } else {
            getMga = false;
        }
        if (getMga) {
            getMga(vec("DELVSIN"));
        }
        // GET45
        new P32P35P72P75(agc).comptgo();                    // initiate task to update TTOGO
        agc.set("QSAVED", agc.get("SUBEXIT"));
        new ServiceRoutines(agc).delayJob(100, 0);                 // CAF 1SEC ; DELAYJOB
        int r = flash(V16N45);
        if (r == 1) {                                     // N45PROC
            if (flag("FINALFLG")) {
                killClock();                              // FINALFLG set -> FLASH V37
                return;
            }
            new PhaseTableMaintenance(agc).phasChng(04024);
            flag("FINALFLG", true);
        } else if (r == 0) {
            killClock();
            return;
        }
        // CLUPDATE (recycle falls through here too)
        agc.set("DISPDEX", 0);
        new PhaseTableMaintenance(agc).phasChng(04024);
        flag("UPDATFLG", false);
        // GOTO QSAVED
    }

    private void killClock() {
        agc.set("DISPDEX", 0);
        gotopooh();
        terminated = true;
    }

    /** DISPLAYE: display V06N55 (ELEV and CENTANG) until proceed. */
    public void displayE() {
        while (true) {
            int r = flash(V06N55);
            if (r == 0) {
                gotopooh();
                terminated = true;
                return;
            }
            if (r == 1) {
                return;
            }
        }
    }

    /** P3XORP7X: true when running P3X (MODREG high bits zero), false for P7X. */
    public boolean p3xorp7x() {
        return (agc.get("MODREG") & 0x7E00) == 0;         // HIGH9 mask
    }

    /** VNPOOH: display the verb/noun, terminate on GOTOPOOH, recycle on V32. */
    public void vnPooh(int vn) {
        agc.set("VERBNOUN", vn);
        while (true) {
            int r = flash(vn);
            if (r == 0) {
                gotopooh();
                terminated = true;
                return;
            }
            if (r == 1) {
                return;
            }
        }
    }

    // =====================================================================
    // INITVEL
    // =====================================================================

    /**
     * INITVEL: required initial velocity VIPRIME for a trajectory of transfer time DELLT4 from
     * RINIT/VINIT to RTARG (Lambert; with NN offsets, repeated with Encke integration to
     * compute an offset target). Outputs RTARG, VIPRIME, VTPRIME, DELVEET3.
     */
    public void initvel() {
        flag("GUESSW", true);                             // COGA guess not available
        double[] rtarg1 = vec("RTARG");
        int x2 = agc.get("RTX2");
        double[] rinit = vec("RINIT");
        double[] vinit = vec("VINIT");
        if (x2 != 0) {                                    // lunar: rescale
            rinit = shift(rinit, 2);
            vinit = shift(vinit, 2);
            rtarg1 = shift(rtarg1, 2);
        }
        int itctr = -1;
        double e4 = Math.cos(EPSFOUR * 2 * Math.PI);
        double cozy4 = e4 / 2;
        int vtargtag = Agc.toSigned(agc.get("INITVEL0")); // NN: number of iterations
        double[] r1 = rinit;
        double[] r2 = rtarg1;
        double tdesired = dp("DELLT4");
        double[] un = VecOps.unit(VecOps.cross(VecOps.unit(rinit), vinit));
        flag("NORMSW", false);
        cozy4 = cozy4 + VecOps.dot(VecOps.unit(rtarg1), VecOps.unit(rinit)) * 0; // (TEMP*RT.URI + COZY4)
        double tmp = VecOps.dot(VecOps.unit(rtarg1), VecOps.unit(rinit));
        cozy4 = tmp + cozy4;
        double[] vtarget = new double[3];
        double[] viprime;
        while (true) {
            // INITVEL2: inside cone around 180 deg transfer -> rotate RC into YC plane
            if (cozy4 < 0) {
                // BPL INITVEL3 not taken: set NORMSW, rotate R2VEC into plane of (R1, V)
                flag("NORMSW", true);
                double mag = VecOps.abs(r2);
                double[] proj = shift(VecOps.scale(un, VecOps.dot(r2, un)), 0);
                r2 = VecOps.scale(VecOps.unit(VecOps.sub(r2, proj)), mag);
                if (itctr >= 0) {
                    rtarg1 = r2;
                }
            }
            // INITVEL3: set up Lambert: sign of transfer from (R2 x R1) . UN
            double[] u2 = VecOps.unit(r2);
            double[] u1 = VecOps.unit(r1);
            double[] mn = VecOps.scale(VecOps.cross(u2, u1), -1);   // -N
            int rtx1 = agc.get("RTX1");
            double mu = muOf(rtx1);
            int geomsgn = VecOps.dot(mn, un) < 0 ? -1 : 1;
            agc.set("GEOMSGN", Agc.fromSigned(geomsgn));
            // CALL SETITCTR / LAMBERT (COASTFLT): VVEC = initial velocity required
            dp("MUEARTH", mu);
            vec("R1VEC", r1);
            vec("R2VEC", r2);
            dp("TDESIRED", tdesired);
            new ConicSubroutines(agc).lambert();
            flag("GUESSW", false);
            viprime = vec("VVEC");
            vec("VIPRIME", viprime);
            if (vtargtag == 0) {
                break;                                    // INITVEL7
            }
            // set up inputs for ENCKE integration
            new IntegrationInitialization(agc).intstall();
            flag("MOONFLAG", x2 != 0 ? true : false);
            vec("R1VEC", rinit);
            vec("RCV", rinit);
            vec("VCV", viprime);
            dp("TET", dp("INTIME"));
            flag("INTYPFLG", false);
            dp("TDEC1", dp("INTIME") + dp("DELLT4"));
            new IntegrationInitialization(agc).integrvs();
            vtarget = vec("VATT1");
            itctr++;
            vtargtag -= 1;
            if (vtargtag == 0) {
                break;                                    // INITVELC / INITVEL6
            }
            // offset conic target vector
            r2 = VecOps.add(VecOps.sub(rtarg1, vec("RATT1")), r2);
        }
        if (vtargtag == 0 && itctr >= 0) {
            rtarg1 = r2;                                  // INITVEL6
        }
        // INITVEL7: compute the delta velocity
        vec("DELVEET3", VecOps.sub(viprime, vinit));
        vec("VTPRIME", itctr >= 0 ? vtarget : vec("VTPRIME"));
        if (x2 != 0) {
            vec("VTPRIME", shift(vec("VTPRIME"), -2));
            vec("VIPRIME", shift(viprime, -2));
            rtarg1 = shift(rtarg1, -2);
            vec("DELVEET3", shift(vec("DELVEET3"), -2));
        }
        // INITVELX: MU/A and MUASTEER
        double muOverA = muOf(agc.get("RTX1")) * dp("R1A") / 2 / Math.max(dp("R1"), 1e-30);
        dp("MU/A", muOverA);
        dp("MUASTEER", shift(muOverA, 6));
        vec("RTARG", rtarg1);
    }

    /** MUTABLE lookup (-2 earth, -10 lunar): gravitational constant. */
    private double muOf(int rtx1) {
        return dp(rtx1 == MOON ? "MUMOON" : "MUEARTH");
    }

    // =====================================================================
    // MIDGIM  (GET+MGA, GET.LVC)
    // =====================================================================

    /** GET+MGA: positive middle gimbal angle for delta-V vector {@code dv} (revolutions). */
    public void getMga(double[] dv) {
        double[] uv = VecOps.unit(dv);
        double[][] smat = VecOps.getM(agc, "REFSMMAT");
        double s = shift(VecOps.dot(uv, smat[1]), 1);      // dot with Y(stable member), +2 -> +1
        s = Math.max(-1, Math.min(1, s));
        double mga = Math.asin(s) / (2 * Math.PI);
        if (mga < 0) {
            mga += HALFREV + HALFREV;                      // -MGA -> +MGA by adding one rev
        }
        dp("+MGA", mga);
        flag("MGLVFLAG", false);
    }

    /** GET.LVC: convert delta-V (inertial) to local vertical coordinates (DELVLVC). */
    public void getLvc(double[] dv) {
        double[] negUr = VecOps.scale(VecOps.unit(vec("RINIT")), -1);        // U(-R) 18D
        double[] uvr = VecOps.unit(VecOps.cross(negUr, vec("VINIT")));       // U(V*R) 12D
        double[] uz = VecOps.unit(VecOps.cross(uvr, negUr));                 // 6D
        double[][] m = {uz, uvr, negUr};
        vec("DELVLVC", shift(VecOps.mxv(m, dv), 1));
        flag("MGLVFLAG", true);
    }

    // =====================================================================
    // SELECTMU, PERIAPO, SETRAD, PRECSET, ACTIVE/PASSIVE, VECSHIFT, SHIFTR1
    // =====================================================================

    /** SELECTMU: pick RTX1/RTX2 and RTMU / RTSR1/MU for earth or moon; then VN1645. */
    public void selectmu() {
        int x1 = 2;     // AXC,1 2D  (earth)
        int x2 = 0;     // AXT,2 0D
        if (flag("CMOONFLG")) {
            x1 = 2;
            x2 = 0;
        } else {
            x1 = 10;
            x2 = 2;
        }
        agc.set("RTX1", x1);
        dp("RTSR1/MU", dp("MUTABLE-2," + x1));
        double rtmu = dp("MUTABLE+4," + x1);
        if (!flag("CMOONFLG")) {
            rtmu = shift(rtmu, 6);
        }
        dp("RTMU", rtmu);
        agc.set("RTX2", x2);
        flag("FINALFLG", false);
        vn1645();
    }

    /**
     * PERIAPO1: shift RVEC/VVEC by the body scale and compute apsides.
     * Returns {apoRadius, apoAltitude, periRadius, periAltitude} (2D,4D,6D,8D).
     */
    public double[] periapo1(double r, double v) {
        return periapo1(new double[] {r, 0, 0}, new double[] {v, 0, 0});
    }

    public double[] periapo1(double[] r, double[] v) {
        int x2 = agc.get("RTX2");
        vec("VVEC", shift(v, -x2));
        vec("RVEC", shift(r, -x2));
        return periapo();
    }

    /** PERIAPO: two-body apocenter and pericenter altitudes (APSIDES) for RVEC, VVEC. */
    public double[] periapo() {
        double xxxalt = setrad();
        dp("XXXALT", xxxalt);
        ConicSubroutines cs = new ConicSubroutines(agc);
        cs.rvec = vec("RVEC");
        cs.vvec = vec("VVEC");
        cs.apsides();                                    // apo radius, peri radius (+ECC)
        double apoR = cs.apoapse;
        double periR = cs.periapse;
        return new double[] {apoR, apoR - xxxalt, periR, periR - xxxalt};
    }

    /** SETRAD: radius of the primary body (pad radius for earth, RLS magnitude for moon). */
    public double setrad() {
        double rad = RPAD;
        if (agc.get("RTX2") != 0) {
            rad = VecOps.abs(vec("RLS"));
        }
        return rad;
    }

    /** PRECSET: advance both LM (LEMPREC) and CSM (CSMPREC) to TDEC1; store ACTIVE/PASSIVE. */
    public void precset() {
        double t = dp("TDEC1");
        dp("TDEC2", t);
        new IntegrationInitialization(agc).lemprec();
        lemstore();
        dp("TDEC1", dp("TDEC2"));
        new IntegrationInitialization(agc).csmprec();
        csmstore();
    }

    /** LEMSTORE: RATT/VATT is the LM state: ACTIVE if AVFLAG clear... else PASSIVE. */
    public void lemstore() {
        if (flag("AVFLAG")) {
            passive();
        } else {
            active();
        }
    }

    /** CSMSTORE: RATT/VATT is the CSM state: PASSIVE if AVFLAG clear... else ACTIVE. */
    public void csmstore() {
        if (flag("AVFLAG")) {
            active();
        } else {
            passive();
        }
    }

    /** ACTIVE: RACT3 = RATT, VACT3 = VATT. */
    public void active() {
        vec("RACT3", vec("RATT"));
        vec("VACT3", vec("VATT"));
    }

    /** PASSIVE: RPASS3 = RATT, VPASS3 = VATT. */
    public void passive() {
        vec("RPASS3", vec("RATT"));
        vec("VPASS3", vec("VATT"));
    }

    /** VECSHIFT: scale VVEC and RVEC by the body scale factors (RTX2/RTX1). */
    public void vecshift(double[] v, double[] r) {
        vec("VVEC", shift(v, -agc.get("RTX2")));
        vec("RVEC", shift(r, -agc.get("RTX1")));
    }

    /** SHIFTR1: scale a DP scalar by 2^RTX2 ("SL* 0,2"). */
    public double shiftr1(double v) {
        return shift(v, agc.get("RTX2"));
    }

    /** TIMETHET: time from Kepler propagation by CENTANG (external). */
    private void timethet() {
        new ConicSubroutines(agc).timethet();
    }

    private double loadTime() {
        new RtbOpCodes(agc).loadtime();       // RTB LOADTIME: time returned in MPAC (DP)
        return dp("MPAC");
    }

    // =====================================================================
    // R36 : OUT-OF-PLANE RENDEZVOUS (V90)
    // =====================================================================

    /**
     * R36: display (V06N90) Y, YDOT and PSI -- out-of-plane distance, rate and angle of the
     * passive vehicle relative to the active vehicle's orbital plane. First asks for time
     * (V06N16N); zero means present time.
     */
    public void r36() {
        dp("DSPTEMX", 0);                                 // time of event = 0 for first display
        while (true) {
            int r = new PinballGameButtonsAndLights(agc).gomarkf(String.valueOf(V06N16N));
            if (r == 0) {
                new ExtendedVerbs(agc).endext();        // TERMINATE
                return;
            }
            if (r == 2) {
                continue;                                 // RECYCLE for astronaut input time
            }
            break;                                        // PROCEED
            }
        long t = VecOps.getDpRaw(agc, "DSPTEMX");
        double time;
        if (t == 0) {
            time = loadTime();                            // ENTTIM2: present time
        } else {
            time = VecOps.getDp(agc, "DSPTEMX");          // ASTROTIM: astronaut input time
        }
        r36int(time);
    }

    /** R36INT: compute RANGE (Y), RRATE (YDOT) and RTHETA (PSI) at time {@code time}. */
    private void r36int(double time) {
        while (true) {
            dp("TDEC1", time);
            new IntegrationInitialization(agc).csmprec()      /* OTHPREC = CSMPREC */;
            double[] vp = vec("VATT");
            double[] rp = vec("RATT");
            vec("RPASS36", rp);
            double[] unp = VecOps.unit(VecOps.cross(rp, vp));
            vec("UNP36", unp);                            // U (normal of passive plane)
            dp("TDEC1", dp("TAT"));
            new IntegrationInitialization(agc).lemprec()      /* THISPREC = LEMPREC */;
            double[] va = vec("VATT");
            double[] ra = vec("RATT");
            double tat = dp("TAT");                       // saved in 30D for redisplay
            double[] los = VecOps.sub(rp, ra);            // R_P - R_A (line of sight)
            dp("RANGE", shift(VecOps.dot(los, unp), 1));  // Y = U . (R_P - R_A)
            dp("RRATE", shift(VecOps.dot(va, unp), 1));   // Ydot = U . V_A
            double[] ura = VecOps.unit(ra);
            double[] h = VecOps.unit(VecOps.unit(shift(
                VecOps.cross(VecOps.cross(ura, va), ura), 2)));   // unit horizontal, forward
            double[] lh = VecOps.unit(VecOps.unit(shift(
                VecOps.sub(los, VecOps.scale(ura, VecOps.dot(los, ura))), 2)));  // LOS in horizontal
            double psi = Math.acos(Math.max(-1, Math.min(1, shift(VecOps.dot(lh, h), 1)))) / (2 * Math.PI);
            if (VecOps.dot(VecOps.cross(lh, h), ura) < 0) {      // R36TAG2 test of sign
                psi = DPPOSMAX - psi;
            }
            dp("RTHETA", psi);
            new RtbOpCodes(agc).sgnagree();   // RTB SGNAGREE on TAT for display
            dp("DSPTEMX", tat);
            int r = new PinballGameButtonsAndLights(agc).gomarkf(String.valueOf(V06N90N));
            if (r == 2) {
                time = tat;                               // REDISPLAY OUTPUT (R36 +3)
                continue;
            }
            new ExtendedVerbs(agc).endext();            // TERMINATE or PROCEED: end of program
            return;
        }
    }
}
