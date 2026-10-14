package apollo.comanche055;

import apollo.agc.Agc;

import java.util.HashMap;
import java.util.Map;

/**
 * P34-35_P74-75 (Comanche055, pp 460-488): Transfer Phase Initiation (TPI, P34 active / P74 passive)
 * and Rendezvous Mid-course (TPM, P35 / P75) programs, with the supporting subroutines
 * S33/34.1 (elevation-angle iteration), S34/35.1, S34/35.2, S34/35.3, S34/35.4, LOMAT, INTINT/GOINT,
 * S34/35.5, VN1645, DISPLAYE, P3XORP7X, VNPOOH, INITVEL, MIDGIM, SELECTMU, PERIAPO, SETRAD, PRECSET,
 * LEMSTORE/CSMSTORE, VECSHIFT/SHIFTR1 and the R36 range/range-rate/angle display routine.
 *
 * The AGC interpretive code works in scaled fixed-point; this translation uses SI doubles (positions in
 * metres, velocities in m/cs, times in cs, angles in revolutions) held in this object (vectors and
 * scalars keyed by their original erasable names). Scale notes are retained in comments. Flags use Agc.
 */
public final class P3435P7475 {
    // ---- constants (CSI/CDH bank) ----
    public static final double TWOPI = 6.283185307;           // 2DEC 6.283185307 B-4
    public static final double MAX250 = 25e3;                 // 2DEC 25 E3 B-28 (seconds * 100 ... max time step)
    public static final double THIRD = .333333333;
    public static final double ELEPS = .27777777e-3;
    public static final int DECTWO = 2;
    public static final double DP_M01 = -.01;                 // OCT 77777, 61337: -.01 for MGA display
    public static final double EPSFOUR = .0416666666;
    public static final double HALFREV = 0.5;                 // 2DEC 1 B-1
    public static final double RPAD = 6373338.0;              // standard radius of pad 37-B (m)
    public static final int V06N37 = 637;
    public static final int V06N55 = 655;
    public static final int V06N58 = 658;
    public static final int V06N59 = 659;
    public static final int V06N81 = 681;
    public static final int V16N45 = 1645;
    public static final int OPTION36 = 412;                   // V04N12
    public static final int V06N16N = 616;
    public static final int V06N90N = 690;
    public static final int V05N09 = 509;

    private final Agc agc;

    /** Vector and scalar working storage (original erasables). */
    private final Map<String, double[]> vec = new HashMap<>();
    private final Map<String, Double> sc = new HashMap<>();

    public P3435P7475(Agc agc) {
        this.agc = agc;
    }

    // ---- storage helpers ----
    private double[] v(String n) {
        return vec.computeIfAbsent(n, k -> new double[3]);
    }

    private void v(String n, double[] value) {
        vec.put(n, value.clone());
    }

    private double d(String n) {
        return sc.getOrDefault(n, 0.0);
    }

    private void d(String n, double value) {
        sc.put(n, value);
    }

    private boolean f(String n) {
        return agc.get(n) != 0;
    }

    private void flag(String n, boolean on) {
        agc.set(n, on ? 1 : 0);
    }

    private static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    private static double abs(double[] a) {
        return Math.sqrt(dot(a, a));
    }

    private static double[] unit(double[] a) {
        double n = abs(a);
        return n == 0 ? new double[3] : new double[] {a[0] / n, a[1] / n, a[2] / n};
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }

    private static double[] sub(double[] a, double[] b) {
        return new double[] {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    private static double[] add(double[] a, double[] b) {
        return new double[] {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
    }

    private static double[] mul(double[] a, double k) {
        return new double[] {a[0] * k, a[1] * k, a[2] * k};
    }

    private static double[] neg(double[] a) {
        return mul(a, -1);
    }

    /** Vector times matrix with rows r0,r1,r2 (MXV): result[i] = row_i . v. */
    private static double[] mxv(double[][] rows, double[] x) {
        return new double[] {dot(rows[0], x), dot(rows[1], x), dot(rows[2], x)};
    }

    /** Vector times matrix (VXM): result = x * M = sum x_i row_i. */
    private static double[] vxm(double[] x, double[][] rows) {
        return add(add(mul(rows[0], x[0]), mul(rows[1], x[1])), mul(rows[2], x[2]));
    }

    private double[][] lvMatrix; // rows from LOMAT

    // ====================================================================================
    // P34 / P74: TRANSFER PHASE INITIATION
    // ====================================================================================

    /** P34: this vehicle active. */
    public void p34() {
        // TC AVFLAGA (set AVFLAG) ; fall to P34/P74A
        agc.set("AVFLAG", 1);
        p34p74a();
    }

    /** P74: this vehicle passive. */
    public void p74() {
        agc.set("AVFLAG", 0);                     // AVFLAGP
        p34p74a();
    }

    /** P34/P74A: set UPDATFLG, TRACKFLG; display TTPI (V06N37), then ELEV and CENTANG. */
    public void p34p74a() {
        flag("UPDATFLG", true);
        flag("TRACKFLG", true);                   // P20FLGON
        vnpooh(V06N37);                           // TTPI
        agc.set("NN", 0);
        displaye();                               // ELEV and CENTANG
        flag("ETPIFLAG", false);
        d("TIG", d("TTPI"));
        if (d("ELEV") == 0) {                     // BZE P34/P74B: TTPI is input; else ELEV is input
            flag("ETPIFLAG", true);
        }
        p34p74b();
    }

    public void p34p74b() {
        selectmu();
        p34p74c();
    }

    /** P34/P74C: iterate S33/34.1 for TTPI or ELEV. */
    public void p34p74c() {
        d("DELELO", 0);                           // DELELO = 26D
        flag("ITSWICH", true);
        if (f("ETPIFLAG")) {
            swchset();
            return;
        }
        flag("ITSWICH", false);
        swchset();
    }

    /** SWCHSET: NOMTPI = 0 (ZEROVECS), then INTLOOP. */
    public void swchset() {
        d("NOMTPI", 0);
        intloop();
    }

    /** INTLOOP: TDEC1 = TTPI + NOMTPI; PRECSET; call S33/34.1; non-zero result -> alarm 611. */
    public void intloop() {
        d("TDEC1", d("TTPI") + d("NOMTPI"));
        precset();
        double result = s3334_1();
        if (result == 0) {
            swchclr();
            return;
        }
        // TC ALARM OCT 611; V05N09 flash: TERMINATE -> GOTOPOOH, PROCEED -> P34/P74A, V32 -> retry flash
        agc.set("ALARM", 0611);
        // BANKCALL GOFLASH V05N09 ...
    }

    /** SWCHCLR: iterate again if ITSWICH was set; else display TTPI (ETPIFLAG) or ELEV and CENTANG. */
    public void swchclr() {
        if (f("ITSWICH")) {
            flag("ITSWICH", false);
            intloop();
            return;
        }
        if (f("ETPIFLAG")) {
            p34p74d();
            return;
        }
        displaye();                               // display ELEV and CENTANG
        p34p74e();
    }

    /** P34/P74D: display TTPI. */
    public void p34p74d() {
        vnpooh(V06N37);
        p34p74e();
    }

    /** P34/P74E: transfer geometry and compute delta V; display; iterate. */
    public void p34p74e() {
        agc.set("X1", agc.get("RTX1"));
        double cent = d("CENTANG");
        d("CSTH", Math.cos(cent * 2 * Math.PI));
        d("SNTH", Math.sin(cent * 2 * Math.PI));
        v("RVEC", v("RPASS3"));
        v("VVEC", v("VPASS3"));
        flag("RVSW", true);
        timethet();
        d("INTIME", d("TTPI"));
        d("TPASS4", d("TTPI") + d("T"));          // rendezvous time, for INITVEL
        s3435_2();
        d("DELVTPI", abs(v("DELVEET3")));
        d("DELVTPF", abs(sub(v("VPASS4"), v("VTPRIME"))));
        v("PERI_IN_R", v("RACT3"));
        v("PERI_IN_V", v("VIPRIME"));
        periapo1();
        d("POSTTPI", shiftr1(d("POSTTPI")));
        d("TIG", d("TTPI"));
        vnpooh(V06N58);
        s3435_5();
        vn1645();
        p34p74c();
    }

    /** TIMETHET: (external) time from transfer angle CENTANG; sets T. */
    private void timethet() {
        // CALL TIMETHET (in conic subroutines): T from RVEC, VVEC, CSTH, SNTH
        d("T", d("T"));
    }

    // ====================================================================================
    // P35 / P75: MIDCOURSE
    // ====================================================================================

    /** P35: active; KT = ATIGINC. */
    public void p35() {
        agc.set("AVFLAG", 1);
        d("KT", d("ATIGINC"));
        p35p75a();
    }

    /** P75: passive; KT = PTIGINC. */
    public void p75() {
        agc.set("AVFLAG", 0);
        d("KT", d("PTIGINC"));
        p35p75a();
    }

    public void p35p75a() {
        flag("UPDATFLG", true);
        flag("TRACKFLG", true);
        selectmu();
        p35p75b();
    }

    /** P35/P75B: TSTRT = now; TIG = INTIME = TDEC1 = TSTRT + KT; PRECSET; S34/35.1 .. .5; VN1645; loop. */
    public void p35p75b() {
        while (true) {
            d("TSTRT", agc.get("LOADTIME"));
            d("TIG", d("TSTRT") + d("KT"));
            d("INTIME", d("TIG"));
            d("TDEC1", d("TIG"));
            precset();                            // advance both vehicles
            s3435_1();                            // get norm and LOS for transform
            s3435_2();                            // get delta V (LV)
            s3435_5();
            vn1645();
        }
    }

    // ====================================================================================
    // S33/34.1: iterate TTPI (or ELEV) until the LOS elevation angle at TPI equals ELEV
    // ====================================================================================

    /** @return 0 if converged; non-zero for NORMEX with failure (too many iterations or bad geometry). */
    public double s3334_1() {
        d("TITER", 0x4000 / 16384.0 * 0 + 0);     // TITER = OCT 40000 (first-iteration marker)
        agc.set("TITER", 040000);
        d("SECMAX", MAX250);
        v("RAPREC", v("RACT3"));
        v("VAPREC", v("VACT3"));
        v("RPPREC", v("RPASS3"));
        v("VPPREC", v("VPASS3"));
        return elcalc();
    }

    /** ELCALC: compute actual elevation EA; DELEL = EA - ELEV; step NOMTPI. */
    private double elcalc() {
        while (true) {
            s3435_1();
            double[] ra = v("RACT3");
            double[] va = v("VACT3");
            double[] ulos = v("ULOS");
            double[] unrm = v("UNRM");
            double[] xchnj = cross(cross(ra, va), ra);        // (RA*VA)*RA
            double[] up = unit(sub(ulos, mul(unit(xchnj), dot(ulos, unit(xchnj)) * 4 / 4))); // projection of ULOS on local horizontal
            double sign = Math.signum(dot(up, unrm) * dot(ulos, unrm) + 0);
            double ea = Math.acos(clamp(dot(up, ulos))) / (2 * Math.PI);
            if (dot(ulos, ra) < 0) {
                ea = 1.0 - ea;                                  // DPPOSMAX - EA
            }
            ea *= Math.signum(dot(ulos, unit(ra))) == 0 ? 1 : 1;
            double eaSigned = dot(ulos, unit(ra)) >= 0 ? ea : -ea;
            if (!f("ITSWICH")) {
                return elex();
            }
            d("DELELO", d("DELEL"));
            double delel = eaSigned - d("ELEV");
            d("DELEL", delel);
            if (Math.abs(delel) - ELEPS < 0) {
                return timex();                                 // converged
            }
            int titer = agc.get("TITER");
            if (titer == 0) {
                return normex();                                // too many iterations
            }
            agc.set("TITER", titer >> 1);                       // SR1 on TITER
            double dt = deltaTime(eaSigned);
            if (agc.get("TITER") == 037777 >> 0 && false) {
                return normex();
            }
            // OKMAX: first iteration test
            if (titer == 040000) {
                agc.set("TITER", 037777);
                d("DELTEEO", dt);
            } else if (d("DELEL") * d("DELELO") > 0) {
                // NEXTES: same side; check progress direction
                if (Math.abs(d("DELEL")) - Math.abs(d("DELELO")) < 0) {
                    d("DELTEEO", Math.copySign(Math.abs(dt), d("DELTEEO")));  // RESIGN
                } else {
                    d("DELTEEO", -d("DELTEEO") / 2);            // REVERS
                    dt = d("DELTEEO");
                }
            } else {
                d("SECMAX", d("SECMAX") * THIRD);               // crossed over solution
                d("DELTEEO", -Math.copySign(Math.abs(d("DELTEEO")) / 2, d("DELTEEO")));
            }
            dt = d("DELTEEO");
            d("NOMTPI", d("NOMTPI") + dt);                      // ADTIME: sum of delta T's
            goint(v("RAPREC"), v("VAPREC"));
            active();                                           // store new RACT3 VACT3
            goint(v("RPPREC"), v("VPPREC"));
            passive();                                          // store new RPASS3 VPASS3
        }
    }

    /** Time step from angle error (D*2PI/..., limited to SECMAX with sign). */
    private double deltaTime(double ea) {
        double[] ra = v("RACT3");
        double[] rp = v("RPASS3");
        double rA = abs(ra);
        double rP = abs(rp);
        double diff = rP - rA;
        double step = (ea - d("ELEV")) * TWOPI * (diff == 0 ? 1 : Math.abs(diff) / 1e6);
        if (Math.abs(step) > d("SECMAX")) {
            step = Math.copySign(d("SECMAX"), step);            // replace time with max time signed
        }
        return step;
    }

    /** ELEX: ITSWICH off -> TTPI += NOMTPI; if ETPIFLAG not set ELEV = result. */
    private double elex() {
        d("TTPI", d("TTPI") + d("NOMTPI"));
        if (!f("ETPIFLAG")) {
            d("ELEV", d("DELEL"));
        }
        return timex();
    }

    private double timex() {
        return 0;                                               // DLOAD ZEROVECS; GOTO NORMEX
    }

    private double normex() {
        return 1;
    }

    private static double clamp(double x) {
        return Math.max(-1, Math.min(1, x));
    }

    // ====================================================================================
    // S34/35.x
    // ====================================================================================

    /** S34/35.1: unit normal and line of sight vectors given positions at time T3. */
    public void s3435_1() {
        double[] ulos = unit(sub(v("RPASS3"), v("RACT3")));
        v("ULOS", ulos);
        v("UNRM", unit(cross(v("RACT3"), v("VACT3"))));
    }

    /** S34/35.2: advance passive vehicle to rendezvous time; get required velocity from Lambert (INITVEL). */
    public void s3435_2() {
        // INTINT from (RPASS3, VPASS3) at INTIME to TPASS4, conic if NN = 0 (hi), else precision with offsets
        int nn = agc.get("NN");
        double tdesired = d("TPASS4");
        intint(v("RPASS3"), v("VPASS3"), d("INTIME"), tdesired, nn != 0);
        v("RTARG", v("RATT"));
        v("VPASS4", v("VATT"));
        double[] ura = unit(v("RACT3"));
        double[] urp = unit(v("RTARG"));
        double phi = Math.acos(clamp(dot(ura, urp)));
        double sgn = dot(cross(ura, urp), v("UNRM"));
        if (sgn < 0) {
            phi = 2 * Math.PI - phi;                            // PHI = PI + (ACOS - PI) SIGN(RA*RP.U)
        }
        d("ACTCENT", phi / (2 * Math.PI));
        d("DELLT4", d("TPASS4") - d("INTIME"));
        v("RINIT", v("RACT3"));
        v("VINIT", v("VACT3"));
        initvel();
        lomat();
        double[] dv = mxv(lvMatrix, v("DELVEET3"));
        v("DELVLVC", mul(dv, 2));
    }

    /** S34/35.3: new DELVLVC input by astronaut -> new DELVEET3, new target vector and DVLOS. */
    public void s3435_3() {
        lomat();
        double[] dvEet = mul(vxm(v("DELVLVC"), lvMatrix), 2);
        v("DELVEET3", dvEet);
        double[] vReq = add(v("VACT3"), dvEet);
        intint(v("RACT3"), vReq, d("TIG"), d("TPASS4"), true);
        v("RTARG", v("RATT"));
        novrwrt();
    }

    /** NOVRWRT: DVLOS = DELVEET3 transformed into the line-of-sight frame. */
    private void novrwrt() {
        double[] ulos = v("ULOS");
        double[] unrm = v("UNRM");
        double[] a = unit(cross(neg(cross(ulos, unrm)), ulos));
        double[] b = unit(cross(ulos, unrm));
        double[][] m = {ulos, b, unit(cross(ulos, b))};
        v("DVLOS", mul(mxv(m, v("DELVEET3")), 2));
    }

    /** S34/35.4: no astronaut overwrite. */
    public void s3435_4() {
        novrwrt();
    }

    /** LOMAT: local vertical matrix (rows X = Z*-Y, Y = -UNRM, Z = -UNIT(RACT3)). */
    public void lomat() {
        double[] y = neg(v("UNRM"));
        double[] z = neg(unit(v("RACT3")));
        double[] x = mul(cross(z, y), 2 / 2);
        lvMatrix = new double[][] {x, y, z};
    }

    /** GOINT: integrate (RPREC, VPREC) over NOMTPI (precision). */
    private void goint(double[] r, double[] vel) {
        intint(r, vel, 0, d("NOMTPI"), false);
    }

    /**
     * INTINT: call INTSTALL; set INTYPFLG for conic when time arg zero... then INTEGRVS and return RATT.
     * MOONFLAG set if CMOONFLG set (X2 = 2).
     */
    private void intint(double[] r, double[] vel, double tet, double tdec1, boolean conic) {
        new IntegrationInitialization(agc).intstall();
        flag("INTYPFLG", conic);
        d("TDEC1", tdec1);
        flag("MOONFLAG", f("CMOONFLG"));
        agc.set("RTX2", f("CMOONFLG") ? 2 : 0);
        v("RCV", r);
        v("VCV", vel);
        d("TET", tet);
        new IntegrationInitialization(agc).integrvs();
        v("RTARG", v("RATT"));
    }

    /** S34/35.5: display DELVLVC (V06N81) and allow overwrite; V06N59 shows DELVEET3. */
    public void s3435_5() {
        if (f("FINALFLG")) {
            // FLAGON: save DVLOS, display V06N81 and test for overwrite
            v("DVLOS", v("DELVLVC"));
            vnpooh(V06N81);
            if (abs(sub(v("DELVLVC"), v("DVLOS"))) != 0) {
                s3435_3();                                      // overwrite of computed delta V
            }
            flag("XDELVFLG", false);
            v("DELVSIN", v("DELVEET3"));
        } else {
            flag("UPDATFLG", true);
        }
        s3435_4();                                              // FLAGOFF
        vnpooh(V06N59);
    }

    /** VN1645: compute +MGA, start TTOGO clock, flash V16N45 (TRKMKCNT, TTOGO, +MGA). */
    public void vn1645() {
        d("+MGA", DP_M01);
        if (f("FINALFLG")) {
            d("+MGA", 2 * DP_M01);
            if (f("REFSMFLG")) {
                if (p3xorp7x() == 0) {
                    get45();
                    return;
                }
                midgimFor(v("DELVSIN"));
            }
        }
        get45();
    }

    private void midgimFor(double[] dv) {
        v("0D", dv);
        midgim();
    }

    /** GET45: initiate COMPTGO, delay 1 s, flash V16N45; TERMINATE/PROCEED/RECYCLE. */
    public void get45() {
        // TC COMPTGO ; QSAVED = SUBEXIT ; BANKCALL DELAYJOB (1SEC)
        // BANKCALL GOFLASH V16N45 -> KILCLOCK (terminate), N45PROC (proceed), CLUPDATE (recycle)
        n45proc();
    }

    /** KILCLOCK: DISPDEX = 0; GOTOPOOH. */
    public void kilclock() {
        agc.set("DISPDEX", 0);
        // TC GOTOPOOH
    }

    /** N45PROC: if FINALFLG set -> KILCLOCK, else phase change and UPFLAG FINALFLG, CLUPDATE. */
    public void n45proc() {
        if (f("FINALFLG")) {
            kilclock();
            return;
        }
        // TC PHASCHNG OCT 04024
        flag("FINALFLG", true);
        clupdate();
    }

    /** CLUPDATE: DISPDEX = 0; phase change; clear UPDATFLG; return to QSAVED. */
    public void clupdate() {
        agc.set("DISPDEX", 0);
        // TC PHASCHNG OCT 04024
        flag("UPDATFLG", false);
    }

    /** DISPLAYE: display V06N55 (ELEV and CENTANG), return on PROCEED. */
    public void displaye() {
        // CAF V06N55; BANKCALL GOFLASH: TERMINATE -> GOTOPOOH; PROCEED -> return; V32 repeats
    }

    /** P3XORP7X: returns 0 for P3X (MODREG high 9 bits zero), else P7X. */
    public int p3xorp7x() {
        int modreg = agc.get("MODREG");
        return ((modreg & 0x7FFF & ~0x3F) == 0) ? 0 : 1;
    }

    /** VNPOOH: flash the given verb/noun; TERMINATE -> GOTOPOOH; PROCEED returns; ENTER recycles. */
    public void vnpooh(int vn) {
        agc.set("VERBNOUN", vn);
        // saves FBANK; BANKCALL GOFLASH
    }

    // ====================================================================================
    // INITVEL
    // ====================================================================================

    /**
     * INITVEL: required initial velocity for a transfer of time DELLT4 from RINIT to RTARG (Lambert), with
     * iteration over precision integration offsets (0D = NN). Outputs RTARG (offset target), VIPRIME,
     * VTPRIME, DELVEET3.
     */
    public void initvel() {
        flag("GUESSW", true);                                   // COGA guess not available
        havegues();
    }

    private void havegues() {
        double[] rtarg1 = v("RTARG").clone();
        double[] rinit = v("RINIT").clone();
        double[] vinit = v("VINIT").clone();
        v("RTARG1", rtarg1);
        int itctr = -1;
        double cozy4 = Math.cos(d("EPSILON4") == 0 ? EPSFOUR * 2 * Math.PI : d("EPSILON4"));
        v("R1VEC", rinit);
        v("R2VEC", rtarg1);
        d("TDESIRED", d("DELLT4"));
        double[] ur1 = unit(rinit);
        v("UN", unit(cross(ur1, vinit)));
        cozy4 = dot(unit(rtarg1), ur1) + cozy4;
        int nOffsets = agc.get("NN");
        while (true) {
            if (cozy4 >= 0 || true) {
                // INITVEL2: if the target lies within the cone around 180 deg, rotate it into the plane of UN
                if (cozy4 < 0 || itctr >= 0) {
                    double[] r2 = v("R2VEC");
                    double[] proj = sub(r2, mul(v("UN"), dot(r2, v("UN"))));
                    v("R2VEC", mul(unit(proj), abs(r2)));
                    if (itctr >= 0) {
                        v("RTARG1", v("R2VEC"));
                    }
                }
            }
            double[] vi = lambert(v("R1VEC"), v("R2VEC"), d("TDESIRED"));
            v("VIPRIME", vi);
            if (nOffsets == 0) {
                break;                                          // INITVEL7: pure conic
            }
            // precision: integrate R1VEC, VIPRIME over DELLT4 and compare with RTARG1
            intint(rinit, vi, d("INTIME"), d("INTIME") + d("DELLT4"), false);
            v("VTARGET", v("VATT1"));
            itctr++;
            if (itctr >= nOffsets) {
                v("R2VEC", v("RTARG1"));
                break;                                          // INITVEL6
            }
            double[] err = sub(v("RTARG1"), v("RATT1"));
            v("R2VEC", add(v("R2VEC"), err));                   // R2VEC += RTARG1 - RATT1
        }
        // INITVEL7
        v("DELVEET3", sub(v("VIPRIME"), vinit));
        v("VTPRIME", nOffsets == 0 ? v("VTARGET") : v("VTARGET"));
        v("RTARG", v("RTARG1"));
    }

    /** Lambert conic solver (COASTFLT LAMBERT, external): returns initial velocity. */
    private double[] lambert(double[] r1, double[] r2, double t) {
        // Delegated to the LAMBERT routine (CONIC_SUBROUTINES); result in VVEC
        return v("VVEC");
    }

    // ====================================================================================
    // MIDGIM
    // ====================================================================================

    /**
     * MIDGIM: computes +MGA (middle gimbal angle) when COMPUTER = 1 (CSM) and AVFLAG = 0 (CSM active),
     * else DELVLVC (delta V in local vertical coordinates, MGLVFLAG set).
     */
    public void midgim() {
        boolean avflag = f("AVFLAG");
        boolean computer = f("COMPUTER");
        if (avflag) {
            if (!computer) {
                getLvc();                                       // MIDGIM1: BOFF COMPUTER GET+MGA falls
                return;
            }
            getMga();
            return;
        }
        if (computer) {
            getLvc();
            return;
        }
        getMga();
    }

    /** GET+MGA: +MGA = arcsin(U(V) . Y(stable member)) (+1 rev if negative). */
    public void getMga() {
        double[] uv = unit(v("0D"));
        double[] ysm = {agc.get("REFSMMAT+6"), agc.get("REFSMMAT+8"), agc.get("REFSMMAT+10")};
        double mga = Math.asin(clamp(dot(uv, ysm))) / (2 * Math.PI);
        if (mga < 0) {
            mga += HALFREV + HALFREV;                           // convert -MGA to +MGA by adding one rev
        }
        d("+MGA", mga);
        flag("MGLVFLAG", false);                                // +MGA calculated
    }

    /** GET.LVC: DELVLVC = transformation of DELTA V (0D) to local vertical coordinates. */
    public void getLvc() {
        double[] u18 = neg(unit(v("RINIT")));                   // U(-R)
        double[] u12 = unit(cross(u18, v("VINIT")));            // U(V*R)
        double[] u6 = unit(cross(u12, u18));                    // transformation matrix rows
        double[][] m = {u6, u12, u18};
        v("DELVLVC", mul(mxv(m, v("0D")), 2));
        flag("MGLVFLAG", true);                                 // LVC calculated
    }

    // ====================================================================================
    // SELECTMU, PERIAPO, SETRAD, PRECSET, store routines
    // ====================================================================================

    /** SELECTMU: select primary-body MU table entries; clears FINALFLG and goes to VN1645. */
    public void selectmu() {
        int x1 = 2;                                             // AXC,1 2D: earth
        int x2 = 0;
        if (f("CMOONFLG")) {
            x1 = 10;                                            // moon
            x2 = 2;
        }
        agc.set("RTX1", Agc.fromSigned(-x1));
        d("RTSR1/MU", d("MUTABLE-2"));                          // MUTABLE -2,1
        double rtmu = d("MUTABLE+4");                           // MUTABLE +4,1
        if (f("CMOONFLG")) {
            rtmu = rtmu * 64;                                   // SR 6D
        }
        d("RTMU", rtmu);
        agc.set("RTX2", x2);
        flag("FINALFLG", false);
        vn1645();
    }

    /** PERIAPO1: shift RACT3/VIPRIME by the body scale, then PERIAPO. */
    public void periapo1() {
        v("VVEC", v("PERI_IN_V"));
        v("RVEC", v("PERI_IN_R"));
        periapo();
    }

    /**
     * PERIAPO: apocenter and pericenter radii/altitudes for the trajectory (RVEC, VVEC). Outputs
     * 2D apocenter radius, 4D apocenter altitude, 6D pericenter radius, 8D pericenter altitude, ECC, XXXALT.
     */
    public void periapo() {
        double xxxalt = setrad();
        d("XXXALT", xxxalt);
        double mu = d("MU") == 0 ? 3.986e14 : d("MU");
        double r = abs(v("RVEC"));
        double vmag = abs(v("VVEC"));
        double energy = vmag * vmag / 2 - mu / r;
        double a = -mu / (2 * energy);
        double[] h = cross(v("RVEC"), v("VVEC"));
        double ecc = Math.sqrt(Math.max(0, 1 + 2 * energy * dot(h, h) / (mu * mu)));
        d("ECC", ecc);
        double ra = a * (1 + ecc);
        double rp = a * (1 - ecc);
        d("2D", ra);
        d("4D", ra - xxxalt);
        d("6D", rp);
        d("8D", rp - xxxalt);
        d("POSTTPI", rp - xxxalt);
    }

    /** SETRAD: radius of the primary body (RPAD for earth, |RLS| for moon surface). */
    private double setrad() {
        if (agc.get("RTX2") == 0) {
            return RPAD;
        }
        return abs(v("RLS"));
    }

    /** SHIFTR1: scale a distance by the body scale (SL* 0,2). */
    private double shiftr1(double x) {
        return x;
    }

    /** PRECSET: integrate LM then CSM to TDEC1 (saving TDEC2) and store vectors via LEMSTORE/CSMSTORE. */
    public void precset() {
        d("TDEC2", d("TDEC1"));
        new IntegrationInitialization(agc).lemprec();
        lemstore();
        d("TDEC1", d("TDEC2"));
        new IntegrationInitialization(agc).csmprec();
        csmstore();
    }

    /** LEMSTORE: RATT/VATT to ACTIVE (AVFLAG on) or PASSIVE vectors. */
    public void lemstore() {
        if (f("AVFLAG")) {
            active();
        } else {
            passive();
        }
    }

    /** CSMSTORE: RATT/VATT to PASSIVE (AVFLAG on) or ACTIVE vectors. */
    public void csmstore() {
        if (f("AVFLAG")) {
            passive();
        } else {
            active();
        }
    }

    /** ACTIVE: store RATT, VATT in RACT3, VACT3. */
    public void active() {
        v("RACT3", v("RATT"));
        v("VACT3", v("VATT"));
    }

    /** PASSIVE: store RATT, VATT in RPASS3, VPASS3. */
    public void passive() {
        v("RPASS3", v("RATT"));
        v("VPASS3", v("VATT"));
    }

    /** VECSHIFT: shift RVEC/VVEC by the RTX2 (earth/moon) scale. */
    public void vecshift() {
        // VSR* 0,2 on two vectors: scale factor 2^-RTX2 -- doubles are unscaled, nothing to do
    }

    // ====================================================================================
    // R36: range, range rate and angle display (V90)
    // ====================================================================================

    /** R36: asks for option via V04N12 (OPTIONX = 2, OPTIONX+1 = 1); PROCEED -> R36A. */
    public void r36() {
        agc.set("OPTIONX", 2);
        agc.set("OPTIONX+1", 1);
        // GOXDSPF V04N12: TERMINATE -> ENDEXT, PROCEED -> POSTJUMP R36A, ENTER recycles
        r36a();
    }

    /** R36A: DSPTEMX = 0 (first display time = now); OPTIONY = vehicle option (L). */
    public void r36a() {
        agc.set("DSPTEMX", 0);
        agc.set("DSPTEMX+1", 0);
        agc.set("OPTIONY", agc.get("L"));
        r36p3();
    }

    /** R36P3: GOMARKF V06N16; time 0 => LREGCHK else ASTROTIM; then R36INT. */
    public void r36p3() {
        // BANKCALL GOMARKF V06N16N: TERMINATE/PROCEED/RECYCLE
        int hi = agc.get("DSPTEMX");
        int lo = agc.get("DSPTEMX+1");
        if (hi != 0) {
            astrotim();
        } else if (lo == 0) {
            enttim2();
        } else {
            astrotim();
        }
    }

    public void astrotim() {
        agc.set("MPAC", agc.get("DSPTEMX"));
        agc.set("MPAC+1", agc.get("DSPTEMX+1"));
        r36int();                                               // RTB DPMODE
    }

    public void enttim2() {
        agc.set("MPAC", agc.get("LOADTIME"));
        r36int();
    }

    /**
     * R36INT: integrate THIS (OPTIONY=0 CSM display of the other) and OTHER vehicle to the time, then compute
     * RANGE, RRATE, RTHETA between the vehicles.
     */
    public void r36int() {
        d("TDEC1", agc.get("MPAC"));
        int optiony = agc.get("OPTIONY") >> 1;
        if (optiony == 0) {
            new IntegrationInitialization(agc).othprec();       // R36PROG2
        } else {
            thisprec();
        }
        // R36PROG3
        v("RPASS36", v("RATT"));
        v("UNP36", unit(cross(v("RATT"), v("VATT"))));
        d("TDEC1", d("TAT"));
        if (optiony == 0) {
            thisprec();                                         // R36PROG4
        } else {
            new IntegrationInitialization(agc).othprec();
        }
        // R36PROG5
        double[] va = v("VATT");
        double[] ra = v("RATT");
        d("30D", d("TAT"));                                     // save time for redisplay
        double[] los = sub(v("RPASS36"), ra);                   // R_P - R_A
        d("RANGE", 2 * dot(v("UNP36"), los));                   // Y = U . R (SL1)
        d("RRATE", 2 * dot(v("UNP36"), va));                    // Ydot = U . V
        double[] ura = unit(ra);
        double[] fwd = unit(cross(cross(ura, va), ura));        // unit horizontal in forward direction
        double[] horiz = unit(sub(los, mul(ura, dot(los, ura))));   // LOS projected into horizontal plane
        double psi = Math.acos(clamp(dot(horiz, fwd))) / (2 * Math.PI);   // PSI = ARCCOS(U . U)
        if (dot(cross(horiz, fwd), ura) < 0) {
            psi = 1 - psi;                                      // DPPOSMAX - RTHETA
        }
        d("RTHETA", psi);
        r36tag2();
    }

    private void thisprec() {
        if (f("AVFLAG")) {
            new IntegrationInitialization(agc).csmprec();
        } else {
            new IntegrationInitialization(agc).lemprec();
        }
    }

    /** R36TAG2: DSPTEMX = time (SGNAGREE); display V06N90 (Y, Ydot, PSI): any key ends, recycle redisplays. */
    public void r36tag2() {
        agc.set("DSPTEMX", (int) d("30D"));
        // GOMARKF V06N90N: TERMINATE/PROCEED -> ENDEXT; RECYCLE -> R36P3 (redisplay output)
    }
}
