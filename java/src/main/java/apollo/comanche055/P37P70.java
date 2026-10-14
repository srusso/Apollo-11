package apollo.comanche055;

import apollo.agc.Agc;
import java.util.HashMap;
import java.util.Map;

/**
 * P37_P70.agc (Colossus 2A / Comanche 055): P37 "Return to Earth" (RTE) targeting program and its
 * subroutines: input/display (P37, P37N, P370GOF, RTEVN, RTEDISP, P370ALRM), conic/precision
 * trajectory iteration (RTE299..RTE390, PREC100), the X(T1)/delta-V iterator (V2T100, GAMDV10,
 * DVCALC, XT1LIM), the landing site computation and the INVC100 initialisation.
 *
 * The interpretive double precision quantities are modelled as {@code double} values held in a
 * local store by the AGC label name ({@link #d}, {@link #vec}); each quantity is kept in its stored
 * (scaled) fraction form, so the "Bn" scale comments and SLn/SRn shifts of the original apply
 * literally. The conic subroutines (TIMERAD, PARAM, GETERAD, CSMPREC, INTEGRVS, AUGEKUGL,
 * LAT-LONG) are external and reached through {@link #extern(String, Object...)}.
 * Alarm codes: 00605 too many iterations, 00610 lambda less than 1, 00612 not in sphere of
 * influence, 00613 reentry angle out of limits.
 */
public class P37P70 {
    // ---- constants (RTECON1) ----
    public static final double ONE_B1 = 1.0 / 2, ONE_B2 = 1.0 / 4, ONE_B3 = 1.0 / 8, ONE_B4 = 1.0 / 16;
    public static final double ONE_B10 = Math.scalb(1.0, -10), ONE_B12 = Math.scalb(1.0, -12);
    public static final double ONE_B13 = Math.scalb(1.0, -13), ONE_B17 = Math.scalb(1.0, -17);
    public static final double ONE_B25 = Math.scalb(1.0, -25), ONE_B28 = Math.scalb(1.0, -28);
    public static final double ZERORTE = 0.0;
    public static final double M144RTE = -144.0 * Math.scalb(1.0, -28);
    public static final double M15RTE = -15.0;
    public static final double TEN_RTE = 10.0;
    public static final double MINUS_0_6 = -0.6;
    public static final double ONE_POINT_ONE_B1 = 1.1 / 2;
    public static final double M6RTEB28 = -6.0;
    public static final double POSMAX2 = 037777.0 / 040000.0;     // 2RTEB1: 3777737777 octal
    public static final double M9RTEB28 = -9.0;
    public static final double M8RTEB28 = -8.0;
    public static final double D30480RTE = 30480.0 * Math.scalb(1.0, -29);
    public static final double VCSPS = 31.510396 * Math.scalb(1.0, -5);   // see 2VEXHUST
    public static final double VCRCS = 27.0664 * Math.scalb(1.0, -5);
    public static final double MDOTRCS = 0.0016375 * Math.scalb(1.0, -3);
    public static final double CSUBT = 0.5;
    public static final int OCT605 = 0605;
    public static final int OCT612 = 0612;
    public static final int OCT613 = 0613;
    public static final double MCOS7_5 = -0.99144486;
    public static final double MSIN7_5 = -0.13052619;
    public static final double MCOS22_5 = -0.92387953 * 0.25;
    public static final double THETA165 = 0.4583333333;
    public static final double THETA210 = 0.5833333333;
    public static final double EPC1RTE = 0.99966 * 0.5;
    public static final double EPC2RTE = 100.0 * Math.scalb(1.0, -29);
    public static final double EPC3RTE = 0.001;
    public static final double EPC4RTE = 0.00001;
    public static final double EPC5RTE = 0.01 * Math.scalb(1.0, -6);
    public static final double EPC6RTE = 0.000007 * 0.5;
    public static final double EPC7RTE = 1000.0 * Math.scalb(1.0, -29);
    public static final double EPC8RTE = 0.002;
    public static final double EPC9RTE = Math.scalb(1.0, -25);
    public static final double EPC10RTE = 0.0001 * Math.scalb(1.0, -7);
    public static final double C4RTE = -6.986643E7 * Math.scalb(1.0, -30);
    public static final double K1RTE = 7.0E6 * Math.scalb(1.0, -29);
    public static final double K2RTE = 6495000.0 * Math.scalb(1.0, -29);
    public static final double K3RTE = -0.06105;
    public static final double K4RTE = -0.10453;
    public static final double RTMURTE = 199650.501 * Math.scalb(1.0, -18);
    public static final double E3RTE = 121920.0 * Math.scalb(1.0, -29);
    /** MAMAX2 polynomial (cubic in R(T1)), coefficients with scale factors applied. */
    private static final double[] MAMAX_POLY = {
        181000434.0 * Math.scalb(1.0, -31), 1.50785145 * Math.scalb(1.0, -2),
        -6.49993057E-9 * Math.scalb(1.0, 27), 9.76938926E-18 * Math.scalb(1.0, 56)};
    /** 1 - exp(-DV/VC) polynomial coefficients (P37T). */
    private static final double[] BURN_POLY = {
        5.66240507E-4 * Math.scalb(1.0, -3), 9.79487897E-1 * Math.scalb(1.0, -1), -0.388281955 * Math.scalb(1.0, 1)};
    /** V(T2) polynomial for X(T2) (RTE369). */
    private static final double[] X2_POLY = {
        0.0, -4.8760771E-2 * Math.scalb(1.0, 4), 4.5419476E-4 * Math.scalb(1.0, 11), -1.4317675E-6 * Math.scalb(1.0, 18)};
    /** Display verb/noun words. */
    public static final String V6N33RTE = "0633", V4N06RTE = "0406", V6N61RTE = "0661",
        V6N39RTE = "0639", V6N60RTE = "0660", V6N81RTE = "0681", V5N09RTE = "0509";

    private final Agc agc;
    private final Map<String, Double> dp = new HashMap<>();
    private final Map<String, double[]> vecs = new HashMap<>();

    /** Thrown for GOTOPOOH / ENDOFJOB. */
    private static final class JobEnd extends RuntimeException {
        private static final long serialVersionUID = 1L;
    }

    /** Thrown for "recycle to P37" requests. */
    private static final class Recycle extends RuntimeException {
        private static final long serialVersionUID = 1L;
    }

    public P37P70(Agc agc) {
        this.agc = agc;
    }

    /** Hook for external routines. Returns nothing; results are written through {@link #d}/{@link #vec}. */
    protected void extern(String routine, Object... args) {
    }

    /**
     * Flash a verb/noun display (GOFLASH/GOFLASHR). Returns {@code true} on PROCEED and
     * {@code false} on RECYCLE (ENTER/V32); a terminate (V34) is signalled by returning null-equivalent
     * through {@link #terminate()}.
     */
    protected boolean goFlash(String verbNoun) {
        return true;
    }

    private void terminate() {
        throw new JobEnd();
    }

    // ---- store helpers ----
    public double d(String name) {
        return dp.getOrDefault(name, 0.0);
    }

    public void d(String name, double v) {
        dp.put(name, v);
    }

    public double[] vec(String name) {
        return vecs.getOrDefault(name, new double[3]);
    }

    public void vec(String name, double[] v) {
        vecs.put(name, v.clone());
    }

    private static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }

    private static double abval(double[] a) {
        return Math.sqrt(dot(a, a));
    }

    private static double[] unit(double[] a) {
        double n = abval(a);
        return n == 0 ? new double[3] : new double[] {a[0] / n, a[1] / n, a[2] / n};
    }

    private static double[] scale(double[] a, double k) {
        return new double[] {a[0] * k, a[1] * k, a[2] * k};
    }

    private static double[] add(double[] a, double[] b) {
        return new double[] {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
    }

    private static double[] sub(double[] a, double[] b) {
        return new double[] {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    private static double poly(double x, double[] c) {
        double r = 0;
        for (int i = c.length - 1; i >= 0; i--) {
            r = r * x + c[i];
        }
        return r;
    }

    private static double sign(double mag, double s) {
        return s < 0 ? -Math.abs(mag) : Math.abs(mag);
    }

    private boolean flag(String name) {
        return d("flag." + name) != 0;
    }

    private void setFlag(String name, boolean on) {
        d("flag." + name, on ? 1 : 0);
    }

    // ===================================================================================
    // P37 main
    // ===================================================================================

    /** P37: return-to-Earth. Not restartable (PHASCHNG group 4 inactive). */
    public void p37() {
        extern("PHASCHNG", 4);
        while (true) {
            try {
                p37Body();
                return;
            } catch (Recycle r) {
                continue;                                  // TCF P37
            } catch (JobEnd e) {
                return;
            }
        }
    }

    private void p37Body() {
        agc.set("ECSTEER", 04000);                          // AXT,1 04000 ; SXA,1 ECSTEER
        d("VPRED", 0);
        d("GAMMAEI", 0);
        while (!goFlash(V6N33RTE)) {                        // input TIG in SPRTETIG
            // recycle: display new data
        }
        d("SPRTETIG", d("TIG"));
        while (!p37gfrb1(V6N60RTE)) {                       // reentry angle (GAMMAEI) and DV (RTEDVD)
            // recycle
        }
        rte299();
    }

    /** RTE299: initialise iteration variables and run the conic solution. */
    public void rte299() {
        agc.set("OVFIND", 0);
        d("RTEDVD", d("VPRED"));
        d("RTEGAM2D", d("GAMMAEI"));
        d("CONICX1", ONE_B13);
        d("MAMAX1", C4RTE);
        invc100();
        setFlag("SLOWFLG", false);
        double dvd = d("RTEDVD");
        if (dvd < 0) {                                      // BPL ABS
            d("RTEDVD", -dvd);
            if (d("R(T1)") - K1RTE >= 0) {
                setFlag("SLOWFLG", true);
            }
        }
        // RTE317: MAMAX2 = C0+C1*R+C2*R**2+C3*R**3 (B30), then SL1
        d("MAMAX2", 2 * poly(d("R(T1)"), MAMAX_POLY));
        d("NN1A", M9RTEB28);
        d("RCON", K2RTE);                                   // RTE320
        rte360Loop();
    }

    /** RTE320..RTE390: iterate on RCON and X(T2) until the conic matches. */
    private void rte360Loop() {
        // initial X(T2)
        double gam = d("RTEGAM2D");
        if (gam != 0) {
            double ang = ONE_B2 - gam;                      // revolutions
            d("X(T2)", Math.cos(2 * Math.PI * ang) / Math.sin(2 * Math.PI * ang));   // COT(GAM2D) B0
        } else if (d("R(T1)") - K1RTE < 0) {
            d("X(T2)", K3RTE);                              // RTE350
        } else {
            d("X(T2)", K4RTE);
        }
        while (true) {
            // RTE360
            if (v2t100() != 0) {
                rtealrm();
                return;
            }
            vec("RVEC", vec("R(T1)/"));                     // RTE367
            d("RDESIRED", d("RCON"));
            vec("VVEC", vec("V2(T1)/"));
            tmrad100();
            d("T2", d("T12") + d("T1"));
            double xcomputed;
            if (d("RTEGAM2D") == 0) {                       // RTE369
                xcomputed = 8 * (poly(abval(vec("V(T2)/")), X2_POLY) + d("RTED1"));   // SL3
            } else {
                xcomputed = d("X(T2)");                     // RTE372
            }
            double xerr = xcomputed - d("X(T2)");           // RTE373, X(T2)ERR B0
            extern("GETERAD", unit(vec("R(T2)/")));         // ALPHAV = UR2
            vec("ALPHAV", unit(vec("R(T2)/")));
            double rconNew = d("GETERAD") + E3RTE;          // RCON' (E1/..+E2 BETA11)**.5 + E3
            boolean radiusConverged = Math.abs(rconNew - d("RCON")) - EPC2RTE < 0;
            if (!(radiusConverged && Math.abs(xerr) - EPC3RTE < 0)) {
                // RTE375
                double nn1a = d("NN1A") + ONE_B28;
                if (nn1a >= 0) {
                    d("MPAC", OCT605);
                    rtealrm();                              // too many iterations
                    return;
                }
                d("NN1A", nn1a);
                double dx;
                if (nn1a - M8RTEB28 == 0) {                 // RTE385
                    dx = xerr;
                } else {
                    double z1 = xerr - d("DRCON");          // X(T2)ERR - X(T2)ERR'
                    double z2 = d("RPRE'") - d("X(T2)");    // X(T2)PRI - X(T2)
                    dx = xerr * (z2 / z1);
                }
                d("RCON", rconNew);                         // RTE390
                d("DRCON", xerr);
                d("RPRE'", d("X(T2)"));
                d("X(T2)", d("X(T2)") + dx);
                continue;                                   // reiterate at RTE360
            }
            break;
        }
        p37e();
    }

    /** P37E: display conic solution, choose apogee/perigee side (PHI2) then precision trajectory. */
    public void p37e() {
        rtevn();
        // RTE505
        if (d("RCON") - d("PCON") * d("BETA1") <= 0) {
            d("PHI2", -ONE_B2);                             // RTE510: entry near perigee
        } else {
            d("PHI2", ONE_B2);                              // entry near apogee
        }
        if (prec100() == 0) {                               // RTE625
            p37g();
        } else {
            rtealrm();
        }
    }

    /** RTEALRM: display the alarm, then recycle to P37. */
    public void rtealrm() {
        p370alrm();
        throw new Recycle();
    }

    /**
     * RTEVN: display preparation (RTEDISP) and V06N61 (lat/long), T21 (V06N39), V06N60
     * (V(T2), FPA), V06N81 (delta V). Recycle returns to P37.
     */
    public void rtevn() {
        rtedisp();
        if (!goFlash(V6N61RTE)) {                           // LAT(SPL), LNG(SPL), blank
            throw new Recycle();
        }
        if (!blank37(4)) {                                  // blank R3 (CAF FOUR; 37BLANK +1)
            throw new Recycle();
        }
        if (!goFlash(V6N39RTE)) {                           // T21 in T3TOT4
            throw new Recycle();
        }
        if (!p37gfrb1(V6N60RTE)) {                          // blank, VPRED, GAMMAEI
            throw new Recycle();
        }
        if (!goFlash(V6N81RTE)) {                           // DELVLVC
            throw new Recycle();
        }
    }

    /** P37G: display, then P37N burn-option / TIG computation. */
    public void p37g() {
        rtevn();
        p37n();
    }

    /** P37N: choose SPS/RCS, compute TIG from burn time and run the target pass loop P37W. */
    public void p37n() {
        agc.set("OPTION1", 7);
        agc.set("OPTION2", 1);                              // SPS assumed
        while (!goFlash(V4N06RTE)) {
            // recycle
        }
        double mdot;
        double vc;
        if (agc.get("OPTION2") - 1 == 0) {                  // RCS (P37Q)
            mdot = MDOTRCS;
            mdot *= flag("NJETSFLG") ? 2 : 4;               // SL1 (and SL1 if NJETSFLG off)
            vc = VCRCS;
        } else {
            mdot = d("EMDOT");                              // SPS
            vc = VCSPS;
        }
        double ratio = d("DV") / vc;                        // DV/VC B7 - B5 = B2
        double a = poly(ratio, BURN_POLY);                  // (1-E)**(-DV/VC) = A B3
        double dtb = d("WEIGHT/G") * a / mdot;              // DTB = (M0/MDOT)A B16
        d("TIG", d("T1") - CSUBT * dtb);                    // TIG = T1 - CT*DTB
        while (!goFlash(V6N33RTE)) {
            // recycle
        }
        agc.set("VHFCNT", 0);
        agc.set("TRKMKCNT", 0);
        rtenck1();                                          // conic integrate R1,V1 over T12
        // UR2 , K/ = -UR1 cos 7.5 - UH sin 7.5
        double[] ur2 = unit(vec("R(T2)/"));
        double[] k = add(scale(vec("UR1/"), MCOS7_5), scale(vec("UH/"), MSIN7_5));
        double kdotur2 = dot(k, ur2);
        if (kdotur2 + MCOS22_5 < 0) {                       // K/.UR2 compared with cos 22.5
            double theta;
            if (dot(vec("UH/"), vec("R(T2)/")) < 0) {
                theta = THETA165;                           // P37U not taken
            } else {
                theta = THETA210;                           // P37U
            }
            d("SNTH", Math.sin(2 * Math.PI * theta));       // P37V
            d("CSTH", Math.cos(2 * Math.PI * theta));
            setFlag("RVSW", false);
            vec("RVEC", vec("R(T1)/"));
            vec("VVEC", vec("V2(T1)/"));
            extern("TIMETHET");
        }
        p37w();
    }

    /** P37W: set up target block and loop displaying VN1645 until terminated. */
    public void p37w() {
        setFlag("XDELVFLG", false);
        setFlag("NORMSW", false);
        setFlag("FINALFLG", true);
        vec("RTARG", vec("R(T1)/"));                        // STADR: RTARG (vector in MPAC)
        d("TPASS4", d("T") + d("T1"));
        vec("DELVSIN", sub(vec("V2(T1)/"), vec("V(T1)/")));
        while (true) {
            extern("VN1645");                               // TRKMKCNT, TTOGO, +MGA display
            if (!p37wContinue()) {
                return;
            }
        }
    }

    /** Hook to terminate the otherwise endless P37W display loop. */
    protected boolean p37wContinue() {
        return false;
    }

    /**
     * P37GFRB1: flash display with a blank option; PROCEED goes on, RECYCLE returns false.
     * (ENDOFJOB on the "blank" path.)
     */
    public boolean p37gfrb1(String vn) {
        boolean r = goFlash(vn);                            // P370GOFR
        if (!r) {
            return false;
        }
        return blank37(1);
    }

    /** 37BLANK: BLANKET the requested register; terminate-on-ENDOFJOB; recycle returns false. */
    public boolean blank37(int which) {
        extern("BLANKET", which);
        return true;
    }

    /** P370GOF / P370GOFR: flash; V34 -> GOTOPOOH; returns whether PROCEED. */
    public boolean p370gof(String vn) {
        return goFlash(vn);
    }

    /** P370ALRM: VARALARM with code in MPAC then V05N09 flash; PROCEED re-flashes (-4). */
    public void p370alrm() {
        agc.set("SPRTEX", 1);
        extern("VARALARM", (int) d("MPAC"));
        while (!goFlash(V5N09RTE)) {
            // loop: any key re-displays
        }
    }

    // ===================================================================================
    // TMRAD100 / RTEDISP
    // ===================================================================================

    /** TMRAD100: TIMERAD for the (R,V) in RVEC/VVEC/RDESIRED; gives R(T2)/, V(T2)/, T12. */
    public void tmrad100() {
        setFlag("RVSW", false);
        agc.set("SGNRDOT", 020000);
        extern("TIMERAD", (int) d("CONICX1"));
        vec("V(T2)/", vec("VTIMERAD"));
        vec("R(T2)/", vec("RTIMERAD"));
        d("T12", d("T"));
    }

    /** RTEDISP: prepare display values; landing site computation. */
    public void rtedisp() {
        double[] v2 = vec("V(T2)/");
        d("VPRED", abval(v2));                              // V(T2)
        d("T3TOT4", d("T2") - d("SPRTETIG"));               // T21
        double cosg = dot(unit(vec("R(T2)/")), unit(v2));
        d("GAMMAEI", ONE_B2 - Math.acos(Math.max(-1, Math.min(1, 2 * cosg))) / (2 * Math.PI));   // FPA at T2
        double[] dv = sub(vec("V2(T1)/"), vec("V(T1)/"));
        double x = -dot(dv, vec("UR1/"));
        double z = dot(dv, vec("UH/"));
        double[] delvlvc = scale(new double[] {x, ZERORTE, z}, 2);   // VSL1
        if (!flag("RETROFLG")) {                            // RETROFLG off: leave as is
            // BOFF DCOMP RETROFLG RTD18
        } else {
            delvlvc[0] = -delvlvc[0];                       // negate X component, retrograde
        }
        vec("DELVLVC", delvlvc);
        d("VGDISP", abval(delvlvc));
        // ***** LANDING SITE COMPUTATION *****
        vec("RVEC", vec("R(T2)/"));
        d("RDESIRED", abval(vec("R(T2)/")) - D30480RTE);
        vec("VVEC", v2);
        tmrad100();                                         // R3, V3, T23
        double[] ur3 = unit(vec("R(T2)/"));
        double[] uv3 = unit(vec("V(T2)/"));
        double vt3 = abval(vec("V(T2)/"));
        double gammaE = Math.asin(Math.max(-1, Math.min(1, 2 * dot(ur3, uv3)))) / (2 * Math.PI);
        d("PHIE", extHook("AUGEKUGL", Math.abs(gammaE), vt3));
        double tls = d("PHIE") + d("T12") + d("T2");        // T(LS) = T2 & T23 & TE
        d("T(LS)", tls);
        double phie = d("PHIE");
        double range = d("P37RANGE");
        if (range != 0) {
            phie = range;                                   // override range (PCR 261)
        }
        d("LNG(SPL)", Math.sin(2 * Math.PI * phie));
        d("LAT(SPL)", Math.cos(2 * Math.PI * phie));
        double[] uh3 = unit(cross(cross(ur3, uv3), ur3));
        vec("ALPHAV", add(scale(ur3, d("LAT(SPL)")), scale(uh3, d("LNG(SPL)"))));
        setFlag("ERADFLAG", false);
        setFlag("LUNAFLAG", false);
        extern("LAT-LONG");
        d("LAT(SPL)", d("LAT"));                            // latitude landing site
        d("LNG(SPL)", d("LONG"));                           // longitude landing site
    }

    private double extHook(String routine, double... args) {
        extern(routine, (Object[]) boxed(args));
        return d(routine);
    }

    private static Object[] boxed(double[] a) {
        Object[] o = new Object[a.length];
        for (int i = 0; i < a.length; i++) {
            o[i] = a[i];
        }
        return o;
    }

    // ===================================================================================
    // INVC100: initial R1, V1 at time TIG
    // ===================================================================================

    /** INVC100: precision-integrate R0,V0 to TIG (R1,V1); compute UR1/, UV1/, CFPA, UH/, RETROFLG. */
    public void invc100() {
        d("TDEC1", d("SPRTETIG"));
        extern("CSMPREC");                                  // precision integration R0,V0 -> R1,V1
        vec("R(T1)/", vec("RATT"));
        vec("V(T1)/", vec("VATT"));
        d("T1", d("TAT"));
        if (d("P(T1)") != 0) {                              // INVC107
            d("MPAC", OCT612);                              // not in the proper sphere of influence
            rtealrm();
            return;
        }
        double[] ur1 = unit(vec("R(T1)/"));                 // INVC109
        vec("UR1/", ur1);
        d("R(T1)", abval(vec("R(T1)/")));                   // B29
        double[] uv1 = unit(vec("V(T1)/"));
        vec("UV1/", uv1);
        double cfpa = 2 * dot(ur1, uv1);                    // SL1
        d("CFPA", cfpa);
        double[] n;
        if (Math.abs(cfpa) - EPC1RTE < 0) {                 // INVC115: not near rectilinear
            n = cross(ur1, uv1);                            // N/ = UR X UV  B2
        } else {
            n = new double[] {0, 0, ONE_B2};                // N/ = (0,0,1)
        }
        setFlag("RETROFLG", false);
        if (n[2] < 0) {                                     // retrograde orbit
            n = scale(n, -1);
            setFlag("RETROFLG", true);
        }
        vec("UH/", unit(cross(n, ur1)));                    // UH/ B1
    }

    // ===================================================================================
    // PREC100: precision trajectory computation
    // ===================================================================================

    /** PREC100: iterate RCON so the precision trajectory reaches RD; returns 0 if OK else alarm code. */
    public double prec100() {
        d("NN1A", TEN_RTE);                                 // 10RTE
        d("RD", d("RCON"));
        while (true) {
            d("DT21PR", POSMAX2);                           // PREC120
            d("NN2", M15RTE);
            rtenck3();
            while (true) {
                extern("PARAM");                            // P, R1A, R1, COGA
                d("P/RPRE", d("P"));                        // PREC125
                d("R/APRE", d("R1A"));
                d("RPRE", d("R1"));
                double xt2pre = 32 * d("COGA");             // SL 5
                d("X(T2)PRE", xt2pre);
                if (Math.abs(-xt2pre + d("X(T2)")) - EPC4RTE < 0) {
                    break;                                  // to PREC175
                }
                // PREC130
                if (d("NN2") >= 0) {
                    return OCT605;                          // PREC132: too many iterations
                }
                double beta4;
                if (d("NN1A") == 0) {                       // PREC162
                    beta4 = d("RD") / d("RPRE");
                } else {
                    double beta2 = 4 * d("P/RPRE") * d("BETA1") * 2;             // B2
                    double beta3 = ONE_B4 - 16 * d("R/APRE") * beta2;
                    if (beta3 < 0) {                        // PREC160
                        beta4 = ONE_B1 / d("R/APRE");
                    } else {                                // PREC155
                        beta4 = beta2 / (ONE_B3 - d("PHI2") * Math.sqrt(4 * beta3));
                    }
                }
                double beta12 = -(beta4 - ONE_B1);          // PREC165
                if (beta12 >= 0 && d("X(T2)PRE") >= 0) {
                    beta12 = -beta12;
                }
                d("BETA12", beta12);
                if (Math.abs(beta12) - EPC6RTE < 0) {
                    break;                                  // PREC168 -> PREC175
                }
                double rf = 2 * beta12 * d("RPRE");         // new radius
                // PREC170
                d("NN2", d("NN2") + ONE_B28);
                vec("RVEC", vec("R(T2)/"));
                setFlag("RVSW", true);
                vec("VVEC", scale(vec("V(T2)/"), beta12 < 0 ? -1 : 1));
                agc.set("SGNRDOT", beta12 < 0 ? 1 : 0);
                d("RDESIRED", rf);
                extern("TIMERAD");                          // DT12: time correction to new radius
                double dt21 = sign(d("T"), beta12);
                double beta13 = dt21 / d("DT21PR");
                double beta14 = beta13 < 0 ? MINUS_0_6 : POSMAX2;
                if (beta13 - ONE_B3 >= 0 || beta14 - ONE_B3 >= 0 && beta13 >= 0) {
                    dt21 = beta14 * d("DT21PR");            // limit DT21 step
                }
                d("DT21", dt21);
                d("DT21PR", dt21);
                rtenck2();                                  // integrate DT21 from T2
            }
            // PREC175
            double rerr = d("RPRE") - d("RD");
            if (Math.abs(rerr) - EPC7RTE < 0) {
                return prec220();
            }
            if (d("NN1A") == 0) {
                return OCT605;
            }
            if (d("NN1A") - TEN_RTE == 0) {                 // PREC207: first pass
                d("RCON", d("RD") * d("RD") / d("RPRE"));
                d("DRCON", d("RCON") - d("RD"));
            } else {                                        // PREC205
                double s = 4 * d("DRCON") / (d("RPRE'") - d("RPRE"));    // B2
                if (s > 4 || s < -4) {
                    s = -4;                                 // S GR 4 or LS -4
                }
                d("DRCON", 4 * s * rerr);                   // DRCON = S(RERR) B29
                d("RCON", d("RCON") + d("DRCON"));
            }
            d("RPRE'", d("RPRE"));                          // PREC210
            d("NN1A", d("NN1A") - ONE_B28);
            double r = v2t100();
            if (r != 0) {
                return r;                                   // PRECX
            }
        }
    }

    private double prec220() {
        if (Math.abs(d("X(T2)") - d("X(T2)PRE")) - EPC8RTE < 0) {
            return 0;                                       // PREC225
        }
        return OCT613;                                      // reentry angle out of limits
    }

    // ===================================================================================
    // Conic integration wrappers (RTENCK1/2/3)
    // ===================================================================================

    /** RTENCK1: integrate R(T1)/,V2(T1)/ from T1 over T2 (INTYPFLG = conic). */
    public void rtenck1() {
        setFlag("INTYPFLG", true);
        rtenck3b();
    }

    /** RTENCK2: continue from R(T2)/,V(T2)/ at T2 with DT21 (INTYPFLG = precision). */
    public void rtenck2() {
        setFlag("INTYPFLG", false);
        vec("RCV", vec("R(T2)/"));
        vec("VCV", vec("V(T2)/"));
        d("TET", d("T2"));
        d("TDEC1", d("T2") + d("DT21"));
        rtenck3d();
    }

    /** RTENCK3: precision integrate R(T1)/,V2(T1)/ from T1 to T2. */
    public void rtenck3() {
        setFlag("INTYPFLG", false);
        rtenck3b();
    }

    private void rtenck3b() {
        vec("RCV", vec("R(T1)/"));
        vec("VCV", vec("V2(T1)/"));
        d("TET", d("T1"));
        d("TDEC1", d("T2"));
        rtenck3d();
    }

    private void rtenck3d() {
        setFlag("MOONFLAG", false);
        extern("INTEGRVS");
        vec("R(T2)/", vec("RATT"));
        d("T2", d("TAT"));
        vec("V(T2)/", vec("VATT"));
    }

    // ===================================================================================
    // V2T100: required velocity at T1 for given RCON and X(T2)
    // ===================================================================================

    /**
     * V2T100: compute V2(T1)/ by iterating on X(T1) (and delta V) for target radius RCON and
     * X(T2). Returns 0 on success, otherwise an alarm code (610 if lambda less than 1, 605 too many).
     */
    public double v2t100() {
        double rcon = d("RCON");
        if (rcon < 0 || rcon - d("R(T1)") >= 0) {
            agc.set("POODOO", 00610);                       // no solution if lambda less than 1
            extern("POODOO", 00610);
            return 00610;
        }
        setFlag("F2RTE", false);
        double lambda = d("R(T1)") / rcon;                  // B1
        double x2 = d("X(T2)");
        double beta1 = ONE_B1 + 0.5 * x2 * x2;              // 1 + X(T2)**2 = BETA1 B1
        d("BETA1", beta1);
        double beta5 = beta1 * lambda;                      // BETA5 (28D)
        d("BETA5", beta5);
        d("THETA1", (beta5 * lambda - ONE_B17) );           // BETA5*LAMBDA - 1 = THETA1 B17
        d("THETA2", 2 * d("R(T1)") * (lambda - ONE_B1));    // 2 R(T1)(LAMBDA-1) = THETA2
        d("THETA3", RTMURTE / d("R(T1)"));                  // MU**.5 / R(T1) = THETA3
        d("XT1MIN", -xt1lim(d("MAMAX1")));                  // X(T1)MIN B5
        d("DXT1MAX", -d("XT1MIN") / 16);                    // DX(T1)MAX
        d("XT1MAX", xt1lim(d("MAMAX2")));                   // X(T1)MAX
        if (d("NN1A") < 0) {
            if (d("RTEDVD") == 0) {                         // V2T105
                return v2t140or145(d("CFPA") < 0);
            }
            return v2t140or145(true);                       // V2T140
        }
        // V2T110: BETA6 = BETA5 (2-LAMBDA) - 1
        double beta6 = beta5 * (2 * ONE_B1 - lambda) - ONE_B17;
        double xlim;
        if (beta6 <= 0) {                                   // V2T115
            if (d("PHI2") >= 0) {
                d("PHI2", -d("PHI2"));
                d("NN1A", TEN_RTE);
            }
            return v2t125();
        }
        xlim = Math.sqrt(128 * beta6);                      // V2T120: SL 7, SQRT
        if (d("PHI2") < 0) {                                // V2T130
            if (d("X(T1)") <= 0) {
                d("XT1MAX", -xlim);                         // V2T135
            } else {
                d("XT1MIN", xlim);
                return v2t140or145(false);
            }
            return v2t125Continue();
        }
        d("XT1MAX", xlim);
        d("XT1MIN", -xlim);
        return v2t125();
    }

    private double v2t125() {
        return v2t125Continue();
    }

    private double v2t125Continue() {
        if (d("X(T1)") <= 0) {                              // V2T125
            return v2t140or145(true);
        }
        return v2t140or145(false);
    }

    /** V2T140 (use X(T1)MIN, +DXMAX) when {@code minSide}, else V2T145 (X(T1)MAX, -DXMAX); then V2T150. */
    private double v2t140or145(boolean minSide) {
        if (minSide) {
            d("X(T1)", d("XT1MIN"));
            d("DX(T1)", d("DXT1MAX"));
        } else {
            d("X(T1)", d("XT1MAX"));
            d("DX(T1)", -d("DXT1MAX"));
        }
        return v2t150();
    }

    /** V2T150: run the X(T1)-DV iterator and test against the desired delta V. */
    private double v2t150() {
        double r = gamdv10();
        if (r != 0) {
            return r;
        }
        double rtedvd = d("RTEDVD");
        if (rtedvd == 0) {
            return 0;                                       // minimum fuel mode: exit
        }
        if (rtedvd - d("DV") < 0) {                         // V2T155: DV exceeds desired
            if (d("NN1A") >= 0) {
                return v2t185();
            }
            if (d("X(T1)") > 0 || d("CFPA") <= 0) {         // V2T160/V2T165 -> V2T300
                return 0;
            }
            d("X(T1)", d("XT1MAX"));
            d("DX(T1)", -d("DXT1MAX"));
            r = gamdv10();
            if (r != 0) {
                return r;
            }
            if (rtedvd - d("DV") < 0) {
                return 0;
            }
        }
        // V2T175
        setFlag("F2RTE", true);
        if (flag("SLOWFLG")) {
            d("XT1MIN", d("X(T1)"));
            d("DX(T1)", d("DXT1MAX"));                      // V2T179
        } else {
            d("XT1MAX", d("X(T1)"));
            d("DX(T1)", -d("DXT1MAX"));
        }
        r = gamdv10();
        if (r != 0) {
            return r;
        }
        if (d("NN1A") < 0) {
            return 0;                                       // V2T300
        }
        return v2t185();
    }

    /** V2T185: refine X(T1) between the bounds then DVCALC; V2T300 returns zero. */
    private double v2t185() {
        double beta7 = Math.abs(d("X(T1)") - d("X(T1),,"));
        if (2 * d("DXT1MAX") - beta7 < 0) {                 // BETA7 larger than 2DX(T1)MAX
            return 0;
        }
        double x = d("X(T1),,");
        if (x - d("XT1MAX") >= 0) {
            x = d("XT1MAX");
        } else if (x - d("XT1MIN") < 0) {
            x = d("XT1MIN");
        }
        d("X(T1)", x);
        dvcalc();
        return 0;                                           // V2T300
    }

    /** GAMDV10: step X(T1) with secant correction until DV matches RTEDVD or bounds close. */
    private double gamdv10() {
        dvcalc();
        double beta8 = d("XT1MAX") - d("XT1MIN");           // B5
        if (beta8 - EPC9RTE < 0) {
            return 0;                                       // GAMDVX: bounds close together
        }
        if (beta8 - d("DXT1MAX") < 0) {                     // GAMDV15
            d("DX(T1)", sign(beta8 / 2, d("DX(T1)")));
        }
        d("NN2", M144RTE);                                  // GAMDV20
        while (true) {
            d("NN2", d("NN2") + ONE_B28);                   // GAMDV25
            if (d("NN2") >= 0) {
                return OCT605;
            }
            double xPrev = d("X(T1)");
            double dvPrev = d("DV");
            d("X(T1),,", xPrev);
            d("X(T1)", xPrev + d("DX(T1)"));
            dvcalc();
            if (!flag("F2RTE")) {                           // fuel critical mode
                if (d("DV") - dvPrev < 0) {
                    // GAMDV33: continue to GAMDV50
                } else {
                    d("DX(T1)", -d("DX(T1)") / 2);          // GAMDV32
                }
            } else {                                        // GAMDV35
                double dverr = d("RTEDVD") - d("DV");
                if (Math.abs(dverr) - EPC10RTE < 0) {
                    return 0;
                }
                double dx = (d("X(T1)") - xPrev) * dverr / (d("DV") - dvPrev);
                if (Double.isNaN(dx) || Double.isInfinite(dx) || Math.abs(dx) - d("DXT1MAX") >= 0) {
                    dx = sign(d("DXT1MAX"), Double.isNaN(dx) ? d("DX(T1)") : dx);  // GAMDV47
                }
                d("DX(T1)", dx);
            }
            // GAMDV50
            double beta9 = d("X(T1)") + 2 * ONE_POINT_ONE_B1 * d("DX(T1)");
            if (beta9 - d("XT1MAX") >= 0) {
                d("DX(T1)", (d("XT1MAX") - d("X(T1)")) / 2);
            } else if (beta9 - d("XT1MIN") < 0) {
                d("DX(T1)", (d("XT1MIN") - d("X(T1)")) / 2);   // GAMDV60
            }
            if (Math.abs(d("DX(T1)")) - EPC9RTE < 0) {      // GAMDV65
                return 0;
            }
        }
    }

    /** DVCALC: PCON, V2(T1)/ and DV = |V2(T1)/ - V(T1)/| for the current X(T1). */
    public void dvcalc() {
        double x = d("X(T1)");
        double pcon = d("THETA2") / (d("THETA1") - x * x / 128);     // B28/26
        d("PCON", pcon);
        double k = d("THETA3") * Math.sqrt(Math.max(pcon, 0));
        double[] v2 = add(scale(vec("UR1/"), x * k), scale(vec("UH/"), k));
        vec("V2(T1)/", v2);                                 // B7/B5
        d("DV", abval(sub(v2, vec("V(T1)/"))));
    }

    /** XT1LIM: X(T1) limit for apse radius MA: SQRT(BETA5 (MA-RT)/(MA-RC) - 1), or 0. */
    public double xt1lim(double ma) {
        double ratio = (ma - d("R(T1)")) / (ma - d("RCON"));
        double beta10 = d("BETA5") * ratio - ONE_B1;
        if (Double.isNaN(beta10) || Double.isInfinite(beta10)) {
            beta10 = POSMAX2;                               // POSMAX if overflow
        }
        if (beta10 < 0) {
            return 0;                                       // XT1LIM5
        }
        return Math.sqrt(beta10);
    }
}
