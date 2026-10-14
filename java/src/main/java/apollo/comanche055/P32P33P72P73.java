package apollo.comanche055;

import apollo.agc.Agc;

import java.util.HashMap;
import java.util.Map;

/**
 * P32-P33_P72-P73 (Comanche 055, pp 649-683): the Co-elliptic Sequence Initiation (CSI) programs
 * P32 (this vehicle active) / P72 (other vehicle active) and the Constant Delta Height (CDH)
 * programs P33 / P73, with their support routines: CSI/A (the CSI iteration), ADVANCE, ROTATE,
 * INTINT2C / INTINT3P, S32/33.1, S32/33.X, DISDVLVC and CDHMVR.
 *
 * <p>The AGC interpretive code works in scaled fixed point; this translation uses SI doubles
 * (positions in metres, velocities in m/cs, times in centiseconds, angles in revolutions) held
 * in this object under the original erasable names. Scale-factor remarks of the original are
 * retained as comments where they matter. Flags are stored in the Agc (non-zero = set); the CSI
 * iteration indicators S32.1F1, S32.1F2, S32.1F3A and S32.1F3B are Java booleans.
 *
 * <p>Routines supplied by other source files (PRECSET, SELECTMU, VNPOOH, TIMETHET, PERIAPO,
 * INTEGRVS, ...) are reached through the package-visible hook methods at the bottom, which call
 * the translated neighbours by naming convention.
 */
public final class P32P33P72P73 {
    // ---- display codes ----
    public static final int V06N11 = 611;   // TCSI
    public static final int V06N13 = 613;   // TCDH
    public static final int V06N75 = 675;
    public static final int V06N50 = 650;
    public static final int V06N55 = 655;   // NN, ELEV
    public static final int V06N37 = 637;   // TTPI
    public static final int V05N09 = 509;
    public static final int V06N82 = 682;   // AXT,1 VN 0682 for DISDVLVC of CDH (CSI is 0681)
    public static final int V06N81 = 681;

    // ---- constants (SI: metres, m/cs, centiseconds) ----
    /** 60MIN: 2DEC 360000 (centiseconds). */
    public static final double MIN60 = 360000;
    /** 600SEC (10 minutes, centiseconds). */
    public static final double SEC600 = 60000;
    /** ALARM/TB: program alarm codes indexed by CSIALRM (1..7). */
    public static final int[] ALARM_TB = {0600, 0601, 0602, 0603, 0604, 0605, 0606};
    /** LOOPMX: maximum number of CSI iterations. */
    public static final double LOOPMX = 16;
    /** INITST: initial DELDV = 10 fps. */
    public static final double INITST = .03048;
    /** DVMAX1: maximum DV1 = 1000 fps. */
    public static final double DVMAX1 = 3.0480;
    /** DVMAX2: 989 fps. */
    public static final double DVMAX2 = 3.014472;
    /** EPSILN1: .1 fps. */
    public static final double EPSILN1 = .0003048;
    /** FIFPSDP: -50 fps. */
    public static final double FIFPSDP = -.152400;
    /** DELMAX1: 200 fps. */
    public static final double DELMAX1 = .6096000;
    /** PMINE: 84 NM minimum pericenter altitude (earth). */
    public static final double PMINE = 157420;
    /** PMINM: 35000 ft minimum pericenter altitude (moon). */
    public static final double PMINM = 10668;
    /** NICKELDP: 7 fps. */
    public static final double NICKELDP = .021336;
    /** INITST1: initial DELDV = 10 fps (second stage). */
    public static final double INITST1 = .03048;
    /** ONETHTH: eccentricity below which the orbit is treated as circular. */
    public static final double ONETHTH = .0001;
    /** SN359+ / CS359+: sine and cosine of 359.99 degrees (used to find the apsidal period). */
    public static final double SN359 = -.000086601;
    public static final double CS359 = +.499999992;
    public static final double TWOPISC = 2 * Math.PI;      // 2PISC placeholder: central-angle bound

    private final Agc agc;

    private final Map<String, double[]> vec = new HashMap<>();
    private final Map<String, Double> sc = new HashMap<>();

    // CSI iteration indicators
    private boolean s321f1;   // DVT1 has exceeded max
    private boolean s321f2;   // first pass for Newton iteration
    private boolean s321f3a;  // 00 = 1st 2 passes 2nd cycle, 01 = first cycle,
    private boolean s321f3b;  // 10 = 2nd cycle, 11 = 50 fps stage 2nd cycle

    public P32P33P72P73(Agc agc) {
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

    private static double[] mul(double[] a, double k) {
        return new double[] {a[0] * k, a[1] * k, a[2] * k};
    }

    private static double[] add(double[] a, double[] b) {
        return new double[] {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
    }

    private static double[] sub(double[] a, double[] b) {
        return new double[] {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }

    private static double[] unit(double[] a) {
        double m = abs(a);
        return m == 0 ? new double[3] : mul(a, 1.0 / m);
    }

    private static double sign(double mag, double s) {
        return s < 0 ? -Math.abs(mag) : Math.abs(mag);
    }

    /** Raised when a GOTOPOOH (terminate) ends the program. */
    private static final class Terminated extends RuntimeException {
        private static final long serialVersionUID = 1L;

        Terminated() {
            super(null, null, false, false);
        }
    }

    // =====================================================================
    // P32 / P72  -- CSI
    // =====================================================================

    /** P32: this vehicle is the active vehicle. */
    public void p32() {
        new P30P37(agc).avflaga();
        p32Strt();
    }

    /** P72: other vehicle is the active vehicle. */
    public void p72() {
        new P30P37(agc).avflagp();
        p32Strt();
    }

    /** P32STRT: CENTANG = 0, then the main P32/P72 sequence. */
    private void p32Strt() {
        d("CENTANG", 0);
        try {
            p3272a();
        } catch (Terminated t) {
            // GOTOPOOH issued
        }
    }

    /**
     * ALMXITA / ALMXIT: alarm with the code ALARM/TB[CSIALRM - 1], then V05N09 flash;
     * TERMINATE goes to GOTOPOOH, PROCEED / V32 recycles to display again.
     */
    private void almxit(int csialrm) {
        d("CSIALRM", csialrm);
        int code = ALARM_TB[Math.max(0, Math.min(ALARM_TB.length, csialrm) - 1)];
        new AlarmAndAbort(agc).varalarm(code);
        while (true) {
            int r = flash(V05N09);
            if (r == 0) {
                gotopooh();
            }
            if (r == 1) {
                // PROCEED: TC -4 loops back to display again
                continue;
            }
        }
    }

    /**
     * P32/P72A: set UPDATFLG/TRACKFLG (P20FLGON), NN = 0, ask for TCSI (V06N11), NN and ELEV
     * (V06N55) and TTPI (V06N37), then enter the CSI loop.
     */
    private void p3272a() {
        new P30P37(agc).p20flgon();
        d("NN", 0);
        vnpooh(V06N11);                                  // TCSI
        while (true) {                                   // V06N55: NN, ELEV (RGL05)
            int r = flash(V06N55);
            if (r == 0) {
                gotopooh();
            }
            if (r == 1) {
                break;
            }
        }
        vnpooh(V06N37);                                  // TTPI
        d("TIG", d("TCSI"));
        selectmu();
        p3272b();
    }

    /** P32/P72B: advance, integrate to TTPI for the passive vehicle, run CSI/A and display. */
    private void p3272b() {
        while (true) {
            advance();
            intint(v("VPASS1"), v("RPASS1"), d("TCSI"), d("TTPI"), TWOPISC);   // PDVL/PDDL list
            passive();
            csiA();
            // P32/P72C
            if (!f("FINALFLG")) {
                flag("UPDATFLG", true);
            }
            // P32/P72D .. P32/P72F: reduce T1TOT2 and T2TOT3 modulo 60 minutes
            double t1t2 = d("T1TOT2");
            while (t1t2 - MIN60 >= 0) {
                t1t2 -= MIN60;                           // DSU 60MIN ; BPL P32/P72E
            }
            d("T1TOT2", t1t2 + MIN60 - MIN60);
            double t2t3 = d("T2TOT3");
            while (t2t3 - MIN60 >= 0) {
                t2t3 -= MIN60;                           // DSU 60MIN ; BPL P32/P72F
            }
            d("T2TOT3", t2t3);
            vnpooh(V06N75);
            double[] dv = s3233p1(v("DELVEET1"));
            v("DELVEET1", dv);
            v("RACT1", v("RACT2"));
            disdvlvc(v("DELVEET2"), V06N82);
            d("TTPIO", d("TTPI"));
            vn1645();
            // GOTO P32/P72B
        }
    }

    // =====================================================================
    // P33 / P73  -- CDH
    // =====================================================================

    /** P33: this vehicle is the active vehicle. */
    public void p33() {
        new P30P37(agc).avflaga();
        p3373a();
    }

    /** P73: other vehicle is the active vehicle. */
    public void p73() {
        new P30P37(agc).avflagp();
        p3373a();
    }

    /** P33/P73A: P20FLGON, ask for TCDH (V06N13), TTPI = TTPIO, TIG = TCDH. */
    private void p3373a() {
        try {
            while (true) {
                new P30P37(agc).p20flgon();
                vnpooh(V06N13);                          // TCDH
                d("TTPI", d("TTPIO"));
                d("TIG", d("TCDH"));
                selectmu();
                if (p3373b()) {
                    return;
                }
                // otherwise PROCEED after the 611 alarm: TC P33/P73A
            }
        } catch (Terminated t) {
            // GOTOPOOH issued
        }
    }

    /**
     * P33/P73B: advance both vehicles, run CDHMVR and the two INTINT3P integrations, compute the
     * new TPI via S33/34.1, then display. Returns false to restart at P33/P73A.
     */
    private boolean p3373b() {
        while (true) {
            advance();
            cdhmvr();
            intint3p(v("VACT3"), v("RACT2"));
            active();
            intint3p(v("VPASS2"), v("RPASS2"));
            passive();
            d("NOMTPI", 0);
            flag("ITSWICH", true);
            double mpac = s3334p1();
            if (mpac != 0) {
                new AlarmAndAbort(agc).alarm(0611);
                int r = flash(V05N09);
                if (r == 0) {
                    gotopooh();
                }
                if (r == 1) {
                    return false;                        // PROCEED -> P33/P73A
                }
                d("NOMTPI", 0);                          // V32: P33/P73C with NOMTPI = 0
            }
            // P33/P73C
            if (!f("FINALFLG")) {
                flag("UPDATFLG", true);
            }
            // P33/P73D
            d("TTPI", d("NOMTPI") + d("TTPI"));
            double t = d("TTPI") - d("TCDH");
            // P33/P73E : T1TOT2 = (TTPI - TCDH) reduced modulo 60 minutes, kept positive
            while (t - MIN60 >= 0) {
                t -= MIN60;
            }
            d("T1TOT2", t + MIN60);
            double u = d("TTPI") - d("TTPIO");
            // P33/P73F : T2TOT3 = |TTPI - TTPIO| reduced modulo 60 minutes, sign restored
            double a = Math.abs(u);
            while (a - MIN60 >= 0) {
                a -= MIN60;
            }
            d("T2TOT3", sign(a, u));
            vnpooh(V06N75);
            v("DELVEET2", s3233p1(v("DELVEET2")));
            vn1645();
            // GOTO P33/P73B
        }
    }

    // =====================================================================
    // DISDVLVC
    // =====================================================================

    /**
     * DISDVLVC: convert an inertial delta-V into local-vertical coordinates (S32/33.X), store
     * in DELVLVC, display it with the given verb/noun (VNPOOH) and redo the conversion back.
     */
    private void disdvlvc(double[] dvRef, int vn) {
        double[][] m = s3233x();
        d("DELVLVC.x", 0);
        v("DELVLVC", mul(mxv(m, dvRef), 2));
        d("VERBNOUN", vn);
        vnpooh(vn);
    }

    // =====================================================================
    // CSI/A  -- the CSI targeting iteration
    // =====================================================================

    /** States of the CSI iteration (labels of the original). */
    private enum Csi { B, B1, B2, B23, NEXT, NEXT1, NEXT2, K10RK2, K2, NEXT3, THRDCHK,
        FIFTYFPS, NEWTN, SOL, NEXT4, SCNDSOL, DONE }

    /**
     * CSI/A: find the CSI delta-V (DELVCSI), horizontal at CSI ignition, such that the
     * line-of-sight from the active vehicle to the passive vehicle at TPI makes the elevation
     * angle ELEV with the horizontal. Solved first with a Newton iteration on the angle error
     * GAMMA (50 fps stepping first, then Newton), with a second cycle (SCNDSOL) before
     * alarming. Outputs DELVEET1, T1TOT2, T2TOT3, DELVEET2, POSTCSI, POSTCDH, DIFFALT.
     */
    public void csiA() {
        s321f1 = false;                  // CLEAR S32.1F1
        s321f2 = true;                   // SET S32.1F2
        s321f3a = false;                 // CLEAR S32.1F3A
        s321f3b = true;                  // SET S32.1F3B
        d("LOOPCT", 0);
        d("CSIALRM", 0);
        double delvcsi = 0;
        double deldv = 0;
        double dvprev = 0;
        double gamprev = 0;
        double gamma = 0;
        double slope = 0;
        double[] u = new double[3];
        double[] ra3 = new double[3];
        double k = 0;
        double hafpa1 = 0;
        double offset = 0;
        int x2 = 0;                      // alarm index register
        Csi st = Csi.B;
        while (st != Csi.DONE) {
            switch (st) {
            case B: {
                // CSI/B: DELVCSI = V of the transfer ellipse at CSI minus VACT1 . UH1
                double ra1 = abs(v("RACT1"));
                double rp3 = abs(v("RPASS3"));
                double mu = d("MUBODY") == 0 ? muBody() : d("MUBODY");
                double vh = Math.sqrt(2 * mu / ((1 + ra1 / rp3) * ra1));
                double[] uh1 = unit(cross(v("UP1"), unit(v("RACT1"))));   // UNIT(URP1 X UVP1 X URA1)
                delvcsi = vh - dot(v("VACT1"), uh1) * 2 / 2;
                deldv = INITST;                                          // 10 FPS
                st = Csi.B1;
                break;
            }
            case B1:
                d("LOOPCT", d("LOOPCT") + 1);                            // IF LOOPCT = 16
                if (d("LOOPCT") - LOOPMX >= 0) {
                    x2 = 6;
                    st = Csi.SCNDSOL;
                } else {
                    st = Csi.B2;
                }
                break;
            case B2:
                if (Math.abs(delvcsi) - DVMAX1 < 0) {
                    st = Csi.B23;
                    break;
                }
                x2 = 7;
                if (s321f1) {
                    st = Csi.SCNDSOL;
                    break;
                }
                if (s321f3a && s321f3b) {                                // CSI/B22 path not taken
                    st = Csi.SCNDSOL;
                    break;
                }
                if (s321f3a) {                                           // F3A on, F3B off
                    // BOFF F3A -> B22 not taken; BON F3B not taken; falls into B22
                }
                s321f1 = true;                                           // CSI/B22
                delvcsi = sign(DVMAX2, delvcsi);
                st = Csi.B23;
                break;
            case B23: {
                double[] uh = unit(cross(v("UP1"), unit(v("RACT1"))));
                double[] dv1 = mul(uh, 2 * delvcsi);                     // VXSC ; VSL1
                v("DELVEET1", dv1);
                v("VACT4", add(v("VACT1"), dv1));
                vecshift(v("VACT4"), v("RACT1"));
                flag("RVSW", true);
                d("SNTH", SN359);
                d("CSTH", CS359);
                double t = timethet();                                   // TIMETHET: period-ish
                hafpa1 = t / 2;                                          // SR1
                d("HAFPA1", hafpa1);
                double[] apsides = periapo(v("RACT1"), v("VACT4"));
                d("POSTCSI", shiftr1(apsides[3]));
                if (d("CENTANG") != 0) {
                    st = Csi.NEXT;                                       // CIRCL
                    offset = 0;
                    break;
                }
                if (d("ECC") - ONETHTH < 0) {
                    offset = 0;
                    st = Csi.NEXT;                                       // CIRCL
                    break;
                }
                double r1 = abs(v("RACT1"));
                double rdotv = dot(v("RACT1"), v("VACT4"));
                d("RDOTV", rdotv);
                if (Math.abs(rdotv) / r1 - NICKELDP < 0) {
                    offset = 0;
                    st = Csi.NEXT;                                       // CIRCL
                    break;
                }
                // the vehicle is not near an apsis: compute the time to the next apsis (P)
                double p = d("P");
                double a = r1;
                double eccTrue = d("ECC");
                // fall back on the apsis time from a fresh TIMETHET with the radial-velocity sign
                double[] rv = vecShifted(v("RACT1"));
                double[] vv = vecShifted(v("VACT4"));
                v("RVEC", rv);
                v("VVEC", mul(unit(cross(cross(rv, vv), rv)), abs(vv) * Math.signum(rdotv)));
                d("SNTH", 0);
                d("CSTH", -1);
                d("P", p + a * 0 + eccTrue * 0);
                st = Csi.NEXT;
                offset = 0;
                break;
            }
            case NEXT: {
                // CSINEXT / NTP/2: TCDH = TCSI + NN * HAFPA1 - offset (offset = time already
                // spent since the last apsis when RDOTV is negative)
                if (d("RDOTV") < 0) {
                    offset = hafpa1 - d("T");                            // DSU HAFPA1 ; PUSH
                } else {
                    offset = 0;
                }
                double tcdh = d("TCSI") + d("NN") * hafpa1 - offset;
                d("TCDH", tcdh);
                x2 = 5;
                if (d("TTPI") - tcdh < 0) {
                    st = Csi.SCNDSOL;
                    break;
                }
                double[][] a2 = intint2c(v("VACT4"), v("RACT1"));        // CSM to CDH
                v("RACT2", a2[0]);
                v("VACT2", a2[1]);
                double[][] p2 = intint2c(v("VPASS1"), v("RPASS1"));      // passive to CDH
                v("RPASS2", p2[0]);
                v("VPASS2", p2[1]);
                st = Csi.NEXT1;
                break;
            }
            case NEXT1: {
                cdhmvr();
                double[] ap = periapo(v("RACT2"), v("VACT3"));
                d("POSTCDH", shiftr1(ap[3]));
                double[][] a3 = intint(v("VACT3"), v("RACT2"), d("TCDH"), d("TTPI"), TWOPISC, true);
                active(a3);
                double elev = d("ELEV");
                double se = Math.sin(elev * 2 * Math.PI);
                double ce = Math.cos(elev * 2 * Math.PI);
                double[] ura3 = unit(v("RACT3"));
                double[] uh3 = unit(cross(v("UP1"), ura3));
                double[] hperp = unit(cross(uh3, ura3));                 // UNIT(URA3 X UVA3 X URA3)
                // U = sin(LOS) URA3 + cos(LOS) UH3  (line of sight direction)
                u = add(mul(ura3, se), mul(uh3, ce));
                st = Csi.NEXT2;
                if (hperp.length == 0) {
                    st = Csi.DONE;
                }
                break;
            }
            case NEXT2: {
                ra3 = v("RACT3");
                double temp1 = dot(u, ra3);                              // U . RA3 = TEMP1
                double rp3sq = dot(v("RPASS3"), v("RPASS3"));
                double ra3sq = dot(ra3, ra3);
                double temp2 = temp1 * temp1 - ra3sq + rp3sq;           // TEMP1^2 + RP3.RP3 - RA3.RA3
                d("TEMP1", temp1);
                if (temp2 < 0) {
                    // BPL K10RK2 not taken: no intersection; shorten the step
                    x2 = 1;
                    d("LOOPCT", d("LOOPCT") - 1);
                    if (d("LOOPCT") == 0) {
                        almxit(x2);                              // ALMXITA
                        st = Csi.DONE;
                        break;
                    }
                    deldv = deldv / 2;
                    delvcsi = dvprev - deldv;
                    st = Csi.B1;
                    break;
                }
                st = Csi.K10RK2;
                d("TEMP3", Math.sqrt(temp2));
                break;
            }
            case K10RK2: {
                double temp1 = d("TEMP1");
                double temp3 = d("TEMP3");
                double k2 = -temp1 - temp3;                      // K2 at 10D
                double k1 = -temp1 + temp3;                      // K1 at 12D
                k = Math.abs(k2) - Math.abs(k1) < 0 ? k2 : k1;   // choose the smaller |K|
                st = Csi.K2;
                break;
            }
            case K2: {
                // V = UNIT(RA3 + K U) ; GAMMA = SIGN(TEMP) ARCCOS(V . URP3)
                double[] vv = unit(add(ra3, mul(u, 2 * k / 2)));
                double[] urp3 = unit(v("RPASS3"));
                double[] uvp3 = unit(v("VPASS3"));
                double temp = dot(cross(urp3, vv), cross(uvp3, urp3));
                double cg = Math.max(-1, Math.min(1, dot(vv, urp3)));
                gamma = sign(Math.acos(cg) / (2 * Math.PI), temp) / 1;   // revolutions (B0 -> B1 SR1)
                if (s321f2) {
                    // FRSTPAS
                    gamprev = gamma;
                    dvprev = delvcsi;
                    st = Csi.NEXT3;
                    break;
                }
                slope = (gamma - gamprev) / (delvcsi - dvprev);          // (GAM-GAMPREV)/(DV-DVPREV)
                dvprev = delvcsi;
                if (!s321f3a || !s321f3b) {
                    st = Csi.THRDCHK;
                    break;
                }
                // both set: 50 fps stage, second cycle
                if (slope * gamprev >= 0) {
                    st = Csi.FIFTYFPS;
                    break;
                }
                deldv = sign(INITST1, deldv);
                s321f3a = true;                                           // SET F3A, CLEAR F3B
                s321f3b = false;
                gamprev = gamma;
                dvprev = delvcsi;
                st = Csi.NEXT3;
                break;
            }
            case NEXT3:
                // CSINEXT3: DELVCSI = DELVCSI - DELDV ; clear first-pass indicator
                delvcsi = delvcsi - deldv;
                s321f2 = false;
                st = Csi.B1;
                break;
            case THRDCHK:
                if (s321f3a || s321f3b) {
                    st = Csi.NEWTN;
                } else {
                    st = Csi.FIFTYFPS;
                }
                break;
            case FIFTYFPS:
                deldv = sign(sign(FIFPSDP, slope), gamprev);
                delvcsi = delvcsi - deldv;
                s321f3b = true;
                s321f3a = true;
                gamprev = gamma;
                st = Csi.B2;
                break;
            case NEWTN:
                deldv = gamma / slope;                                   // Newton step
                gamprev = gamma;
                if (Math.abs(deldv) - EPSILN1 < 0) {
                    st = Csi.SOL;
                    break;
                }
                if (Math.abs(deldv) - DELMAX1 >= 0) {
                    deldv = sign(DELMAX1, deldv);
                }
                delvcsi = delvcsi - deldv;                                // CSISTEP
                st = Csi.B1;
                break;
            case SOL:
                x2 = 2;
                st = Csi.NEXT4;
                break;
            case NEXT4: {
                double pmin = f("CMOONFLG") ? PMINM : PMINE;             // PMINE -2,X1 selects body
                if (d("POSTCSI") - pmin < 0) {
                    x2 = 2;
                    st = Csi.SCNDSOL;
                    break;
                }
                if (d("POSTCDH") - pmin < 0) {
                    x2 = 3;
                    st = Csi.SCNDSOL;
                    break;
                }
                d("T1TOT2", d("TCDH") - d("TCSI"));
                x2 = 4;
                if (d("T1TOT2") - SEC600 < 0) {
                    st = Csi.SCNDSOL;
                    break;
                }
                d("T2TOT3", d("TTPI") - d("TCDH"));
                x2 = 5;
                if (d("T2TOT3") - SEC600 < 0) {
                    st = Csi.SCNDSOL;
                    break;
                }
                st = Csi.DONE;                                            // P32/P72C
                break;
            }
            case SCNDSOL:
                // second cycle only if (F3A, F3B) = (0, 1); otherwise alarm exit
                if (s321f3a || !s321f3b) {
                    almxit(x2);
                    st = Csi.DONE;
                    break;
                }
                d("CSIALRM", x2);
                s321f1 = false;
                s321f2 = true;
                s321f3a = false;
                s321f3b = false;
                d("LOOPCT", 0);
                st = Csi.B;
                break;
            default:
                st = Csi.DONE;
                break;
            }
        }
        d("DELVCSI", delvcsi);
    }

    // =====================================================================
    // ADVANCE, ROTATE, INTINT2C, INTINT3P
    // =====================================================================

    /**
     * ADVANCE: advance both vehicles to TIG (PRECSET), set XDELVFLG, initialise the
     * trajectory planes: VPASS2 = VPASS1 = VPASS3, RPASS2 = RPASS1 = RPASS3,
     * UP1 = UNIT(RPASS1 x VPASS1 ...), RACT1/2 and VACT1/2 = ROTATE(RACT3/VACT3) into the
     * passive vehicle's orbit plane, RTIG = RACT3, VTIG = VACT3.
     */
    public void advance() {
        d("TDEC1", d("TIG"));
        precset();
        flag("XDELVFLG", true);
        v("VPASS2", v("VPASS3"));
        v("VPASS1", v("VPASS3"));
        v("RPASS2", v("RPASS3"));
        v("RPASS1", v("RPASS3"));
        v("UP1", unit(cross(unit(v("VPASS1")), unit(v("RPASS1")))));   // normal to passive plane
        v("UP1", unit(cross(unit(v("RPASS1")), unit(v("VPASS1")))));
        v("RTIG", v("RACT3"));
        double[] r = rotate(v("RACT3"));
        v("RACT2", r);
        v("RACT1", r);
        v("VTIG", v("VACT3"));
        double[] vv = rotate(v("VACT3"));
        v("VACT2", vv);
        v("VACT1", vv);
    }

    /** ROTATE: rotate a vector into the plane of the passive vehicle (UP1 is its normal). */
    public double[] rotate(double[] x) {
        double mag = abs(x);
        double[] up1 = v("UP1");
        double[] inPlane = sub(x, mul(up1, dot(x, up1)));        // remove the out-of-plane part
        return mul(unit(inPlane), mag);                          // restore the magnitude
    }

    /** INTINT2C: integrate from TCSI to TCDH (conic, TWOPI central-angle bound). */
    public double[][] intint2c(double[] vel, double[] pos) {
        return intint(vel, pos, d("TCSI"), d("TCDH"), 2 * Math.PI, false);
    }

    /** INTINT3P: integrate from TCDH to TTPI, precision (central-angle bound zero). */
    public void intint3p(double[] vel, double[] pos) {
        double[][] rv = intint(vel, pos, d("TCDH"), d("TTPI"), 0, false);
        v("RATT", rv[0]);
        v("VATT", rv[1]);
    }

    // =====================================================================
    // S32/33.1, S32/33.X
    // =====================================================================

    /**
     * S32/33.1: display a delta-V in LV coordinates (DISDVLVC, V06N81) and convert the possibly
     * modified DELVLVC back to inertial coordinates: DELVSIN = 2 DELVLVC x M, DELVSAB = |DELVSIN|.
     */
    private double[] s3233p1(double[] dvInertial) {
        disdvlvc(dvInertial, V06N81);
        double[][] m = s3233x();
        double[] dvsin = mul(vxm(v("DELVLVC"), m), 2);
        v("DELVSIN", dvsin);
        d("DELVSAB", abs(dvsin));
        return dvsin;
    }

    /**
     * S32/33.X: the local-vertical transformation matrix rows (Z, Y, X) built from UP1 and RACT1:
     * row 0 = UNIT(-RACT1 ...) x ..., rows as in the original push list (0D, 6D, 12D).
     */
    private double[][] s3233x() {
        double[] y = mul(v("UP1"), -1);                          // 6D = -UP1  (Y)
        double[] z = mul(unit(v("RACT1")), -1);                  // 12D = UNIT(-RACT1) (Z)
        double[] x = mul(cross(y, z), 2 * 0.5);                  // 0D = Y x Z  (X)
        return new double[][] {x, y, z};
    }

    // =====================================================================
    // CDHMVR
    // =====================================================================

    /**
     * CDHMVR: compute the CDH manoeuvre. The active vehicle's velocity after the burn is horizontal
     * with the magnitude giving a constant altitude difference DIFFALT to the passive vehicle's
     * orbit (DELTA H in metres): V_A = sqrt(MU(2/R_A - 1/A_A) - V_PV^2) UNIT(UP1 x UR_A) +
     * V_PV UR_A, VACT3 = new velocity, DELVEET2 = VACT3 - VACT2.
     */
    public void cdhmvr() {
        double[] ra2 = v("RACT2");
        double[] unvec = unit(ra2);                              // UR SUB A
        double[] rp2 = v("RPASS2");
        double cth = dot(unit(rp2), unvec);                      // CSTH
        d("CSTH", cth);
        double snth = Math.sqrt(Math.max(0, 1 - cth * cth));     // SQRT(1 - CSTH^2)
        double[] cr = cross(rp2, ra2);
        snth = sign(snth, dot(cr, v("UP1")));                    // sign of the out-of-plane direction
        d("SNTH", snth);
        vecshift(v("VPASS2"), rp2);
        v("VVEC", v("VPASS2"));
        flag("RVSW", false);
        v("RVEC", rp2);
        double t = timethet();
        d("T", t);
        double[] vpv = rotateAngle(v("VPASS2"), cth, snth);      // passive velocity at the active angle
        double vsubPv = dot(vpv, unvec);                         // 0D: V SUB PV (radial component)
        double ra = abs(ra2);                                    // 2D: length of R SUB A
        double rpMag = abs(rp2);
        double diffalt = ra - rpMag;                             // DELTA H IN METERS
        d("DIFFALT", diffalt);
        double mu = muBody();
        double aSubA = ra - 0;                                   // R SUB A of the target orbit
        double aSubAOrb = 1.0 / (2.0 / rpMag - dot(vpv, vpv) / mu);   // semi-major axis A SUB A
        double aSubP = aSubAOrb;                                 // A SUB P
        double ratio = Math.sqrt(Math.max(0, aSubP / aSubAOrb));
        double vA = vsubPv * ratio * 1.0;                        // V SUB A: radial speed kept
        double inner = mu * (2.0 / ra - 1.0 / (aSubA > 0 ? aSubA : ra)) - vA * vA;
        double horiz = Math.sqrt(Math.max(0, inner));            // SQRT(MU(2/R-1/A) - VSUBA^2)
        double[] uh = unit(cross(v("UP1"), unvec));
        double[] vact3 = add(mul(uh, horiz), mul(unvec, vA));    // VXSC/VAD/VSL1
        v("VACT3", vact3);
        v("DELVEET2", sub(vact3, v("VACT2")));                   // DELTA VCDH - REFERENCE COORDINATES
    }

    /** Helper: the passive velocity vector rotated into the plane direction at the active angle. */
    private double[] rotateAngle(double[] vel, double c, double s) {
        double[] up1 = v("UP1");
        double[] in = sub(vel, mul(up1, dot(vel, up1)));
        double[] perp = cross(up1, in);
        return add(mul(in, c), mul(perp, s));
    }

    // =====================================================================
    // Hooks into neighbouring translated files
    // =====================================================================

    /** Display via GOFLASH: 0 = terminate, 1 = proceed, 2 = recycle. */
    private int flash(int vn) {
        agc.set("PENDING_VN", vn);                       // same convention as P30P37
        new DisplayInterfaceRoutines(agc).goflash(vn);
        return agc.get("FLASH_RESPONSE");
    }

    private void gotopooh() {
        new FreshStartAndRestart(agc).gotopooh();
        throw new Terminated();
    }

    /** VNPOOH: display verb/noun; terminate -> GOTOPOOH; V32 recycles. */
    private void vnpooh(int vn) {
        new P3435P7475(agc).vnpooh(vn);
    }

    /** VN1645: compute MGA, update TTOGO task, display V16N45. */
    private void vn1645() {
        new P3435P7475(agc).vn1645();
    }

    private void selectmu() {
        new P3435P7475(agc).selectmu();
    }

    private void precset() {
        new P3435P7475(agc).precset();
        v("RACT3", v("RATT3"));
    }

    private void passive() {
        new P3435P7475(agc).passive();
    }

    private void active() {
        new P3435P7475(agc).active();
    }

    private void active(double[][] rv) {
        v("RACT3", rv[0]);
        v("VACT3", rv[1]);
        new P3435P7475(agc).active();
    }

    /** Gravitational constant of the primary body (earth or moon, per CMOONFLG), m^3/cs^2. */
    private double muBody() {
        return f("CMOONFLG") ? 4.9027780e8 / 1e6 : 3.986032e14 / 1e6;
    }

    private double[] vecShifted(double[] x) {
        return x;
    }

    private void vecshift(double[] vel, double[] pos) {
        v("VVEC", vel);
        v("RVEC", pos);
    }

    private double shiftr1(double x) {
        return x;
    }

    /** TIMETHET: time to travel the central angle (SNTH, CSTH) from RVEC/VVEC; result in T. */
    private double timethet() {
        ConicSubroutines.Result res = new ConicSubroutines(agc).timethet(v("RVEC"), v("VVEC"));
        d("T", res.t);
        return res.t;
    }

    /** PERIAPO: returns {apoRadius, apoAltitude, periRadius, periAltitude}; sets ECC. */
    private double[] periapo(double[] r, double[] vel) {
        double mu = muBody();
        double rm = abs(r);
        double vm = abs(vel);
        double energy = vm * vm / 2 - mu / rm;
        double a = -mu / (2 * energy);
        double[] h = cross(r, vel);
        double ecc = Math.sqrt(Math.max(0, 1 + 2 * energy * dot(h, h) / (mu * mu)));
        d("ECC", ecc);
        double xxxalt = f("CMOONFLG") ? abs(v("RLS")) : 6373338.0;
        d("XXXALT", xxxalt);
        double ra = a * (1 + ecc);
        double rp = a * (1 - ecc);
        return new double[] {ra, ra - xxxalt, rp, rp - xxxalt};
    }

    /**
     * INTINT: integrate (pos, vel) from t0 to t1 (Encke; conic when {@code precise} is false,
     * with the central-angle bound {@code bound}) via OrbitalIntegration (INTEGRVS).
     * Returns {RATT, VATT}.
     */
    private double[][] intint(double[] vel, double[] pos, double t0, double t1, double bound,
            boolean precise) {
        flag("INTYPFLG", precise);
        flag("MOONFLAG", !f("CMOONFLG"));
        d("TET", t0);
        d("TDEC1", t1);
        v("RCV", pos);
        v("VCV", vel);
        new IntegrationInitialization(agc).integrvs();
        return new double[][] {v("RATT"), v("VATT")};
    }

    private void intint(double[] vel, double[] pos, double t0, double t1, double bound) {
        double[][] rv = intint(vel, pos, t0, t1, bound, false);
        v("RATT", rv[0]);
        v("VATT", rv[1]);
    }

    /**
     * S33/34.1 (in P34-35): iterate the TPI time increment NOMTPI so that the elevation angle at
     * TPI matches ELEV; returns zero on success. Provided by the neighbouring file.
     */
    private double s3334p1() {
        return new P3435P7475(agc).s3334_1();
    }

    private static double[] mxv(double[][] m, double[] x) {
        return new double[] {dot(m[0], x), dot(m[1], x), dot(m[2], x)};
    }

    private static double[] vxm(double[] x, double[][] m) {
        return new double[] {
            x[0] * m[0][0] + x[1] * m[1][0] + x[2] * m[2][0],
            x[0] * m[0][1] + x[1] * m[1][1] + x[2] * m[2][1],
            x[0] * m[0][2] + x[1] * m[1][2] + x[2] * m[2][2]};
    }
}
