package apollo.luminary099;

import apollo.agc.Agc;

/**
 * P32-P35 / P72-P75 (Luminary 099), first part: the Co-elliptic Sequence Initiation program
 * (P32 active vehicle / P72 passive vehicle), the Constant Delta Height program (P33 / P73) and
 * their subroutines AVFLAGA/AVFLAGP/P20FLGON, DISDVLVC, CSI/A, ADVANCE, ROTATE, INTINT2C,
 * INTINT3P, S32/33.1, S32/33.X, CDHMVR and COMPTGO.
 *
 * <p>The original is interpretive code on scaled fixed-point vectors. Here vectors are kept in the
 * erasable table (fractions, via {@link VecOps}) and converted to physical units (metres,
 * metres/centisecond, centiseconds) for the arithmetic; the scale-factor comments of the original
 * are not repeated except where useful. Times are double-precision centisecond words stored
 * with {@link VecOps#getDpRaw}. Interpretive switches live in the erasable symbol table.
 *
 * <p>Display responses use the convention 0 = terminate, 1 = proceed, 2 = recycle. A terminate
 * (GOTOPOOH) unwinds the program by throwing {@link Terminate}.
 */
public final class P32P35P72P75 {
    // ---- verb/noun codes (VN macro) ----
    static final int V06N11 = 6 * 128 + 11; // TCSI
    static final int V06N13 = 6 * 128 + 13; // TCDH
    static final int V06N75 = 6 * 128 + 75;
    static final int V06N55 = 6 * 128 + 55; // NN, ELEV (RGLOS)
    static final int V06N37 = 6 * 128 + 37; // TTPI
    static final int V06N81 = 6 * 128 + 81; // DELVLVC at CSI
    static final int V06N82 = 6 * 128 + 82; // DELVLVC at CDH
    static final int V05N09 = 5 * 128 + 9;

    // ---- constants (CSI/CDH banks) ----
    /** SN359+: sin of 359 deg-ish transfer used to get a full-period time (scaled +1). */
    static final double SN359 = -.000086601 * 2;
    /** CS359+: cos (scaled +1). */
    static final double CS359 = .499999992 * 2;
    static final double P30ZERO = 0;
    /** 60MIN: 360000 centiseconds. */
    static final double MIN60 = 360000;
    static final double TWOPI = 2 * Math.PI;
    /** ALARM/TB: alarm codes 00600 ... 00606. */
    static final int[] ALARM_TB = {0600, 0601, 0602, 0603, 0604, 0605, 0606};

    static final int LOOPMX = 16;
    static final double INITST = .03048;     // 10 fps in m/cs
    static final double DVMAX1 = 3.0480;     // 1000 fps
    static final double DVMAX2 = 3.014472;   // 989 fps
    static final double ONE_DPB2 = 1.0;
    static final double PMINE = 157420;      // 85 NM in metres (earth)
    static final double EPSILN1 = .0003048;  // .1 fps
    static final double NICKELDP = .021336;  // 7 fps
    static final double FIFPSDP = -.152400;  // 50 fps
    static final double PMINM = 10668;       // 35000 ft (moon)
    static final double DELMAX1 = .6096000;  // 200 fps
    static final double ONETHTH = .0001;
    static final double TMIN = 60000;        // 10 minutes in centiseconds

    /** Thrown to unwind after a terminate response (GOTOPOOH). */
    static final class Terminate extends RuntimeException {
        private static final long serialVersionUID = 1L;
    }

    private final Agc agc;

    // CSI local erasables
    private int loopct, csialrm, x2;
    private double delvcsi, deldv, dvprev, gamprev, postcsi, postcdh, hafpa1;

    public P32P35P72P75(Agc agc) {
        this.agc = agc;
    }

    // ================= erasable access helpers =================

    private boolean flag(String n) { return agc.get(n) != 0; }
    private void upflag(String n) { agc.set(n, 1); }
    private void downflag(String n) { agc.set(n, 0); }

    private boolean moon() { return flag("CMOONFLG"); }
    private double posScale() { return moon() ? 134217728.0 : 536870912.0; }  // 2^27 / 2^29
    private double velScale() { return moon() ? 32.0 : 128.0; }                // 2^5 / 2^7

    private double[] posGet(String sym) { return VecOps.scale(VecOps.getV(agc, sym), posScale()); }
    private void posSet(String sym, double[] v) { VecOps.setV(agc, sym, VecOps.scale(v, 1.0 / posScale())); }
    private double[] velGet(String sym) { return VecOps.scale(VecOps.getV(agc, sym), velScale()); }
    private void velSet(String sym, double[] v) { VecOps.setV(agc, sym, VecOps.scale(v, 1.0 / velScale())); }
    private double[] unitGet(String sym) { return VecOps.scale(VecOps.getV(agc, sym), 2.0); }
    private void unitSet(String sym, double[] v) { VecOps.setV(agc, sym, VecOps.scale(v, 0.5)); }
    /** DP centisecond word (B28). */
    private double timeGet(String sym) { return VecOps.getDpRaw(agc, sym); }
    private void timeSet(String sym, double cs) { VecOps.setDpRaw(agc, sym, Math.round(cs)); }
    private double posDpGet(String sym) { return VecOps.getDp(agc, sym) * posScale(); }
    private void posDpSet(String sym, double m) { VecOps.setDp(agc, sym, m / posScale()); }
    private double bodyConst(double earth, double moonVal) { return moon() ? moonVal : earth; }

    /** 2*MU of the current central body (RTMU in the original is the matching constant). */
    private double rtmu() { return 2 * bodyConst(3.986032e10, 4.902778e8); }

    /** GOFLASH: display V/N flashing; returns 0 = terminate, 1 = proceed, 2 = recycle. */
    private int goflash(int vn) {
        agc.set("VERBNOUN", vn);
        return agc.get("FLASH_RESPONSE");
    }

    /** GOTOPOOH: go to P00H and unwind. */
    private void gotopooh() {
        new FreshStartAndRestart(agc).goToPooh();
        throw new Terminate();
    }

    /** VNPOOH: flashing display; terminate goes to P00H, proceed returns, recycle redisplays. */
    private void vnPooh(int vn) {
        while (true) {
            int r = goflash(vn);
            if (r == 0) {
                gotopooh();
            }
            if (r == 1) {
                return;
            }
        }
    }

    // ================= P32 / P72 =================

    /** P32: CSI program, this vehicle active. */
    public void p32() {
        avflaga();
        p32strt();
    }

    /** P72: CSI program, this vehicle passive. */
    public void p72() {
        avflagp();
        p32strt();
    }

    private void p32strt() {
        try {
            timeSet("CENTANG", P30ZERO); // DCA P30ZERO / DXCH CENTANG
            p32p72a();
        } catch (Terminate t) {
            // program ended through GOTOPOOH
        }
    }

    /** ALMXIT: alarm from the CSI table, flash V05N09; terminate -> P00H; otherwise redisplay. */
    private void almxit() {
        int code = ALARM_TB[Math.max(0, Math.min(ALARM_TB.length - 1, csialrm - 1))]; // ALARM/TB -1,1
        new AlarmAndAbort(agc).varAlarm(code);
        while (true) {
            int r = goflash(V05N09);
            if (r == 0) {
                gotopooh();
            }
            // TC -4: redisplay
        }
    }

    /** ALMXITA: save the alarm index (SXA,2 CSIALRM) and go to ALMXIT. */
    private void almxita() {
        csialrm = x2;
        almxit();
    }

    /** P32/P72A: enter TCSI. */
    private void p32p72a() {
        p20flgon();
        agc.set("NN+1", 0);
        timeSet("TCSI", 0);
        while (true) {
            vnPooh(V06N11); // VN0611: TCSI
            double tcsi = timeGet("TCSI");
            if (tcsi > 0) {
                break; // VN0655
            }
            // TCSI not positive: use TETLEM, find the time of apocenter (DPPOSMAX) with TIMERAD
            timeSet("TDEC1", timeGet("TETLEM"));
            precset();
            double[] rv = posGet("RACT3");
            double[] vv = velGet("VACT3");
            upflag("RVSW");
            posDpSet("RDESIRED", posScale() * 0.5);   // DPPOSMAX
            Res res = timerad(rv, vv);
            timeSet("TCSI", res.t + timeGet("TDEC2"));
        }
        // VN0655: NN, ELEV
        while (true) {
            int r = goflash(V06N55);
            if (r == 0) {
                gotopooh();
            }
            if (r == 1) {
                break; // TC +2
            }
            // TC -5: redisplay
        }
        vnPooh(V06N37); // TTPI
        timeSet("TIG", timeGet("TCSI"));
        selectmu();
        p32p72b();
    }

    /** P32/P72B: main CSI loop; repeated after each crew modification. */
    private void p32p72b() {
        while (true) {
            advance();
            // integrate the passive vehicle from TCSI to TTPI
            double[] rp = posGet("RPASS1");
            double[] vp = velGet("VPASS1");
            intint(rp, vp, timeGet("TCSI"), timeGet("TTPI"), TWOPI);
            passive();
            csiA();
            // P32/P72C
            if (!flag("FINALFLG")) {
                upflag("UPDATFLG");
            }
            timeSet("T1TOT2", mod60(timeGet("T1TOT2")));  // P32/P72D, E
            timeSet("T2TOT3", mod60(timeGet("T2TOT3")));  // P32/P72F
            vnPooh(V06N75);
            double[] dv1 = s3233_1(velGet("DELVEET1"));
            velSet("DELVEET1", dv1);
            posSet("RACT1", posGet("RACT2"));
            disdvlvc(velGet("DELVEET2"), V06N82);
            timeSet("TTPIO", timeGet("TTPI"));
            vn1645();
        }
    }

    /** Repeatedly subtract 60 minutes while the result stays non-negative (P32/P72E loop). */
    private static double mod60(double t) {
        double v = t;
        do {
            v -= MIN60;
        } while (v >= 0);
        return v + MIN60;
    }

    // ================= P33 / P73 =================

    /** P33: CDH program, this vehicle active. */
    public void p33() {
        avflaga();
        p33p73a();
    }

    /** P73: CDH program, this vehicle passive. */
    public void p73() {
        avflagp();
        p33p73a();
    }

    private void p33p73a() {
        try {
            while (true) {
                p20flgon();
                vnPooh(V06N13); // TCDH
                timeSet("TTPI", timeGet("TTPIO"));
                timeSet("TIG", timeGet("TCDH"));
                selectmu();
                if (p33p73b()) {
                    return;
                }
                // alarm exit with enter: TC P33/P73A
            }
        } catch (Terminate t) {
            // program ended through GOTOPOOH
        }
    }

    /** P33/P73B and following; returns true when the program loops are finished (never normally). */
    private boolean p33p73b() {
        while (true) {
            advance();
            cdhmvr();
            intint3p(posGet("RACT2"), velGet("VACT3"));
            active();
            intint3p(posGet("RPASS2"), velGet("VPASS2"));
            passive();
            timeSet("NOMTPI", P30ZERO);
            upflag("ITSWICH");
            double r = s3334_1();
            if (r != 0) {
                new AlarmAndAbort(agc).alarm(0611, 0);
                int resp = goflash(V05N09);
                if (resp == 0) {
                    gotopooh();
                }
                if (resp != 1) {
                    return false; // TC P33/P73A
                }
                timeSet("NOMTPI", P30ZERO);
            }
            // P33/P73C
            if (!flag("FINALFLG")) {
                upflag("UPDATFLG");
            }
            // P33/P73D
            timeSet("TTPI", timeGet("NOMTPI") + timeGet("TTPI"));
            double v = timeGet("TTPI") - timeGet("TCDH");
            do {                                   // P33/P73E
                v -= MIN60;
            } while (v >= 0);
            timeSet("T1TOT2", v + MIN60);
            double d = timeGet("TTPI") - timeGet("TTPIO");
            double a = Math.abs(d);
            do {                                   // P33/P73F
                a -= MIN60;
            } while (a >= 0);
            a += MIN60;
            timeSet("T2TOT3", d < 0 ? -a : a);
            vnPooh(V06N75);
            double[] dv2 = s3233_1(velGet("DELVEET2"));
            velSet("DELVEET2", dv2);
            vn1645();
            // GOTO P33/P73B
        }
    }

    // ================= flag subroutines =================

    /** AVFLAGA: AVFLAG = LEM (up). */
    public void avflaga() { upflag("AVFLAG"); }

    /** AVFLAGP: AVFLAG = CSM (down). */
    public void avflagp() { downflag("AVFLAG"); }

    /** P20FLGON: set UPDATFLG and TRACKFLG. */
    public void p20flgon() {
        upflag("UPDATFLG");
        upflag("TRACKFLG");
    }

    // ================= DISDVLVC =================

    /**
     * DISDVLVC: converts the reference-frame velocity change into local-vertical coordinates
     * (DELVLVC = M . DV, M from S32/33.X), displays it with the given verb/noun and lets the
     * crew modify it; the (possibly modified) DELVLVC is left in the erasable.
     */
    void disdvlvc(double[] dv, int vn) {
        velSet("DELVLVC", dv);
        double[][] m = s3233_x();
        double[] lv = VecOps.mxv(m, velGet("DELVLVC"));
        velSet("DELVLVC", lv);
        agc.set("VERBNOUN", vn);
        vnPooh(vn);
    }

    // ================= CSI/A =================

    private static final int DONE = 0, RESTART = 1, AGAIN = 2;

    /**
     * CSI/A: iterate the CSI delta-V (horizontal at CSI ignition) so that the resulting CDH-to-TPI
     * geometry gives the commanded line of sight (ELEV) at TPI. Uses a secant/Newton scheme on
     * the flight-path angle GAMMA of the required transfer. Two cycles are attempted (flags
     * S32.1F3A/B select the stage); failure ends in ALMXIT with one of alarms 00600-00606.
     * Normal exit is P32/P72C (return from this method).
     */
    void csiA() {
        downflag("S32.1F1");   // DVT1 exceeded max indicator
        upflag("S32.1F2");     // first pass of Newton iteration
        downflag("S32.1F3A");
        upflag("S32.1F3B");
        loopct = 0;
        csialrm = 0;
        while (true) { // CSI/B (re-entered by SCNDSOL for the second cycle)
            double[] ract1 = posGet("RACT1");
            double[] vact1 = velGet("VACT1");
            double[] up1 = unitGet("UP1");
            double ra1 = VecOps.abs(ract1);
            double rp3 = VecOps.abs(posGet("RPASS3"));
            // coelliptic speed at CSI: SQRT(2 MU / ((1 + RA1/RP3) RA1))
            double vh = Math.sqrt(rtmu() / ((1 + ra1 / rp3) * ra1));
            double[] uh1 = VecOps.unit(VecOps.cross(up1, VecOps.unit(ract1))); // UH1
            delvcsi = vh - VecOps.dot(vact1, uh1);
            deldv = INITST;
            while (true) { // CSI/B1
                loopct = loopct + 1;
                x2 = 6;
                int r = loopct - LOOPMX >= 0 ? scndsol() : csiB2();
                if (r == AGAIN) {
                    continue;
                }
                if (r == DONE) {
                    return;
                }
                break; // RESTART: CSI/B
            }
        }
    }

    /** CSI/B2 ... CSI/SOL. Returns DONE (P32/P72C), RESTART (second cycle) or AGAIN (CSI/B1). */
    private int csiB2() {
        while (true) {
            x2 = 7;
            if (Math.abs(delvcsi) - DVMAX1 >= 0) {
                if (flag("S32.1F1")) {
                    return scndsol();
                }
                if (flag("S32.1F3A") && flag("S32.1F3B")) {
                    return scndsol();
                }
                upflag("S32.1F1"); // CSI/B22
                delvcsi = (delvcsi < 0 ? -1 : 1) * DVMAX2;
            }
            // CSI/B23
            double[] ract1 = posGet("RACT1");
            double[] up1 = unitGet("UP1");
            double[] uh1 = VecOps.unit(VecOps.cross(up1, VecOps.unit(ract1)));
            double[] delveet1 = VecOps.scale(uh1, delvcsi);
            velSet("DELVEET1", delveet1);
            double[] vact4 = VecOps.add(velGet("VACT1"), delveet1);
            velSet("VACT4", vact4);
            // post-CSI orbit: transfer through ~359 deg gives (almost) a period, so HAFPA1
            upflag("RVSW");
            Res full = timethet(ract1, vact4, SN359, CS359);
            hafpa1 = full.t / 2.0;
            postcsi = periapo(ract1, vact4);                 // PERIAPO then SHIFTR1
            double ecc = VecOps.getDp(agc, "ECC") * 8.0;
            double r1 = VecOps.abs(ract1);
            double rdotv = VecOps.dot(ract1, vact4);
            double p = pParam();
            // CIRCL when CENTANG is non-zero, the orbit is nearly circular, or /RDOTV/R1 < 7 fps
            boolean circ = timeGet("CENTANG") != 0 || ecc - ONETHTH < 0
                || Math.abs(rdotv) / r1 - NICKELDP < 0;
            double tapse;
            if (circ) {
                tapse = 0;
            } else {
                double mu = rtmu() / 2;
                double sinPart = Math.sqrt(p / mu) * rdotv / r1;   // e sin f (12D)
                double cosPart = p - 1;                            // e cos f (14D)
                double[] sc = VecOps.unit(new double[] {sinPart, cosPart, 0});
                double[] vNeg = VecOps.scale(vact4, rdotv < 0 ? 1 : -1);
                upflag("RVSW");
                Res tt = timethet(ract1, vNeg, sc[0], sc[1]);
                tapse = rdotv >= 0 ? tt.t : hafpa1 - tt.t;    // NTP/2 entry
            }
            // NTP/2
            double nn = Agc.toSigned(agc.get("NN"));
            double tcdh = timeGet("TCSI") + nn * hafpa1 - tapse;
            timeSet("TCDH", tcdh);
            x2 = 5;
            if (timeGet("TTPI") - tcdh < 0) {
                return scndsol();
            }
            // state at CDH for both vehicles
            intint2c(ract1, vact4);
            posSet("RACT2", posGet("RATT"));
            velSet("VACT2", velGet("VATT"));
            intint2c(posGet("RPASS1"), velGet("VPASS1"));
            posSet("RPASS2", posGet("RATT"));
            velSet("VPASS2", velGet("VATT"));
            cdhmvr();
            postcdh = periapo(posGet("RACT2"), velGet("VACT3"));
            // propagate the active vehicle from CDH to TPI
            intint(posGet("RACT2"), velGet("VACT3"), timeGet("TCDH"), timeGet("TTPI"), TWOPI);
            active();
            // line of sight at TPI: choose K so that |RA3 + K U| = |RP3|
            double elev = VecOps.getDp(agc, "ELEV") * Math.PI * 2;
            double[] ra3 = posGet("RACT3");
            double[] rpass3 = posGet("RPASS3");
            double[] ura3 = VecOps.unit(ra3);
            double[] uh3 = VecOps.unit(VecOps.cross(up1, ura3));
            double[] u = VecOps.add(VecOps.scale(uh3, Math.cos(elev)), VecOps.scale(ura3, Math.sin(elev)));
            double temp1 = VecOps.dot(u, ra3);
            double temp2 = temp1 * temp1 - VecOps.dot(ra3, ra3) + VecOps.dot(rpass3, rpass3);
            if (temp2 < 0) {
                if (loopct - 1 == 0) {
                    x2 = 1;
                    almxita();
                }
                deldv = deldv / 2;
                delvcsi = dvprev - deldv;
                return AGAIN;
            }
            double temp3 = Math.sqrt(temp2);
            double k2 = -temp1 - temp3;
            double k1 = -temp1 + temp3;
            double k = Math.abs(k2) < Math.abs(k1) ? k2 : k1;     // K2.
            double[] v = VecOps.unit(VecOps.add(ra3, VecOps.scale(u, k)));
            double[] urp3 = VecOps.unit(rpass3);
            double[] uvp3 = VecOps.unit(velGet("VPASS3"));
            double temp = VecOps.dot(VecOps.cross(urp3, v), VecOps.cross(uvp3, urp3));
            double cosg = Math.max(-1, Math.min(1, VecOps.dot(v, urp3)));
            double gamma = (temp < 0 ? -1 : 1) * Math.acos(cosg);
            // iteration on GAMMA
            if (flag("S32.1F2")) {
                return frstpas(gamma);
            }
            double slope = (gamma - gamprev) / (delvcsi - dvprev);
            dvprev = delvcsi;
            boolean a = flag("S32.1F3A");
            boolean b = flag("S32.1F3B");
            if (a && b) {
                if (slope * gamprev < 0) {
                    deldv = (deldv < 0 ? -1 : 1) * INITST;
                    upflag("S32.1F3A");
                    downflag("S32.1F3B");
                    return frstpas(gamma);
                }
                fiftyfps(gamma, slope);
                continue; // CSI/B2
            }
            if (!a && !b) {
                fiftyfps(gamma, slope); // THRDCHK falls through
                continue;
            }
            // NEWTN
            deldv = gamma / slope;
            gamprev = gamma;
            if (Math.abs(deldv) - EPSILN1 < 0) {
                return csiSol();
            }
            if (Math.abs(deldv) - DELMAX1 >= 0) {
                deldv = (deldv < 0 ? -1 : 1) * DELMAX1;
            }
            delvcsi = delvcsi - deldv;                        // CSISTEP
            return AGAIN;
        }
    }

    /** FRSTPAS: remember GAMMA and DELVCSI, step DELVCSI by -DELDV. */
    private int frstpas(double gamma) {
        gamprev = gamma;
        dvprev = delvcsi;
        delvcsi = delvcsi - deldv;
        downflag("S32.1F2");
        return AGAIN;
    }

    /** FIFTYFPS: step of 50 fps against the slope sign; moves to the "11" stage. */
    private void fiftyfps(double gamma, double slope) {
        double d = Math.abs(FIFPSDP);
        d = slope < 0 ? -d : d;
        d = gamprev < 0 ? -d : d;
        deldv = d;
        delvcsi = -deldv + delvcsi;
        upflag("S32.1F3B");
        upflag("S32.1F3A");
        gamprev = gamma;
    }

    /** CSI/SOL: validity checks of the converged solution. */
    private int csiSol() {
        double pmin = bodyConst(PMINE, PMINM);
        x2 = 2;
        if (postcsi - pmin < 0) {
            return scndsol();
        }
        x2 = 3;
        if (postcdh - pmin < 0) {
            return scndsol();
        }
        timeSet("T1TOT2", timeGet("TCDH") - timeGet("TCSI"));
        x2 = 4;
        if (timeGet("T1TOT2") - TMIN < 0) {
            return scndsol();
        }
        timeSet("T2TOT3", timeGet("TTPI") - timeGet("TCDH"));
        x2 = 5;
        if (timeGet("T2TOT3") - TMIN >= 0) {
            return DONE; // P32/P72C
        }
        return scndsol();
    }

    /**
     * SCNDSOL: the first cycle failed. If this was the first cycle start the second one (flags
     * reset, loop count zeroed, CSIALRM = X2) and return RESTART; otherwise exit via ALMXIT.
     */
    private int scndsol() {
        if (flag("S32.1F3A") || !flag("S32.1F3B")) {
            almxit();
        }
        csialrm = x2;
        loopct = 0;
        downflag("S32.1F1");
        upflag("S32.1F2");
        downflag("S32.1F3A");
        downflag("S32.1F3B");
        return RESTART;
    }

    // ================= ADVANCE / ROTATE =================

    /**
     * ADVANCE: advance both vehicles to TIG (PRECSET), set XDELVFLG, and rotate the active vehicle
     * state into the plane of the passive vehicle orbit (UP1 = UNIT(URP3 x VP3)).
     */
    void advance() {
        timeSet("TDEC1", timeGet("TIG"));
        precset();
        upflag("XDELVFLG");
        double[] vp3 = velGet("VPASS3");
        double[] rp3 = posGet("RPASS3");
        velSet("VPASS2", vp3);
        velSet("VPASS1", vp3);
        posSet("RPASS2", rp3);
        posSet("RPASS1", rp3);
        double[] up1 = VecOps.unit(VecOps.cross(VecOps.unit(rp3), vp3));
        unitSet("UP1", up1);
        double[] ra3 = posGet("RACT3");
        posSet("RTIG", ra3);
        double[] ra = rotate(ra3, up1);
        posSet("RACT2", ra);
        posSet("RACT1", ra);
        double[] va3 = velGet("VACT3");
        velSet("VTIG", va3);
        double[] va = rotate(va3, up1);
        velSet("VACT2", va);
        velSet("VACT1", va);
    }

    /** ROTATE: removes the component along UP1 and restores the original magnitude. */
    static double[] rotate(double[] v, double[] up1) {
        double[] inPlane = VecOps.sub(v, VecOps.scale(up1, VecOps.dot(v, up1)));
        return VecOps.scale(VecOps.unit(inPlane), VecOps.abs(v));
    }

    // ================= INTINT2C / INTINT3P =================

    /** INTINT2C: INTINT from TCSI to TCDH with the TWOPI option. */
    void intint2c(double[] r, double[] v) {
        intint(r, v, timeGet("TCSI"), timeGet("TCDH"), TWOPI);
    }

    /** INTINT3P: INTINT from TCDH to TTPI with option P30ZERO. */
    void intint3p(double[] r, double[] v) {
        intint(r, v, timeGet("TCDH"), timeGet("TTPI"), P30ZERO);
    }

    /** INTINT wrapper: results are left in RATT / VATT. */
    private void intint(double[] r, double[] v, double t1, double t2, double option) {
        posSet("RINT", r);
        velSet("VINT", v);
        timeSet("INTT1", t1);
        timeSet("INTT2", t2);
        VecOps.setDp(agc, "INTOPT", option / TWOPI);
        new P3435P7475(agc).intInt(r, v, t1, t2, option, 0);   // INTINT
    }

    // ================= S32/33.1 and S32/33.X =================

    /**
     * S32/33.1: displays a delta V (V06N81) in LV coordinates, accepts crew changes and returns
     * the resulting delta V in reference coordinates; also stores DELVSIN and DELVSAB.
     */
    double[] s3233_1(double[] dv) {
        disdvlvc(dv, V06N81);
        double[][] m = s3233_x();
        double[] delvsin = VecOps.scale(VecOps.vxm(velGet("DELVLVC"), m), 2.0 / 2.0);
        velSet("DELVSIN", delvsin);
        velSet("DELVSAB", new double[] {VecOps.abs(delvsin), 0, 0});
        return delvsin;
    }

    /**
     * S32/33.X: local-vertical transformation matrix. Rows: UP1 x -URA1 ... i.e. (-URA1) x UP1,
     * -UP1, -URA1.
     */
    double[][] s3233_x() {
        double[] up1 = unitGet("UP1");
        double[] ura1 = VecOps.unit(posGet("RACT1"));
        double[] row0 = VecOps.cross(VecOps.scale(ura1, -1), up1);
        return new double[][] {row0, VecOps.scale(up1, -1), VecOps.scale(ura1, -1)};
    }

    // ================= CDHMVR =================

    /**
     * CDHMVR: constant-delta-height maneuver. Rotates the passive vehicle state to the active
     * vehicle's true anomaly (TIMETHET), forms DIFFALT, the semimajor axis of the active orbit that
     * keeps the altitude difference constant, and the resulting velocity VACT3 at CDH; DELVEET2 =
     * VACT3 - VACT2.
     */
    void cdhmvr() {
        double[] ract2 = posGet("RACT2");
        double[] unvec = VecOps.unit(ract2);
        double[] rpass2 = posGet("RPASS2");
        double cs = VecOps.dot(VecOps.unit(rpass2), unvec);
        double sn = Math.sqrt(Math.max(0, 1 - cs * cs));
        double[] up1 = unitGet("UP1");
        double sgn = VecOps.dot(VecOps.cross(VecOps.scale(unvec, -1), rpass2), up1);
        sn = sgn < 0 ? -sn : sn;
        downflag("RVSW");
        Res tt = timethet(rpass2, velGet("VPASS2"), sn, cs);
        double[] rpp = tt.r;                       // passive position at the active's anomaly
        double[] vpp = tt.v;
        double vsubpv = VecOps.dot(vpp, unvec);    // radial component of the passive velocity
        double rpl = VecOps.abs(rpp);
        double ral = VecOps.abs(ract2);
        double diffalt = rpl - ral;                // DELTA H in metres
        posDpSet("DIFFALT", diffalt);
        double r1 = VecOps.abs(rpass2);
        double r1a = VecOps.getDp(agc, "R1A") * 64.0;
        double asubp = r1a != 0 ? r1 / r1a : r1;   // passive semimajor axis
        double asuba = asubp - diffalt;            // active semimajor axis
        double ratio = asubp / asuba;
        double vsubav = vsubpv * ratio * Math.sqrt(ratio);
        double mu = rtmu() / 2;
        double vr2 = mu * (2 / ral - 1 / asuba) - vsubav * vsubav;
        double vt = Math.sqrt(Math.max(0, vr2));
        double[] uf = VecOps.unit(VecOps.cross(up1, unvec));
        double[] vact3 = VecOps.add(VecOps.scale(uf, vt), VecOps.scale(unvec, vsubav));
        velSet("VACT3", vact3);
        velSet("DELVEET2", VecOps.sub(vact3, velGet("VACT2")));
    }

    // ================= COMPTGO =================

    /** COMPTGO: clear DISPDEX, start CLOKTASK on the waitlist (2 cs), 2PHSCHNG 40036/05024/13000. */
    public void comptgo() {
        agc.set("DISPDEX", 0);
        new Waitlist(agc).waitlist(2, () -> { /* CLOKTASK */ });
        agc.set("PHASE6", 040036);
        agc.set("PHASE4", 05024);
        agc.set("PHASE4B", 013000);
    }

    // ================= glue to neighbouring routines =================

    private void selectmu() {
        new P3435P7475(agc).selectmu();
    }

    private void precset() {
        new P3435P7475(agc).precset();
    }

    private void passive() {
        new P3435P7475(agc).passive();
    }

    private void active() {
        new P3435P7475(agc).active();
    }

    private double s3334_1() {
        return new P3435P7475(agc).s3334p1();
    }

    private void vn1645() {
        new P3435P7475(agc).vn1645();
    }

    /** Result of a TIMETHET / TIMERAD call: time T and, when RVSW is clear, the terminal state. */
    static final class Res {
        double t;
        double[] r;
        double[] v;
    }

    private Res conicResult(ConicSubroutines c, double t) {
        Res res = new Res();
        res.t = t;
        res.r = c.newrvec;
        res.v = c.newvvec;
        return res;
    }

    private Res timerad(double[] r, double[] v) {
        ConicSubroutines c = new ConicSubroutines(agc);
        c.moon = moon();
        c.rvec = r;
        c.vvec = v;
        c.rvsw = flag("RVSW");
        c.rdesired = posDpGet("RDESIRED");
        c.sgnrdot = agc.get("SGNRDOT") == 0 ? 1 : Agc.toSigned(agc.get("SGNRDOT"));
        return conicResult(c, c.timerad());
    }

    private Res timethet(double[] r, double[] v, double sn, double cs) {
        ConicSubroutines c = new ConicSubroutines(agc);
        c.moon = moon();
        c.rvec = r;
        c.vvec = v;
        c.snth = sn;
        c.csth = cs;
        c.rvsw = flag("RVSW");
        return conicResult(c, c.timethet());
    }

    /** PERIAPO: radius of pericenter of the conic through r, v (ECC is left in the erasable). */
    private double periapo(double[] r, double[] v) {
        ConicSubroutines c = new ConicSubroutines(agc);
        c.moon = moon();
        c.rvec = r;
        c.vvec = v;
        c.apsides();
        VecOps.setDp(agc, "ECC", c.ecc / 8.0);
        return c.periapse;
    }

    private double pParam() {
        return VecOps.getDp(agc, "P") * 16.0;
    }
}
