package apollo.luminary099;

import apollo.agc.Agc;

/**
 * Translation of P40-P47.agc (Luminary 1A build 099).
 *
 * P40 (DPS burn), P41 (RCS burn), P42 (APS burn) and P47 (thrust monitor) programs, plus the
 * burn subroutines S40.1 (thrust direction / VG), S40.2,3 (preferred IMU orientation), S40.8
 * (cross-product steering), S40.13 (burn time), S40.9 (Lambert VTOGAIN, RASTEER1), TRIMGIMB
 * (DPS gimbal trim) and S41.1 (reference to body transformation), and the engine-off
 * machinery (IMPLBURN, ENGINOFF, COASTSET, UPDATEVG, STEERING, NSTEER).
 *
 * Interpretive sections are translated as straight-line vector arithmetic on double[3]
 * values read from/written to erasable triples (see {@link #vec}/{@link #setVec}); the
 * original scaling is noted in comments. Routines outside this file (BANKCALL targets,
 * FINDVAC, WAITLIST, PHASCHNG, ...) go through {@link #extern(String, Object...)}, which a
 * host simulator may override.
 */
public class P40P47 {
    // ---- constants ----
    public static final int SEC15 = 1500;
    public static final int SEC45 = 4500;
    public static final int FIVE_SEC = 500;
    public static final int SECS26 = 2600;
    public static final int SEC30_DP = 3000;
    public static final int V16N40 = 1640;
    public static final int V16N85B = 1685;
    public static final int V1683 = 1683;
    public static final double EPS1 = 2.777777778e-2;     // 10 degrees at 1 revolution
    public static final double EPS2 = 9.722222222e-2;     // 35 degrees at 1 revolution
    public static final double THETACON = .31830989 / 256; // .31830989 B-8
    public static final double FOUR_DT = -800.0 / (1 << 18);  // -FOURDT: -4 (200 CS) B-18
    public static final double FOURSECS = 400;            // 400 CS at 2^28 CS
    public static final double TWO_PI_3 = 3.141592653 / 4; // 2PI+3, B-2
    public static final double SEC1_2D = 100.0 / (1 << 14);
    public static final double SEC3_5 = 350.0 / (1 << 13);
    public static final double SECS5 = 500.0 / (1 << 14);
    public static final double SECS6 = 600.0 / (1 << 14);
    public static final double SECS89 = 8900.0 / (1 << 14);
    private static final int BIT1 = 1;
    private static final int PRIO5 = 05000;
    private static final int EBANK5 = 02400;
    private static final int BIT10 = 01000;
    private static final int BIT12 = 04000;

    private final Agc agc;

    public P40P47(Agc agc) {
        this.agc = agc;
    }

    /** Hook for calls to routines outside this file; the default implementation does nothing. */
    protected void extern(String routine, Object... args) {
    }

    // ---- helpers ----

    private boolean flag(String f) {
        return agc.get(f) != 0;
    }

    private void upflag(String f) {
        agc.set(f, 1);
    }

    private void downflag(String f) {
        agc.set(f, 0);
    }

    /** Reads a DP pair as a signed fraction (name, name+1). */
    private double dp(String sym) {
        long v = ((long) Agc.toSigned(agc.get(sym)) << 14) + Agc.toSigned(agc.get(sym + "+1"));
        return v / (double) (1L << 28);
    }

    private void setDp(String sym, double value) {
        long v = Math.round(value * (1L << 28));
        agc.set(sym, Agc.fromSigned((int) (v >> 14)));
        agc.set(sym + "+1", Agc.fromSigned((int) (v & 037777)));
    }

    /** Reads a DP vector stored at sym, sym+2, sym+4 (each a DP word pair). */
    private double[] vec(String sym) {
        return new double[] {dp(sym), dp(sym + "+2"), dp(sym + "+4")};
    }

    private void setVec(String sym, double[] v) {
        setDp(sym, v[0]);
        setDp(sym + "+2", v[1]);
        setDp(sym + "+4", v[2]);
    }

    private static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    private static double abval(double[] a) {
        return Math.sqrt(dot(a, a));
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }

    private static double[] unit(double[] a) {
        double m = abval(a);
        return m == 0 ? new double[3] : new double[] {a[0] / m, a[1] / m, a[2] / m};
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

    /** Matrix (stored by rows at sym, 18 words) times vector (MXV). */
    private double[] mxv(String matrix, double[] v) {
        double[] r = new double[3];
        for (int i = 0; i < 3; i++) {
            double[] row = {dp(matrix + "+" + (6 * i)), dp(matrix + "+" + (6 * i + 2)), dp(matrix + "+" + (6 * i + 4))};
            r[i] = dot(row, v);
        }
        return r;
    }

    // ---- P40 ----

    /** P40LM: P40 entry, called via job from V37E. */
    public void p40lm() {
        extern("PHASCHNG", 04024);
        agc.set("WHICH", agc.get("P40ADRES"));        // initialization for BURNBABY
        if ((agc.get("FLGWRD10") & agc.get("APSFLBIT")) != 0) {
            extern("P40ALM");                        // descent unit staged: alarm 1706
            return;
        }
        extern("R02BOTH");                           // IMU status check
        // initialize DVMON
        int thr = ((~agc.get("DAPBOOLS")) & agc.get("CSMDOCKD")) != 0 ? agc.get("THRESH1") : 0;
        agc.set("DVTHRUSH", Agc.add(thr, agc.get("THRESH3")));
        agc.set("DVCNTR", 4);
        // load constants for DPS burn: F, MDOT, TDECAY from FDPS; clear NOTHROTL
        extern("LOADFDPS");
        downflag("NOTHROTL");
        agc.set("VEX", Agc.fromSigned(-agc.get("DPSVEX") / 2)); // DCOMP SR1: exhaust velocity
        p40in();
    }

    /** P40IN: common to P40 and P42 once F, MDOT, VEX are loaded. */
    public void p40in() {
        s401();                                      // computes UT and VGTIG
        s4023();                                     // preferred IMU orientation
        extern("PFLITEDB");                          // zero attitude errors, DB to one degree
        p40sxt4();
        extern("BURNBABY");
    }

    /** P40SXT4 / P41MANU: do the attitude maneuver (R60LEM) with 3AXISFLG cleared. */
    public void p40sxt4() {
        p41manu();
    }

    public void p41manu() {
        downflag("3AXISFLG");                        // R60 uses VECPOINT
        extern("R60LEM");                            // attitude maneuver routine
    }

    /** POSTBURN: display final values of TG, VG, DELTAVM (V16N40) and wait for PRO/TERM. */
    public void postburn() {
        agc.set("DISPDEX", 0);
        agc.set("AVEGEXIT", agc.get("ACADN85"));
        extern("GOFLASHR", V16N40);
        // V34E: term40; PROCEED: tignow; ENTER: repeat
        tignow();
    }

    /** P40PHS1. */
    public void p40phs1() {
        extern("PHASCHNG", 00014);
        extern("ENDOFJOB");
    }

    /** TIGNOW: zero attitude errors, min deadband, display V16N85 (VG in body axes). */
    public void tignow() {
        extern("ZATTEROR");
        extern("SETMINDB");
        extern("REFLASHR", V16N85B);
        term40();
    }

    /** TERM40: restore servicer exit, zero rendezvous counters, restore DB, go to P00. */
    public void term40() {
        agc.set("AVEGEXIT", agc.get("SERVCADR"));
        agc.set("TRKMKCNT", 0);
        agc.set("DISPDEX", 0);
        extern("RESTORDB");
        extern("GOTOPOOH");
    }

    // ---- P41 ----

    /** P41LM: RCS burn. */
    public void p41lm() {
        agc.set("WHICH", agc.get("P41ADRES"));
        extern("R02BOTH");
        // if NJETSFLG is set load 2-jet F (FRCS2) else 4-jet F (FRCS4)
        setDp("F", flag("NJETSFLG") ? dp("FRCS2") : dp("FRCS4"));
        p41in();
    }

    public void p41in() {
        s401();
        p41norm();
    }

    public void p41norm() {
        s4023();                                     // preferred IMU orientation, sets PFRATFLG
        extern("ZATTEROR");
        extern("SETMINDB");                          // 0.3 degree deadband
        p40sxt4();
        // transform VGTIG to body axes for V16N85 (2^7 m/cs)
        setVec("VGBODY", s411(vec("VGTIG")));
        extern("GODSPRET", V16N85B);
        agc.set("DISPDEX", PRIO5);                   // for safety only
        extern("FINDVAC", "DYNMDISP");
        extern("2PHSCHNG", 00076, 04024);            // group 6 restarts at REDO6.7
        extern("B*RNB*B*");
    }

    /** BLNKWAIT / REDO6.7. */
    public void blnkwait() {
        extern("DELAYJOB", 100);
        redo67();
    }

    public void redo67() {
        if (Agc.toSigned(agc.get("DISPDEX")) + 2 == 0) {  // blanking between TIG-35 and TIG-30
            blnkwait();
            return;
        }
        extern("GODSPRET", V16N85B);
        extern("PRIOCHNG", PRIO5);
        dynmdisp();
    }

    /** DYNMDISP: update N85 once a second until past TIG-35. */
    public void dynmdisp() {
        while (Agc.toSigned(agc.get("DISPDEX")) > 0) {
            setVec("VGBODY", s411(vec("VGPREV")));
            extern("DELAYJOB", 100);
        }
        extern("ENDOFJOB");
    }

    /** CALCN85: servicer-driven update of N85. */
    public void calcn85() {
        updatevg();
        setVec("VGBODY", s411(vec("VGPREV")));
        extern("SERVEXIT");
    }

    // ---- P42 ----

    /** P42LM: APS burn. */
    public void p42lm() {
        extern("PHASCHNG", 04024);
        agc.set("WHICH", agc.get("P42ADRES"));
        if ((~agc.get("FLGWRD10") & agc.get("APSFLBIT")) != 0) {
            extern("P40ALM");                        // APS not staged
            return;
        }
        extern("R02BOTH");
        agc.set("DVTHRUSH", agc.get("THRESH2"));     // initialize DVMON
        agc.set("DVCNTR", 4);
        upflag("AVFLAG");
        extern("LOADFAPS");                          // F, MDOT, TDECAY from FAPS
        agc.set("VEX", agc.get("APSVEX"));
        p40in();
    }

    // ---- P47 ----

    /** P47LM: thrust monitor. */
    public void p47lm() {
        extern("R02BOTH");
        extern("MIDTOAV2");
        extern("TWIDDLE", "STARTP47", agc.get("MPAC+1"));
        extern("ENDOFJOB");
    }

    public void startp47() {
        extern("PHASCHNG", 05014, 077777);
        agc.set("AVEGEXIT", agc.get("ACADN83"));
        extern("FINDVAC", "P47BODY");
        extern("REDO4.2");                           // checks phase 5 and goes to PREREAD
    }

    /** CALCN83: servicer-driven DELVIMU computation for P47. */
    public void calcn83() {
        double[] d = add(vec("DELVCTL"), vec("DELVREF"));
        setVec("DELVSIN", d);                        // temp storage for restarts
        setVec("DELVIMU", s411(d));
        extern("PHASCHNG", 010035);                  // REREADAC and here
        setVec("DELVCTL", vec("DELVSIN"));
        extern("SERVEXIT");
    }

    public void p47bod() {
        extern("GOFLASHR", V1683);
        // V34E: GOTOPOOH; PROCEED/ENTER: repeat P47BODY or P40PHS1
        extern("GOTOPOOH");
    }

    public void p47body() {
        setVec("DELVIMU", new double[3]);
        setVec("DELVCTL", new double[3]);
        p47bod();
    }

    // ---- engine-off machinery ----

    /** IMPLBURN: impulsive burn: schedule ENGOFTSK after TGO+1 and clear flags. */
    public void implburn() {
        int dt = getdt(agc.get("TGO+1"));
        extern("TWIDDLE", "ENGOFTSK", dt);
        downflag("IGNFLAG");
        downflag("ASTNFLAG");
        downflag("IMPULSW");
        extern("PHASCHNG", 040114);                  // restart protect ENGOFTSK
        extern("FIXDELAY", 50);                      // wait half a second
        extern("NOULLAGE");
        extern("TASKOVER");
    }

    /** ENGOFTSK: waitlist task wrapper around ENGINOFF. */
    public void engoftsk() {
        enginoff();
        extern("TASKOVER");
    }

    /** ENGINOFF: schedule POSTBURN job, COASTSET task, then turn the engine off. */
    public void enginoff() {
        extern("FINDVAC", "POSTBURN", 012);          // PRIO12, lower than CLOCKJOB
        enginof2();
    }

    public void enginof2() {
        extern("WAITLIST", 1, "COASTSET");
        enginof1();
    }

    public void enginof1() {
        agc.set("FLAGWRD7", agc.get("FLAGWRD7") | agc.get("IDLEFBIT")); // set the idle bit
        extern("NOULLAGE");
        enginof4();
    }

    public void enginof4() {
        agc.set("TEVENT", agc.get("TIME2"));         // DXCH TIME2 -> TEVENT
        agc.set("TEVENT+1", agc.get("TIME2+1"));
        enginof3();
    }

    /** ENGINOF3: clear ENGONFLG, turn off the DPS/APS, trim gimbal and auto-throttle. */
    public void enginof3() {
        agc.set("FLAGWRD5", agc.get("FLAGWRD5") & ~agc.get("ENGONBIT"));
        int ds = agc.get("DSALMOUT") & ~030;          // clear engine on/off bits (PRIO30)
        agc.set("DSALMOUT", ds);                      // turn off the engine
        agc.set("DAPBOOLS", agc.get("DAPBOOLS") & ~agc.get("USEQRJTS")); // trim gimbal off
        agc.set("THRUST", 0);                         // zero auto-throttle
        agc.set("CHAN14", agc.get("CHAN14") | 010);   // BIT4: hardware throttle zero
        extern("ISWRETRN");
    }

    /** COASTSET: DAP coasting initialization (waitlist task). */
    public void coastset() {
        extern("ALLCOAST");
        extern("TASKOVER");
    }

    // ---- UPDATEVG / STEERING ----

    /** UPDATEVG: cross-product steering update; call Lambert (S40.9) when needed. */
    public void updatevg() {
        s408();
        if (!flag("XDELVFLG")) {
            if (flag("NORMSW")) {
                // skip TIGSAVEP update
            } else {
                double t = dp("PIPTIME") - dp("TIGSAVE") - dp("TNEWA");
                if (t >= 0) {
                    // BMN GETRANS not taken
                    setDp("TIGSAVEP", dp("TIGSAVE") + dp("TNEWA"));
                    t180setup();
                    return;
                }
                getrans();
                return;
            }
        }
        t180setup();
    }

    private void t180setup() {
        if (Agc.toSigned(agc.get("PHASE2")) != 0) {   // CCS PHASE2: Lambert already scheduled
            return;                                   // NO.9
        }
        extern("FINDVAC", "S40.9", 010);              // Lambert VTOGAIN
        extern("2PHSCHNG", 00172, 010035);
        endsteer();
    }

    /** ENDSTEER. */
    public void endsteer() {
        setDp("TIGSAVE", dp("TIGSAVEP"));
        setVec("RINIT", vec("RN"));
        setVec("VINIT", vec("VN"));
        getrans();
    }

    /** GETRANS: DELLT4 = TPASS4 - PIPTIME. */
    public void getrans() {
        setDp("DELLT4", dp("TPASS4") - dp("PIPTIME"));
    }

    /** STEERING: UPDATEVG then fall into NSTEER. */
    public void steering() {
        updatevg();
        nsteer();
    }

    /** NSTEER: impulse-switch / IDLEFLAG handling; schedules ENGOFTSK for the final seconds. */
    public void nsteer() {
        if ((~agc.get("FLAGWRD2") & agc.get("IMPULBIT")) != 0 || !flag("IMPULSW")) {
            extern("SERVEXIT");                       // IMPULSW = 0: exit
            return;
        }
        if ((~agc.get("FLAGWRD7") & agc.get("IDLEFBIT")) != 0) {
            extern("SERVEXIT");                       // DVMON on: thrusting, IMPULSW via S40.8
            return;
        }
        extern("STOPRATE");
        downflag("IMPULSW");
        upflag("IDLEFLAG");                           // turn off DVMON
        int dt = Agc.toSigned(agc.get("TIG+1")) - Agc.toSigned(agc.get("TIME2+1")); // TIG - TIME2
        dt = getdt(dt);
        extern("TPAGREE");
        extern("TWIDDLE", "ENGOFTSK", dt);
        extern("2PHSCHNG", 040114, 00035);            // ENGOFTSK; servicer REREADAC
        extern("ENDOFJOB");
    }

    /** GETDT: TGO = (max(A,0)+1) as DP in TGO; returns the delay. */
    public int getdt(int a) {
        int v = a > 0 ? a : 0;
        v += 1;
        agc.set("TGO", 0);
        agc.set("TGO+1", v);
        return v;
    }

    // ---- S40.1 ----

    /**
     * S40.1: compute initial thrust direction UT and initial VG (VGTIG). For a delta-V maneuver
     * (XDELVFLG) uses DELVSIN; otherwise derives VG from the Lambert/conic solution (S40.1B).
     */
    public void s401() {
        setDp("TIGSAVE", dp("TIG"));
        if (!flag("XDELVFLG")) {
            s401b();
            return;
        }
        double[] vtig = vec("VTIG");
        double[] rtig = vec("RTIG");
        setVec("VINIT", vtig);
        double[] up = unit(cross(vtig, rtig));
        setVec("UT", up);                              // "UP" held in UT
        setVec("RINIT", rtig);
        double rr = dot(rtig, rtig);                   // VSQ of RTIG (36D)
        double thetat = rr * THETACON * dp("DELVSAB") * dp("WEIGHT/G") / dp("F");
        double[] delvsin = vec("DELVSIN");
        double[] dvUp = scale(up, dot(delvsin, up));   // (DELTAV . UP) UP
        double[] dvp = sub(delvsin, dvUp);             // DELTA VP
        double[] t1 = scale(unit(cross(dvp, up)), Math.sin(thetat / 2));
        double[] vg = add(t1, scale(unit(dvp), Math.cos(thetat / 2) * abval(dvp)));
        vg = add(vg, dvUp);
        setVec("VGTIG", vg);                           // VG at ignition, 2^7 m/cs
        setVec("UT", unit(vg));                        // thrust direction
        getLvc(vg);
    }

    /** S40.1B: aim-point steering: extrapolate state to TIG, then INITVEL for VGTIG. */
    private void s401b() {
        setDp("TDEC1", dp("TIG"));
        setDp("DELLT4", dp("TPASS4") - dp("TIG"));     // intercept time - TIG
        extern("LEMPREC");
        double[] r = vec("RATT");
        double[] v = vec("VATT");
        setVec("RTIG", r);
        setVec("RINIT", r);
        setVec("UNIT/R/", unit(r));
        setVec("VTIG", v);
        setVec("VINIT", v);
        double eps = flag("NORMSW") ? EPS1 : EPS1 + EPS2;  // EPSILON4: 10 or 45 degrees
        agc.set("NUMIT", 0);
        setDp("EPSILON4", eps);
        extern("INITVEL");
        double[] vg = vec("DELVEET3");                // VGTIG = VR - VN
        setVec("VGTIG", vg);
        setVec("UT", unit(vg));
        getLvc(vg);
    }

    private void getLvc(double[] vg) {
        extern("GET.LVC", vg);                         // VGTIG in LV coordinates to DELVLVC
    }

    // ---- S40.2,3 ----

    /**
     * S40.2,3: preferred IMU orientation (wings-level heads-up): XSCREF = UT,
     * YSCREF = UNIT(UT x RTIG) (or UNIT(XSCREF x VTIG) if tiny), ZSCREF = XSCREF x YSCREF;
     * POINTVSM = REFSMMAT * UT; SCAXIS = +X; sets PFRATFLG.
     */
    public void s4023() {
        double[] ut = vec("UT");
        setVec("POINTVSM", scale(mxv("REFSMMAT", ut), 2));   // VSL1 rescale
        setVec("SCAXIS", new double[] {0.5, 0, 0});         // UNITX
        setVec("XSCREF", ut);
        double[] c = cross(ut, vec("RTIG"));
        double[] y;
        if (abval(c) == 0) {                                   // FIXY
            y = unit(cross(ut, vec("VTIG")));
        } else {
            y = unit(c);
        }
        setVec("YSCREF", y);
        setVec("ZSCREF", cross(ut, y));                        // = XSCREF x YSCREF
        upflag("PFRATFLG");
    }

    // ---- S40.8 ----

    /**
     * S40.8: update VG, compute TGO and call FINDCDUW. Sets alarm 1407 if VG and DELVREF are
     * more than 90 degrees apart.
     */
    public void s408() {
        if (!flag("XDELVFLG")) {
            rasteer1();
        }
        double[] vg = sub(vec("VGPREV"), vec("DELVREF"));     // VGAIN*
        setVec("VG", vg);
        setVec("UNFC/2", scale(mxv("REFSMMAT", vg), 2));
        setDp("VGDISP", abval(vg));
        setVec("VGPREV", vg);
        if (!flag("STEERSW")) {
            return;                                           // VG update only
        }
        double[] delv = unit(vec("DELVREF"));
        double d = dot(delv, vg);
        if (d < 0) {                                          // BPL ALARMIT
            alarmit();
            return;
        }
        // (VG . UNIT(DELVREF))/VEX + 1/2 ... scaled and combined with -FOURDT and TDECAY;
        // the fixed-point scaling shifts (SR 10D, DDV 36D) are folded into the double arithmetic.
        double tgo = (d / dp("VEX") + 0.5) * FOUR_DT + dp("TDECAY");
        setDp("TGO", tgo);
        setDp("TIG", tgo + dp("PIPTIME"));
        if (tgo - FOURSECS >= 0) {
            extern("FINDCDUW");
            return;
        }
        upflag("IMPULSW");
        downflag("STEERSW");
        extern("FINDCDUW");
    }

    private void alarmit() {
        extern("ALARM", 01407);
        extern("FINDCDUW");                                   // skip TGO computation
    }

    // ---- S40.13 ----

    /**
     * S40.13 (TIMEBURN): determine whether the burn is long enough to steer. Outputs IMPULSW
     * (0 = steer, 1 = attitude hold), NOTHROTL and TGO.
     */
    public void s4013() {
        downflag("IMPULSW");
        double vgmag = abval(vec("VGTIG"));
        // correct VG for 4 secs of 2-jet ullage
        double temp = vgmag - (dp("4SEC(17)") * dp("FRCS2") / dp("WEIGHT/G")) * 2;
        double tgo;
        if (!flag("APSFLAG")) {
            // DPS: s40.13D
            s4013d(temp);
            return;
        }
        upflag("NOTHROTL");
        double k1 = dp("K1VAL") / dp("WEIGHT/G");
        if (k1 - temp < 0) {
            // S40.131: TGO less than 100 CS
            double w = dp("WEIGHT/G") / 2;
            tgo = w + dp("K2VAL") / dp("K3VAL");
        } else {
            double t2 = dp("WEIGHT/G") - dp("MDOT") * SEC3_5;
            double t3 = dp("F") == 0 ? 1 : dp("F");
            tgo = SECS5 * (t2 / t3) + SEC1_2D;
        }
        upflag("IMPULSW");
        extern("TPAGREE");
        setDp("TGO", tgo);
        s40134();
    }

    private void s4013d(double vgCorrected) {
        double tgo = vgCorrected * dp("WEIGHT/G");
        if (flag("APSFLAG")) {
            tgo = tgo / dp("FAPS") * 4;                       // APSTGO
            storetgo(tgo);
            return;
        }
        downflag("NOTHROTL");
        double s40136 = dp("S40.136");
        tgo = tgo / (s40136 == 0 ? 1 : s40136);
        if (tgo - SECS6 >= 0) {
            tgo += SECS6;                                     // S40.127
            upflag("IMPULSW");
            storetgo(tgo);
            return;
        }
        // S40.138
        if (tgo - SECS89 < 0) {
            upflag("NOTHROTL");
        }
        storetgo(tgo);
    }

    private void storetgo(double tgo) {
        upflag("IMPULSW");
        setDp("TGO", tgo);
        s40134();
    }

    /** S40.134: restart protection then end of job. */
    public void s40134() {
        extern("PHASCHNG", 00003);
        extern("ENDOFJOB");
    }

    // ---- S40.9 / RASTEER1 ----

    /** S40.9: Lambert VTOGAIN every two computation cycles; sets AVFLAG. */
    public void s409() {
        upflag("AVFLAG");
        double eps = dp("HI6ZEROS") + (flag("NORMSW") ? EPS1 : EPS1 + EPS2); // 10 or 45 degrees
        setDp("EPSILON4", eps);
        extern("HAVEGUES");
        extern("PHASCHNG", 2);
        extern("ENDOFJOB");
    }

    /**
     * RASTEER1: generates VR and VG for aim-point maneuvers (Lambert-like targeting); leaves
     * DELVEET3 = VR - VN (+ oblateness term outside the Moon's sphere of influence).
     */
    public void rasteer1() {
        double[] rn = vec("RN");
        double rmag = abval(rn);
        double[] rtarg = vec("RTARG");
        double[] c = sub(rtarg, rn);
        double cmag = abval(c);
        double ss = (rmag + abval(rtarg) + cmag) / 2;        // SS = (R1 + R2 + C)/2
        double mu = dp("MU/A");
        double muast = dp("MUASTEER");
        double a2 = Math.abs((ss - rmag) * (muast - ss * mu));
        double aSq = Math.sqrt(a2) * (flag("GEOMSGN") ? -1 : 1);
        double bTerm = Math.sqrt(Math.abs(ss - cmag) / (ss == 0 ? 1 : ss));
        double angle = Math.asin(Math.min(1, bTerm)) * TWO_PI_3;
        double b = Math.sqrt(Math.abs(muast)) / (Math.abs(angle) + 1e-30);
        double dt = dp("TPASS4") - dp("PIPTIME");
        b = Math.copySign(b, dt) * (flag("GEOMSGN") ? -1 : 1);
        double[] ic = c;
        double[] ur = vec("UNIT/R/");
        double[] vprime;
        if (!flag("NORMSW")) {
            vprime = add(scale(unit(sub(ic, ur)), b), scale(unit(add(ic, ur)), aSq));
        } else {
            vprime = add(scale(unit(ic), b), scale(ur, aSq));
        }
        setVec("VIPRIME", vprime);
        double[] dv = sub(vprime, vec("VN1"));
        // FIRSTTME: oblateness term outside lunar sphere (RTX2 == 0 means Earth)
        if (agc.get("RTX2") == 0) {
            double k = (dp("PIPTIME") - dp("GOBLTIME")) * dp("EARTHMU") / (rmag * rmag);
            dv = add(dv, scale(vec("UNITGOBL"), k));
        }
        setVec("DELVEET3", dv);
    }

    // ---- TRIMGIMB ----

    /**
     * TRIMGIMB (formerly S40.6): trim the DPS engine to minimize thrust/CG offset. Gimbal to
     * full +pitch and +roll, then run -pitch / -roll for PITTIME / ROLLTIME.
     */
    public void trimgimb() {
        downflag("GMBDRVSW");
        agc.set("CHAN12", agc.get("CHAN12") & ~PRIO5);        // turn off -pitch, -roll
        agc.set("CHAN12", agc.get("CHAN12") | EBANK5);        // turn on +pitch, +roll
        extern("FIXDELAY", 6000);                              // one minute to reach full travel
        agc.set("CHAN12", agc.get("CHAN12") & ~EBANK5);
        agc.set("CHAN12", agc.get("CHAN12") | PRIO5);          // turn on -pitch, -roll
        extern("TWIDDLE", "PITCHOFF", agc.get("PITTIME"));
        extern("VARDELAY", agc.get("ROLLTIME"));
        agc.set("CHAN12", agc.get("CHAN12") & ~BIT12);         // shut off roll
        rollover();
    }

    /** ROLLOVER: if both finished return to R03, else set GMBDRVSW. */
    public void rollover() {
        if ((agc.get("FLAGWRD6") & agc.get("GMBDRBIT")) == 0) {
            upflag("GMBDRVSW");                                // roll or pitch done, not both
            extern("TASKOVER");
            return;
        }
        extern("NOVAC", "TRIMDONE", 010);                      // return to R03
        extern("TASKOVER");
    }

    /** PITCHOFF: waitlist task shutting off -pitch. */
    public void pitchoff() {
        agc.set("CHAN12", agc.get("CHAN12") & ~BIT10);
        rollover();
    }

    // ---- S41.1 ----

    /**
     * S41.1: transform a vector (2^7 m/cs) from reference coordinates to LM body axes using
     * REFSMMAT and the current CDU angles (via CDU*SMNB), returning the body-axis vector.
     */
    public double[] s411(double[] vRef) {
        double[] sm = scale(mxv("REFSMMAT", vRef), 2);        // MXV VSL1
        extern("CDU*SMNB", sm);
        return sm;
    }
}
