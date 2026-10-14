package apollo.comanche055;

import apollo.agc.Agc;

/**
 * P20-P25.agc (Comanche 055) - rendezvous navigation programs and routines:
 * <ul>
 * <li>P20 (PROG20) rendezvous navigation, PIKUP20 / R52 / R61 tracking</li>
 * <li>P22 (PROG22) orbital navigation (landmark sightings), S22.1 and subroutines</li>
 * <li>P23 (cislunar navigation), R23 backup marking, R57 optics calibration</li>
 * <li>R21 (CSM marking), R22 (rendezvous tracking data processing, REND*), VHF range read (RANGERD / VHFREAD)</li>
 * <li>R61 / CRS61.1 / CRS61.2 / R63 / V89 (preferred tracking attitude), W-matrix helpers (INITIALW, 9DWTO6DW)</li>
 * <li>V67 (W matrix RMS display), HORIZ, RECT.1, LDPLANET, GOTOV56</li>
 * </ul>
 *
 * Interpretive vector arithmetic is modelled with double[] and the AGC scaling (B-n) is not reproduced; the
 * original labels are kept as method names (lowerCamelCase, punctuation removed). Erasable words go through
 * {@link Agc}; flags are kept as agc symbols "FLAG.<name>" via {@link #upflag}/{@link #downflag}. Calls into
 * other AGC programs (BANKCALL, FINDVAC, NOVAC, INTEGRV, INCORP1 ...) go through the {@link Services} hook.
 */
public class P20P25 {
    private final Agc agc;

    /** Display/job responses from GOFLASH-like routines: the three return slots after "TC BANKCALL". */
    public enum Response { TERMINATE, PROCEED, ENTER_RECYCLE }

    /** Calls into other AGC routines by name (BANKCALL, FINDVAC, NOVAC, interpretive CALLs...). */
    public interface Services {
        void call(String routine);
        Response display(String routine, String verbNoun);
        void fork(String job, int priority, String routine);
        void phaschng(int... octal);
    }

    private Services services = new Services() {
        @Override public void call(String routine) { }
        @Override public Response display(String routine, String verbNoun) { return Response.PROCEED; }
        @Override public void fork(String job, int priority, String routine) { }
        @Override public void phaschng(int... octal) { }
    };

    public void setServices(Services s) { this.services = s; }

    public P20P25(Agc agc) {
        this.agc = agc;
    }

    // ---------------------------------------------------------------- constants
    public static final String V06N45B = "0645", V05N7022 = "00570", V05N7122 = "00571";
    public static final String V06N87NB = "0687", V59NB = "5900", V51NB = "5100", V53 = "5300", V0694 = "0694";
    public static final String V06N49 = "0649", V06N49EE = "00649", V06N89B = "00689", VB04N06 = "0406", VB06N18 = "0618";
    public static final String V05N70 = "0570", V05N71 = "0571", V6N49 = "0649", V06N99A = "0699", VNPLAN23 = "0688";
    public static final int V50N25P = 0202;
    public static final int OCT203 = 0203;
    public static final int OCT17 = 017, OC40200 = 040200, OCT00077 = 077, SPSIX = 6;
    public static final int NEG37 = -37;
    public static final int BITS7_9 = 0700, BITS4_6 = 070;
    public static final int N19_77DEG = 061740; // 19.77DEG: fixed trunnion bias
    /** SXTVAR / IMUVAR: sextant and IMU error variance .04 (mr)^2. */
    public static final double SXTVAR = 0.04e-6, IMUVAR = 0.04e-6;
    public static final double SCTVAR = 1.0e-6, IMUVARR = 0.04e-6;
    public static final double RANGEB14 = 303431.7; // 16384 x 18.52 m
    public static final double CONVRNGE = 18.52;    // VHF input range .01 NM -> meters
    public static final double DEGREE10 = 0.05556;  // 10 deg in revs
    public static final double RVCS_RDS = 15.915494; // 100/2PI rev-csec/rad-sec
    public static final double TENTH = 0.1;
    public static final double POINT8 = 0.8;
    public static final double PI_4 = 0.785398164;  // PI/4.0
    public static final double[] PRFUNIT = {0.40957602, 0.0, 0.28678822}; // 55 deg track axis
    public static final double[] JCAXIS = {0.2688190620, 0.0, 0.4215878460};
    /** MBDYTCTL: body to control axes rotation (7.25 deg negative X-axis rotation). */
    public static final double[][] MBDYTCTL = {
        {1, 0, 0}, {0, 0.99200495, -0.12619897}, {0, 0.12619897, 0.99200495}};
    public static final double ONE_C = 0.333564049e-6;
    public static final double AEARTH = 6378166, RADMOON = 1738090;
    public static final double TRUNVAR = 2.5e-9, VARSUBL3 = 3.4299040e6;
    public static final int TRUN19 = 01604;
    public static final double DP1MIN = 6000;       // 1 minute in cs
    public static final double FT99999 = 30479;
    public static final double V67DEC2 = 2;
    public static final int NINE_DW26D = 52, NINE_DWID = 2;

    // ------------------------------------------------------------ small helpers
    private int g(String s) { return agc.get(s); }
    private void s(String sym, int v) { agc.set(sym, v); }
    private boolean flag(String n) { return agc.get("FLAG." + n) != 0; }
    private void upflag(String n) { agc.set("FLAG." + n, 1); }
    private void downflag(String n) { agc.set("FLAG." + n, 0); }

    private static double[] vsub(double[] a, double[] b) { return new double[] {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
    private static double[] vadd(double[] a, double[] b) { return new double[] {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
    private static double[] vxsc(double[] a, double k) { return new double[] {a[0] * k, a[1] * k, a[2] * k}; }
    private static double dot(double[] a, double[] b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
    private static double abval(double[] a) { return Math.sqrt(dot(a, a)); }
    private static double[] unit(double[] a) { double m = abval(a); return m == 0 ? a : vxsc(a, 1 / m); }
    private static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }

    // --------------------------------------------------- interpretive state
    public double marktime, vhftime, vhfrange;
    public double[] rclp = new double[3], usstar = new double[3], ustar = new double[3];
    public double[] bvector = new double[9];
    public double deltaq, variance, tempor1;
    public double[] csmpos = new double[3], lempos = new double[3];
    public double[] um = new double[3], ucl = new double[3];
    public double[] rcvcsm = new double[3], rcvlem = new double[3], deltacsm = new double[3], deltalem = new double[3];
    public double[] deltax = new double[6];
    public double[] n49disp = new double[3];
    public double[] rzc = new double[3], vzc = new double[3];
    public double[][] w = new double[9][9];
    public double p21time;
    public double[] savepos = new double[3], savevel = new double[3];
    public double[] pointvsm = new double[3], thetad = new double[3], savedcdu = new double[3];
    public double[] cphix = new double[3], praxis = new double[3];

    // ================================================================ P20
    /** PROG20: rendezvous navigation program. */
    public void prog20() {
        services.call("R02BOTH");            // IMU status check
        s("TRKMKCNT", 0);                    // zero REND tracking mark counter
        s("VHFCNT", 0);                      // zero REND VHF range mark counter
        upflag("PRFTRKAT");                  // preferred tracking attitude flag
        downflag("VEHUPFLG");                // LEM to be updated
        upflag("TRACKFLG");
        upflag("UPDATFLG");
        upflag("RNDVZFLG");
        services.phaschng(4, 05022, 026000); // 2PHSCHNG
        marktime = loadtime();
        setintg();                           // set integration flags
        if (flag("RENDWFLG")) {
            upflag("DIM0FLAG");              // integrate the W matrix
        }
        if (flag("VEHUPFLG")) {
            downflag("VINTFLAG");            // set for LM integration
        }
        services.call("INTEGRV");
        services.call("GRP2PC");
        setintg();
        if (flag("VEHUPFLG")) {
            downflag("VINTFLAG");
        }
        services.call("INTEGRV");
        services.fork("R22", 026, "R22");    // CAF PRIO26 / FINDVAC R22
        services.phaschng(072, 0111);        // 2PHSCHNG
        pikup20();
    }

    /** PIKUP20: allow higher priority than Lambert, start R61 and R52. */
    public void pikup20() {
        services.call("PRIOCHNG:PRIO14");
        if (!flag("TRACKFLG") || !flag("REFSMFLG")) {
            return; // ENDOFJOB
        }
        s("R61CNTR", 0);
        services.call("R61CSM");
        upflag("TARG1FLG");                  // set target flag to LEM
        p20r52jb();
    }

    /** P20R52JB: set up auto optics job (R52), MKRLEES, hold present attitude. */
    public void p20r52jb() {
        services.call("R52");
        services.call("MKRLEES");
        s("HOLDFLAG", 1);                    // hold present attitude
        // TC ENDOFJOB
    }

    // ================================================================ P22
    /** PROG22: orbital navigation. Computes the angle between Y and V x R (stable member) and displays V06N45. */
    public void prog22() {
        downflag("RNDVZFLG");
        services.call("R02BOTH");
        double t = loadtime();
        // CSMCONIC integrates to present time (TDEC1 = t); VATT, RATT result
        double[] vatt = new double[3], ratt = new double[3];
        double[] vxr = unit(cross(vatt, ratt));
        double[] refsmmatRow = new double[] {0, 0, 0}; // REFSMMAT +6 (second row)
        double mga = Math.acos(Math.min(1, Math.abs(dot(vxr, refsmmatRow)) * 2)); // ABS; SL1; ARCCOS
        s("+MGA", Agc.fromSigned((int) (mga * 16384)));
        downflag("RENDWFLG");
        Response r = services.display("GOFLASHR", V06N45B);
        if (r == Response.TERMINATE) {
            gotopooh();
        } else if (r == Response.PROCEED) {
            prog22a();
        } else {
            prog22(); // ENTER: TC -5 redisplay
        }
    }

    /** PROG22A: select landmark, LUNAFLAG for Moon, R52 sighting. */
    public void prog22a() {
        s("LANDMARK", g("LANDMARK") & ~07000 & Agc.WORD_MASK); // result 70777: offset number = 0
        downflag("P22MKFLG");
        upflag("ERADFLAG");
        if (flag("CMOONFLG")) {
            upflag("LUNAFLAG");                // Moon
            s22n7071(V05N7022, true);          // MPAC=V05N70, MPAC+1 non-zero for N70
            callr52();
        } else {
            downflag("LUNAFLAG");              // Earth orbit
            upflag("KNOWNFLG");
            p22subrb();                        // lat/long/alt from astronaut
            callr52();
        }
    }

    /** CALLR52: phase change, MARKINDX = 5, TARG2FLG for landmark sighting, R52, then V05N71. */
    public void callr52() {
        services.phaschng(4, 05022, 013000);
        s("MARKINDX", 5);
        upflag("TARG2FLG");
        downflag("TARG1FLG");
        services.call("R52");
        dov5n71();
    }

    /** DOV5N71: display N71 (offset=0), then S22.1 to establish landmark. */
    public void dov5n71() {
        s22n7071(V05N7122, false);           // MPAC+1 = 0 for N71
        // PROG22C: LXC,2 MARKSTAT ; 8NN = QPRET,2
        s("8NN", g("QPRET"));
        s22_1();
        p22over();
    }

    /** P22OVER: phase change then back to point A (PROG22A). */
    public void p22over() {
        services.phaschng(04022);
        prog22a();
    }

    /** S22LSITE: convert RLS from moon-fixed to basic reference (lat/long/alt from landing site). */
    public void s22lsite() {
        // RP-TO-R(RLS, t); VSR2 -> ALPHAV; LAT-LONG at present time
        services.call("RP-TO-R");
        services.call("LAT-LONG");
        s22toff();
    }

    /** OBTAINLL: get lat/long/alt from astronaut (N89). */
    public void obtainll() {
        p22subrb();
        s22toff();
    }

    /** P22SUBRB: V06N89 load lat/long/alt; test |lat|<=90deg and |long|<=90deg (redisplay on error). */
    public void p22subrb() {
        while (true) {
            Response r = services.display("GOFLASH", V06N89B);
            if (r == Response.TERMINATE) {
                gotopooh();
                return;
            }
            if (r == Response.ENTER_RECYCLE) {
                continue; // TC -5
            }
            double lat = Agc.toSigned(g("LANDLAT")), lon = Agc.toSigned(g("LANDLONG"));
            double quarter = 0x2000; // DP1/4TH
            if (Math.abs(lat) > quarter || Math.abs(lon) > quarter) {
                services.call("FALTON"); // alarm, redisplay
                continue;
            }
            llasrda();
            return;
        }
    }

    /** S22N7071: store 8KK, flash V05N70/V05N71 and validate landmark designators. */
    public void s22n7071(String vn, boolean isN70) {
        s("8KK", isN70 ? 1 : 0);
        s("8KK+1", isN70 ? 1 : 0);
        s22dsppa(vn);
    }

    private void s22dsppa(String vn) {
        while (true) {
            Response r = services.display("GOFLASHR", vn);
            if (r == Response.TERMINATE) {
                gotopooh();
                return;
            }
            if (r == Response.ENTER_RECYCLE) {
                continue; // V32E recycle
            }
            // V33E proceed -> fall to landmark validation. (immediate return would blank R1,R3)
            int landmark = g("LANDMARK");
            int cxoff = landmark & 07000; // 08000 = offset indicator
            s("CXOFF", cxoff);
            if (cxoff - 05000 > 0) { // OFF=0 thru 5 ok
                continue; // S22DSPP: illegal, redisplay
            }
            downflag("KNOWNFLG");
            int a = landmark & 070000; // 'A' digit of ABCDE
            if (a == 0 || a > 020000) {
                services.call("FALTON");
                continue;
            }
            int aDigit = a >> 12;
            if (aDigit == 3) { // A=3 illegal
                services.call("FALTON");
                continue;
            }
            if (aDigit == 1) { // known landmark: check DE
                upflag("KNOWNFLG");
                int de = landmark & 077; // 000DE
                if (de <= 1) {
                    s22abde();
                    return;
                }
                if (g("8KK+1") == 0) { // N71: DE must be 0 or 1
                    services.call("FALTON");
                    continue;
                }
                if ((de & 070) != 050) { // N70: DE = 50-57 for advanced orbit
                    services.call("FALTON");
                    continue;
                }
                deGr50();
                return;
            }
            s22abde(); // A=2 unknown landmark, DE meaningless
            return;
        }
    }

    /** S22ABDE: unknown landmark -> exit; known -> RLS or astronaut lat/long. */
    public void s22abde() {
        if (!flag("KNOWNFLG")) {
            s22toff();
            return;
        }
        int de = g("LANDMARK") & 077;
        if (de == 0) {
            obtainll();    // get lat/long/alt from astronaut, N89
        } else {
            s22lsite();    // from RLS
        }
    }

    /** DE-GR-50: advanced orbit; do ADVORB then V05N71. */
    public void deGr50() {
        services.phaschng(4, 05022, 013000);
        s("MARKINDX", 5);
        upflag("TARG2FLG");
        downflag("TARG1FLG");
        services.call("ADVORB");
        dov5n71();
    }

    private void s22toff() {
        // GOTO S22RTNEX (return through QPRET/S22RTNEX)
    }

    // ================================================================ R57
    /** R57: optics calibration (EBANK7). Aborts if marking system or extended verb is already active. */
    public void r57() {
        if ((g("EXTVBACT") & 06) != 0) {
            p23abrt();
            return;
        }
        s("EXTVBACT", g("EXTVBACT") | 02);   // bit2 = marking system in use
        r57a();
    }

    /** R57A: set V59FLAG and ask for the calibration mark. */
    public void r57a() {
        upflag("V59FLAG");
        Response r = services.display("GOMARKFR", V59NB);
        if (r == Response.TERMINATE) {
            gotopooh();
        } else if (r == Response.PROCEED || r == Response.ENTER_RECYCLE) {
            endr57();
        }
    }

    /** MARKDISP: V06N87 display of trunnion and shaft data. */
    public void markdisp() {
        Response r = services.display("GOMARKFR", V06N87NB);
        if (r == Response.TERMINATE) {
            gotopooh();
        } else if (r == Response.PROCEED) {
            r57b();
        } else {
            r57a(); // ENTER (recycle)
        }
    }

    /** R57B: TRUNBIAS = MRKBUF1+5 (trunnion) - 19.77 deg, converting 2's to 1's complement. */
    public void r57b() {
        int trunion = g("MRKBUF1+5");
        int l = N19_77DEG;
        s("TRUNBIAS", Agc.add(trunion, ~l & Agc.WORD_MASK));
        endr57();
    }

    /** ENDR57: reset V59FLAG, set EBANK5, start ENDMARK job so R53 can take over. */
    public void endr57() {
        downflag("V59FLAG");
        services.fork("ENDMARK", 014, "ENDMARK");
        // GOTO EGRESS
    }

    /** P23ABRT: BAILOUT with alarm 01211. */
    public void p23abrt() {
        services.call("BAILOUT:01211");
    }

    // ================================================================ R21 / R23 marking
    /** R21CSM: set R21MARK and fall into R23CSM. */
    public void r21csm() {
        upflag("R21MARK");
        r23csm();
    }

    /** R23CSM: clear mark buffers; backup marking (R23FLG) or regular R21 marking. */
    public void r23csm() {
        s("MRKBUF1", Agc.fromSigned(-1));
        s("MRKBUF2", Agc.fromSigned(-1));
        if (!flag("R23FLG")) {
            r21c1();
            return;
        }
        Response r = services.display("GOMARKF", V0694); // display shaft + trunnion
        if (r == Response.TERMINATE) {
            r21end();
            return;
        }
        r23csm1();
    }

    /** R23CSM1: alt lost-sight mark (V53). */
    public void r23csm1() {
        while (true) {
            Response r = services.display("GOMARK2", V53);
            if (r == Response.TERMINATE) {
                r21end();
                return;
            }
            if (r == Response.PROCEED) {
                r21csma();
                return;
            }
            // transfer MRKBUF1 to MRKBUF2 (GENTRAN, 6 words); read time and CDUs into MRKBUF1
            for (int i = 0; i <= 6; i++) {
                s("MRKBUF2+" + i, g("MRKBUF1+" + i));
            }
            s("MRKBUF1", g("TIME2"));
            s("MRKBUF1+1", g("TIME1"));
            s("MRKBUF1+2", g("CDUY"));
            s("MRKBUF1+4", g("CDUZ"));
            s("MRKBUF1+6", g("CDUX"));
        }
    }

    /** R21C1: regular R21 marking via V51. */
    public void r21c1() {
        while (true) {
            Response r = services.display("GOMARK2", V51NB);
            if (r == Response.TERMINATE) {
                r21end();
                return;
            }
            if (r == Response.PROCEED) {
                r21csma();
                return;
            }
        }
    }

    /** R21CSMA: if a mark was made (MRKBUF1 != -1) transfer MRKBUF1 to MRKBUF2. */
    public void r21csma() {
        if (Agc.add(g("MRKBUF1"), 1) == 0 || Agc.add(g("MRKBUF1"), 1) == Agc.WORD_MASK) {
            r21end(); // no mark
            return;
        }
        for (int i = 0; i <= 6; i++) {
            s("MRKBUF2+" + i, g("MRKBUF1+" + i));
        }
        r21end();
    }

    /** R21END: KLEENEX, reset R21MARK, ENDMARK. */
    public void r21end() {
        services.call("KLEENEX");
        downflag("R21MARK");
        services.call("ENDMARK");
    }

    // ================================================================ R61
    /** R61CSM: preferred tracking attitude; R61CNTR counts four passes. */
    public void r61csm() {
        int savbnk = g("EBANK"); // switch to EBANK 6, save EBANK
        s("EBANK", 6);
        int cntr = Agc.toSigned(g("R61CNTR"));
        if (cntr > 0) {
            decrm61(cntr - 1, savbnk); // not ready to do R61
            return;
        }
        if (cntr < 0) { // -1: KALCMANU done earlier
            decrm61(cntr + 0, savbnk);
            return;
        }
        // CALL CRS61.1: LOS determination + vehicle attitude; MPAC index selects return
        int mpac = crs61_1();
        if (mpac == 0) {
            r61end(savbnk); // subroutine driving DAP (exit R61)
            return;
        }
        downflag("3AXISFLG");
        s("R61CNTR", Agc.fromSigned(-1)); // negative to indicate KALCMANU
        upflag("PDSPFLAG");
        services.call("R60CSM");
        downflag("PDSPFLAG");
        services.phaschng(0111);
        decrm61(0, savbnk);
    }

    private void r61end(int savbnk) {
        decrm61(3, savbnk);
    }

    private void decrm61(int newCount, int savbnk) {
        s("R61CNTR", Agc.fromSigned(newCount));
        s("EBANK", savbnk); // restore EBANK, return via BANKJUMP
    }

    /**
     * CRS61.1: compute DCDU(T) (R63 pass 1), check auto mode / DAP / STIKFLAG, ten degree gimbal check
     * (STEP2CK); if all OK compute CDU increments and LOS rate (CRS61.2) and load the DAP. Returns MPAC:
     * 0 = exit R61 (ASET), 1 = special return to caller for KALCMANU (LOONE).
     */
    public int crs61_1() {
        p21time = loadtime();
        r63();
        for (int i = 0; i < 3; i++) {
            savedcdu[i] = thetad[i];
        }
        // STEP2CK: UPACTOFF; check |ACDU - DCDU| <= 10 deg on all three axes
        services.call("UPACTOFF");
        String[] cdu = {"CDUX", "CDUY", "CDUZ"};
        for (int i = 2; i >= 0; i--) {
            double diff = Math.abs(Agc.toSigned(g(cdu[i])) - thetad[i]) / 16384.0 * 0.5; // revs
            if (diff - DEGREE10 > 0) {
                // STKTEST
                if (!flag("STIKFLAG")) {
                    return manuexis();
                }
                s("DSALMOUT", g("DSALMOUT") | 04); // turn on UPACTY light
                return aset();
            }
        }
        // AUTOCK: auto mode selected (CHAN31 bits 15-13 = 011)
        boolean autoMode = (((g("CHAN31") ^ 030000) & 070000) == 0);
        if (!autoMode) {
            return aset();
        }
        if (flag("STIKFLAG")) {
            return aset();       // someone on RHC
        }
        return crs61_2();
    }

    private int aset() {
        return 0;
    }

    /** MANUEXIS / MANUEXIT: ACDU-DCDU exceeds 10 degrees: save DCDU in CPHI, CTHETA, CPSI for KALCMANU. */
    private int manuexis() {
        s("CPHI", Agc.fromSigned((int) savedcdu[0]));
        s("CTHETA", Agc.fromSigned((int) savedcdu[1]));
        s("CPSI", Agc.fromSigned((int) savedcdu[2]));
        return 1; // MPAC+0 = 1 (LOONE): special return
    }

    /** CRS61.2 (STEP3CK): LOS rate OMEGTHSM, SMCDURES CDU increments, loads DAP variables. */
    private int crs61_2() {
        double[] dv = vsub(new double[3], savevel); // DV = VL - VC (DCDU holds VATT of LEM)
        double absdv = abval(dv);
        double[] udv = vxsc(unit(dv), -1);
        double[] cr = vxsc(cross(udv, savepos), RVCS_RDS); // (-UNITDV) x UNITLOS
        double quot = absdv / Math.max(p21time, 1e-30);   // ABSDV / ABSLOS
        double[] omegthsm = vxsc(cr, quot);
        double[] dtheta = vxsc(omegthsm, TENTH);           // DTHETASM
        // CDUTRIG; SMCDURES gives DELCDU; *SMNB* turns OMEGA into body rates, MBDYTCTL -> control axes, x POINT8
        double[] omegaBody = vxsc(omegthsm, POINT8);
        // INHINT; transfer desired gimbal angles and omega/DELCDU to the DAP
        s("CDUXD+1", 0);
        s("CDUYD+1", 0);
        s("CDUZD+1", 0);
        s("CDUXD", Agc.fromSigned((int) savedcdu[0]));
        s("CDUYD", Agc.fromSigned((int) savedcdu[1]));
        s("CDUZD", Agc.fromSigned((int) savedcdu[2]));
        for (int i = 0; i < 3; i++) {
            double ob = 0;
            for (int j = 0; j < 3; j++) {
                ob += MBDYTCTL[i][j] * omegaBody[j];
            }
            s("WBODY" + i, Agc.fromSigned((int) (ob * 16384)));
            s("DELCDU" + "XYZ".charAt(i), Agc.fromSigned((int) (dtheta[i] * 16384)));
        }
        s("HOLDFLAG", Agc.fromSigned(-1));   // now DAP variables loaded; set HOLDFLAG to -1
        return aset();
    }

    /** R63: calculate DCDU(T) for the CSM-LEM line of sight. */
    public void r63() {
        // CSMCONIC to P21TIME: hold RATT/VATT in SAVEPOS/SAVEVEL
        // LEMCONIC to same time; LOS = RL - RC; unit LOS into SAVEPOS; |LOS| into P21TIME
        double[] losvec = vsub(new double[3], savepos);
        savepos = unit(losvec);
        pointvsm = savepos.clone();          // * REFSMMAT
        p21time = abval(losvec);
        // VECPOINT with +X axis (UNITX) -> CPHIX (N96); PRFUNIT -> PRAXIS (N95)
        services.call("VECPOINT");
        thetad = flag("PRFTRKAT") ? praxis.clone() : cphix.clone(); // CRSTOR / CRSTOR1
    }

    // ================================================================ BVECTORS / GETUM
    /** BVECTORS: B0 = unit(USTAR x UCL...), DELTAQ = angle error times range, B1 = B2 = 0. */
    public void bvectors() {
        double[] uclUnit = unit(rclp);
        double rclpAbs = abval(rclp);
        double[] newUstar = unit(cross(ustar, uclUnit)); // USTAR = UNIT(US x UCL)
        System.arraycopy(newUstar, 0, bvector, 0, 3);
        services.call("GRP2PC");
        ustar = newUstar;
        double cosAngle = Math.max(-1, Math.min(1, dot(ustar, um) * 2));
        deltaq = (Math.acos(cosAngle) - Math.PI / 2) * PI_4 * rclpAbs; // ACOS, DSU DP1/4TH, DMP PI/4.0
        for (int i = 3; i < 9; i++) {
            bvector[i] = 0;
        }
    }

    /** GETUM: unit line-of-sight vector UM from sextant shaft/trunnion angles via SXTNB then NBSM, times REFSMMAT. */
    public double[] getum(double[] markdata) {
        // MARKDOWN downlink copy of the mark data; SXTNB -> NBSM -> VXM REFSMMAT, VSL1
        services.call("SXTNB");
        services.call("NBSM");
        return um;
    }

    // ================================================================ R22
    /** R22: rendezvous tracking data processing (state machine over the REND labels). */
    public void r22() {
        s("PHSPRDT2", 026);
        services.call("PRIOCHNG");
        s("MRKBUF2", Agc.fromSigned(-3));
        vhftime = loadtime();
        String label = "REND1";
        while (label != null) {
            switch (label) {
                case "REND1": services.call("GRP2PC"); waitone(); label = "REND1A"; break;
                case "REND1A": {
                    int m = Agc.toSigned(g("MRKBUF2"));
                    if (m == 0) { label = "REND2"; break; }
                    label = m < 0 ? "REND3A" : "REND2";
                    break;
                }
                case "REND2":
                    for (int i = 0; i <= 6; i++) s("MARKTIME+" + i, g("MRKBUF2+" + i));
                    s("MRKBUF2", Agc.fromSigned(-3)); // negative to indicate values used
                    downflag("SOURCFLG");             // 0 = optics data
                    services.call("GRP2PC");
                    label = "REND4";
                    break;
                case "REND3A": label = "REND3"; break;
                case "REND3":
                    services.call("GRP2PC");
                    waitone();
                    if (!flag("VHFRFLAG")) { label = "REND1A"; break; }
                    if (loadtime() - 6000 - vhftime < 0) { label = "REND1A"; break; } // 60SECDP
                    rangerd();
                    marktime = loadtime();
                    vhftime = marktime;
                    label = "REND4";
                    break;
                case "REND4":
                    setintg();
                    if (flag("VEHUPFLG")) {
                        label = "CSMUPP";
                        break;
                    }
                    services.call("INTEGRV");
                    services.call("GRP2PC");
                    setintg();
                    downflag("VINTFLAG");             // integration vehicle = LM
                    label = "REND5";
                    break;
                case "CSMUPP":
                    downflag("VINTFLAG");
                    services.call("INTEGRV");
                    services.call("GRP2PC");
                    setintg();
                    label = "REND5";
                    break;
                case "REND5":
                    if (flag("RENDWFLG")) upflag("DIM0FLAG"); // do not integrate W if flag = 0
                    services.call("INTEGRV");
                    shiftndx();
                    cmpos();
                    upflag("INCORFLG");               // set for 1st pass
                    lmpos();
                    downflag("ORBWFLAG");
                    if (!flag("RENDWFLG")) {
                        initialw(g("WRENDPOS"), g("WRENDVEL"));
                        s("VHFCNT", 0);
                        s("TRKMKCNT", 0);
                    }
                    upflag("RENDWFLG");
                    rclp = vsub(lempos, csmpos);      // LM - CSM
                    label = "REND7";
                    break;
                case "REND7":
                    if (flag("SOURCFLG")) { label = "REND14"; break; } // radar data
                    ucl = unit(rclp);
                    if (flag("INCORFLG")) {
                        um = getum(null);
                        double[] c = cross(ucl, um);
                        if (abval(c) == 0) { label = "REND3"; break; } // overflow: ignore mark
                        ustar = unit(c);
                    }
                    label = "REND9";
                    break;
                case "REND9":
                    bvectors();
                    if (!flag("VEHUPFLG")) {
                        for (int i = 0; i < 3; i++) bvector[i] = -bvector[i];
                    }
                    services.call("GRP2PC");
                    if (flag("R23FLG")) { variance = ALTVAR_IMU(); label = "REND10"; break; }
                    variance = SXTVAR + IMUVAR;
                    label = "REND10";
                    break;
                case "REND10": {
                    double r = abval(rclp);
                    variance = variance * r * r; // scaled B-40
                    variance += g("INTVAR") / 16384.0;
                    if (!flag("SOURCFLG")) { /* not VHF radar */ }
                    downflag("DMENFLG");          // 6x6 W matrix
                    services.call("INCORP1");
                    services.call("GRP2PC");
                    if (!flag("INCORFLG")) { label = "REND12"; break; }
                    shiftndx();
                    double dr = abval(new double[] {deltax[0], deltax[1], deltax[2]});
                    double dv = abval(new double[] {deltax[3], deltax[4], deltax[5]});
                    n49disp[0] = dr;
                    n49disp[2] = dv;
                    if (dr > g("RMAX") || dv > g("VMAX")) { label = "RENDISP"; break; }
                    label = "REND12";
                    break;
                }
                case "REND12":
                    services.call("INCORP2");
                    if (flag("SOURCFLG")) { label = "REND16"; break; }
                    if (!flag("INCORFLG")) { label = "REND17"; break; }
                    shiftndx();
                    if (flag("VEHUPFLG")) cmpos(); else lmpos();
                    services.call("GRP2PC");
                    rclp = vsub(lempos, csmpos);
                    downflag("INCORFLG");
                    label = "REND7";                  // second pass this optics mark
                    break;
                case "REND14":
                    // VHF radar BVECTOR
                    for (int i = 0; i < 9; i++) bvector[i] = 0;
                    System.arraycopy(unit(rclp), 0, bvector, 0, 3);
                    deltaq = vhfrange - abval(rclp);
                    upflag("INCORFLG");
                    if (!flag("VEHUPFLG")) {
                        for (int i = 0; i < 3; i++) bvector[i] = -bvector[i];
                    }
                    services.call("GRP2PC");
                    variance = g("RVAR");
                    label = "REND10";
                    break;
                case "REND16":
                    s("VHFCNT", g("VHFCNT") + 1);
                    label = "REND1";
                    break;
                case "REND17":
                    s("TRKMKCNT", g("TRKMKCNT") + 1);
                    label = "REND3";
                    break;
                case "RENDISP":
                    label = rendisp();
                    break;
                default:
                    label = null;
            }
        }
    }

    private double ALTVAR_IMU() {
        return g("ALTVAR") / 16384.0 + IMUVAR;
    }

    /** RENDISP: V06N49 display of position/velocity update exceeding limits; RENDISP3 recycle path. */
    private String rendisp() {
        s("N49DISP+4", flag("SOURCFLG") ? 2 : 1);
        s("TEMPOR1", 0);
        services.phaschng(04022);
        services.fork("RENDISP2", 027, "RENDISP2");
        // RENDISP7: wait for answer in TEMPOR1: negative = proceed, positive = recycle
        int t = Agc.toSigned(g("TEMPOR1"));
        if (t < 0) {
            return "REND12";
        }
        return rendisp3();
    }

    /** RENDISP2: PRIODSP of V06N49; terminate exits P20 via V56; proceed sets TEMPOR1 = -1. */
    public void rendisp2() {
        Response r = services.display("PRIODSP", V06N49);
        if (r == Response.TERMINATE) {
            gotov56();
            return;
        }
        s("TEMPOR1", Agc.fromSigned(r == Response.PROCEED ? -1 : 1));
    }

    private String rendisp3() {
        if (flag("SOURCFLG")) {
            return "REND1"; // data was radar: look for optics next
        }
        s("MRKBUF1", Agc.fromSigned(-1));  // erase mark one buffer
        s("MRKBUF2", Agc.fromSigned(-1));  // erase mark two buffer
        return "REND3";
    }

    /** CMPOS: CSM position = RCVCSM + DELTACSM scaled by 2^-7 shifted. */
    public void cmpos() {
        csmpos = vadd(rcvcsm, vxsc(deltacsm, 1.0 / 128));
    }

    /** LMPOS: LM position. */
    public void lmpos() {
        lempos = vadd(rcvlem, vxsc(deltalem, 1.0 / 128));
    }

    /** SETINTG: reserve integration; set flags (STATEFLG, VINTFLAG, D6OR9FLG; clear INTYPFLG, DIM0FLAG). */
    public void setintg() {
        services.call("INTSTALL");
        s("TDEC1", (int) marktime);
        upflag("STATEFLG");
        downflag("INTYPFLG");
        downflag("DIM0FLAG");
        upflag("VINTFLAG");
        upflag("D6OR9FLG");
    }

    /** CNTCHK / CONTCHK / WAITONE: wait 4 sec while the tracking flags allow continuation. */
    public void cntchk() {
        contchk();
    }

    private void contchk() {
        if (!flag("REFSMFLG")) {
            return; // ENDPLAC
        }
        if (Agc.toSigned(g("R61CNTR")) < 0) {
            waitone1();
            return;
        }
        if (flag("UPDATFLG") && !flag("TRACKFLG")) {
            return; // ENDPLAC
        }
    }

    public void redor22() {
        services.phaschng(0132);
        services.call("PRIOCHNG:PRIO26");
        waitone1();
    }

    public void waitone() {
        waitone1();
    }

    private void waitone1() {
        services.call("DELAYJOB:4SECS");   // wait 4 seconds
        contchk();                          // check again now
    }

    /** RANGERD: read the VHF radar range; set SOURCFLG on success. */
    public void rangerd() {
        s("CHAN13", g("CHAN13") & ~OCT17);           // zero bits 1-4 of channel 13
        s("CHAN13", g("CHAN13") | 011);              // generate shift pulses to radar, set R bit
        marktime = g("TIME2");
        downflag("SKIPVHF");
        services.call("RADSTALL");                   // wait for range complete
        // bad data good bit => LIGHTON; good => TRFAILOF
        int rm = g("RM");
        if (rm == 0 || rm == Agc.WORD_MASK) {
            lighton();
            return;
        }
        if ((rm & 0x4000) == 0) {
            vhfrange = rm * CONVRNGE;                // convert range to meters (RANGERD4)
        } else {
            vhfrange = (rm & Agc.WORD_MASK >> 1 & 0x3FFF) * CONVRNGE + RANGEB14; // RANGERD3 adds sign bit value
        }
        upflag("SOURCFLG");                          // source of data = VHF radar
    }

    private void lighton() {
        if (vhfrange != 0) {
            services.call("TRFAILON");
        }
        vhftime = marktime;
        // GOTO REND1
    }

    /** VHFREAD: T4/radar interrupt service: save RNRAD in RM, test data good bit (CHAN33 bit 2). */
    public void vhfread() {
        s("RUPTAGN", Agc.WORD_MASK);
        if (flag("SKIPVHF")) {
            return; // TASKOVER
        }
        if (!flag("UPDATFLG") || !flag("PDSPFLAG")) {
            bypass1();
            return;
        }
        s("RM", g("RNRAD"));
        if ((g("CHAN33") & 02) == 0) { // data good
            services.call("GOODEND");
        } else {
            bypass(g("RNRAD"));
        }
    }

    private void bypass1() {
        bypass(0);
    }

    private void bypass(int v) {
        s("VHFRANGE", v);
        services.call("BADEND");
    }

    /** SHIFTNDX: scaling index for Earth (0) or Moon (-2) sphere of influence of the current vehicle. */
    public int shiftndx() {
        boolean moon = flag("VEHUPFLG") ? flag("CMOONFLG") : flag("LMOONFLG");
        int x2 = moon ? -2 : 0;
        s("X2", Agc.fromSigned(x2));
        return x2;
    }

    /** INITIALW: initialise the 6x6 W matrix diagonals (position, velocity values) and clear the rest. */
    public void initialw(double posVal, double velVal) {
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 9; j++) {
                w[i][j] = 0;
            }
        }
        for (int i = 0; i < 3; i++) {
            w[i][i] = posVal;
            w[i + 3][i + 3] = velVal;
        }
    }

    // ================================================================ S22.1 (landmark orbit determination)
    /** S22.1: store marks, integrate to each mark time, incorporate (INCORP1/2), loop for all marks. */
    public void s22_1() {
        s("S1", 6);
        s("8KK", 1);
        s("S22LOC", g("SVMRKDAT"));
        // copy mark data (5 sets) from MARKSTAT area to SVMRKDAT for restart protection
        upflag("P22MKFLG");
        services.call("MKRELEAS");
        services.phaschng(4, 05022, 013000);
        s("S22EORM", flag("CMOONFLG") ? -2 : 0);
        fig2exit();
    }

    /** FIG2EXIT: set up W for the first integration, then the per-mark loop (S22NXTIN). */
    private void fig2exit() {
        services.call("INTSTALL");
        s22flgs();
        if (!flag("ORBWFLAG")) { // SETWW5D: D=0 path, initialise W
            downflag("DIM0FLAG");
            downflag("RENDWFLG");
            initialw(g("WORBPOS"), g("WORBVEL"));
        } else {
            downflag("D6OR9FLG"); // setvandi path
        }
        downflag("DMENFLG");      // 0 = 6x6 W, 1 = 9x9 W
        s22nxtin();
    }

    /** S22NXTIN: integrate to time TF (GETTF); compute RC, UM; D = UM.RL; build U, W, E matrices; incorporate. */
    private void s22nxtin() {
        services.call("INTEGRV");   // to time returned by GETTF
        s22calrc();
        um = getum(null);
        int i = g("8KK");
        int off = (g("CXOFF") >> 6) & 7; // CXOFF scaled
        if (i == off && off != 0) {
            s22offEqI();
            return;
        }
        if (flag("DMENFLG")) {
            s22d9();
            return;
        }
        services.call("GRP2PC");
        upflag("ORBWFLAG");
        upflag("DMENFLG");   // = 0 on first pass for D=0 or 6
        upflag("22DSPFLG");  // = 1 display DR,DV on first pass
        upflag("ERADFLAG");  // compute Fischer radius
        if (!flag("KNOWNFLG")) {
            s22box22();
            return;
        }
        // KNOWN landmark: W = U*W, E matrix etc. (S22C=I ... S22SCLW): initial W construction
        double[] rc = csmpos;
        double[] alphav = unit(rc);
        services.call("SETRE");
        s22f2410();
        // U = I - (UM)(RL^T)/D ; W(I+18)=U*W(I); E = 1/2 rho^2 VARSCT U U^T + VARRP R0/D^2 UM UM^T
        // then Cholesky-like factors W72..W78 (S22W72X..S22SCLW) place the 3x3 position error factors
        services.call("S22UMRL:S22UUT:W+144");
        s22savet();
    }

    private void s22savet() {
        s("S22TPRIM", g("S22LOC")); // SAVE PRESENT TIME FOR PIOS
        s22iEqN();
    }

    /** S22I=N: test I = N; if not done step to next mark (I+1, S22LOC+7) and integrate again. */
    private void s22iEqN() {
        services.phaschng(04022);
        if (Agc.toSigned(g("8NN")) - Agc.toSigned(g("8KK")) <= 0) {
            s22f244x();
            return;
        }
        s("8KK", g("8KK") + 1);
        s("S22LOC", g("S22LOC") + 7);
        services.phaschng(04022);
        services.call("INTSTALL");
        s22flgs();
        if (flag("DMENFLG")) {
            s22nxtin();           // return always except offset point mark 1
        } else if (!flag("ORBWFLAG")) {
            upflag("DIM0FLAG");   // offset mark 1: no W integration
            s22nxtin();
        } else {
            downflag("D6OR9FLG"); // offset mark 1: integrate W 6x6
            s22nxtin();
        }
    }

    /** S22OFF=I: time sub OFF, U sub OFF; then test I=N. */
    private void s22offEqI() {
        s("S22TOFF", g("S22LOC"));
        s22iEqN();
    }

    /** S22D=9: 9-dimensional path: transform RL via R-TO-RP/RP-TO-R to time TF, form RCL, USTAR, variance, B vectors. */
    private void s22d9() {
        services.call("R-TO-RP");   // from t' to TF
        services.call("RP-TO-R");
        upflag("INCORFLG");
        // RCL = RL - RC ; USTAR = UNIT(UNIT(RCL) x UM)
        s22box12();
    }

    /** S22BOX12: variance and B vectors, INCORP1, optional N49 display, INCORP2. */
    private void s22box12() {
        upflag("DMENFLG");
        upflag("VEHUPFLG");
        variance = SCTVAR + IMUVARR;
        double r = abval(rclp);
        variance *= r * r;           // scaled B-40 for both Earth and Moon
        services.call("BVECTORS");   // calc B0, B1, DELTAQ, new USTAR; B2 = -B0
        services.call("INCORP1");
        services.call("GRP2PC");
        if (flag("22DSPFLG")) {
            downflag("22DSPFLG");
            services.call("GRP2PC");
            n49disp[0] = abval(new double[] {deltax[0], deltax[1], deltax[2]});
            n49disp[2] = abval(new double[] {deltax[3], deltax[4], deltax[5]});
            Response rsp = services.display("GOFLASHR", V06N49EE);
            if (rsp == Response.TERMINATE) {
                gotopooh();
                return;
            }
            if (rsp == Response.ENTER_RECYCLE) {
                s22exex();            // V32E recycle
                return;
            }
        }
        s22box42();
    }

    /** S22BOX42: INCORP2, recompute RC; if INCORFLG then next RCL, else TF test. */
    private void s22box42() {
        services.call("INCORP2");
        s22calrc();
        if (!flag("INCORFLG")) {
            s22savet();
            return;
        }
        services.call("GRP2PC");
        downflag("INCORFLG");
        // RCL = RL - RC with RL = X789
        s22box12();
    }

    /** S22BOX22: unknown landmark - clear W6..W8, set diagonal S22WSUBL, compute RL from lat/long (LALOTORV). */
    private void s22box22() {
        // CLRW678; W+144, +152, +160 = S22WSUBL
        downflag("LUNAFLAG");
        if (flag("CMOONFLG")) {
            upflag("LUNAFLAG");
        }
        services.call("LALOTORV");   // RL = f(lat, long, alt, TF); RL scaled B-27 for the Moon
        s22box32();
    }

    private void s22box32() {
        upflag("INCORFLG");
        // RCL = RL - RC; USTAR = UNIT(UNIT(RCL) x UM)
        s22box12();
    }

    /** S22F244X / S22F244: Figure 2.4-4: offset-point handling, then lat/long display (N89) and landing site save. */
    private void s22f244x() {
        if (g("CXOFF") == 0 || Agc.toSigned(g("8NN")) < (g("CXOFF") >> 9)) {
            s22box44();
            return;
        }
        services.call("GRP2PC");
        services.call("CSMPREC");  // RC at offset time
        s22bx44a();
    }

    private void s22box44() {
        s("S22TOFF", g("S22LOC"));  // present time for lat-long setup
        s22bx44a();
    }

    private void s22bx44a() {
        downflag("LUNAFLAG");
        if (flag("CMOONFLG")) {
            upflag("LUNAFLAG");
        }
        services.call("LAT-LONG");   // alt output always B-29
        llasrd();
        Response r = services.display("GOFLASH", V06N89B);
        if (r == Response.TERMINATE) {
            s22gtp();
        } else if (r == Response.PROCEED) {
            services.call("R-TO-RP");   // convert RLS to moon-fixed; store landing-site vector RLS
        } else {
            s22_981x();                  // recycle: point A in GSOP
        }
    }

    private void s22_981x() {
        ninedwto6dw();
        prog22a();
    }

    private void s22exex() {
        // GOTO S22RTNEX without converting W
    }

    private void s22gtp() {
        ninedwto6dw();   // convert W before GOTOPOOH
        gotopooh();
    }

    /**
     * S22F2410: formula 2.4.10: RL = RC + RHO * UM where RHO = (cosA - sqrt((R0/RC)^2 - sinA^2)) * |RC|,
     * cosA = -(UM.RC)/|RC|. Result stored in X789; RHO in S22RHO.
     */
    public double[] s22f2410() {
        double rcAbs = abval(csmpos);
        double[] urc = unit(csmpos);
        double cosA = -dot(um, urc);
        double sinA2 = 1 - cosA * cosA;
        double r0 = g("ERADM") / 16384.0;
        double ratio = r0 / rcAbs;
        double rho = (cosA - Math.sqrt(Math.max(0, ratio * ratio - sinA2))) * rcAbs;
        s("S22RHO", Agc.fromSigned((int) (rho * 16384)));
        double[] x789 = vadd(vxsc(um, rho), csmpos);
        return x789;
    }

    /** S22CALRC: RC = RCVCSM + DELTACSM scaled by the Earth/Moon index (S22EORM). */
    public void s22calrc() {
        csmpos = vadd(rcvcsm, vxsc(deltacsm, Math.pow(2, -7 - Agc.toSigned(g("S22EORM")))));
    }

    /** S2231X13: 3x1 times 1x3 outer product, stored as 3x3 in S22UMRL. */
    public double[][] s2231x13(double[] a3x1, double[] b1x3) {
        double[][] m = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                m[i][j] = a3x1[j] * b1x3[i];
            }
        }
        return m;
    }

    /** GETTF: MPAC = time TF at (S22LOC). */
    public double gettf() {
        return g("S22LOC");
    }

    /** S22FLGS: integration flags for S22. */
    public void s22flgs() {
        upflag("DIM0FLAG");
        upflag("D6OR9FLG");
        upflag("VINTFLAG");
        upflag("STATEFLG");
        downflag("INTYPFLG");
    }

    /** LLASRD: copy ALT, LONG (halved) to LANDALT, LANDLONG. */
    public void llasrd() {
        s("LANDALT", g("ALT"));
        s("LANDLONG", Agc.fromSigned(Agc.toSigned(g("LONG")) / 2));
    }

    /** LLASRDA: reverse of LLASRD. */
    public void llasrda() {
        s("ALT", g("LANDALT"));
        s("LONG", Agc.fromSigned(Agc.toSigned(g("LANDLONG")) * 2));
    }

    /**
     * 9DWTO6DW: convert the 9x9 lower-triangular W matrix to a 6x6: form E = W W^T (6x6 block of rows) and
     * Cholesky-factor E back into W (Figure 2.4-6); clears W6..W8 afterwards.
     */
    public void ninedwto6dw() {
        int n = 9;
        double[][] e = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j <= i; j++) {
                double sum = 0;
                for (int k = 0; k < n; k++) {
                    sum += w[i][k] * w[j][k];   // ROWDOT
                }
                e[i][j] = sum;
            }
        }
        services.call("GRP2PC");
        double[][] nw = new double[n][n];
        for (int j = 0; j < 6; j++) {
            for (int i = j; i < 6; i++) {
                double sum = e[i][j];
                for (int k = 0; k < j; k++) {
                    sum -= nw[i][k] * nw[j][k];
                }
                if (i == j) {
                    nw[i][j] = sum <= 0 ? 0 : Math.sqrt(sum);   // W = SQRT(WSQ) or 0
                } else {
                    nw[i][j] = nw[j][j] == 0 ? 0 : sum / nw[j][j];
                }
            }
        }
        services.call("GRP2PC");
        w = nw;                           // W6, W7, W8 (temp EMATRIX) cleared
    }

    // ================================================================ V89 / R63
    /** V89CALL: IMU status check; select tracking axis (V04N06) then maneuver per N18. */
    public void v89call() {
        services.call("R02BOTH");
        s("OPTION1", 3);
        s("OPTION1+1", 1);
        Response r = services.display("GOFLASH", VB04N06);
        if (r == Response.TERMINATE) { endext(); return; }
        while (r == Response.ENTER_RECYCLE) {
            r = services.display("GOFLASH", VB04N06);
            if (r == Response.TERMINATE) { endext(); return; }
        }
        if (Agc.add(Agc.fromSigned(-g("OPTION1+1")), 1) == 0) {
            setpaf();    // 1 = preferred axis
        } else {
            downflag("RNGSCFLG");   // X axis
            v89recl();
        }
    }

    private void setpaf() {
        upflag("RNGSCFLG");
        v89recl();
    }

    /** V89RECL: P21TIME = now + 1 min, R63COMP, display V06N18, maneuver with R60CSM. */
    private void v89recl() {
        while (true) {
            p21time = loadtime() + DP1MIN;
            r63();
            Response r = services.display("GOFLASH", VB06N18);
            if (r == Response.TERMINATE) { endext(); return; }
            if (r == Response.PROCEED) break;
        }
        downflag("3AXISFLG");
        services.call("R60CSM");
        endext();
    }

    private void endext() {
        services.call("ENDEXT");
    }

    // ================================================================ P23
    /** P23: cislunar midcourse navigation (star/landmark/horizon sighting). */
    public void p23() {
        downflag("RNDVZFLG");
        services.phaschng(0, 012);          // leave group 4, enter group 2
        s("PHSPRDT2", 013);
        s("MARKINDX", 1);
        downflag("TARG2FLG");
        downflag("TARG1FLG");
        s("STARIND", 0);
        s("BESTI", 0);
        downflag("R57FLAG");                // set = do not reperform R57
        downflag("V94FLAG");                // set = allow V94
        p23_00();
    }

    private void p23_00() {
        if (flag("REFSMFLG")) {
            p23_05();
        } else {
            r57();                           // optics calibration if IMU not aligned
            r53();
            p23_60();
        }
    }

    private void r53() {
        services.call("R53");
    }

    /** P23.05 / P23.06: identification display V05N70 (star/landmark, horizon). */
    private void p23_05() {
        downflag("SAVECFLG");
        p23_06();
    }

    private void p23_06() {
        Response r = services.display("GOFLASH", V05N70);
        if (r == Response.TERMINATE) { gotopooh(); return; }
        if (r == Response.ENTER_RECYCLE) { p23_06(); return; }
        p23_15();
    }

    /** P23.15: validate LANDMARK / HORIZON (C = 1 or 2) and STARCODE, set LUNAFLAG / NORFHOR. */
    private void p23_15() {
        int landmark = g("LANDMARK"), horizon = g("HORIZON");
        int c;
        if (landmark != 0) {
            if (horizon != 0) { r23_10(); return; }   // operator DSKY error
            c = landmark;
        } else {
            if (horizon == 0) { r23_10(); return; }
            c = horizon;
        }
        int cc = c & BITS7_9;
        if (cc == 0100) {
            downflag("LUNAFLAG");                      // C = 1
        } else if (cc == 0200) {
            upflag("LUNAFLAG");                        // C = 2
        } else {
            r23_10();
            return;
        }
        int star = Agc.toSigned(g("STARCODE"));
        if (star == 0) { p23_176(); return; }
        if (star < 0 || star > 37) { r23_10(); return; }
        // P23.17: STARSAV2 = star vector from catalog, BESTI = 6 x star number
        s("BESTI", 6 * star);
        s("STARSAV2", star);
        p23_176();
    }

    private void p23_176() {
        int horizon = g("HORIZON");
        if (horizon == 0) {
            p23_20();
            return;
        }
        int h = horizon & BITS4_6;
        if (h == 010) {
            downflag("NORFHOR");
        } else if (h == 020) {
            upflag("NORFHOR");
        } else {
            r23_10();
            return;
        }
        p23_30();
    }

    private void p23_20() {
        p22subrb();      // lat/long/alt from astronaut
        p23_30();
    }

    /** P23.30: planet (STARCODE=0) uses LDPLANET to get the vector to the planet. */
    private void p23_30() {
        if (g("STARCODE") == 0) {
            ldplanet();
        }
        p23_31();
    }

    private void p23_31() {
        if (flag("SAVECFLG")) {
            p23_85();
            return;
        }
        Response r = services.display("GOPERF1", "00202");   // V50N25: please perform
        if (r == Response.TERMINATE) { gotopooh(); return; }
        v94enter();
    }

    /** V94ENTER: point CSM at the target (POINTAXS gives LOS), maneuver with R60CSM, then R57 and R52. */
    public void v94enter() {
        marktime = loadtime();
        pointaxs();
        downflag("3AXISFLG");
        services.call("R60CSM");
        services.phaschng(04022);
        if (!flag("R57FLAG")) {
            r57();
        }
        upflag("V94FLAG");
        upflag("R57FLAG");
        services.call("R52");
        downflag("V94FLAG");
        downflag("R57FLAG");
        p23_60();
    }

    /** P23.60: read mark data (MARKSTAT), save MARKTIME/TRUNION, display V05N71, store data (P23.65). */
    private void p23_60() {
        s("MARKDATA", g("MARKSTAT") & 01777);
        marktime = g("MARKTIME");
        s("TRUNION", g("MARKDATA+5"));
        Response r = services.display("GOFLASH", V05N71);
        if (r == Response.TERMINATE) { gotopooh(); return; }
        if (r == Response.ENTER_RECYCLE) { p23_60(); return; }
        p23_65();
    }

    private void p23_65() {
        upflag("SAVECFLG");
        p23_15();
    }

    private void p23_85() {
        downflag("RENDWFLG");
        pointaxs();
        r23_55();
    }

    /** POINTAXS / R23.05..R23.5: integrate CSM to MARKTIME, compute LOS RCLL to landmark or horizon. */
    public double[] pointaxs() {
        if (!flag("ORBWFLAG")) {
            initialw(g("WMIDPOS"), g("WMIDVEL"));
        }
        setintg();
        if (flag("ORBWFLAG")) {
            upflag("DIM0FLAG");
        }
        upflag("ORBWFLAG");
        services.call("INTEGRV");       // integrate CSM state vector to MARKTIME
        services.phaschng(04022);
        rect1();
        if (flag("ZMEASURE")) {
            services.call("LUNPOS");
            rzc = flag("CMOONFLG") ? vsub(rzc, new double[3]) : vadd(rzc, new double[3]);
        }
        double[] rcll;
        if (g("LANDMARK") != 0) {
            upflag("ERADFLAG");
            services.call("LALOTORV");
            rcll = new double[3];
        } else {
            rcll = horiz(rzc, new double[3]);
        }
        return vsub(rcll, rzc);
    }

    /** R23.55: build B vectors and DELTAQ from the star/landmark line of sight and the measured trunnion angle. */
    private void r23_55() {
        double[] rcll = new double[3];
        double[] uclstar = unit(vadd(unit(rcll), vxsc(vzcRel(), ONE_C)));
        double[] us = new double[3];
        double[] usstar = unit(vadd(us, vxsc(vzcRel(), ONE_C)));
        double cosq = dot(usstar, uclstar);
        double[] b0 = unit(vsub(usstar, vxsc(uclstar, cosq)));   // USSTAR - COSQ(UCLSTAR)
        System.arraycopy(b0, 0, bvector, 0, 3);
        for (int i = 3; i < 9; i++) bvector[i] = 0;
        double ang = Math.acos(Math.max(-1, Math.min(1, cosq)));
        double trunion = Agc.toSigned(g("TRUNION") - 0) / 16384.0; // 2's to 1's conversion with VARSUBL = 0
        deltaq = (ang - (trunion - g("TRUNBIAS") / 16384.0 + TRUN19 / 16384.0)) * PI_4;
        variance = TRUNVAR + VARSUBL3 * 0;
        downflag("DMENFLG");
        services.call("INCORP1");
        services.phaschng(04022);
        n49disp[2] = abval(new double[] {deltax[3], deltax[4], deltax[5]});
        n49disp[0] = abval(new double[] {deltax[0], deltax[1], deltax[2]});
        r23_6();
    }

    private double[] vzcRel() {
        return vzc;
    }

    /** R23.6: V06N49 display; proceed incorporates the data (R23.7), otherwise terminate. */
    private void r23_6() {
        Response r = services.display("GOFLASHR", V6N49);
        if (r == Response.TERMINATE) { gotopooh(); return; }
        if (r == Response.PROCEED) { r23_7(); return; }
        gotopooh();
    }

    private void r23_7() {
        upflag("VEHUPFLG");
        services.call("INCORP2");
        r23End();
    }

    private void r23End() {
        gotopooh();
    }

    private void r23_10() {
        services.call("FALTON");
        p23_06();
    }

    /**
     * HORIZ: horizon subroutine: given the CSM position RZC and star vector US, pick the point of tangency T
     * on the Earth (Fischer ellipsoid) or Moon horizon (RADMOON). Returns the line-of-sight target vector.
     */
    public double[] horiz(double[] rzcv, double[] us) {
        double[] ubar2 = unit(cross(us, rzcv));
        double[] ubar0 = unit(cross(ubar2, us));
        double[] ubar1 = unit(cross(ubar2, ubar0));
        double ah, bh;
        double horizalt = g("HORIZALT") / 16384.0;
        if (flag("LUNAFLAG")) {
            ah = RADMOON;
            bh = RADMOON;
        } else {
            double rad = AEARTH;             // radius of Fischer ellipsoid via GETERAD
            ah = AEARTH + horizalt;
            bh = rad + horizalt;
        }
        double[] rh = {dot(rzcv, ubar0), dot(rzcv, ubar1), dot(rzcv, ubar2)}; // RH in (XH,YH,ZH)
        double[] ush = {dot(us, ubar0), dot(us, ubar1), dot(us, ubar2)};
        double xh = rh[0], yh = rh[1];
        double a = (xh / ah) * (xh / ah) + (yh / bh) * (yh / bh);
        double sqa = Math.sqrt(Math.max(0, a - 1));
        double alpha = (xh / a) / ah * 0;
        double xa = xh / a, ya = yh / a;
        double t0x = xa + (ah / bh) * (sqa / a) * yh;
        double t0y = ya - (bh / ah) * (sqa / a) * xh;
        double t1x = xa - (ah / bh) * (sqa / a) * yh;
        double t1y = ya + (bh / ah) * (sqa / a) * xh;
        double[] t0 = {t0x, t0y, 0};
        double[] t1 = {t1x, t1y, 0};
        double a0 = dot(ush, unit(vsub(t0, rh)));
        double a1 = dot(ush, unit(vsub(t1, rh)));
        double[] t = (a0 - a1 < 0) == flag("NORFHOR") ? t1 : t0;
        if (a1 - a0 >= 0 && !flag("NORFHOR")) t = t0;
        // VXM UBAR0 back to reference coordinates (rows are the UBAR vectors)
        return new double[] {
            t[0] * ubar0[0] + t[1] * ubar1[0] + t[2] * ubar2[0],
            t[0] * ubar0[1] + t[1] * ubar1[1] + t[2] * ubar2[1],
            t[0] * ubar0[2] + t[1] * ubar1[2] + t[2] * ubar2[2]};
    }

    /** DIVIDE: normalising double precision divide (A / B), returns the quotient. */
    public static double divide(double num, double den) {
        return num / den;
    }

    /** RECT.1: pick up CSM state vector from the permanent store, set ZMEASURE. */
    public void rect1() {
        boolean moon = flag("CMOONFLG");
        boolean lunar = flag("LUNAFLAG");
        if (moon) {
            if (!lunar) upflag("ZMEASURE"); else downflag("ZMEASURE");
        } else {
            if (!lunar) downflag("ZMEASURE"); else upflag("ZMEASURE");
        }
        double sc = moon ? 4.0 : 1.0;                          // B27 vs B29 scaling
        rzc = vadd(vxsc(deltacsm, 1.0 / 128), rcvcsm);
        rzc = vxsc(rzc, sc);
        vzc = vxsc(vzcRel(), sc);
    }

    /** LDPLANET: allow planet vector to be stored in STARSAV2 if stored stars are not visible (V06N88). */
    public void ldplanet() {
        Response r = services.display("GOFLASH", VNPLAN23);
        if (r == Response.TERMINATE) {
            gotopooh();
            return;
        }
        if (r == Response.ENTER_RECYCLE) {
            ldplanet();
            return;
        }
        // STARSAV2 = UNIT(STARSAV3 * 1/SQR3)
        p23_31();
    }

    /** LOWMEMRY: retrieve star vector CATLOG,1 from low memory. */
    public void lowmemry() {
        // VLOAD* CATLOG,1 ; RVQ
    }

    /** GOTOV56: P20 terminates by GOTOV56 instead of GOTOPOOH: 2CADR TRACKTRM via SUPDXCHZ. */
    public void gotov56() {
        services.call("SUPDXCHZ:TRACKTRM");
    }

    // ================================================================ V67 (W matrix RMS)
    /** V67CALL: compute RMS position / velocity of the W matrix and display V06N99; R3 option stores new values. */
    public void v67call() {
        v67ww();
        while (true) {
            Response r = services.display("GOXDSPF", V06N99A);
            if (r == Response.TERMINATE) { endext(); return; }
            if (r == Response.PROCEED) break;
        }
        v06n9933();
    }

    private void v06n9933() {
        int wwopt = Agc.toSigned(g("WWOPT"));
        if (wwopt == 0) {
            endext();
            return;
        }
        // V6N99INP: store WWPOS / WWVEL in WORB, WMID or WREND depending on option (1,2,3)
        int pos = g("WWPOS"), vel = g("WWVEL");
        int diff = wwopt - (int) V67DEC2;
        if (diff == 0) {
            s("WORBPOS", pos);
            s("WORBVEL", vel);
        } else if (diff > 0) {
            s("WMIDPOS", pos);
            s("WMIDVEL", vel);
        } else {
            s("WRENDPOS", pos);
            s("WRENDVEL", vel);
        }
        downflag("ORBWFLAG");
        downflag("RENDWFLG");
        endext();
    }

    /** V67WW: WWPOS = RMS of position W elements, WWVEL = RMS of velocity W elements (limited to FT99999). */
    public void v67ww() {
        double pos = 0, vel = 0;
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 9; j++) {
                if (i < 3) pos += w[i][j] * w[i][j]; else vel += w[i][j] * w[i][j];
            }
        }
        double wwpos = Math.sqrt(pos), wwvel = Math.sqrt(vel);
        if (wwpos > FT99999) {
            wwpos = FT99999;
        }
        s("WWPOS", Agc.fromSigned((int) wwpos));
        s("WWVEL", Agc.fromSigned((int) wwvel));
        s("WWOPT", 0);
    }

    // ------------------------------------------------------------ shared small routines
    private double loadtime() {
        return (g("TIME2") * 16384.0) + g("TIME1"); // RTB LOADTIME: TIME2,1 into MPAC
    }

    private void gotopooh() {
        services.call("GOTOPOOH");
    }
}
