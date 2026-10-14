package apollo.luminary099;

import apollo.agc.Agc;

/**
 * P20-P25.agc (Luminary 1A / 099) - LM rendezvous navigation programs and radar routines:
 * <ul>
 * <li>P20 rendezvous navigation (PROG20, ORBCHG*, UPPSV state vector update)</li>
 * <li>P25 preferred tracking attitude (PROG25)</li>
 * <li>R22 rendezvous radar data read (R22LEM, LRS22.1 / LRS22.2 / LSR22.3 incorporation)</li>
 * <li>R21 RR designate, R23 manual acquisition, R24 search, R61/R65 preferred tracking attitude</li>
 * <li>Radar support: RRLIMCHK, REMODE, RRZERO, RRDESSM / RRDESNB / DODES, INITREAD / RADAREAD</li>
 * <li>R29 (LM rendezvous radar designation during servicer), V67 (W matrix RMS), radar lamps</li>
 * </ul>
 *
 * Interpretive code (vector arithmetic, scaling B-n) is modelled with doubles; original labels are kept
 * as method names (lowerCamelCase) and in Javadoc. Erasable words go through {@link Agc}; flags are
 * kept as agc symbols "FLAG.name". Calls into other AGC programs (BANKCALL, FINDVAC, NOVAC, WAITLIST,
 * INTEGRV, ...) go through the {@link Services} hook. Time quantities are centiseconds.
 */
public class P20P25 {
    private final Agc agc;

    /** Operator response after a flashing display: TC GOTOPOOH / TC +1 (proceed) / TC -5 (enter). */
    public enum Response { TERMINATE, PROCEED, ENTER }

    /** Outcome of RADSTALL: bad end (+0) or good end (+1). */
    public enum RadResult { BAD, GOOD }

    /** Calls into other AGC routines by name (BANKCALL, FINDVAC, NOVAC, WAITLIST, interpretive CALLs). */
    public interface Services {
        void call(String routine);
        Response display(String routine, int verbNoun);
        void fork(String kind, int priority, String routine);
        void waitlist(int centiseconds, String task);
        void phaschng(int... octal);
        RadResult radstall();
    }

    private Services services = new Services() {
        @Override public void call(String routine) { }
        @Override public Response display(String routine, int verbNoun) { return Response.PROCEED; }
        @Override public void fork(String kind, int priority, String routine) { }
        @Override public void waitlist(int centiseconds, String task) { }
        @Override public void phaschng(int... octal) { }
        @Override public RadResult radstall() { return RadResult.GOOD; }
    };

    public void setServices(Services s) { this.services = s; }

    public P20P25(Agc agc) {
        this.agc = agc;
    }

    // ---------------------------------------------------------------- constants
    public static final int ALRM526 = 0526, OCT201 = 0201, ALRM514 = 0514, MAXTRIES = 60, OCT00012 = 012, P22ONE = 1;
    public static final int ALRM525 = 0525, V06N05 = 605, V06N49NB = 649, DEC1500 = 1500, DEC250 = 250;
    public static final int ALRM503 = 0503, ALRM527 = 0527, ALRM530 = 0530, V06N72PV = 672, OCT501PV = 0501, OCT205 = 0205;
    public static final int V16N80 = 1680, V06N33 = 633, V06N99 = 699;
    public static final int BIT10_15 = 041000, BIT7_9PV = 0500, DEC600 = 600, B14_B2 = 020002;
    /** 2DEC 1.0 B-2. */
    public static final double ONEB_2 = 0.25;
    public static final double TENSEC = 1000, HALFSEC = 50, SEC3 = 300, SEC6 = 600, SEC1_5 = 150;
    public static final double COS15DEG = 0.96593, COS1_2DG = 0.999961923, MCTOMS = 100;
    /** Octal DEC constants of the limit tests (half-revolutions). */
    public static final double LIM_T_MODE1 = -.30555, LIM_S_MODE1 = -.35833, LIM_S_MODE2 = -.31667, LIM_T_MODE2 = -.69444;
    public static final double DEG5_5 = .03056, DEG82 = .45556, DEG20_5 = .11389, M50DEGSR = -.27778, M80DEGSR = -.44444;
    public static final double RRSPGAIN = .59062, RDESGAIN = .53624, RR29GAIN = -.53624;
    public static final int RRLIMIT = 384, HISCALIM = 460, DGBITS = 0230, BITS5_8 = 0220, BIT11_1 = 02001, WSIZE = 161;
    public static final double FHNM = 740800;          // 400 nautical miles in metres
    public static final double THREEDEG = .008333333, DEG30 = .083333333;
    public static final double TWO_PI_8 = 3.141592653 / 4;
    public static final double SIN60DEG = .86603, COS60DEG = .5, OFFSTFAC = 0.05678;
    public static final int ALL1S = 11111, FT99999 = 30479;
    public static final double IMUVAR = 1e-6;
    public static final int OCT13_7_2 = 010102;
    public static final double HALF_SEC_B17 = 50;

    // ------------------------------------------------------------ small helpers
    private int g(String s) { return agc.get(s); }
    private void s(String sym, int v) { agc.set(sym, v); }
    private boolean flag(String n) { return agc.get("FLAG." + n) != 0; }
    private void upflag(String n) { agc.set("FLAG." + n, 1); }
    private void downflag(String n) { agc.set("FLAG." + n, 0); }
    private void call(String r) { services.call(r); }
    private void phaschng(int... o) { services.phaschng(o); }
    private boolean bit(String word, int mask) { return (g(word) & mask) != 0; }
    private void setBits(String word, int mask) { s(word, g(word) | mask); }
    private void clrBits(String word, int mask) { s(word, g(word) & ~mask); }
    private int chan(int n) { return g("CHAN" + n); }
    private void wor(int n, int mask) { s("CHAN" + n, g("CHAN" + n) | mask); }
    private void wand(int n, int mask) { s("CHAN" + n, g("CHAN" + n) & mask); }

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
    public double tdec1, lnchtm, tet, transtm, mktime, deltatm, repostm, variance, deltaq, mlosv, r22disp, r22disp2;
    public double[] rsubl = new double[3], rsubc = new double[3], vsubc = new double[3], ucsm = new double[3];
    public double[] rvec = new double[3], vvec = new double[3], newvel = new double[3], newpos = new double[3];
    public double[] ncsmvel = new double[3], lmpos = new double[3], lmvel = new double[3], rrtarget = new double[3];
    public double[] losvel = new double[3], rrborsit = new double[3], bvector = new double[9];
    public double[] mx = new double[3], my = new double[3], mz = new double[3], ulc = new double[3];
    public double[] rlmsrch = new double[3], savlemv = new double[3], losdesrd = new double[3], vxrcm = new double[3];
    public double[] uxvect = new double[3], uyvect = new double[3];
    public double rlc, rm, rdotm, rrtrun, rrshaft, dsptem1, omegdisp, rxz, sinth, costh, sintheta;
    public double[] w = new double[162];
    public int scalshft;
    /** Radar sampling state (INITREAD / RADAREAD). */
    public int nsamp, samplim, timehold, oldatagd, dnindex;
    public long samplsum;

    // ================================================================ P20
    /** PROG20 (= PROG22): rendezvous navigation program. */
    public void prog20() {
        phaschng(4, 05022, 026000);          // 2PHSCHNG, priority 26
        if (!lunsfchk()) {                   // not on lunar surface
            downflag("VEHUPFLG");            // LM state vector to be updated
            prog20a();
            return;
        }
        orbchgo();
    }

    /** ORBCHGO: on the surface - ask CSM orbit-change option (R2) and launch time. */
    public void orbchgo() {
        upflag("VEHUPFLG");                  // CSM state vector to be updated
        s("OPTION2", 1);
        Response r;
        do {
            r = services.display("GOPERF4", OCT00012);
            if (r == Response.TERMINATE) { call("GOTOPOOH"); return; }
        } while (r == Response.ENTER);
        if (g("OPTION2") == P22ONE) {
            prog20a();
            return;
        }
        do {
            r = services.display("GOFLASH", V06N33);  // estimated time of launch
            if (r == Response.TERMINATE) { call("GOTOPOOH"); return; }
        } while (r == Response.ENTER);
        orbchg3();
    }

    /** ORBCHG3: compute CSM terminal state at launch for plane change; show CSM orbit change. */
    public void orbchg3() {
        call("INTSTALL");
        lnchtm = g("TIG");
        tdec1 = lnchtm;                      // estimated launch time
        downflag("VINTFLAG");                // LM integration
        downflag("INTYPFLG");                // precision - Encke
        downflag("DIM0FLAG");                // no W matrix
        downflag("D6OR9FLG");
        call("INTEGRV");                     // planetary inertial orientation
        call("GRP2PC");
        rsubl = vec("RATT1");                // save LM position
        tdec1 = g("TAT");
        call("INTSTALL");
        upflag("VINTFLAG");                  // CSM integration
        downflag("INTYPFLG");
        downflag("DIM0FLAG");
        if (flag("RENDWFLG")) {              // W matrix valid
            upflag("DIM0FLAG");
            upflag("D6OR9FLG");
        }
        call("INTEGRV");                     // CSM integration
        call("GRP2PC");
        vsubc = vec("VATT1");                // save CSM velocity
        rsubc = vec("RATT1");                // save CSM position
        double[] nsub1 = unit(cross(rsubc, vsubc));    // normal to CSM orbital plane
        ucsm = unit(cross(rsubl, nsub1));              // UCSM = UNIT(R(LM) x NSUB1)
        double csth = 2 * dot(ucsm, unit(rsubc));      // cos A (SL1)
        double snth = Math.sqrt(Math.max(0, ONEB_2 * 4 - csth * csth)); // sine A for time-theta
        rvec = rsubc;                        // CSM position at est. launch time (time-theta)
        vvec = vxsc(vsubc, -1);
        call("TIMETHET");
        newvel = vxsc(vvec, -1);             // terminal velocity of CSM
        transtm = g("T");                    // transfer time
        newpos = vec("RVEC");                // terminal position of CSM
        double[] nsub2 = unit(cross(newpos, rsubl));
        ncsmvel = vxsc(unit(cross(nsub2, newpos)), abval(newvel) * 2); // rotate terminal vel into desired plane
        call("GRP2PC");
        call("INTSTALL");
        tet = lnchtm - transtm;              // launch time minus transfer time
        s("RCV", 0);
        call("MINIRECT");                    // RRECT = newpos; VRECT = ncsmvel
        call("ATOPCSM");                     // AXT,2 2
        call("INTWAKE0");
        prog20a();
    }

    /** Vector fetch placeholder: pulls a 3-vector from erasable storage by symbol. */
    private double[] vec(String sym) {
        return new double[] {g(sym), g(sym + "+2"), g(sym + "+4")};
    }

    /** (on the lunar surface) DOWNFLAG VEHUPFLG - LM state vector to be updated; falls into PROG20A. */
    public void prog20a() {
        if (!flag("VEHUPFLG") && false) { downflag("VEHUPFLG"); }
        call("R02BOTH");
        upflag("UPDATFLG");                  // set update flag
        upflag("TRACKFLG");                  // set track flag
        upflag("RNDVZFLG");                  // set rendezvous flag
        downflag("SRCHOPTN");                // insure search option off
        downflag("ACMODFLG");                // manual acquisition flag reset
        downflag("R04FLAG");                 // ensure alarm 521 if can't read radar
        downflag("NORRMON");                 // enable R25 gimbal monitor
        downflag("LOSCMFLG");                // LOS being computed flag reset
        clradmod();
        p20lem1();
    }

    /** P20LEM1: LOS determination and 400 NM range test. */
    public void p20lem1() {
        phaschng(04022);
        s("MARKCTR", 0);
        tdec1 = loadtime();
        lps20_1();
        if (lps20_2() == 0) {
            p20lema();
        } else {
            alarm526();
        }
    }

    /** 526ALARM: range greater than 400 NM. */
    public void alarm526() {
        Response r;
        do {
            r = services.display("PRIOLARM", ALRM526);
            if (r == Response.TERMINATE) { gotov56(); return; }
        } while (r == Response.PROCEED);     // PROC illegal
        p20lem1();                           // ENTER recycle
    }

    /** P20LEMA: check lunar surface, call R61LEM. */
    public void p20lema() {
        phaschng(04022);
        if (!lunsfchk()) {
            r61lem();                        // preferred tracking attitude
        }
        p20lemb();
    }

    /** P20LEMB: restart at priority 10, restore priority 26, check track flag. */
    public void p20lemb() {
        phaschng(05022, 010000);
        call("PRIOCHNG:PRIO26");
        if (!flag("TRACKFLG")) {
            p20lemwt();
            return;
        }
        p20lemb7();
    }

    /** P20LEMB7: is RR auto mode discrete present? */
    public void p20lemb7() {
        if ((chan(33) & 02) == 0) {          // bit 2 zero = auto mode
            p20lemb3();
        } else {
            p20lemb5();
        }
    }

    /** P20LEMB5: RR not in auto - check major mode 20 / 22 and request mode selection. */
    public void p20lemb5() {
        int mm = g("MODREG");
        if (mm == 020 || mm == 022) {
            p20lemb6();
            return;
        }
        Response r = services.display("PRIOLARM", ALRM514);   // RR goes out of auto mode while in use
        if (r == Response.TERMINATE) { gotov56(); return; }
        p20lemb();                           // proceed and enter both go back
    }

    /** P20LEMB6: request RR auto mode selection (V50 N72 style). */
    public void p20lemb6() {
        Response r = services.display("GOPERF1", OCT201);
        if (r == Response.TERMINATE) { gotov56(); return; }
        if (r == Response.PROCEED) { p20lemb(); return; }
        if (lunsfchk()) {                    // R23 not allowed on the surface
            p20lemb2();
        } else {
            r23lem();
        }
    }

    /** P20LEMB1: return from R23 - lockon achieved. */
    public void p20lemb1() {
        upflag("ACMODFLG");
        p20lemb();
    }

    /** P20LEMB2: operator error light, go back. */
    public void p20lemb2() {
        call("FALTON");
        p20lemb();
    }

    /** P20LEMB3: wait while RR CDUs are zeroed, otherwise check search/manual acquisition flags. */
    public void p20lemb3() {
        if (bit("RADMODES", RCDU0BIT)) {     // CDUs being zeroed - wait 2.5 s
            services.waitlist(DEC250, "P20LEMB3");
            return;
        }
        if (!flag("SRCHOPTN") && !flag("ACMODFLG")) {   // zero means automatic RR acquisition
            p20lemc3();
            return;
        }
        downflag("SRCHOPTN");
        downflag("ACMODFLG");
        p20lemwt();
    }

    /** P20LEMC3: update permanent state vectors to present time, then fall into P20LEMC. */
    public void p20lemc3() {
        tdec1 = loadtime();
        uppsv();
        p20lemc4();
    }

    /** P20LEMC4/P20LEMC: rendezvous flag still set? track flag set -> R21LEM else wait 15 s. */
    public void p20lemc4() {
        phaschng(04022);
        if (!flag("RNDVZFLG")) { return; }   // ENDOFJOB - exit P20
        if (flag("TRACKFLG")) {
            r21lem();
        } else {
            p20lemd();
        }
    }

    /** P20LEMWT: wait 2.5 seconds then go to RR data read (task P20LEMC1). */
    public void p20lemwt() {
        services.waitlist(DEC250, "P20LEMC1");
        if (!flag("TRACKFLG")) { return; }
        phaschng(040072);
    }

    /** P20LEMC1: waitlist task - schedule R22 job if track flag set. */
    public void p20lemc1() {
        if (!flag("RNDVZFLG")) { return; }   // TASKOVER
        if (flag("TRACKFLG")) {
            services.fork("FINDVAC", 026, "R22LEM42");
        } else {
            services.waitlist(1500, "P20LEMC1");  // track flag not set, wait 15 s
        }
    }

    /** P20LEMD: waitlist 15 s for designate job. */
    public void p20lemd() {
        services.waitlist(DEC1500, "P20LEMD1");
    }

    /** P20LEMD1: task - if track flag set schedule designate job. */
    public void p20lemd1() {
        if (flag("TRACKFLG")) {
            p20lemd2();
        } else {
            services.waitlist(1500, "P20LEMD1");
        }
    }

    /** P20LEMD2: schedule job to do R21 (start at P20LEMC3). */
    public void p20lemd2() {
        services.fork("FINDVAC", 026, "P20LEMC3");
    }

    /** UPPSV: update permanent state vectors to present time. */
    public void uppsv() {
        call("INTSTALL");
        setiflgs();
        if (flag("RENDWFLG")) {              // W matrix valid: integrate it
            upflag("DIM0FLAG");
            if (!flag("SURFFLAG")) {
                upflag("D6OR9FLG");          // 9x9 unless on the lunar surface
            }
        }
        if (flag("VEHUPFLG")) {              // CSM being updated
            uppsv3();
            return;
        }
        upflag("VINTFLAG");
        call("INTEGRV");
        call("GRP2PC");
        call("INTSTALL");
        tdec1 = g("TETCSM");
        downflag("VINTFLAG");
        uppsv4();
    }

    private void uppsv3() {
        downflag("VINTFLAG");
        call("INTEGRV");
        call("GRP2PC");
        call("INTSTALL");
        upflag("VINTFLAG");
        tdec1 = g("TETLEM");
        uppsv4();
    }

    /** UPPSV4: integrate other vehicle without W matrix. */
    private void uppsv4() {
        setiflgs();
        call("INTEGRV");
        if (!flag("SURFFLAG")) {
            return;                          // P20LEMC4
        }
        lmpos = vxsc(vec("RCVLEM"), 0.25);   // VSR2
        lmvel = vxsc(vec("VCVLEM"), 0.25);
        ls21x();
    }

    private void setiflgs() { call("SETIFLGS"); }
    private void ls21x() { /* return point of LPS20.1 / UPPSV (STQ LS21X) */ }

    // ================================================================ P25
    /** PROG25: preferred tracking attitude program. */
    public void prog25() {
        phaschng(4, 05022, 026000);          // group 4 inactive, priority 26
        call("R02BOTH");
        upflag("TRACKFLG");
        upflag("P25FLAG");
        p25lem1();
    }

    /** P25LEM1: loop: if P25FLAG clear exit; if tracking call R65 else wait one minute. */
    public void p25lem1() {
        phaschng(04022);
        if (!flag("P25FLAG")) { return; }
        if (flag("TRACKFLG")) {
            s("R65CNTR", 7);
            r65lem();
            p25lem1();
            return;
        }
        p25lmwt1();
    }

    public void p25lemwt() { phaschng(0112); p25lmwt1(); }

    /** P25LMWT1: wait one minute (6000 cs) then check again. */
    public void p25lmwt1() {
        services.waitlist(DEC600 * 10, "P25LEM2");
    }

    /** P25LEM2: waitlist task - schedule P25LEM1 at priority 14. */
    public void p25lem2() {
        services.fork("FINDVAC", 014, "P25LEM1");
    }

    // ================================================================ R22
    /** R22LEM: rendezvous radar data read; processes marks to update the state vector. */
    public void r22lem() {
        phaschng(04022);
        if (!flag("RNDVZFLG")) { return; }   // exit R22 and P20
        if (!flag("TRACKFLG")) { r22wait(); return; }
        r22lem12();
    }

    /** R22LEM12: RR auto-track enable and auto mode discrete checks. */
    public void r22lem12() {
        if ((chan(12) & BIT14) == 0) { p20lema(); return; }   // monitor reposition cleared it
        if ((chan(33) & 02) != 0) { p20lemb5(); return; }     // not in auto - set it
        if (!bit("RADMODES", RCDU0BIT)) {                     // CDUs zeroing in process
            r22lem42();
            return;
        }
        phaschng(0152);
        int rc = lrs22_1();                  // data read subroutine, MPAC code
        if (rc == 0) { r22lem2(); return; }
        if (rc == 1) { p20lemc4(); return; } // could not read radar - redesignate
        Response r = services.display("PRIOLARM", ALRM525);   // RR LOS not within 3 degrees
        if (r == Response.TERMINATE) { gotov56(); return; }
        if (r == Response.PROCEED) { r22lem1(); }
    }

    /** R22LEM1: display delta theta (V06N05). */
    public void r22lem1() {
        phaschng(04022);
        Response r = services.display("PRIODSP", V06N05);
        if (r == Response.TERMINATE) { gotov56(); return; }
        if (r == Response.PROCEED) { r22lem2(); return; }
        p20lemc4();                          // ENTER recycle (P20LEMC)
    }

    /** R22LEM2: check lunar surface / track flag / 30 degree boresight. */
    public void r22lem2() {
        phaschng(04022);
        if (!lunsfchk()) {                   // surface: bypass LRS22.2
            r22lem3();
            return;
        }
        if (!flag("TRACKFLG")) { r22wait(); return; }
        if (lrs22_2() != 0) {                // not within 30 deg of +Z axis
            r61lem();
            r22wait();
            return;
        }
        r22lem3();
    }

    /** R22LEM3: bypass state vector update if NOUPFLG set or UPDATFLG clear. */
    public void r22lem3() {
        if (flag("NOUPFLG") || !flag("UPDATFLG")) { r22lem42(); return; }
        s("PHSPRDT2", 026);                  // insure high priority in restart
        lsr22_3();
    }

    /** R22LEM93: normal exit from LSR22.3. */
    public void r22lem93() {
        phaschng(04022);
        r22lem44();
    }

    /** R22LEM96: excessive state vector update - display N49 and wait for answer. */
    public void r22lem96() {
        s("N49FLAG", 0);
        phaschng(04022);
        services.fork("NOVAC", 027, "N49DSP");
        while (g("N49FLAG") == 0) { /* loop until display answered */ break; }
        if (Agc.toSigned(g("N49FLAG")) < 0) {   // proceed
            r22lem7();
        } else {                                // recycle
            if (!lunsfchk()) { r22wait(); return; }
            r22lem45(0);
        }
    }

    /** R22LEM7: proceed - phase change and go incorporate the data (ASTOK). */
    public void r22lem7() {
        call("GRP2PC");
        astok();
    }

    /** R22LEM44: increment mark counter, schedule next fine tracking attitude. */
    public void r22lem44() {
        s("MARKCTR", g("MARKCTR") + 1);
        if (!lunsfchk()) { r22lem46(); return; }
        r22lem45(5);
    }

    public void r22lem42() {
        if (!lunsfchk()) { r22lem46(); return; }
        r22lem45(2);
    }

    /** R22LEM45: set R65CNTR and call R65LEM, then recycle R22. */
    public void r22lem45(int count) {
        s("R65CNTR", count);
        r65lem();
        r22lem();
    }

    /** R22WAIT: wait 15 seconds. */
    public void r22wait() { services.waitlist(DEC1500, "P20LEMC1"); }

    /** R22LEM46: wait 2 seconds and take another mark. */
    public void r22lem46() {
        call("DELAYJOB:2SECS");
        r22lem();
    }

    /** N49DSP: V06 N49 display (excess state vector update). Proceed sets N49FLAG = -1. */
    public void n49dsp() {
        Response r = services.display("PRIODSP", V06N49NB);
        if (r == Response.TERMINATE) { gotov56(); return; }
        s("N49FLAG", r == Response.PROCEED ? Agc.fromSigned(-1) : 1);
    }

    /** R22RSTRT: restart protection - take a range-rate reading that is not used. */
    public void r22rstrt() {
        phaschng(0152);
        rrrdot();
        if (services.radstall() == RadResult.BAD) { p20lemc4(); return; }
        r22lem();
    }

    /**
     * LUNSFCHK: closed subroutine - returns true (TC Q, "+1") if SURFFLAG (P22FLAG) is set,
     * false (return to caller +2) if not set.
     */
    public boolean lunsfchk() { return flag("SURFFLAG"); }

    // ================================================================ R21
    /** R21LEM: RR designate routine - point the radar at the CSM until acquisition. */
    public void r21lem() {
        wand(12, ~BIT14);                    // remove RR self-track enable
        if (lunsfchk()) {                    // on lunar surface -> R21LEM5
            if (bit("RADMODES", BIT12)) {    // already in mode 2
                r21lem10();
                return;
            }
            s("TANG", 040000);               // BIT15
            s("TANG+1", Agc.fromSigned(-0x2000)); // -HALF
        } else {                             // command antenna to mode centre (mode 1: T=0, S=0)
            s("TANG", 0);
            s("TANG+1", 0);
        }
        r21lem6();
    }

    /** R21LEM6: RRDESNB then wait for designate. */
    public void r21lem6() {
        downflag("LOKONSW");
        rrdesnb();
        if (services.radstall() == RadResult.BAD) { r21_503(); return; }
        r21lem10();
    }

    /** R21LEM10: allow 60 passes (about 45 s) to designate and lock on. */
    public void r21lem10() {
        upflag("LOSCMFLG");
        s("DESCOUNT", MAXTRIES);
        r21lem2();
    }

    public void r21lem2() {
        s("LOSCOUNT", 3);
        r21lem1();
    }

    /** R21LEM1: LOS at present time + .5 s via LPS20.1. */
    public void r21lem1() {
        tdec1 = loadtime() + HALFSEC;
        lps20_1();
        r21lem3();
    }

    /** R21LEM3: designate; exits for mode-2 coverage, vehicle maneuver or lockon. */
    public void r21lem3() {
        upflag("LOKONSW");
        downflag("NORRMON");
        int rc = rrdessm();                  // 0: not in coverage, 1: vehicle maneuver, 2: designating
        if (rc == 0) { r21lem4(); return; }
        if (rc == 1) { p20lema(); return; }
        if (services.radstall() == RadResult.BAD) { r21_503(); return; }
        r21end();
    }

    /** R21-503: alarm 503, lock-on not achieved in 60 tries. */
    public void r21_503() {
        Response r = services.display("PRIOLARM", ALRM503);
        if (r == Response.TERMINATE) { gotov56(); return; }
        if (r == Response.PROCEED) { r21srch(); return; }
        p20lemc3();
    }

    /** R21END: lockon achieved - reset LOSCMFLG, put up verify display. */
    public void r21end() {
        downflag("LOSCMFLG");
        r21disp();
    }

    /** R21SRCH: phase change, search routine R24. */
    public void r21srch() {
        phaschng(04022);
        r24lem();
    }

    /** R21LEM4: LOS not in mode 2 coverage on lunar surface - reposition every 10 s for up to 600 s. */
    public void r21lem4() {
        s("REPOSCNT", MAXTRIES);
        upflag("FSPASFLG");
        downflag("LOSCMFLG");
        r21lem12();
    }

    /** R21LEM12: LOS at T + 10 s. */
    public void r21lem12() {
        r21lem12Body(loadtime() + TENSEC);
    }

    /** Body of R21LEM12 after the DAD TENSEC (also entered at "R21LEM12 +2" with REPOSTM loaded). */
    private void r21lem12Body(double t) {
        repostm = t;
        tdec1 = repostm;
        lps20_1();
        int rc = rrdessm();
        if (rc == 0) { r21lem13(); return; }
        if (rc == 1) { return; }             // ENDOFJOB - vehicle maneuver required
        call("KILLTASK:BEGDES");
        if (!flag("FSPASFLG")) { r21lemb(); return; }   // continuous designate
        downflag("FSPASFLG");
        r21lem7_1();
    }

    /** R21LEM13: have we tried 60 times? */
    public void r21lem13() {
        int c = g("REPOSCNT");
        if (c == 0) { r21lem11(); return; }
        r21lem7(c - 1);
    }

    /** R21LEM7: store counter, add 10 s to REPOSTM, recompute LOS. */
    private void r21lem7(int count) {
        s("REPOSCNT", count);
        r21lem12Body(repostm + TENSEC);
    }

    private void r21lem7_1() { r21lem7(g("REPOSCNT")); }

    /** R21LEMB: continuous designate: update state vectors at REPOSTM, schedule R21LEM9 at that time. */
    public void r21lemb() {
        tdec1 = repostm;
        uppsv();
        upflag("CDESFLAG");                  // radmodes bit 15, continuous designation
        downflag("LOKONSW");
        upflag("NORRMON");
        rrdesnb();
        deltatm = repostm - loadtime();
        services.waitlist((int) deltatm, "R21LEM9");   // LONGCALL
    }

    /** R21LEM9: long-call task - stop designate, set up R21LEM10. */
    public void r21lem9() {
        call("KILLTASK:STDESIG");
        clradmod();
        services.fork("FINDVAC", 026, "R21LEM10");
    }

    /** R21LEM11: alarm 530 - LOS not in coverage after trying to designate for 600 s. */
    public void r21lem11() {
        services.display("PRIOLARM", ALRM530);
        gotov56();
    }

    /** R21DISP: flash V50 N72 - main lobe lockon verification. */
    public void r21disp() {
        phaschng(04022);
        Response r = services.display("GOPERF2R", V06N72PV);
        if (r == Response.TERMINATE) { gotov56(); return; }
        if (r == Response.PROCEED) { p20lemwt(); return; }
        call("LINUS:BIT7");                  // priority display
    }

    // ================================================================ R23
    /** R23LEM: manual acquisition of the CSM by the rendezvous radar. */
    public void r23lem() {
        upflag("NORRMON");
        call("IBNKCALL:SETMINDB");           // minimum deadband
        r23lem1();
    }

    public void r23lem1() {
        wor(12, BIT14);                      // enable tracker
        Response r = services.display("GOPERF1", OCT205);
        if (r == Response.TERMINATE) { r23lem2(); return; }
        if (r == Response.ENTER) { r23lem3(); return; }
        r23lem11();
    }

    /** R23LEM11: proceed - check antenna within limits. */
    public void r23lem11() {
        if (!rrlimchk("CDUT")) { outoflim(); return; }
        call("IBNKCALL:RESTORDB");
        downflag("NORRMON");
        p20lemb1();                          // radar locked on, continue in P20
    }

    /** OUTOFLIM: alarm 501 - RR antenna not within limits. */
    public void outoflim() {
        Response r = services.display("PRIOLARM", OCT501PV);
        if (r == Response.TERMINATE) { r23lem2(); return; }
        if (r == Response.PROCEED) { outoflim(); return; }   // illegal
        r23lem3();
    }

    public void r23lem2() {
        downflag("NORRMON");
        gotov56();
    }

    public void r23lem3() {
        r61lem();
        r23lem1();
    }

    // ================================================================ R24
    /** R24LEM: rendezvous radar search routine (hexagonal pattern about the LOS). */
    public void r24lem() {
        upflag("SRCHOPTN");
        downflag("LOSCMFLG");
        r24lem1();
    }

    public void r24lem1() {
        s("DATAGOOD", 0);
        s("OMEGAD", 0);
        s("OMEGAD+1", 0);
        r24lem2();
    }

    /** R24LEM2: V16 N80 display. */
    public void r24lem2() {
        phaschng(04022);
        Response r = services.display("PRIODSPR", V16N80);
        if (r == Response.TERMINATE) { gotov56(); return; }
        if (r == Response.PROCEED) { r24end(); return; }
        r24lem3();
    }

    /** R24END: kill designate loop, clear radar modes, go to range check. */
    public void r24end() {
        call("KILLTASK:CALLDGCH");
        clradmod();
        p20lem1();
    }

    /**
     * CLRADMOD: clear bits 10 and 15 of RADMODES, disable RR error counters (user will RELINT).
     */
    public void clradmod() {
        clrBits("RADMODES", BIT10_15);
        wand(12, ~02);
    }

    /** R24LEM3: recycle - kill next-point task, call R61 for maneuver. */
    public void r24lem3() {
        phaschng(04022);
        call("KILLTASK:CALLDGCH");
        clradmod();
        call("DELAYJOB:.5SEC");
        if (!lunsfchk()) {
            r61lem();
        }
        s("RADCADR", 0);                     // so that RRDESSM returns to caller
        r24lem2();
    }

    // ================================================================ R61 / R65
    /** R61LEM: preferred tracking attitude routine (sets R61FLAG). Entry saved in GENRET. */
    public void r61lem() {
        upflag("R61FLAG");
        r61cL01();
    }

    /** R65LEM: fine preferred tracking attitude (clears R61FLAG). */
    public void r65lem() {
        downflag("R61FLAG");
        r61cL01();
    }

    /** R61C+L01: track flag test, track axis = HIUNITZ. */
    public void r61cL01() {
        if (!flag("TRACKFLG")) { r65wait(); return; }
        // SCAXIS = HIUNITZ (track axis unit vector)
        r61lem1();
    }

    /** R61LEM1: LOS at center of the six second period; compute final CDU angles. */
    public void r61lem1() {
        tdec1 = loadtime() + SEC3;           // extrapolate to centre of six second period
        lps20_1();                           // LOS determination and vehicle attitude
        call("VECPNT1");                     // POINTVSM = RRTARGET; final angles CPHI/CTHETA/CPSI from READCDUD
        readcdud();
        phaschng(04022);
        if (!flag("TRACKFLG")) { r65wait(); return; }
        call("G+N,AUTO");                    // check for auto mode
        if (g("A") != 0) {                   // not in auto
            call("BALLANGS");                // convert angles to FDAI
            r61cL06();
            return;
        }
        double z = g("MPAC+5") / 16384.0;    // Z component of RRTARGET in NB axes (CDU*SMNB): cos PHI
        if (z - COS15DEG >= 0) {             // PHI not greater than 15 degrees: command CDU desired angles directly
            s("CDUXD", g("CPHI"));
            s("CDUXD+1", g("CPHI+1"));
            s("CDUZD", g("CPSI"));
            r61cL06();
            return;
        }
        r61cL05();
    }

    /** R61C+L05: PHI greater than 15 degrees - maneuver via R60LEM with minimum deadband. */
    public void r61cL05() {
        call("IBNKCALL:ZATTEROR");
        call("IBNKCALL:SETMINDB");           // reduce attitude error
        downflag("3AXISFLG");
        upflag("PDSPFLAG");                  // priority display flag
        call("R60LEM");
        call("IBNKCALL:RESTORDB");
        phaschng(04022);
        downflag("PDSPFLAG");
        r61cL06();
    }

    /** R61C+L06: if R61 exit, else count down R65CNTR and reschedule in 6 s. */
    public void r61cL06() {
        if (flag("R61FLAG")) { r61cL4(); return; }
        int c = g("R65CNTR");
        if (c == 0) { r61cL4(); return; }    // R65CNTR = 0 - exit routine
        s("R65CNTR", c - 1);
        services.waitlist(DEC600, "R61C+L2");   // 06SEC
    }

    /** R61C+L2: task - FINDVAC at priority 26 for R61C+L01. */
    public void r61cL2() {
        services.fork("FINDVAC", 026, "R61C+L01");
    }

    /** R61C+L4: exit R61 (BANKJUMP to caller in GENRET). */
    public void r61cL4() { call("BANKJUMP:GENRET"); }

    /** R61C+L1: rendezvous or P25 flag set? else end job. */
    public void r61cL1() {
        if (!flag("RNDVZFLG") && !flag("P25FLAG")) { return; }
        r61cL06();
    }

    /** R65WAIT: POSTJUMP to P20LEMWT. */
    public void r65wait() { p20lemwt(); }

    /** READCDUD: read desired CDU's (CDUXD, CDUYD) into MPAC as TP; EBANK 6 switch. */
    public void readcdud() {
        s("MPAC", g("CDUXD"));
        s("MPAC+1", g("CDUYD"));
        s("MPAC+2", g("CDUYD+1"));
        s("MODE", 1);                        // TMODE
    }

    // ================================================================ RRSUB
    /** MAGSUB: true if |a| <= |lim| (return to L+3), false if greater (L+2); lim is the negated constant. */
    private boolean magsub(int word, double lim) {
        double a = Math.abs(Agc.toSigned(word) / 16384.0);
        return a + lim <= 0 || a <= -lim;
    }

    /** Angle word (half-revs scaled to 1.0) helper. */
    private double frac(int word) { return Agc.toSigned(word) / 16384.0; }

    /**
     * RRLIMCHK: check RR gimbal angles (trunnion T at sym, shaft S at sym+1) against the limits of the
     * current antenna mode. Mode 1: |T| &lt;= 55 deg and |S + 5.5| &lt;= 64.5 deg (shaft limits 59, -70).
     * Mode 2: |T| &gt; 125 deg and |S - 82| &lt;= 57 deg. Returns true when in limits (TC RRLIMOK).
     */
    public boolean rrlimchk(String sym) {
        int t = g(sym), sh = g(sym + "+1");
        s("ITEMP1", t);
        s("ITEMP2", sh);
        if (bit("RADMODES", ANTENBIT)) { return mode2chk(); }
        if (!magsub(t, LIM_T_MODE1)) { return false; }
        double s5 = Agc.toSigned(sh) / 16384.0 + DEG5_5;
        return Math.abs(s5) <= -LIM_S_MODE1;
    }

    /** MODE2CHK: mode 2 test. */
    public boolean mode2chk() {
        double sh = Agc.toSigned(g("ITEMP2")) / 16384.0 + DEG82;
        if (Math.abs(sh) > -LIM_S_MODE2) { return false; }
        return !magsub(g("ITEMP1"), LIM_T_MODE2) ? true : false;
    }

    /**
     * RRLIMNB: as RRLIMCHK except the mode 1 shaft lower limit is -85 degrees instead of -70 (V41 check).
     */
    public boolean rrlimnb(String sym) {
        int t = g(sym), sh = g(sym + "+1");
        s("ITEMP1", t);
        s("ITEMP2", sh);
        if (bit("RADMODES", ANTENBIT)) { return mode2chk(); }
        if (!magsub(t, LIM_T_MODE1)) { return false; }
        double shaft = Agc.toSigned(sh) / 16384.0;
        shaft += shaft < 0 ? DEG20_5 : DEG5_5;     // NEGSHAFT adds 20.5 deg
        return Math.abs(shaft) <= -LIM_S_MODE1;
    }

    /** SETTRKF: set tracker-fail lamp state unless lamp test is in progress (IMODES33 bit 1). */
    public void settrkf() {
        if (bit("IMODES33", BIT1)) { return; }
        rrtrkf();
    }

    /** RRTRKF / RRCHECK / TRKFLON / FLIP: flip the tracker-fail bit (bit 8 of DSPTAB+11D) if the desired state differs. */
    public void rrtrkf() {
        int desired = 0;
        if ((g("RADMODES") & OCT13_7_2) != 0 || (g("RADMODES") & RRDATABT) != 0) {
            desired = BIT8;                  // CDU failed or RR data failed
        }
        trkflon(desired);
    }

    /** TRKFLON: compare with present DSPTAB+11D and flip (XOR) if changed. */
    public void trkflon(int desired) {
        int cur = g("DSPTAB+11");
        if (((cur ^ desired) & BIT8) == 0) { return; }   // no change
        s("DSPTAB+11", ((cur ^ BIT8) & Agc.WORD_MASK & ~040000) | 040000);   // FLIP; keep bit 15 set
    }

    /** RRTURNON: zero RR CDUs; wait 1 s before removing the turn-on flag. */
    public void rrturnon() {
        rrzerosb();
        services.waitlist(100, "RRTURNON2");
    }

    public void rrturnon2() {
        clrBits("RADMODES", TURNONBT);
    }

    /**
     * RRZEROSB: zero RR CDUs (turn on ZERO RR CDU, wait 2 cs, zero CDUT/CDUS, wait 10 s, reset
     * zeroing-in-process, set antenna mode bit from CDUT).
     */
    public void rrzerosb() {
        wor(12, BIT1);                       // turn on ZERO RR CDU (bit 13 of RADMODES must be set first)
        services.waitlist(2, "RRZEROSB2");
        s("CDUT", 0);
        s("CDUS", 0);
        wand(12, ~BIT1);                     // remove zeroing bit
        services.waitlist(1000, "RRZEROSB3");     // reset fail inhibit in 10 s
        clrBits("RADMODES", RCDU0BIT);
        int mode = magsub(g("CDUT"), -.5) ? 0 : ANTENBIT;    // mode 2 if |T| > 90 deg
        int rm = g("RADMODES");
        s("RADMODES", (rm & ~ANTENBIT) | mode);
        settrkf();
    }

    /** DORREPOS: reposition RR to the nominal position after a monitor-detected out of limits. */
    public void dorrepos() {
        setrrecr();
        services.waitlist(2, "DORREPOS2");
        rrtonly(bit("RADMODES", ANTENBIT) ? 040000 : 0);        // 0 for mode 1, 180 deg for mode 2
        rrsonly(bit("RADMODES", ANTENBIT) ? Agc.fromSigned(-0x2000) : 0);   // -90 for mode 2
        reposrpt();
    }

    /** REPOSRPT: remove reposition bit; begin designate if waiting, else disable error counters. */
    public void reposrpt() {
        clrBits("RADMODES", REPOSBIT);
        if (bit("RADMODES", DESIGBIT)) { begdes(); return; }
        wand(12, ~BIT2);
    }

    /** SETRRECR: set up and enable RR CDU error counters (does not clear last command if already enabled). */
    public void setrrecr() {
        if ((chan(12) & BIT2) != 0) { return; }
        s("LASTYCMD", 0);
        s("LASTXCMD", 0);
        wor(12, BIT2);
    }

    /** REMODE: change RR antenna mode - drive trunnion to 0 (180), shaft to -45 / -130 / -180, invert mode. */
    public void remode() {
        boolean m2 = bit("RADMODES", ANTENBIT);
        rrtonly(m2 ? 040000 : 0);
        rrsonly(Agc.fromSigned((int) (-0.25 * 16384)));          // -45DEGSR
        int d = !m2 ? Agc.fromSigned((int) (M80DEGSR * 16384)) : 0;
        rrtonly(Agc.fromSigned((int) (M50DEGSR * 16384) + Agc.toSigned(d)));  // T = -130 (-50)
        rrtonly(!m2 ? 040000 : 0);           // T = -180 (+0)
        rrsonly(!m2 ? Agc.fromSigned(-0x2000) : 0);              // S = -90 (+0)
        rmodinv();
        clrBits("RADMODES", REMODBIT);
        if (!bit("RADMODES", DESIGBIT)) { rgoodend(); return; }   // no - return to caller in RADSTALL
        stdesig();
    }

    /** RMODINV: invert the RR mode bit in RADMODES. */
    public void rmodinv() { s("RADMODES", g("RADMODES") ^ ANTENBIT); }

    /** RRTONLY: drive trunnion only to the desired angle (RDES). */
    public void rrtonly(int desired) {
        s("RDES", desired);
        rr1axis(0);
    }

    /** RRSONLY: drive shaft only (commands unresolved since T = 0 or 180). */
    public void rrsonly(int desired) {
        s("RDES", desired);
        rr1axis(1);
    }

    /**
     * RR1AXIS / RR1AX2 / NXTRR1AX: drive one RR axis (index 0 = T, 1 = S) at 2 samples per second until
     * within one degree, nulling .7 of the error each half second via RROUT; aborts to REPOSRPT if a
     * designate requires a remode, returns early when RR leaves auto mode.
     */
    public void rr1axis(int index) {
        s("RRINDEX", index);
        for (;;) {
            if ((~g("RADMODES") & PRIO22) == 0) { reposrpt(); return; }   // remode requested during reposition
            int cdu = g(index == 0 ? "CDUT" : "CDUS");
            double err = Agc.toSigned(g("RDES")) - Agc.toSigned(cdu);
            int cmd = (int) (err * RRSPGAIN);
            boolean outOfAuto = bit("RADMODES", AUTOMBIT);
            if (Math.abs(err / 16384.0) <= -(-.00555)) {
                if (outOfAuto) { return; }
                return;                      // within one degree: done
            }
            if (outOfAuto) { return; }       // RR out of auto mode: exit
            s("TANG", index == 0 ? cmd : 0);
            s("TANG+1", index == 1 ? cmd : 0);
            rrout();
            services.waitlist(50, "NXTRR1AX");   // two samples per second
            if (index >= 0) { return; }      // waitlist task re-enters at NXTRR1AX
        }
    }

    /** RROUT: output both TANG commands (limited to +/-384) to the RR CDU counters and enable drives. */
    public void rrout() {
        for (int i = 1; i >= 0; i--) {
            int cmd = Agc.toSigned(g(i == 0 ? "TANG" : "TANG+1"));
            if (Math.abs(cmd) > RRLIMIT) { cmd = cmd < 0 ? -RRLIMIT : RRLIMIT; }   // RROUTLIM
            String last = i == 0 ? "LASTXCMD" : "LASTYCMD";
            int diff = cmd - Agc.toSigned(g(last));          // count out difference from last command
            s(last, Agc.fromSigned(cmd));
            s(i == 0 ? "CDUTCMD" : "CDUSCMD", Agc.fromSigned(diff));
        }
        wor(14, PRIO6);                      // put on CDU drives S and T
    }

    /** RRZERO: re-zero RR CDUs unless monitor reposition or not in auto. */
    public void rrzero() {
        if ((g("RADMODES") & BIT11_1) != 0) { radnoop(); return; }
        setBits("RADMODES", RCDU0BIT);       // zeroing in progress
        services.waitlist(1, "RRZ2");
        if (bit("RADMODES", AUTOMBIT)) {
            call("ALARM:510");               // auto discrete not present
        }
    }

    public void rrz2() {
        rrzerosb();
        endradar();
    }

    // ================================================================ designate (RRDESSM / RRDESNB)
    /**
     * RRDESSM: designate RR to RRTARGET (stable member). Returns 0 if LOS is not in mode 2 coverage (lunar
     * surface), 1 if a vehicle maneuver is required, 2 if designating (no maneuver required).
     */
    public int rrdessm() {
        downflag("RRNBSW");
        call("CDUTRIG");
        call("*SMNB*");
        call("RRANGLES");                    // gimbal angles in present and alternate mode
        if (rrlimchk("MODEA")) {             // configuration for current mode OK
            return okdessm();
        }
        return trysws();
    }

    private int okdessm() {
        startdes();
        return 2;
    }

    /** TRYSWS: try the other antenna mode. */
    private int trysws() {
        rmodinv();
        if (!rrlimchk("MODEB")) {
            return nodessm();                // vehicle maneuver required
        }
        rmodinv();
        setBits("RADMODES", REMODBIT);       // flag for remode
        return okdessm();
    }

    /** NODESSM: re-invert mode and return to caller +1 (or +0 on the lunar surface). */
    private int nodessm() {
        rmodinv();
        if (lunsfchk()) {                    // cannot designate in mode 2 on the surface
            nordstal();
            return 0;
        }
        if (flag("RNDVZFLG")) { return 1; }  // not on Moon - call for attitude maneuver
        return 1;                            // ... but not in R29 (ENDOFJOB)
    }

    /** STARTDES: show designate required, set up counters, schedule BEGDES. */
    public void startdes() {
        setBits("RADMODES", DESIGBIT);
        if (!bit("RADMODES", REPOSBIT)) {    // error counters not already set up
            setrrecr();
            services.waitlist(2, "BEGDES");
        }
        desretrn();
    }

    /** DESRETRN: first pass through designate returns to caller; later passes end the job. */
    public void desretrn() {
        if (g("RADCADR") != 0) { return; }   // not the first pass: ENDOFJOB
        desrtrn();
    }

    public void desrtrn() { /* RELINT; BANKJUMP to caller in DESRET (+2) */ }

    /** NORDSTAL: zero RADCADR to wipe out anyone waiting in RADSTALL. */
    public void nordstal() {
        s("RADCADR", 0);
        desrtrn();
    }

    /** RRDESNB: designate RR to trunnion/shaft angles in TANG (V41), nav base coordinates. */
    public void rrdesnb() {
        downflag("LOSCMFLG");                // prevent DODES from going back to R21
        s("DESCOUNT", MAXTRIES);             // MAXTRYS
        if (!rrlimnb("TANG")) {              // try the other mode
            rmodinv();
            if (!rrlimnb("TANG")) { nodesnb(); return; }
            rmodinv();
            setBits("RADMODES", REMODBIT);
        }
        okdesnb();
    }

    private void okdesnb() {
        s("TANGNB", g("TANG"));
        s("TANGNB+1", g("TANG+1"));
        call("RRNB");                        // LOS in NB coords
        upflag("RRNBSW");
        startdes();
    }

    /** NODESNB: bad input angles - alarm 502. */
    private void nodesnb() {
        rmodinv();
        call("ALARM:502");
        clradmod();
    }

    // ================================================================ designate loop
    /** BEGDES: begin designation; remode first if requested. */
    public void begdes() {
        if (bit("RADMODES", REMODBIT)) { remode(); return; }
        desloop();
    }

    /** DESLOOP: two samples per second. */
    public void desloop() { services.waitlist(50, "STDESIG"); }

    /** STDESIG: check bounds, continuous-designate bit, end-of-problem and time limit. */
    public void stdesig() {
        if (bit("RADMODES", REPOSBIT)) { baddes(); return; }   // monitor found us out of bounds
        if (bit("RADMODES", CDESBIT)) {
            moredes(true);                   // continuous designate: skip bit-10 test
            return;
        }
        if (!bit("RADMODES", DESIGBIT)) { endradar(); return; }   // end of problem
        stdesig1();
    }

    /** STDESIG1: time limit test. */
    public void stdesig1() {
        int c = g("DESCOUNT");
        if (c != 0) { s("DESCOUNT", c - 1); moredes(false); return; }
        wand(12, ~B14_B2);                   // out of time: remove ECR enable and tracker
        baddes();
    }

    /** BADDES: remove designate flag, bad end. */
    public void baddes() {
        clrBits("RADMODES", DESIGBIT);
        rdbadend();
    }

    /** MOREDES: update gyro torque commands (DODES job), loop. */
    public void moredes(boolean cont) {
        services.fork("FINDVAC", 026, "DODES");
        desloop();
    }

    /**
     * DODES: compute RR trunnion/shaft commands (TANG/TANG+1) from the LOS in RRTARGET, test pointing
     * error against 0.5 degree, and output commands via RROUT. Shaft/trunnion command =
     * (sin/cos of present CDU angles dotted with LOS) scaled by RDESGAIN. In the interpretive code the LOS
     * is first extrapolated 1 s by LOSVEL and transformed SM to NB unless RRNBSW is set.
     */
    public void dodes() {
        s("TANG", g("CDUT"));
        s("TANG+1", g("CDUS"));
        double[] los = rrtarget;
        if (!flag("RRNBSW")) {               // target in stable member -> navigation base
            los = unit(vadd(vxsc(los, mlosv), vxsc(losvel, MCTOMS * 2 / 16384.0)));
            call("*SMNB*");
        }
        double t = frac(g("TANG")) * Math.PI;   // trunnion angle (half-revolutions scaled to 1.0)
        double sh = frac(g("TANG+1")) * Math.PI;
        // commands: shaft = V.(cos S, 0, -sin S); trunnion = V.U with U = (sinT sinS, cosT, sinT cosS)
        double shaftCmd = los[0] * Math.cos(sh) - los[2] * Math.sin(sh);
        double trunCmd = -(Math.sin(t) * Math.sin(sh) * los[0] + Math.cos(t) * los[1] + Math.sin(t) * Math.cos(sh) * los[2]);
        s("TANG", Agc.fromSigned((int) (trunCmd * RDESGAIN * 16384)));
        double polarity = bit("RADMODES", BIT12) ? -1 : 1;     // relay reverses shaft polarity in mode 2
        s("TANG+1", Agc.fromSigned((int) (polarity * -shaftCmd * RDESGAIN * 16384)));
        // URR in NB axes: (cosS cosT?, -sinT, ...) compared with the LOS: cos(error) vs cos(0.5 deg)
        double[] urr = {Math.sin(sh) * Math.cos(t), -Math.sin(t), Math.cos(sh) * Math.cos(t)};
        double cosErr = dot(urr, los);
        s("MPAC+1", cosErr - COS1_2DG * 4 >= 0 ? 0 : 1);       // zero if RR pointed OK, one if not
        if (bit("RADMODES", CDESBIT)) {
            dorrout();                       // continuous designate: commands without magnitude check
            return;
        }
        if (g("MPAC+1") == 0) {              // both axes within .5 degree
            if (!flag("LOKONSW")) { rrdesdun(); return; }
            wor(12, BIT14);                  // enable the tracker
        }
        dgoodchk();
    }

    /** DGOODCHK: data good received yet (bit 4 of channel 33, 0 = good)? */
    public void dgoodchk() {
        if ((chan(33) & BIT4) == 0) { rrdesdun(); return; }
        dorrout();
    }

    /** RRDESDUN: problem done - drop bit 10, reset LOSCMFLG, disable RR error counter. */
    public void rrdesdun() {
        clrBits("RADMODES", DESIGBIT);
        downflag("LOSCMFLG");
        wand(12, ~BIT2);
    }

    /** DORROUT: apply velocity adjustment unless P22/search, output commands, then LOS recompute counter. */
    public void dorrout() {
        if (flag("LOSCMFLG") || flag("SRCHOPTN")) {
            // multiply unit LOS by magnitude, add .5 s of relative velocity, unitize
            double[] v = vadd(vxsc(rrtarget, mlosv * 2), vxsc(losvel, MCTOMS * 2 / 16384.0 / 2));
            mlosv = abval(v) / 2;
            rrtarget = unit(v);
        }
        notp20();
    }

    /** NOTP20: output RROUT commands unless monitor reposition took over; handle LOSCOUNT. */
    public void notp20() {
        if (!bit("RADMODES", REPOSBIT)) { rrout(); }
        if (!flag("LOSCMFLG")) { return; }   // don't test LOS counter
        int c = g("LOSCOUNT");
        if (c != 0) { s("LOSCOUNT", c - 1); return; }   // DODESEND
        call("KILLTASK:DESLOOP+2");
        r21lem2();                           // time for a new LOS
    }

    // ================================================================ radar data read
    /** LRALT: landing radar altitude (all bits; 1 sample). */
    public void lralt() { initread(1, ALLREAD); }
    /** LRVELZ / LRVELY / LRVELX: landing radar velocity beams, five samples. */
    public void lrvelz() { initread(5, 016); }
    public void lrvely() { initread(5, 015); }
    public void lrvelx() { initread(5, 014); }
    /** RRRDOT: rendezvous radar range rate (1 sample). */
    public void rrrdot() { initread(1, 012); }
    /** RRRANGE: rendezvous radar range (1 sample). */
    public void rrrange() { initread(1, 011); }
    public static final int ALLREAD = 017;

    /** LRVEL: read velocity beam by index (0 = X, 1 = Y, 2 = Z) with five samples. */
    public void lrvel(int vbeam) {
        s("TIMEHOLD", vbeam);
        if (vbeam == 2) { lrvelz(); } else if (vbeam == 1) { lrvely(); } else { lrvelx(); }
    }

    /**
     * INITREAD: set up a radar read of {@code n} samples for radar select code {@code code}: compute mid-point
     * time, zero the sum, select the radar on CHAN13.
     */
    public void initread(int n, int code) {
        nsamp = n;
        samplim = n + 1;
        timehold = (n * 3) / 2;              // MP BIT3: DT of midpoint of nominal sampling interval
        oldatagd = chan(33) & DGBITS;
        wand(13, ~ALLREAD);                  // remove all radar bits
        wor(13, code);                       // set new radar bits
        timehold += g("TIME2");              // time of nominal midpoint
        samplsum = 0;
    }

    /**
     * RADAREAD: radar-done interrupt - read one sample from RNRAD. Handles LR (altitude, velocity) and RR
     * (range, range rate) data, scale changes, data-good checks and sample averaging. Ends via
     * RGOODEND / RDBADEND (BANKJUMP into GOODEND / BADEND with RADSTALL waiting).
     */
    public void radaread() {
        dnindex = chan(13) & 07;             // radar select
        if (dnindex != 0) { s("DNRRANGE" + (dnindex - 1), g("RNRAD")); }    // data for downlist
        int c = samplim;
        if (c < 0) {                         // 520 alarm - too many reads
            call("ALARM:520");
            return;
        }
        if (c == 0) { nomore(); return; }
        plenty();
    }

    /** NOMORE: no more tries - alarm 521 unless LRBYPASS / R04 running. */
    public void nomore() {
        if (flag("LRBYPASS") && flag("R04FLAG") == false) {
            call("ALARM:521");               // P20 wants the alarm
        }
        samplim = -1;
        rdbadend();
    }

    /** BADRAD: bad radar data. */
    public void badrad() { samplim = -1; rdbadend(); }

    /** PLENTY: decide which radar (bit 3 of channel 13: 0 = RR, 1 = LR) and process the sample. */
    public void plenty() {
        samplim = samplim - 1;
        if ((chan(13) & BIT3) == 0) { rendrad(); return; }
        if (r77check()) { return; }          // R77 quits here
        lrposchk();
    }

    /** LRPOSCHK: check LR in desired position (alarm 522 if not). */
    public void lrposchk() {
        if (((g("RADMODES") ^ chan(33)) & BIT6) != 0) {
            call("ALARM:522");
            badrad();
            return;
        }
        velchk();
    }

    /** VELCHK: velocity beams versus altitude. */
    public void velchk() {
        if (((chan(13) ^ 03) & 03) == 0) { lrheight(); return; }   // take an LR range reading
        int rn = g("RNRAD");
        long sample = (rn & Agc.WORD_MASK & 0x3FFF) + g("LVELBIAS");
        long itemp = sample + (((rn << 1) & BIT1) << 14);
        dgcheck(BIT8, itemp);                // data good not checked until after reading data
        if (nsamp != 0) { noend(); return; }
        goodrad();
    }

    /** GOODRAD: enough good data gathered. */
    public void goodrad() {
        samplim = -1;
        clrBits("RADMODES", g("ITEMP1"));    // reset data fail flags for SETTRKF
        radlites();
        rgoodend();
    }

    private void noend() {
        nsamp = nsamp - 1;
        resample();
    }

    /** RESAMPLE: see if any more tries should be made. */
    public void resample() {
        if (samplim == 0) { datafail(); return; }
        wor(13, BIT4);                       // reset activity bit
    }

    /** LRHEIGHT: LR altitude sample (data good bit 5, scale bit 9). */
    public void lrheight() {
        s("ITEMP1", BIT5);
        scalechk(BIT9);
    }

    /** RENDRAD: rendezvous radar sample - antenna limits, CDU failure, range / range-rate. */
    public void rendrad() {
        if (bit("RADMODES", REPOSBIT)) { badrad(); return; }     // antenna out of limits
        if (bit("RADMODES", RCDUFBIT)) { badrad(); return; }     // RR CDU failed
        s("ITEMP1", BIT4);                   // data good bit position in channel 33
        boolean rdot = (chan(13) & BIT1) != 0;                   // RR RDOT select
        if (rdot) { radin(true, 0); return; }                    // no scale check for RR RDOT
        scalechk(BIT3);
    }

    /** SCALECHK: compare scale status now (channel 33) against RADMODES; a change is a bad sample. */
    public void scalechk(int scaleBit) {
        s("L", scaleBit);
        int now = chan(33) & scaleBit;
        int before = g("RADMODES") & scaleBit;
        if ((now ^ before) != 0) { scalchng(scaleBit); return; }
        radin(false, scaleBit);
    }

    /** RADIN: split RNRAD into magnitude and scale bits, add to the sample sum after scale adjustment. */
    public void radin(boolean rdot, int scaleBit) {
        long mag = g("RNRAD") & 0x3FFF;      // POSMAX mask
        long extra = (g("RNRAD") << 1) & BIT1;
        long itemp = mag + (extra << 14);
        if (rdot) {
            itemp -= g("RDOTBIAS");          // throw out bias
        } else if ((chan(33) & scaleBit) == 0 && scaleBit != 0) {
            itemp = scaladj(itemp, mag);
        }
        dgcheck2(itemp);
    }

    /** SCALADJ: adjust for high scale (RR x8 or LR altitude x2/x4 depending on HISCALIM). */
    private long scaladj(long itemp, long mag) {
        if ((g("DNINDEX") & BIT3) == 0 && dnindex < 4) { return itemp << 3; }    // RR range high scale x8
        // LRSCK: LR altitude - 2x or 4x according to HISCALIM
        if (itemp == 0 || mag > HISCALIM) {
            setBits("FLGWRD11", SCABBIT);
            return itemp << 2;
        }
        clrBits("FLGWRD11", SCABBIT);
        return itemp << 2;
    }

    private void dgcheck2(long itemp) { dgcheck(g("ITEMP1"), itemp); if (nsamp != 0) { noend(); } else { goodrad(); } }

    /** SCALCHNG: scale changed - update RADMODES and OLDATAGD, set RNGSCFLG, bad radar. */
    public void scalchng(int scaleBit) {
        s("RADMODES", g("RADMODES") ^ scaleBit);
        oldatagd = chan(33) & DGBITS;
        upflag("RNGSCFLG");                  // for LRS24.1
        badrad();
    }

    /** R77CHECK: if R77 is running update LR data-good bits in RADMODES and end the read. */
    public boolean r77check() {
        if (!flag("R77FLAG")) { return false; }
        int good = chan(33) & BITS5_8;
        s("RADMODES", (g("RADMODES") & ~BITS5_8) | good);
        rgoodend();
        return true;
    }

    /**
     * DGCHECK: update data-good bits in OLDATAGD and require them on before and after the sample;
     * otherwise RESAMPLE. On success add the sample to SAMPLSUM.
     */
    public void dgcheck(int bitmask, long sample) {
        s("ITEMP1", bitmask);
        int now = chan(33) & bitmask;
        int stillBad = (oldatagd | now) & bitmask;       // 1 = data good absent (active low)
        oldatagd = now;
        if (stillBad != 0) { resample(); return; }
        samplsum += sample;
    }

    /** DATAFAIL: unable to gather N samples - flag failure in RADMODES, use last sample only. */
    public void datafail() {
        setBits("RADMODES", g("ITEMP1"));
        radlites();
        nomore();
    }

    // ----------------------------------------------------------------- landing radar position 2
    /** LRPOS2: command LR antenna to position 2 and wait for the inbit. */
    public void lrpos2() {
        setBits("RADMODES", LRPOSBIT);
        if ((chan(33) & BIT7) == 0) { radnoop(); return; }       // already there
        wor(12, BIT13);                      // command to position 2
        services.waitlist(600, "LRPOSCAN");  // start scanning after 6 s
    }

    /** LRPOSCAN: 15 samples, once per second. */
    public void lrposcan() { lrposnxt(14); }

    public void lrposnxt(int count) {
        samplim = count;
        if ((chan(33) & BIT7) == 0) { lastlrdt(); return; }      // wait final second for bounce
        if (count != 0) { services.waitlist(100, "LRPOSNXT"); return; }
        wand(12, ~BIT13);                    // time up: disable command and alarm
        rdbadend();
    }

    /** RADNOOP: nothing further requested - RGOODEND via waitlist. */
    public void radnoop() { services.waitlist(1, "RGOODEND"); }

    /** LASTLRDT: wait 2 s after the inbit for antenna bounce to die out. */
    public void lastlrdt() {
        services.waitlist(200, "LASTLRDT2");
    }

    public void lastlrdt2() {
        wand(12, ~BIT13);
        rgoodend();
    }

    /** ENDRADAR: check RR CDU fail before ending. */
    public void endradar() {
        if (bit("RADMODES", RCDUFBIT)) { rdbadend(); return; }
        rgoodend();
    }

    public void rgoodend() { call("GOODEND"); }
    public void rdbadend() { call("BADEND"); }

    // ================================================================ LPS20 (LOS determination)
    /** LOADTIME: RTB LOADTIME - present time in centiseconds. */
    private double loadtime() { return g("TIME2") * 16384.0 + g("TIME1"); }

    /**
     * LPS20.1: LOS determination. Extrapolates LM (LEMCONIC, skipped in R21 on the surface) and CSM
     * (CSMCONIC) to TDEC1, forms relative velocity (LOSVEL, SM, B-7) and the unit LOS (RRTARGET) with
     * magnitude MLOSV; range over 400 NM leads to 526ALARM.
     */
    public void lps20_1() {
        boolean skipLem = !flag("LOSCMFLG") ? false : flag("SURFFLAG");
        if (!skipLem) {
            call("LEMCONIC");                // extrapolate LEM
            lmpos = vec("RATT");             // B-29
            lmvel = vec("VATT");             // B-7
            tdec1 = g("TAT");
        }
        call("CSMCONIC");                    // extrapolate CSM
        double[] vatt = vec("VATT"), ratt = vec("RATT");
        losvel = vxsc(vsub(vatt, lmvel), 2);                       // MXV REFSMMAT ; VSL1
        call("KILLTASK:DESLOOP+2");          // kill the task which calls DODES
        double[] los = vsub(ratt, lmpos);
        if (flag("RNDVZFLG")) { los = vxsc(los, 1); }              // otherwise VSL 9D shift
        mlosv = abval(los);                  // range greater than 400 NM overflows -> 526ALARM
        if (mlosv > FHNM * 2) {
            alarm526();
            return;
        }
        rrtarget = unit(los);                // MXV REFSMMAT ; VSL1 (stable member)
        downflag("RRNBSW");
        ls21x();
    }

    /** LPS20.2: range test, returns 1 (TOFAR) if LOS magnitude exceeds 400 NM, else 0. */
    public int lps20_2() { return mlosv - FHNM > 0 ? 1 : 0; }

    // ================================================================ LRS22 (RR data read)
    /**
     * LRS22.1: read RR range rate and range (one sample each), CDU angles and time; compute RR LOS (RRNB)
     * and the angle to the state-vector LOS. Returns MPAC code: 0 normal, 1 could not read radar,
     * 2 LOS not within 3 degrees.
     */
    public int lrs22_1() {
        downflag("RNGSCFLG");
        s("L", chan(33) & BIT3);             // RR range scale
        s("RADMODES", (g("RADMODES") & ~RRRSBIT) | (chan(33) & BIT3));
        for (;;) {                           // READRDOT
            rrrdot();
            if (services.radstall() == RadResult.BAD) { return 1; }  // EREXIT1
            double mid = timehold;           // mark time = midpoint
            long rdot = samplsum;            // range-rate reading
            s("RDOTMSAV", (int) rdot);
            int cdux = g("CDUX"), cduy = g("CDUY"), cduz = g("CDUZ");
            double tcdu = g("TIME2");        // time of CDUY readings
            s("TANG", g("CDUT"));            // trunnion and shaft angles for RRNB
            s("TANG+1", g("CDUS"));
            rrrange();
            if (services.radstall() == RadResult.BAD) {
                if (flag("RNGSCFLG")) { continue; }   // CHEXERR: scale changed - read again
                return 1;
            }
            s("RANGRDOT", g("DNRRANGE+1"));  // raw data for downlink
            mktime = mid;
            s("AIG", cduy);
            s("AMG", cduz);
            s("TANGNB", g("TANG"));
            s("TANGNB+1", g("TANG+1"));
            s("AOG", cdux);
            // RDOT -> m/cs: reading scaled B-28, .6278 ft/s per bit; RDOTCONV
            rdotm = rdot * 0.6278 * 0.3048 / 100;
            rrtrun = Agc.toSigned(g("TANG")) / 16384.0 / 2;         // CDULOGIC: DP revolutions
            rrshaft = Agc.toSigned(g("TANG+1")) / 16384.0 / 2;
            rm = samplsum * 4 * 0.9;         // RANGCONV: range units to metres (B-29)
            call("RRNB");                    // radar LOS in NB axes -> RRBORSIT
            tdec1 = tcdu;
            lps20_1();                       // state vector LOS at time of CDU read
            s("CDUSPOT", g("AIG"));          // IMU CDU angles at marktime
            s("CDUSPOT+2", g("AMG"));
            s("CDUSPOT+4", g("AOG"));
            call("TRG*SMNB");                // rotate LOS at marktime SM to NB
            double ang = Math.acos(Math.max(-1, Math.min(1, dot(rrtarget, rrborsit)))) / (2 * Math.PI);
            dsptem1 = ang;                   // stored for possible display
            return ang - THREEDEG < 0 ? 0 : 2;
        }
    }

    /** LRS22.2: check RR LOS within 30 degrees of the spacecraft +Z axis (returns 0 = OK, 1 = not). */
    public int lrs22_2() {
        double z = rrborsit[2];
        double ang = Math.acos(Math.max(-1, Math.min(1, z))) / (2 * Math.PI);
        return ang - DEG30 < 0 ? 0 : 1;
    }

    /**
     * LSR22.3: incorporate RR range, range rate, shaft and trunnion angle data. Integrates the vehicle
     * state(s) to the mark time, then for each measurement builds the B vectors, DELTAQ and VARIANCE and
     * calls INCORP1/INCORP2 through LGCUPDTE. Finally returns to R22LEM93.
     */
    public void lsr22_3() {
        call("GRP2PC");
        upflag("DMENFLG");
        if (flag("SURFFLAG")) { lsr22_4(); return; }
        if (!flag("VEHUPFLG")) { dolem(); return; }
        call("INTSTALL");                    // LM precision integration
        downflag("VINTFLAG");
        setiflgs();
        intgrcal();
        call("GRP2PC");
        call("INTSTALL");
        downflag("DIM0FLAG");
        if (flag("RENDWFLG")) { upflag("DIM0FLAG"); upflag("D6OR9FLG"); }   // CSM with W-matrix integration
        upflag("VINTFLAG");
        downflag("INTYPFLG");
        upflag("STATEFLG");
        intgrcal();
        marktest();
    }

    private void dolem() {
        call("INTSTALL");
        upflag("VINTFLAG");
        setiflgs();
        intgrcal();
        call("GRP2PC");
        call("INTSTALL");
        downflag("DIM0FLAG");
        if (flag("RENDWFLG")) { upflag("DIM0FLAG"); upflag("D6OR9FLG"); }   // LM with W-matrix integration
        downflag("INTYPFLG");
        downflag("VINTFLAG");
        upflag("STATEFLG");
        intgrcal();
        marktest();
    }

    /** MARKTEST: reinitialise W if invalidated, then range, range-rate, shaft, trunnion incorporation. */
    private void marktest() {
        if (!flag("RENDWFLG")) { wlinit(); }
        rangebq();
    }

    /** RANGEBQ: R65 call (not on surface), scale shift, then range incorporation. */
    public void rangebq() {
        if (!flag("SURFFLAG")) {
            s("R65CNTR", 0);
            r65lem();
        }
        rangebq1();
    }

    private void rangebq1() {
        scalshft = flag("LMOONFLG") ? 0 : 2;   // 0 - Moon, 2 - Earth sphere of influence
        call("GRP2PC");
        s("WHCHREAD", 1);                    // range code (1) for R3 in noun 49
        double rvarmin = g("RVARMIN") / 4096.0 / Math.pow(2, 28);    // SR 28D to triple precision B-40
        getulc();
        boolean lmCorrected = !flag("VEHUPFLG");
        double[] b0 = lmCorrected ? vxsc(ulc, -1) : ulc;     // B0, complemented if LM being corrected
        System.arraycopy(b0, 0, bvector, 0, 3);
        System.arraycopy(new double[3], 0, bvector, 3, 3);   // B1 = 0
        System.arraycopy(new double[3], 0, bvector, 6, 3);   // B2 = 0
        deltaq = rm - rlc;                   // RM - (magnitude RCSM - RLM)
        double rangevar = g("RANGEVAR");
        variance = rlc * rlc * rangevar;
        double q = variance - rvarmin;       // DCOMP TAD
        if (q < 0) { variance = rvarmin; }
        lgcupdte();                          // range incorporation (INCORP1)
        s("WHCHREAD", 2);                    // range-rate code (2)
        getulc();
        double[] vlc = vsub(vec("VCVCSM"), vec("VCVLEM"));    // VC - VL
        double rdotEst = dot(ulc, vlc);
        variance = rdotEst * rdotEst * g("RATEVAR");
        double vvarmin = g("VVARMIN") / 16.0;
        if (vvarmin - variance > 0) { variance = vvarmin; }   // use minimum variance
        deltaq = rdotm - rdotEst;
        double[] b0r = cross(cross(ulc, vlc), ulc);          // (ULC x VLC) x ULC
        bvector = new double[9];
        System.arraycopy(lmCorrected ? vxsc(b0r, -1) : b0r, 0, bvector, 0, 3);
        System.arraycopy(lmCorrected ? vxsc(ulc, -rlc) : vxsc(ulc, rlc), 0, bvector, 3, 3);   // B1 = RLC * ULC
        variance = variance * rlc * rlc;
        lgcupdte();
        call("GRP2PC");
        if (flag("SURFFLAG")) { rendend(); return; }          // angles not done on surface
        mxmymz();
    }

    /** MXMYMZ: gimbal angle vectors MX, MY, MZ in reference coordinates from AIG/AMG/AOG. */
    public void mxmymz() {
        s("CDUSPOT", g("AIG"));
        s("CDUSPOT+2", g("AMG"));
        s("CDUSPOT+4", g("AOG"));            // gimbal angles for TRG*NBSM
        call("TRG*NBSM");
        mx = vec("MX");                      // UNITX through NBSM and REFSMMAT
        my = vec("MY");                      // UNITY
        mz = vec("MZ");                      // UNITZ
        shaftbq();
    }

    /** SHAFTBQ: shaft angle incorporation (code 3). */
    public void shaftbq() {
        radarang();
        s("WHCHREAD", 3);
        double sinth = dot(ulc, mx) * 2;
        double costh = dot(ulc, mz) * 2;
        double shaftEst = Math.atan2(sinth, costh) / (2 * Math.PI);
        deltaq = (rrshaft - shaftEst) * TWO_PI_8 * rxz;
        double[] b0 = unit(cross(ulc, my));
        bvector = new double[9];
        System.arraycopy(flag("VEHUPFLG") ? b0 : vxsc(b0, -1), 0, bvector, 0, 3);
        bvector[8] = -rxz;                   // B2 shaft term
        variance = (g("SHAFTVAR") + IMUVAR) * rxz * rxz;
        lgcupdte();
        call("GRP2PC");
        trunbq();
    }

    /** TRUNBQ: trunnion angle incorporation (code 4). */
    public void trunbq() {
        radarang();
        s("WHCHREAD", 4);
        double[] b0 = cross(cross(ulc, my), ulc);
        bvector = new double[9];
        System.arraycopy(flag("VEHUPFLG") ? b0 : vxsc(b0, -1), 0, bvector, 0, 3);
        bvector[7] = -rxz;
        variance = (g("TRUNVAR") + IMUVAR) * rxz * rxz;
        double trunEst = Math.asin(Math.max(-1, Math.min(1, sintheta))) / (2 * Math.PI);
        deltaq = (rrtrun - trunEst) * TWO_PI_8 * rxz;
        lgcupdte();
        call("GRP2PC");
        rendend();
    }

    /** RENDEND: GOTO R22LEM93. */
    public void rendend() { r22lem93(); }

    /** LSR22.4: lunar surface case - LM position from surface, 6x6 W matrix. */
    public void lsr22_4() {
        call("INTSTALL");
        upflag("STATEFLG");
        downflag("VINTFLAG");                // get LM pos and vel in reference coordinates
        intgrcal();
        call("GRP2PC");
        downflag("DMENFLG");                 // 6x6 for incorporation
        call("INTSTALL");
        if (g("MARKCTR") == 0) {             // first time through: initialise 6x6 W matrix
            wlinit();
            upflag("VINTFLAG");
            setiflgs();
            intgrcal();
            rangebq();
            return;
        }
        downflag("D6OR9FLG");
        upflag("DIM0FLAG");
        upflag("VINTFLAG");
        downflag("INTYPFLG");
        intgrcal();
        rangebq();
    }

    /** INTGRCAL: integrate to MKTIME. */
    public void intgrcal() {
        tdec1 = mktime;
        call("INTEGRV");
    }

    /**
     * WLINIT: initialise the W matrix (161 words) to zero, then diagonal elements from WRENDPOS, WRENDVEL
     * (or WSURFPOS / WSURFVEL on the surface), WSHAFT and WTRUN; sets RENDWFLG and zeroes MARKCTR.
     */
    public void wlinit() {
        w = new double[WSIZE + 1];
        double pos = flag("SURFFLAG") ? g("WSURFPOS") : g("WRENDPOS");
        pos = pos / 32.0;                    // SR 5: shift to B-19 scale
        w[0] = w[8] = w[16] = pos;
        double vel = flag("SURFFLAG") ? g("WSURFVEL") : g("WRENDVEL");
        w[72] = w[80] = w[88] = vel;
        w[144] = g("WSHAFT");
        w[152] = g("WTRUN");
        upflag("RENDWFLG");                  // W matrix valid
        s("MARKCTR", 0);
    }

    /** GETULC: ULC = unit(RCSM - RLM) with magnitude RLC, positions scaled for Earth/Moon sphere. */
    public void getulc() {
        double[] rl = vadd(vxsc(vec("DELTALEM"), 1.0 / (1 << (9 - 0))), vec("RCVLEM"));
        double[] rc = vadd(vxsc(vec("DELTACSM"), 1.0 / (1 << 9)), vec("RCVCSM"));
        double[] d = vsub(rc, rl);
        rlc = abval(d);
        ulc = unit(d);
    }

    /** RADARANG: sin theta = -MY.ULC, RXZ = RLC cos theta (normalised). */
    public void radarang() {
        getulc();
        sintheta = -2 * dot(my, ulc);
        rxz = rlc * Math.sqrt(Math.max(0, 1 - sintheta * sintheta));
    }

    /**
     * LGCUPDTE: INCORP1 (state vector update), compute displayed position / velocity corrections
     * (R22DISP) and compare against RMAX / VMAX; on exceeding go to R22LEM96 display else ASTOK/INCORP2.
     */
    public void lgcupdte() {
        call("INCORP1");
        double dv = abval(new double[] {g("DELTAX+6"), g("DELTAX+8"), g("DELTAX+10")});
        r22disp2 = dv;                       // velocity display B-7
        double dr = abval(new double[] {g("DELTAX"), g("DELTAX+2"), g("DELTAX+4")});
        r22disp = dr;                        // position display B-29
        if (g("RMAX") * 1024.0 - r22disp < 0) { r22lem96(); return; }
        if (g("VMAX") - r22disp2 < 0) { r22lem96(); return; }
        astok();
    }

    /** ASTOK: CALL INCORP2. */
    public void astok() { call("INCORP2"); }

    // ================================================================ R24 search (LRS24)
    /** LRS24.1: search routine; N = search pattern point (0 = along LOS, 1..6 around it). */
    public void lrs24_1() {
        s("NSRCHPNT", 0);
        chksrch();
    }

    /** CHKSRCH: enable auto track, check SRCHOPT, schedule CALLDGCH in 6 s, compute pattern point. */
    public void chksrch() {
        wor(12, BIT14);                      // auto track enable to radar
        if (!flag("SRCHOPTN")) { return; }   // terminate job
        services.waitlist((int) SEC6, "CALLDGCH");
        if (!bit("RADMODES", REMODBIT)) { return; }     // remode in progress - wait six seconds
        tdec1 = loadtime() + SEC1_5;         // LOS at present time + 1.5 s
        call("LEMCONIC");
        rlmsrch = vec("RATT");               // save LEM position
        savlemv = vec("VATT");               // save LEM velocity
        tdec1 = g("TAT");
        call("CSMCONIC");
        losdesrd = unit(vsub(vec("RATT"), rlmsrch));   // desired LOS
        vxrcm = unit(cross(unit(vec("VATT")), vec("RATT")));
        savlemv = vxsc(vsub(vec("VATT"), savlemv), 2);     // VLC (stable member)
        int n = g("NSRCHPNT");
        if (n == 0) { desglos(); return; }
        if (n == 1) { calcxy(); return; }
        double[] uxpr = uxvect, uypr = uyvect;     // rotate X-Y axes 60 degrees to the next search point
        uxvect = unit(vadd(vxsc(uypr, SIN60DEG), vxsc(uxpr, COS60DEG)));
        uyvect = unit(vsub(vxsc(uypr, COS60DEG), vxsc(uxpr, SIN60DEG)));
        offcalc();
    }

    /** OFFCALC: offset vector K(UY) added to the LOS defines the desired point in the search pattern. */
    public void offcalc() {
        double[] p = unit(vadd(vxsc(uyvect, OFFSTFAC), losdesrd));
        rrtarget = vxsc(p, 2);                 // MXV REFSMMAT ; VSL1 (stable member coordinates)
        contdesg();
    }

    /** CONTDESG: LOSVEL = SAVLEMV; kill pending designate task; designate to RRTARGET. */
    public void contdesg() {
        losvel = savlemv;
        call("KILLTASK:DESLOOP+2");
        setBits("RADMODES", CDESBIT);        // continuous designate wanted
        int rc = rrdessm();
        if (rc != 2) { limalarm(); return; }
        omegcalc();
    }

    /** OMEGCALC: OMEGA = angle between RR LOS and spacecraft Z axis, for R2 display. */
    public void omegcalc() {
        s("TANGNB", g("CDUT"));
        s("TANGNB+1", g("CDUS"));
        call("RRNB");
        omegdisp = Math.acos(Math.max(-1, Math.min(1, g("RRNB_Z") / 16384.0)));
    }

    /** CALCXY: UX = (VLM x RLM) x LOS ; UY = LOS x UX. */
    public void calcxy() {
        uxvect = unit(cross(vxrcm, losdesrd));
        uyvect = unit(cross(losdesrd, uxvect));
        offcalc();
    }

    /** DESGLOS: N = 0, designate along LOS. */
    public void desglos() {
        rrtarget = vxsc(losdesrd, 2);        // MXV REFSMMAT ; VSL1
        contdesg();
    }

    /** CALLDGCH: task - if rendezvous flag set, schedule DATGDCHK job at priority 25. */
    public void calldgch() {
        if (!flag("RNDVZFLG")) { return; }
        services.fork("FINDVAC", 025, "DATGDCHK");
    }

    /** DATGDCHK: if data good, store 11111 for R1 display; else advance to next pattern point (wrap after 6). */
    public void datgdchk() {
        if ((chan(33) & BIT4) == 0) { store1s(); return; }
        int n = g("NSRCHPNT");
        if (n == 6) { lrs24_1(); return; }   // reset N = 0 and start around again
        s("NSRCHPNT", n + 1);
        chksrch();
    }

    /** STORE1S: store 11111 for display in R1, delete designate task. */
    public void store1s() {
        s("DATAGOOD", ALL1S);
        call("KILLTASK:DESLOOP+2");
    }

    /** LIMALARM: alarm 527 - LOS not in mode 2 coverage (P22) or vehicle maneuver required (P20). */
    public void limalarm() {
        call("ALARM:527");
        call("KILLTASK:CALLDGCH");
    }

    /** GOTOV56: P20 terminates by V56 coding instead of GOTOPOOH (DCA VB56CADR -> TRMTRACK). */
    public void gotov56() { call("TRMTRACK"); }

    // ================================================================ R29
    /** R29: LM rendezvous radar designation service (called from servicer) with up-to-date LOS. */
    public void r29() {
        if (!bit("RADMODES", DESIGBIT)) {    // designation not already on
            setBits("RADMODES", DESIGBIT);
            wand(12, ~BIT14);                // remove RR track enable discrete
            downflag("LOSCMFLG");            // designate loop off
            clrBits("STATE", OLDESBIT);      // designate loop not requested
            setrrecr();                      // enable RR error counters
            if (bit("RADMODES", ANTENBIT)) { // mode 2
                services.waitlist(1, "PREPOS29");
                setBits("RADMODES", REPOSBIT);
            } else {                         // mode 1: must remode, needs own job to RADSTALL in
                services.fork("NOVAC", 021, "R29REM0J");
                clrBits("RADMODES", DESIGBIT);
                setBits("RADMODES", REMODBIT);
            }
            call("NOR29NOW");
            return;
        }
        r29Los();
    }

    /** R29REM0J: job for remode (REMODE runs as a task; wait with RADSTALL). */
    public void r29rem0j() {
        services.waitlist(1, "REMODE");
        services.radstall();
    }

    /** PREPOS29: task - drive trunnion to 180 degrees, reposition off. */
    public void prepos29() {
        rrtonly(040000);                     // NEGMAX: -180 deg
        clrBits("RADMODES", REPOSBIT);
    }

    /** R29.LOS: up-to-date LOS (SM) from CSM state: R(CSM) - R + (V(CSM) - V)(T - PIPTIME). */
    public void r29Los() {
        double dt = (g("TIME2") - g("PIPTIME")) / 16384.0;   // B-17
        double[] vrel = vsub(vec("V(CSM)"), vec("V"));
        double[] los = vsub(vadd(vec("R(CSM)"), vxsc(vrel, dt)), vec("R"));
        if (!flag("LOSCMFLG")) { setup29d(los); }   // designate loop off: change LOSCMFLG and set up new data
    }

    /** SETUP29D: LOSSM = LOS, LOSVDT/4 = .5 s of LOS velocity; request the designate loop if needed. */
    public void setup29d(double[] los) {
        rrtarget = los;
        losvel = vxsc(losvel, HALF_SEC_B17 / 16384.0);
        downflag("LOSCMFLG");                // let R29DLOOP use new data
        if (!bit("STATE", OLDESBIT)) {
            setBits("STATE", OLDESBIT);
            services.waitlist(g("PIPCTR") != 0 ? 100 : 4, "BEGDES29");
        }
    }

    /** BEGDES29: start R29DODES job twice a second. */
    public void begdes29() {
        services.fork("NOVAC", 021, "R29DODES");
        r29dloop();
    }

    /** R29DLOOP: .5 s delay; quit if designation called off; wait for new data. */
    public void r29dloop() {
        services.waitlist(50, "R29DLOOP2");
    }

    public void r29dloop2() {
        if (!bit("RADMODES", DESIGBIT)) { return; }   // designation called off
        if (!flag("LOSCMFLG")) {             // SETUP29D supplying new data
            upflag("LOSCMFLG");              // show that designate loop is on
            begdes29();
            return;
        }
        services.waitlist(1, "R29DLOOP2");   // wait a centisecond for new data
    }

    /**
     * R29DODES: job twice a second - project LOS 1/2 s and 1 s ahead, unitize, convert to NB axes with the
     * CDU angles, compute raw shaft and trunnion commands, scale by RR29GAIN, and send through RROUT unless
     * data good is present (then RR reading is requested via R29READ).
     */
    public void r29dodes() {
        double[] los = rrtarget.clone();
        double[] half = vadd(los, losvel);   // 1/2 second ahead
        double[] one = vadd(half, losvel);   // 1 second ahead
        double[] ulos = unit(half);
        int cdut = g("CDUT"), cdus = g("CDUS");
        s("SAVECDUT", cdut);
        s("SAVECDUT+1", cdus);
        double t = frac(cdut) * Math.PI, sh = frac(cdus) * Math.PI;
        // ULOSNB' (1 s ahead) used for raw commands; ULOSNB (1/2 s) for cos(error)
        double[] ulosnbp = unit(one);
        double cosErr = ulos[0] * Math.sin(sh) * Math.cos(t) - ulos[1] * Math.sin(t) + ulos[2] * Math.cos(sh) * Math.cos(t);
        if (cosErr > 1 - 1e-9) { wor(12, BIT14); }     // plus overflow: set selftrack enable
        double shaftCmd = ulosnbp[0] * Math.cos(sh) - ulosnbp[2] * Math.sin(sh);
        double trunCmd = ulosnbp[0] * Math.sin(t) * Math.sin(sh) + ulosnbp[1] * Math.cos(t) + ulosnbp[2] * Math.sin(t) * Math.cos(sh);
        s("TANG", Agc.fromSigned((int) (trunCmd * RR29GAIN * 16384)));
        s("TANG+1", Agc.fromSigned((int) (shaftCmd * RR29GAIN * 16384)));
        if ((chan(33) & BIT4) == 0) {        // DGOOD? data-good present
            r29lokon();
            return;
        }
        rrout();                             // data-good absent, so send commands
        end29dod();
    }

    /** R29LOKON: designation over; disable error counters; request reading. */
    public void r29lokon() {
        clrBits("RADMODES", DESIGBIT);
        wand(12, ~BIT2);
        setBits("FLAGWRD3", READRBIT);       // reading has been requested
        services.waitlist(g("PIPCTR") != 0 ? 100 : 4, "R29READ");
        end29dod();
    }

    private void end29dod() { downflag("LOSCMFLG"); }

    /** R29READ: waitlist task - start reading job; two seconds later see if reading still allowed. */
    public void r29read() {
        services.fork("NOVAC", 026, "R29RDJOB");
        services.waitlist(200, "R29READ2");
    }

    public void r29read2() {
        if (bit("FLAGWRD3", READRBIT)) { r29read(); }
    }

    /** R29RDJOB: read RR range rate and range and lay out the R29 downlink. */
    public void r29rdjob() {
        if (flag("NOR29FLG") || bit("RADMODES", AUTOMBIT)) { endrrd29(); return; }
        rrrdot();
        if (services.radstall() == RadResult.BAD) { endrrd29(); return; }
        long rdot = samplsum;
        s("MPAC", (int) timehold);           // time of RR reading for downlink
        r29range(rdot);
    }

    private void r29range(long rdot) {
        rrrange();
        if (services.radstall() == RadResult.BAD) {   // R29RRR?: scale change?
            if (flag("RNGSCFLG")) {
                downflag("RNGSCFLG");
                r29range(rdot);              // read again
                return;
            }
            endrrd29();
            return;
        }
        rm = samplsum;                       // copycycle to lay out new R29 downlink
        mktime = timehold;
        s("TRKMKCNT", 1);                    // downlink data is consistent
    }

    /** ENDRRD29: leave RR reading mode; discredit downtelemetry. */
    public void endrrd29() {
        s("TRKMKCNT", 0);
        downflag("READRFLG");
        wand(12, ~BIT14);                    // remove track-enable discrete
    }

    // ================================================================ V67 (W matrix RMS)
    /** V67CALL: compute and display W matrix RMS values (V06 N99), allow load of new values. */
    public void v67call() {
        v67ww();
        s("WWBIAS+2", g("WWPOS"));           // save present N99 values for comparison after the display
        s("WWBIAS+4", g("WWVEL"));
        s("WWBIAS+6", g("WWBIAS"));
        for (;;) {
            Response r = services.display("GOXDSPF", V06N99);
            if (r == Response.TERMINATE) { return; }              // ENDEXT
            if (r == Response.PROCEED) { break; }
        }
        int sum = 0;
        for (int i = 5; i >= 0; i--) {       // N99LOOP: the sum of all differences must be zero
            sum += Agc.toSigned(g("WWPOS+" + i)) - Agc.toSigned(g("WWPOS+" + (i + 6)));
        }
        if (sum != 0) { upflag("V67FLAG"); } // V06N9933
        if (!flag("V67FLAG")) { return; }    // ENDEXT
        double pos = g("WWPOS") * 32 * 2, vel = g("WWVEL") * 32 * 2;
        double bias = g("WWBIAS") * Math.pow(2, 10);          // shift from noun scaling (B-5) to internal (B+5)
        if (flag("SURFFLAG")) {
            s("WSURFPOS", (int) pos);
            s("WSURFVEL", (int) vel);
        } else {
            s("WRENDPOS", (int) pos);
            s("WRENDVEL", (int) vel);
        }
        s("WTRUN", (int) bias);              // V67CLRF
        s("WSHAFT", (int) bias);
        downflag("RENDWFLG");
    }

    /**
     * V67WW: RMS of the position, velocity and bias diagonal blocks of the W matrix:
     * WWPOS = sqrt(sum W(54D+..)^2), WWVEL from W 108D+, WWBIAS from W 162D+. Overflow gives DPPOSMAX.
     */
    public void v67ww() {
        downflag("V67FLAG");
        call("INTSTALL");
        double pos = 0, vel = 0, bias = 0;
        for (int x = 0; x < 54; x += 6) {    // NXPOSVEL: AXT,1 54 ; TIX,1
            pos += sq3(x + 54);
            vel += sq3(x + 108);
            bias += sq3(x + 162);
        }
        double wwpos = Math.sqrt(pos), wwvel = Math.sqrt(vel), wwbias = Math.sqrt(bias) / 1024.0;
        if (Double.isInfinite(wwpos)) { wwpos = wwvel = wwbias = 0x3FFFFFFF; }  // BOV: DPPOSMAX
        if (wwpos > FT99999) { wwpos = FT99999; }
        s("WWPOS", (int) wwpos);
        s("WWVEL", (int) wwvel);
        s("WWBIAS", (int) wwbias);
        call("INTWAKE");
    }

    private double sq3(int idx) {
        double a = idx < w.length ? w[idx] : 0;
        return a * a;
    }

    // ================================================================ radar lamps
    /** R12LITES: radar lamp update for R12 (LR); returns to ISWRETRN. */
    public void r12lites() {
        if (!bit("IMODES33", BIT1)) {        // no action during lamp test
            hlight();
        }
        call("ISWRETRN");
    }

    /** RADLITES: update tracker (VLIGHT) or altitude/velocity lamps unless lamp test. */
    public void radlites() {
        if (bit("IMODES33", BIT1)) { return; }
        if ((g("ITEMP1") - BIT5) > 0) {
            vlight(Agc.fromSigned(-1));
            return;
        }
        rrtrkf();
    }

    /** HLIGHT: altitude lamp - on if scale bit abnormal or LR altitude bad. */
    public void hlight() {
        s("ITEMP5", 0);                      // zero ITEMP5 for H index
        s("L", HLITE);
        if (bit("FLGWRD11", SCABBIT)) { onlites(); return; }
        bothlits(LRALTBIT);
    }

    private void bothlits(int mask) {
        if ((g("RADMODES") & mask) != 0) { onlites(); return; }
        if ((g("FLGWRD11") & (g("ITEMP5") == 0 ? HFLSHBIT : VFLSHBIT)) != 0) { rrtrkf(); return; }
        litit();
    }

    private void onlites() {
        clrBits("FLGWRD11", g("ITEMP5") == 0 ? HFLSHBIT : VFLSHBIT);
        litit();
    }

    private void litit() { trkflon(g("L") != 0 ? BIT8 : 0); rrtrkf(); }

    /** VLIGHT: velocity lamp. */
    public void vlight(int index) {
        s("ITEMP5", 1);
        s("L", VLITE);
        bothlits(BIT8);
    }

    // ---------------------------------------------------------------- bit and word definitions
    public static final int BIT1 = 1, BIT2 = 2, BIT3 = 4, BIT4 = 010, BIT5 = 020, BIT6 = 040, BIT7 = 0100, BIT8 = 0200,
        BIT9 = 0400, BIT10 = 01000, BIT11 = 02000, BIT12 = 04000, BIT13 = 010000, BIT14 = 020000, BIT15 = 040000;
    public static final int PRIO6 = 06000, PRIO22 = 022000;
    /** RADMODES bits. */
    public static final int RRRSBIT = BIT3, AUTOMBIT = BIT1, ANTENBIT = BIT12, DESIGBIT = BIT10, REPOSBIT = BIT11,
        REMODBIT = BIT8, RCDU0BIT = BIT13, RCDUFBIT = BIT14, CDESBIT = BIT15, TURNONBT = BIT9, LRPOSBIT = BIT6,
        RRDATABT = BIT4;
    public static final int OLDESBIT = BIT2, READRBIT = BIT9, SCABBIT = BIT7, LRALTBIT = BIT5, HFLSHBIT = BIT3,
        VFLSHBIT = BIT4, HLITE = BIT5, VLITE = BIT3;
}
