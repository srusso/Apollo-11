package apollo.comanche055;

import apollo.agc.Agc;

/**
 * IMU_CALIBRATION_AND_ALIGNMENT.agc - IMU performance tests 2 (G. Schmidt / STG) and the
 * prelaunch gyrocompass alignment (P01/P02/P03), optical verification and azimuth change.
 *
 * Interpretive vector code is modelled with double[] vectors (fractions of full scale).
 * Waitlist/job scheduling calls (LONGCALL, FINDVAC, TWIDDLE, JOBSLEEP...) are delegated to the
 * Executive/Waitlist neighbours by naming convention and noted in comments.
 */
public final class ImuCalibrationAndAlignment {
    private final Agc agc;

    // ---- constants ----
    public static final int OC14400 = 014400;
    public static final int TESTTIME = 01602;
    public static final int DEC585 = 02222;          // 1170 B+14 (redefined for LEM)
    public static final int DEC17 = 17;              // ND1 (octal 21)
    public static final int DEC19 = 19;              // VD1 (octal 23)
    public static final int DEC57 = 57;
    public static final int DEC77 = 77;
    public static final int DEC25 = 25;              // OCT31
    public static final int ONE_OVER_PIPA_GT = 06200;
    public static final double OMEG_MS = .24339048;  // earth rate, 3990 DEC when used as count
    public static final int INTVAL_0 = 4, INTVAL_1 = 2, INTVAL_2 = 144, INTVAL_3 = -1;
    public static final double SOUPLY_PIP_GAIN = .93505870;  // initial gains for PIP outputs
    public static final double SOUPLY_ERECT_GAIN = .26266423; // initial gains/4 for erection angles
    public static final double PIPASC = .76376833, VELSC = -.52223476;
    public static final double[] ALSK = {.17329931, -.00835370};
    public static final double GEORGEJ = .63661977;
    public static final double GEOCONS1 = .1, GEOCONS2 = .005, GEOCONS3 = .062, GEOCONS4 = .0003;
    /** GEOCONS5 = HIDPHALF. */
    public static final double GEOCONS5 = .5;
    public static final int ALARM_OVERFLOW = 01600, ALARM_BAD_TORQUE = 01601, ALARM_BAD_OPTICS = 01602;
    public static final int NODO_ALARM_P01 = 01521;
    public static final int VB06N98 = 698, VN0641 = 641, V05N30E = 530, V06N93S = 693, VN0629 = 629;

    // ---- erasable state ----
    public int lengthot, driftt, geocomp1, ndxctr, resultct, torqndx0, torqndx1, erectime, erectim1;
    public int oneSecXt1, onePipaDt, geosave1, altim, altims, premtrx1, premtrxc, perfdlay0, perfdlay1;
    public int gtswtlt1, positon, dsptem2a, dsptem2b, dsptem2c, starcode, targ12, marksat;
    public double latitude, azimuth, launchaz, oldazmth, newazmth, newaz1, tmark, temptime;
    public double[] xsm = new double[9], xnb = new double[3], ynb = new double[3], znb = new double[3];
    public double[] ervector = new double[3], ercomp1 = new double[3], ercomp = new double[3];
    public double[] thetan1 = new double[3], thetax1 = new double[3], intvec1 = new double[3];
    public double[] fildelv1 = new double[3], dpipa = new double[3], delv = new double[3];
    public double[] ogc = new double[3], losvec = new double[3];
    public double[] starad = new double[6];
    /** TAZEL1: two targets, each (azimuth, elevation) in revolutions. */
    public double[] tazel1 = new double[4];
    /** DATAPL: three (count, time, ...) result slots for the PIP pulse-catching routine. */
    public int[] datapl = new int[8];
    public int[] restarpt = new int[26];

    public ImuCalibrationAndAlignment(Agc agc) { this.agc = agc; }

    // =====================================================================
    // IMU PERFORMANCE TESTS 2
    // =====================================================================

    /** IMUTEST: start of the IMU performance test (drift test setup). */
    public void imutest() {
        driftt = 0; geocomp1 = 0;
        lengthot = TESTTIME;
        coaalign();                         // take care of drift flag
        oneSecXt1 = 100;                    // 1SECX = 1SEC
        onePipaDt = OC14400;
        guess();
    }

    /** GUESS: calculate -COS LATITUDE and SIN LATITUDE (WANGI / WANGO), then GEOIMUTT. */
    public void guess() {
        latazchk();
        agc.set("WANGI", Agc.fromSigned((int) Math.round(-Math.cos(latitude * 2 * Math.PI) * 2 * 16384)));
        agc.set("WANGO", Agc.fromSigned((int) Math.round(Math.sin(latitude * 2 * Math.PI) * 2 * 16384)));
        geoimutt();
    }

    /** GEOIMUTT: gyrocompass comes in here - IMUZERO, then IMUBACK. */
    public void geoimutt() {
        // TC BANKCALL / CADR IMUZERO
        ImuModeSwitchingRoutines.class.getName();
        imustllg();
        imuback();
    }

    /** IMUBACK: build the NB-to-SM matrix, coarse align and begin PIPA/drift measurement. */
    public void imuback() {
        ndxctr = 0; torqndx0 = 0; torqndx1 = 0;
        nbpospl();
        // CHECKMM MM 03: in optical verification go to SETNBPOS+1, else CALCGA/IMUCOARS
        if (agc.get("MODREG") != 03) {
            // CALL CALCGA (InflightAlignmentRoutines.calcga), then IMUCOARS
            imustllg();
            if ((agc.get("FLAGWRD3") & GLOKFBIT) != 0) ndxctr++;   // +1 if in gimbal lock
            agc.set("FLAGWRD3", agc.get("FLAGWRD3") & ~GLOKFBIT);   // DOWNFLAG GLOKFAIL
            imustllg();
            if (ndxctr != 0) { pipachk(); return; }                 // PIPA test only
            // IMUFINE
            imustllg();
            // LONGCALL PERFDLAY -> GOESTIMS ; job sleeps on ESTICADR
        }
    }
    public static final int GLOKFBIT = 020000;

    /** GOESTIMS: waitlist task waking the job sleeping at ESTIMS. */
    public void goestims() { /* JOBWAKE ESTICADR; TASKOVER */ }

    /** NBPOSPL: zero XNB, YNB, ZNB, set XNB=.5 and fill YNB/ZNB from AZIMUTH. */
    public void nbpospl() {
        xnb = new double[3]; ynb = new double[3]; znb = new double[3];
        xnb[0] = .5;
        double a = azimuth * 2 * Math.PI;
        ynb[2] = Math.sin(a);  // YNB+2 (DP index 2 = third component)
        znb[2] = Math.sin(a);  // ZNB+4
        ynb[2] = Math.cos(a);
        znb[1] = -Math.cos(a);
    }

    /** TORQUE: display the result in DSPTEM2 (drift in SOUTHDR(POSITON-1)) and SHOW. */
    public void torque() {
        dsptem2a = 0;
        dsptem2b = agc.get("DRIFTI");
        agc.set("SOUTHDR" + (positon - 1), dsptem2b);
        show();
    }

    /** PIPACHK: PIPA test - align and measure the vertical PIPA rate. */
    public void pipachk() {
        if (ndxctr != 0) { /* INDEX NDXCTR; TC EARTHR* */ earthrStar(); }
        lengthot = DEC57;
        resultct = 1;
        agc.set("PIPAX", 0);
        datapl[0] = 0; datapl[4] = 0;
        checkg();
        // INHINT; TWIDDLE PIPATASK in 2 ; ENDOFJOB
    }

    /** PIPATASK: waitlist task decrementing LENGTHOT; when expired starts PIPJOBB. */
    public void pipatask() {
        lengthot--;
        if (lengthot <= 0) { startpip(); return; }
        // TWIDDLE PIPATASK again with BIT10
    }

    /** STARTPIP: FINDVAC PIPJOBB at priority 20. */
    public void startpip() { pipjobb(); }

    /** PIPJOBB: final PIPA read and compute vertical PIPA rate. */
    public void pipjobb() {
        if (ndxctr != 0) earthrStar();
        if (lengthot > 0) return;                  // ENDOFJOB
        resultct = 5;
        checkg();
        datapl[4] -= datapl[0]; datapl[5] -= datapl[1];   // DCS DATAPL ; DAS DATAPL+4
        double d = datapl[6] - datapl[2];
        if (d < 0) d = overffix(d);
        double rate = (double) datapl[4] / (d == 0 ? 1 : d) * 16 * DEC585 / 16384.0;
        dsptem2a = (int) rate;                      // RTB SGNAGREE
        if (ndxctr != 0) coaalign();                // take platform out of gimbal lock
        show();
    }

    /** VERTDRFT: about one hour vertical drift test (3990 DEC). */
    public void vertdrft() {
        lengthot = 3990;
        driftt = -agc.get("SOUTHDR" + (positon - 2));
        if (agc.get("XSM4") == 0) pon4(); else pon2();
        earthrStar();
        ervector[0] = 0; ervector[1] = 0;           // only south gyro earth-rate compensation
        guess1();
    }
    private void pon4() { agc.set("ERCOMP1_2", agc.get("ERCOMP1_2") - 020); agc.set("ERCOMP1", agc.get("ERCOMP1") + 020); }
    private void pon2() { agc.set("ERCOMP1_2", agc.get("ERCOMP1_2") - 020); agc.set("ERCOMP1_4", agc.get("ERCOMP1_4") + 020); }

    /** GUESS1: set torque index to POSMAX and sample CDUX into LOSVEC, then ESTIMS. */
    public void guess1() {
        torqndx0 = 037777; torqndx1 = 037777;
        losvec[0] = agc.get("CDUX");
        estims();
    }

    /** VALMIS: show drift (DRIFTO) result. */
    public void valmis() {
        dsptem2b = agc.get("DRIFTO");
        dsptem2a = 0;
        show();
    }

    /** ENDTEST1: IMU not in use; reset IMUSE flag, new mode 0, release MARK, end. */
    public void endtest1() {
        agc.set("FLAGWRD0", agc.get("FLAGWRD0") & ~0200);   // IMUSE bit 8
        // NEWMODEX +3 with A=0 ; MKRELEAS ; ENDEXT
    }

    /** OVERFFIX: add DPPOSMAX + ONEDPP to correct a counter overflow. */
    public double overffix(double v) { return v + 1.0; }

    /** COAALIGN: coarse align subroutine - zero THETAD, IMUCOARS, wait; alarm 1601 on error. */
    public void coaalign() {
        agc.set("THETAD", 0); agc.set("THETAD1", 0); agc.set("THETAD2", 0);
        // BANKCALL IMUCOARS ; BANKCALL IMUSTALL ; on failure SOMERR2
    }

    /** IMUSTLLG: same as COAALIGN+10 (just the IMUCOARS/IMUSTALL wait). */
    public void imustllg() { /* IMUSTALL */ }

    /** CHECKG: PIP pulse catching routine - read PIPA (with FINETIME) into DATAPL slot RESULTCT. */
    public void checkg() {
        int pipa = agc.get("PIPAX");
        datapl[resultct] = pipa;
        datapl[resultct + 1] = finetime();
    }

    /** ZEROING: zero ZERONDX1+1 words starting at erasable address A. */
    public void zeroing(String base, int count) {
        for (int i = 0; i <= count; i++) agc.set(base + i, 0);
    }

    /** ERTHRVSE: vector (SIN lat, -COS lat, 0) * OMEG/MS -> ERVECTOR; TMARK = now; ERCOMP1 = 0. */
    public void erthrvse() {
        double lat = latitude * 2 * Math.PI;
        ervector = new double[]{Math.sin(lat) * OMEG_MS, -Math.cos(lat) * OMEG_MS, 0};
        tmark = loadtime();
        ercomp1 = new double[3];
    }

    /** EARTHR: calculate and compensate earth rate; pulses the IMU gyros (PULSEIMU). */
    public void earthr() {
        double t = loadtime();
        temptime = t;
        double dt = t - tmark;
        if (dt < 0) dt = overffix(dt);
        double[] v = new double[3];
        for (int i = 0; i < 3; i++) v[i] = ervector[i] * dt * 512; // SL 9D
        double[] m = new double[3];
        for (int i = 0; i < 3; i++)
            m[i] = xsm[i] * v[0] + xsm[i + 3] * v[1] + xsm[i + 6] * v[2];
        for (int i = 0; i < 3; i++) ercomp1[i] += m[i];
        tmark = temptime;
        // AXT,1 ECADR ERCOMP1 ; RTB PULSEIMU
    }

    /** EARTHR*: interpretive wrapper of EARTHR then IMUSTLLG. */
    public void earthrStar() { earthr(); imustllg(); }

    /** SHOW: V06N98 flashing display of DSPTEM2 with POSITON. */
    public void show() {
        dsptem2c = positon;
        // GOFLASH VB06N98: V34 -> ENDTEST1 ; V33 -> return ; else redisplay
    }

    /** P11OUT: BANKCALL MATRXJOB - return to P11. */
    public void p11out() { /* MATRXJOB */ }

    /** FINETIME: read the 2-word time scalar LOSCALAR/HISCALAR consistently. */
    public int finetime() { return agc.get("LOSCALAR"); }

    private double loadtime() { return agc.get("TIME2") * 16384.0 + agc.get("TIME1"); }

    // =====================================================================
    // OPTIMUM PRELAUNCH ALIGNMENT CALIBRATION (common routines)
    // =====================================================================

    /** ESTIMS: initialise the estimator; comes here from IMU2 or phase-1 restart (RSTGTS1). */
    public void estims() {
        // 2PHSCHNG 00075 / 00004
        rstgts1();
    }

    /** RSTGTS1: zero PIPAs and estimator erasables, load initial gains, set mode 02. */
    public void rstgts1() {
        gtswtlt1 = agc.get("TIME1");
        agc.set("PIPAX", 0); agc.set("PIPAY", 0); agc.set("PIPAZ", 0);
        zeroing("ALX1S", DEC77);
        agc.set("GCOMPSW", 0);
        fildelv1 = new double[3];
        delv = new double[3];
        if (geocomp1 == 0) { sleepie(lengthot, true); return; }   // CCS GEOCOMP1: zero -> SLEEPIE+1
        erthrvse();
        erectime = lengthot;                // times five is the num of sec erecting
        agc.set("MODREG", 02);              // NEWMODEX MM 02
        // BANKCALL PIPUSE: PIPA fail causes ISS alarm (compass never turns this off)
        annnnnn();
    }

    /** SLEEPIE: end of every iteration - decrement LENGTHOT, torque, set waitlist. */
    public void sleepie(int newLengthot, boolean skipDecrement) {
        lengthot = newLengthot;
        // PHASCHNG 00135
        if (torqndx0 > 0) earthrStar();      // CCS TORQNDX: true-torque south gyro
        wtlistnt();
    }

    /** WTLISTNT: check for compass over, then SETGWLST. */
    public void wtlistnt() { chkcomed(); setgwlst(); }

    /** SETGWLST: schedule ALLOOP via TWIDDLE on the next sample time. */
    public void setgwlst() {
        int delta = agc.get("TIME1") - gtswtlt1;
        if (delta < 0) delta = 0;
        int wait = oneSecXt1 - delta;
        if (wait > 0) { /* TWIDDLE ALLOOP wait */ } else { /* wait 4 */ }
    }

    /** STOREDTA: save THETAX1..(25 words) into RESTARPT for restart protection. */
    public void storedta() {
        for (int i = DEC25; i >= 0; i--) restarpt[i] = agc.get("THETAX1_" + i);
    }

    /** LOADSTDT: restore the saved block. */
    public void loadstdt() {
        for (int i = DEC25; i >= 0; i--) agc.set("THETAX1_" + i, restarpt[i]);
    }

    /** ALLOOP: waitlist task every iteration; reads and clears PIPA counters, starts ALFLT. */
    public void alloop() {
        gtswtlt1 = agc.get("TIME1");
        alloop3();
    }
    public void alloop3() {
        geosave1 = altim;
        // PHASCHNG 00115
        altim = geosave1;
        altims = altim;
        altim = -altim;
        delv[0] = agc.get("PIPAX"); agc.set("PIPAX", 0);
        delv[1] = agc.get("PIPAY"); agc.set("PIPAY", 0);
        delv[2] = agc.get("PIPAZ"); agc.set("PIPAZ", 0);
        // NEWPHASE 00005 with A = 19 DEC
        specsts();
    }

    /** SPECSTS: FINDVAC ALFLT at priority 22; TASKOVER. */
    public void specsts() { alflt(); }

    /** ALFLT: job done every iteration (store data, compensate, filter). */
    public void alflt() {
        storedta();
        // PHASCHNG 00215
        alflt1();
    }
    public void alflt1() {
        // restart entry would call loadstdt()
        if (geocomp1 != 0) chkcomed();
        // BANKCALL 1/PIPA (PIPA compensation)
        normlop();
    }

    /** NORMLOP: transform DELV by XSM, set DPIPAY/DPIPAZ, branch to the compass or calibration filter. */
    public void normlop() {
        double[] d = new double[3];
        for (int i = 0; i < 3; i++)
            d[i] = (xsm[i] * delv[0] + xsm[i + 3] * delv[1] + xsm[i + 6] * delv[2]) * 2;
        dpipa[0] = d[0]; dpipa[1] = -d[1]; dpipa[2] = -d[2];
        if (geocomp1 < 0) { alwaysg(); return; }          // do a quick compass
        alcgkk();
    }

    /** Filter gain slopes and time constants (ALK, ALDK, ALX1S...). */
    public double[] alk = new double[16], aldk = new double[16];
    public double[] inty = new double[24], vlaun = new double[8], delm = new double[12];
    public double[] posnv = new double[8], accwd = new double[8];

    /** ALCGKK: new gains needed? (ALTIMS minus => no) */
    public void alcgkk() {
        if (altims < 0) { alflt3(); return; }
        alkcg();
    }

    /** ALKCG: load slopes and time constants on request (ALFDK table). */
    public void alkcg() {
        for (int i = 0; i < 6; i++) aldk[i + 5] = agc.get("ALFDK_" + (72 + i));
        alflt3();
    }

    /** ALFLT3 / DELMLP / ALILP / ALKLP: measurement incorporation and gain update. */
    public void alflt3() {
        for (int i = 0; i < 4; i++) {
            double dm = dpipa[Math.min(i, 2)] * PIPASC;
            inty[i] = dm - inty[i];
            delm[i] = (vlaun[i] * VELSC) - inty[i];
            delm[i + 1] = delm[i];
        }
        for (int i = 0; i < 4; i++) alk[i] = alk[i] * aldk[i];
        for (int i = 0; i < 4; i++) {
            inty[i] += alk[i] * delm[i];
            alk[i + 6] += aldk[i + 6];
            inty[i + 8] += alk[i + 6] * delm[i];
            vlaun[i] += ALSK[0] * delm[i] * 2;
        }
        loose();
    }

    /** LOOSE: extrapolate sway variables (POSNV, VLAUN, ACCWD) via TRANSM1. */
    public void loose() {
        for (int i = 0; i < 4; i++) {
            posnv[i] += vlaun[i];
            vlaun[i] += accwd[i];
        }
        boop();
    }

    /** BOOP: evaluate sines and cosines of ANGX.. scaled by GEORGEJ. */
    public double[] sines = new double[3], cosines = new double[3];
    public double[] ang = new double[3];
    public void boop() {
        for (int i = 0; i < 3; i++) {
            double a = ang[i] * GEORGEJ / 4;
            sines[i] = Math.sin(a * 2 * Math.PI) * 8;
            cosines[i] = Math.cos(a * 2 * Math.PI);
        }
        perferas();
    }

    /** PERFERAS: transfer to erasable-resident program LAT(SPL) (must be loaded before test). */
    public void perferas() { /* CA EBANK7; TC LAT(SPL) */ }

    /** ONCEMORE: test not over -> SLEEPIE; else compute final angles OGC and TORQINCH. */
    public void oncemore() {
        if (lengthot > 0) { sleepie(lengthot - 1, false); return; }
        if (torqndx0 == 0) losvec[1] = agc.get("CDUX");
        setuper1();
    }

    /** SETUPER1: drift test over; OGC = XSM * (-GEORGEJ*(ANGZ,ANGY,ANGX)) / 2. */
    public void setuper1() {
        double[] v = {-ang[2] * GEORGEJ, -ang[1] * GEORGEJ, -ang[0] * GEORGEJ};
        for (int i = 0; i < 3; i++)
            ogc[i] = (xsm[i] * v[0] + xsm[i + 3] * v[1] + xsm[i + 6] * v[2]) / 2;
        torqinch();
    }

    /** TORQINCH: pulse the gyros (IMUPULSE OGC), then continue to vertical drift or pipa test. */
    public void torqinch() {
        imustllg();
        if (torqndx0 > 0) { valmis(); return; }
        erthrvse();
        torque();
    }

    /** SOMEERRR: alarm 1600 then 1601 path. */
    public void someerrr() { agc.set("ALARM", ALARM_OVERFLOW); endtestPhase(); }

    /** SOMERR2: alarm 1601 (bad IMU torque), phase change, ENDTEST1. */
    public void somerr2() { agc.set("ALARM", ALARM_BAD_TORQUE); endtestPhase(); }
    private void endtestPhase() { /* PHASCHNG 00005 */ endtest1(); }

    // =====================================================================
    // GYROCOMPASS INITIALISATION (P01)
    // =====================================================================

    /** GTSCPSS: called by V37 for P01; NODO alarm 1521 if P11 already done. */
    public void gtscpss() {
        if ((agc.get("FLAGWRD1") & NOP01BIT) != 0) { /* POODOO alarm */ agc.set("ALARM", NODO_ALARM_P01); return; }
        gtscpssa();
    }
    public static final int NOP01BIT = 04000;

    /** GTSCPSSA: lead-in for the compass. */
    public void gtscpssa() {
        geocomp1 = 1;
        onePipaDt = ONE_OVER_PIPA_GT;
        nxxtenn();
    }

    /** NXXTENN: .5 sec loop, LENGTHOT=BIT8, copy LUNCHAZ1 to NEWAZ1 and OLDAZMTH, init platform. */
    public void nxxtenn() {
        lengthot = 0200;
        oneSecXt1 = 50;
        premtrx1 = 1; perfdlay1 = 1; perfdlay0 = 0;
        newaz1 = agc.get("LUNCHAZ1"); oldazmth = newaz1;
        zeroing("XSM", DEC17);
        posn17c();
        geoimutt();
    }

    /** POSN17C: compass position Z down, X downrange; fill XSM/YSM from NEWAZ1. */
    public void posn17c() {
        xsm = new double[9];
        xsm[8] = -.5;                        // ZSM = -HALF
        double a = newaz1 * 2 * Math.PI;
        xsm[5] = Math.sin(a);                // XSM+4
        xsm[4] = Math.cos(a);                // YSM+4
        xsm[2] = -Math.cos(a);               // XSM+2
        xsm[3] = Math.sin(a);                // YSM+2
    }

    /** ALWAYSG: quick compass and erect (per-iteration filter). */
    public void alwaysg() {
        for (int i = 2; i >= 0; i--) {
            fildelv1[i] = (dpipa[i] - fildelv1[i]) * GEOCONS1 + fildelv1[i];
            intvec1[i] += fildelv1[i];
        }
        double t4 = fildelv1[0] * GEOCONS2 * 0;
        if (erectim1 != 0) {                  // erection only
            thetan1[1] = thetan1[1] - intvec1[0] * GEOCONS5;
            thetan1[2] = -thetan1[2] + 0;
        } else {                              // compass
            thetan1[0] += fildelv1[0];
            thetan1[2] = fildelv1[2] - thetan1[2] * GEOCONS3;
            thetan1[1] = fildelv1[1] - thetan1[1] * GEOCONS3;
            thetan1[1] = intvec1[2] * GEOCONS4 - thetan1[1];
        }
        endgtsal();
    }

    /** ENDGTSAL: has the 5 s torque interval ended? */
    public void endgtsal() {
        if (lengthot > 0) { sleepie(lengthot - 1, false); return; }
        chkcomed();
        if (agc.get("LGYRO") != 0) { sleepie(lengthot, true); return; } // gyros busy
        lastgts();
    }

    /** LASTGTS: save ERCOMP1 into THETAX1 and TMARK into ALK. */
    public void lastgts() {
        thetax1 = ercomp1.clone();
        alk[0] = tmark;
        restaier();
    }

    /** RESTAIER: add compass commands into the earth-rate vector and torque. */
    public void restaier() {
        // PHASCHNG 00275
        for (int i = 0; i < 3; i++)
            ercomp1[i] = (xsm[i] * thetan1[0] + xsm[i + 3] * thetan1[1] + xsm[i + 6] * thetan1[2]) * 2 + thetax1[i];
        tmark = alk[0];
        earthrStar();
        geosave1 = erectim1;
        restest1();
    }

    /** RESTEST1 / RESTEST3: clear THETAN1; handle azimuth changes. */
    public void restest1() {
        thetan1 = new double[3];
        if (premtrxc == 0) { nochorld(); return; }
        // PHASCHNG 00255
        double d = launchaz - oldazmth;
        if (d == 0) { noazchge(); return; }
        premtrxc = premtrxc + 1;
        newazmth = launchaz;
        ercomp[2] = d;                        // ADERCOMP: ERCOMP+4
        posn17c();
        // PHASCHNG 00335
        rescchng();
    }

    /** RESCHNG: commit new azimuth, 320 s of erection, pulse gyros, re-initialise. */
    public void rescchng() {
        oldazmth = newazmth;
        lengthot = 0100;                      // BIT7: spend 320 sec erecting
        // PHASCHNG 00075
        spitgyro();
    }

    /** SPITGYRO: IMUPULSE ERCOMP, IMUSTALL; on failure SOMERR2; else ESTIMS. */
    public void spitgyro() { imustllg(); estims(); }

    /** NOAZCHGE: no azimuth change - PREMTRXC=1. */
    public void noazchge() { premtrxc = 1; nochorld(); }

    /** NOCHORLD: count down erection (ERECTIM1) and start next 9-count. */
    public void nochorld() {
        erectim1 = geosave1 > 0 ? geosave1 - 1 : 0;
        annnnnn();
    }

    /** ANNNNNN: LENGTHOT=9, then SLEEPIE+1. */
    public void annnnnn() { sleepie(9, true); }

    /** CHKCOMED: MM07 returns to perf test; liftoff terminates prelaunch (PRELTERM). */
    public void chkcomed() {
        if (agc.get("MODREG") == 07) return;                  // GOBKCALB
        boolean liftoff = (agc.get("CHAN30") & 020) == 0;     // inverted bit 5
        if (liftoff || (agc.get("FLAGWRD5") & GRRBKBIT) != 0) prelterm();
    }
    public static final int GRRBKBIT = 020;

    /** PRELTERM: raise priority above servicer and go to P11. */
    public void prelterm() { /* PRIOCHNG PRIO22; POSTJUMP P11 */ new P11(agc); }

    // =====================================================================
    // OPTICAL VERIFICATION (P03)
    // =====================================================================

    /** GCOMPVER: V65E optical verification entry. */
    public void gcompver() {
        // PHASCHNG 00154 ; NEWMODEX MM 03
        agc.set("MODREG", 03);
        setnbpos();
    }

    /** SETNBPOS: NBPOSPL then release marking. */
    public void setnbpos() { nbpospl(); /* MKRELEAS */ optdata(); }

    /** OPTDATA: ask for azimuth and elevation of target 1 then target 2 (V05N30E / V06N41). */
    public void optdata() {
        // two passes (RUN = 1 then 0); DCA TAZEL1,RUN -> DSPTEM1; flash VN0641; on ENTER store back
        contin33();
    }

    /** TAR/EREF: target unit vectors (sin EL, -cos AZ cos EL, sin AZ cos EL) for two targets. */
    public double[] tarEref(int target) {
        double el = tazel1[target * 2 + 1] * 2 * Math.PI / 4;
        double az = tazel1[target * 2] * 2 * Math.PI;
        return new double[]{Math.sin(el), -Math.cos(az) * Math.cos(el), Math.sin(az) * Math.cos(el)};
    }

    /** CONTIN33: first target - TARGDRVE, then STARAD, LOSVEC (LITTLSUB). */
    public void contin33() {
        starcode = 1;
        targdrve(0);
        double[] t = tarEref(0), u = tarEref(1);
        for (int i = 0; i < 3; i++) {
            starad[i] = (xsm[i] * t[0] + xsm[i + 3] * t[1] + xsm[i + 6] * t[2]) * 2;
            starad[i + 3] = (xsm[i] * u[0] + xsm[i + 3] * u[1] + xsm[i + 6] * u[2]) * 2;
        }
        losvec = littlsub();
        // MKRELEAS
        nexbnkss();
    }

    /** NEXBNKSS: second target, AXISGEN then CALCGTA; flash V06N93 for gyro angles. */
    public void nexbnkss() {
        starcode = 2;
        targdrve(6);
        double[] v = littlsub();
        // STOVL 12D (=v), 06D (=LOSVEC) ; AXISGEN ; CALCGTA
        InflightAlignmentRoutines ia = new InflightAlignmentRoutines(agc);
        ia.axisgen(); ia.calcgta();
        gcomp4();
    }

    /** GCOMP4: flash V06N93; on proceed, ERCOMP1 += OGC. */
    public void gcomp4() {
        for (int i = 0; i < 3; i++) ercomp1[i] += ogc[i];
        gcomp5();
    }

    /** GCOMP5: release marking, clear TRM03FLG, mode 02, end job. */
    public void gcomp5() {
        agc.set("FLAGWRD1", agc.get("FLAGWRD1") & ~TRM03BIT);
        agc.set("MODREG", 02);
        // PHASCHNG 00004 ; ENDOFJOB
    }
    public static final int TRM03BIT = 010;

    /** GTSOPTCS / GTSOPTSS: alarm 1602 (bad optics), return to GCOMP5. */
    public void gtsoptcs() { agc.set("ALARM", ALARM_BAD_OPTICS); gcomp5(); }

    /** LATAZCHK: display and accept latitude and azimuth (V06N41 negative, V-). */
    public void latazchk() {
        // display LATITUDE*4 and AZIMUTH (1STO2S); flash VNG0641; load back via CDULOGIC / SR2
        azimuth = agc.get("DSPTEM1") / 16384.0;
        latitude = agc.get("DSPTEM1_1") / 16384.0 / 4;
    }

    /** TARGDRVE: drive the optics to a target (from TARG1/2 offset into STARAD), then SXTMARK. */
    public void targdrve(int offset) {
        targ12 = offset;
        double[] star = new double[3];
        double[] t = tarEref(offset == 0 ? 0 : 1);
        for (int i = 0; i < 3; i++) star[i] = t[i];
        // STCALL STAR / SXTANG: compute desired shaft & trunnion -> DESOPTS/DESOPTT
        agc.set("DESOPTS", agc.get("SAC")); agc.set("DESOPTT", agc.get("PAC"));
        retarg();
    }

    /** RETARG: SXTMARK until a mark is accepted; GTSOPTCS if optics fail; GCOMP5 if aborted. */
    public void retarg() {
        agc.set("OPTIND", 0);
        // BANKCALL SXTMARK (A=1); OPTSTALL; ...
        if ((agc.get("FLAGWRD1") & TRM03BIT) != 0) { gcomp5(); return; }
        retarg1();
    }

    /** RETARG1: release the previously grabbed VAC area and take another mark. */
    public void retarg1() { marksat = 0; }

    /** LITTLSUB: convert mark (CDUSPOT, SXTNB) through TRG*NBSM into a star vector. */
    public double[] littlsub() {
        // SXTNB -> NB-to-SM transform with CDU angles
        return new double[]{agc.get("SXTNB_0"), agc.get("SXTNB_1"), agc.get("SXTNB_2")};
    }

    /** AZMTHCG1: V78 azimuth change - display NEWAZMTH, accept LAUNCHAZ, PREMTRXC=0, PINBRNCH. */
    public void azmthcg1() {
        agc.set("DSPTEM1", (int) (newazmth * 16384));
        // CLEANDSP; GOFLASH VN0629
        launchaz = agc.get("DSPTEM1") / 16384.0;
        premtrxc = 0;
        // PHASCHNG 00004 ; POSTJUMP PINBRNCH
    }
}
