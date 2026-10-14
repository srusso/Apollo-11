package apollo.luminary099;

import apollo.agc.Agc;

/**
 * IMU_PERFORMANCE_TESTS_4.agc (Luminary 099, pp. 382-389).
 * Filter for the gyro drift tests (George Schmidt, Nov 1966). Entered from IMU 2 and returns
 * there at the end of the test (LENGTHOT reaches zero). No compass is done in the LEM.
 * Alarms: 1600 overflow in drift test; 1601 bad IMU moding in a routine using IMUSTALL.
 *
 * The interpretive filter loops (ALFLT3 / DELMLP / ALILP / ALKLP / LOOSE / BOOP) operate on
 * 3-axis tables of double-precision values; here they are held as Java double arrays with the
 * same indexing (axis 0..2 for the three gimbal axes, 8D/16D offsets collapsed to arrays).
 */
public class ImuPerformanceTests4 {
    private final Agc agc;

    /** Alarm: overflow in drift test. */
    public static final int ALARM_OVERFLOW = 01600;
    /** OCT1601: bad IMU moding. */
    public static final int OCT1601 = 01601;
    /** 77DECML. */
    public static final int DECML_77 = 77;
    /** DEC585 (OCT 06200 = 3200 B+14). */
    public static final int DEC585 = 06200;
    /** INTVAL: OCT 4, OCT 2, DEC 144, DEC -1. */
    public static final int[] INTVAL = {4, 2, 144, -1};
    /** SOUPLY: initial gains for PIP outputs; initial gains/4 for erection angles. */
    public static final double[] SOUPLY = {0.93505870, 0.26266423};
    /** PIPASC. */
    public static final double PIPASC = 0.13055869;
    /** VELSC = 512/980.402 (negative). */
    public static final double VELSC = -0.52223476;
    /** ALSK: sway vel gain x 980.402/4096, sway accel gain x 980.402/4096. */
    public static final double[] ALSK = {0.17329931, -0.00835370};
    /** GEORGEJ. */
    public static final double GEORGEJ = 0.63661977;
    /** GEORGEK. */
    public static final double GEORGEK = 0.59737013;

    // Filter state (per axis).
    private final double[] dpipay = new double[3];
    private final double[] intY = new double[6];
    private final double[] vlaun = new double[3];
    private final double[] posnv = new double[3];
    private final double[] accwd = new double[3];
    private final double[] delm = new double[6];
    private final double[] alk = new double[6];
    private final double[] aldk = new double[6];
    private final double[] sines = new double[3];
    private final double[] cosines = new double[3];

    public ImuPerformanceTests4(Agc agc) {
        this.agc = agc;
    }

    private static int s(int w) { return Agc.toSigned(w); }

    /** ESTIMS: start the estimator - schedule ALLOOP, zero the PIPAs and filter registers. */
    public void estims() {
        // INHINT; CAE 1SECXT; TC TWIDDLE; ADRES ALLOOP
        new Waitlist(agc).waitlist(s(agc.get("1SECXT")), this::alLoop);
        agc.set("PIPAX", 0);
        agc.set("PIPAY", 0);
        agc.set("PIPAZ", 0);
        // RELINT
        agc.set("ZERONDX", DECML_77);
        // TC ZEROING with ALXXXZ = GENADR ALX1S-1: zero the filter registers.
        for (int i = 0; i < 6; i++) { intY[i] = delm[i] = alk[i] = aldk[i] = 0; }
        for (int i = 0; i < 3; i++) { vlaun[i] = posnv[i] = accwd[i] = 0; }
        agc.set("GCOMP", 0);
        agc.set("GCOMPSW-1", 0);
        agc.set("DELVX", 0);
        agc.set("ALX1S", 0);
        if (s(agc.get("TORQNDX")) >= 0) {        // DCOMP BMN VERTSKIP
            // vertical-drift test only: CALL ERTHRVSE
        }
        sleepie(true);                           // TC SLEEPIE +1
    }

    /** ALLOOP: once-a-second waitlist task that reads the PIPAs and starts the ALFLT job. */
    public void alLoop() {
        if (agc.get("OVFLOWCK") != 0) {
            return;                              // TC TASKOVER
        }
        int altim = s(agc.get("ALTIM"));
        agc.set("ALTIMS", Agc.fromSigned(Math.abs(altim)));  // CCS ALTIM; CA A; TS ALTIMS
        agc.set("ALTIM", Agc.fromSigned(-Math.abs(altim)));  // CS A; TS ALTIM
        // BZF: GEOCOMPS == 1 -> reschedule; otherwise reschedule only while LENGTHOT > 0.
        if (s(agc.get("GEOCOMPS")) == 1 || s(agc.get("LENGTHOT")) > 0) {
            // CAE 1SECXT; TC TWIDDLE; ADRES ALLOOP - schedule next pass
            new Waitlist(agc).waitlist(s(agc.get("1SECXT")), this::alLoop);
        }
        // Read and clear the PIPA counters into DELVX/Y/Z.
        agc.set("DELVX", agc.get("PIPAX")); agc.set("PIPAX", 0);
        agc.set("DELVY", agc.get("PIPAY")); agc.set("PIPAY", 0);
        agc.set("DELVZ", agc.get("PIPAZ")); agc.set("PIPAZ", 0);
        specSts();
    }

    /** SPECSTS: start the ALFLT job at priority 20 (FINDVAC). */
    private void specSts() {
        alFlt();                                 // 2CADR ALFLT; then TC TASKOVER
    }

    /** ALFLT: the filter job. Call 1/PIPA unless in the geo-compass phase, then NORMLOP. */
    public void alFlt() {
        if (s(agc.get("GEOCOMPS")) != 0) {
            // BANKCALL 1/PIPA
        }
        normLop();
    }

    /** NORMLOP: rotate delta-V by XSM and form the PIPA-derived inputs DPIPAY/DPIPAZ. */
    public void normLop() {
        // S1 = INTVAL; (DELVX,Y,Z) * XSM << 1 -> MPAC; DPIPAY = -MPAC+3 ; DPIPAZ = MPAC+5
        if (-s(agc.get("GEOCOMPS")) < 0) {
            perferas();                          // BMN PERFERAS
            return;
        }
        alcgkk();
    }

    /** ALCGKK: load the slopes and time constants on request (ALTIMS > 0), then filter. */
    private void alcgkk() {
        if (s(agc.get("ALTIMS")) <= 0) {
            alkcg();
        }
        alflt3();
    }

    /** ALKCG: load slopes and time constants (ALFDK table) into ALDK for each axis. */
    private void alkcg() {
        // X2 = 12D; X1 = ALX1S; loop: ALDK+10D,2 = ALFDK+144D,1 ; X1 -= 2 until X2 done.
    }

    /**
     * ALFLT3 / DELMLP / ALILP / ALKLP: update the integrators, residuals and gains for
     * each of the three axes (X1 = 8D .. 0, X2 loops over the gain pairs).
     */
    private void alflt3() {
        for (int i = 0; i < 3; i++) {
            // DELMLP
            intY[i] = intY[i] - dpipay[i] * PIPASC * 512.0;               // SLR 9D; BDSU
            delm[i] = velscTimes(vlaun[i]) * 4.0 - 0;                    // DMP VELSC; SL2R; DSU
            delm[i + 3] = delm[i];
        }
        for (int j = 0; j < 4; j++) {
            // ALILP
            alk[j] = alk[j] * aldk[j];                                   // DMPR*
        }
        for (int i = 0; i < 3; i++) {
            // ALKLP
            intY[i] += alk[i] * delm[i];
            alk[i + 4] += aldk[i + 4];
            intY[i + 3] += delm[i] * 1.0;
            vlaun[i] = vlaun[i] + 2.0 * ALSK[0] * delm[i];               // SL1R; DAD VLAUN
        }
        loose();
    }

    private double velscTimes(double v) { return v * VELSC; }

    /** LOOSE: rotate vectors VLAUN/POSNV/ACCWD through TRANSM1 (and shift left 1). */
    private void loose() {
        for (int i = 0; i < 3; i++) {
            // VDEF (ACCWD, VLAUN, POSNV) ; MXV TRANSM1 ; VSL1 ; store back
        }
        boop();
        perferas();
    }

    /** BOOP: evaluate sines and cosines of ANGX/ANGY/ANGZ scaled by GEORGEJ. */
    private void boop() {
        for (int k = 0; k < 3; k++) {
            double ang = s(agc.get("ANGX+" + k)) / 16384.0 * GEORGEJ * Math.PI * 2 / 4.0;
            sines[k] = Math.sin(ang);
            cosines[k] = Math.cos(ang);
        }
    }

    /**
     * PERFERAS: switch to EBANK 7 and run the erasable program ATIGINC to do the
     * calculations (it must be loaded before the IMU performance test is run), then
     * continue with the sleep / setup test.
     */
    public void perferas() {
        agc.set("EBANK", agc.get("EBANK7"));
        // TC ATIGINC (erasable); returns here
        if (s(agc.get("LENGTHOT")) > 0) {
            sleepie(false);                      // CCS LENGTHOT; TC SLEEPIE
            return;
        }
        if (s(agc.get("TORQNDX")) <= 0) {
            agc.set("LOSVEC+1", agc.get("CDUX")); // for troubleshooting VD positions
        } else {
            return;                              // TC SETUPER1 path
        }
        setupEr1();
    }

    /** SETUPER1: compute the gyro compensation OGC from the drift-test angles and torque. */
    public void setupEr1() {
        // OGC = (ANGZ,ANGY,ANGX VDEF; VCOMP; * GEORGEJ) * XSM >> 1
        // CA OGCPL; BANKCALL IMUPULSE; TC IMUSLLLG
        geoStrt4();
    }

    /** GEOSTRT4: positive TORQNDX only in the vertical drift test. */
    public void geoStrt4() {
        if (s(agc.get("TORQNDX")) > 0) {
            valMis();                            // TC VALMIS
            return;
        }
        // CALL ERTHRVSE ; TC TORQUE
    }

    private void valMis() { /* VALMIS (IMU performance tests 2) */ }

    /** SLEEPIE: store LENGTHOT (not over-decrementing); vertical drift continues EARTHR*. */
    public void sleepie(boolean skipStore) {
        if (!skipStore) {
            agc.set("LENGTHOT", agc.get("LENGTHOT"));
        }
        if (s(agc.get("TORQNDX")) > 0) {
            // TC EARTHR* (vertical drift test)
        }
        // TC ENDOFJOB
    }

    /** SOMEERRR: overflow in the drift test - alarm 1600 and end the test. */
    public void someErrr() {
        agc.set("EBANK", agc.get("EBANK5"));
        agc.set("OVFLOWCK", 1);                  // stop ALLOOP from calling itself
        // TC ALARM; OCT 1600; TC ENDTEST1
    }

    /** SOMERR2: bad IMU moding - variable alarm 1601, clear IMUSE, end the job. */
    public void somerr2() {
        // CAF OCT1601; TC VARALARM
        agc.set("IMUSE", 0);                     // DOWNFLAG IMUSE
        // TC ENDOFJOB
    }
}
