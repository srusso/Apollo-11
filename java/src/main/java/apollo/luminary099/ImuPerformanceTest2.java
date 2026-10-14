package apollo.luminary099;

import apollo.agc.Agc;

/**
 * IMU_PERFORMANCE_TEST_2.agc (Luminary 1A build 099, pages 373-381).
 * IMU performance tests 2 (P07): positioning routines and some of the tests themselves
 * (system test group, March 20, 1967; see STG memo 685, ref. E-1973). Includes the PIPA pulse
 * catching routine CHECKG, earth-rate compensation (ERTHRVSE/EARTHR) and the FINETIME reader.
 *
 * Cross-file calls (BANKCALL, NEWMODEX, FINDVAC, LONGCALL, ...) are expressed as calls to
 * helper classes by naming convention.
 */
public final class ImuPerformanceTest2 {
    public static final int DEC17 = 17;
    public static final int DEC58 = 58;
    public static final int DEC3990 = 3990;
    /** VB06N98: verb 06 noun 98. */
    public static final int VB06N98 = 0698;
    /** VN0641: verb 06 noun 41. */
    public static final int VN0641 = 0641;

    private final Agc agc;

    public ImuPerformanceTest2(Agc agc) {
        this.agc = agc;
    }

    private int g(String s) { return agc.get(s); }
    private void set(String s, int v) { agc.set(s, v); }

    /** REDO: restart into major mode 07. */
    public void redo() {
        Executive.newModeX(agc, 07);
        geoImuTt();
    }

    /** GEOIMUTT: zero the IMU, then initialise counters. */
    public void geoImuTt() {
        imuZerr();
        imuBack();
    }

    /** IMUBACK / NBPOSPL: clear index counters, zero XNB through ZNB+4 and set XNB to HALF. */
    public void imuBack() {
        set("NDXCTR", 0);
        set("TORQNDX", 0);
        set("TORQNDX+1", 0);
        set("OVFLOWCK", 0);
        set("ZERONDX", DEC17);
        zeroing(agc.get("XNB"), DEC17); // CA XNBADR ; TC ZEROING
        set("XNB", 0x2000); // HALF
        latAzChk();
    }

    /**
     * GUESS / LATAZCHK: display latitude and azimuth for loading (V06 N41 flash), then compute the
     * nav-base-to-stable-member matrix columns from azimuth/latitude (WANGI = -cos(lat)*2,
     * WANGO = sin(lat)*2; YNB, ZNB from sin/cos of azimuth) and go to POSGMBL.
     */
    public void latAzChk() {
        double lat = Agc.toSigned(g("LATITUDE")) / 16384.0;
        double az = Agc.toSigned(g("AZIMUTH")) / 16384.0;
        set("DSPTEM1+1", Agc.fromSigned((int) (lat * 4 * 16384)));
        set("DSPTEM1", g("AZIMUTH"));
        // CAF VN0641 ; BANKCALL GOFLASH ; TC ENDTEST1 (V34) / TC +2 (V33) / TC -5 (V32 recycle)
        int response = Executive.goFlash(agc, VN0641);
        if (response == Executive.TERMINATE) {
            endTest1();
            return;
        }
        if (response == Executive.RECYCLE) {
            latAzChk();
            return;
        }
        // Load azimuth and latitude from the display (SR2: undo the SL2 scaling).
        set("AZIMUTH", g("DSPTEM1"));
        set("LATITUDE", Agc.fromSigned(Agc.toSigned(g("DSPTEM1+1")) >> 2));
        lat = Agc.toSigned(g("LATITUDE")) / 16384.0;
        az = Agc.toSigned(g("AZIMUTH")) / 16384.0;
        double radLat = lat * 2 * Math.PI;
        double radAz = az * 2 * Math.PI;
        set("WANGI", Agc.fromSigned((int) (-Math.cos(radLat) * 2 * 16384))); // COS, DCOMP, SL1
        set("WANGO", Agc.fromSigned((int) (Math.sin(radLat) * 2 * 16384)));  // SIN, SL1
        set("YNB+2", Agc.fromSigned((int) (Math.sin(radAz) * 16384)));
        set("ZNB+4", Agc.fromSigned((int) (Math.sin(radAz) * 16384)));
        set("YNB+4", Agc.fromSigned((int) (Math.cos(radAz) * 16384)));
        set("ZNB+2", Agc.fromSigned((int) (-Math.cos(radAz) * 16384))); // DCOMP, POSGMBL
        posGmbl();
    }

    /** POSGMBL: CALCGA, coarse align the IMU, then fine align and wait for suspension to settle. */
    public void posGmbl() {
        // STCALL ZNB+2 / CALCGA ; BANKCALL IMUCOARS
        Imu.calcGa(agc);
        Imu.imuCoars(agc);
        if ((g("FLAGWRD3") & 0x2000) != 0) { // BIT14 set: gimbal lock
            set("NDXCTR", g("NDXCTR") + 1); // +1 if in gimbal lock, otherwise 0
        }
        Executive.downFlag(agc, "GLOKFAIL"); // reset gimbal lock flag
        imuSlLlg();
        if (g("NDXCTR") > 0) { // CCS NDXCTR : if one, PIPA test only
            pipaChk();
            return;
        }
        finImuDd();
        // EXTEND DCA PERFDLAY ; TC LONGCALL ; 2CADR GOESTIMS : delay while suspension stabilises
        Executive.longCall(agc, g("PERFDLAY"), this::goEstims);
        Executive.jobSleep(agc, "ESTICADR");
    }

    /** GOESTIMS: wake the sleeping job at ESTIMS (ESTICADR) and end the task. */
    public void goEstims() {
        Executive.jobWake(agc, "ESTICADR");
        Executive.taskOver(agc);
    }

    /** TORQUE: display drift of the current position. */
    public void torque() {
        set("DSPTEM2", 0);
        set("DSPTEM2+1", g("DRIFTI"));
        set("SOUTHDR" + (g("POSITON") - 1), g("DSPTEM2+1")); // INDEX POSITON ; TS SOUTHDR -1
        show();
    }

    /** PIPACHK: PIPA test. Aligns and measures the vertical PIPA rate. */
    public void pipaChk() {
        if (g("NDXCTR") == 1) {
            earthRStar(); // INDEX NDXCTR ; TC +1 / TC EARTHR*
        }
        set("DATAPL+4", DEC17); // allow PIP counter to overflow 17 times in the interval
        set("LENGTHOT", DEC58);
        set("RESULTCT", 1);
        set("PIPAX+" + g("PIPINDEX"), 0);
        set("DATAPL", 0);
        checkG();
        // INHINT ; CAF TWO ; TC TWIDDLE ; ADRES PIPATASK ; TC ENDOFJOB
        Executive.twiddle(agc, 2, this::pipaTask);
        Executive.endOfJob(agc);
    }

    /** PIPATASK: timer task counting down LENGTHOT; when expired start PIPJOBB. */
    public void pipaTask() {
        set("LENGTHOT", Math.max(0, g("LENGTHOT") - 1)); // DIM
        if (Agc.toSigned(g("LENGTHOT")) > 0) {
            Executive.twiddle(agc, 0x200 /* BIT10 */, this::pipaTask);
            return;
        }
        startPip();
    }

    /** STARTPIP: FINDVAC priority 20 job PIPJOBB, then TASKOVER. */
    public void startPip() {
        Executive.findVac(agc, 020, this::pipJobB);
        Executive.taskOver(agc);
    }

    /** PIPJOBB: pick up the final PIPA counts, compute drift and display via SHOW. */
    public void pipJobB() {
        if (g("NDXCTR") == 1) {
            earthRStar();
        }
        if (Agc.toSigned(g("LENGTHOT")) <= 0) {
            // fall through to compute
        } else {
            Executive.endOfJob(agc);
            return;
        }
        set("RESULTCT", 5);
        checkG();
        int dataPlPlus1 = g("DATAPL+1");
        if (Agc.toSigned(dataPlPlus1) <= 0) {
            // CCSHOLE / negate: DATAPL+4 = -DATAPL+4 ; DATAPL+4 DP -= DATAPL
            set("DATAPL+4", Agc.fromSigned(-Agc.toSigned(g("DATAPL+4"))));
        }
        set("DATAPL+4", Agc.fromSigned(Agc.toSigned(g("DATAPL+4")) - Agc.toSigned(g("DATAPL"))));
        // Interpretive: DLOAD DATAPL+6 ; DSU DATAPL+2 ; BPL AINGOTN else CALL OVERFFIX
        double dt = Agc.toSigned(g("DATAPL+6")) - Agc.toSigned(g("DATAPL+2"));
        if (dt < 0) {
            dt = overfFix(dt);
        }
        // AINGOTN: PDDL DDV: delta pulses / delta time ; DMPR DEC585 ; RTB SGNAGREE
        double rate = dt == 0 ? 0 : Agc.toSigned(g("DATAPL+4")) / dt;
        set("DSPTEM2", Agc.fromSigned((int) (rate * 585))); // DEC585 redefined for LEM
        if (g("NDXCTR") > 0) {
            coAlign(); // take platform out of gimbal lock
        }
        show();
    }

    /** VERTDRFT: about one-hour vertical drift test. */
    public void vertDrft() {
        set("LENGTHOT", DEC3990);
        set("DRIFTT", Agc.fromSigned(-Agc.toSigned(g("SOUTHDR" + (g("POSITON") - 2)))));
        // Offset platform to miss PIP dead-zones
        if (Agc.toSigned(g("PIPINDEX")) > 0) {
            // PON2: X-UP
            set("ERCOMP+2", Agc.add(g("ERCOMP+2"), Agc.fromSigned(-16)));
            set("ERCOMP+4", Agc.add(g("ERCOMP+4"), 16));
        } else {
            // PON4: Z-UP in position 4
            set("ERCOMP+2", Agc.add(g("ERCOMP+2"), Agc.fromSigned(-16)));
            set("ERCOMP", Agc.add(g("ERCOMP"), 16));
        }
        pon();
    }

    /** PON: apply earth rate; only the south gyro compensated; start estimation. */
    private void pon() {
        earthRStar();
        set("ERVECTOR", 0);
        set("ERVECTOR+1", 0);
        guess1();
    }

    /** GUESS1: set torque index to POSMAX, record CDUX, run ESTIMS. */
    public void guess1() {
        set("TORQNDX", 0x3FFF);
        set("TORQNDX+1", 0x3FFF);
        set("LOSVEC", g("CDUX"));
        // TC ESTIMS (defined elsewhere)
        valMis();
    }

    /** VALMIS: display drift (DRIFTO) via SHOW. */
    public void valMis() {
        set("DSPTEM2+1", g("DRIFTO"));
        set("DSPTEM2", 0);
        show();
    }

    /** ENDTEST1: clear IMUSE flag, set mode back to "none" (NEWMODEA with -0 style ZERO) and end. */
    public void endTest1() {
        Executive.downFlag(agc, "IMUSE");
        Executive.newModeA(agc, 0xFFFF & Agc.WORD_MASK);
        Executive.endExt(agc);
    }

    /** OVERFFIX: adds DPPOSMAX and ONEDPP to correct for PIPA clock overflow (DP value). */
    public double overfFix(double x) {
        return x + 0x3FFF + 1; // DPPOSMAX + ONEDPP
    }

    /** COAALIGN: coarse align subroutine - zero THETAD and coarse align the IMU. */
    public void coAlign() {
        set("THETAD", 0);
        set("THETAD+1", 0);
        set("THETAD+2", 0);
        Imu.imuCoars(agc);
        alignCoa();
    }

    /** ALIGNCOA: wait for IMU (IMUSTALL); on error SOMERR2. */
    public void alignCoa() {
        if (!Imu.imuStall(agc)) {
            Executive.someErr2(agc);
        }
    }

    /** IMUSLLLG: coarse align then wait. */
    public void imuSlLlg() {
        alignCoa();
    }

    /** FINIMUDD: IMUFINE (fine align mode) then ALIGNCOA. */
    public void finImuDd() {
        Imu.imuFine(agc);
        alignCoa();
    }

    /** IMUZERR: IMUZERO (zero the IMU CDUs) then ALIGNCOA. */
    public void imuZerr() {
        Imu.imuZero(agc);
        alignCoa();
    }

    /**
     * CHECKG: PIP pulse catching routine. Waits (releasing the processor to other jobs through
     * CHANG1 when NEWJOB is set) until the PIPA register changes, then stores the PIPA count in
     * DATAPL[RESULTCT], the fine time in DATAPL[RESULTCT]+1 and the low scalar in +2.
     */
    public void checkG() {
        while (true) {
            if (g("NEWJOB") != 0 && Agc.toSigned(g("NEWJOB")) > 0) {
                Executive.chang1(agc);
            }
            String pipa = "PIPAX+" + g("PIPINDEX");
            int prev = Agc.fromSigned(-Agc.toSigned(g(pipa))); // CS PIPAX
            set("ZERONDX", prev);
            int now = g(pipa);
            if (Agc.add(now, prev) == 0 || Agc.add(now, prev) == Agc.WORD_MASK) {
                continue; // CHECKG1: no change yet
            }
            int r = g("RESULTCT");
            set("DATAPL+" + r, now);
            set("DATAPL+" + (r + 1), fineTime());
            // LXCH DATAPL+2: low-order fine time word
            return;
        }
    }

    /** ZEROING / ZEROING1: zero {@code count+1} consecutive erasable words starting at {@code address}. */
    public void zeroing(int address, int count) {
        for (int i = count; i >= 0; i--) {
            set("E" + (address + (count - i)), 0);
        }
    }

    /** ERTHRVSE: earth rate reversal. ERVECTOR = (OMEG/MS)*(sin lat? -cos lat, 0). */
    public void erthrvse() {
        double lat = Agc.toSigned(g("LATITUDE")) / 16384.0 * 2 * Math.PI;
        // PD24 = (SIN -COS 0) * OMEG/MS (VDEF, VXSC)
        double omega = Agc.toSigned(g("OMEG/MS")) / 16384.0;
        double[] v = {Math.sin(lat) * omega, -Math.cos(lat) * omega, 0};
        for (int i = 0; i < 3; i++) set("ERVECTOR" + (i == 0 ? "" : "+" + i), Agc.fromSigned((int) (v[i] * 16384)));
        set("TMARK", g("TIME2")); // RTB LOADTIME
        set("ERCOMP", 0);
        set("ERCOMP+1", 0);
        set("ERCOMP+2", 0);
    }

    /**
     * EARTHR: accumulate earth-rate compensation. ERCOMP += XSM * (ERVECTOR * (now - TMARK)*2^9);
     * pulses the IMU gyros (PULSEIMU) with ERCOMP, and updates TMARK.
     */
    public void earthR() {
        int now = g("TIME2"); // RTB LOADTIME
        set("TEMPTIME", now);
        double dt = Agc.toSigned(now) - Agc.toSigned(g("TMARK"));
        if (dt < 0) {
            dt = overfFix(dt); // BPL ERTHR else CALL OVERFFIX
        }
        double scaled = dt * 512; // SL 9D
        for (int i = 0; i < 3; i++) {
            String ev = "ERVECTOR" + (i == 0 ? "" : "+" + i);
            double comp = Agc.toSigned(g(ev)) * scaled / 16384.0;
            String ec = "ERCOMP" + (i == 0 ? "" : "+" + i);
            set(ec, Agc.add(g(ec), Agc.fromSigned((int) comp))); // MXV XSM, VAD ERCOMP (matrix omitted)
        }
        set("TMARK", g("TEMPTIME"));
        Imu.pulseImu(agc, "ERCOMP"); // AXT,1 ECADR ERCOMP ; RTB PULSEIMU
    }

    /** EARTHR*: basic-language wrapper: run EARTHR interpretively, then IMUSLLLG. */
    public void earthRStar() {
        earthR();
        imuSlLlg();
    }

    /** SHOW / SHOW1: display position (V06 N98); V33 proceeds, V34 ends the test, else repeat. */
    public void show() {
        while (true) {
            set("DSPTEM2+2", g("POSITON"));
            int response = Executive.goFlash(agc, VB06N98);
            if (response == Executive.TERMINATE) {
                endTest1();
                return;
            }
            if (response == Executive.PROCEED) {
                return;
            }
        }
    }

    /**
     * FINETIME: returns with interrupt inhibited the 2-word fine time: re-reads LOSCALAR until two
     * successive reads agree (guarding against a rollover), then reads HISCALAR.
     */
    public int fineTime() {
        int lo, again;
        do {
            lo = g("LOSCALAR");
            if (lo != g("LOSCALAR")) { // RXOR test: if differing, re-read
                lo = g("LOSCALAR");
            }
            again = Agc.add(~Agc.WORD_MASK & Agc.WORD_MASK, 0); // CS POSMAX ; AD L (retry if L == POSMAX)
        } while (lo == 0x3FFF);
        set("FINETIME_L", lo);
        return g("HISCALAR");
    }
}
