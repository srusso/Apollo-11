package apollo.luminary099;

import apollo.agc.Agc;

/**
 * AGS INITIALIZATION (R47), AGS_INITIALIZATION.agc.
 *
 * <p>Provides the AGS Abort Electronics Assembly (AEA) with the LEM and CSM state vectors
 * (position, velocity, time) in LEM IMU coordinates via the LGC digital downlink, zeroes the
 * ICDU/LGC/AEA gimbal angle counters simultaneously, and establishes the ground elapsed time
 * of AEA clock zero (AGSK). Entered when the astronaut keys V47E. Alarm 220 if REFSMMAT is bad.
 * Erasables: SAMPTIME (time of ENTER), AGSK (AEA clock zero), AGSBUFF (14D output buffer),
 * AGSWORD (previous downlist).
 */
public final class AgsInitialization {
    private final Agc agc;

    public AgsInitialization(Agc agc) {
        this.agc = agc;
    }

    // ---- constants (end of file) ----
    /** LAGSLIST = ONE: downlist code for the AGS list. */
    public static final int LAGSLIST = 1;
    public static final int V01N14 = 0114;
    public static final int V50N00A = 05000;
    public static final int V00N25 = 031;
    public static final int V06N16 = 0616;
    public static final int V00N34 = 34;
    public static final int V50N16 = 05016;
    /** CSEC to SEC scale factor: 100 B-10. */
    public static final long TSCALE = Math.round(100 * Math.pow(2, -10) * (1L << 28));
    /** 20 seconds in centiseconds. */
    public static final int SEC20 = 2000;
    /** Meters to feet scale factor: 3.280839 B-3. */
    public static final long RSCALE = Math.round(3.280839 * Math.pow(2, -3) * (1L << 28));
    /** Meters/cs to feet/sec scale factor: 3.280839 E2 B-9. */
    public static final long VSCALE = Math.round(3.280839e2 * Math.pow(2, -9) * (1L << 28));
    /** AGS rounding vector 1 (2OCT 0000060000 x3). */
    public static final long[] AGSRND1 = {060000L, 060000L, 060000L};
    /** AGS rounding vector 2 (2OCT 0000037777 x3). */
    public static final long[] AGSRND2 = {037777L, 037777L, 037777L};

    private static final int REFSMBIT = 0; // FLAGWRD3 bit mask for REFSMFLG, defined in FLAGWORD_ASSIGNMENTS
    private static final int BIT6 = 040;

    /** AGSINIT: entry point of R47 (V47E). */
    public void agsinit() {
        if (agc.get("FLAGWRD3") != 0 && (agc.get("FLAGWRD3") & refsmbit()) != 0) {
            redsptem();                         // REFSMMAT IS OK
            return;
        }
        new AlarmAndAbort(agc).alarm(0220, 0);     // REFSMMAT IS BAD
        endext();
    }

    private int refsmbit() {
        return agc.get("REFSMBIT");
    }

    /** NEWAGS: the time of the ENTER keystroke becomes the new AEA clock "zero". */
    public void newags() {
        // EXTEND; DCA SAMPTIME; DXCH AGSK
        int hi = agc.get("SAMPTIME");
        int lo = agc.get("SAMPTIME+1");
        agc.set("AGSK", hi);
        agc.set("AGSK+1", lo);
        redsptem();
    }

    /** REDSPTEM: display the current AGSK (V06N16: HRS in R1, MIN in R2, SEC in R3). */
    public void redsptem() {
        agc.set("DSPTEMX", agc.get("AGSK"));
        agc.set("DSPTEMX+1", agc.get("AGSK+1"));
        agsdispk();
    }

    /** AGSDISPK: flash V06N16 and handle the terminate / proceed / enter returns. */
    public void agsdispk() {
        int ret = new PinballGameButtonsAndLights(agc).gomarkf(String.valueOf(V06N16)); // 0 = terminate, 1 = proceed
        if (ret == 0) {
            endext();                           // TERMINATE RETURN
            return;
        }
        // PROCEED RETURN: AGSVCALC
        if (Agc.add(Agc.fromSigned(-BIT6), agc.get("MPAC")) == 0 || agc.get("MPAC") == BIT6) {
            newags();                           // ENTER VIA A V32: use keystroke time as new AGSK
            return;
        }
        // NEW AGSK LOADED VIA V25 INTO DSPTEMX BY KEYING HRS, MINS, SECS
        agc.set("AGSK", agc.get("DSPTEMX"));
        agc.set("AGSK+1", agc.get("DSPTEMX+1"));
        redsptem();                             // TC REDSPTEM -1 (display the new K)
    }

    /** AGSVCALC: extrapolate the LEM and CSM state vectors and send them down. */
    public void agsvcalc() {
        Interpreter interp = new Interpreter(agc);
        interp.set("NODOFLAG");                 // DON'T ALLOW V37
        interp.set("XDSPFLAG");
        new DisplayInterfaceRoutines(agc).exDspRet(V06N16);

        interp.rtb("LOADTIME");                 // LOAD MPAC WITH TIME2,TIME1
        agc.set("TDEC1", agc.get("MPAC"));
        agc.set("TDEC1+1", agc.get("MPAC+1"));
        new IntegrationInitialization(agc).lemprec();    // CALCULATE LEM STATE VECTOR
        scalevec();                             // CONVERT TO SM COORDS AND SCALE
        storeVector("AGSBUFF", 0);
        // TAT = time to which RATT1 and VATT1 are computed (cs since clock start, B-28)
        long tat = dp(agc.get("TAT"), agc.get("TAT+1"));
        agc.set("TDEC1", agc.get("TAT"));
        agc.set("TDEC1+1", agc.get("TAT+1"));
        new IntegrationInitialization(agc).csmprec();    // CSM STATE VECTOR FOR SAME TIME
        scalevec();
        storeVector("AGSBUFF", 6);

        // AGSBUFF+12D = (TAT - AGSK) / TSCALE
        long t = dp(agc.get("TAT"), agc.get("TAT+1")) - dp(agc.get("AGSK"), agc.get("AGSK+1"));
        long q = (long) ((double) t * (1L << 28) / TSCALE);
        agc.set("AGSBUFF+12", (int) ((q >> 14) & Agc.WORD_MASK));
        agc.set("AGSBUFF+13", (int) (q & Agc.WORD_MASK));
        if (tat == Long.MIN_VALUE) {
            return; // unreachable; keeps tat referenced as in the original TAT reload
        }

        agc.set("DNLSTCOD", LAGSLIST);          // SEND AGS DOWNLIST
        new ServiceRoutines(agc).delayJob(SEC20, 0); // DELAY 20 SEC WHILE THE DOWNLIST IS TRANSMITTED
        agc.set("DNLSTCOD", agc.get("AGSWORD")); // RETURN TO THE OLD DOWNLIST

        if ((agc.get("FLAGWRD0") & agc.get("IMUSEBIT")) != 0) {
            agsend();                           // IMU IS BEING USED -- DO NOT ZERO
            return;
        }
        ckstall();
    }

    /** CKSTALL: wait until IMU compensation is not using the IMU, then zero it. */
    public void ckstall() {
        while (true) {
            int imucadr = agc.get("IMUCADR");   // CCS IMUCADR
            if (imucadr == 0) {
                break;                          // FREE: go ahead with IMU zero (+6)
            }
            new ServiceRoutines(agc).delayJob(10, 0); // WAIT .1 SEC AND TRY AGAIN (+3)
        }
        new ImuModeSwitchingRoutines(agc).imuZero(); // SET IMU ZERO DISCRETE FOR 320 MSECS
        new ImuModeSwitchingRoutines(agc).imuStall(); // WAIT 3 SEC FOR COUNTERS TO INCREMENT
        agsend();
    }

    /** AGSEND: allow V37 again and display V50N16 (please perform). */
    public void agsend() {
        new Interpreter(agc).clear("NODOFLAG"); // TC DOWNFLAG / ADRES NODOFLAG: ALLOW V37
        new DisplayInterfaceRoutines(agc).goMark3(V50N16);
        endext();
    }

    /**
     * SCALEVEC: convert VATT1/RATT1 to stable-member coordinates (MXV REFSMMAT), scale to feet
     * (VSCALE / RSCALE), add rounding vectors AGSRND1/AGSRND2 and correct for the AGS being a
     * 2's complement machine (RTB VECSGNAG). Result left in VATT1 / RATT1 (LXA/SXA sequence).
     */
    public void scalevec() {
        Interpreter interp = new Interpreter(agc);
        interp.vload("VATT1");
        interp.mxv("REFSMMAT");
        interp.vxsc(VSCALE);
        interp.vsl2();
        interp.vad(AGSRND1);                    // ROUND THE VECTOR
        interp.vad(AGSRND2);                    // TWO VECTORS ADDED TO DEFEAT ALSIGNAG
        interp.rtb("VECSGNAG");                 // 1'S TO 2'S COMPLEMENT
        interp.stovl("VATT1");
        interp.vload("RATT1");
        interp.mxv("REFSMMAT");
        interp.vxsc(RSCALE);
        interp.vsl8();
        interp.vad(AGSRND1);
        interp.vad(AGSRND2);
        interp.rtb("VECSGNAG");
        // LXA,1 VATT1 / SXA,1 MPAC+1 / LXA,1 VATT1+2 / SXA,1 MPAC+4 / LXA,1 VATT1+4 / SXA,1 MPAC+6
        agc.set("MPAC+1", agc.get("VATT1"));
        agc.set("MPAC+4", agc.get("VATT1+2"));
        agc.set("MPAC+6", agc.get("VATT1+4"));
    }

    private void storeVector(String base, int offset) {
        for (int i = 0; i < 6; i++) {
            agc.set(sym(base, offset + i), agc.get(sym("MPAC", i)));
        }
    }

    private static String sym(String base, int i) {
        return i == 0 ? base : base + "+" + i;
    }

    private static long dp(int hi, int lo) {
        return ((long) Agc.toSigned(hi) << 14) + Agc.toSigned(lo);
    }

    private void endext() {
        new ExtendedVerbs(agc).endext();
    }
}
