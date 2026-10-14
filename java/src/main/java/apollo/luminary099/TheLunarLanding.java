package apollo.luminary099;

import apollo.agc.Agc;

/**
 * THE_LUNAR_LANDING.agc (Luminary 099): P63 (the lunar landing, braking phase) start-up and
 * ignition algorithm (IGNALG / DDUMCALC), the burn-attitude set-up (IGNALGRT / ASTNCLOK /
 * P63SPOT2..4) and P68 (landing confirmation, LANDJUNK).
 *
 * <p>Interpretive code is written with doubles holding the stored fraction of full scale
 * ({@link VecOps}); push-down list slots become Java locals. Flags are erasable symbols
 * (non-zero = set).
 */
public class TheLunarLanding {
    public static final int PHASE_04024 = 04024;
    public static final int CODE500 = 00500;
    public static final int V06N43 = 643;
    /** ASTNDEX = MD1 (OCT 25): index for CLOKTASK. */
    public static final int ASTNDEX = 025;
    /** 99999CON: 2DEC 30479.7 B-24. */
    public static final double CON_99999 = 30479.7 / (1L << 24);
    /** GUIDDURN: 2DEC +66440 (6.64400314E+2 s, in centiseconds scaled B-28). */
    public static final double GUIDDURN = 66440.0 / (1L << 28);
    /** DDUMCRIT: criterion for IGNALG convergence, +8 B-28. */
    public static final double DDUMCRIT = 8.0 / (1L << 28);
    public static final int NIGNLOOP_COUNT = 40; // SSP NIGNLOOP 40D

    private final Agc agc;

    public TheLunarLanding(Agc agc) {
        this.agc = agc;
    }

    // ---- helpers ----
    private boolean flag(String n) {
        return agc.get(n) != 0;
    }

    private void flag(String n, boolean on) {
        agc.set(n, on ? 1 : 0);
    }

    private double dp(String s) {
        return VecOps.getDp(agc, s);
    }

    private void dp(String s, double v) {
        VecOps.setDp(agc, s, v);
    }

    private double[] vec(String s) {
        return VecOps.getV(agc, s);
    }

    private void vec(String s, double[] v) {
        VecOps.setV(agc, s, v);
    }

    private void phaschng(int code) {
        new PhaseTableMaintenance(agc).phaschng(code);
    }

    private int flash(int vn) {
        return new PinballGameButtonsAndLights(agc).goflash(String.valueOf(vn));
    }

    private void gotopooh() {
        new ServiceRoutines(agc).gotopooh();
    }

    // =====================================================================
    // P63 : THE LUNAR LANDING, BRAKING PHASE
    // =====================================================================

    /** P63LM: initialise braking phase variables, flags, then compute the ignition algorithm. */
    public void p63lm() {
        phaschng(PHASE_04024);
        new ImuModeSwitchingRoutines(agc).r02both();     // BANKCALL R02BOTH: IMU status check
        agc.set("WHICH", agc.get("P63TABLE"));           // P63ADRES = GENADR P63TABLE ; initialise BURNBABY
        agc.set("DVTHRUSH", agc.get("DPSTHRSH"));        // INITIALIZE DVMON
        agc.set("DVCNTR", 4);
        agc.set("WCHPHASE", Agc.fromSigned(-1));         // CS ONE ; TS WCHPHASE
        agc.set("FLPASS0", 0);
        // CS BIT14 ; EXTEND ; WAND CHAN12 : remove track-enable discrete
        agc.set("CHAN12", agc.get("CHAN12") & ~(1 << 13));
        flagOrgy();
        ignalg();
    }

    /** FLAGORGY: Dionysian flag waving. */
    public void flagOrgy() {
        flag("NOTHROTL", false);
        flag("REDFLAG", false);
        flag("LRBYPASS", false);
        flag("MUNFLAG", true);
        flag("P25FLAG", false);                          // terminate P25 if running
        flag("RNDVZFLG", false);                         // terminate P20 if running
    }

    /**
     * IGNALG: set up inputs for RP-TO-R (landing site in moon-fixed frame, time of landing),
     * integrate the state forward to ignition and iterate (IGNALOOP / DDUMCALC) until the
     * ignition time converges.
     */
    public void ignalg() {
        // SETPD 0 ; VLOAD RLS ; PDDL TLAND ; PUSH ; STCALL TPIP ; RP-TO-R
        double[] rls = vec("RLS");
        double tland = dp("TLAND");
        dp("TPIP", tland);                               // also set TPIP for first guidance pass
        double[] r = new LatitudeLongitudeSubroutines(agc).rpToR(rls, tland); // MPAC non-zero: lunar
        double[] land = VecOps.mxv(VecOps.getM(agc, "REFSMMAT"), VecOps.scale(r, 16));  // VSL4 MXV
        vec("LAND", land);
        new BurnBabyBurn(agc).guidinit();                // GUIDINIT initialises WM and /LAND/
        dp("TDEC1", tland - GUIDDURN);                   // integrate state forward to that time
        new OrbitalIntegration(agc).lemprec();
        agc.set("NIGNLOOP", NIGNLOOP_COUNT);             // SSP NIGNLOOP 40D
        vec("CG", new double[] {1, 0, 0});               // UNITX
        vec("CG+6", new double[] {0, 1, 0});             // UNITY
        vec("CG+14", new double[] {0, 0, 1});            // UNITZ (as stored)
        dp("DELTAH", CON_99999);                         // INITIALIZE DELTAH FOR V16N68 DISPLAY
        vec("UNFC/2", new double[3]);                    // INITIALIZE TRIM VELOCITY CORRECTION TERM
        dp("TTF/8", 0);                                  // HI6ZEROS
        ignaloop();
    }

    /**
     * IGNALOOP: one pass: PIPTIME1 = TAT, position in RATT1 rotated to the stable member (R),
     * gravity (MUNGRAV) and ?GUIDSUB which delivers N passes of guidance, then DDUMCALC.
     */
    public void ignaloop() {
        while (true) {
            dp("PIPTIME1", dp("TAT"));
            double[] r = VecOps.mxv(VecOps.getM(agc, "REFSMMAT"), VecOps.scale(vec("RATT1"), 16));
            vec("R", r);
            vec("GDT/2", new BurnBabyBurn(agc).mungrav(r));
            int n = new BurnBabyBurn(agc).guidsub();     // ?GUIDSUB -> NIGNLOOP count in A
            if (!ddumcalc(n)) {
                break;
            }
        }
    }

    /**
     * DDUMCALC: ignition time correction DDUM =
     * [ (RIGNZ - RGU2)/16 + 16 RGU1^2 KIGNY/B8 + (RGU0 - RIGNX) KIGNX/B4 + (|VGU| - VIGN) KIGNV/B4 ]
     * / [ 2^10 (VGU2 - 16 VGU0 KIGNX/B4) ]. The numerator is metres at 2^28, the denominator
     * velocity units 2^10 m/cs; the quotient is a time in 2^18 cs and is rescaled to 2^28.
     * There is no damping. KIGNX/B4, KIGNY/B8, KIGNV/B4 are all negative.
     *
     * @return true to run another IGNALOOP pass; false once DDUMGOOD was reached.
     */
    public boolean ddumcalc(int nIgnLoop) {
        agc.set("NIGNLOOP", nIgnLoop);
        double[] vgu = vec("VGU");
        double[] rgu = vec("RGU");
        double kignxB4 = dp("KIGNX/B4");
        double kignyB8 = dp("KIGNY/B8");
        double kignvB4 = dp("KIGNV/B4");
        double den = vgu[2] - 16 * vgu[0] * kignxB4;                       // FORM DENOMINATOR FIRST
        double num = (dp("RIGNZ") - rgu[2]) / 16
            + 16 * rgu[1] * rgu[1] * kignyB8
            + (rgu[0] - dp("RIGNX")) * kignxB4
            + (VecOps.abs(vgu) - dp("VIGN")) * kignvB4;
        double ddum = den == 0 ? 0 : num / den / 1024.0;                   // DDV ; SRR 10D
        double newGuess = ddum + dp("PIPTIME1");                           // PUSH DAD PIPTIME1
        dp("TDEC1", newGuess);                                             // new guess for next integration
        if (Math.abs(ddum) - DDUMCRIT < 0) {                               // BMN DDUMGOOD
            ddumgood();
            return false;
        }
        new Interpreter(agc).intstall();                                   // CALL INTSTALL
        flag("INTYPFLG", true);
        flag("MOONFLAG", true);
        dp("TET", dp("PIPTIME1"));                                         // hopefully ?GUIDSUB did not clobber
        vec("RCV", vec("RATT1"));                                          //   RATT1 and VATT1
        vec("VCV", vec("VATT1"));
        new OrbitalIntegration(agc).integrvs();
        return true;                                                       // GOTO IGNALOOP
    }

    /**
     * DDUMGOOD: TIG = TDEC1 - ZOOMTIME/2^14; compute the distance the landing site will be
     * out of the LM orbital plane at ignition (positive if to the right/north) into OUTOFPLN;
     * UNFC/2 is saved temporarily in R60VSAVE (R60INIT).
     */
    public void ddumgood() {
        double zoom = Agc.toSigned(agc.get("ZOOMTIME")) / 16384.0;
        dp("TIG", dp("TDEC1") - zoom);                                      // SLOAD SR 14D ; BDSU TDEC1
        double[] v = vec("V");
        double[] r = vec("R");
        double[] n = VecOps.unit(VecOps.cross(v, r));
        dp("OUTOFPLN", 2 * VecOps.dot(n, vec("LAND")));                     // DOT SL1 LAND
        vec("R60VSAVE", vec("UNFC/2"));                                     // R60INIT
        ignalgrt();
    }

    // =====================================================================
    // Burn attitude set-up
    // =====================================================================

    /** IGNALGRT: prevent repeating IGNALG, start the clock task, continue in a new job. */
    public void ignalgrt() {
        phaschng(PHASE_04024);
        astnclok();
    }

    /** ASTNCLOK: CS ASTNDEX ; BANKCALL STCLOK2 ; ENDOFJOB (resumes at ASTNRET in a new job). */
    public void astnclok() {
        new ServiceRoutines(agc).stclok2(~ASTNDEX & Agc.WORD_MASK);
        new ServiceRoutines(agc).endofjob();             // RETURN IN NEW JOB AND IN EBANK FIVE
    }

    /**
     * ASTNRET: pick up the display at the end of R51 ("PROCEED" does a fine alignment,
     * "ENTER" returns to P63SPOT2).
     */
    public void astnret() {
        agc.set("QMAJ", agc.get("P63SPOT2"));
        new ServiceRoutines(agc).r51p63();               // RTB R51P63
        p63spot2();
    }

    /** P63SPOT2: initialise KALCMANU for burn attitude and run R60LEM. */
    public void p63spot2() {
        double[] u = VecOps.unit(vec("R60VSAVE"));
        vec("POINTVSM", u);
        vec("SCAXIS", new double[] {1, 0, 0});           // UNITX
        agc.set("EBANK", 7);                             // CAF EBANK7 ; TS EBANK
        // INHINT ; IBNKCALL PFLITEDB ; RELINT
        new BurnBabyBurn(agc).pflitedb();
        new R6062(agc).r60lem();                         // BANKCALL R60LEM
        phaschng(PHASE_04024);                           // prevent recalling R60
        p63spot3();
    }

    /** P63SPOT3: is the landing radar antenna in position 1 yet? Ask the astronaut to crank it. */
    public void p63spot3() {
        while ((agc.get("CHAN33") & (1 << 5)) != 0) {    // CA BIT6 ; RAND CHAN33 ; BZF -> position 1
            int r = new PinballGameButtonsAndLights(agc).gopErf1(CODE500);
            if (r == 0) {                                // TERMINATE
                gotopooh();
                return;
            }
            // PROCEED: see if he's lying
        }
        p63spot4();
    }

    /** P63SPOT4: initialise landing radar (SETPOS1) and POSTJUMP to BURNBABY. */
    public void p63spot4() {
        new RadarLeadinRoutines(agc).setpos1();
        new BurnBabyBurn(agc).burnbaby();
    }

    // =====================================================================
    // P68 : LANDING CONFIRMATION
    // =====================================================================

    /**
     * LANDJUNK (P68): zero attitude error, set 5 degree deadband, set SURFFLAG, clear LETABORT,
     * set APSFLAG, record the landing site (RLS) via LAT-LONG and R-TO-RP, display V06N43, then
     * initialise GSAV/REFMF for P57 and go to GOTOPOOH ("PLEASE SELECT P57").
     */
    public void landjunk() {
        phaschng(PHASE_04024);
        // INHINT
        new DapInterfaceSubroutines(agc).zatteror();     // ZERO ATTITUDE ERROR
        new DapInterfaceSubroutines(agc).setmaxdb();     // SET 5 DEGREE DEADBAND
        flag("SURFFLAG", true);
        flag("LETABORT", false);
        flag("APSFLAG", true);
        double[] rn = vec("RN");
        vec("ALPHAV", rn);
        dp("ALPHAV+6", dp("PIPTIME"));
        flag("LUNAFLAG", true);
        new LatitudeLongitudeSubroutines(agc).latLong(dp("PIPTIME"));  // CALL LAT-LONG
        // COMPUTE RLS AND STORE IT AWAY
        double[] rls = new LatitudeLongitudeSubroutines(agc).rToRp(
            VecOps.scale(vec("RN"), 4), dp("PIPTIME"));   // VSL2 ; PUSH ; CALL R-TO-RP
        vec("RLS", rls);
        while (true) {
            int r = flash(V06N43);                        // NOW LOOK WHERE YOU ENDED UP
            if (r == 0) {
                gotopooh();                               // TERMINATE
                return;
            }
            if (r == 1) {
                break;                                    // PROCEED
            }
        }
        // INITIALIZE GSAV AND (USING REFMF) YNBSAV, ZNBSAV AND ATTFLAG FOR P57
        vec("GSAV", new double[] {1, 0, 0});              // UNITX
        new LatitudeLongitudeSubroutines(agc).refmf();    // STCALL REFMF
        gotopooh();                                       // ASTRONAUT: PLEASE SELECT P57
    }
}
