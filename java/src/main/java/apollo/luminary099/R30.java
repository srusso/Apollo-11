package apollo.luminary099;

import apollo.agc.Agc;

/**
 * R30.agc (Luminary 099, pp. 712-721) - Verb 82 (V82CALL) orbital parameter display and
 * subroutine SR30.1.
 *
 * V82CALL: calculate and display HAPOX, HPERX and TFF (V16N44). Double-precision
 * quantities are modelled with {@link #dp}/{@link #setDp} as signed 28-bit values
 * (high word at the symbol, low word at symbol+1).
 */
public class R30 {
    private final Agc agc;

    /** MINPERM: 10668 B-27, 35 kft minimum perigee height for the Moon (-27) m. */
    public static final double MINPERM = 10668;
    /** MINPERE: 91440 B-29, 300 kft minimum perigee height for the Earth (-29) m. */
    public static final double MINPERE = 91440;
    /** MAXNM: 2OCT 0106505603 = 9999.9 NM limit for displayed HAPO/HPER. */
    public static final long MAXNM = 0x0106505603L & 0x0FFFFFFFL; // 2OCT 0106505603 (DP)
    /** OPTIONVN: VN 412 (V04 N06). */
    public static final int OPTIONVN = 412;
    /** V16N44: VN 1644. */
    public static final int V16N44 = 1644;
    /** DP centiseconds in one second (1SEC = 100). */
    public static final int ONE_SEC = 100;
    /** Flag bits in V82FLAGS. */
    public static final int FLAG_TPER = 1, FLAG_TFF = 2;

    public R30(Agc agc) {
        this.agc = agc;
    }

    private static int s(int w) { return Agc.toSigned(w); }

    private long dp(String sym) {
        return ((long) s(agc.get(sym)) << 14) + s(agc.get(sym + "+1"));
    }

    private void setDp(String sym, long v) {
        agc.set(sym, Agc.fromSigned((int) (v >> 14)));
        agc.set(sym + "+1", Agc.fromSigned((int) (v & 0x3FFF)));
    }

    /** V82CALL: Verb 82 ENTER. Branch on AVEGFLAG (average G on/off). */
    public void v82Call() {
        if (agc.get("AVEGFLAG") != 0) {
            v82Gon();
        } else {
            v82Goff();
        }
    }

    /** V82GOFF: average G off - let the astronaut select the vehicle (V04N06). */
    public void v82Goff() {
        agc.set("OPTIONX", 2);
        agc.set("OPTIONX+1", 1);                 // 1 = this vehicle, not 1 = other
        // GOXDSPF with OPTIONVN; terminate -> ENDEXT; data in -> redisplay.
        // CAF BIT4 / TC WAITLIST / 2CADR TICKTEST: start TICKTEST in 80 ms.
        v82GoFlp();
    }

    /** V82GOFLP: major recycle loop entry. */
    public void v82GoFlp() {
        agc.set("V82FLAGS", 0);                  // inhibit decrementing of TFF and -TPER
        // FINDVAC at PRIO7: V82GOFF1 does the state vector update and orbit calc.
        v82Goff1();
        v82Stall();
    }

    /** V82STALL: wait (1 s steps) until the state vector update sets one of our flag bits. */
    public void v82Stall() {
        while ((agc.get("V82FLAGS") & 3) == 0) {
            // CAF 1SEC / BANKCALL DELAYJOB
            break;
        }
        flagGon();
    }

    /** FLAGGON: monitor HAPO, HPER, TFF (V16N44). Terminate/proceed -> B5OFF; recycle -> V82GOFLP. */
    public void flagGon() {
        // GOXDSPF V16N44
    }

    /** B5OFF: clear bit 5 of EXTVBACT, telling TICKTEST (or V82GON1) to kill itself. */
    public void b5Off() {
        agc.set("EXTVBACT", agc.get("EXTVBACT") & ~0x10);
    }

    /** V82GOFF1: update the state vector of the selected vehicle and do orbit calculations. */
    public void v82Goff1() {
        long now = loadTime();
        setDp("TDEC1", now);                     // time for state vector update
        setDp("TSTART82", now);                  // time for internal use
        if (s(agc.get("OPTIONX+1")) == 1) {
            thisShip();
        } else {
            othShip();
        }
    }

    /** OTHSHIP: update the other vehicle's state vector (OTHPREC). */
    public void othShip() {
        // CALL OTHPREC
        bothShip();
    }

    /** THISSHIP: update this vehicle's state vector (THISPREC). */
    public void thisShip() {
        // CALL THISPREC
        bothShip();
    }

    /**
     * BOTHSHIP: move RATT/VATT into RONE/VONE for SR30.1, load the body constants
     * (index X2: 0 Earth, 2 Moon) and branch to the pad radius computation.
     */
    public void bothShip() {
        // RONE = RATT (-29 m); VONE = VATT (-7 m/cs)
        // TFF/RTMU = 1/RTMUE,2 ; HPERMIN = MINPERE,2
        if (s(agc.get("X2")) == 0) {
            earthPad();
        } else {
            moonPad();
        }
    }

    /** EARTHPAD: pad 37-B radius (-29) m; clear V82EMFLG. */
    public void earthPad() {
        agc.set("V82EMFLG", 0);
        bothPad(dp("RPAD"));
    }

    /** MOONPAD: radius of the landing site |RLS| (-27) m; set V82EMFLG. */
    public void moonPad() {
        agc.set("V82EMFLG", 1);
        bothPad(absVal("RLS"));
    }

    private long absVal(String vec) {
        double x = dp(vec), y = dp(vec + "+2"), z = dp(vec + "+4");
        return (long) Math.sqrt(x * x + y * y + z * z);
    }

    /** BOTHPAD: RPADTEM = radius; SR30.1; then correct TFF or -TPER for elapsed time. */
    private void bothPad(long radius) {
        setDp("RPADTEM", radius);
        sr301();
        long elapsed = loadTime() - dp("TSTART82");   // present time - time V82GOFF1 began
        setDp("TSTART82", elapsed);
        if (dp("-TPER") == 0) {
            tickTiff(elapsed);
        } else {
            tickTper(elapsed);
        }
    }

    /** TICKTPER: -TPER nonzero, so TFF was not computed - tick -TPER only. */
    private void tickTper(long elapsed) {
        setDp("-TPER", dp("-TPER") + elapsed);
        agc.set("V82FLAGS", FLAG_TPER);
        // ENDOFJOB
    }

    /** TICKTIFF: -TPER = 0, TFF was computed - tick TFF only. */
    private void tickTiff(long elapsed) {
        setDp("TFF", dp("TFF") + elapsed);
        agc.set("V82FLAGS", FLAG_TFF);
        // ENDOFJOB
    }

    /** TICKTEST: waitlist task repeating once per second until bit 5 of EXTVBACT is clear. */
    public void tickTest() {
        if ((agc.get("EXTVBACT") & 0x10) != 0) {
            doTick();
        } else {
            // NOVAC ENDEXT at PRIO25 to terminate V82 (cannot call ENDEXT in a rupt).
            // TC TASKOVER
        }
    }

    /** DOTICK: re-request TICKTEST in 1 s and increment TFF or -TPER according to V82FLAGS. */
    private void doTick() {
        // CAF 1SEC / TC WAITLIST / 2CADR TICKTEST
        switch (agc.get("V82FLAGS") & 3) {
            case 0:
                break;                                  // no flag bits: continue the loop only
            case 1:
                tperTick();
                break;
            default:
                tffTick();
                break;
        }
    }

    /** TFFTICK: increment TFF by one second. */
    private void tffTick() { setDp("TFF", dp("TFF") + ONE_SEC); }

    /** TPERTICK: increment -TPER by one second. */
    private void tperTick() { setDp("-TPER", dp("-TPER") + ONE_SEC); }

    /** V82GON: average G on - use the current state vector, once a second. */
    public void v82Gon() {
        // FINDVAC at PRIO7: V82GON1; CCS NEWJOB / CHANG1 holds off V16N44 until first calc.
        v82Gon1();
        v82ReDsp();
    }

    /** V82REDSP: monitor HAPO, HPER, TFF; terminate/proceed -> B5OFF; recycle -> redisplay. */
    public void v82ReDsp() {
        // GOXDSPF V16N44
    }

    /** V82GON1: perpetuating EXEC job (once a second) until bit 5 of EXTVBACT clears. */
    public void v82Gon1() {
        // RONE = RN, VONE = VN (same state vector update); then select the body.
        if (agc.get("MOONTHIS") != 0) {
            moonGon();
        } else {
            earthGon();
        }
    }

    /** MOONGON: lunar parameters for SR30.1. */
    private void moonGon() {
        agc.set("V82EMFLG", 1);
        // TFF/RTMU = 1/RTMUM; HPERMIN = MINPERM; RPADTEM = |RLS|
        setDp("RPADTEM", absVal("RLS"));
        v82Gon2();
    }

    /** EARTHGON: Earth parameters for SR30.1. */
    private void earthGon() {
        agc.set("V82EMFLG", 0);
        // TFF/RTMU = 1/RTMUE; HPERMIN = MINPERE; RPADTEM = RPAD
        setDp("RPADTEM", dp("RPAD"));
        v82Gon2();
    }

    /** V82GON2: common code; call SR30.1 then check for terminate. */
    private void v82Gon2() {
        sr301();
        v82Gon3();
    }

    /** V82GON3 (= SPLRET): end if EXTVBACT bit 5 is clear, else wait 1 s and repeat. */
    public void v82Gon3() {
        if ((agc.get("EXTVBACT") & 0x10) == 0) {
            // TC ENDEXT
            return;
        }
        // CAF 1SEC / BANKCALL DELAYJOB, then V82GON1 again.
    }

    // -------------------------------------------------------------------- SR30.1

    /**
     * SR30.1: compute RAPO, RPER, HAPOX, HPERX, -TPER and TFF from RONE/VONE.
     * Moon-centred values are rescaled (RONE, VONE shifted left 2) before TFFCONMU.
     */
    public void sr301() {
        boolean moon = agc.get("V82EMFLG") != 0;
        if (moon) {
            // RONE <<= 2; VONE <<= 2 (B-27 / B-5 scaling for the TFF routines)
        }
        // CALL TFFCONMU ; CALL TFFRP/RA (RAPO in MPAC)
        long rapo = dp("RAPO");
        long hapo = rapo - dp("RPADTEM");
        long hapoDisp = moon ? hapo >> 2 : hapo;       // BOFF SR2R: rescale to (-29) m
        setDp("HAPOX", maxChk(hapoDisp));               // STORHAPO (limited to 9999.9 NM)
        long hper = dp("RPER") - dp("RPADTEM");         // saved in MPAC+4 for the comparison
        long hperDisp = moon ? hper >> 2 : hper;
        setDp("HPERX", maxChk(hperDisp));               // STORHPER
        long minus;
        if (hper - dp("HPERMIN") >= 0) {
            // DOTPER: CALL CALCTPER; DCOMP -> -TPER
            minus = -calcTper(dp("RPER"));
        } else {
            minus = 0;                                  // HPER < HPERMIN: zero into -TPER
        }
        setDp("-TPER", minus);
        // CALCTFF gives 59M59S if TPER was nonzero; TFF = -result
        setDp("TFF", -calcTff(dp("HPERMIN") + dp("RPADTEM")));
    }

    /** MAXCHK: if the value exceeds 9999.9 NM limit it to MAXNM. */
    public long maxChk(long v) {
        return v - MAXNM > 0 ? MAXNM : v;
    }

    private long calcTper(long rper) { return 0; /* CALCTPER (R31 library) */ }

    private long calcTff(long radius) { return 0; /* CALCTFF (R31 library) */ }

    private long loadTime() {
        return ((long) s(agc.get("TIME2")) << 14) + s(agc.get("TIME1"));
    }
}
