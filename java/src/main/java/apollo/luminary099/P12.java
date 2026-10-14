package apollo.luminary099;

import apollo.agc.Agc;

/**
 * P12.agc (Luminary 099, pp. 838-842) - powered ascent guidance initialisation (P12).
 * Interpretive vector/scalar operations are expressed as comments of the form
 * {@code // X = f(...)} next to calls into the helper routines they invoke.
 */
public class P12 {
    private final Agc agc;

    /** 49FPS: expected RDOT at tipover, .149352 B-6 (2DEC). */
    public static final double FPS_49 = 0.149352;
    /** VINJNOM: 5509.5 fps (APO=30NM with RDOT=19.5FPS), 16.7924 B-7. */
    public static final double VINJNOM = 16.7924;
    /** RDOTDNOM: 19.5 fps, .059436 B-7. */
    public static final double RDOTDNOM = 0.059436;
    /** PHASCHNG code used throughout P12: OCT 04024. */
    public static final int PHASE_04024 = 04024;

    public P12(Agc agc) {
        this.agc = agc;
    }

    private static int s(int w) { return Agc.toSigned(w); }

    /** Outcome of a GOFLASH display request. */
    public enum Response { TERMINATE, PROCEED, ENTER }

    /** P12LM: initialise flags, DVMON counters and ask the crew to confirm TIG (V06N33). */
    public void p12Lm() {
        phasChng(PHASE_04024);
        // BANKCALL R02BOTH: check the status of the IMU.
        // UPFLAG MUNFLAG; UPFLAG ACC4-2FL (4-jet translation); UPFLAG R10FLAG (no R10 outputs).
        agc.set("MUNFLAG", 1);
        agc.set("ACC4-2FL", 1);
        agc.set("R10FLAG", 1);
        // CLRADMOD: initialise RADMODES for R29.
        // DOWNFLAG RNDVZFLG: clear rendezvous flag for P22.
        agc.set("RNDVZFLG", 0);
        agc.set("DVTHRUSH", agc.get("THRESH2"));   // initialise DVMON
        agc.set("DVCNTR", 4);
        agc.set("TRKMKCNT", 0);                    // R29 downlink data not ready
        Response r;
        do {
            r = goFlash("V06N33A");                // flash TIG
            if (r == Response.TERMINATE) {
                goToPooh();
                return;
            }
        } while (r == Response.ENTER);             // ENTER -> TCF -5 (re-flash)
        phasChng(PHASE_04024);
        guidInit();                                // initialise WM and /LAND/
        agc.set("FLPI", 1);                        // SET FLPI
        p12Init();
        p12LmB();
    }

    /** P12LMB: rotate state vectors to ignition time and initialise guidance parameters. */
    public void p12LmB() {
        // TGO = (TGO)A; TDEC1 = TIG; CALL LEMPREC (rotate state vectors to TIG).
        agc.set("TGO", agc.get("(TGO)A"));
        agc.set("TDEC1", agc.get("TIG"));
        // V1S = VATT * REFSMMAT * 2: velocity at TIG, scaled 2(-7) m/cs.
        // R   = RATT * REFSMMAT * 2^6: position at TIG, scaled 2(-24) m.
        // CALL MUNGRAV: GDT1/2(TIG) scaled 2(-7) m/cs.
        // UNIT/R/ = UNIT(R); CALL YCOMP.
        // XRANGE = DCOMP(YCOMP result) >> 5, initialised for noun 76.
        // ZDOTD = VINJNOM; RDOTD = RDOTDNOM.
        phasChng(PHASE_04024);
        newLoad();
    }

    /** NEWLOAD: flash cross-range and apolune values (V06N76). */
    public void newLoad() {
        Response r;
        do {
            r = goFlash("V06N76");
            if (r == Response.TERMINATE) {
                goToPooh();
                return;
            }
        } while (r == Response.ENTER);             // ENTER NEW DATA -> NEWLOAD
        agc.set("WHICH", agc.get("P12ADRES"));     // P12ADRES = REMADR P12TABLE
        phasChng(PHASE_04024);
        // YCO = (XRANGE << 5) + Y
        // V   = V1S + 49FPS * UNIT/R/        (V(TIPOVER) = V(IGN) + 57 fps UNIT/R/)
        // RDOT = (V . UNIT/R/) << 1          (RDOT * 2(-7))
        // ZAXIS1 = UNIT(UNIT/R/ x QAXIS)
        agc.set("FLVR", 1);                        // SETGO FLVR, ASCENT
        ascent();
    }

    /**
     * P12RET: compute YAW and PITCH from the thrust attitude.
     * YAW = ARCSIN( ATY / sqrt(ATP^2 + ATY^2) ) (zero when the radicand is zero);
     * PITCH = -ARCCOS( UNFC/2 . UNIT/R/ << 1 ).
     */
    public void p12Ret() {
        double atp = s(agc.get("ATP"));
        double aty = s(agc.get("ATY"));
        double rad = atp * atp + aty * aty;
        double yaw = rad == 0 ? 0 : Math.asin(aty / Math.sqrt(rad));
        agc.set("YAW", Agc.fromSigned((int) Math.round(yaw / Math.PI * 16384)));
        // PITCH = DCOMP( ARCCOS( UNIT(UNFC/2) . UNIT/R/ << 1 ) )
        phasChng(PHASE_04024);
        agc.set("FLPI", 0);                        // DOWNFLAG FLPI
        // INHINT; IBNKCALL PFLITEDB; RELINT
        // POSTJUMP BURNBABY
    }

    /** P12INIT: initialise engine data (used for P12 and P71). */
    public void p12Init() {
        agc.set("1/DV3", agc.get("(1/DV)A"));
        agc.set("1/DV2", agc.get("(1/DV)A"));
        agc.set("1/DV1", agc.get("(1/DV)A"));
        agc.set("AT", agc.get("(AT)A"));
        agc.set("TBUP", agc.get("(TBUP)A"));
        // TTO = -ATDECAY << 11
        agc.set("TTO", Agc.fromSigned(-(s(agc.get("ATDECAY")) << 11)));
        // VE = -APSVEX >> 2
        agc.set("VE", Agc.fromSigned(-(s(agc.get("APSVEX")) >> 2)));
        if (agc.get("FLAP") == 0) {                // BOFF RVQ FLAP COMMINIT
            return;                                // RVQ
        }
        commInit();
    }

    /** COMMINIT: initialise target data (P12, P70, and P71 if it does not follow P70). */
    public void commInit() {
        // RCO = HINJECT + /LAND/
        agc.set("RCO", Agc.add(agc.get("HINJECT"), agc.get("/LAND/")));
        agc.set("TXO", agc.get("HI6ZEROS"));
        agc.set("YCO", agc.get("HI6ZEROS"));
        agc.set("RDOTD", agc.get("HI6ZEROS"));
        agc.set("YDOTD", agc.get("HI6ZEROS"));
        // QAXIS = UNIT( (VRECTCSM x RRECTCSM) * REFSMMAT )
    }

    /** GUIDINIT: initialise WM (moon rate in stable member) and /LAND/ (landing site radius). */
    public void guidInit() {
        // TEMPR60 = Q; PUSH UNITZ; RTB LOADTIME; CALL RP-TO-R (landing site position in RLS)
        // WM = (UNITZ rotated by REFSMMAT) * MOONRATE
        // /LAND/ = |RLS| << 3
        // STCALL /LAND/ ; return through TEMPR60
    }

    // ------------------------------------------------------------ helper calls

    private void phasChng(int code) { /* TC PHASCHNG; OCT code */ }

    private Response goFlash(String verbNoun) {
        // CAF verbNoun; TC BANKCALL; CADR GOFLASH  -> terminate / proceed / enter returns
        return Response.PROCEED;
    }

    private void goToPooh() { /* TCF GOTOPOOH */ }

    private void ascent() { /* ASCENT (P12 ascent guidance continues) */ }
}
