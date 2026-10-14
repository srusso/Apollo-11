package apollo.luminary099;

import apollo.agc.Agc;

/**
 * R63.agc - LM rendezvous-radar/optical pointing routine: points the LM +Z or +X axis
 * at the CSM (V89 "ROUTINE 63" ), computes the required gimbal angles and calls R60LEM.
 */
public class R63 {
    public static final int VB04N12 = 4 * 100 + 12; // verb 04 noun 12
    public static final int VB06N18 = 6 * 100 + 18; // verb 06 noun 18
    /** DP1MIN: 2DEC 6000 (one minute in centiseconds, DP). */
    public static final int DP1MIN = 6000;

    private final Agc agc;

    public R63(Agc agc) {
        this.agc = agc;
    }

    /** V89CALL: display V04N12 (option code request) with option 3 / 1, then V89RECL. */
    public void v89Call() {
        // BANKCALL R02BOTH
        agc.set("OPTIONX", 3);
        agc.set("OPTIONX+1", 1);
        // V04N12 flash; on terminate go to ENDEXT; on proceed fall through; on enter-data loop back
        v89Recl();
    }

    /** V89RECL: compute CSM and LEM positions at the time, and the pointing vector in the SM frame. */
    public void v89Recl() {
        // RTB LOADTIME ; DAD DP1MIN ; STORE TSTART82 ; STCALL TDEC1 ; CSMCONIC
        long tstart82 = (long) agc.get("LOADTIME") + DP1MIN; // time one minute ahead (DP)
        agc.set("TSTART82", (int) tstart82);
        agc.set("TDEC1", (int) tstart82);
        // CALL CSMCONIC ; RONE <- RATT
        agc.set("RONE", agc.get("RATT"));
        agc.set("TDEC1", agc.get("TSTART82"));
        // CALL LEMCONIC ; POINTVSM <- unit( REFSMMAT * (RONE - RATT) )
        // vector math is performed by the interpreter (VSU, MXV, NORMUNIT)
        agc.set("POINTVSM", agc.get("RONE") - agc.get("RATT"));
        if (agc.get("OPTIONX+1") == 1) {
            alignX();   // ALINEX: align +X axis (UNITX)
        } else {
            alignZ();   // ALINEZ: align +Z axis (UNITZ)
        }
    }

    /** ALINEX: SCAXIS = UNITX. */
    public void alignX() {
        agc.set("SCAXIS", 0x2000);
        v89Call1();
    }

    /** ALINEZ: SCAXIS = UNITZ. */
    public void alignZ() {
        agc.set("SCAXIS", 0);
        v89Call1();
    }

    /** V89CALL1: STCALL SCAXIS, VECPOINT -> CPHI, then display V06N18 ball angles. */
    public void v89Call1() {
        // CALL VECPOINT ; STORE CPHI
        // BANKCALL BALLANGS
        // V06N18 flash: terminate -> ENDEXT; proceed -> continue; enter -> V89RECL (recompute)
        // DOWNFLAG 3AXISFLG
        // BANKCALL R60LEM ; TCF ENDEXT
    }
}
