package apollo.luminary099;

import apollo.agc.Agc;

/**
 * P30_P37 (Luminary 099, bank 32, P30S): External Delta-V targeting (P30) and the S30.1 helper.
 */
public class P30P37 {
    /** V06N33: display time of ignition. */
    public static final String V06N33 = "0633";
    /** V06N42: display apogee, perigee, delta V. */
    public static final String V06N42 = "0642";
    /** V06N81: display delta V (LV). */
    public static final String V06N81 = "0681";

    private final Agc agc;

    public P30P37(Agc agc) {
        this.agc = agc;
    }

    /** P30: set update and track flags, then run the display sequence. */
    public void p30() {
        upflag("UPDATFLG");
        upflag("TRACKFLG");
        p30n33();
    }

    /** P30N33: display TIG (N33) then delta V (N81); recycle redisplays, terminate goes to POOH. */
    public void p30n33() {
        vnpooh(V06N33);
        vnpooh(V06N81);
        downflag("UPDATFLG");
        s301();
        upflag("UPDATFLG");
        param30();
    }

    /** PARAM30: display apogee, perigee, delta V; continue to P40's external delta-V guidance. */
    public void param30() {
        vnpooh(V06N42);
        upflag("XDELVFLG");     // SETGO XDELVFLG: for P40's external delta-V guidance
        // GOTO REVN1645: TRKMKCNT, T60, +MGA display
        new P30P37Hooks().revn1645();
    }

    /**
     * S30.1: integrate to ignition and compute delta V in inertial coordinates, plus HPER and HAPO
     * (rescaled and limited to 9999.9 n.mi.).
     */
    public void s301() {
        agc.set("TDEC1", agc.get("TIG"));    // TIG scaled 2(+28) cs
        // CALL LEMPREC: Encke routine for LEM -> RATT, VATT
        // RTIG = RATT (radius vector at ignition); VTIG = VATT.
        // DELVSIN = ZRF/LV (unit -RTIG), YRF/LV = unit(VTIG x RTIG) ...
        // DELVSIN (inertial) = M * DELVSLV  (scaled B+7 M/CS); DELVSAB = |DELVSIN|
        // CALL PERIAPO1 with VREQUIRED = VTIG + DELVSIN; HPER, HAPO = shifted & MAXCHK-limited.
        // The vector arithmetic runs in the interpreter and is left to the orbital-math classes.
        agc.set("QTEMP", agc.get("QTEMP"));
    }

    private void upflag(String flag) { agc.set(flag, 1); }
    private void downflag(String flag) { agc.set(flag, 0); }

    private void vnpooh(String verbNoun) {
        // TC VNPOOH: flashing display; returns on PROCEED, to POOH on TERMINATE
    }

    /** Placeholder for the cross-file label REVN1645 (see Pinball/Extended verbs). */
    private static final class P30P37Hooks {
        void revn1645() { }
    }
}
