package apollo.luminary099;

import apollo.agc.Agc;

/**
 * GENERAL_LAMBERT_AIMPOINT_GUIDANCE.agc (Luminary 1A build 099, pages 651-653).
 * Written by Rama M. Aiyawar. Program P31: accepts targeting parameters (TIG, RTARG, DELLT4)
 * from outside the LM and computes the required velocity and other initial conditions for the
 * maneuver. Applicable to Earth or lunar orbit.
 *
 * Displays: V06N33 (TIG), V06N42 (HAPO, HPER, VGDISP), V16N35 (time from TIG), V06N45 (time
 * from ignition and middle gimbal angle), V06N81 (DELVLVC).
 * Outputs: HAPO, HPER, VGDISP (B+7 m/cs Earth, B+5 m/cs Moon), MIDGIM, XDELVFLG reset.
 * Alarms: none. Restarts: group 4.
 */
public final class GeneralLambertAimpointGuidance {
    public static final String V06N33 = "V06N33";
    public static final String V06N42 = "V06N42";
    public static final String V06N81 = "V06N81";

    private final Agc agc;

    public GeneralLambertAimpointGuidance(Agc agc) {
        this.agc = agc;
    }

    /** Hook for external routines (VNPOOH, LEMPREC, INITVEL, PERIAPO1, SHIFTR1, MAXCHK, VN1645, ...). */
    protected void extern(String routine, Object... args) {
    }

    /** P31: general Lambert aimpoint guidance (EBANK = SUBEXIT, COUNT* $$/P31). */
    public void p31() {
        extern("p20FlgOn");
        extern("vnPooh", V06N33);                 // display stored TIG
        // --- interpretive section ---
        agc.set("UPDATFLG", 0);                   // CLEAR UPDATFLG
        agc.set("TDEC1", agc.get("TIG"));         // integrate state vectors to TIG
        extern("lemPrec");
        // RTIG = RATT; RINIT = RATT; VTIG = VATT; VINIT = VATT
        copyVec("RATT", "RTIG");
        copyVec("RATT", "RINIT");
        copyVec("VATT", "VTIG");
        copyVec("VATT", "VINIT");
        // PUSH P30ZERO (E4 and NUMIT = 0); PDDL DELLT4; DAD TIG -> TPASS4
        extern("pushDp", "P30ZERO");
        agc.set("TPASS4", Agc.add(agc.get("DELLT4"), agc.get("TIG")));   // DP add, low word elided
        extern("sxa1", "RTX1");
        extern("sxa2", "RTX2");
        extern("initVel");
        // DELVSIN = DELVEET3; VGDISP = |DELVEET3|; CLEAR XDELVFLG
        copyVec("DELVEET3", "DELVSIN");
        extern("abval", "DELVEET3");
        agc.set("XDELVFLG", 0);
        agc.set("VGDISP", agc.get("MPAC"));
        extern("getLvc");                         // GET.LVC
        // periapsis / apoapsis from RTIG and VIPRIME
        extern("periapo1");
        extern("shiftr1");
        extern("maxChk");                         // limit display to 9999.9 n.mi.
        agc.set("HPER", agc.get("MPAC"));
        extern("shiftr1");
        extern("maxChk");                         // limit display to 9999.9 n.mi.
        agc.set("HAPO", agc.get("MPAC"));
        // --- back to basic ---
        extern("vnPooh", V06N81);                 // DELVLVC
        extern("vnPooh", V06N42);                 // HAPO, HPER, VGDISP
        revn1645();
    }

    /** REVN1645: SET FINALFLG; CALL VN1645 (TRKMKCNT, TTOGO, +MGA); loop forever. */
    public void revn1645() {
        while (true) {
            agc.set("FINALFLG", 1);
            extern("vn1645");
            if (!loops()) {
                return;
            }
        }
    }

    /** Hook so the endless REVN1645 loop can be terminated by subclasses/tests. */
    protected boolean loops() {
        return false;
    }

    private void copyVec(String from, String to) {
        for (int i = 0; i < 6; i += 2) {
            agc.set(to + "+" + i, agc.get(from + "+" + i));
            agc.set(to + "+" + (i + 1), agc.get(from + "+" + (i + 1)));
        }
    }
}
