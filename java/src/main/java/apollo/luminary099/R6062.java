package apollo.luminary099;

import apollo.agc.Agc;

/**
 * R60_62 (Luminary 099): R60 (attitude maneuver routine), BALLANGS, VECPOINT and R62.
 *
 * R60LEM maneuvers the LM to a specified attitude: computes final CDU angles (VECPOINT) unless the
 * 3-axis flag is set, computes FDAI ball angles (BALLANGS), flashes V50 N18 "please perform auto
 * maneuver", and on PROCEED with G&amp;N control and AUTO mode drives the maneuver via GOMANUR.
 * Calling sequence: TC BANKCALL / CADR R60LEM. Inputs: SCAXIS, POINTVSM (for VECPOINT), 3AXISFLG.
 * Debris: CPHI, CTHETA, CPSI, 3AXISFLG, TBASE2.
 *
 * Display return convention assumed for GOFLASH-type calls: 0 = TERMINATE, 1 = PROCEED, 2 = ENTER.
 */
public final class R6062 {
    private final Agc agc;

    public static final String V06N18 = "V06N18";
    public static final String V06N22 = "V06N22";
    public static final int BIT14_7 = 020100;
    public static final int OCT203 = 0203;

    /** SIN(59 deg) scaled $2. */
    public static final double SINGIMLC = 0.4285836003;
    /** SIN(49.4 deg) scaled $2. */
    public static final double SINVEC1 = 0.3796356537;
    /** SIN(29.5 deg) scaled $2. */
    public static final double SINVEC2 = 0.2462117800;
    /** 50 degrees, scaled $360. */
    public static final double VECANG1 = 0.1388888889;
    /** 35 degrees, scaled $360. */
    public static final double VECANG2 = 0.09722222222;
    /** DPB-14: smallest valid cross-product magnitude (2^-14). */
    public static final double DPB_14 = 1.0 / 16384.0;

    public R6062(Agc agc) {
        this.agc = agc;
    }

    private boolean flag(String name) {
        return agc.get(name) != 0;
    }

    /** R60LEM: save caller's return address, then fall into REDOMANN. */
    public void r60lem() {
        agc.set("TEMPR60", agc.get("Q")); // TC MAKECADR ; TS TEMPR60
        redomann();
    }

    /** REDOMANN: if 3AXISFLG not set compute final angles by VECPOINT -> CPHI, then TOBALL. */
    public void redomann() {
        if (!flag("3AXISFLG")) {
            storeAngles(vecpoint());
        }
        toball();
    }

    private void storeAngles(int[] cdu) {
        agc.set("CPHI", cdu[0]);
        agc.set("CTHETA", cdu[1]);
        agc.set("CPSI", cdu[2]);
    }

    /** TOBALL: convert angles to FDAI ball angles, then TOBALLA. */
    public void toball() {
        new R6062(agc).ballangs();
        toballa();
    }

    /** TOBALLA: display V06N18 (PLEASE PERFORM AUTO MANEUVER) and branch on the response. */
    public void toballa() {
        int response = new PinballGameButtonsAndLights(agc).goperf2r(V06N18);
        if (response == 0) {
            r61test();              // TERMINATE
        } else if (response == 1) {
            redomanc();             // PROCEED
        } else {
            endmanu1();             // ENTER: finished with R60
        }
    }

    /** REDOMANC: recompute final angles and ball angles; maneuver only if G+N and AUTO. */
    public void redomanc() {
        if (!flag("3AXISFLG")) {
            storeAngles(vecpoint());
        }
        ballangs();                 // TOBALLC
        if (gnAuto() > 0) {
            toballa();              // not in G+N / AUTO: re-request auto maneuver
            return;
        }
        automanv();
    }

    /** AUTOMANV: static display during auto maneuver, then start the maneuver via KALCMANU. */
    public void automanv() {
        new PinballGameButtonsAndLights(agc).godspr(V06N18);
        chklinus();
        startmnv();
    }

    /** STARTMNV: perform maneuver via GOMANUR; ENDMANUV loops back to TOBALLA. */
    public void startmnv() {
        new ImuModeSwitchingRoutines(agc).goManur();
        toballa();                  // ENDMANUV
    }

    /** ENDMANU1: reset the 3-axis flag and return to the caller (TEMPR60). */
    public void endmanu1() {
        agc.set("3AXISFLG", 0);     // TC DOWNFLAG
        // CAE TEMPR60 ; TC BANKJUMP -> return to caller
    }

    /**
     * CHKLINUS: if the priority display flag (PDSPFLAG) is set, do a phase change and request the
     * LINUS priority display setup; otherwise return immediately.
     */
    public void chklinus() {
        if (!flag("PDSPFLAG")) {
            return;
        }
        int q = agc.get("Q");
        agc.set("MPAC+2", q);
        agc.set("TBASE2", Agc.add(agc.get("BUF2"), Agc.fromSigned(-3)));  // restart location
        // TC PHASCHNG ; OCT 00132
        new PhaseTableMaintenance(agc).phasChng(0132);
        // CAF BIT7 ; TC LINUS : set bits for priority display
        new PinballGameButtonsAndLights(agc).linus(0100);
    }

    /** RELINUS: restore priority and decide how to resume after a priority display. */
    public void relinus() {
        new Executive(agc).prioChng(agc.get("PRIO26")); // restore original priority
        if (flag("TRACKFLG")) {
            rer60();                // continue R60 only if TRACKFLG on
            return;
        }
        if (flag("RNDVZFLG")) {
            // P20: set 2.7 spot
            new PhaseTableMaintenance(agc).phasChng(040072);
        } else {
            // must be P25: set 2.11 spot
            new PhaseTableMaintenance(agc).phasChng(040112);
        }
        new Executive(agc).endOfJob();
    }

    /** RER60: set priority display flag after restart and go back to TBASE2. */
    public void rer60() {
        agc.set("PDSPFLAG", 1);     // TC UPFLAG
        // TC TBASE2
    }

    /** R61TEST: terminate handling. In P00 (MODREG = 0) it is V49/V89, so end via ENDMANU1. */
    public void r61test() {
        if (agc.get("MODREG") == 0) {
            endmanu1();
        } else if (!flag("PDSPFLAG")) {
            new FreshStartAndRestart(agc).goToPooh();
        } else {
            new P20P25(agc).gotov56();   // in R61 (P20 or P25)
        }
    }

    /**
     * G+N,AUTO: check for G&amp;N control and AUTO stabilization.
     * Returns positive if not set for G+N/AUTO, +0 if the switches are set.
     */
    public int gnAuto() {
        int chan30 = agc.get("CHAN30");
        if ((chan30 & 0x200) != 0) {   // BIT10: not in G+N
            return 1;
        }
        return isitauto();
    }

    /** ISITAUTO: channel 31 bit 14: (+) = not in AUTO, (+0) = OK. */
    public int isitauto() {
        return (agc.get("CHAN31") & 0x2000) != 0 ? 1 : 0;
    }

    /**
     * BALLANGS: compute LM FDAI ball display angles (roll, pitch, yaw) from the gimbal angles
     * CPHI (outer), CTHETA (inner), CPSI (middle). Angles are SP 2's complement, half-revolution scale.
     * Outputs FDAIX, FDAIY, FDAIZ.
     */
    public void ballangs() {
        agc.set("BALLEXIT", agc.get("Q"));
        double x = angle(agc.get("CPHI"));     // CDUSPOT+4 -> CDUX (outer)
        double y = angle(agc.get("CTHETA"));   // CDUSPOT   -> CDUY (inner)
        double z = angle(agc.get("CPSI"));     // CDUSPOT+2 -> CDUZ (middle)
        double sx = Math.sin(x), cx = Math.cos(x);
        double sy = Math.sin(y), cy = Math.cos(y);
        double sz = Math.sin(z), cz = Math.cos(z);
        // YAW = ARCSIN(-SXCZ)
        double yaw = Math.asin(-sx * cz);
        // ROLL = ARCTAN(SZ / CZCX)
        double roll = Math.atan2(sz, cz * cx);
        // PITCH = ARCTAN(CXSY + SXSZCY, CXCY - SXSZSY)
        double pitch = Math.atan2(cx * sy + sx * sz * cy, cx * cy - sx * sz * sy);
        agc.set("FDAIX", toHalfRev(roll));
        agc.set("FDAIY", toHalfRev(pitch));
        agc.set("FDAIZ", toHalfRev(yaw));
        // ENDBALL: CA BALLEXIT ; TC BANKJUMP
    }

    private static double angle(int word) {
        return Agc.toSigned(word) / 16384.0 * Math.PI;   // half-revolution scale
    }

    private static int toHalfRev(double radians) {
        return Agc.fromSigned((int) Math.round(radians / Math.PI * 16384.0));
    }

    /**
     * VECPOINT: point a spacecraft axis (SCAXIS, half-unit DP vector in S/C coordinates) in a
     * desired direction (POINTVSM, half-unit DP vector in stable member coordinates).
     * Computes the cross-product rotation and, if that would bring the vehicle into gimbal lock
     * (|sin(middle gimbal)| &gt;= sin 59 deg), rotates about the pointed axis by 50 deg (AOT) or
     * 35 deg (transponder / Y-Z plane vectors) to leave gimbal lock.
     * Returns desired outer, inner, middle gimbal angles (2's complement, MPAC, MPAC+1, MPAC+2).
     * VECPNT1 is the same but uses desired CDU angles (MPAC) instead of present ones.
     */
    public int[] vecpoint() {
        double[][] mis = VecOps.getM(agc, "MIS");      // S/C axes to stable member axes (from CDUTODCM)
        double[] scaxis = VecOps.getV(agc, "SCAXIS");
        double[] vf = VecOps.unit(VecOps.mxv(transpose(mis), VecOps.getV(agc, "POINTVSM")));
        double[] cof = VecOps.unit(VecOps.cross(vf, scaxis));
        double cmag = VecOps.abs(VecOps.cross(vf, scaxis));
        if (cmag < DPB_14) {
            return pickaxis(vf, scaxis);
        }
        double angle = Math.acos(Math.max(-1, Math.min(1, VecOps.dot(scaxis, vf) * 2)));
        double[][] mfs = compmatx(mis, cof, angle);
        if (Math.abs(mfs[0][2]) >= SINGIMLC / 2
                && Math.abs(scaxis[0]) >= SINVEC1 / 2) {
            // Pointing a vector that would enter gimbal lock: rotate about +-SCAXIS
            double[] ig = mfs[1];
            double[] u = VecOps.dot(VecOps.cross(ig, scaxis), new double[] {1, 0, 0}) < 0
                    ? scaxis : VecOps.scale(scaxis, -1);
            double extra = Math.abs(scaxis[0]) < SINVEC2 / 2 ? VECANG1 : VECANG2;
            mfs = compmatx(mfs, u, extra * 2 * Math.PI);
        }
        return findgimb(mfs);
    }

    /** COMPMATX: apply a rotation of the given angle about axis (DELCOMP) to a matrix (MXM3). */
    private double[][] compmatx(double[][] m, double[] axis, double angle) {
        double c = Math.cos(angle), s = Math.sin(angle), t = 1 - c;
        double[] u = VecOps.unit(axis);
        double[][] del = {
            {c + u[0] * u[0] * t, u[0] * u[1] * t - u[2] * s, u[0] * u[2] * t + u[1] * s},
            {u[1] * u[0] * t + u[2] * s, c + u[1] * u[1] * t, u[1] * u[2] * t - u[0] * s},
            {u[2] * u[0] * t - u[1] * s, u[2] * u[1] * t + u[0] * s, c + u[2] * u[2] * t}};
        double[][] r = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    r[i][j] += m[i][k] * del[k][j];
                }
            }
        }
        return r;
    }

    private static double[][] transpose(double[][] m) {
        double[][] t = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                t[i][j] = m[j][i];
            }
        }
        return t;
    }

    /** FINDGIMB: extract the commanded CDU angles from the matrix (DCMTOCDU), convert to 2's complement. */
    private int[] findgimb(double[][] m) {
        double middle = Math.asin(Math.max(-1, Math.min(1, m[0][2])));
        double inner = Math.atan2(-m[2][0], m[0][0]);
        double outer = Math.atan2(-m[1][2], m[1][1]);
        return new int[] {
            Agc.fromSigned((int) Math.round(outer / (2 * Math.PI) * 16384)),
            Agc.fromSigned((int) Math.round(inner / (2 * Math.PI) * 16384)),
            Agc.fromSigned((int) Math.round(middle / (2 * Math.PI) * 16384))};
    }

    /**
     * PICKAXIS: VF x VI = 0. If VF = VI the desired CDU equals present CDU; if antiparallel
     * (ROT180) a 180 degree rotation about a vector perpendicular to VI is used.
     */
    private int[] pickaxis(double[] vf, double[] scaxis) {
        double[][] mis = VecOps.getM(agc, "MIS");
        if (VecOps.dot(vf, scaxis) >= 0) {
            return new int[] {agc.get("25D"), agc.get("26D"), agc.get("27D")};  // present CDU angles
        }
        return rot180(mis, scaxis);
    }

    /** ROT180: pick a vector in the plane of X(I), Y(SM) perpendicular to VI (else the X axis) and rotate 180 deg. */
    private int[] rot180(double[][] mis, double[] scaxis) {
        double[] ysm = mis[1];
        double[] cof = VecOps.cross(VecOps.unit(VecOps.cross(ysm, new double[] {0.5, 0, 0})), scaxis);
        if (VecOps.abs(cof) < DPB_14) {
            cof = new double[] {0.5, 0, 0};             // PICKX
        }
        return findgimb(compmatx(mis, VecOps.unit(cof), Math.PI));  // XROT -> COMPMATX
    }

    // ---------------------------------------------------------------- R62

    /** R62FLASH (R62DISP): flash V06N22 ICDU angles; astronaut may load new ICDUs. */
    public void r62flash() {
        int response;
        do {
            response = new PinballGameButtonsAndLights(agc).goflash(V06N22);
            if (response == 0) {
                new ExtendedVerbs(agc).endext();      // TERMINATE
                return;
            }
        } while (response != 1);                         // ENTER: redisplay
        gomove();
    }

    /** GOMOVE: set 3-axis flag, run R60, then end R62. */
    public void gomove() {
        agc.set("3AXISFLG", 1);
        r60lem();
        new ExtendedVerbs(agc).endext();
    }
}
