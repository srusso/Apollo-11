package apollo.luminary099;

import apollo.agc.Agc;

/**
 * STABLE_ORBIT (Luminary 099): Stable Orbit Rendezvous programs P38/P78 (SOI and SOR maneuvers)
 * and Stable Orbit Midcourse programs P39/P79. Bank 04 at STBLEORB, EBANK = SUBEXIT.
 *
 * P38/P78 compute the delta V and initial conditions to (1) place the active vehicle on a transfer
 * trajectory intercepting the passive vehicle orbit DELTAR ahead of/behind the passive vehicle (SOI),
 * and (2) place the active vehicle in the passive orbit with a DELTAR separation (SOR).
 * Input: TIG, CENTANG, DELTAR, OPTION (1 = SOI, 2 = SOR), DELTTIME and TINT saved from the SOI phase.
 * Output: TRKMKCNT, TTOGO, +MGA, DSPTEM1 (intercept time, SOI), POSTTPI, DELVTPI, DELVTPF, DELVLVC.
 * Display-response convention for flashing displays: 0 = TERMINATE, 1 = PROCEED, 2 = RECYCLE.
 */
public final class StableOrbit {
    private final Agc agc;

    public static final String V06N33SR = "V06N33"; // TIG
    public static final String V06N55SR = "V06N55"; // CENTANG
    public static final String V04N06SR = "V04N06"; // option code: 1 = SOI, 2 = SOR
    public static final String V06N57SR = "V06N57"; // DELTA R
    public static final String V06N34SR = "V06N34"; // time of intercept
    public static final String V06N58SR = "V06N58"; // HP, DELTA V, DELTA V (final)
    public static final String V06N81SR = "V06N81"; // DELTA V (LV)
    public static final int DECTWO = 2;

    /** Return address emulation for the interpretive subroutines (RTRN). */
    private Runnable rtrn;

    public StableOrbit(Agc agc) {
        this.agc = agc;
    }

    private int flash(String vn) {
        return new PinballGameButtonsAndLights(agc).goflashr(vn);
    }

    private void gotopooh() {
        new FreshStartAndRestart(agc).goToPooh();
    }

    private void endofjob() {
        new Executive(agc).endOfJob();
    }

    private void blanket(int mask) {
        new DisplayInterfaceRoutines(agc).blanket(mask);
    }

    private void flag(String name, boolean on) {
        agc.set(name, on ? 1 : 0);
    }

    private boolean flag(String name) {
        return agc.get(name) != 0;
    }

    /** P38: this vehicle is the active vehicle. */
    public void p38() {
        new P32P35P72P75(agc).avflaga();
        p3878(true);
    }

    /** P78: other vehicle is the active vehicle. */
    public void p78() {
        new P32P35P72P75(agc).avflagp();
        p3878(false);
    }

    private void p3878(boolean thisVehicle) {
        new P32P35P72P75(agc).p20flgon();       // set UPDATFLG, TRACKFLG
        agc.set("NN", DECTWO);
        vndsply(V06N33SR);                    // display TIG
        // display CENTANG (R3 may be loaded); immediate return blanks R1, R2
        int r;
        while ((r = flash(V06N55SR)) == 2) { /* RECYCLE */ }
        if (r == 0) { gotopooh(); return; }
        if (r == 3) { blanket(3); endofjob(); return; }
        agc.set("OPTION1", 5);
        agc.set("OPTION2", 1);                // option code = 1
        while ((r = flash(V04N06SR)) == 2) { /* RECYCLE */ }
        if (r == 0) { gotopooh(); return; }
        if (r == 3) { blanket(0x4); endofjob(); return; }    // BIT3: blank R3
        // INTERPRETIVE: SLOAD OPTION2 ; SR1 ; BHIZ OPTN1
        if ((Agc.toSigned(agc.get("OPTION2")) >> 1) == 0) {
            optn1();
        } else {
            // SOR: save TINT for SOR phase, clear OPTNSW (ON = SOI, OFF = SOR)
            VecOps.setDp(agc, "TINTSOI", VecOps.getDp(agc, "TINT"));
            flag("OPTNSW", false);
            junctn1();
        }
    }

    /** OPTN1: SOI. */
    private void optn1() {
        flag("OPTNSW", true);
        flag("UPDATFLG", false);
        double tf = precTt();                 // CALL PREC/TT -> transfer time T
        flag("UPDATFLG", true);
        VecOps.setDp(agc, "TINT", VecOps.getDp(agc, "TIG") + tf);   // TI = TIG + TF
        int r;
        while ((r = flash(V06N57SR)) == 2) { /* RECYCLE: display DELTA R */ }
        if (r == 0) { gotopooh(); return; }
        if (r == 3) { blanket(6); endofjob(); return; }
        // +5: DSPTEM1 = TINT for display of intercept time
        VecOps.setDpRaw(agc, "DSPTEM1", VecOps.getDpRaw(agc, "TINT"));
        vndsply(V06N34SR);
        junctn1();
    }

    /** JUNCTN1: clear P39/79SW, select MU (also clears FINALFLG, goes to VN1645), then RECYCLE. */
    private void junctn1() {
        flag("P39/79SW", false);
        new P3435P7475(agc).selectmu();
        recycle();
    }

    /** RECYCLE: compute the target time TARGTIME then fall into MAINRTNE. */
    private void recycle() {
        double t = precTt();                                    // CALL PREC/TT
        if (!flag("OPTNSW")) {
            // OPTN2 (SOR): TI = TINTSOI + T
            VecOps.setDp(agc, "TINT", VecOps.getDp(agc, "TINTSOI") + t);
        } else {
            // precision update passive vehicle to intercept time
            VecOps.setDp(agc, "TDEC1", VecOps.getDp(agc, "TINT"));
            intrpvp();
            double[] ratt = VecOps.getV(agc, "RATT");
            double[] vatt = VecOps.getV(agc, "VATT");
            double[] ur = VecOps.unit(ratt);
            double vxr = VecOps.abs(VecOps.cross(vatt, ur));
            // DELTTIME = (RP) DELTA R / (VP X RP) : DELTA T
            double deltaT = vxr == 0 ? 0 : VecOps.getDp(agc, "DELTAR") / vxr;
            VecOps.setDp(agc, "DELTTIME", deltaT);
        }
        // JUNCTN2: TT = TI - DELTA T
        VecOps.setDp(agc, "TARGTIME", VecOps.getDp(agc, "TINT") - VecOps.getDp(agc, "DELTTIME"));
        mainrtne();
    }

    /**
     * MAINRTNE: precision-update the passive vehicle to the target time, compute the delta V
     * (S3435.25), and display results (V06N58 for P38/78, V06N81 for all), then loop to RECYCLE
     * (P38/78) or to P39/P79B (P39/79).
     */
    public void mainrtne() {
        VecOps.setDp(agc, "TDEC1", VecOps.getDp(agc, "TARGTIME"));
        intrpvp();                                              // precision update passive vehicle
        VecOps.setDpRaw(agc, "INTIME", VecOps.getDpRaw(agc, "TIG"));
        agc.set("SUBEXIT", 0);                                  // SSP SUBEXIT TEST3979
        new P3435P7475(agc).s343525(VecOps.getV(agc, "RATT"));   // CALL S3435.25
        // TEST3979
        if (flag("P39/79SW")) {
            if (!flag("FINALFLG")) {
                flag("UPDATFLG", true);
            }
            dsply81();                                          // P39P79: EXIT, DSPLY81
            return;
        }
        // MAINRTN1
        double[] delveet3 = VecOps.getV(agc, "DELVEET3");
        VecOps.setDp(agc, "DELVTPI", VecOps.abs(delveet3));      // DELTA V
        double[] vpass4 = VecOps.getV(agc, "VPASS4");
        double[] vtprime = VecOps.getV(agc, "VTPRIME");
        VecOps.setDp(agc, "DELVTPF", VecOps.abs(VecOps.sub(vpass4, vtprime)));  // DELTA V (FINAL) = V'T - VT
        // PERIAPO1 on (RACT3, VIPRIME) -> perigee altitude; SHIFTR1 -> POSTTPI
        P3435P7475 p3435 = new P3435P7475(agc);
        double[] apsides = p3435.periapo1(VecOps.getV(agc, "RACT3"), VecOps.getV(agc, "VIPRIME"));
        VecOps.setDp(agc, "POSTTPI", p3435.shiftr1(apsides[3]));   // perigee altitude
        if (!flag("FINALFLG")) {
            flag("UPDATFLG", true);
        }
        vndsply(V06N58SR);                                       // DSPLY58: HP, DELTA V, DELTA V (FINAL)
        dsply81();
    }

    /** DSPLY81: display DELTA V (LV), then DELVSIN = DELVEET3 and VN1645 (TRKMKCNT, TTOGO, +MGA). */
    private void dsply81() {
        vndsply(V06N81SR);
        flag("XDELVFLG", false);
        VecOps.setV(agc, "DELVSIN", VecOps.getV(agc, "DELVEET3"));
        new P3435P7475(agc).vn1645();
        if (flag("P39/79SW")) {
            p39p79b();
        } else {
            recycle();
        }
    }

    // ---------------------------------------------------------------- P39 / P79

    /** P39: this vehicle active; preparation time from ATIGINC. */
    public void p39() {
        new P32P35P72P75(agc).avflaga();
        p39p79a(VecOps.getDpRaw(agc, "ATIGINC"));
    }

    /** P79: other vehicle active; preparation time from PTIGINC. */
    public void p79() {
        new P32P35P72P75(agc).avflagp();
        p39p79a(VecOps.getDpRaw(agc, "PTIGINC"));
    }

    /** P39/P79A: KT = time to prepare for burn; set UPDATFLG/TRACKFLG, select MU, enter P39/P79B. */
    private void p39p79a(long ktRaw) {
        VecOps.setDpRaw(agc, "KT", ktRaw);
        new P32P35P72P75(agc).p20flgon();
        flag("P39/79SW", true);
        new P3435P7475(agc).selectmu();       // also clears FINALFLG and goes to VN1645
        p39p79b();
    }

    /** P39/P79B: TIG = present time + preparation time; update vehicles to TIG; MAINRTNE at TARGTIME. */
    private void p39p79b() {
        new RtbOpCodes(agc).loadtime();                         // RTB LOADTIME
        double loadtime = VecOps.getDp(agc, "MPAC");
        VecOps.setDp(agc, "TIG", loadtime + VecOps.getDp(agc, "KT"));
        VecOps.setDp(agc, "TDEC1", VecOps.getDp(agc, "TIG"));
        new P3435P7475(agc).precset();                           // precision update active and passive
        new P3435P7475(agc).s3435p1();                           // get unit normal
        mainrtne();                                              // with TARGTIME
    }

    // ---------------------------------------------------------------- subroutines

    /**
     * PREC/TT: precision update active and passive vehicles to TIG, then compute the transfer
     * time from CENTANG of the passive vehicle (TIMETHET). Returns T.
     */
    private double precTt() {
        VecOps.setDp(agc, "TDEC1", VecOps.getDp(agc, "TIG"));
        new P3435P7475(agc).precset();
        double[] rvec = VecOps.getV(agc, "RPASS3");              // RVEC = RPASS3 shifted by 0,2
        double[] vvec = VecOps.getV(agc, "VPASS3");              // VVEC = VPASS3 shifted
        double centang = VecOps.getDp(agc, "CENTANG");
        agc.set("CSTH", 0);
        VecOps.setDp(agc, "CSTH", Math.cos(centang * 2 * Math.PI));
        VecOps.setDp(agc, "SNTH", Math.sin(centang * 2 * Math.PI));
        flag("RVSW", true);
        VecOps.setV(agc, "RVEC", rvec);
        VecOps.setV(agc, "VVEC", vvec);
        new ConicSubroutines(agc).timethet();                          // transfer time from CENTANG
        new P3435P7475(agc).s3435p1();                           // get unit normal
        return VecOps.getDp(agc, "T");
    }

    /** INTRPVP: precision update the passive vehicle to TDEC1 (CSMPREC or LEMPREC). */
    private void intrpvp() {
        if (flag("AVFLAG")) {
            new IntegrationInitialization(agc).csmprec();   // LEM is active vehicle: passive is the CSM... see AVFLAG
        } else {
            new IntegrationInitialization(agc).lemprec();   // OTHERV
        }
    }

    /** VNDSPLY: flash a verb/noun display; TERMINATE exits via GOTOPOOH, PROCEED returns, RECYCLE redisplays. */
    private void vndsply(String vn) {
        agc.set("VERBNOUN", 0);
        int r;
        while ((r = flash2(vn)) == 2) { /* RECYCLE */ }
        if (r == 0) {
            gotopooh();
        }
    }

    private int flash2(String vn) {
        return new PinballGameButtonsAndLights(agc).goflash(vn);
    }
}
