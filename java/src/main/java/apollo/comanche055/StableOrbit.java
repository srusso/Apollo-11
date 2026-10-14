package apollo.comanche055;

import apollo.agc.Agc;

/**
 * STABLE_ORBIT: P38 / P78 (Stable Orbit Insertion / Rendezvous) and P39 / P79
 * (Stable Orbit Insertion Maneuver planning). P38/P39 act on this vehicle; P78/P79 on the other.
 */
public class StableOrbit {
    public static final int V06N33SR = 0x0633; // time of ignition
    public static final int V06N55SR = 0x0655; // central angle
    public static final int V04N06SR = 0x0406; // option code: 1 = SOI, 2 = SOR
    public static final int V06N57SR = 0x0657; // delta R
    public static final int V06N34SR = 0x0634; // time of intercept
    public static final int V06N58SR = 0x0658; // HP, delta V, delta V (final)
    public static final int V06N81SR = 0x0681; // delta V (LV)

    private final Agc agc;

    public StableOrbit(Agc agc) {
        this.agc = agc;
    }

    /** P38: this vehicle active (AVFLAGA), then common code. */
    public void p38() {
        // TC AVFLAGA
        stableOrbitCommon();
    }

    /** P78: other vehicle active (AVFLAGP), then common code. */
    public void p78() {
        // TC AVFLAGP
        stableOrbitCommon();
    }

    private void stableOrbitCommon() {
        // TC P20FLGON: set UPDATFLG, TRACKFLG
        vndsply(V06N33SR);          // display TIG
        // V06N55SR flash (center angle); terminate -> GOTOPOOH, recycle -> repeat, proceed
        // immediate return (V); blank R1,R2 -> ENDOFJOB
        agc.set("OPTION1", 5);
        agc.set("OPTION2", 1);      // option code 1 = SOI, 2 = SOR
        // V04N06SR flash
        // INTPRET:
        agc.set("NN", 2);
        int option = Agc.toSigned(agc.get("OPTION2")) >> 1; // SLOAD SR1
        if (option == 0) {
            // BHIZ OPTN1: SOI
            optn1();
            return;
        }
        // SOR phase: TINTSOI <- TINT ; CLRGO OPTNSW -> JUNCTN1
        agc.set("TINTSOI", agc.get("TINT"));
        agc.set("OPTNSW", 0);
        junctn1();
    }

    private void optn1() {
        agc.set("OPTNSW", 1);       // SOI
        agc.set("UPDATFLG", 0);
        prectt();
        agc.set("UPDATFLG", 1);
        // TINT = TIG + TF (DAD TIG)
        int tint = Agc.add(agc.get("TIG"), agc.get("TF"));
        agc.set("TINT", tint);
        agc.set("DSPTEM1", tint);
        // V06N57SR flash: DELTA R ; then V06N34SR display: time of intercept
        vndsply(V06N57SR);
        vndsply(V06N34SR);
        junctn1();
    }

    private void junctn1() {
        agc.set("P39/79SW", 0);
        selectmu();                 // select MU, clear FINALFLG, go to VN1645
        recycle();
    }

    private void selectmu() {
        agc.set("FINALFLG", 0);
        // VN1645 display
    }

    /** RECYCLE: update passive vehicle to intercept time and compute delta T, then target time. */
    public void recycle() {
        prectt();
        if (agc.get("OPTNSW") != 0) {
            // SOI: precision update passive vehicle to TINT; DELTTIME = |RP| DELTAR / |VP x RP/|RP||
            agc.set("TDEC1", agc.get("TINT"));
            intrpvp();
            // VLOAD UNIT RATT ; PDVL VXV VATT ; ABVAL NORM ; DDV DELTAR ; SL*  -> DELTTIME
            agc.set("DELTTIME", agc.get("DELTAR"));
        } else {
            optn2();
        }
        junctn2();
    }

    private void optn2() {
        // SOR: TINT = TINTSOI + T
        agc.set("TINT", Agc.add(agc.get("TINTSOI"), agc.get("T")));
    }

    private void junctn2() {
        // TARGTIME = TINT - DELTTIME
        agc.set("TARGTIME", Agc.add(agc.get("TINT"), Agc.fromSigned(-Agc.toSigned(agc.get("DELTTIME")))));
        mainrtne();
    }

    /** MAINRTNE: update passive vehicle to target time, compute delta V via S3435.25. */
    public void mainrtne() {
        agc.set("TDEC1", agc.get("TARGTIME"));
        intrpvp();
        agc.set("INTIME", agc.get("TIG"));
        agc.set("SUBEXIT", 0); // SSP SUBEXIT TEST3979
        // CALL S3435.25 (Lambert targeting) using RATT
        test3979();
    }

    private void test3979() {
        if (agc.get("P39/79SW") == 0) {
            mainrtn1();
            return;
        }
        if (agc.get("FINALFLG") == 0) {
            agc.set("UPDATFLG", 1);
        }
        dsply81();
    }

    /** MAINRTN1: compute delta V magnitudes, perigee altitude, then display. */
    public void mainrtn1() {
        agc.set("DELVTPI", agc.get("DELVEET3"));      // |DELVEET3|
        agc.set("DELVTPF", Agc.add(agc.get("VPASS4"), Agc.fromSigned(-Agc.toSigned(agc.get("VTPRIME")))));
        // PERIAPO1 -> perigee altitude; SHIFTR1 -> POSTTPI
        if (agc.get("FINALFLG") == 0) {
            agc.set("UPDATFLG", 1);
        }
        dsply58();
    }

    private void dsply58() {
        vndsply(V06N58SR);          // display HP, delta V, delta V (final)
        dsply81();
    }

    private void dsply81() {
        vndsply(V06N81SR);          // display delta V (LV)
        agc.set("XDELVFLG", 0);
        agc.set("DELVSIN", agc.get("DELVEET3"));
        // VN1645: display TRKMKCNT, TTOGO, +MGA
        if (agc.get("P39/79SW") != 0) {
            p39p79b();
        } else {
            recycle();
        }
    }

    /** P39: this vehicle active; preparation time ATIGINC. */
    public void p39() {
        // TC AVFLAGA
        p39p79a(agc.get("ATIGINC"), agc.get("ATIGINC+1"));
    }

    /** P79: other vehicle active; preparation time PTIGINC. */
    public void p79() {
        // TC AVFLAGP
        p39p79a(agc.get("PTIGINC"), agc.get("PTIGINC+1"));
    }

    private void p39p79a(int hi, int lo) {
        agc.set("KT", hi);
        agc.set("KT+1", lo);
        // TC P20FLGON: set UPDATFLG, TRACKFLG
        agc.set("P39/79SW", 1);
        selectmu();
        p39p79b();
    }

    private void p39p79b() {
        // RTB LOADTIME ; DAD KT : TIG = T(present) + preparation time
        agc.set("TIG", Agc.add(agc.get("TIME2"), agc.get("KT")));
        agc.set("TDEC1", agc.get("TIG"));
        // PRECSET: precision update active and passive vehicles to TIG
        // CALL S34/35.1: get unit normal
        // DLOAD TARGTIME ; GOTO MAINRTNE
        agc.set("MPAC", agc.get("TARGTIME"));
        mainrtne();
    }

    /** PREC/TT: update to TIG, compute transfer time TF from the passive vehicle's central angle. */
    public void prectt() {
        agc.set("TDEC1", agc.get("TIG"));
        // PRECSET ; RVEC = RPASS3 shifted ; CSTH = COS(CENTANG) ; SNTH = SIN(CENTANG)
        // SET RVSW ; VVEC = VPASS3 shifted ; TIMETHET -> transfer time T
        // CALL S34/35.1 (unit normal) ; DLOAD T -> returns T in MPAC
        agc.set("MPAC", agc.get("T"));
    }

    /** INTRPVP: precision update the passive vehicle to TDEC1 (CSMPREC if AVFLAG set, else LEMPREC). */
    public void intrpvp() {
        if (agc.get("AVFLAG") != 0) {
            // BOFF AVFLAG OTHERV not taken -> CALL CSMPREC
        } else {
            // OTHERV: CALL LEMPREC
        }
    }

    /** VNDSPLY: flash the verb/noun in A (TS VERBNOUN; BANKCALL GOFLASH); terminate -> GOTOPOOH. */
    public void vndsply(int verbNoun) {
        agc.set("VERBNOUN", verbNoun);
        // BANKCALL GOFLASH: TERMINATE -> GOTOPOOH ; PROCEED -> return ; RECYCLE -> redisplay
    }
}
