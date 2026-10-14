package apollo.comanche055;

import apollo.agc.Agc;

/**
 * R30.agc - V82CALL (verb 82: orbital parameter display, R30) and SR30.1.
 *
 * <p>V82CALL calculates and displays orbital parameters. With average G off it flashes V04N06 so the
 * astronaut can choose the ship (OPTIONX+1 == 1 for this vehicle), updates that state vector
 * (THISPREC/OTHPREC) and calls SR30.1; with average G on it re-runs SR30.1 about once a second with the
 * current state vector. It then flash-monitors V16N44 (HAPOX, HPERX, TFF). Named flag bits
 * (AVEGFLAG, AMOONFLG, V82EMFLG) are kept in erasable symbols of the same name (non-zero = set).
 *
 * <p>Scaling: positions (-29) m for Earth, (-27) m for Moon; velocities (-7) / (-5) m/cs;
 * times (-28) cs.
 */
public final class R30 {
    /** 2DEC constants, pairwise indexed [earth, moon] (X2 = 0 Earth, 2 Moon). */
    private static final double RTMUM_INV = .45162595E-4;   // 1/RTMUM, B14
    private static final double RTMUE_INV = .50087529E-5;   // 1/RTMUE, B17
    private static final double MINPERM = 10668.0;          // 35 KFT min perigee height, moon (-27) m
    private static final double MINPERE = 91440.0;          // 300 KFT, earth (-29) m
    /** MAXNM = 9999.9 NM scaled (-29) m: 2OCT 0106505603. */
    public static final long MAXNM_RAW = 0106505603L;

    private static final int BIT5 = 020;
    private static final int PRIO7 = 07000;
    private static final int PRIO25 = 025000;
    private static final int ONE_SEC = 100;
    private static final int OPTIONVN = 0412;   // V04N12... VN 0412
    private static final int V16N44 = 01644;

    private final Agc agc;
    private final DisplayInterfaceRoutines display;
    private final Executive executive;
    private final Waitlist waitlist;

    public R30(Agc agc) {
        this.agc = agc;
        this.display = new DisplayInterfaceRoutines(agc);
        this.executive = new Executive(agc);
        this.waitlist = new Waitlist(agc);
    }

    private boolean flag(String name) {
        return agc.get(name) != 0;
    }

    private void setFlag(String name, boolean on) {
        agc.set(name, on ? 1 : 0);
    }

    /** V82CALL: entered by VERB 82 ENTER, priority 10. */
    public void v82call() {
        if (flag("AVEGFLAG")) {
            v82gon();
        } else {
            v82goff();
        }
    }

    /** V82GOFF: average G off - let the astronaut select the vehicle, then update and display. */
    public void v82goff() {
        agc.set("OPTIONX", 2);
        agc.set("OPTIONX+1", 1);               // initial choice: this vehicle
        while (true) {
            int reply = display.goxdspf(OPTIONVN);   // V04N12-style flashing display
            if (reply == DisplayInterfaceRoutines.TERMINATE) {
                display.endext();
                return;
            }
            if (reply == DisplayInterfaceRoutines.PROCEED) {
                break;
            }
            // otherwise DATA IN: OPTIONX+1 = 1 for this vehicle, not 1 for the other; ask again
        }
        waitlist.waitlist(8, this::ticktest);        // 80 ms (BIT4)
        v82goflp();
    }

    /** V82GOFLP: major recycle loop entry. */
    private void v82goflp() {
        while (true) {
            agc.set("EBANK", agc.get("TFF"));       // TFFBANK
            agc.set("V82FLAGS", 0);                 // inhibits decrementing of TFF and -TPER
            executive.findvac(PRIO7, 0, 0);
            v82goff1();                             // the FINDVAC job: state vector update + orbit calc
            // V82STALL: wait until the update sets one of our flag bits
            while ((agc.get("V82FLAGS") & 3) == 0) {
                new ServiceRoutines(agc).delayjob(ONE_SEC);
            }
            // FLAGGON: monitor HAPO, HPER, TFF
            int reply = display.goxdspf(V16N44);
            if (reply == DisplayInterfaceRoutines.RECYCLE) {
                continue;                           // recompute state vector + display
            }
            new ServiceRoutines(agc).b5off();       // TERM or PROCEED: tell TICKTEST to kill itself
            return;
        }
    }

    /** V82GOFF1: state vector update and orbit calculation for the selected vehicle (FINDVAC job). */
    public void v82goff1() {
        double now = loadtime();
        DpWords.set(agc, "TDEC1", now);             // time for state vector update
        DpWords.set(agc, "TSTART82", now);          // time for internal use
        int x2;                                     // 0 = Earth centred, 2 = Moon centred
        if (agc.get("OPTIONX+1") == 1) {
            x2 = new TagsForRelativeSetloc(agc).thisprec();
        } else {
            x2 = new TagsForRelativeSetloc(agc).othprec();
        }
        // BOTHSHIP: move results into TFFCONIC storage areas
        DpWords.setVec(agc, "RONE", DpWords.getVec(agc, "RATT"));   // (-29) m
        DpWords.setVec(agc, "VONE", DpWords.getVec(agc, "VATT"));   // (-7) m/cs
        DpWords.set(agc, "TFF/RTMU", x2 == 0 ? RTMUE_INV : RTMUM_INV);
        DpWords.set(agc, "HPERMIN", x2 == 0 ? MINPERE : MINPERM);
        double rpad;
        if (x2 == 0) {                              // EARTHPAD: PAD 37-B radius, (-29) m
            rpad = DpWords.get(agc, "RPAD");
            setFlag("V82EMFLG", false);             // earth scaling for SR30.1
        } else {                                    // MOONPAD: from RLS vector, (-27) m
            rpad = DpWords.abval(DpWords.getVec(agc, "RLS"));
            setFlag("V82EMFLG", true);              // moon scaling for SR30.1
        }
        DpWords.set(agc, "RPADTEM", rpad);
        sr30dot1();
        if (agc.get("MODREG") == 0) {               // in P00: do DELRSPL
            candel();
        } else {
            splret1();
        }
    }

    /** SPLRET1: correct -TPER / TFF for the time since V82GOFF1 began and set V82FLAGS. */
    private void splret1() {
        double elapsed = loadtime() - DpWords.get(agc, "TSTART82");
        DpWords.set(agc, "TSTART82", elapsed);
        if (DpWords.getRaw(agc, "-TPER") != 0) {
            // TICKTPER: TFF was not computed (59m59s); tick -TPER, display both
            DpWords.set(agc, "-TPER", DpWords.get(agc, "-TPER") + elapsed);
            agc.set("V82FLAGS", 1);                 // TICKTEST increments only -TPER
        } else {
            // TICKTFF: tick TFF only
            DpWords.set(agc, "TFF", DpWords.get(agc, "TFF") + elapsed);
            agc.set("V82FLAGS", 2);                 // TICKTEST increments only TFF
        }
        executive.endofjob();
    }

    /**
     * TICKTEST: waitlist task that perpetuates itself every second until bit 5 of EXTVBACT is zero,
     * incrementing TFF or -TPER according to V82FLAGS.
     */
    public void ticktest() {
        if ((agc.get("EXTVBACT") & BIT5) == 0) {
            executive.novac(PRIO25, 0, 0);          // terminate V82: ENDEXT cannot be called in a rupt
            display.endext();
            return;                                 // TASKOVER
        }
        waitlist.waitlist(ONE_SEC, this::ticktest);  // DOTICK: re-request
        switch (agc.get("V82FLAGS") & 3) {
            case 1: // TPERTICK
                DpWords.das(agc, "-TPER", ONE_SEC);
                break;
            case 2: // TFFTICK
                DpWords.das(agc, "TFF", ONE_SEC);
                break;
            default: // no flag bits set: leave TFF and -TPER alone
                break;
        }
    }

    /** V82GON: average G on - use the current state vector, orbit calc once a second. */
    public void v82gon() {
        executive.findvac(PRIO7, 0, 0);             // V82GON1 does the orbit calculations
        // CCS NEWJOB / CHANG1: let V82GON1 finish before the first display
        executive.chang1();
        v82gon1();
        v82redsp();
    }

    /** V82REDSP: monitor HAPO, HPER, TFF (recycle until terminate/proceed). */
    private void v82redsp() {
        int reply;
        do {
            reply = display.goxdspf(V16N44);
        } while (reply == DisplayInterfaceRoutines.RECYCLE);
        new ServiceRoutines(agc).b5off();           // TERM/PROCEED: tells V82GON1 to kill itself
    }

    /** V82GON1: executive job repeated once a second until bit 5 of EXTVBACT is cleared. */
    public void v82gon1() {
        do {
            // RN and VN fetched together so they belong to the same state vector update
            DpWords.setVec(agc, "RONE", DpWords.getVec(agc, "RN"));  // (-29) m
            DpWords.setVec(agc, "VONE", DpWords.getVec(agc, "VN"));  // (-7) m/cs
            double rpad;
            if (flag("AMOONFLG")) {                 // MOONGON
                setFlag("V82EMFLG", true);
                DpWords.set(agc, "TFF/RTMU", RTMUM_INV);
                DpWords.set(agc, "HPERMIN", MINPERM);
                rpad = DpWords.abval(DpWords.getVec(agc, "RLS"));    // (-27) m
            } else {                                // EARTHGON
                setFlag("V82EMFLG", false);
                DpWords.set(agc, "TFF/RTMU", RTMUE_INV);
                DpWords.set(agc, "HPERMIN", MINPERE);
                rpad = DpWords.get(agc, "RPAD");
            }
            DpWords.set(agc, "RPADTEM", rpad);      // V82GON2
            sr30dot1();
            if (agc.get("MODREG") == 11 || agc.get("MODREG") == 0) {
                candel();                           // in mode 11 or 00: DELRSPL
            }
            // V82GON3
            if ((agc.get("EXTVBACT") & BIT5) == 0) {
                display.endext();                   // astronaut signalled terminate
                return;
            }
            new ServiceRoutines(agc).delayjob(ONE_SEC);
        } while (true);
    }

    /** CANDEL: call DELRSPL (does INTWAKE) with TFF in MPAC, then resume (SPLRET). */
    private void candel() {
        new IntegrationInitialization(agc).intstall();
        new P30P37(agc).delrspl(DpWords.get(agc, "TFF"));
        if (agc.get("MODREG") == 0) {
            splret1();
        }
    }

    private double loadtime() {
        return new Rt8OpCodes(agc).loadtime();
    }

    /**
     * SR30.1: orbital parameters for nouns 32 and 44. Calls TFFCONMU and TFFRP/RA to get RAPO and RPER,
     * then HAPOX/HPERX (limited to 9999.9 NM), -TPER (zero if HPER &lt; HPERMIN) and TFF.
     */
    public void sr30dot1() {
        boolean moon = flag("V82EMFLG");
        double[] r = DpWords.getVec(agc, "RONE");
        double[] v = DpWords.getVec(agc, "VONE");
        if (moon) {                                 // rescale (-29)/(-7) to (-27)/(-5) for TFF routines
            DpWords.setVec(agc, "RONE", DpWords.scale(r, 4));
            DpWords.setVec(agc, "VONE", DpWords.scale(v, 4));
        }
        TimeOfFreeFall tff = new TimeOfFreeFall(agc);
        tff.tffconmu();
        tff.tffrpRa();                              // RAPO in MPAC; RAPO, RPER stored
        double rpadtem = DpWords.get(agc, "RPADTEM");
        double hapo = DpWords.get(agc, "RAPO") - rpadtem;
        if (moon) {
            hapo /= 4.0;                            // SR2R: rescale from (-27) m to (-29) m
        }
        DpWords.set(agc, "HAPOX", maxchk(hapo));    // STORHAPO
        double hper = DpWords.get(agc, "RPER") - rpadtem;
        double hperForCompare = hper;               // MPAC+4 saved for comparison with HPERMIN
        if (moon) {
            hper /= 4.0;
        }
        DpWords.set(agc, "HPERX", maxchk(hper));    // STORHPER
        double negTper;
        if (hperForCompare - DpWords.get(agc, "HPERMIN") >= 0) {   // DOTPER
            negTper = -tff.calctper(DpWords.get(agc, "RPER"));      // TPER stored negative
        } else {
            negTper = 0;                            // HI6ZEROS
        }
        DpWords.set(agc, "-TPER", negTper);         // SKIPTPER
        double rLimit = DpWords.get(agc, "HPERMIN") + rpadtem;
        DpWords.set(agc, "TFF", -tff.calctff(rLimit)); // 59M59S if RPER > HPERMIN + RPADTEM
    }

    /** MAXCHK: limit a distance to 9999.9 NM (MAXNM); also used by P30-P37. */
    public double maxchk(double value) {
        double maxnm = MAXNM_RAW / 268435456.0;
        return value > maxnm ? maxnm : value;
    }
}
