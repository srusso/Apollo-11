package apollo.comanche055;

import apollo.agc.Agc;

/**
 * INTEGRATION_INITIALIZATION (Comanche055, pp 1309-1333): entry points and flag set-up for orbital
 * integration (CSMPREC, LEMPREC, CSMCONIC, LEMCONIC, INTEGRVS, INTEGRV), the integration
 * stall/wake mechanism (INTSTALL, INTWAKE), state vector transfers between the permanent
 * (RRECTCSM / RRECTLEM) and temporary (RRECT) areas, STATEINT periodic update, AVETOMID,
 * MIDTOAV1 / MIDTOAV2 and INTWAKEU (state-vector update requests).
 *
 * The state vector blocks are 42 words (FORTYONE = 41 as last index). Interpretive control flow
 * (SET/CLEAR flags, BON/BOF branches) is shown with agc.set/get on the flag symbols.
 * Integration proper (RECTIFY, KEPPREP, TIMESTEP, ORIGCHNG, ...) lives in the INTEGRATION_ROUTINES class.
 */
public final class IntegrationInitialization {
    /** 600SECS: 2DEC 60000 centiseconds. */
    public static final int SECS_600 = 60000;
    public static final int DT2_MIN = 3;           // DT/2MIN 2DEC 3 B-20
    public static final int DT2_MAX = 4000 * 100;  // DT/2MAX 2DEC 4000 E2 B-20
    public static final int CSECS_3 = 3;
    public static final int TIMEDELT = 1250;       // centiseconds
    public static final int SECS_5_6 = 560;
    public static final int FORTYONE = 41;
    public static final double OMEGMOON = 2.66169947E-8; // B+23
    /** INTBITAB: RASFLAG bits for the three stall areas. */
    public static final int[] INTBITAB = {020100, 010040, 004020};
    /** UPMNSVCD: update-moon-state-vector code (2) and 0. */
    public static final int[] UPMNSVCD = {2, 0};
    /** INTFLBIT / REINTBIT select bits in RASFLAG (high bits of INTBITAB entries). */
    private static final int REINTBIT = 000100 << 0;

    private final Agc agc;

    public IntegrationInitialization(Agc agc) {
        this.agc = agc;
    }

    // ------------------------------------------------------------------ flag helpers
    private void set(String flag) {
        agc.set(flag, 1);
    }

    private void clear(String flag) {
        agc.set(flag, 0);
    }

    private boolean on(String flag) {
        return agc.get(flag) != 0;
    }

    // ------------------------------------------------------------------ STATEINT
    /** STATEINT: restart-protected periodic state vector update job request (PRIO5). */
    public void stateint() {
        // TC PHASCHNG OCT 00052; CAF PRIO5; FINDVAC STATINT1 (EBANK=RRECTCSM)
        statint1();
    }

    /** STATINT1: update the permanent state vectors to current time (TDEC1 = LOADTIME). */
    public void statint1() {
        if (on("QUITFLAG")) {
            noint();                              // no STATEINT if V96
            return;
        }
        agc.set("TDEC1", agc.get("LOADTIME"));
        clear("V96ONFLG");
        intstall();
        set("NODOFLAG");
        setiflgs();
        stateup();
    }

    /** STATEUP: (in INTEGRATION_ROUTINES) integrate and update permanent vectors, then ENDINT. */
    private void stateup() {
        new FreshStartAndRestart(agc).stateup();
    }

    /** ENDINT: clear STATEFLG; phase change; schedule STATEINT again in 600 s (LONGCALL). */
    public void endint() {
        clear("STATEFLG");
        // TC PHASCHNG OCT 20032
        // EXTEND DCA 600SECS; TC LONGCALL; 2CADR STATEINT
        // TC ENDOFJOB
    }

    /** SETIFLGS: SET STATEFLG, CLEAR INTYPFLG, DIM0FLAG, D6OR9FLG. */
    public void setiflgs() {
        set("STATEFLG");
        clear("INTYPFLG");
        clear("DIM0FLAG");
        clear("D6OR9FLG");
    }

    /** NOINT: phase change, clear QUITFLAG (DOWNFLAG), end job. */
    public void noint() {
        // TC PHASCHNG OCT 2
        clear("QUITFLAG");
        // TC ENDOFJOB
    }

    // ------------------------------------------------------------------ permanent <-> temp transfers
    /** Copy 42 words between a source and destination block. */
    private void moveBlock(String from, String to) {
        for (int i = FORTYONE; i >= 0; i--) {
            agc.set(i == 0 ? to : to + "+" + i, agc.get(i == 0 ? from : from + "+" + i));
        }
    }

    /** ATOPCSM: transfer RRECT .. RRECT+41 to RRECTCSM .. RRECTCSM+41, saving moon flag in CMOONFLG. */
    public void atopcsm() {
        moveBlock("RRECT", "RRECTCSM");               // MOVEACSM
        agc.set("CMOONFLG", agc.get("MOONFLAG"));     // SET CMOONFLG via SVDWN1; clear if MOONFLAG off
    }

    /** PTOACSM: transfer RRECTCSM block to RRECT and set/clear MOONFLAG and PBODY. */
    public void ptoacsm() {
        moveBlock("RRECTCSM", "RRECT");               // MOVEPCSM
        if (on("CMOONFLG")) {
            setmoon();
        } else {
            clrmoon();
        }
    }

    /** CLRMOON: clear MOONFLAG, PBODY = 0 (earth). */
    public void clrmoon() {
        clear("MOONFLAG");
        agc.set("PBODY", 0);
    }

    /** SETMOON: set MOONFLAG, PBODY = 2 (moon). */
    public void setmoon() {
        set("MOONFLAG");
        agc.set("PBODY", 2);
    }

    /** ATOPLEM: transfer RRECT block to RRECTLEM, saving moon flag in LMOONFLG. */
    public void atoplem() {
        moveBlock("RRECT", "RRECTLEM");               // MOVEALEM
        agc.set("LMOONFLG", agc.get("MOONFLAG"));
    }

    /** PTOALEM: if LM on the surface use PIOS (USEPIOS), else transfer permanent LM vector to RRECT. */
    public void ptoalem() {
        if (on("SURFFLAG")) {
            usepios();
            return;
        }
        moveBlock("RRECTLEM", "RRECT");               // MOVEPLEM
        if (on("LMOONFLG")) {
            setmoon();
        } else {
            clrmoon();
        }
    }

    /**
     * USEPIOS: LM on lunar surface; use planetary inertial orientation subroutines to get RCV, VCV
     * (VCV = OMEGMOON x RCV), TDELTAV and TNUV cleared, then A-PCHK.
     */
    public void usepios() {
        agc.set("TET", agc.get("TDEC1"));
        // CALL RP-TO-R (RLS, TDEC1, 5/8) -> RCV; UNIT ZUNIT; MOONFLAG set for X1; VCV = RCV x OMEGMOON*ZUNIT
        set("MOONFLAG");
        for (int i = 0; i < 6; i += 2) {
            agc.set(i == 0 ? "TDELTAV" : "TDELTAV+" + i, 0);
        }
        agc.set("PBODY", 2);
        OrbitalIntegration.of(agc).tnuv();
        apchk();
    }

    /** SETBANK: select integration bank; returns FORTYONE as the index counter. */
    public int setbank() {
        agc.set("BBANK", 0);
        return FORTYONE;
    }

    // ------------------------------------------------------------------ special purpose entries
    /** CSMPREC: precision integration of CSM state vector to TDEC1 (VINTFLAG set, INTYPFLG clear). */
    public void csmprec() {
        intstall();
        set("VINTFLAG");
        iflagp();
    }

    /** IFLAGP: PRECIFLG set, DIM0FLAG clear, INTYPFLG clear, go INTEGRV1. */
    public void iflagp() {
        set("PRECIFLG");
        clear("DIM0FLAG");
        clear("INTYPFLG");
        integrv1();
    }

    /** LEMPREC: precision integration of LM state vector (VINTFLAG clear). */
    public void lemprec() {
        intstall();
        clear("VINTFLAG");
        iflagp();
    }

    /** CSMCONIC: conic integration of CSM (VINTFLAG set). */
    public void csmconic() {
        intstall();
        set("VINTFLAG");
        iflagc();
    }

    /** IFLAGC: DIM0FLAG clear, INTYPFLG set, go INTEGRV1. */
    public void iflagc() {
        clear("DIM0FLAG");
        set("INTYPFLG");
        integrv1();
    }

    /** LEMCONIC: conic integration of LM (VINTFLAG clear). */
    public void lemconic() {
        intstall();
        clear("VINTFLAG");
        iflagc();
    }

    /**
     * INTEGRVS: integrate given state vector (RCV, VCV, TET) to TDEC1; sets PRECIFLG, PBODY from
     * MOONFLAG, zeroes TDELTAV/TNUV, rectifies, clears DIM0FLAG, sets NEWIFLG and RPQFLAG, then ALOADED.
     */
    public void integrvs() {
        set("PRECIFLG");
        agc.set("PBODY", on("MOONFLAG") ? 2 : 0);
        agc.set("IRETURN", agc.get("QPRET"));         // STQ IRETURN
        for (int i = 0; i < 6; i += 2) {
            agc.set(i == 0 ? "TDELTAV" : "TDELTAV+" + i, 0);
        }
        OrbitalIntegration.of(agc).tnuv();
        OrbitalIntegration.of(agc).rectify();
        clear("DIM0FLAG");
        set("NEWIFLG");
        set("RPQFLAG");
        aloaded();
    }

    /** INTEGRV: entry which permits the caller (navigation) to set the integration flags. */
    public void integrv() {
        agc.set("IRETURN", agc.get("QPRET"));
        integrv1();
    }

    /** INTEGRV2: as INTEGRV1 but without setting RPQFLAG and NEWIFLG (restart point after RELOADSV). */
    public void integrv2() {
        agc.set("QPRET", 1);                          // SSP QPRET ALOADED
        if (on("VINTFLAG")) {
            ptoacsm();
        } else {
            ptoalem();
        }
        aloaded();
    }

    /** INTEGRV1: set RPQFLAG, NEWIFLG; load permanent state vector of CSM or LM; go ALOADED. */
    public void integrv1() {
        set("RPQFLAG");
        set("NEWIFLG");
        agc.set("QPRET", 1);                          // SSP QPRET ALOADED
        if (on("VINTFLAG")) {
            ptoacsm();
        } else {
            ptoalem();
        }
        aloaded();
    }

    /** ALOADED: TDEC = TDEC1; precision -> TESTLOOP, conic -> RVCON. */
    public void aloaded() {
        agc.set("TDEC", agc.get("TDEC1"));
        if (!on("INTYPFLG")) {
            testloop();
        } else {
            rvcon();
        }
    }

    /**
     * A-PCHK: after a time step check for origin change (unless MIDFLAG clear), then ANDOUT:
     * ORIGCHNG when the sphere of influence boundary is crossed.
     */
    public void apchk() {
        if (on("MIDFLAG")) {
            // CALL CHKSWTCH; BPL ANDOUT; CALL ORIGCHNG (make the switch)
            if (OrbitalIntegration.of(agc).chkswtch() < 0) {
                OrbitalIntegration.of(agc).origchng();
            }
        }
        andout();
    }

    /** ANDOUT: if STATEFLG set (and cleared), do phase change and permanent update; else RECTOUT. */
    public void andout() {
        if (!on("STATEFLG")) {
            rectout();
            return;
        }
        clear("STATEFLG");
        // TC PHASCHNG OCT 04022; UPFLAG REINTFLG (phase change occurred between INTSTALL and INTWAKE)
        set("REINTFLG");
        agc.set("QPRET", 2);                          // SSP QPRET PHEXIT
        if (on("VINTFLAG")) {
            atopcsm();
        } else {
            atoplem();
        }
        phexit();
    }

    /** PHEXIT: GRP2PC (group-2 phase change), then RECTOUT. */
    public void phexit() {
        grp2pc();
        rectout();
    }

    /** RECTOUT: rectify and place RATT, VATT, TAT, RATT1, VATT1, MU in the push list. */
    public void rectout() {
        OrbitalIntegration.of(agc).rectify();
        int x2 = on("MOONFLAG") ? 2 : 0;
        agc.set("X2", x2);
        agc.set("X1", Agc.fromSigned(on("MOONFLAG") ? -10 : -2));
        // RATT = RRECT << X2 ; VATT = VRECT << X2 ; TAT = TET ; RATT1 = RRECT ; VATT1 = VRECT ; MU(P)
        intexit();
    }

    /** INTEXIT: clear MIDAVFLG, AVEMIDSW, PRECIFLG; restore QPRET from IRETURN; INTWAKE. */
    public void intexit() {
        clear("MIDAVFLG");
        clear("AVEMIDSW");                            // allow update of downlink state vector
        clear("PRECIFLG");
        agc.set("QPRET", agc.get("IRETURN"));
        intwake();
    }

    /** RVCON: conic solution for position and velocity over the interval (TET - TDEC). */
    public void rvcon() {
        agc.set("TAU.", Agc.toSigned(agc.get("TDEC")) - Agc.toSigned(agc.get("TET")));
        OrbitalIntegration.of(agc).rectify();
        OrbitalIntegration.of(agc).kepprep();
        agc.set("TET", Agc.add(agc.get("TET"), agc.get("TC")));
        rectout();
    }

    /** TESTLOOP: precision time-step controller (selects DT/2 and calls TIMESTEP). */
    public void testloop() {
        if (!on("QUITFLAG") || on("STATEFLG")) {
            // BOF QUITFLAG +3 ... CLRGO STATEFLG INTEXIT: if QUITFLAG set, clear STATEFLG and stop
        }
        if (on("QUITFLAG")) {
            clear("STATEFLG");
            intexit();                                // stop integration
            return;
        }
        // RC = |RCV|; MIDFLAG = 0 if R > RMP (sphere of influence), else set
        double rc = Math.sqrt(sq("RCV") + sq("RCV+2") + sq("RCV+4"));
        double rme = agc.get(on("MOONFLAG") ? "RME+2" : "RME");
        if (rc - rme >= 0) {
            clear("MIDFLAG");
        } else {
            set("MIDFLAG");
        }
        // DT = sqrt(RC * 34D / MUEARTH(PBODY)) * .3D / 8 ... truncated to a multiple of 128 cs
        double dt2 = 0;
        long maxdt = DT2_MAX;
        long dt = Math.min(maxdt, Math.max(DT2_MIN, (long) dt2));
        long remaining = Agc.toSigned(agc.get("TDEC")) - Agc.toSigned(agc.get("TET"));
        agc.set("DT/2", (int) Math.min(Math.abs(remaining), dt) * (remaining < 0 ? -1 : 1));
        poohchk(Math.abs(remaining), dt);
    }

    /** POOHCHK: if |DT/2| below minimum, the end is reached (A-PCHK); else TIMESTEP. */
    private void poohchk(long remainingAbs, long dt2) {
        if (Math.abs(Agc.toSigned(agc.get("DT/2"))) < DT2_MIN) {
            apchk();
            return;
        }
        if (agc.get("MODREG") == 0 || on("PRECIFLG")) {
            OrbitalIntegration.of(agc).timestep();
            return;
        }
        // not called via CSM(LEM)PREC: check enough time exists for another step (NEWIFLG), no backward integration
        if (remainingAbs <= 0) {
            intexit();
            return;
        }
        if (4 * Math.abs(Agc.toSigned(agc.get("DT/2"))) >= remainingAbs / 4) {
            intexit();
            return;
        }
        OrbitalIntegration.of(agc).timestep();
    }

    private double sq(String sym) {
        double v = Agc.toSigned(agc.get(sym));
        return v * v;
    }

    // ------------------------------------------------------------------ INTSTALL / INTWAKE
    /** INTSTALL: stall (sleep job) until the integration area is free, then grab it. */
    public void intstall() {
        allstall(0);
    }

    /** ERASTAL1: stall on erasable area 1. */
    public void erastal1() {
        allstall(1);
    }

    /** ERASTAL2: stall on erasable area 2. */
    public void erastal2() {
        allstall(2);
    }

    /** ALLSTALL: sleep (JOBSLEEP) until INTBITAB[l] is free in RASFLAG, then OKTOGRAB. */
    private void allstall(int l) {
        while ((agc.get("RASFLAG") & INTBITAB[l] & 077) != 0) {
            // INDEX L; CAF WAKESTAL; TC JOBSLEEP (job is woken by INTWAKE and rechecks)
            break; // single-threaded model: avoid blocking
        }
        // OKTOGRAB: INDEX L; CAF INTFLBIT; ADS RASFLAG
        agc.set("RASFLAG", agc.get("RASFLAG") | (INTBITAB[l] & 0x7FFF));
    }

    /** INTWAKE0: EXIT, then INTWAKE1. */
    public void intwake0() {
        intwake1();
    }

    /** INTWAKE: if the stalled routine was restarted, do phase change and keep Q; else INTWAKE1. */
    public void intwake() {
        if ((agc.get("RASFLAG") & REINTBIT) == 0) {
            intwake1();
            return;
        }
        agc.set("TBASE2", agc.get("QPRET"));          // dont restart with someone else's Q
        // TC PHASCHNG OCT 04022
        agc.set("QPRET", agc.get("TBASE2"));
        if ((agc.get("RASFLAG") & REINTBIT) == 0) {
            return;                                   // GOBAC: dont INTWAKE if we came here via restart
        }
        intwake1();
    }

    /** INTWAKE1 / WAKE: wake up every job waiting on the integration stall areas and release RASFLAG. */
    public void intwake1() {
        wake(0);
    }

    public void erasewak1() {
        wake(1);
    }

    public void erasewak2() {
        wake(2);
    }

    private void wake(int stalltem) {
        agc.set("STALTEM", stalltem);
        // INDEX STALTEM; CAF WAKESTAL; JOBWAKE (loop while more to wake up)
        agc.set("RASFLAG", agc.get("RASFLAG") & ~INTBITAB[stalltem] & Agc.WORD_MASK); // release stall area
        // GOBAC: TC INTPRET; RVQ
    }

    // ------------------------------------------------------------------ AVETOMID
    /**
     * AVETOMID: transition from thrusting phase to coast: integrate W-matrix through the burn if valid,
     * then initialise this vehicle's permanent state vector from RN, VN, PIPTIME; integrate the other
     * vehicle to PIPTIME; zero TRKMKCNT.
     */
    public void avetomid() {
        agc.set("EGRESS", agc.get("QPRET"));
        if (on("RENDWFLG") || on("ORBWFLAG")) {
            intw();                                   // W-matrix valid; integrate it
            return;
        }
        setcoast();
    }

    /** SETCOAST: move properly scaled RN, VN, PIPTIME to integration erasables, make permanent. */
    public void setcoast() {
        intstall();
        int x2 = on("AMOONFLG") ? 2 : 0;
        agc.set("X2", x2);
        // RRECT = RN >> X2; RCV = RRECT; TET = PIPTIME; VRECT = VN >> X2
        agc.set("TET", agc.get("PIPTIME"));
        OrbitalIntegration.of(agc).minirect();      // finish setting up state vector
        // RTB MOVATHIS: put temporary state vector into permanent; TRKMKCNT = 0
        agc.set("TRKMKCNT", 0);
        set("CMOONFLG");
        if (!on("AMOONFLG")) {
            clear("CMOONFLG");
        }
        if (on("SURFFLAG")) {
            egress();                                 // FAZAB5: no coasting LM
            return;
        }
        agc.set("TDEC1", agc.get("PIPTIME"));
        setiflgs();
        clear("VINTFLAG");
        integrv();
        egress();
    }

    /** INT/W: integrate W-matrix through burn to PIPTIME without clobbering RN, VN, PIPTIME. */
    public void intw() {
        agc.set("MPAC", agc.get("PIPTIME"));
        intstall();
        set("DIM0FLAG");
        set("AVEMIDSW");
        clear("D6OR9FLG");
        set("VINTFLAG");
        agc.set("TDEC1", agc.get("MPAC"));
        integrv();
        setcoast();
    }

    private void egress() {
        agc.set("QPRET", agc.get("EGRESS"));          // return to caller
    }

    // ------------------------------------------------------------------ MIDTOAV
    /**
     * MIDTOAV2: integrate the CSM state vector to current time plus increments of TIMEDELT such that the
     * difference between current time and the state vector time is at least 5.6 s.
     */
    public void midtoav2() {
        agc.set("IRETURN1", agc.get("QPRET"));
        intstall();
        agc.set("T-TO-ADD", TIMEDELT);
        clear("MID1FLAG");
        entmid2();
    }

    /**
     * MIDTOAV1: integrate (precision) to TDEC1; if time + TIMEDELT is past the desired time, alarm 1703
     * and switch to MIDTOAV2 behaviour (NOTIME).
     */
    public void midtoav1() {
        agc.set("IRETURN1", agc.get("QPRET"));
        intstall();
        set("MID1FLAG");
        int now = agc.get("LOADTIME");
        if (Agc.toSigned(now) + TIMEDELT - Agc.toSigned(agc.get("TDEC1")) > 0) {
            notime();
            entmid2();
            return;
        }
        entmid1();
    }

    private void entmid2() {
        agc.set("TDEC1", Agc.fromSigned(Agc.toSigned(agc.get("LOADTIME")) + Agc.toSigned(agc.get("T-TO-ADD"))));
        entmid1();
    }

    /** ENTMID1: integrate without W-matrix and publish the state vector to RN1/VN1/PIPTIME1. */
    private void entmid1() {
        clear("DIM0FLAG");
        set("VINTFLAG");                              // THISVINT
        clear("INTYPFLG");
        set("MIDAVFLG");                              // let integration know the call is for MIDTOAV
        integrv();
        int rtx2 = agc.get("X2");
        clear("AMOONFLG");
        if (rtx2 != 0) {
            set("AMOONFLG");
        }
        // RN1 = RATT; VN1 = VATT; PIPTIME1 = TAT
        agc.set("PIPTIME1", agc.get("TAT"));
        // MPAC = TAT - TIME2 (delta time), return to IRETURN1 via BANKJUMP
        agc.set("MPAC", Agc.fromSigned(Agc.toSigned(agc.get("TAT")) - Agc.toSigned(agc.get("TIME2"))));
    }

    /** CKMID2: check time step in MIDTOAV integration. */
    public void ckmid2() {
        if (!on("MID1FLAG")) {
            mid2();
            return;
        }
        if (Agc.toSigned(agc.get("LOADTIME")) + TIMEDELT - Agc.toSigned(agc.get("TDEC")) > 0) {
            notime();
            return;
        }
        testloop();
    }

    /** TIMEINC: TDEC = LOADTIME + T-TO-ADD; TESTLOOP. */
    public void timeinc() {
        agc.set("TDEC", Agc.fromSigned(Agc.toSigned(agc.get("LOADTIME")) + Agc.toSigned(agc.get("T-TO-ADD"))));
        testloop();
    }

    /** MID2: ensure 5.6 s is available to the caller; else add TIMEDELT to T-TO-ADD and retry. */
    public void mid2() {
        int diff = Math.abs(Agc.toSigned(agc.get("TDEC")) - Agc.toSigned(agc.get("TET")));
        if (diff - CSECS_3 > 0) {
            timeinc();
            return;
        }
        if (Agc.toSigned(agc.get("LOADTIME")) - Agc.toSigned(agc.get("TET")) - SECS_5_6 >= 0) {
            apchk();                                  // yes, get out
            return;
        }
        agc.set("T-TO-ADD", Agc.toSigned(agc.get("T-TO-ADD")) + TIMEDELT);
        timeinc();
    }

    /** NOTIME: too late; clear MID1FLAG, set error exit, ALARM 1703, T-TO-ADD = TIMEDELT. */
    public void notime() {
        clear("MID1FLAG");
        agc.set("IRETURN1", agc.get("IRETURN1") + 1); // error exit (CALLOC + 2)
        // TC ALARM OCT 1703: insufficient time for integration -- TIG will be slipped
        agc.set("T-TO-ADD", TIMEDELT);
    }

    // ------------------------------------------------------------------ INTWAKEU
    /**
     * INTWAKEU: process a CSM/LEM state vector update request (UPSVFLAG). Moves RRECT/VRECT into RCV/VCV,
     * selects earth/moon sphere (UPMNSVCD), updates the permanent CSM (UPSVFLAG &gt;= 0) or LM vector,
     * removes the update indicator and releases the integration (INTWAKE0).
     */
    public void intwakeu() {
        int upsv = Agc.toSigned(agc.get("UPSVFLAG"));
        if (upsv != 0) {
            for (int i = 0; i < 6; i += 2) {
                String r = i == 0 ? "RRECT" : "RRECT+" + i;
                String v = i == 0 ? "VRECT" : "VRECT+" + i;
                agc.set(i == 0 ? "RCV" : "RCV+" + i, agc.get(r));
                agc.set(i == 0 ? "VCV" : "VCV+" + i, agc.get(v));
            }
            OrbitalIntegration.of(agc).rectify13d();    // store VRECT into VCV; zero TDELTAV, TNUV, TC, XKEP
            if (Math.abs(upsv) - UPMNSVCD[0] == 0) {
                agc.set("X2", 2);                         // INTWAKEM: lunar sphere
                set("MOONFLAG");
            } else {
                agc.set("X2", 0);                         // earth sphere
                clear("MOONFLAG");
            }
            if (upsv >= 0) {
                atopcsm();                                // update CSM state vector
                clear("ORBWFLAG");
            } else {
                atoplem();                                // INTWAKLM: update LM state vector
            }
            clear("RENDWFLG");
        }
        agc.set("UPSVFLAG", 0);                           // remove update state vector indicator
        intwake0();
        // TC PHASCHNG OCT 04026; TC INTWAKUQ (return)
    }

    /** GRP2PC: phase change for group 2 (OCT 04022) then return to caller (GRP2SVQ). */
    public void grp2pc() {
        // TC PHASCHNG OCT 04022; GOTO GRP2SVQ
    }

    // ------------------------------------------------------------------ routines declared elsewhere
    /** OTHPREC: precision integration of the OTHER vehicle (see P76); dispatches on vehicle flag. */
    public void othprec() {
        if (on("VINTFLAG")) {
            lemprec();
        } else {
            csmprec();
        }
    }

    /** MINIRECT: rectification stub (implemented in INTEGRATION_ROUTINES). */
    public void minirect() {
        OrbitalIntegration.of(agc).minirect();
    }

    /** ATOPOTH: store the temporary state into the OTHER vehicle's permanent area. */
    public void atopoth() {
        if (on("VINTFLAG")) {
            atoplem();
        } else {
            atopcsm();
        }
    }
}
