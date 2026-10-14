package apollo.luminary099;

import apollo.agc.Agc;

/**
 * INTEGRATION INITIALIZATION (INTEGRATION_INITIALIZATION.agc, Luminary 099 pp. 1205-1226).
 *
 * <p>Entrances to the orbital integration program: CSMPREC/LEMPREC (precision Encke
 * integration), CSMCONIC/LEMCONIC (Kepler), INTEGRVS (integrate a supplied state vector),
 * INTEGRV (caller sets flags), the permanent/temporary state vector transfer routines
 * (ATOPCSM, PTOACSM, ATOPLEM, PTOALEM), the INTSTALL/INTWAKE stall mechanism, AVETOMID and
 * MIDTOAV1/MIDTOAV2 (thrust/coast transitions) and INTWAKEU (state-vector update requests).
 *
 * <p>Conventions of this translation: interpretive flags (MOONFLAG, MIDFLAG, VINTFLAG ...) are
 * stored as 0/1 in erasable symbols of the same name; state vectors are the 42-register blocks
 * RRECT (temporary), RRECTCSM and RRECTLEM (permanent), copied word for word; double-precision
 * times are signed 28-bit raw values (centiseconds, scale B-28) accessed via {@link VecOps}.
 * The interpretive push-down list is modelled by the symbols PD0..; interpretive "CALL"
 * becomes a Java call and "STQ/RVQ" return linkage becomes the Java call stack.
 *
 * <p>Input TDEC1 (time to integrate to) and flags VINTFLAG (1 = CSM, 0 = LM), INTYPFLG
 * (1 = conic, 0 = precision), DIM0FLAG (1 = integrate W-matrix), D6OR9FLG (1 = 9x9, 0 = 6x6),
 * STATEFLG (request permanent update). Output RATT/VATT/TAT, RATT1/VATT1, MU(P).
 */
public class IntegrationInitialization {
    /** Number of registers in a state vector block (RRECT .. RRECT+41D). */
    public static final int STATE_VECTOR_WORDS = 42;
    /** FORTYONE: DEC 41, the CCS index start for the block moves. */
    public static final int FORTYONE = 41;
    /** 600SECS: 2DEC 60000 (600 seconds in centiseconds). */
    public static final long SECS600 = 60000L;
    /** DT/2MIN: 2DEC 3 B-20 (centiseconds, scaled B-20). */
    public static final double DT2MIN = 3.0 / (1 << 20) * (1L << 28);
    /** DT/2MAX: 2DEC 4000 E2 B-20. */
    public static final double DT2MAX = 4000e2 / (1 << 20) * (1L << 28);
    /** 3CSECS: 2DEC 3. */
    public static final long CSECS3 = 3L;
    /** TIMEDELT: 2DEC 2000 (20 seconds of computation allowance). */
    public static final long TIMEDELT = 2000L;
    /** INTBITAB: OCT 20100, RASFLAG bits for the integration stall area. */
    public static final int INTBITAB = 020100;
    /** UPMNSVCD: update-moon-state-vector code. */
    public static final int UPMNSVCD = 2;
    /** Alarm code 1703: insufficient time for integration. */
    public static final int ALARM_1703 = 01703;

    private final Agc agc;

    public IntegrationInitialization(Agc agc) {
        this.agc = agc;
    }

    // ------------------------------------------------------------------ helpers

    private boolean flag(String name) {
        return agc.get(name) != 0;
    }

    private void set(String name) {
        agc.set(name, 1);
    }

    private void clear(String name) {
        agc.set(name, 0);
    }

    private static String sym(String base, int i) {
        return i == 0 ? base : base + "+" + i;
    }

    /** Copies the 42-register state vector block {@code from} to {@code to} (CCS loop 41..0). */
    private void moveBlock(String from, String to) {
        for (int i = FORTYONE; i >= 0; i--) {
            agc.set(sym(to, i), agc.get(sym(from, i)));
        }
    }

    private long dpRaw(String s) {
        return VecOps.getDpRaw(agc, s);
    }

    private void setDpRaw(String s, long v) {
        VecOps.setDpRaw(agc, s, v);
    }

    private long loadtime() {
        return dpRaw("TIME2");  // RTB LOADTIME: TIME2,TIME1 as a DP value
    }

    private InterpretiveCalls ic() {
        return new InterpretiveCalls(agc);
    }

    /** Hook for neighbours (PHASCHNG, FINDVAC, LONGCALL ...) not translated in this file. */
    private static final class InterpretiveCalls {
        private final Agc agc;

        InterpretiveCalls(Agc agc) {
            this.agc = agc;
        }

        void phaschng(int code) {
            new PhaseTableMaintenance(agc).phaschng(code);
        }
    }

    // ------------------------------------------------------------------ STATEINT

    /** STATEINT: periodic (every 600 s) state vector integration task started under the Waitlist. */
    public void stateint() {
        ic().phaschng(052);
        new Executive(agc).findvac(PRIO5, this::statint1);   // CAF PRIO5; TC FINDVAC; 2CADR STATINT1
        // TC TASKOVER
    }

    private static final int PRIO5 = 05000;

    /** STATINT1: the job body; integrates to the present time and updates the permanent vectors. */
    public void statint1() {
        if (flag("QUITFLAG")) {              // KILL INTEGRATION UNTIL NEXT P00.
            noint();
            return;
        }
        setDpRaw("TDEC1", loadtime());       // RTB LOADTIME; STORE TDEC1
        intstall();
        set("NODOFLAG");
        setiflgs();
        new OrbitalIntegration(agc).stateup();   // GOTO STATEUP
    }

    /** ENDINT: end of the STATEINT job; reschedules itself 600 seconds later. */
    public void endint() {
        clear("STATEFLG");
        ic().phaschng(020032);
        new Waitlist(agc).longcall(SECS600, this::stateint); // DCA 600SECS; TC LONGCALL; 2CADR STATEINT
        // TC ENDOFJOB
    }

    /** SETIFLGS: set STATEFLG; clear INTYPFLG, DIM0FLAG, D6OR9FLG (precision, no W-matrix). */
    public void setiflgs() {
        set("STATEFLG");
        clear("INTYPFLG");
        clear("DIM0FLAG");
        clear("D6OR9FLG");
    }

    /** NOINT: integration killed; restore phase and allow integration restart (clear QUITFLAG). */
    public void noint() {
        ic().phaschng(02);
        clear("QUITFLAG");                   // TC DOWNFLAG; ADRES QUITFLAG
        // TC ENDOFJOB
    }

    // ------------------------------------------------------------------ state vector transfers

    /**
     * ATOPCSM: transfers RRECT..RRECT+41 to RRECTCSM..RRECTCSM+41 (A-memory to permanent) and
     * saves MOONFLAG in CMOONFLG. Normal exit at L+2.
     */
    public void atopcsm() {
        moveacsm();
        // SET CMOONFLG; CALL SVDWN1; BON MOONFLAG ...; CLRGO CMOONFLG
        if (flag("MOONFLAG")) {
            set("CMOONFLG");
        } else {
            clear("CMOONFLG");
        }
    }

    /** MOVEACSM: RRECT block to RRECTCSM block. */
    public void moveacsm() {
        moveBlock("RRECT", "RRECTCSM");
    }

    /** PTOACSM: transfers RRECTCSM block to RRECT block and sets MOONFLAG/PBODY from CMOONFLG. */
    public void ptoacsm() {
        movepcsm();
        if (flag("CMOONFLG")) {
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

    /** MOVEPCSM: RRECTCSM block to RRECT block. */
    public void movepcsm() {
        moveBlock("RRECTCSM", "RRECT");
    }

    /** ATOPLEM: RRECT block to RRECTLEM block, saving MOONFLAG in LMOONFLG. */
    public void atoplem() {
        movealem();
        if (flag("MOONFLAG")) {
            set("LMOONFLG");
        } else {
            clear("LMOONFLG");
        }
    }

    /** MOVEALEM: RRECT block to RRECTLEM block. */
    public void movealem() {
        moveBlock("RRECT", "RRECTLEM");
    }

    /**
     * PTOALEM: RRECTLEM block to RRECT block. If the LM is on the lunar surface (SURFFLAG) the
     * state is instead computed from the planetary inertial orientation (USEPIOS).
     */
    public void ptoalem() {
        if (flag("SURFFLAG")) {
            usepios();
            return;
        }
        moveplem();
        if (flag("LMOONFLG")) {
            setmoon();
        } else {
            clrmoon();
        }
    }

    /** MOVEPLEM: RRECTLEM block to RRECT block. */
    public void moveplem() {
        moveBlock("RRECTLEM", "RRECT");
    }

    /**
     * USEPIOS: LM on the lunar surface: position from RP-TO-R of the landing site vector RLS at
     * time TDEC1, velocity = OMEGMOON x R, zero deviations, X2 = PBODY = 2 (moon).
     */
    public void usepios() {
        long t = dpRaw("TDEC1");
        setDpRaw("TET", t);                           // PDDL TDEC1; STODL TET
        double[] rls = VecOps.getV(agc, "RLS");       // VLOAD RLS (pushed at 0)
        double[] r = new PlanetaryInertialOrientation(agc).rpToR(rls, t, 5.0 / 8.0); // 5/8 scale, CALL RP-TO-R
        VecOps.setV(agc, "RCV", r);                   // STOVL RCV
        // ZUNIT stored at 0D, TET at 6D (push-down list for the second RP-TO-R call)
        set("MOONFLAG");                              // NEEDED FOR SETTING X1 ON EXIT
        double[] zunit = {0.0, 0.0, 1.0};
        double[] v = VecOps.cross(r, VecOps.getV(agc, "OMEGMOON")); // VXV; VXSC
        VecOps.setV(agc, "VCV", v);
        VecOps.setV(agc, "TDELTAV", new double[] {0, 0, 0});
        agc.set("PBODY", 2);                          // AXT,2 2; SXA,2 PBODY
        VecOps.setV(agc, "TNUV", new double[] {0, 0, 0});
        if (zunit.length != 3) {
            return;
        }
        apchk();                                      // STCALL TNUV; A-PCHK
    }

    /** SETBANK: select the integration bank (INTEGRV) and return the block length 41 in A. */
    public int setbank() {
        return FORTYONE;
    }

    // ------------------------------------------------------------------ special entries

    /** CSMPREC: precision integration of the CSM state vector to TDEC1. */
    public void csmprec() {
        intstall();
        set("VINTFLAG");
        iflagp();
    }

    /** IFLAGP: precision integration flags (PRECIFLG set, DIM0FLAG and INTYPFLG clear). */
    public void iflagp() {
        set("PRECIFLG");
        clear("DIM0FLAG");
        clear("INTYPFLG");
        integrv1();
    }

    /** LEMPREC: precision integration of the LM state vector to TDEC1. */
    public void lemprec() {
        intstall();
        clear("VINTFLAG");
        iflagp();
    }

    /** CSMCONIC: conic (Kepler) integration of the CSM state vector to TDEC1. */
    public void csmconic() {
        intstall();
        set("VINTFLAG");
        iflagc();
    }

    /** IFLAGC: conic integration flags (DIM0FLAG clear, INTYPFLG set). */
    public void iflagc() {
        clear("DIM0FLAG");
        set("INTYPFLG");
        integrv1();
    }

    /** LEMCONIC: conic integration of the LM state vector to TDEC1. */
    public void lemconic() {
        intstall();
        clear("VINTFLAG");
        iflagc();
    }

    /** INTEGRVS: integrate a caller-supplied state vector (RCV, VCV, TET, RFINAL, flags). */
    public void integrvs() {
        set("PRECIFLG");
        agc.set("PBODY", 0);
        if (flag("MOONFLAG")) {
            agc.set("PBODY", 2);
        }
        VecOps.setV(agc, "TDELTAV", new double[] {0, 0, 0});     // VLOAD ZEROVEC; STORE TDELTAV
        agc.set("TNUV", 0);
        new ConicSubroutines(agc).rectify();                      // STCALL TNUV; RECTIFY
        clear("DIM0FLAG");
        set("NEWIFLG");
        set("RPQFLAG");
        aloaded();
    }

    /** INTEGRV: general entry; the caller has set VINTFLAG, INTYPFLG, DIM0FLAG, D6OR9FLG. */
    public void integrv() {
        integrv1();
    }

    /** INTEGRV1: entry used by CSMPREC et al.; copies the permanent vector into A-memory. */
    public void integrv1() {
        set("RPQFLAG");
        set("NEWIFLG");
        integrv2();
    }

    /** INTEGRV2: bring the proper permanent state vector (CSM or LM) into RRECT. */
    public void integrv2() {
        agc.set("QPRET", 0); // SSP QPRET ALOADED
        if (flag("VINTFLAG")) {
            ptoacsm();
        } else {
            ptoalem();
        }
        aloaded();
    }

    /** ALOADED: A-memory holds the state vector; start integrating toward TDEC (= TDEC1). */
    public void aloaded() {
        setDpRaw("TDEC", dpRaw("TDEC1"));
        if (!flag("INTYPFLG")) {         // BOFF INTYPFLG TESTLOOP: precision
            testloop();
        } else {
            rvcon();                      // conic
        }
    }

    /** A-PCHK: after a timestep, optional permanent update and phase change; then RECTOUT. */
    public void apchk() {
        if (flag("STATEFLG")) {
            ic().phaschng(04022);
            if (flag("REINTFLG")) {
                // PHASE CHANGE HAS OCCURRED BETWEEN INTSTALL AND INTWAKE: UPFLAG REINTFLG
            }
            set("REINTFLG");
            agc.set("QPRET", 0);          // SSP QPRET PHEXIT
            if (flag("VINTFLAG")) {
                atopcsm();
            } else {
                atoplem();
            }
            phexit();
            return;
        }
        rectout();
    }

    /** PHEXIT: GRP2PC then fall through to RECTOUT. */
    public void phexit() {
        grp2pc();
        rectout();
    }

    /**
     * RECTOUT: rectify and output RATT, VATT, TAT and MU for the current origin; sets X1 (mutable
     * entry, -10D earth or -2 moon) and finishes through INTEXIT.
     */
    public void rectout() {
        new ConicSubroutines(agc).rectify();
        int shift = agc.get("X2");                 // VSL* 0,2: scale between earth and moon
        double[] r = VecOps.getV(agc, "RRECT");
        double[] v = VecOps.getV(agc, "VRECT");
        VecOps.setV(agc, "RATT", scalePow2(r, shift));    // RATT TO PD0
        VecOps.setV(agc, "VATT", scalePow2(v, shift));    // VATT TO PD6
        setDpRaw("TAT", dpRaw("TET"));                    // TAT TO PD12
        // PDVL VRECT; PDDL* MUEARTH,2; PUSH: MU(P) TO PD26
        agc.set("X1", Agc.fromSigned(flag("MOONFLAG") ? -2 : -10));
        intexit();
    }

    private static double[] scalePow2(double[] v, int shift) {
        double[] out = new double[v.length];
        double f = Math.pow(2, shift);
        for (int i = 0; i < v.length; i++) {
            out[i] = v[i] * f;
        }
        return out;
    }

    /** INTEXIT: clear AVEMIDSW and PRECIFLG and STATEFLG, restore Q, wake stalled users. */
    public void intexit() {
        // SETPD 0; BOV +1 (clear overflow)
        clear("AVEMIDSW");              // ALLOW UPDATE OF DOWNLINK STATE VECTOR
        clear("PRECIFLG");
        clear("STATEFLG");
        agc.set("QPRET", agc.get("IRETURN"));  // SLOAD IRETURN; CA MPAC; INDEX FIXLOC; TS QPRET
        intwake();
    }

    /** RVCON: conic solution over the interval (TET - TDEC): set TAU. and call KEPPREP. */
    public void rvcon() {
        setDpRaw("TAU.", dpRaw("TDEC") - dpRaw("TET"));
        ConicSubroutines cs = new ConicSubroutines(agc);
        cs.rectify();
        cs.kepprep();
        setDpRaw("TET", dpRaw("TC") + dpRaw("TET"));
        rectout();
    }

    // ------------------------------------------------------------------ timestep control

    /**
     * TESTLOOP: decides whether integration is finished and picks the next timestep DT/2.
     * Time steps are limited to DT/2MAX, truncated to a multiple of 128 cs, and must not overshoot
     * TDEC; when the step falls below DT/2MIN the final step is taken.
     */
    public void testloop() {
        if (flag("QUITFLAG")) {                   // BOF QUITFLAG +3
            clear("STATEFLG");                    // CLRGO STATEFLG INTEXIT: STOP INTEGRATION
            intexit();
            return;
        }
        int body = agc.get("PBODY");              // LXA,2 PBODY
        double[] rcv = VecOps.getV(agc, "RCV");
        double rc = VecOps.abs(rcv);              // RC TO 10D
        clear("MIDFLAG");
        // DSU* RME,2; BMN +3: MIDFLAG set unless RC is below the sphere-of-influence radius
        double rme = VecOps.getDp(agc, sym("RME", body));
        if (!(rc - rme < 0)) {
            set("MIDFLAG");
        }
        norfinal(rc, body);
    }

    private void norfinal(double rc, int body) {
        double v2 = VecOps.getDp(agc, "34D");     // pushed velocity-squared term (PD 34D)
        double mu = VecOps.getDp(agc, sym("MUEARTH", body));
        // DT is truncated to a multiple of 128 CSECS: DMP, SR1R, DDV* MUEARTH,2, SQRT, DMP .3D, SR3 SR4
        double dt = Math.sqrt(rc * v2 / (2 * mu)) * 0.3 / 32.0;
        double dtMax = Math.floor(dt * (1 << 15)) / (1 << 15);      // SL 15D truncation
        dt2comp(dtMax);
    }

    private void dt2comp(double dtMax) {
        double remaining = dpRaw("TDEC") - dpRaw("TET");
        double half = remaining * 256.0 / (1L << 28);               // SGNAGREE; SL 8D -> DT/2 (B-19)
        boolean overflow = Math.abs(half) >= 1.0;
        if (overflow) {
            half = Math.signum(remaining);                           // GETMAXDT: sign of remaining
        }
        double ceil = Math.min(dtMax, DT2MAX / (1L << 28));
        if (Math.abs(half) > ceil) {                                  // DSU 12D; BMN POOHCHK
            half = Math.copySign(ceil, half);                         // USEMAXDT
        }
        poohchk(half);
    }

    private void poohchk(double half) {
        agc.set("DT/2", 0);
        VecOps.setDp(agc, "DT/2", half);
        if (Math.abs(half) - DT2MIN / (1L << 28) < 0) {              // DT/2 TOO SMALL
            apchk();
            return;
        }
        if (agc.get("MODREG") == 0 || flag("PRECIFLG")) {            // SLOAD MODREG; BHIZ; PRECIFLG
            new OrbitalIntegration(agc).timestep();
            return;
        }
        if (Math.abs(half) - 12.0 < 0 && flag("NEWIFLG")) {
            clear("NEWIFLG");                                         // BOFCLR NEWIFLG TIMESTEP
            new OrbitalIntegration(agc).timestep();
            return;
        }
        long remaining = dpRaw("TDEC") - dpRaw("TET");
        if (remaining < 0) {                                          // NO BACKWARD INTEGRATION
            intexit();
            return;
        }
        // IS 4(DT) LS (TDEC - TET): if not, exit; otherwise take the timestep
        if (4 * Math.abs(half) * (1L << 28) / 256.0 < remaining) {
            new OrbitalIntegration(agc).timestep();
        } else {
            intexit();
        }
    }

    // ------------------------------------------------------------------ INTSTALL / INTWAKE

    /**
     * INTSTALL: caller must call this before setting integration flags. If another user holds the
     * integration stall area (RASFLAG & INTBITAB) the job sleeps until INTWAKE; then grabs it.
     */
    public void intstall() {
        while ((agc.get("RASFLAG") & INTBITAB) != 0) {
            new Executive(agc).jobsleep(this::intstall);   // CAF WAKESTAL; TC JOBSLEEP
            return;
        }
        // OKTOGRAB
        agc.set("RASFLAG", agc.get("RASFLAG") + INTFLBIT);  // INHINT; ADS RASFLAG
    }

    private static final int INTFLBIT = 0100;

    /** INTWAKE0: wake path used from INTWAKEU. */
    public void intwake0() {
        intwake1();
    }

    /** INTWAKE: release the stall area; handles restarts via REINTBIT. */
    public void intwake() {
        if ((agc.get("RASFLAG") & REINTBIT) == 0) {          // not to be restarted
            intwake1();
            return;
        }
        int qpret = agc.get("QPRET");                        // DON'T RESTART WITH SOMEONE ELSE'S Q
        agc.set("TBASE2", qpret);
        ic().phaschng(04022);
        agc.set("QPRET", agc.get("TBASE2"));
        if ((agc.get("RASFLAG") & REINTBIT) == 0) {
            return;                                          // GOBAC: came here via restart
        }
        intwake1();
    }

    private static final int REINTBIT = 020000;

    /** INTWAKE1: wake all jobs sleeping on WAKESTAL and release the stall area. */
    public void intwake1() {
        new Executive(agc).jobwake(this::intstall);          // CAF WAKESTAL; TC JOBWAKE (repeat while woken)
        agc.set("RASFLAG", agc.get("RASFLAG") & ~INTBITAB & Agc.WORD_MASK);  // RELEASE STALL AREA
        // RELINT; GOBAC
    }

    // ------------------------------------------------------------------ AVETOMID

    /**
     * AVETOMID: transition from thrusting to coast. Integrates the W-matrix (if valid) to
     * PIPTIME with the pre-thrust state vector, integrates the other vehicle to PIPTIME,
     * then installs the AVERAGEG state (RN, VN, PIPTIME) as this vehicle's permanent state
     * vector, and zeroes TRKMKCNT.
     */
    public void avetomid() {
        if (flag("RENDWFLG") || flag("ORBWFLAG")) {
            intW();                                          // W-MATRIX VALID, GO INTEGRATE IT
        } else {
            others();
        }
    }

    /** OTHERS: integrate the other vehicle, then move RN, VN, PIPTIME into the permanent vector. */
    public void others() {
        setDpRaw("TDEC1", dpRaw("PIPTIME"));        // DLOAD PIPTIME (desired time)
        intstall();
        set("VINTFLAG");                            // CM
        setiflgs();                                 // NONE W-MAT. PERMANENT INTEG.
        integrv();
        intstall();
        int x2 = flag("MOONTHIS") ? 2 : 0;          // BON MOONTHIS; AXT,2 2 / 0
        double[] rn = VecOps.getV(agc, "RN");
        double[] vn = VecOps.getV(agc, "VN");
        VecOps.setV(agc, "RRECT", scalePow2(rn, -x2));
        VecOps.setV(agc, "RCV", scalePow2(rn, -x2));
        setDpRaw("TET", dpRaw("PIPTIME"));
        VecOps.setV(agc, "VCV", scalePow2(vn, -x2));
        new ConicSubroutines(agc).minirect();       // FINISH SETTING UP STATE VECTOR
        new OrbitalIntegration(agc).movathis();     // PUT TEMP STATE VECTOR INTO PERMANENT
        agc.set("TRKMKCNT", 0);
        new MeasurementIncorporation(agc).fazab5(); // GOTO FAZAB5
    }

    /** INT/W: integrate the W-matrix through the burn, then continue with OTHERS. */
    public void intW() {
        setDpRaw("TDEC1", dpRaw("PIPTIME"));        // INTEGRATE W THRU BURN
        intstall();
        set("DIM0FLAG");                            // DO W-MATRIX
        set("AVEMIDSW");                            // SO WON'T CLOBBER RN,VN,PIPTIME
        set("D6OR9FLG");                            // 9X9 FOR LM
        clear("VINTFLAG");                          // LM
        integrv();
        others();                                   // NOW GO DO THE OTHER VEHICLE
    }

    // ------------------------------------------------------------------ MIDTOAV1 / MIDTOAV2

    /**
     * MIDTOAV2: integrate this vehicle's state vector to the present time plus TIMEDELT.
     * Returns the Java equivalent of the basic return: true if the integration was finished,
     * false if the insufficient-time exit (CALLOC+2, alarm 1703) was taken.
     */
    public boolean midtoav2() {
        clear("MID1FLAG");
        return entmid2();
    }

    /**
     * MIDTOAV1: integrate (precision) to TDEC1. If TDEC1 is not in the future by TIMEDELT the
     * alarm 1703 is set and the integration is done to the present time (MIDTOAV2).
     */
    public boolean midtoav1() {
        set("MID1FLAG");
        long now = loadtime();
        if (dpRaw("TDEC1") - (now + TIMEDELT) >= 0) {        // INITIAL CHECK, IS TDEC1 IN THE FUTURE
            return entmid1();
        }
        notime();                                            // NO: SET ALARM, SWITCH TO MIDTOAV2
        entmid2();
        return false;
    }

    private boolean entmid2() {
        setDpRaw("TDEC1", loadtime() + TIMEDELT);
        return entmid1();
    }

    private boolean entmid1() {
        intstall();
        clear("DIM0FLAG");                                   // NO W-MATRIX
        thisvint();
        clear("INTYPFLG");
        set("MIDAVFLG");                                      // LET INTEG. KNOW THE CALL IS FOR MIDTOAV
        integrv();
        clear("MIDAVFLG");
        VecOps.setV(agc, "RN1", VecOps.getV(agc, "RATT"));
        VecOps.setV(agc, "VN1", VecOps.getV(agc, "VATT"));
        setDpRaw("PIPTIME1", dpRaw("TAT"));
        agc.set("RTX2", agc.get("X2"));
        agc.set("RTX1", agc.get("X1"));
        // INHINT; DCS TIME2; DAS MPAC: MPAC = TAT - current time
        long mpac = dpRaw("TAT") - loadtime();
        setDpRaw("MPAC", mpac);
        return !flag("MID1FLAG") || agc.get("IRETURN1") == 0;
    }

    /** THISVINT: sets VINTFLAG according to this vehicle (LM program: clear VINTFLAG). */
    private void thisvint() {
        clear("VINTFLAG");                 // THIS VEHICLE IS THE LM
    }

    /** CKMID2: test whether the next midcourse timestep would overshoot the wanted time. */
    public void ckmid2() {
        if (!flag("MID1FLAG")) {
            mid2();
            return;
        }
        long lhs = loadtime() + TIMEDELT;
        if (lhs - dpRaw("TDEC") >= 0) {         // BPL: not enough time
            notime();
            return;
        }
        testloop();
    }

    /** TIMEINC: advance TDEC by the computation allowance and re-test. */
    public void timeinc() {
        setDpRaw("TDEC", loadtime() + TIMEDELT);
        testloop();
    }

    /** MID2: if |TDEC - TET| is under 3 cs finish via A-PCHK, else TIMEINC. */
    public void mid2() {
        long d = Math.abs(dpRaw("TDEC") - dpRaw("TET"));
        if (d - CSECS3 < 0) {
            apchk();
        } else {
            timeinc();
        }
    }

    /** NOTIME: too late. Clears MID1FLAG, bumps IRETURN1 (error exit) and raises alarm 1703. */
    public void notime() {
        clear("MID1FLAG");
        agc.set("IRETURN1", agc.get("IRETURN1") + 1);  // SET ERROR EXIT (CALLOC +2)
        new AlarmAndAbort(agc).alarm(ALARM_1703, 0);   // INSUFFICIENT TIME -- TIG WILL BE SLIPPED
    }

    // ------------------------------------------------------------------ INTWAKEU (update bank)

    /**
     * INTWAKEU: process a CSM/LEM state vector update request (UPSVFLAG) from the ground update
     * program: move RRECT/VRECT into RCV/VCV, zero TDELTAV, TNUV, TC and XKEP via RECTIFY+13D,
     * select earth or moon (|UPSVFLAG| vs UPMNSVCD), update the CSM (UPSVFLAG positive) or LEM
     * (negative) permanent vector, then release the integration "grab".
     */
    public void intwakeu() {
        int up = Agc.toSigned(agc.get("UPSVFLAG"));
        if (up != 0) {
            VecOps.setV(agc, "RCV", VecOps.getV(agc, "RRECT"));
            VecOps.setV(agc, "VCV", VecOps.getV(agc, "VRECT"));
            new ConicSubroutines(agc).rectify13();             // RECTIFY +13D
            if (Math.abs(up) - UPMNSVCD == 0) {
                agc.set("X2", 2);                              // LUNAR SPHERE OF INFLUENCE
                set("MOONFLAG");
            } else {
                agc.set("X2", 0);                              // EARTH SPHERE OF INFLUENCE
                clear("MOONFLAG");
            }
            if (up < 0) {                                      // intwaklm
                atoplem();                                     // UPDATE LM STATE VECTOR
            } else {
                atopcsm();                                     // UPDATE CSM STATE VECTOR
                clear("ORBWFLAG");
            }
            clear("RENDWFLG");
        }
        agc.set("UPSVFLAG", 0);                                // REMOVE UPDATE STATE VECTOR INDICATOR
        intwake0();                                            // RELEASE `GRAB' OF ORBIT INTEG.
        ic().phaschng(04026);
    }

    /** GRP2PC: phase change to group 2 (P20-P25), code 04022. */
    public void grp2pc() {
        ic().phaschng(04022);
    }
}
