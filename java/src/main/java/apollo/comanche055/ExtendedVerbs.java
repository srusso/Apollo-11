package apollo.comanche055;

import apollo.agc.Agc;

/**
 * EXTENDED_VERBS: the extended verb (V40-V99) dispatch table and the routines for the
 * non-update extended verbs: IMU zero/coarse/fine align, attitude meters, flag set/reset,
 * rendezvous tracking, DAP data load (R03), state vector transfers, SHOWSUM, etc.
 * (V70-V73 are in {@link UpdateProgram}.)
 *
 * External routines (BANKCALL, FINDVAC, NOVAC, display and mode-switching programs) are routed
 * through {@link #extern(String, Object...)}, which a host simulator may override.
 */
public class ExtendedVerbs {
    private static final int BIT1 = 0x0001;
    private static final int BIT2 = 0x0002;
    private static final int BIT4 = 0x0008;
    private static final int BIT5 = 0x0010;
    private static final int BIT14 = 0x2000;
    private static final int OC24100 = 024100;
    private static final int OCT24 = 024;
    private static final int OCT30 = 030;
    private static final int OCT50 = 050;
    private static final int OCT60000 = 060000 & Agc.WORD_MASK;
    private static final int OPIMDIFF = -71;

    /** Verb/noun codes used by extended verbs. */
    public static final int VNLODCDU = 2522;
    public static final int IMUCOARV = 4100;
    public static final int VNLD0CDU = 2492;
    public static final int OPTCOARV = IMUCOARV;
    public static final int VNLODGYR = 2593;
    public static final int IMUFINEV = 4200;
    public static final int VNLODDT = 2524;
    public static final int V04N46 = 446;
    public static final int V06N47 = 647;
    public static final int V0648 = 648;
    public static final int VNCON = 501;
    public static final int DEC23 = 23;

    /** The LST2FAN table: verb 40..99 -> label of the routine, in order. */
    public static final String[] LST2FAN = {
        "VBZERO", "VBCOARK", "IMUFINEK", "IMUATTCK", "SETSURF", "RESTSRF", "STABLISH", "LMTOCMSV",
        "DAPDISP", "CREWMANU", "GOLOADLV", "GOLOADLV", "V52", "GOLOADLV", "GOTOR23", "ALINTIME",
        "TRACKTRM", "GOTOR21", "ENATMA", "GOLOADLV", "V60", "V61", "V62", "V63",
        "VB64", "CKOPTVB", "ATTACHED", "V67", "STROKON", "VERB69", "V70UPDAT", "V71UPDAT",
        "V72UPDAT", "V73UPDAT", "DNEDUMP", "LFTFLGON", "SETPRFLG", "RESETPRF", "CHAZFOGC", "ALM/END",
        "LEMVEC", "CSMVEC", "V82PERF", "V83PERF", "ALM/END", "V85PERF", "V86PERF", "SETVHFLG",
        "RESETVHF", "V89PERF", "V90PERF", "GOSHOSUM", "SYSTEST", "WMATRXNG", "VERB94", "ALM/END",
        "VERB96", "GOLOADLV", "ALM/END", "GOLOADLV"
    };

    private final Agc agc;
    private final UpdateProgram update;

    public ExtendedVerbs(Agc agc) {
        this.agc = agc;
        this.update = new UpdateProgram(agc);
    }

    /** Hook for calls to routines outside this file; the default implementation does nothing. */
    protected void extern(String routine, Object... args) {
    }

    // ---- flag helpers (UPFLAG / DOWNFLAG take the flag name in the following ADRES) ----

    private void upflag(String flag) {
        agc.set(flag, 1);
    }

    private void downflag(String flag) {
        agc.set(flag, 0);
    }

    // ---- dispatch ----

    /** GOEXTVB: dispatch on verb-40 held in MPAC (INDEX MPAC; TC LST2FAN). */
    public void goextvb() {
        int idx = agc.get("MPAC");
        if (idx < 0 || idx >= LST2FAN.length) {
            almEnd();
            return;
        }
        switch (LST2FAN[idx]) {
            case "VBZERO": vbzero(); break;
            case "VBCOARK": vbcoark(); break;
            case "IMUFINEK": imufinek(); break;
            case "IMUATTCK": imuattck(); break;
            case "SETSURF": setsurf(); break;
            case "RESTSRF": restsrf(); break;
            case "STABLISH": stablish(); break;
            case "LMTOCMSV": lmtocmsv(); break;
            case "DAPDISP": dapdisp(); break;
            case "CREWMANU": crewmanu(); break;
            case "GOLOADLV": goloadlv(); break;
            case "V52": v52(); break;
            case "GOTOR23": gotor23(); break;
            case "ALINTIME": alintime(); break;
            case "TRACKTRM": tracktrm(); break;
            case "GOTOR21": gotor21(); break;
            case "ENATMA": enatma(); break;
            case "V60": v60(); break;
            case "V61": v61(); break;
            case "V62": v62(); break;
            case "V63": v63(); break;
            case "VB64": vb64(); break;
            case "CKOPTVB": ckoptvb(); break;
            case "ATTACHED": attached(); break;
            case "V67": v67(); break;
            case "STROKON": strokon(); break;
            case "VERB69": verb69(); break;
            case "V70UPDAT": update.v70updat(); break;
            case "V71UPDAT": update.v71updat(); break;
            case "V72UPDAT": update.v72updat(); break;
            case "V73UPDAT": update.v73updat(); break;
            case "DNEDUMP": dnedump(); break;
            case "LFTFLGON": lftflgon(); break;
            case "SETPRFLG": setprflg(); break;
            case "RESETPRF": resetprf(); break;
            case "CHAZFOGC": chazfogc(); break;
            case "LEMVEC": lemvec(); break;
            case "CSMVEC": csmvec(); break;
            case "V82PERF": v82perf(); break;
            case "V83PERF": v83perf(); break;
            case "V85PERF": v85perf(); break;
            case "V86PERF": v86perf(); break;
            case "SETVHFLG": setvhflg(); break;
            case "RESETVHF": resetvhf(); break;
            case "V89PERF": v89perf(); break;
            case "V90PERF": v90perf(); break;
            case "GOSHOSUM": showsum(); break;
            case "SYSTEST": systest(); break;
            case "WMATRXNG": wmatrxng(); break;
            case "VERB94": verb94(); break;
            case "VERB96": verb96(); break;
            default: almEnd(); break;
        }
    }

    // ---- display-system interlock ----

    /** TESTXACT: grab the extended verb display system if free, else operator error. */
    public boolean testxact() {
        if (agc.get("EXTVBACT") != 0) {
            almEnd();
            return false;
        }
        if ((agc.get("FLAGWRD4") & OC24100) != 0) { // priority displays using DSKY
            almEnd();
            return false;
        }
        setxtact(OCT24);
        // CS TWO; NVSUB: blank everything except MM and verb
        extern("NVSUB", -2);
        return true;
    }

    /** SETXTACT: set EXTVBACT to show the extended verb display is in use. */
    public void setxtact(int value) {
        agc.set("EXTVBACT", value);
    }

    /** XACTALM: operator error light then ENDEXT. */
    public void xactalm() {
        extern("FALTON");
        endext();
    }

    /** TERMEXTV / ENDEXTVB = ENDEXT: release mark and extended verb display system. */
    public void endext() {
        extern("ENDEXT");
    }

    /** XACT0: release mark and extended verb display system (EXTVBACT = 0). */
    public void xact0() {
        setxtact(0);
    }

    /** ALM/END: operator error light, then ENDEXT. */
    public void almEnd() {
        extern("FALTON");
    }

    /** GOPIN: POSTJUMP to PINBRNCH. */
    public void gopin() {
        extern("PINBRNCH");
    }

    // ---- IMU zero / coarse / fine align ----

    /** OP/INERT: noun check; returns 1 for N20 (inertial CDUs), 2 for N91 (optics CDUs), 0 if illegal. */
    public int opInert() {
        int noun = agc.get("NOUNREG");
        if (noun == OCT24) {
            return 1;                    // N20 (octal 24): ICDU
        }
        if (noun == 91) {
            return 2;
        }
        almEnd();
        return 0;
    }

    /** VBZERO: V40 zero the ISS CDUs (N20 only). */
    public void vbzero() {
        int n = opInert();
        if (n == 1) {
            imuzerok();
        } else {
            almEnd();
        }
    }

    /** IMUZEROK: keyboard request for ISS CDU zero. */
    public void imuzerok() {
        if (!ckmodcad()) {
            return;
        }
        extern("IMUZERO");
        extern("IMUSTALL");
        gopin();
    }

    /** VBCOARK: V41 coarse align (N20 inertial, N91 optics). */
    public void vbcoark() {
        int n = opInert();
        if (n == 1) {
            imucoark();
        } else if (n == 2) {
            optcoark();
        }
    }

    /** IMUCOARK: coarse align the IMU from the keyboard. */
    public void imucoark() {
        if (!ckmodcad() || !testxact()) {
            return;
        }
        extern("GOXDSPF", VNLODCDU);    // call for THETAD load; TERMINATE -> TERMEXTV
        icork2();
    }

    /** ICORK2: redisplay the coarse align verb, call IMUCOARS, stall until done. */
    public void icork2() {
        extern("EXDSPRET", IMUCOARV);
        extern("IMUCOARS");
        extern("IMUSTALL");
        endext();
    }

    /** OPTCOARK: coarse align the optics (N91). */
    public void optcoark() {
        if (!ckmodcad(agc.get("OPTCADR")) || !testxact()) {
            return;
        }
        agc.set("EBANK", agc.get("EBANK5"));
        if (Agc.toSigned(agc.get("SWSAMPLE")) <= 0) { // switch not at computer (CCS non-positive)
            extern("FALTON");
            extern("ALARM", 0115);
        }
        int optind = Agc.toSigned(agc.get("OPTIND"));
        if (optind < 0 || (optind == 0 && agc.get("OPTIND") != 0)) {
            extern("ALARM", 0117);       // optics reserved (OPTIND = -0)
            endext();
            return;
        }
        optc1();
    }

    /** OPTC1: load optics CDUs (V25N92), set DESOPTS/DESOPTT and start coarse working. */
    public void optc1() {
        extern("GOXDSPF", VNLD0CDU);
        agc.set("DESOPTS", agc.get("SAC"));
        agc.set("DESOPTT", agc.get("PAC"));
        extern("EXDSPRET", OPTCOARV);
        agc.set("OPTIND", 1);            // set coarse working
        endext();
    }

    /** IMUFINEK: V42 fine align with gyro torquing. */
    public void imufinek() {
        if (!ckmodcad() || !testxact()) {
            return;
        }
        extern("GOXDSPF", VNLODGYR);     // call for load of gyro commands
        extern("EXDSPRET", IMUFINEV);
        extern("IMUFINE");
        extern("IMUSTALL");
        endext();
    }

    /** FINEK2: pulse the gyros with commands left in OGC registers by PINBALL. */
    public void finek2() {
        extern("IMUPULSE", "OGC");
        extern("IMUSTALL");
        endext();
    }

    /** CKMODCAD: returns true if MODECADR is free (zero), else operator error. */
    public boolean ckmodcad() {
        return ckmodcad(agc.get("MODECADR"));
    }

    private boolean ckmodcad(int modecadr) {
        if (agc.get("MODECADR") == 0) {
            return true;
        }
        almEnd();
        return false;
    }

    /** GOLOADLV: V50,51,53,59,97,99: flash off, turn on FE7 and go to LOADLV1. */
    public void goloadlv() {
        extern("FLASHOFF");
        agc.set("SUPERBNK", agc.get("PINSUPBT"));
        extern("LOADLV1");
    }

    // ---- attitude display verbs ----

    /** V60: set CPHIX (N17) equal to present CDU angles. */
    public void v60() {
        agc.set("CPHIX", agc.get("CDUX"));
        agc.set("CPHIX+1", agc.get("CDUX+1"));
        agc.set("CPHIX+2", agc.get("CDUZ"));
        gopin();
    }

    /** V61: mode I - phase plane A/P following error displayed. */
    public void v61() {
        downflag("NEEDLFLG");
        gopin();
    }

    /** V62: mode II - total attitude error with respect to N22. */
    public void v62() {
        upflag("NEEDLFLG");
        upflag("N22ORN17");
        gopin();
    }

    /** V63: mode III - total attitude error with respect to N17. */
    public void v63() {
        upflag("NEEDLFLG");
        downflag("N22ORN17");
        gopin();
    }

    /** ALINTIME: V55 align time; request delta time (V25N24) and increment the AGC clock. */
    public void alintime() {
        if (!testxact()) {
            return;
        }
        extern("GOMARKF", VNLODDT);
        if (agc.get("MPAC") != DEC23) {
            endext();
            return;
        }
        updatime();
    }

    /** UPDATIME: add DSPTEM1/+1 delta time (centiseconds, DP) to TIME2/TIME1 with sign agreement. */
    public void updatime() {
        long clock = ((long) Agc.toSigned(agc.get("TIME2")) << 14) + Agc.toSigned(agc.get("TIME1"));
        long delta = ((long) Agc.toSigned(agc.get("DSPTEM2+1")) << 14) + Agc.toSigned(agc.get("DSPTEM2+2"));
        long sum = clock + delta;
        agc.set("TIME2", Agc.fromSigned((int) (sum >> 14)));
        agc.set("TIME1", Agc.fromSigned((int) (sum & 0x3FFF)));
        endext();
    }

    // ---- system test and prelaunch ----

    /** SYSTEST: V92 operate IMU performance test (not after P11 liftoff). */
    public void systest() {
        if (!chkpooh()) {
            return;
        }
        if ((agc.get("FLAGWRD1") & agc.get("NOP01BIT")) != 0) {
            extern("POODOO", 01521);     // NODO alarm for P07
        }
        exdapoff();
        extern("FINDVAC", 020, "REDO"); // PRIO20
        gopin();
    }

    /** CKOPTVB: V65 prelaunch optical verification (only during P02). */
    public void ckoptvb() {
        if (!checkmm(2)) {
            almEnd();
            return;
        }
        extern("FINDVAC", 016, "COMPVER"); // PRIO16
        gopin();
    }

    /** CHAZFOGC: V78 change gyrocompass launch azimuth (only during P02). */
    public void chazfogc() {
        if (!checkmm(2)) {
            almEnd();
            return;
        }
        extern("FINDVAC", 016, "AZMTHCG1");
        extern("PHASCHNG", 0174);
        gopin();
    }

    /** IMUATTCK: V43 load IMU attitude error meters. */
    public void imuattck() {
        if (!chkpooh()) {
            return;
        }
        if ((agc.get("CHAN12") & OCT30) != 0) {
            almEnd();                    // not allowed if IMU coarse or zero on
            return;
        }
        if (!ckLiftoffBits()) {          // CKLFTBTS: liftoff has occurred -> TESTXACT
            testxact();
        }
        agc.set("CHAN12", agc.get("CHAN12") & ~OCT50 & Agc.WORD_MASK); // remove coarse and ECTR enable
        extern("GOXDSPF", VNLODCDU);
        agc.set("EBANK", agc.get("EBANK6"));
        extern("NEEDLE11");              // initialise current DAC and command values
        extern("NEEDLER2");              // enable error counters
        extern("WAITLIST", 2, "ATTCK1"); // 4 ms minimum
        trmattck();
    }

    /** TRMATTCK. */
    public void trmattck() {
        if (ckLiftoffBits()) {
            return; // before liftoff: GOPIN
        }
        endext();
    }

    /** ATTCK1: waitlist task - transfer loaded THETAD to AK and send the commands to the needles. */
    public void attck1() {
        agc.set("AK", agc.get("THETAD"));
        agc.set("AK+1", agc.get("THETAD+1"));
        agc.set("AK+2", agc.get("THETAD+2"));
        extern("NEEDLES");               // limited to +-384 pulses; counters left enabled
    }

    /** CKLFTBTS: returns true if liftoff has not occurred (so caller takes before-liftoff path). */
    public boolean ckLiftoffBits() {
        if ((agc.get("GRRBKBIT") & agc.get("FLAGWRD5")) != 0) {
            return false;                // liftoff occurred (GRR backup flag)
        }
        return (agc.get("CHAN30") & BIT5) != 0 ? false : true;
    }

    // ---- misc verbs ----

    /** VB64: V64 calculate and display S-band antenna angles (P00 only). */
    public void vb64() {
        if (!chkpooh() || !testxact()) {
            return;
        }
        extern("FINDVAC", 4, "SBANDANT");
    }

    /** ENATMA: V58 enable automatic attitude maneuver (reset STIKFLAG). */
    public void enatma() {
        downflag("STIKFLAG");
        gopin();
    }

    /** STROKON: V68 TVC stroke test on (only in TVC mode); job STRKTSTI. */
    public void strokon() {
        if (!isTvc()) {
            almEnd();
            return;
        }
        extern("NOVAC", 030, "STRKTSTI");
        gopin();
    }

    /** TVC mode test: CS FLAGWRD6, MASK OCT60000 positive non-zero (bit 15 set, bit 14 clear). */
    private boolean isTvc() {
        int comp = ~agc.get("FLAGWRD6") & OCT60000 & Agc.WORD_MASK;
        return comp != 0 && (comp & Agc.SIGN_BIT) == 0;
    }

    /** STABLISH: V46 establish G&C control (DAP configuration). */
    public void stablish() {
        agc.set("EBANK", agc.get("EBANK6"));
        if (isTvc()) {
            if ((agc.get("DAPDATR1") & BIT14) != 0) {
                extern("PRESWTCH");      // CSM/LM: HB to LB switch-over
            } else {
                almEnd();                // CSM only: alarm and exit
            }
            return;
        }
        extern("DAPFIG");                // set up RCS, entry or Saturn-stick DAP
    }

    /** CREWMANU: V49 start automatic attitude maneuver (R62). */
    public void crewmanu() {
        if (!chkpooh() || !testxact()) {
            return;
        }
        extern("FINDVAC", 010, "R62DISP");
    }

    /** DAPDISP: V48 load DAP data (R03). Not allowed in TVC. */
    public void dapdisp() {
        if (isTvc()) {
            almEnd();                    // not permitted in TVC
            return;
        }
        if (!testxact()) {
            return;
        }
        dapdisp1();
    }

    /** DAPDISP1: N46 (DAPDATR1/2), N47 (weights), N48 (trims) displays and mass properties. */
    public void dapdisp1() {
        agc.set("EBANK", agc.get("EBANK6"));
        extern("PRIOCHNG", 010);
        // DONOUN46: R1 = DAPDATR1, R2 = DAPDATR2
        extern("GOXDSPF", V04N46);
        if ((agc.get("DAPDATR1") & BIT4) != 0) {
            maxin();
        } else {
            downflag("MAXDBFLG");
        }
        maxout();
    }

    /** MAXIN: set MAXDBFLG. */
    public void maxin() {
        upflag("MAXDBFLG");
        maxout();
    }

    /** MAXOUT: S41.2, then N47 (CSM/LM weights) and mass properties. */
    public void maxout() {
        extern("S41.2");
        extern("GOXDSPF", V06N47);       // CSM weight, LEM weight
        int cfg = agc.get("DAPDATR1") & 030;        // PRIO30 mask: configuration bits
        boolean doMass = !(cfg == 0 || cfg == 030);   // skip if 0,4 or 3,7
        if (doMass) {
            extern("MASSPROP");          // update IXX, IAVG, IAVG/TLX
            extern("S40.14");            // compute RCS DAP stuff
        }
        extern("GOXDSPF", V0648);        // PTRIM, YTRIM
    }

    // ---- R3x performance verbs ----

    /** V82PERF: V82 orbit parameter display (R30). */
    public void v82perf() {
        if (!testxact()) {
            return;
        }
        extern("PRIOCHNG", 7);
        extern("V82CALL");               // must not be a FINDVAC job
    }

    /** V83PERF: V83 range, range rate and +X axis (R31). */
    public void v83perf() {
        if (!testxact()) {
            return;
        }
        agc.set("FLAGWRD9", agc.get("FLAGWRD9") | agc.get("R31FLBIT"));
        extern("NOVAC", 5, "R31CALL");
    }

    /** V85PERF: V85 range, range rate and SLOS (R32): R31 flag cleared then as V83. */
    public void v85perf() {
        if (!testxact()) {
            return;
        }
        agc.set("FLAGWRD9", agc.get("FLAGWRD9") & ~agc.get("R31FLBIT") & Agc.WORD_MASK);
        extern("NOVAC", 5, "R31CALL");
    }

    /** GOTOR21: V57 rendezvous track sight mark routine (R21). */
    public void gotor21() {
        downflag("R23FLG");
        r2xCommon();
    }

    /** GOTOR23: V54 mark (R23 backup). */
    public void gotor23() {
        upflag("R23FLG");
        r2xCommon();
    }

    private void r2xCommon() {
        if (!testxact()) {
            return;
        }
        if ((agc.get("FLAGWRD0") & agc.get("RNDVZBIT")) == 0
                || (agc.get("FLAGWRD1") & agc.get("TRACKBIT")) == 0) {
            extern("ALARM", 0406);       // neither rendezvous nor track flag on
            endext();
            return;
        }
        if ((agc.get("FLAGWRD1") & agc.get("R23BIT")) == 0) {
            extern("NOVAC", 016, "R21CSM");
        } else {
            extern("NOVAC", 016, "R23CSM");
        }
    }

    /** V86PERF: V86 backup mark reject (R23): MRKBUF1 = -1. */
    public void v86perf() {
        agc.set("EBANK", agc.get("EBANK7"));
        agc.set("MRKBUF1", Agc.fromSigned(-1));
        gopin();
    }

    /** TRACKTRM: V56 terminate tracking (P20 and P25). */
    public void tracktrm() {
        if ((agc.get("FLAGWRD0") & agc.get("RNDVZBIT")) == 0) {
            gopin();
            return;
        }
        downflag("RNDVZFLG");
        if ((agc.get("FLAGWRD1") & agc.get("TRACKBIT")) == 0) {
            gopin();
            return;
        }
        downflag("TRACKFLG");
        downflag("UPDATFLG");
        downflag("IMUSE");
        agc.set("EBANK", agc.get("EBANK6"));
        extern("STOPRATE");
        agc.set("OPTIND", Agc.fromSigned(-1));
        extern("INTSTALL");
        extern("2PHSCHNG", 2, 1);        // kill groups 2 and 1 to halt P20 activity
        cleanout();
    }

    /** CLEANOUT: cause a restart (ENEMA). */
    public void cleanout() {
        extern("ENEMA");
    }

    /** LEMVEC: V80 update LEM state vector (VEHUPFLG down). */
    public void lemvec() {
        downflag("VEHUPFLG");
        gopin();
    }

    /** CSMVEC: V81 update CSM state vector (VEHUPFLG up). */
    public void csmvec() {
        upflag("VEHUPFLG");
        gopin();
    }

    /** DNEDUMP / V74: initialise the down-telemetry dump program. */
    public void dnedump() {
        agc.set("DNTMGOTO", 0 /* REMADR DNDUMPI */);
        gopin();
    }

    /** LFTFLGON: V75 set liftoff flag (GRRBKFLG). */
    public void lftflgon() {
        upflag("GRRBKFLG");
        gopin();
    }

    /** CHKPOOH: demand program 00; operator error otherwise. */
    public boolean chkpooh() {
        if (agc.get("MODREG") == 0) {
            return true;
        }
        almEnd();
        return false;
    }

    /** CHECKMM: returns true if the current major mode equals mm. */
    private boolean checkmm(int mm) {
        return Agc.toSigned(agc.get("MODREG")) == mm;
    }

    /** EXDAPOFF: set T5 to idle (T5IDLOC) and reset the DAP bits 1 and 2 of FLAGWRD6. */
    public void exdapoff() {
        agc.set("T5LOC", 0 /* 2CADR T5IDLOC */);
        agc.set("FLAGWRD6", agc.get("FLAGWRD6") & ~OCT60000 & Agc.WORD_MASK);
    }

    /** V89PERF: V89 align X or PRF CSM axis to LOS (R63). */
    public void v89perf() {
        if (!chkpooh() || !testxact()) {
            return;
        }
        extern("FINDVAC", 010, "V89CALL");
    }

    /** WMATRXNG: V93 reset RENDWFLG and ORBWFLAG. */
    public void wmatrxng() {
        downflag("RENDWFLG");
        downflag("ORBWFLAG");
        gopin();
    }

    // ---- SHOWSUM ----

    /** SHOWSUM / GOSHOSUM: V91 display bank checksums (rope check). */
    public void showsum() {
        if (!chkpooh() || !testxact()) {
            return;
        }
        agc.set("SKEEP6", 1);            // SHOWSUM option
        agc.set("SMODE", 0);             // turn off self-check
        agc.set("SELFRET", agc.get("SELFADRS"));
        extern("STSHOSUM");
    }

    /** SDISPLAY / NOKILL: display bank number (SKEEP2) and bugger word (SKEEP3) with V05N01. */
    public void sdisplay() {
        agc.set("MPAC+2", agc.get("ADRS1"));
        extern("GOXDSPF", VNCON);        // PROCEED: NXTBNK ; recycle: NOKILL
        agc.set("SKEEP1", agc.get("SELFADRS"));
        endext();
    }

    /** ENDSUMS: restart SELFCHK (if ROPECHK) or SHOWSUM. */
    public void endsums() {
        if (agc.get("SKEEP6") == 0) {
            extern("SELFCHK");
        } else {
            extern("STSHOSUM");
        }
    }

    // ---- more flag verbs ----

    public void setsurf() {
        upflag("SURFFLAG");
        gopin();
    }

    public void restsrf() {
        downflag("SURFFLAG");
        gopin();
    }

    public void setprflg() {
        upflag("PRFTRKAT");
        gopin();
    }

    public void resetprf() {
        downflag("PRFTRKAT");
        gopin();
    }

    public void setvhflg() {
        upflag("VHFRFLAG");
        gopin();
    }

    public void resetvhf() {
        downflag("VHFRFLAG");
        extern("TRFAILOF");              // tracker fail light off
        gopin();
    }

    /** VERB69: V69 cause restart (loops until the hardware restart). */
    public void verb69() {
        extern("RESTART");
    }

    /** VERB96: V96 set QUITFLAG to stop integration and go to P00. */
    public void verb96() {
        upflag("QUITFLAG");
        upflag("V96ONFLG");
        extern("V37", 0);
    }

    /** VERB94: V94 do R64 (only in P23 and after V94FLAG set). */
    public void verb94() {
        if ((agc.get("FLAGWRD9") & agc.get("V94FLBIT")) == 0) {
            almEnd();
            return;
        }
        downflag("V94FLAG");
        if (!checkmm(23)) {
            almEnd();
            return;
        }
        extern("PHASCHNG", 0112);        // set group 2 to do R64
        cleanout();
    }

    /** V90PERF: V90 out-of-plane parameters (R36). */
    public void v90perf() {
        if (!testxact()) {
            return;
        }
        extern("FINDVAC", 7, "R36");
    }

    /** V52: set offset number for P22 (landmark) from MARKSTAT. */
    public void v52() {
        if (!checkmm(22)) {
            almEnd();
            return;
        }
        agc.set("EBANK", agc.get("LANDBANK"));
        int markstat = agc.get("MARKSTAT");
        int offset = 5 - ((markstat >> 12) & 07);  // bits 14-12 of MARKSTAT subtracted from 5
        int landmark = agc.get("LANDMARK") & ~(07 << 9) & Agc.WORD_MASK;
        agc.set("LANDMARK", landmark + (offset << 9));
        gopin();
    }

    /** V67: WMATRIX monitor job. */
    public void v67() {
        if (!testxact()) {
            return;
        }
        extern("FINDVAC", 5, "V67CALL");
    }

    // ---- ATTACHED / LMTOCM ----

    /** ATTACHED: V66 state vector of this vehicle copied to the other vehicle. */
    public void attached() {
        extern("FINDVAC", 010, "ATTACHIT");
    }

    /** ATTACHIT: interpretive; copy RRECTHIS into RRECTOTH via GENTRAN. */
    public void attachit() {
        extern("INTSTALL");
        if (agc.get("MOONOTH") != 0) {
            // BON MOONTHIS: MOONOTH stays set when MOONTHIS is set
            if (agc.get("MOONTHIS") == 0) {
                agc.set("MOONOTH", 0);   // else clear
            }
        } else {
            agc.set("MOONOTH", 1);
        }
        extern("GENTRAN", 051, "RRECTHIS", "RRECTOTH");
        tachexit();
    }

    /** LMTOCMSV: V47 move LM state vector into CM. */
    public void lmtocmsv() {
        extern("FINDVAC", 010, "LMTOCM");
    }

    /** LMTOCM: copy RRECTOTH into RRECTHIS. */
    public void lmtocm() {
        extern("INTSTALL");
        if (agc.get("MOONTHIS") != 0 && agc.get("MOONOTH") != 0) {
            // both on: leave MOONTHIS
        } else {
            agc.set("MOONTHIS", agc.get("MOONOTH") != 0 ? 1 : 0);
        }
        extern("GENTRAN", 051, "RRECTOTH", "RRECTHIS");
        tachexit();
    }

    /** TACHEXIT: update RN, VN, R-OTHER, V-OTHER (PTOACSM), SVDWN1/2, then INTWAKE via TCPIN. */
    public void tachexit() {
        extern("PTOACSM");
        extern("SVDWN1", "PBODY");
        extern("SVDWN2");
        agc.set("QPRET", 0 /* TCPIN: RTB PINBRNCH */);
        extern("INTWAKE");
    }
}
