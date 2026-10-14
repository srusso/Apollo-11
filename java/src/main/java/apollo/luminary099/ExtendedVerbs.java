package apollo.luminary099;

import apollo.agc.Agc;

/**
 * Translation of EXTENDED_VERBS.agc (Luminary 1A build 099).
 *
 * The extended verb (V40-V99) dispatch table (LST2FAN) and the routines for the non-update
 * extended verbs: IMU zero/coarse/fine align, attitude error meters, rendezvous and landing
 * radar tests (R04/R77), R33 (align time), R03 (DAP data load) with TRIMGIMB hand-off,
 * state vector moves, SHOWSUM, RR LOS display, etc.
 *
 * External routines (BANKCALL, FINDVAC, WAITLIST, NVSUB, display and mode-switching
 * programs) are routed through {@link #extern(String, Object...)}, which a host simulator
 * may override.
 */
public class ExtendedVerbs {
    private static final int BIT1 = 0x0001;
    private static final int BIT2 = 0x0002;
    private static final int BIT3 = 0x0004;
    private static final int BIT5 = 0x0010;
    private static final int BIT6 = 0x0020;
    private static final int BIT8 = 0x0080;
    private static final int BIT13 = 0x1000;
    private static final int BIT14 = 0x2000;
    private static final int BIT15 = 0x4000;
    private static final int OC24100 = 024100;
    private static final int OCT24 = 024;
    private static final int OCT41000 = 041000;
    private static final int LRPOSCAL = 0444;
    private static final int RRIMUDIF = -52;
    private static final int DEC22 = 22;
    private static final int DEC23 = 23;
    private static final int BITS9_7 = 0500;
    private static final int DSPLYMSK = 033113;
    private static final int BOOLSMSK = 013113;
    private static final int BITS4_5 = 030;
    private static final int BIT13_14 = BIT13 | BIT14;
    private static final int C13_14_15 = 070000;
    private static final int ONE_SEC_PLUS1 = 101;
    private static final int MS250_PLUS1 = 26;

    /** Verb/noun codes. */
    public static final int VNLODCDU = 2522;
    public static final int IMUCOARV = 4100;
    public static final int VNLDRCDU = 2473;
    public static final int OPTCOARV = IMUCOARV;
    public static final int V04N1272 = 412;
    public static final int VNLODGYR = 2593;
    public static final int IMUFINEV = 4200;
    public static final int VNLODDT = 2524;
    public static final int V16N72 = 1672;
    public static final int V16N78 = 1678;
    public static final int V16N66 = 1666;
    public static final int V16N67 = 1667;
    public static final int V04N12X = 412;
    public static final int V50N25X = 5025;
    public static final int V43K = 4300;
    public static final int V01N46 = 146;
    public static final int V0647 = 647;
    public static final int V06N48 = 648;
    public static final int V50N48 = 5048;
    public static final int V16N56 = 1656;
    public static final int VNCON = 501;
    public static final int R201R04 = 00201;

    /** DAP stick scaling constants (R03). */
    public static final double NORMAL = .660214;     // 20 D/S
    public static final double FINE = .165054;       // 4 D/S
    public static final double ONE_TENTH = .1;       // CSM-docked scaling
    public static final int MINUS_0_6_DS = -218;
    public static final int MINUS_0_3_DS = -109;
    public static final double ATTSCALE = 0.1;

    /** The LST2FAN table: verb 40..99 -> label of the routine, in order. */
    public static final String[] LST2FAN = {
        "VBZERO", "VBCOARK", "IMUFINEK", "IMUATTCK", "RRDESEND", "ALM/END", "ALM/END", "V47TXACT",
        "DAPDISP", "CREWMANU", "GOLOADLV", "ALM/END", "GOLOADLV", "GOLOADLV", "GOLOADLV", "ALINTIME",
        "TRMTRACK", "LRON", "LROFF", "ALM/END", "LRPOS2K", "DAPATTER", "TOTATTER", "R04",
        "VB64", "SNUFFOUT", "ATTACHED", "V67", "ALM/END", "VERB69", "V70UPDAT", "V71UPDAT",
        "V72UPDAT", "V73UPDAT", "DNEDUMP", "OUTSNUFF", "MINIMP", "NOMINIMP", "R77", "R77END",
        "LEMVEC", "CSMVEC", "V82PERF", "V83PERF", "ALM/END", "VERB85", "ALM/END", "ALM/END",
        "ALM/END", "V89PERF", "V90PERF", "GOSHOSUM", "SYSTEST", "WMATRXNG", "ALM/END", "UPDATOFF",
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

    private boolean flag(String f) {
        return agc.get(f) != 0;
    }

    private void upflag(String f) {
        agc.set(f, 1);
    }

    private void downflag(String f) {
        agc.set(f, 0);
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
            case "RRDESEND": rrdesend(); break;
            case "V47TXACT": v47txact(); break;
            case "DAPDISP": dapdisp(); break;
            case "CREWMANU": crewmanu(); break;
            case "GOLOADLV": goloadlv(); break;
            case "ALINTIME": alintime(); break;
            case "TRMTRACK": trmtrack(); break;
            case "LRON": lron(); break;
            case "LROFF": lroff(); break;
            case "LRPOS2K": lrpos2k(); break;
            case "DAPATTER": dapatter(); break;
            case "TOTATTER": totatter(); break;
            case "R04": r04(); break;
            case "VB64": vb64(); break;
            case "SNUFFOUT": snuffout(); break;
            case "ATTACHED": attached(); break;
            case "V67": v67(); break;
            case "VERB69": verb69(); break;
            case "V70UPDAT": update.v70updat(); break;
            case "V71UPDAT": update.v71updat(); break;
            case "V72UPDAT": update.v72updat(); break;
            case "V73UPDAT": update.v73updat(); break;
            case "DNEDUMP": dnedump(); break;
            case "OUTSNUFF": outsnuff(); break;
            case "MINIMP": minimp(); break;
            case "NOMINIMP": nominimp(); break;
            case "R77": r77(); break;
            case "R77END": r77end(); break;
            case "LEMVEC": lemvec(); break;
            case "CSMVEC": csmvec(); break;
            case "V82PERF": v82perf(); break;
            case "V83PERF": v83perf(); break;
            case "VERB85": verb85(); break;
            case "V89PERF": v89perf(); break;
            case "V90PERF": v90perf(); break;
            case "GOSHOSUM": showsum(); break;
            case "SYSTEST": systest(); break;
            case "WMATRXNG": wmatrxng(); break;
            case "UPDATOFF": updatoff(); break;
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
        if ((agc.get("FLAGWRD4") & OC24100) != 0) {  // priority displays using DSKY
            almEnd();
            return false;
        }
        setxtact(OCT24);                             // set bits 3 and 5
        extern("NVSUB", -2);                         // blank everything except MM and verb
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

    /** ALM/END: operator error light, then (fall into) GOPIN. */
    public void almEnd() {
        extern("FALTON");
    }

    /** GOPIN: POSTJUMP to PINBRNCH. */
    public void gopin() {
        extern("PINBRNCH");
    }

    /** CHKPOOH: returns normally only if running P00 or P00-; otherwise operator error. */
    public boolean chkpooh() {
        if (agc.get("MODREG") == 0) {
            return true;
        }
        almEnd();
        return false;
    }

    /** CKMODCAD: operator error if MODECADR is in use. */
    public boolean ckmodcad() {
        if (agc.get("MODECADR") == 0) {
            return true;
        }
        almEnd();
        return false;
    }

    /** OP/INERT: noun check; returns 1 for N20 (ICDU), 2 for N72 (RCDU), 0 if illegal. */
    public int opInert() {
        int noun = agc.get("NOUNREG");
        if (noun == OCT24) {
            return 1;
        }
        if (noun - OCT24 + RRIMUDIF == 0) {          // RR CDUs (N72)
            return 2;
        }
        almEnd();
        return 0;
    }

    // ---- radar use checks ----

    /** RDRUSECK: returns false (after alarm) if the radars are in use by R29, R77, R12, P20, P22 or P25. */
    public boolean rdruseck() {
        return rdrusecK();
    }

    private boolean rdrusecK() {
        if ((~agc.get("FLAGWRD3") & agc.get("NR29FBIT")) != 0) {   // R29 on
            almEnd();
            return false;
        }
        if ((agc.get("FLAGWRD5") & agc.get("R77FLBIT")) != 0) {    // R77 running
            almEnd();
            return false;
        }
        // servicer running and possibly R12 using LR?
        if ((~agc.get("FLAGWRD7") & agc.get("V37FLBIT")) == 0) {
            if ((~agc.get("FLGWRD11") & agc.get("LRBYBIT")) == 0) { // R12 on
                almEnd();
                return false;
            }
        }
        // CHECKRR: track flag and possibly P20 using the RR
        if ((~agc.get("FLAGWRD1") & agc.get("TRACKBIT")) == 0) {
            if ((agc.get("FLAGWRD0") & agc.get("RNDVZBIT")) != 0) { // CKRNDBIT: P25?
                almEnd();
                return false;
            }
        }
        // CHECKP22
        if (agc.get("MODREG") == DEC22) {
            almEnd();
            return false;
        }
        return true;
    }

    // ---- IMU zero / coarse / fine align ----

    /** VBZERO (V40): IMU CDU zero (N20) or RR CDU zero (N72). */
    public void vbzero() {
        switch (opInert()) {
            case 1:                                     // IMUZEROK
                if (!ckmodcad()) {
                    return;
                }
                extern("IMUZERO");
                extern("IMUSTALL");
                gopin();
                break;
            case 2:                                     // RRZEROK
                if (!rdrusecK()) {
                    return;
                }
                extern("RRZERO");
                extern("RADSTALL");
                gopin();
                break;
            default:
                break;
        }
    }

    /** LRPOS2K (V60): command LR to position 2. */
    public void lrpos2k() {
        if (!rdrusecK()) {
            return;
        }
        extern("LRPOS2");
        extern("RADSTALL");
        lrp2alm();                                     // BADEND path; GOODEND goes to GOPIN
    }

    /** LRP2ALM: alarm 523 then GOPIN. */
    public void lrp2alm() {
        extern("ALARM", 0523);
        gopin();
    }

    /** DAPATTER (V61): display DAP attitude error. */
    public void dapatter() {
        downflag("NEEDLFLG");
        gopin();
    }

    /** TOTATTER (V62): display total attitude error. */
    public void totatter() {
        upflag("NEEDLFLG");
        gopin();
    }

    /** VBCOARK (V41): coarse align of ICDU (N20) or designate RR (N72). */
    public void vbcoark() {
        switch (opInert()) {
            case 1: imucoark(); break;
            case 2: rrdesnbk(); break;
            default: break;
        }
    }

    /** IMUCOARK: coarse align from the keyboard. */
    public void imucoark() {
        if (!ckmodcad() || !testxact()) {
            return;
        }
        extern("GOXDSPF", VNLODCDU);                    // call for THETAD load
        extern("EXDSPRET", IMUCOARV);                   // ICORK2: redisplay coarse align verb
        extern("IMUCOARS");                             // mode switching
        extern("IMUSTALL");
        endext();
    }

    /** RRDESNBK: designate RR to the CDU angles keyed in (V41 N72). */
    public void rrdesnbk() {
        if (!rdrusecK() || !testxact()) {
            return;
        }
        if ((agc.get("RNDVZBIT") & agc.get("FLAGWRD0")) != 0) {
            xactalm();                                  // operator error if in P20
            return;
        }
        agc.set("RADMODES", agc.get("RADMODES") & ~OCT41000);  // terminate present designation
        extern("GOXDSPF", VNLDRCDU);                    // ask for gimbal angles
        extern("AURLOKON");                             // ask for lock-on requirements
        extern("EXDSPRET", OPTCOARV);
        extern("FINDVAC", "RRDESK2", 020);
        endext();                                       // frees display
    }

    /** RRDESK2: job doing the designate and waiting on its completion. */
    public void rrdesk2() {
        extern("RRDESNB");
        extern("RADSTALL");                             // wait for completion of designate
        // BADEND (no lock-on or out of limits) alarms 503; GOODEND ends the job
        extern("ENDOFJOB");
    }

    /** RRDESEND (V44): terminate continuous designate only. */
    public void rrdesend() {
        if (Agc.toSigned(agc.get("RADMODES")) <= 0 && (agc.get("RADMODES") & OCT41000) == 0) {
            gopin();
            return;
        }
        agc.set("RADMODES", agc.get("RADMODES") & ~OCT41000);
        extern("CLRADMOD");
        extern("DELAYJOB", 100);
        downflag("NORRMON");                            // enable R25 gimbal monitor
        gopin();
    }

    /** AURLOKON: ask the operator for the lock-on option (V04N12). */
    public void aurlokon() {
        agc.set("OPTIONX+1", 2);
        agc.set("OPTIONX", 6);                          // option code for V04N12
        extern("GOMARKFR", V04N1272);                   // R2: 00001 lock-on
        if ((agc.get("OPTIONX+1") & BIT2) != 0) {
            // NOLOKON: continuous designation wanted (to be terminated by V44)
            downflag("LOKONSW");
            upflag("CDESFLAG");
            upflag("NORRMON");                          // disable R25 RR gimbal monitor in T4RUPT
        } else {
            upflag("LOKONSW");
        }
    }

    // ---- landing radar ----

    /** LRON (V57): permit incorporation of LR data. */
    public void lron() {
        upflag("LRINH");
        gopin();
    }

    /** LROFF (V58): inhibit incorporation of LR data. */
    public void lroff() {
        downflag("LRINH");
        gopin();
    }

    // ---- fine align ----

    /** IMUFINEK (V42): fine align with gyro torquing. */
    public void imufinek() {
        if (!ckmodcad() || !testxact()) {
            return;
        }
        extern("GOXDSPF", VNLODGYR);                    // call for load of gyro commands
        extern("EXDSPRET", IMUFINEV);                   // redisplay our own verb
        extern("IMUFINE");                              // mode switch program
        extern("IMUSTALL");
        finek2();
    }

    /** FINEK2: pulse the gyro commands left in OGC by PINBALL. */
    public void finek2() {
        extern("IMUPULSE", "OGC");
        extern("IMUSTALL");
        endext();
    }

    /** GOLOADLV: flash off, select PINSUPER, jump to LOADLV1. */
    public void goloadlv() {
        extern("FLASHOFF");
        extern("SUPERBNK", "PINSUPBT");
        extern("LOADLV1");
    }

    /** V47TXACT (V47): AGS initialization. */
    public void v47txact() {
        if (!testxact()) {
            return;
        }
        extern("FINDVAC", "AGSINIT", 04);
        extern("ENDOFJOB");
    }

    // ---- R33 align time ----

    /** ALINTIME (V55): align time. */
    public void alintime() {
        if (!testxact()) {
            return;
        }
        r33();
    }

    /** R33: load delta time (V25N24) and add it to AGC time. */
    public void r33() {
        extern("PRIOCHNG", 07);
        extern("GOXDSPF", VNLODDT);
        if (agc.get("MPAC") != DEC23) {                  // only data in (V23) continues
            endext();
            return;
        }
        // UPDATIME: DELTA TIME is in DSPTEM2+1 (DP)
        long inc = ((long) Agc.toSigned(agc.get("DSPTEM2+1")) << 14) + Agc.toSigned(agc.get("DSPTEM2+2"));
        long time = ((long) Agc.toSigned(agc.get("TIME2")) << 14) + Agc.toSigned(agc.get("TIME2+1"));
        time += inc;                                      // TPAGREE forces sign agreement
        agc.set("TIME2", Agc.fromSigned((int) (time >> 14)));
        agc.set("TIME2+1", Agc.fromSigned((int) (time & 037777)));
        endext();
    }

    // ---- R04 / R77 radar tests ----

    /** R77 (V78): start LR spurious return test. */
    public void r77() {
        if (!rdrusecK()) {
            return;
        }
        if ((agc.get("FLAGWRD3") & agc.get("R04FLBIT")) != 0) {   // R04 running
            almEnd();
            return;
        }
        upflag("R77FLAG");
        r04z();
    }

    /** R04 (V63): sample radar once per second. */
    public void r04() {
        if (!rdrusecK() || !testxact()) {
            return;
        }
        upflag("R04FLAG");                                // set R04FLAG for alarms
        r04z();
    }

    private void r04z() {
        agc.set("RSAMPDT", ONE_SEC_PLUS1);                // sample once per second
        agc.set("RTSTLOC", 0);
        agc.set("RFAILCNT", 0);                           // zero bad sample counter
        // initialize: bit 9 LR range low scale, bit 6 LR pos 1, bit 3 RR range low scale = 0
        int rm = agc.get("RADMODES") & ~LRPOSCAL;
        rm += agc.get("CHAN33") & LRPOSCAL;
        agc.set("RADMODES", rm);
        if (flag("R04FLAG") && (agc.get("FLAGWRD3") & agc.get("R04FLBIT")) != 0) {
            r04k();                                       // R04: ask 4 LR components/second? (see R04K)
            return;
        }
        agc.set("OPTIONX+1", 1);                          // rendezvous desired
        agc.set("OPTIONX", BIT3);                         // option code for V04N12
        extern("GOMARKFR", V04N12X);                      // R2: 00001 RR, 00002 LR
        agc.set("RTSTDEX", agc.get("OPTIONX+1"));         // save desired option RR=1 LR=2
        r04x();
    }

    private void r04x() {
        if ((6 & agc.get("RTSTDEX")) != 0) {              // LR desired
            r04l();
            return;
        }
        agc.set("RTSTBASE", 0);                           // RR: base 0, max 1
        r04b();
    }

    private void r04b() {
        if ((agc.get("CHAN33") & BIT2) != 0) {            // RR auto mode discrete absent
            agc.set("DSPTEM1", R201R04);
            extern("GOMARK4", V50N25X);                   // request selection of RR auto mode
        }
        r04c();
    }

    private void r04c() {
        agc.set("CHAN12", agc.get("CHAN12") | BIT14);     // enable RR auto tracker
        agc.set("RTSTMAX", 2);                            // for sequential storage
        extern("WAITLIST", 2, "RADSAMP");
        if (!flag("R04FLAG")) {
            gopin();                                      // R77
            return;
        }
        if ((6 & agc.get("RTSTDEX")) != 0) {
            r04lr();
        } else {
            r04rr();
        }
    }

    private void r04rr() {
        extern("GOMARKF", V16N72);                        // RR CDU angles (trunnion, shaft)
        extern("GOMARKF", V16N78);                        // RR range and range rate
        r04y();
    }

    private void r04lr() {
        extern("GOMARKF", V16N66);                        // LR range and position
        extern("GOMARKF", V16N67);                        // LR VELX, VELY, VELZ
        r04y();
    }

    /** R04Y: terminate sampling briefly and switch RR/LR. */
    private void r04y() {
        agc.set("RSAMPDT", 0);
        extern("DELAYJOB", 200);                          // wait for last RADARUP
        agc.set("RSAMPDT", ONE_SEC_PLUS1);
        agc.set("RTSTLOC", 0);
        agc.set("RTSTDEX", (6 & agc.get("RTSTDEX")) != 0 ? 1 : 2);   // toggle RR <-> LR
        r04x();
    }

    private void r04k() {
        agc.set("RSAMPDT", MS250_PLUS1);                  // sample 4 LR components per second
        r04l();
    }

    private void r04l() {
        agc.set("RTSTBASE", 2);                           // LR: base 2, max 3
        agc.set("RTSTDEX", 6);
        r04c();
    }

    /** R04END: terminate sampling, disable the RR auto tracker, signal end of R04. */
    public void r04end() {
        agc.set("RSAMPDT", 0);
        extern("DELAYJOB", BIT8);                         // wait 1.28 s for a pending rupt
        agc.set("CHAN12", agc.get("CHAN12") & ~BIT14);
        downflag("R04FLAG");
        endext();
    }

    /** R77END (V79): terminate LR spurious return test. */
    public void r77end() {
        agc.set("RSAMPDT", 0);
        extern("DELAYJOB", BIT6);                         // wait 320 ms for pending rupt
        downflag("R77FLAG");
        gopin();
    }

    // ---- S-band / attitude meters ----

    /** VB64 (V64): calculate and display S-band antenna angles; demands P00. */
    public void vb64() {
        if (!chkpooh() || !testxact()) {
            return;
        }
        extern("FINDVAC", "SBANDANT", 04);
        extern("ENDOFJOB");
    }

    /** IMUATTCK (V43): load IMU attitude error meters. */
    public void imuattck() {
        if (!chkpooh()) {
            return;
        }
        if ((agc.get("CHAN12") & BITS4_5) != 0) {         // coarse align or zero on
            almEnd();
            return;
        }
        if (((agc.get("CHAN31") ^ BIT13_14) & BIT13_14) != 0) {  // needles on
            almEnd();
            return;
        }
        if (!testxact()) {
            return;
        }
        extern("GOXDSPF", VNLODCDU);
        extern("EXDSPRET", V43K);                         // redisplay our verb
        agc.set("CHAN12", agc.get("CHAN12") | BIT6);      // enable error counters
        extern("WAITLIST", 2, "ATTCK2");                  // put out commands in .32 s
        endext();
    }

    /** ATTCK2: waitlist task putting out commands (THETAD * ATTSCALE) to the CDU counters. */
    public void attck2() {
        String[] cmd = {"CDUXCMD", "CDUYCMD", "CDUZCMD"};
        for (int i = 2; i >= 0; i--) {
            double th = Agc.toSigned(agc.get("THETAD" + (i == 0 ? "" : "+" + i))) * ATTSCALE;
            agc.set(cmd[i], Agc.fromSigned((int) Math.round(th)));
        }
        agc.set("CHAN14", agc.get("CHAN14") | C13_14_15);  // leave error counters enabled
        extern("TASKOVER");
    }

    // ---- verbs that start jobs ----

    /** V82PERF: orbit parameter display (R30, V82). */
    public void v82perf() {
        if (!testxact()) {
            return;
        }
        extern("PRIOCHNG", 07);
        extern("SUPDXCHZ", "V82CALL");                    // in a different superbank
    }

    /** V83PERF: rendezvous parameter display (R31, V83). */
    public void v83perf() {
        if (!testxact()) {
            return;
        }
        extern("WAITLIST", 2, "R31CALL");
        extern("ENDOFJOB");
    }

    /** V89PERF: align LEM axis along LOS (R63). */
    public void v89perf() {
        if (!chkpooh() || !testxact()) {
            return;
        }
        extern("FINDVAC", "V89CALL", 010);
        extern("ENDOFJOB");
    }

    /** V90PERF: out-of-plane rendezvous display (R36). */
    public void v90perf() {
        if (!testxact()) {
            return;
        }
        extern("FINDVAC", "R36", 07);
        extern("ENDOFJOB");
    }

    /** MINIMP (V76): minimum impulse mode (PULSES = 1). */
    public void minimp() {
        agc.set("DAPBOOLS", agc.get("DAPBOOLS") | agc.get("PULSES"));
        gopin();
    }

    /** NOMINIMP (V77): rate command mode. */
    public void nominimp() {
        agc.set("DAPBOOLS", agc.get("DAPBOOLS") & ~agc.get("PULSES"));
        extern("ZATTEROR");
        gopin();
    }

    /** CREWMANU (V49): start automatic attitude maneuver (R62). */
    public void crewmanu() {
        if (!chkpooh() || !testxact()) {
            return;
        }
        extern("FINDVAC", "R62DISP", 010);
        extern("ENDOFJOB");
    }

    /** TRMTRACK (V56): terminate tracking (P20 + P25). */
    public void trmtrack() {
        if ((BITS9_7 & agc.get("FLAGWRD0")) == 0) {       // rendezvous or P25 flag on?
            gopin();
            return;
        }
        downflag("RNDVZFLG");
        downflag("P25FLAG");
        downflag("SRCHOPTN");                             // ensure search flag is off
        if ((agc.get("TRACKBIT") & agc.get("FLAGWRD1")) == 0) {
            gopin();
            return;
        }
        trmtrak1();
    }

    /** TRMTRAK1: halt P20 activity and cause a restart through GOPROG2. */
    public void trmtrak1() {
        downflag("UPDATFLG");
        downflag("TRACKFLG");
        downflag("IMUSE");
        extern("INTSTALL");                               // don't interrupt integration
        extern("PHASCHNG", 2);                            // kill group 2
        extern("STOPRATE");                               // zero commanded rates
        extern("RESTORDB");
        extern("CLRADMOD");                               // clear bits 10 and 15 of RADMODES
        agc.set("CHAN12", agc.get("CHAN12") & ~BIT14);    // disable lock-on
        extern("GOPROG2");                                // cause restart
    }

    /** DNEDUMP (V74): initialize down-telemetry for erasable dump. */
    public void dnedump() {
        agc.set("DNTMGOTO", agc.get("DNDUMPI"));
        gopin();
    }

    /** LEMVEC (V80): update LEM state vector. */
    public void lemvec() {
        downflag("VEHUPFLG");                             // VEHUPFLG down indicates LEM
        noupdown();
    }

    /** CSMVEC (V81): update CSM state vector. */
    public void csmvec() {
        upflag("VEHUPFLG");                               // VEHUPFLG up indicates CSM
        noupdown();
    }

    private void noupdown() {
        downflag("NOUPFLAG");
        gopin();
    }

    /** UPDATOFF (V95): no state vector update allowed. */
    public void updatoff() {
        upflag("NOUPFLAG");
        gopin();
    }

    /** SYSTEST (V92): IMU performance test. */
    public void systest() {
        if (!chkpooh() || !testxact()) {
            return;
        }
        extern("FINDVAC", "REDO", 026);
        extern("ENDOFJOB");
    }

    /** WMATRXNG (V93): clear RENDWFLG. */
    public void wmatrxng() {
        agc.set("FLAGWRD5", agc.get("FLAGWRD5") & ~agc.get("RENDWBIT"));
        gopin();
    }

    // ---- SHOWSUM ----

    /** SHOWSUM / GOSHOSUM (V91): display bank checksums (turns self-check off while active). */
    public void showsum() {
        if (!chkpooh() || !testxact()) {
            return;
        }
        extern("PRIOCHNG", 07);
        agc.set("SKEEP6", 1);                             // SHOWSUM option
        agc.set("SMODE", 0);                              // turn off self-check
        agc.set("SELFRET", agc.get("SELFADRS"));
        extern("STSHOSUM");                               // enter ROPECHK
    }

    /** SDISPLAY / NOKILL: display bank number and bugger word (N01). */
    public void sdisplay() {
        agc.set("SKEEP2", agc.get("SKEEP3"));              // LXCH swaps bank # and bugger word
        agc.set("MPAC+2", agc.get("ADRS1"));
        extern("GOXDSPF", VNCON);                          // V05N01 style display
        // PROCEED -> NXTBNK; ENTER -> redisplay; TERMINATE -> end
        agc.set("SKEEP1", agc.get("SELFADRS"));
        endext();
    }

    /** ENDSUMS: finished all banks: restart SELFCHK or SHOWSUM. */
    public void endsums() {
        if (agc.get("SKEEP6") == 0) {
            extern("SELFCHK");
        } else {
            extern("STSHOSUM");
        }
    }

    // ---- R03: DAP data load ----

    /** DAPDISP (V48): load autopilot data. */
    public void dapdisp() {
        if (!testxact()) {
            return;
        }
        extern("PRIOCHNG", 07);                            // R03
        dapdata1();
    }

    /** DAPDATA1: display/load DAP configuration word (N46). */
    public void dapdata1() {
        int r1 = BOOLSMSK & agc.get("DAPBOOLS");
        if ((~agc.get("FLGWRD10") & agc.get("APSFLBIT")) != 0) {
            r1 += BIT14;                                    // bit 14 = complement of APSFLAG
        }
        for (;;) {                                           // CHKDATA1
            if ((r1 & BIT13_14) == 0) {
                r1 += BIT13;                                // FORCEONE
            }
            r1 &= DSPLYMSK;                                 // no illegal bits set by crew
            agc.set("DAPDATR1", r1);
            extern("GOXDSPFR", V01N46);
            // V34E terminate -> ENDEXT; V33E proceed -> DPDAT1; E -> recheck
            break;
        }
        dpdat1();
    }

    /** DPDAT1: set flag bits, mass and jet selection from the displayed DAPDATR1. */
    public void dpdat1() {
        int r1 = agc.get("DAPDATR1");
        // APSFLAG is complement of bit 14
        int fl10 = agc.get("FLGWRD10") & ~agc.get("APSFLBIT");
        if ((~r1 & BIT14) != 0) {
            fl10 |= agc.get("APSFLBIT");
        }
        agc.set("FLGWRD10", fl10);
        // CSMDOCKD only if both bits 13 and 14 set
        int mask = ((r1 & BIT13_14) != 0 ? ~agc.get("CSMDOCKD") : 0) & 0x7FFF;
        int boolsNew = ((mask + BOOLSMSK) & r1) + (agc.get("DAPBOOLS") & ~BOOLSMSK);
        agc.set("DAPBOOLS", boolsNew);
        // mass in accordance with CSMDOCKD
        int mass = (boolsNew & agc.get("CSMDOCKD")) != 0 ? agc.get("CSMMASS") : 0;
        agc.set("MASS", Agc.add(mass, agc.get("LEMMASS")));
        // 2 or 4 jet X-translation: NJTSFLAG cleared for 4 jets, set for 2
        if ((boolsNew & agc.get("ACC4OR2X")) != 0) {
            agc.set("FLAGWRD1", agc.get("FLAGWRD1") & ~BIT15);
        } else {
            agc.set("FLAGWRD1", agc.get("FLAGWRD1") | BIT15);
        }
        agc.set("RATEINDX", (boolsNew & 3) * 2);             // maneuver rate index 0,2,4,6 (DP)
        stikload();
    }

    /** STIKLOAD: set stick sensitivity and rate deadband, then go to N47 mass load. */
    public void stikload() {
        double sens = (agc.get("RHCSCALE") & agc.get("DAPBOOLS")) != 0 ? NORMAL : FINE;
        int stik = (int) Math.round(sens * 0x4000);
        agc.set("-RATEDB", Agc.fromSigned(MINUS_0_6_DS));   // LM-only breakout .6 d/s
        if ((agc.get("CSMDOCKD") & agc.get("DAPBOOLS")) != 0) {
            stik = (int) Math.round(stik * ONE_TENTH);      // CSM docked: divide by 10
            agc.set("-RATEDB", Agc.fromSigned(MINUS_0_3_DS));
        }
        agc.set("STIKSENS", stik);
        dapdata2();
    }

    /** DAPDATA2: load N47 (masses); V34E terminates via ENDR03. */
    public void dapdata2() {
        extern("GOXDSPFR", V0647);
        dapdat2();
    }

    /** ENDR03: restore deadband and end. */
    public void endr03() {
        extern("RESTORDB");
        endext();
    }

    /** DAPDAT2: validate masses, compute MASS, restore deadband / inertias. */
    public void dapdat2() {
        int minLm = (agc.get("FLGWRD10") & agc.get("APSFLBIT")) == 0 ? agc.get("MINLMD") : 0;
        int lemMin = Agc.add(Agc.add(minLm, agc.get("MINMINLM")), agc.get("LEMMASS"));
        if (Agc.toSigned(lemMin) <= 0) {
            dapdata2();
            return;
        }
        if ((agc.get("DAPBOOLS") & agc.get("CSMDOCKD")) != 0) {
            if (Agc.toSigned(Agc.add(Agc.fromSigned(-Agc.toSigned(agc.get("MINCSM"))), agc.get("CSMMASS"))) <= 0) {
                dapdata2();                                  // ask for new masses
                return;
            }
            agc.set("MASS", Agc.add(agc.get("CSMMASS"), agc.get("LEMMASS")));
        } else {
            agc.set("MASS", agc.get("LEMMASS"));             // LEM alone
        }
        agc.set("MASS+1", 0);
        extern("RESTORDB");                                  // deadband and moments of inertia
        dapdata3();
    }

    /** DAPDATA3: unless staged, display trim angles (N48) and request response. */
    public void dapdata3() {
        if ((agc.get("FLGWRD10") & agc.get("APSFLBIT")) != 0) {
            endext();                                        // LEM has staged
            return;
        }
        extern("GOXDSPFR", V06N48);
        dpdat3();
    }

    /** DPDAT3: schedule TRIMGIMB via WAITLIST (so it can use FIXDELAY/VARDELAY). */
    public void dpdat3() {
        extern("WAITLIST", 1, "TRIMGIMB");
        extern("ENDOFJOB");
    }

    /** TRIMDONE: trim is finished; ask crew to terminate R03 (V50N48). */
    public void trimdone() {
        extern("GOMARK3R", V50N48);
        endext();
    }

    // ---- misc verbs ----

    /** ATTACHED (V66): move this vehicle's state vector to the other (docked). */
    public void attached() {
        extern("FINDVAC", "ATTACHIT", 010);
        extern("ENDOFJOB");
    }

    /** ATTACHIT: copy RRECTHIS into RRECTOTH via GENTRAN and update R-OTHER, V-OTHER. */
    public void attachit() {
        extern("INTSTALL");
        if (flag("MOONTHIS")) {
            if (!flag("MOONOTH")) {
                upflag("MOONOTH");
            }
        } else {
            downflag("MOONOTH");
        }
        extern("GENTRAN", 051, "RRECTHIS", "RRECTOTH");     // our state vector into other
        extern("PTOALEM");                                   // update R-OTHER, V-OTHER
        extern("SVDWN1", "PBODY");
        agc.set("QPRET", 0);                                 // TCPIN: RTB PINBRNCH
        extern("INTWAKE");                                   // free integration and exit
    }

    /** VERB96 (V96): interrupt integration and go to P00. */
    public void verb96() {
        upflag("QUITFLAG");                                  // causes integration to exit at next timestep
        agc.set("A", 0);
        extern("V37");                                       // go to P00
    }

    /** VERB69 (V69): force a hardware restart (loops on itself in the original). */
    public void verb69() {
        extern("VERB69");
    }

    /** V67: W matrix monitor. */
    public void v67() {
        if (!testxact()) {
            return;
        }
        extern("FINDVAC", "V67CALL", 05);
        extern("ENDOFJOB");
    }

    /** SNUFFOUT (V65): disable U,V jets during DPS burns. */
    public void snuffout() {
        upflag("SNUFFER");
        gopin();
    }

    /** OUTSNUFF (V75): enable U,V jets during DPS burns. */
    public void outsnuff() {
        downflag("SNUFFER");
        gopin();
    }

    // ---- V85: RR LOS display ----

    /** VERB85: display RR line-of-sight azimuth and elevation. */
    public void verb85() {
        if (!testxact()) {
            return;
        }
        dsprrlos();
    }

    /** DSPRRLOS: start RRLOSDSP job and flash V16N56. */
    public void dsprrlos() {
        extern("FINDVAC", "RRLOSDSP", 05);
        extern("PRIOCHNG", 04);
        extern("GOMARKFR", V16N56);
        extern("ENDOFJOB");
    }

    /**
     * RRLOSDSP: compute RR LOS in body axes from CDUT (RRNBMPAC), derive elevation (angle of
     * the LOS projection in the X-Z plane) and azimuth, both 0-360 degrees; repeat each second
     * while the extended verb display is active.
     */
    public void rrlosdsp() {
        do {
            extern("RRNBMPAC");                              // RR LOS in body axes -> MPAC
            double los[] = {dp("MPAC"), dp("MPAC+2"), dp("MPAC+4")};
            double xz = Math.hypot(los[0], los[2]);
            double[] u = xz == 0 ? new double[] {0, 0, 0} : new double[] {los[0] / xz, 0, los[2] / xz};
            // elevation: COSTH = U.UNITZ, SINTH = UNITX.U
            double elev = Math.atan2(u[0], u[2]) / (2 * Math.PI);
            if (elev < 0) {
                elev += 1;                                   // ensure display of 0-360 deg
            }
            setDp("RR-ELEV", elev);
            // azimuth: SINTH = LOS.UNITY, COSTH = LOS.U
            double cos = los[0] * u[0] + los[2] * u[2];
            double az = Math.atan2(los[1], cos) / (2 * Math.PI);
            if (az < 0) {
                az += 1;
            }
            setDp("RR-AZ", az);
            extern("DELAYJOB", 100);
        } while ((BIT5 & agc.get("EXTVBACT")) != 0);
        endext();
    }

    private double dp(String sym) {
        long v = ((long) Agc.toSigned(agc.get(sym)) << 14) + Agc.toSigned(agc.get(sym.equals("MPAC") ? "MPAC+1" : sym + "+1"));
        return v / (double) (1L << 28);
    }

    private void setDp(String sym, double value) {
        long v = Math.round(value * (1L << 28));
        agc.set(sym, Agc.fromSigned((int) (v >> 14)));
        agc.set(sym + "+1", Agc.fromSigned((int) (v & 037777)));
    }
}
