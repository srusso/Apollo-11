package apollo.luminary099;

import apollo.agc.Agc;

/**
 * DISPLAY_INTERFACE_ROUTINES.agc - the program interface to the DSKY display system
 * (GOFLASH, GOPERF*, GODSP*, GOMARK*, etc.).
 *
 * The routines arbitrate between three possible "users" of the display, indexed by COPINDEX:
 * 0 = MARK (extended-verb mark display), 1 = NORMAL (normal display), 2 = PRIORITY display.
 * Per-user state is kept in erasable arrays EBANKSAV, CADRFLSH, NVWORD; the status word PRIOOCT /
 * MARKOCT / (next) are bit masks used by UPENT2/DOWNENT2 to record job status; FLAGWRD4 holds
 * the system status bits.
 *
 * Calling convention (original): the caller loads A with the verb/noun code (VN) and does
 * TC BANKCALL / CADR GOFLASH etc.; the display job returns to the CADR in PLAYTEM3.
 * Java: each entry is a method taking the verb/noun code (and optionally the return CADR).
 */
public class DisplayInterfaceRoutines {
    // ---- bit and mask constants ----
    public static final int BIT3 = 04;
    public static final int BIT4 = 010;
    public static final int BIT5 = 020;
    public static final int BIT6 = 040;
    public static final int BIT8 = 0200;
    public static final int BIT12 = 04000;
    public static final int BIT13 = 010000;
    public static final int BIT14 = 020000;
    public static final int BIT15 = 040000;
    public static final int LOW7 = 0177;
    public static final int SEVEN = 07;
    public static final int PRIO30 = 030;
    public static final int PRIO32 = 032;
    public static final int PRIO33 = 033;
    public static final int PRIO37 = 037;
    public static final int PRIO15 = 015;

    public static final int MARKFMSK = 040010;
    public static final int MPERFMSK = 040030;
    public static final int MARK3MSK = 040230;
    public static final int MARK4MSK = 040036;
    public static final int PERFMASK = 0036;
    public static final int PERF2MSK = 0030;
    public static final int PERF4MSK = 014;
    public static final int REDOMASK = 020010;
    public static final int CLOCKCON = 024030;
    public static final int BITS7_4 = 0110;
    public static final int BITS15_7 = 040100;
    public static final int BITS5_11 = 02020;
    public static final int BITS4_10 = 01010;
    public static final int BIT15_6 = BIT15 | BIT6;
    public static final int OCT3400 = 03400;
    public static final int OCT24100 = 024100;
    public static final int OCT20100 = 020100;
    public static final int OCT40400 = 040400;
    public static final int OCT10200 = 010200;
    public static final int OCT34300 = 034300;
    public static final int OCT60000 = 060000;
    public static final int NBUSMASK = 011210;
    public static final int NVBUSMSK = 0700;
    public static final int PMMASK = 066521;
    public static final int ASTROMSK = 0704;
    public static final int WAITMASK = 03004;
    public static final int IDLEMASK = 074004;
    public static final int IDLESLEP = 074704;
    public static final int PINIDMSK = 074044;
    public static final int ITISMASK = 040040;
    public static final int CADRMASK = 050;
    public static final int OCT50 = 050;
    public static final int BUSYMASK = 077730;
    public static final int OCT67777 = 067777;
    public static final int OCT40420 = 040420;
    public static final int PINMASK = 0xE;     // EQUALS 13,14,15 (flag bits)
    public static final int PRIOOCT_0 = 020144;
    public static final int MARKOCT_0 = 042424;
    public static final int MARKOCT_1 = 011254;
    public static final int V05N00M1 = 01177;
    public static final int LOWLOAD = 22;

    public static final int V01N25 = 125;
    public static final int V06N07 = 607;
    public static final int V50N00 = 5000;
    public static final int V04N06 = 406;
    public static final int V97N00 = 9700;
    public static final int MINUS_2SEC = -200;

    private final Agc agc;

    public DisplayInterfaceRoutines(Agc agc) {
        this.agc = agc;
    }

    // Erasable aliases
    private int mpac(int i) { return agc.get(i == 0 ? "MPAC" : "MPAC+" + i); }
    private void mpac(int i, int v) { agc.set(i == 0 ? "MPAC" : "MPAC+" + i, v); }
    private int playTem1() { return mpac(1); }
    private void playTem1(int v) { mpac(1, v); }
    private int playTem3() { return mpac(3); }
    private void playTem3(int v) { mpac(3, v); }
    private int playTem4() { return mpac(4); }
    private void playTem4(int v) { mpac(4, v); }
    private int copMpac() { return mpac(3); }
    private int tempor2() { return mpac(4); }
    private void tempor2(int v) { mpac(4, v); }
    private int outHere() { return mpac(5); }
    private void outHere(int v) { mpac(5, v); }
    private int genMask() { return mpac(6); }
    private void genMask(int v) { mpac(6, v); }
    private int copIndex() { return agc.get("LOC"); }
    private void copIndex(int v) { agc.set("LOC", v); }
    private int flag4() { return agc.get("FLAGWRD4"); }
    private void flag4(int v) { agc.set("FLAGWRD4", v); }

    /** Indexed element access helpers, e.g. NVWORD[copindex]. */
    private int idx(String base) { return agc.get(base + "+" + copIndex()); }
    private void idx(String base, int v) { agc.set(base + "+" + copIndex(), v); }

    /** UPENT2: OR mask into the per-user status word (PRIOOCT, MARKOCT, ...), returns result. */
    private void upEnt2(int mask) {
        genMask(genMask() | mask);
    }

    /** DOWNENT2: clear mask bits in the per-user status word. */
    private void downEnt2(int mask) {
        genMask(genMask() & ~mask);
    }

    // ------------------------------------------------------------------
    // Mark (extended verb) support
    // ------------------------------------------------------------------

    /** BLANKET: blank (or restore) areas of the display; A = mask, index in MPAC+5. */
    public void blanket(int a) {
        mpac(6, a);
        int mask = a & ~agc.get("PLAYTEM4");  // CS PLAYTEM4 ; MASK MPAC+6
        mask = (~agc.get("PLAYTEM4")) & a;
        agc.set("PLAYTEM4+" + mpac(5), Agc.add(agc.get("PLAYTEM4+" + mpac(5)), mask));
    }

    /** ENDMARK: TC POSTJUMP / CADR MARKEND (end of marking). */
    public void endMark() {
        markEnd();
    }

    /** CLEARMRK: clear EXTVBACT and fall into CLEARMRK+2. */
    public void clearMrk() {
        agc.set("EXTVBACT", 0);
        clearMrk2();
    }

    /** CLEARMRK+2: clear XDSPBIT in FLAGWRD4. */
    public void clearMrk2() {
        // INHINT
        flag4(flag4() & ~agc.get("XDSPBIT"));
        // RELINT
    }

    /** MARKEND: clear marks then go to MARKOVER. */
    public void markEnd() {
        clearMrk();
        markOver();
    }

    // ------------------------------------------------------------------
    // Entry points: A holds the VN code
    // ------------------------------------------------------------------

    /** GOMARK / GOXDSP: display with MARK form (BIT15). */
    public void goMark(int vn) {
        playTem1(vn);
        goMars();
    }

    public void goMars() {
        goFlash2(BIT15);
    }

    /** KLEENEX: clean display (A = 0), GOMARKF. */
    public void kleenex() {
        goMarkF(0);
    }

    /** GOMARKF / GOXDSPF: flashing mark display with MARKFMSK. */
    public void goMarkF(int vn) {
        playTem1(vn);
        goFlash2(MARKFMSK);
    }

    /** GOMARK2: mark display with MPERFMSK. */
    public void goMark2(int vn) {
        playTem1(vn);
        markForm();
    }

    public void markForm() {
        goFlash2(MPERFMSK);
    }

    /** GOMARK3. */
    public void goMark3(int vn) {
        playTem1(vn);
        goFlash2(MARK3MSK);
    }

    /** GOMARK4. */
    public void goMark4(int vn) {
        playTem1(vn);
        goFlash2(MARK4MSK);
    }

    /** GOMARKR / GOXDSPR: restart-protected mark display. */
    public void goMarkR(int vn) {
        playTem1(vn);
        goDspr2(BIT15);
    }

    /** GOMARKFR. */
    public void goMarkFR(int vn) {
        playTem1(vn);
        goDsprs(MARKFMSK);
    }

    /** GOMARK2R. */
    public void goMark2R(int vn) {
        playTem1(vn);
        goDsprs(MPERFMSK);
    }

    /** GOMARK3R. */
    public void goMark3R(int vn) {
        playTem1(vn);
        goDsprs(MARK3MSK);
    }

    /** MAKEMARK: copy the mark user's data and queue the mark display unless busy. */
    public void makeMark() {
        copies(1);
        if ((flag4() & OCT34300) != 0) {
            chkPrio();
            return;
        }
        if ((flag4() & agc.get("MRKNVBIT")) == 0) {
            markPlay();
        }
        // else TCF ENDOFJOB
    }

    /** MARKPLAY: set the MARK-display-in-progress bits in FLAGWRD4 (clear low 5, set low 1). */
    public void markPlay() {
        // INHINT
        flag4(Agc.add(flag4() & ~5, 1));
        // RELINT
        goGoMark();
    }

    /** GOGOMARK: toggle MARKNV if MARKFLAG bit5 is clear; then PRIOPLAY with COPINDEX = 1. */
    public void goGoMark() {
        if ((~agc.get("MARKFLAG") & BIT5) == 0) {
            agc.set("MARKNV", agc.get("MARKNV"));
        }
        prioPlay(1);
    }

    /** COPYTOGO: restore MPAC+2 from MPAC2SAV; fall into COPYPACS. */
    public void copyToGo() {
        mpac(2, agc.get("MPAC2SAV"));
        copyPacs();
    }

    /** COPYPACS: GENMASK = PRIOOCT[COPINDEX]; TEMPOR2 = EBANKSAV[COPINDEX]; select EBANK. */
    public void copyPacs() {
        genMask(agc.get("PRIOOCT+" + copIndex()));
        tempor2(agc.get("EBANKSAV+" + copIndex()));
        agc.set("EBANK", tempor2());
    }

    /** CHKPRIO: if priority bits set - MARSLEEP; else mark-overrun bit set then SETNORM. */
    public void chkPrio() {
        if ((flag4() & OCT24100) != 0) {
            marSleep();
            return;
        }
        // CS FLAGWRD4 ; MASK MKOVBIT ; INHINT ; ADS FLAGWRD4
        flag4(flag4() + (~flag4() & agc.get("MKOVBIT")));
        setNorm();
    }

    /** MARKPERF: A = MARKNV & VERBMASK; then NV50DSP. */
    public void markPerf() {
        nv50Dsp(agc.get("MARKNV") & agc.get("MID7"));
    }

    // ------------------------------------------------------------------
    // Normal display entries
    // ------------------------------------------------------------------

    /** GODSP: non-flashing display. */
    public void goDsp(int vn) {
        playTem1(vn);
        goDsp2();
    }

    public void goDsp2() {
        goFlash2(0);
    }

    /** GODSPRET. */
    public void goDspRet(int vn) {
        playTem1(vn);
        goFlash2(BIT6);
    }

    /** GODSPR: restart-protected display. */
    public void goDspR(int vn) {
        playTem1(vn);
        goDspr1();
    }

    public void goDspr1() {
        goDspr2(0);
    }

    public void goDspr2(int mask) {
        playTem4(mask);
        goDsprs1(0);
    }

    /** CLEANDSP: reflash with A = 0. */
    public void cleanDsp() {
        reflash(0);
    }

    /** REFLASH. */
    public void reflash(int vn) {
        playTem1(vn);
        goFlash2(REDOMASK);
    }

    /** REFLASHR. */
    public void reflashR(int vn) {
        playTem1(vn);
        goDsprs(REDOMASK);
    }

    /** REGODSP. */
    public void regoDsp(int vn) {
        playTem1(vn);
        goFlash2(BIT14);
    }

    /** REGODSPR. */
    public void regoDspR(int vn) {
        playTem1(vn);
        goDspr2(BIT14);
    }

    /** CLOCPLAY. */
    public void clocPlay(int vn) {
        playTem1(vn);
        goFlash2(CLOCKCON);
    }

    /** GOFLASH: flashing display (BIT4). */
    public void goFlash(int vn) {
        playTem1(vn);
        goFlash2(BIT4);
    }

    /** GOFLASH2: PLAYTEM4 = A; save locations, RELINT, go MAKEPLAY. */
    public void goFlash2(int mask) {
        playTem4(mask);
        saveLocs();
        // RELINT
        makePlay();
    }

    /** PRIODSPR. */
    public void prioDspr(int vn) {
        playTem1(vn);
        goDsprs(BITS7_4);
    }

    /** PRIODSP. */
    public void prioDsp(int vn) {
        playTem1(vn);
        setPrio();
    }

    public void setPrio() {
        goFlash2(BITS7_4);
    }

    /**
     * MAKEPRIO: priority display request. If LINUSCHR says display is free (or lockout bits clear)
     * and priority mask bits not blocking, proceed; else abort 1502.
     */
    public void makePrio() {
        copIndex(0);
        if (!linusChr()) {          // skip-return -> HIPRIO
            hiPrio();
            return;
        }
        if ((flag4() & OCT20100) != 0) {
            prioBort();
            return;
        }
        hiPrio();
    }

    /** HIPRIO. */
    public void hiPrio() {
        if ((flag4() & OCT40400) == 0) {
            askIfNrm();
        } else {
            setMark();
        }
    }

    /** SETMARK: JOBXCHS with A = 0. */
    public void setMark() {
        jobXchs(0);
    }

    /** ASKIFNRM. */
    public void askIfNrm() {
        if ((flag4() & OCT10200) == 0) {
            okToCopy();
        } else {
            setNorm();
        }
    }

    /** SETNORM: JOBXCHS with A = 1. */
    public void setNorm() {
        jobXchs(1);
    }

    /** OKTOCOPY: copy normal data, wake priority jobs, put at end. */
    public void okToCopy() {
        copyNorm();
        witchOne();
        // JOBWAKE ; XCHTOEND
        xchToEnd();
    }

    /** REDOPRIO: PRIOTIME = TIME1; KEEPPRIO. */
    public void redoPrio() {
        agc.set("PRIOTIME", agc.get("TIME1"));
        keepPrio();
    }

    /** KEEPPRIO: PRIOPLAY with A = 0. */
    public void keepPrio() {
        prioPlay(0);
    }

    /**
     * MAKEPLAY: job entered to place the display request: sets USERPRIO from the job priority,
     * raises to PRIO33, then dispatches on PLAYTEM4 (priority / illegal / mark).
     */
    public void makePlay() {
        agc.set("MODE", agc.get("PRIORITY") & PRIO37);     // USERPRIO = MODE alias
        // CAF PRIO33 ; TC PRIOCHNG
        agc.set("PRIORITY", PRIO33);
        int a = playTem4() & BITS15_7;
        if (a != 0) {
            makePrio();           // CCS > 0
        } else {
            ifLegal();            // +0 / -0 cases: IFLEGAL (for zero); MAKEMARK for others
        }
    }

    /** IFLEGAL: normal display request: COPINDEX = 2; check that the display is free, else PRIOBORT. */
    public void ifLegal() {
        copIndex(2);
        if (!linusChr()) {
            okToPlay();
            return;
        }
        if ((~agc.get("EBANKTEM") & BIT4) != 0) {
            okToPlay();
            return;
        }
        if ((flag4() & NBUSMASK) == 0) {
            okToPlay();
            return;
        }
        prioBort();
    }

    /** PRIOBORT: POODOO 1502 - illegal display request while display busy. */
    public void prioBort() {
        new AlarmAndAbort(agc).poodoo(01502);
    }

    /** OKTOPLAY: copy request, record job state, GOSLEEPS or wake others. */
    public void okToPlay() {
        copies2();
        agc.set("RESTREG", agc.get("MODE") & PRIO37 | agc.get("SUPERBNK"));
        int a = flag4() & PMMASK;
        if (a != 0) {
            goSleeps();
        } else {
            witchOne();
            xchToEnd();
        }
    }

    /** PLAYJUM1: PRIOPLAY with COPINDEX = 2. */
    public void playJum1() {
        prioPlay(2);
    }

    /** PRIOPLAY: set COPINDEX and GOPLAY (=NVDSP). */
    public void prioPlay(int copindex) {
        copIndex(copindex);
        nvDsp();
    }

    /** EXDSPRET. */
    public void exDspRet(int vn) {
        playTem1(vn);
        goFlash2(BIT15_6);
    }

    /** GOPERF1: perform with NORMTEM1 = A and V01N25. */
    public void goPerf1(int a) {
        agc.set("NORMTEM1", a);
        goPerfs(V01N25);
    }

    public void goPerfs(int vn) {
        playTem1(vn);
        goFlash2(PERFMASK);
    }

    /** GOPERF2. */
    public void goPerf2(int vn) {
        playTem1(vn);
        goFlash2(PERF2MSK);
    }

    /** GOPERF4: PURRS4 then GOFLASH2 with PERF4MSK. */
    public void goPerf4(int option1) {
        goFlash2(purrs4(option1));
    }

    /** GOFLASHR. */
    public void goFlashR(int vn) {
        playTem1(vn);
        goDsprs(BIT4);
    }

    /** GODSPRS: PLAYTEM4 = mask, A = 3, GODSPRS1. */
    public void goDsprs(int mask) {
        playTem4(mask);
        goDsprs1(3);
    }

    /**
     * GODSPRS1: restart-protected display: starts MAKEPLAY as a NOVAC job (non-flashing)
     * or a VAC job (bit4: flashing), then saves the MPAC contents into the new job's VAC area.
     */
    public void goDsprs1(int a) {
        // INHINT
        agc.set("RUPTREG3", a);
        agc.set("NEWPRIO", agc.get("PRIORITY") & PRIO37);
        if ((playTem4() & BIT4) != 0) {
            vacDsp();
        } else {
            // NOVAC 2CADR MAKEPLAY at NEWPRIO
            agc.set("NOVAC_JOB", 1);
            bothJobs();
        }
    }

    /** VACDSP: start MAKEPLAY with a VAC area via SPVAC (CADR MAKEGEN). */
    public void vacDsp() {
        agc.set("L", agc.get("BBANK") | agc.get("SUPERBNK"));
        agc.set("SPVAC_JOB", 1);
        bothJobs();
    }

    /** BOTHJOBS: copy MPAC+1..+4 into the new job's MPAC (LOCCTR) and BANKJUMP back to caller. */
    public void bothJobs() {
        saveLocs();
        agc.set("MPAC+5", agc.get("LOCCTR"));
        saveLocr();
        // RELINT ; BANKJUMP to PLAYTEM3
    }

    /** GOPERF1R. */
    public void goPerf1R(int a) {
        agc.set("NORMTEM1", a);
        goPerfRs(V01N25);
    }

    public void goPerfRs(int vn) {
        playTem1(vn);
        goDsprs(PERFMASK);
    }

    /** GOPERF2R. */
    public void goPerf2R(int vn) {
        playTem1(vn);
        goDsprs(PERF2MSK);
    }

    /** GOPERF4R. */
    public void goPerf4R(int option1) {
        goDsprs(purrs4(option1));
    }

    /** PURRS4: OPTION1 = A, PLAYTEM1 = V04N06, returns PERF4MSK. */
    public int purrs4(int a) {
        agc.set("OPTION1", a);
        playTem1(V04N06);
        return PERF4MSK;
    }

    /** SAVELOCS: PLAYTEM4 = (PLAYTEM4 &amp; ~OCT3400) + EBANK, then SAVELOCR. */
    public void saveLocs() {
        // INHINT
        playTem4(Agc.add(~OCT3400 & playTem4() & Agc.WORD_MASK, agc.get("EBANK")));
        saveLocr();
    }

    /** SAVELOCR: PLAYTEM3 = MAKECADR of the caller; returns RUPTREG3 + A. */
    public void saveLocr() {
        playTem3(agc.get("Q"));    // MAKECADR converts return address to a CADR
    }

    /** COPYNORM: COPINDEX = 0 then COPIES. */
    public void copyNorm() {
        copies(0);
    }

    /** COPIES: COPINDEX = A; COPIES2. */
    public void copies(int copindex) {
        copIndex(copindex);
        copies2();
    }

    /** COPIES2: save EBANK/CADR/NV word of the request into the user's slot. */
    public void copies2() {
        // INHINT
        idx("EBANKSAV", playTem4());
        if ((playTem4() & CADRMASK) != 0) {
            idx("CADRFLSH", playTem3());
        }
        idx("NVWORD", playTem1());
        relIntQ();
    }

    /** GOSLEEPS: set wait bits in the user's status and put job to sleep. */
    public void goSleeps() {
        genMask(agc.get("PRIOOCT+" + copIndex()) & WAITMASK);
        upEnt2(0);
        agc.set("FACEREG", copIndex() - 1);
        xchSleep();
    }

    /** XCHSLEEP: wake the other user (JOBWAKE / XCHTOEND), then put this user to sleep. */
    public void xchSleep() {
        // INHINT ; CAF WAKECADR[FACEREG] ; JOBWAKE ; XCHTOEND ; CAF WAKECADR[FACEREG] ; JOBSLEEP
        xchToEnd();
        agc.set("JOBSLEEP_CADR", agc.get("WAKECADR+" + agc.get("FACEREG")));
    }

    /** JOBXCHS: FACEREG = A; swap job location; adjust the status mask for who waits. */
    public void jobXchs(int a) {
        agc.set("FACEREG", a);
        witchOne();
        agc.set("FACEREG+" + agc.get("LOCCTR"), agc.get("FACEREG"));
        xchNyLoc(agc.get("XCHQADD"));
        int mask = agc.get("MARKOCT+" + agc.get("FACEREG")) & IDLESLEP;
        genMask(mask);
        downEnt2(IDLEMASK);
        // CA BIT5[FACEREG] ; AD FOUR ; UPENT2
        upEnt2(BIT5 + 4);
        if ((flag4() & agc.get("MKOVBIT")) > 0) {
            markPlay();             // GENMARK
        } else {
            okToCopy();
        }
    }

    /** MARKWAKE: WAKEPLAY with A = 0. */
    public void markWake() {
        wakePlay(0);
    }

    /** WAKEPLAY: wake the display job of user A and remove its wait bits. */
    public void wakePlay(int user) {
        tempor2(user);
        genMask(agc.get("BITS5+11") + 4);
        downEnt2(MARKFMSK);
        // INHINT ; CAF WAKECADR[TEMPOR2] ; JOBWAKE
        endRet();
    }

    /** INITDSP: restore EBANK/priority from RESTREG, then BANKJUMP back into the displayed program. */
    public void initDsp() {
        agc.set("EBANK", agc.get("EBANKTEM"));
        agc.set("SUPERSW", agc.get("RESTREG"));
        agc.set("PRIORITY", agc.get("PRIORITY") & PRIO37);
        // CS THREE ; AD TEMPFLSH ; BANKJUMP
    }

    /** PINBRNCH: pinball (DSKY) branch after ENTER on a mark-type display. */
    public void pinBrnch() {
        // RELINT
        mpac(2, agc.get("MARK2PAC"));
        int a = flag4() & PINMASK;
        if (a > 0) {
            // TCF +3 -> MARKPLAY
            markPlay();
        } else {
            eraser();
        }
    }

    /** NORMBNCH: set PINBRFLG; if PRIODBIT set keep priority display, else PLAYJUM1. */
    public void normBnch() {
        agc.set("PINBRFLG", 1);
        if ((flag4() & agc.get("PRIODBIT")) != 0) {
            keepPrio();
        } else {
            playJum1();
        }
    }

    /**
     * NVDSP: core of the display: extracts the user's NV word and decides how to display it
     * (NVMONOPT, BLANKSUB, etc.), flashing, or finishes (CLEANEND).
     */
    public void nvDsp() {
        copyPacs();
        agc.set("L", tempor2() & SEVEN);
        int dspflg = agc.get("DSPFLG+" + copIndex()) & ~BIT13;
        agc.set("DSPFLG+" + copIndex(), dspflg);
        agc.set("TEM1", dspflg & BIT8);
        agc.set("MPAC2SAV", mpac(2));
        agc.set("MARK2PAC", mpac(2));
        int nv = agc.get("NVWORD+" + copIndex());
        int a;
        if (Agc.toSigned(nv) > 0) {
            a = nv;
        } else if (Agc.toSigned(nv) == 0 && nv == 0) {
            cleanEnd();
            return;
        } else {
            agc.set("MARKNV", agc.get("MARKNV"));
            a = (agc.get("MARKNV") & LOW7) + V05N00M1 + agc.get("TEM1");
        }
        nv50Dsp(a + 1);
    }

    /**
     * NV50DSP: display noun/verb A through NVMONOPT; the multi-way return handles
     * REST / flash off / restore / clear flags, then BLANKCHK ... (see NVSUB in PINBALL).
     */
    public void nv50Dsp(int a) {
        agc.set("NVMONOPT_ARG", a);
        // TC NVMONOPT: returns (1) -> REST, (2) normal: FLASHOFF, COPYTOGO, clear flags
        agc.set("FLASHOFF_CALLED", 1);
        copyToGo();
        agc.set("MRKNVFLG", 0);
        agc.set("NRMNVFLG", 0);
        agc.set("PRONVFLG", 0);
        blankChk();
    }

    /** BLANKCHK: BLANKSUB for TEMPOR2 then repeat NVDSP. */
    public void blankChk() {
        agc.set("BLANKSUB_ARG", tempor2());
        nvDsp();
    }

    /** PERFCHEK: bit5 of TEMPOR2 selects 1STOR2ND (perform check) else GOANIDLE. */
    public void perfChek() {
        if ((tempor2() & BIT5) != 0) {
            firstOrSecond();
        } else {
            goAnIdle();
        }
    }

    /** GOANIDLE: flash if bit4; else test N00 for ISITN00, else return to the caller (ENDIT). */
    public void goAnIdle() {
        if ((tempor2() & BIT4) != 0) {
            flashSub();
            return;
        }
        if ((~tempor2() & BIT6) != 0) {
            isItN00();
            return;
        }
        mpac(3, agc.get("CADRFLSH+" + copIndex()));
        endIt();
    }

    /** ISITN00: if noun is 00 flash, else end job. */
    public void isItN00() {
        if ((agc.get("NVWORD+" + copIndex()) & LOW7) == 0) {
            flashSub();
        }
        // else ENDOFJOB
    }

    /** 1STOR2ND: if BIT13 set go GOANIDLE; else set BIT13 and pick the verb to monitor. */
    public void firstOrSecond() {
        if ((tempor2() & BIT13) != 0) {
            goAnIdle();
            return;
        }
        agc.set("DSPFLG+" + copIndex(), agc.get("DSPFLG+" + copIndex()) + BIT13);
        agc.set("L", 0);
        int nv = agc.get("NVWORD+" + copIndex());
        if (Agc.toSigned(nv) <= 0 && false) {
            markPerf();
            return;
        }
        if ((nv & BIT12) == 0) {
            nv50Dsp(V50N00);
        } else {
            nv50Dsp(V97N00 - agc.get("NVWORD1"));
        }
    }

    /** WITCHONE: clear display-alarm output bit and set L to indicate which user is busy. */
    public void witchOne() {
        agc.set("DSALMOUT", agc.get("DSALMOUT") & ~BIT5);
        agc.set("L", (flag4() & NVBUSMSK) != 0 ? 1 : 0);
        agc.set("CADRSTOR+" + agc.get("L"), 0);
        // INHINT
    }

    /** XCHTOEND: XCHNYLOC with ENDINST. */
    public void xchToEnd() {
        xchNyLoc(agc.get("ENDINST"));
    }

    /** XCHNYLOC: swap LOCCTR with A; if negative just RELINT; else store LOC. */
    public void xchNyLoc(int a) {
        int old = agc.get("LOCCTR");
        agc.set("LOCCTR", a);
        if (Agc.toSigned(old) <= 0) {
            relIntQ();
            return;
        }
        agc.set("LOCCTR", old);
        agc.set("LOC+" + old, old);
        relIntQ();
    }

    /** RELINTQ: RELINT ; TC Q. */
    public void relIntQ() {
        // RELINT
    }

    /** CLEANEND: schedule JAMTERM at PRIO32 (terminates the display) then FLASHSUB+1. */
    public void cleanEnd() {
        agc.set("FINDVAC_PRIO", PRIO32);
        agc.set("FINDVAC_JOB", 1);          // 2CADR JAMTERM
        flashSub1();
    }

    /** ISITPRIO. */
    public void isItPrio() {
        if ((flag4() & ITISMASK) == 0) {
            prioBort();
        }
        // else ENDOFJOB
    }

    /** REST: if CADRSTOR holds something end job, else sleep (RESTSLEP). */
    public void rest() {
        int c = agc.get("CADRSTOR");
        if (Agc.toSigned(c) > 0) {
            return;                      // ENDOFJOB
        }
        if (c != 0) {
            return;                      // negative-zero etc
        }
        restSlep();
    }

    /** RESTSLEP: record astronaut-wait bits (ASTROMSK) and call NVSUBUSY for the restart CADR. */
    public void restSlep() {
        genMask(agc.get("GENMASK") & ASTROMSK);
        upEnt2(OCT24100);
        // CAF NVCADR[COPINDEX] ; TC NVSUBUSY
        agc.set("NVSUBUSY_ARG", agc.get("NVCADR+" + copIndex()));
    }

    /** FLASHSUB: FLASHON; remember COPINDEX; clear wait bits (IDLEMASK) and continue. */
    public void flashSub() {
        agc.set("FLASHON_CALLED", 1);
        flashSub1();
    }

    /** FLASHSUB+1: save index, record in GENMASK, test R1SAVE, dispatch on keyboard response. */
    public void flashSub1() {
        agc.set("COPMPAC", copIndex());
        genMask(agc.get("GENMASK") & IDLEMASK);
        upEnt2(ITISMASK);
        int r1 = agc.get("R1SAVE");
        if ((r1 & (copIndex() == 0 ? 0 : BIT3)) != 0) {
            unsetR1();
            return;
        }
        int c = agc.get("CADRSTOR");
        if (Agc.toSigned(c) > 0 || c == 0) {
            isItPrio();
            return;
        }
        endIdle();
    }

    /** ENDIDLE and the three keyboard returns: TERMATE, PROCEED, ENTER (data loaded). */
    public void endIdle() {
        // TC ENDIDLE ; IDLERET1: TERMINATE / PROCEED / ENTER return points
        int mpac0 = mpac(0);
        if (Agc.toSigned(Agc.add(~LOWLOAD & Agc.WORD_MASK, mpac0)) - 0 == 0) {
            loadItIs();
        } else {
            okToEnt();
        }
    }

    /** OKTOENT: ENDOUT with A = 2. */
    public void okToEnt() {
        endOut(2);
    }

    /** ENDOUT: OUTHERE = A; dispatch on mark/normal bits of FLAGWRD4. */
    public void endOut(int a) {
        outHere(a);
        int f = flag4() & OCT60000;
        if (f > 0 && (f & 040000) != 0 && (f & 020000) != 0) {
            timeChek();
        } else if ((f & 040000) != 0) {
            markRet();
        } else {
            normRet();
        }
    }

    /** TIMECHEK: if priority display has been up more than 2 seconds keep it; else NORMRET. */
    public void timeChek() {
        int elapsed = Agc.toSigned(Agc.add((~agc.get("TIME1")) & Agc.WORD_MASK, agc.get("PRIOTIME")));
        if (elapsed <= 0) {
            elapsed = elapsed + 037776 + 1;
        }
        if (elapsed + MINUS_2SEC <= 0) {
            keepPrio();
        } else {
            normRet();
        }
    }

    /** NORMWAKE: WAKEPLAY with A = 1. */
    public void normWake() {
        wakePlay(1);
    }

    /** ENDRET: finish: if OUTHERE &gt; 0 add CADRFLSH offset and BANKJUMP; else ENDOFJOB. */
    public void endRet() {
        int o = Agc.toSigned(outHere());
        if (o <= 0) {
            return;                          // ENDOFJOB
        }
        mpac(3, Agc.add(o, agc.get("CADRFLSH+" + agc.get("COPMPAC"))));
        genMask(genMask() & PINIDMSK);
        downEnt2(PINIDMSK);
        // CS THREE ; NVSUB
        agc.set("NVSUB_ARG", -3);
        endIt();
    }

    /** ENDIT: restore user priority; BANKJUMP to MPAC+3 (caller's CADR). */
    public void endIt() {
        agc.set("PRIORITY", agc.get("USERPRIO") & PRIO37);
        // CA MPAC+3 ; BANKJUMP
    }

    /** UNSETR1: clear the bit in R1SAVE for the index, then return through IMMEDRET. */
    public void unsetR1() {
        agc.set("R1SAVE", agc.get("R1SAVE") & ~BIT3);
        agc.set("SUPERSW", 0);
        immedRet(3);
    }

    /** IMMEDRET: BANKJUMP to CADRFLSH[COPINDEX] + A. */
    public void immedRet(int a) {
        mpac(0, Agc.add(a, agc.get("CADRFLSH+" + copIndex())));
    }

    /** TERMATE: ENDOUT with A = 0. */
    public void termate() {
        endOut(0);
    }

    /**
     * LINUSCHR: true if the display (index COPINDEX) is available to the caller (skip-return);
     * false if the display is busy and display is locked out (DSPLOCK: end job if locked).
     */
    public boolean linusChr() {
        if ((~playTem4() & BIT14) == 0) {
            return false;                    // CCS +: not BIT14 -> Q+1
        }
        int diff = Agc.add((~playTem3()) & Agc.WORD_MASK, agc.get("CADRFLSH+" + copIndex()));
        if (diff == 0 || diff == Agc.WORD_MASK) {
            return false;                    // BZF +2 : same caller
        }
        return true;
    }

    /** PROCEED: ENDOUT with A = 1. */
    public void proceed() {
        endOut(1);
    }

    /** MARKRET: clear mark-display bits in FLAGWRD4, then ENDRET. */
    public void markRet() {
        // INHINT
        flag4(flag4() & ~6);
        // RELINT
        endRet();
    }

    /** MARKOVER: OUTHERE = -1; dispatch on whether a priority/normal bit is set in FLAGWRD4. */
    public void markOver() {
        outHere(-1);
        if ((flag4() & PRIO30) != 0) {
            normBnch();
        } else {
            normRet();
        }
    }

    /** NORMRET: wake MARK users, NORMAL users, or schedule PLAYJUM1 if NVSAVE needs it. */
    public void normRet() {
        if ((flag4() & BITS5_11) != 0) {
            markWake();
            return;
        }
        if ((flag4() & BITS4_10) != 0) {
            normWake();
            return;
        }
        if ((agc.get("EBANKTEM") & OCT50) != 0) {
            endRet();
            return;
        }
        if (agc.get("NVSAVE") == 0) {
            endRet();
            return;
        }
        // CAF PRIO15 ; INHINT ; NOVAC 2CADR PLAYJUM1
        agc.set("NOVAC_PRIO", PRIO15);
        agc.set("NOVAC_JOB", 1);
        endRet();
    }

    /** MARSLEEP: if mark user active end job; else GOSLEEPS. */
    public void marSleep() {
        if ((flag4() & BITS5_11) != 0) {
            return;                          // ENDOFJOB
        }
        goSleeps();
    }

    /** LOADITIS: compare the loaded noun/verb with MPAC+1; match -> OKTOENT, else PINBRNCH. */
    public void loadItIs() {
        int nv = agc.get("NVWORD+" + agc.get("COPMPAC")) & LOW7;
        int diff = Agc.add((~nv) & Agc.WORD_MASK, mpac(1));
        if (diff == 0 || diff == Agc.WORD_MASK) {
            okToEnt();
        } else {
            pinBrnch();
        }
    }

    /** ERASER: NVSUB(-3) then end job (erase the display). */
    public void eraser() {
        agc.set("NVSUB_ARG", -3);
    }

    /** Static tables from the file. */
    public static final int[] PRIOOCT = {PRIOOCT_0};
    public static final int[] MARKOCT = {MARKOCT_0, MARKOCT_1};
    public static final int[] NVCADR_TABLE = {0 /* REDOPRIO */};
    public static final String[] WAKECADR_TABLE = {"MARKPLAY", "PLAYJUM1"};
}
