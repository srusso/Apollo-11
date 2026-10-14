package apollo.comanche055;

import apollo.agc.Agc;

/**
 * DISPLAY_INTERFACE_ROUTINES (Comanche055, pp 1452-1476): the GOFLASH / GODSP / GOPERF / GOMARK
 * family of display request routines between mission programs and PINBALL (NVSUB).
 *
 * Display categories: priority (PRIO), extended verb (MARK), normal (NORM). Each has a slot in the
 * parallel arrays EBANKSAV (DSPFLG), CADRFLSH, NVWORD indexed by COPINDEX
 * (0 = PRIO, 1 = MARK, 2 = NORM). Original temporaries live in MPAC+n (FACEREG=MPAC,
 * PLAYTEM1=MPAC+1, PLAYTEM3=MPAC+3, PLAYTEM4=MPAC+4, COPMPAC=MPAC+3, TEMPOR2=MPAC+4,
 * OUTHERE=MPAC+5, GENMASK=MPAC+6); COPINDEX=LOC, USERPRIO=MODE.
 *
 * Control flow: every label is a method; "TCF label" is a tail call; A (accumulator) on entry is an
 * int parameter. The scheduler operations (NOVAC/FINDVAC/JOBWAKE/JOBSLEEP/ENDOFJOB) are delegated
 * to Executive / Waitlist through the Agc symbol interface.
 */
public final class DisplayInterfaceRoutines {
    // ---- masks and verb/noun constants ----
    public static final int WAITMASK = 03004;
    public static final int IDLEMASK = 074004;
    public static final int MARKFMSK = 040010;       // MARK, FLASH
    public static final int PERFMASK = 00036;        // FLASH, PERFORM, BLANK R2 AND R3
    public static final int V01N25 = 125;
    public static final int V06N07 = 607;
    public static final int V50N00 = 5000;
    public static final int PERF2MSK = 00030;        // FLASH, PERFORM
    public static final int V04N06 = 406;
    public static final int PERF4MSK = 014;          // FLASH, BLANK R3
    public static final int REDOMASK = 020010;       // BITS 4 AND 14
    public static final int MARK3MSK = 040230;       // MARK, DECIMAL NOUN, PERFORM, FLASH
    public static final int MARK4MSK = 040036;       // MARK, PERFORM, FLASH, BLANK 2 AND 3
    public static final int OCT3400 = 03400;         // EBANK mask
    public static final int NBUSMASK = 011210;
    public static final int PMMASK = 066521;
    public static final int VERBMASK = 037600;       // MID7
    public static final int V05N00M1 = 01177;        // V05 minus one
    public static final int NVBUSMSK = 0700;
    public static final int ASTROMSK = 0704;
    public static final int MPERFMSK = 040030;       // BIT 15,5,4 FOR MARK, PERFORM, FLASH
    public static final int OCT34300 = 034300;
    public static final int BITS15_7 = 040100;
    public static final int BITS7_4 = 0110;
    public static final int BITS5_11 = 02020;
    public static final int BITS4_10 = 01010;
    public static final int LOWLOAD = 22;
    public static final int BUSYMASK = 077730;
    public static final int CADRMASK = 050;
    public static final int PRIOOCT = 020144;        // PRIO
    public static final int MARKOCT = 042424;        // MARK
    public static final int NORMOCT = 011254;        // NORM
    public static final int IDLESLEP = 074704;
    public static final int OCT67777 = 067777;
    public static final int OCT40420 = 040420;
    public static final int OCT10200 = 010200;
    public static final int V97N00 = 9700;           // PASTE FOR V97 OR V99
    public static final int OCT20100 = 020100;
    public static final int CLOCKCON = 024030;       // FLASH, PERFORM, V99 OR V97 PASTE, REFLASH
    public static final int OCT24100 = 024100;
    public static final int ITISMASK = 040040;
    public static final int PINIDMSK = 074044;
    public static final int OCT40400 = 040400;
    public static final int OCT60000 = 060000;
    public static final int OCT50 = 050;
    public static final int OCT37776 = 037776;
    /** Index-selected tables: PRIOOCT, MARKOCT, NORM. */
    public static final int[] OCT_TABLE = {PRIOOCT, MARKOCT, NORMOCT};

    private static final int BIT1 = 1, BIT3 = 4, BIT4 = 010, BIT5 = 020, BIT6 = 040, BIT8 = 0200,
            BIT9 = 0400, BIT12 = 04000, BIT13 = 010000, BIT14 = 020000, BIT15 = 040000;

    private final Agc agc;

    public DisplayInterfaceRoutines(Agc agc) {
        this.agc = agc;
    }

    // ---- accessors for the overlaid MPAC temporaries ----
    private int g(String s) {
        return agc.get(s);
    }

    private void st(String s, int v) {
        agc.set(s, v);
    }

    private int copindex() {
        return g("LOC");
    }

    private String ix(String base) {
        int i = copindex();
        return i == 0 ? base : base + "+" + i;
    }

    // ------------------------------------------------------------------------------------------
    /** BLANKET (LINUS): A = bits to blank; PLAYTEM4[MPAC+5] &= ~bits via (CS PLAYTEM4; MASK MPAC+6; ADS). */
    public void blanket(int a) {
        st("MPAC+6", a);
        int playtem4 = g("PLAYTEM4");
        int add = (~playtem4) & a & Agc.WORD_MASK;
        String slot = "PLAYTEM4" + (g("MPAC+5") == 0 ? "" : "+" + g("MPAC+5"));
        st(slot, Agc.add(g(slot), add));
    }

    /** ENDMARK: exit marks to MARKEND via POSTJUMP. */
    public void endmark() {
        markend();
    }

    /** CLEARMRK: EXTVBACT = 0; clear FLAGWRD4 bit 1. */
    public void clearmrk() {
        st("EXTVBACT", 0);
        st("FLAGWRD4", g("FLAGWRD4") & ~BIT1 & Agc.WORD_MASK);
    }

    /** MARKEND: clear the mark and go to MARKOVER. */
    public void markend() {
        clearmrk();
        markover();
    }

    /** GOMARK: entrance for mark GODSP. */
    public void gomark(int a) {
        st("PLAYTEM1", a);
        gomars();
    }

    /** GOMARS: BIT15 set for all mark requests. */
    public void gomars() {
        goflash2(BIT15);
    }

    /** KLEENEX: clean out extended verbs. */
    public void kleenex() {
        gomarkf(0);
    }

    /** GOMARKF: entrance for mark GOFLASH. */
    public void gomarkf(int a) {
        st("PLAYTEM1", a);
        goflash2(MARKFMSK);
    }

    /** GOMARK2: mark GOPERFS-3 astronaut returns. */
    public void gomark2(int a) {
        st("PLAYTEM1", a);
        markform();
    }

    /** MARKFORM: mark, perform, flash. */
    public void markform() {
        goflash2(MPERFMSK);
    }

    /** GOMARK3: used for 3COMP decimal perform. */
    public void gomark3(int a) {
        st("PLAYTEM1", a);
        goflash2(MARK3MSK);
    }

    /** GOMARK4: mark, perform, flash, blank. */
    public void gomark4(int a) {
        st("PLAYTEM1", a);
        goflash2(MARK4MSK);
    }

    /** GOMARKR: entrance for mark GODSPR. */
    public void gomarkr(int a) {
        st("PLAYTEM1", a);
        godspr2(BIT15);
    }

    /** GOMARKFR: entrance for mark GOFLASHR. */
    public void gomarkfr(int a) {
        st("PLAYTEM1", a);
        godsprs(MARKFMSK);
    }

    /** GOMARK2R: mark GOPERFS-3, astronaut returns + immediate return. */
    public void gomark2r(int a) {
        st("PLAYTEM1", a);
        godsprs(MPERFMSK);
    }

    /** GOMARK3R. */
    public void gomark3r(int a) {
        st("PLAYTEM1", a);
        godsprs(MARK3MSK);
    }

    /** MAKEMARK: schedule a mark display (COPIES index 1). */
    public void makemark() {
        copies(1);
        if ((g("FLAGWRD4") & OCT34300) != 0) {         // norm or prio busy or waiting
            chkprio();
            return;
        }
        if ((g("FLAGWRD4") & BIT9) == 0) {             // mark not sleeping due to astro busy
            markplay();
            return;
        }
        endofjob();
    }

    /** MARKPLAY: reset mark-over-norm, set mark in FLAGWRD4. */
    public void markplay() {
        // CS FIVE; MASK FLAGWRD4; AD ONE
        st("FLAGWRD4", ((g("FLAGWRD4") & ~5) + 1) & Agc.WORD_MASK);
        gogomark();
    }

    /** GOGOMARK: perform mark; flip MARKNV when MARKFLAG bit 5 is clear. */
    public void gogomark() {
        if (((~g("MARKFLAG")) & BIT5) != 0) {
            st("MARKNV", (~g("MARKNV")) & Agc.WORD_MASK);
        }
        markcop();
    }

    /** MARKCOP: mark index = 1, go PRIOPLAY. */
    public void markcop() {
        prioplay(1);
    }

    /** COPYTOGO: restore MPAC+2 from MPAC2SAV then COPYPACS. */
    public void copytogo() {
        st("MPAC+2", g("MPAC2SAV"));
        copypacs();
    }

    /** COPYPACS: GENMASK = PRIOOCT[COPINDEX]; TEMPOR2 = EBANK = EBANKSAV[COPINDEX]. */
    public void copypacs() {
        st("MPAC+6", OCT_TABLE[copindex()]);          // GENMASK
        int e = g(ix("EBANKSAV"));
        st("MPAC+4", e);                               // TEMPOR2
        st("EBANK", e);
    }

    /** CHKPRIO: mark attempt during PRIO. */
    public void chkprio() {
        if ((g("FLAGWRD4") & OCT24100) != 0) {
            marsleep();
            return;
        }
        st("FLAGWRD4", g("FLAGWRD4") | BIT3);          // set mark over norm
        setnorm();
    }

    /** MARKPERF: display noun/verb for mark perform. */
    public void markperf() {
        nv50dsp(g("MARKNV") & VERBMASK);
    }

    /** GODSP: standard display. */
    public void godsp(int a) {
        st("PLAYTEM1", a);
        godsp2();
    }

    public void godsp2() {
        goflash2(0);
    }

    /** GODSPRET: GODSP with a paste (BIT6 set: go back to user after NVSUB). */
    public void godspret(int a) {
        st("PLAYTEM1", a);
        goflash2(BIT6);
    }

    /** GODSPR: display with immediate return. */
    public void godspr(int a) {
        st("PLAYTEM1", a);
        godspr1();
    }

    public void godspr1() {
        godspr2(0);
    }

    public void godspr2(int a) {
        st("PLAYTEM4", a);
        godsprs1(0);                                   // DONT MOVE
    }

    /** CLEANDSP / REFLASH: flash and permit. */
    public void cleandsp() {
        reflash(0);
    }

    public void reflash(int a) {
        st("PLAYTEM1", a);
        goflash2(REDOMASK);
    }

    public void regodsp(int a) {
        st("PLAYTEM1", a);
        goflash2(BIT14);
    }

    public void regodspr(int a) {
        st("PLAYTEM1", a);
        godspr2(BIT14);
    }

    public void clocplay(int a) {
        st("PLAYTEM1", a);
        goflash2(CLOCKCON);
    }

    /** GOFLASH: leave only flash bit set. */
    public int goflash(int a) {
        st("PLAYTEM1", a);
        goflash2(BIT4);
        return reply;
    }

    /** GOFLASH2: PLAYTEM4 = flag word; SAVELOCS; RELINT; branch directly to MAKEPLAY. */
    public void goflash2(int a) {
        st("PLAYTEM4", a);
        savelocs();
        makeplay();
    }

    public void priodspr(int a) {
        st("PLAYTEM1", a);
        godsprs(BITS7_4);
    }

    public void priodsp(int a) {
        st("PLAYTEM1", a);
        setprio();
    }

    public void setprio() {
        goflash2(BITS7_4);
    }

    /** MAKEPRIO: priority display start. */
    public void makeprio() {
        st("LOC", 0);                                  // COPINDEX = 0
        if (linuschr()) {                              // LINUS return -> HIPRIO
            hiprio();
            return;
        }
        if ((g("FLAGWRD4") & OCT20100) != 0) {         // prio in endidle or busy
            priobort();
            return;
        }
        hiprio();
    }

    /** HIPRIO: if mark active -> SETMARK; else if normal active -> SETNORM; else OKTOCOPY. */
    public void hiprio() {
        if ((g("FLAGWRD4") & OCT40400) == 0) {
            askifnrm();
            return;
        }
        setmark();
    }

    public void setmark() {
        jobxchs(0);
    }

    public void askifnrm() {
        if ((g("FLAGWRD4") & OCT10200) == 0) {         // bits 13+8
            oktocopy();
            return;
        }
        setnorm();
    }

    public void setnorm() {
        jobxchs(1);
    }

    /** OKTOCOPY: copy norm, wake sleepers, exchange to end. */
    public void oktocopy() {
        copynorm();
        witchone();
        // TC JOBWAKE; TC XCHTOEND
        xchtoend();
    }

    /** REDOPRIO: save time PRIODSP sent out. */
    public void redoprio() {
        st("PRIOTIME", g("TIME1"));
        keepprio();
    }

    /** KEEPPRIO: start up PRIO display. */
    public void keepprio() {
        prioplay(0);
    }

    /** MAKEPLAY: save user priority; raise to PRIO33; dispatch on display type. */
    public void makeplay() {
        st("USERPRIO", g("PRIORITY") & 037);           // MODE, PRIO37 mask
        // CAF PRIO33; TC PRIOCHNG (raise priority for fast jobs after wake)
        int f = g("PLAYTEM4") & BITS15_7;
        if (f == 0) {
            iflegal();                                 // normal display
        } else if ((g("PLAYTEM4") & BIT14) != 0) {
            makeprio();
        } else {
            makemark();
        }
    }

    /** IFLEGAL: normal display; check legality against EBANKTEM and asleep norms. */
    public void iflegal() {
        st("LOC", 2);
        if (linuschr()) {
            oktoplay();
            return;
        }
        if (((~g("EBANKTEM")) & BIT4) != 0 && (g("FLAGWRD4") & NBUSMASK) != 0) {
            priobort();
            return;
        }
        oktoplay();
    }

    /** PRIOBORT: POODOO alarm 1502. */
    public void priobort() {
        // TC POODOO OCT 1502
        agc.set("ALARM", 01502);
    }

    /** OKTOPLAY: copy request, save user priority/superbank; sleep if prio or mark going. */
    public void oktoplay() {
        copies2();
        st("RESTREG", g("USERPRIO"));
        if ((g("FLAGWRD4") & PMMASK) != 0) {
            gosleeps();
            return;
        }
        witchone();
        xchtoend();
    }

    /** PLAYJUM1: NORM index 2 then PRIOPLAY. */
    public void playjum1() {
        prioplay(2);
    }

    public void prioplay(int copindex) {
        st("LOC", copindex);
        goplay();
    }

    public void exdspret(int a) {
        st("PLAYTEM1", a);
        goflash2(BIT15 | 077);                         // BIT15+6
    }

    /** GOPERF1: store checklist value and display V01N25 (checklist value in R1). */
    public void goperf1(int a) {
        st("NORMTEM1", a);
        goperfs(V01N25);
    }

    public void goperfs(int a) {
        st("PLAYTEM1", a);
        goflash2(PERFMASK);
    }

    public void goperf2(int a) {
        st("PLAYTEM1", a);
        goflash2(PERF2MSK);
    }

    public void goperf4(int a) {
        goflash2(purrs4(a));
    }

    public void goflashr(int a) {
        st("PLAYTEM1", a);
        godsprs(BIT4);
    }

    public void godsprs(int a) {
        st("PLAYTEM4", a);
        godsprs1(3);
    }

    /**
     * GODSPRS1: immediate return is CALL CADR + 4; schedule MAKEPLAY as NOVAC (or VAC if flashing R display).
     */
    public void godsprs1(int a) {
        st("RUPTREG3", a);
        int newprio = g("PRIORITY") & 037;             // one higher than user
        st("NEWPRIO", newprio);
        if ((g("PLAYTEM4") & BIT4) != 0) {
            vacdsp();
        } else {
            // TC NOVAC (NEWPRIO), 2CADR MAKEPLAY
            bothjobs();
        }
    }

    public void vacdsp() {
        // SPVAC with MAKEGEN (MAKEPLAY) and BBANK/superbank saved in L
        bothjobs();
    }

    /** BOTHJOBS: copy temps into permanent regs; save NVWORD and user's MPAC+2, CADR, flags, EBANK. */
    public void bothjobs() {
        savelocs();
        int loc = g("LOCCTR");
        st("MPAC+5", loc);
        savelocr();
        // TCF BANKJUMP: call CADR + 4
    }

    public void goperf1r(int a) {
        st("NORMTEM1", a);
        goperfrs(V01N25);
    }

    public void goperfrs(int a) {
        st("PLAYTEM1", a);
        godsprs(PERFMASK);
    }

    public void goperf2r(int a) {
        st("PLAYTEM1", a);
        godsprs(PERF2MSK);
    }

    public void goperf4r(int a) {
        godsprs(purrs4(a));
    }

    /** PURRS4: OPTION1 = option code; PLAYTEM1 = V04N06; returns PERF4MSK. */
    public int purrs4(int a) {
        st("OPTION1", a);
        st("PLAYTEM1", V04N06);
        return PERF4MSK;
    }

    /** SAVELOCS: merge EBANK bits into PLAYTEM4. */
    public void savelocs() {
        int p4 = (g("PLAYTEM4") & ~OCT3400) + g("EBANK");
        st("PLAYTEM4", p4);
        savelocr();
    }

    /** SAVELOCR: PLAYTEM3 = caller's CADR; returns caller's CADR + RUPTREG3. */
    public void savelocr() {
        st("PLAYTEM3", g("Q"));                        // MAKECADR
    }

    public void copynorm() {
        copies(0);
    }

    public void copies(int idx) {
        st("LOC", idx);
        copies2();
    }

    /** COPIES2: store flag word and (if flash/GODSPRET) the CADR and VN word in slot COPINDEX. */
    public void copies2() {
        int flag = g("PLAYTEM4");
        st(ix("EBANKSAV"), flag);                      // EQUIV TO DSPFLG
        if ((flag & CADRMASK) != 0) {
            st(ix("CADRFLSH"), g("PLAYTEM3"));
        }
        st(ix("NVWORD"), g("PLAYTEM1"));
        relintq();
    }

    /** GOSLEEPS: put display to sleep in the WAKECADR slot. */
    public void gosleeps() {
        int a = OCT_TABLE[copindex()] & WAITMASK;
        upent2(a);
        st("FACEREG", copindex() - 1);
        xchsleep();
    }

    /** XCHSLEEP: wake the job via WAKECADR[FACEREG]; replace with new job area; JOBSLEEP. */
    public void xchsleep() {
        // INDEX FACEREG; CAF WAKECADR; JOBWAKE; XCHTOEND; INDEX FACEREG; CAF WAKECADR; JOBSLEEP
    }

    /** JOBXCHS: FACEREG controls which type of display is put to sleep. */
    public void jobxchs(int a) {
        st("FACEREG", a);
        witchone();
        // TC JOBWAKE
        st("FACEREG", a);
        int idle = MARKOCT & IDLESLEP;
        downent2(idle);
        upent2((a == 0 ? BIT5 : BIT4) + 4);            // bit shows PRIO interrupted NORM or MARK
        xchsleep();
    }

    /** MARKWAKE / WAKEPLAY: wake display TEMPOR2. */
    public void markwake() {
        wakeplay(0);
    }

    public void wakeplay(int idx) {
        st("MPAC+4", idx);
        downent2(BITS5_11 + 4);
        endret();
    }

    /** INITDSP: restore most recent normal EBANK, superbank and job priority; return to saved flash. */
    public void initdsp() {
        st("EBANK", g("EBANKTEM"));
        // SUPERSW restore superbank; PRIOCHNG(RESTREG & PRIO37); BANKJUMP to TEMPFLSH - 3
    }

    /** PINBRNCH: pinball interrupted the display; decide whether to resume the mark, erase, or norm. */
    public void pinbrnch() {
        st("MPAC+2", g("MARK2PAC"));
        if ((g("FLAGWRD4") & 013) /* 13,14,15 */ != 0) {
            markplay();
            return;
        }
        eraser();
    }

    /** NORMBNCH: set PINBRFLG; keep PRIO if interrupted, else PLAYJUM1. */
    public void normbnch() {
        st("PINBRFLG", 1);
        if ((g("FLAGWRD4") & BIT14) != 0) {
            keepprio();
        } else {
            playjum1();
        }
    }

    /** NVDSP: set up and issue the verb/noun via NVSUB. */
    public void nvdsp() {
        copypacs();
        int tempor2 = g("MPAC+4");
        st("L", tempor2 & 7);
        st(ix("DSPFLG"), g(ix("DSPFLG")) & ~BIT13);
        int tem1 = g(ix("DSPFLG")) & BIT8;
        st("MPAC2SAV", g("MPAC+2"));
        st("MARK2PAC", g("MPAC+2"));
        int nv = g(ix("NVWORD"));
        if (nv == 0) {
            cleanend();
            return;
        }
        int a;
        if (Agc.toSigned(nv) < 0) {                    // CCS: negative/neg-zero branch
            st("MARKNV", (~g("MARKNV")) & Agc.WORD_MASK);
            a = (g("MARKNV") & 0177) + V05N00M1 + tem1;
        } else {
            a = nv - 1;                                // CCS gives DABS
        }
        nvdsp1(a + 1);
    }

    public void nvdsp1(int a) {
        nv50dsp(a);
    }

    /** NV50DSP: NVMONOPT, then busy -> REST; else flash off, COPYTOGO, clear sleeping flags. */
    public void nv50dsp(int a) {
        // TC NVMONOPT ... (display via NVSUB); if busy return goes to REST
        boolean busy = false;
        if (busy) {
            rest();
            return;
        }
        // TC FLASHOFF
        copytogo();
        st("MRKNVFLG", 0);
        st("NRMNVFLG", 0);
        st("PRONVFLG", 0);
        blankchk();
    }

    /** BLANKCHK: blank bits 1,2,3 if set, then NVDSP again; else perfcheck. */
    public void blankchk() {
        // TC BLANKSUB(TEMPOR2); TCF NVDSP
        nvdsp();
    }

    /** PERFCHEK: PERFORM bit -> 1STOR2ND; else GOANIDLE. */
    public void perfchek() {
        if ((g("MPAC+4") & BIT5) != 0) {
            firstOrSecond();
        } else {
            goanidle();
        }
    }

    /** GOANIDLE: flash -> FLASHSUB; GODSPRET -> CADRFLSH; paste -> ISITN00. */
    public void goanidle() {
        if ((g("MPAC+4") & BIT4) != 0) {
            flashsub();
            return;
        }
        if (((~g("MPAC+4")) & BIT6) != 0) {
            isitn00();
            return;
        }
        st("MPAC+3", g(ix("CADRFLSH")));
        endit();
    }

    /** ISITN00: paste check. */
    public void isitn00() {
        if ((g(ix("NVWORD")) & 0177) == 0) {
            flashsub();                                // assume paste always on flash
            return;
        }
        endofjob();                                    // not flash, not GOPERF
    }

    /** 1STOR2ND: first or second part of a GOPERF display. */
    public void firstOrSecond() {
        if ((g("MPAC+4") & BIT13) != 0) {
            goanidle();                                // second
            return;
        }
        st(ix("DSPFLG"), g(ix("DSPFLG")) + BIT13);
        st("L", 0);
        int a = g("MPAC");
        if (Agc.toSigned(a) <= 0 && false) {
            markperf();                                // yes, mark
            return;
        }
        if ((a & BIT12) == 0) {
            v50paste();
            return;
        }
        nv50dsp((~g("NVWORD1") & Agc.WORD_MASK) + V97N00);
    }

    public void v50paste() {
        nv50dsp(V50N00);                               // display second part of GOPERF
    }

    /** WITCHONE: turn off key release light; choose which CADRSTOR slot (NVSUB asleep?). */
    public void witchone() {
        st("DSALMOUT", g("DSALMOUT") & ~BIT5);
        int l = ((g("FLAGWRD4") & NVBUSMSK) != 0) ? 1 : 0;
        st("L", l);
        st("CADRSTOR" + (l == 0 ? "" : "+" + l), 0);
    }

    /** XCHTOEND: TC ENDOFJOB replaces GENADR in LOC for the sleeping job. */
    public void xchtoend() {
        xchnyloc(0 /* ENDINST */);
    }

    public void xchnyloc(int inst) {
        int old = g("LOCCTR");
        st("LOCCTR", inst);
        if (Agc.toSigned(old) < 0 || old == 0) {
            relintq();                                 // was this address sleeping? no
            return;
        }
        st("LOC", old);
        relintq();
    }

    public void relintq() {
        // RELINT; TC Q
    }

    /** CLEANEND: JAMTERM job at PRIO32 then FLASHSUB+1. */
    public void cleanend() {
        // TC FINDVAC PRIO32 2CADR JAMTERM
        flashsub();
    }

    public void isitprio() {
        if ((g("FLAGWRD4") & ITISMASK) == 0) {
            priobort();
            return;
        }
        endofjob();
    }

    /** REST: someone in ENDIDLE -> ENDOFJOB; else RESTSLEP. */
    public void rest() {
        if (g("CADRSTOR") != 0) {
            endofjob();
            return;
        }
        restslep();
    }

    /** RESTSLEP: set NVSLEEP bits, NVSUBUSY. */
    public void restslep() {
        upent2(g("MPAC+6") & ASTROMSK);
        // INDEX COPINDEX; CAF NVCADR; TC NVSUBUSY
    }

    /** FLASHSUB: FLASHON; save COPMPAC; set ENDIDLE bits; call ENDIDLE. */
    public void flashsub() {
        // TC FLASHON
        st("COPMPAC", copindex());
        upent2(g("MPAC+6") & IDLEMASK);
        // R1SAVE bit3 => repeat and return display
        if ((g("R1SAVE") & (BIT3 << copindex())) != 0) {
            unsetr1();
            return;
        }
        if (g("CADRSTOR") != 0) {
            isitprio();
            return;
        }
        endidle();
    }

    /** ENDIDLE returns here: TERMATE / PROCEED / ENTER-with-load. */
    public void endidle() {
        // TC ENDIDLE: control returns to IDLERET1, +1 on PROCEED, +2 on ENTER (or load)
    }

    /** Outcome of the last flashing display: TERMINATE (+0), PROCEED (+1), ENTER/RECYCLE (+2). */
    public static final int TERMINATE = 0, PROCEED = 1, RECYCLE = 2;
    private int reply = PROCEED;

    public void idleret1(int outcome) {
        reply = outcome;
        if (outcome == 0) {
            termate();
        } else if (outcome == 1) {
            proceed();
        } else {
            int verbreg = g("MPAC");
            if (verbreg - LOWLOAD == 0 || verbreg - LOWLOAD + 1 == 0 || verbreg - LOWLOAD + 2 == 0) {
                loaditis();                            // V21, V22 or V23 on DSKY
            } else {
                oktoent();
            }
        }
    }

    public void oktoent() {
        endout(2);
    }

    /** ENDOUT: OUTHERE = outcome; check ENDIDLE return type (PRIO, NORM, MARK). */
    public void endout(int outhere) {
        st("MPAC+5", outhere);
        int f = g("FLAGWRD4") & OCT60000;
        if (f != 0 && (f & 040000) != 0) {
            timechek();
        } else if (f == 0) {
            normret();
        } else {
            markret();
        }
    }

    /** TIMECHEK: PRIO ENDIDLE return; if displayed less than 2 sec ago, KEEPPRIO, else NORMRET. */
    public void timechek() {
        int elapsed = Agc.toSigned(g("TIME1")) - Agc.toSigned(g("PRIOTIME"));
        if (elapsed <= 200) {                          // -2SEC (centiseconds)
            keepprio();
        } else {
            normret();
        }
    }

    public void normwake() {
        wakeplay(1);
    }

    /** ENDRET: return to user according to OUTHERE (0 terminate, 1 proceed, 2 enter, - = ENDOFJOB). */
    public void endret() {
        int out = Agc.toSigned(g("MPAC+5"));
        if (out < 0) {
            endofjob();
            return;
        }
        st("MPAC+3", g(ix("CADRFLSH")) + out + 1);     // return address CADRFLSH + OUTHERE (+1)
        st("MPAC+6", g("MPAC+6") & PINIDMSK);          // remove ENDIDLE and PINBRANCH bits
        downent2(g("MPAC+6"));
        // CS THREE; TC NVSUB: blank everything except MM
        endit();
    }

    /** ENDIT: return to user's priority; BANKJUMP to MPAC+3. */
    public void endit() {
        // CA USERPRIO; MASK PRIO37; TC PRIOCHNG; CA MPAC+3; TCF BANKJUMP
    }

    /** UNSETR1: reset the repeat-and-return request; return to user's immediate return location. */
    public void unsetr1() {
        st("R1SAVE", g("R1SAVE") & ~(BIT3 << copindex()));
        // CAF ZERO; TC SUPERSW (superbank 0)
        immedret(3);
    }

    public void immedret(int a) {
        st("MPAC+3", g(ix("CADRFLSH")) + a);
        // TCF BANKJUMP
    }

    public void termate() {
        endout(0);
    }

    public void proceed() {
        endout(1);
    }

    /** LINUSCHR: true if this is a LINUS return (not LINUS or already in ENDIDLE and ASTRO not busy). */
    public boolean linuschr() {
        if (((~g("PLAYTEM4")) & BIT14) != 0) {
            return true;                               // Q+1: not a LINUS
        }
        if (g("PLAYTEM3") - g(ix("CADRFLSH")) != 0) {
            return false;                              // not already in ENDIDLE
        }
        if (g("DSPLOCK") != 0) {
            endofjob();                                // astronaut busy: end the new display
        }
        return false;
    }

    /** MARKRET: clear mark bits (CS SIX) in FLAGWRD4. */
    public void markret() {
        st("FLAGWRD4", g("FLAGWRD4") & ~6 & Agc.WORD_MASK);
        endret();
    }

    /** MARKOVER: OUTHERE = -1 (ENDOFJOB in ENDRET); check ENDIDLE for NORM or PRIO. */
    public void markover() {
        st("MPAC+5", Agc.fromSigned(-1));
        if ((g("FLAGWRD4") & 030) != 0) {              // PRIO30
            normbnch();
            return;
        }
        normret();
    }

    /** NORMRET: wake sleeping mark, interrupted norm, else restart norm from NVSAVE. */
    public void normret() {
        if ((g("FLAGWRD4") & BITS5_11) != 0) {
            markwake();
            return;
        }
        if ((g("FLAGWRD4") & BITS4_10) != 0) {
            normwake();
            return;
        }
        if ((g("EBANKTEM") & OCT50) != 0) {            // flash request or GODSPRET
            endret();
            return;
        }
        if (g("NVSAVE") == 0) {
            endret();
            return;
        }
        // CAF PRIO15; NOVAC 2CADR PLAYJUM1
        endret();
    }

    /** MARSLEEP: if mark already on, ENDOFJOB; else GOSLEEPS. */
    public void marsleep() {
        if ((g("FLAGWRD4") & BITS5_11) != 0) {
            endofjob();
            return;
        }
        gosleeps();
    }

    /** LOADITIS: verify a load against the noun; pinbranch if same noun. */
    public void loaditis() {
        int nv = g(ix("NVWORD")) & 0177;
        if (g("MPAC+1") - nv == 0) {
            oktoent();                                 // load is valid
        } else {
            pinbrnch();                                // accept load but ask for last again
        }
    }

    /** ERASER: blank everything except MM (NVSUB -3). */
    public void eraser() {
        // CS THREE; TC NVSUB; TCF ENDOFJOB twice
        endofjob();
    }

    // ---- aliases (EQUALS) ----
    public void goxdsp(int a) {
        gomark(a);
    }

    public void goxdspr(int a) {
        gomarkr(a);
    }

    public int goxdspf(int a) {
        gomarkf(a);
        return reply;
    }

    public void goxdspfr(int a) {
        gomarkfr(a);
    }

    public void endext() {
        endmark();
    }

    public void goagin() {
        pinbrnch();
    }

    public void goplay() {
        nvdsp();
    }

    // ---- flag-word helpers (UPENT2 / DOWNENT2 set/clear bits in the DSPFLG word) ----
    private void upent2(int bits) {
        st("FLAGWRD4", g("FLAGWRD4") | bits);
        relintq();
    }

    private void downent2(int bits) {
        st("FLAGWRD4", g("FLAGWRD4") & ~bits & Agc.WORD_MASK);
        relintq();
    }

    private void endofjob() {
        // TC ENDOFJOB
    }
}
