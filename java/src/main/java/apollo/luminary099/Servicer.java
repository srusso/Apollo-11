package apollo.luminary099;

import apollo.agc.Agc;

/**
 * SERVICER.agc (Luminary 099, pages 857-897).
 *
 * The SERVICER / Average-G machinery: PREREAD, READACCS (every 2 s), SERVICER job (PIPA compensation,
 * DV monitor, average-G state vector integration CALCRVG / MUNRVG), the PIPA reader (PIPASR / REREADAC),
 * the landing radar (LR) altitude and velocity update logic of R12 (LRHTASK, HIGATASK, POSUPDAT, VELUPDAT,
 * LRVJOB, LRHJOB, RDGIMS, HIGATJOB, SETPOS1/SETPOS2) and the R29 / R10 hooks in COPYCYC2.
 *
 * Conventions: flag-word bit masks (LRBYBIT, MUNFLBIT, ...) are symbols whose values come from
 * the erasable/flag assignments and are read through {@link Agc#get(String)}. Vectors are double[3] in physical
 * units (original scaling given in comments) and are stored via {@link VecOps}. Jumps within a routine are
 * Java control flow; TC/TCF to other files are calls on injected collaborators or are documented no-ops.
 */
public class Servicer {
    // ---- constants
    /** OCT37771: used to synchronise READACCS with the DAP (TIME5). */
    public static final int OCT37771 = 037771;
    /** OCT10002: RADMODES mask (RR not zeroing CDUs, RR in AUTO). */
    public static final int OCT10002 = 010002;
    /** EIGHTEEN: words copied by NORMLIZ2 (no mass copy). */
    public static final int EIGHTEEN = 18;
    /** OCT523: alarm code, LR failed to reach position 2. */
    public static final int OCT523 = 0523;
    /** BITS4-7: clears LR measurement-made discretes. */
    public static final int BITS4_7 = 0110;
    /** Max decimal-20 words in a COPYCYC. */
    public static final int COPYCYC_WORDS = 20;
    /** 1.95SECS (centiseconds). */
    public static final int SECS_1_95 = 195;
    /** 7.5 ft/sec at 2^6 m/cs (2DEC .02286 B-6). */
    public static final double FT7_5 = 0.02286;
    /** 2SEC(18) = 200 B-18 ; 2SEC(28) = 200 cs at B-28 ; 4SEC(28) = 400 B-28. */
    public static final long SEC2_18 = 200, SEC2_28 = 0310L, SEC4_28 = 400;
    /** DP1/20 = 0.05 ; SHIFT11 = 1 B-11. */
    public static final double DP1_20 = 0.05, SHIFT11 = 1.0 / 2048.0;
    /** 2KFT/SEC: 6.096 B-7 (2000 ft/sec at 2^7 m/cs). */
    public static final double KFT2_PER_SEC = 6.096 / 128.0;
    /** ALTCRIT table (metres at 2^-24): 25KFT, 50KFT, 50FT, 30KFT. */
    public static final double[] ALTCRIT = {7620, 15240, 15.24, 9144};
    public static final double ALT_SCALE = 1.0 / (1L << 24);
    /** Priority/phase constants. */
    public static final int PRIO20 = 020000, PRIO21 = 021000, PRIO22 = 022000, PRIO23 = 023000,
        PRIO25 = 025000, PRIO31 = 031000, PRIO32 = 032000;

    private final Agc agc;
    private final ServiceRoutines service;

    // ---- collaborators (set by the caller; absent neighbours are tolerated)
    private Executive exec;
    private Waitlist waitlist;
    private Runnable gotoPooh;

    private double[] vec(String s) { return VecOps.getV(agc, s); }
    private void setVec(String s, double[] v) { VecOps.setV(agc, s, v); }

    public Servicer(Agc agc) {
        this.agc = agc;
        this.service = new ServiceRoutines(agc);
    }

    public void setExecutive(Executive e) { this.exec = e; }
    public void setWaitlist(Waitlist w) { this.waitlist = w; }
    public void setGotoPooh(Runnable r) { this.gotoPooh = r; }

    // ---- helpers
    private int g(String s) { return agc.get(s); }
    private void s(String sym, int v) { agc.set(sym, v); }
    /** Flag bit mask symbol (value defined by the flag-word assignments). */
    private int m(String bit) { return agc.get(bit); }
    private boolean test(String word, String bit) { return (g(word) & m(bit)) != 0; }
    private void setBits(String word, String bit) { s(word, g(word) | m(bit)); }
    private void clearBits(String word, String bit) { s(word, g(word) & ~m(bit)); }
    private void upflag(String word, String bit) { setBits(word, bit); }
    private void downflag(String word, String bit) { clearBits(word, bit); }
    private void negZero(String sym) { s(sym, Agc.WORD_MASK); s(sym + "+1", Agc.WORD_MASK); }
    private int sgn(String s) { return Agc.toSigned(g(s)); }
    private void incr(String s) { s(s, g(s) + 1); }

    // ================================================================ PREREAD ... READACCS

    /**
     * PREREAD: 5.7 spot (skip LASTBIAS after restart); do the last gyro compensation in free fall
     * (LASTBIAS job), then BIBIBIAS.
     */
    public void preread() {
        gnufaze5(7);                                   // CAF SEVEN, TC GNUFAZE5
        if (exec != null) {
            exec.novac(PRIO21, this::lastbias);        // 2CADR LASTBIAS
        }
        bibibias();
    }

    /** Hook for the LASTBIAS job (in IMU compensation); supplied by the caller. */
    protected void lastbias() { }

    /**
     * BIBIBIAS: clear and read the PIPAs last time in free fall (PIPASR+3), set V37FLAG and AVEGFLAG,
     * reset DRIFTFLAG, initialise the DV monitor and OUTROUTE, start NORMLIZE and go to READACCS.
     */
    public void bibibias() {
        pipasr3();                                     // do not destroy PIPTIME1
        setBits("FLAGWRD7", "SUPER011");               // V37FLAG and AVEGFLAG (bits 5 and 6)
        clearBits("FLAGWRD2", "DRFTBIT");              // RESET DRIFTFLAG
        s("PIPAGE", 4);                                // INITIALIZE DV MONITOR
        s("OUTROUTE", g("ENDJBCAD"));                  // OUTROUTE -> SERVEXIT+2 (end of job)
        if (exec != null) {
            exec.findVac(PRIO22, this::normlize);      // first entry to average G
        }
        goreadax(2);                                   // 5.2 spot for REREADAC and NORMLIZE
    }

    /** GOREADAX: phase change, then wait two seconds for READACCS. */
    private void goreadax(int phase) {
        gnutfaz5(phase);
        if (waitlist != null) {
            waitlist.waitlist(200, this::readaccs);    // CA 2SECS ... VARDELAY
        }
    }

    /**
     * READACCS: synchronise with the DAP (a PAXIS rupt about 70 ms after), read the PIPAs, set the
     * 5.5 spot, establish the SERVICER job, then (LM descent) R10/R11 timing, and recall.
     */
    public void readaccs() {
        int d = Agc.toSigned(g("TIME5")) - Agc.toSigned(OCT37771);       // CS OCT37771 ; AD TIME5
        // CCS A: >0 -> add -1, otherwise add +1 (via CS ONE / CA ONE)
        s("TIME5", g("TIME5") + (d > 0 ? Agc.fromSigned(-1) : 1));
        pipasr();                                       // READ THE PIPAS
        pipsdone();
    }

    /** PIPSDONE: 5.5 spot; PIPAGE=1; FINDVAC SERVICER; test connector bit; AVEGOUT / MAKEACCS. */
    public void pipsdone() {
        gnufaze5(5);
        redo55();
    }

    /** REDO5.5. */
    public void redo55() {
        s("PIPAGE", 1);
        if (exec != null) {
            exec.findVac(PRIO20, this::servicer);       // SET UP SERVICER JOB
        }
        s("DSALMOUT", g("DSALMOUT") | 0400);            // TURN ON TEST CONNECTOR OUTBIT (BIT9)
        if (!test("FLAGWRD7", "AVEGFBIT")) {            // AVEGFLAG DOWN
            avegout();
            return;
        }
        if (!test("FLAGWRD6", "MUNFLBIT")                // MUNFLAG CLEAR: BYPASS LR AND DISP.
            || sgn("PHASE2") != 0) {                     // PHASE 2 ACTIVATED: AVOID MULTIPLE R10
            makeaccs();
            return;
        }
        s("PIPCTR", 7);                                 // PIPCTR FOR 4X/SEC RATE
        // TBASE2 .05 SECONDS IN THE PAST: -TIME1 + 5 - 1/2 - 1/2 = 5 - TIME1 - 16383.. kept as (TIME1 - 5)
        s("TBASE2", Agc.fromSigned(Agc.toSigned(g("TIME1")) - 5));
        s("PHASE2+1", 17);                               // 2.21 SPOT FOR R10,R11 (-PHASE2 complement)
        s("PHASE2", 17);
        s("-PHASE2", Agc.fromSigned(~17 & Agc.WORD_MASK));
        if (waitlist != null) {
            waitlist.waitlist(020, this::r10r11);        // FIRST R10,R11 IN .200 SECONDS (OCT24)
        }
        makeaccs();
    }

    /** Hook: R10,R11 waitlist task (landing analog displays). */
    protected void r10r11() { }

    /** MAKEACCS: 5.4 phase change and recall of READACCS. */
    public void makeaccs() { goreadax(4); }

    /** AVEGOUT: set up final SERVICER exit (AVGEND), 5.4 spot, end task. */
    public void avegout() {
        s("AVGEXIT", g("AVOUTCAD"));                    // AVOUTCAD = 2CADR AVGEND
        gnutfaz5(4);                                    // REREADAC will end the task without READACCS
        // TC TASKOVER
    }

    /** GNUTFAZ5: set TBASE5 from TIME1 then fall into GNUFAZE5. */
    public void gnutfaz5(int phase) {
        s("TBASE5", Agc.fromSigned(-Agc.toSigned(g("TIME1"))));   // CS TIME1, TS TBASE5 (as original)
        gnufaze5(phase);
    }

    /** GNUFAZE5: -PHASE5 / PHASE5 set from A and its complement. */
    public void gnufaze5(int phase) {
        s("PHASE5", phase);
        s("-PHASE5", Agc.fromSigned(~phase & Agc.WORD_MASK));
    }

    // ================================================================ SERVICER job

    /**
     * SERVICER: restart PHASCHNG (OCT 16035 / 20000 GETABVAL), initialise 1/PIPADT, 1/PIPA compensation,
     * then GETABVAL.
     */
    public void servicer() {
        s("1/PIPADT", PRIO31);                          // CAF PRIO31, TS 1/PIPADT
        pipaComp();                                     // BANKCALL 1/PIPA
        getabval();
    }

    /** Hook: 1/PIPA PIPA compensation (IMU compensation package). */
    protected void pipaComp() { }

    /**
     * GETABVAL: ABDELV = |DELV| (cm/sec x 2^-14), ABDVCONV = ABDELV*KPIP (m/cs x 2^-5), MASS1 = MASS;
     * MASSMON: update mass unless on the surface; MOONSPOT: DVTOTAL += ABDELV*KPIP1; AVERAGEG.
     */
    public void getabval() {
        double[] delv = vec("DELV");
        int abdelv = (int) Math.round(VecOps.abs(delv) * 16384);
        s("ABDELV", abdelv);
        long abdvconv = (long) abdelv * sgn("KPIP");    // EXTEND MP KPIP -> DP
        VecOps.setDpRaw(agc, "ABDVCONV", abdvconv);
        VecOps.setDpRaw(agc, "MASS1", VecOps.getDpRaw(agc, "MASS"));
        // MASSMON
        if (test("FLAGWRD8", "SURFFBIT")) {              // CS FLAGWRD8 / MASK SURFFBIT : BZF => on surface
            boolean aps = test("FLGWRD10", "APSFLBIT");
            long vex = VecOps.getDpRaw(agc, aps ? "APSVEX" : "DPSVEX");
            if (vex != 0) {
                // DV Q: ABDVCONV / VEX, then MP MASS and DAS MASS1
                double ratio = (double) VecOps.getDpRaw(agc, "ABDVCONV") / (double) vex;
                long dm = Math.round(ratio * sgn("MASS"));
                VecOps.setDpRaw(agc, "MASS1", VecOps.getDpRaw(agc, "MASS1") + dm);
            }
        }
        // MOONSPOT
        long inc = (long) abdelv * sgn("KPIP1");         // ABDELV at 2^-14 cm/sec -> 2^-7 m/cs
        VecOps.setDpRaw(agc, "DVTOTAL", VecOps.getDpRaw(agc, "DVTOTAL") + inc);   // DAS DVTOTAL
        tmptospt();
        quictrig();                                      // BANKCALL QUICTRIG
        fleshpot(g("XNBPIPAD"));                         // BANKCALL FLESHPOT
        averageg();
    }

    /** Hook: QUICTRIG (sines/cosines of CDUSPOT). */
    protected void quictrig() { }

    /** Hook: FLESHPOT (transform XNBPIP column unit vectors). */
    protected void fleshpot(int adr) { }

    /** AVERAGEG: BON MUNFLAG RVBOTH ... else CALCRVG; then GOSERV (QUIKFAZ5) and COPYCYCL. */
    public void averageg() {
        if (test("FLAGWRD6", "MUNFLAG")) {
            rvboth();
        } else {
            calcrvg();
        }
        goserv();
    }

    /** GOSERV: QUIKFAZ5 then COPYCYCL. */
    public void goserv() {
        quikfaz5();
        copycycl();
    }

    /**
     * COPYCYCL: COPYCYC, zero PIPATMP*, clear STEERSW, then DVMON: use IDLEFLAG/AUXFLAG toggling,
     * compare ABDELV with DVTHRUSH.
     */
    public void copycycl() {
        copycyc();
        s("PIPATMPX", 0);
        s("PIPATMPY", 0);
        s("PIPATMPZ", 0);
        clearBits("FLAGWRD2", "STEERBIT");               // CLEAR STEERSW PRIOR TO DVMON
        if (test("FLAGWRD7", "IDLEFBIT")) {              // IDLEFLAG = 1: set AUXFLAG to 0
            clearBits("FLAGWRD6", "AUXFLBIT");
            usejets();
            return;
        }
        if (!test("FLAGWRD6", "AUXFLBIT")) {             // AUXFLAG = 0: set it to 1
            setBits("FLAGWRD6", "AUXFLBIT");
            usejets();
            return;
        }
        dvmon();
    }

    /** DVMON: ABDELV - DVTHRUSH <= 0 -> LOTHRUST; else set STEERSW and DVCNTSET. */
    public void dvmon() {
        if (sgn("ABDELV") - sgn("DVTHRUSH") <= 0) {
            lothrust();
            return;
        }
        setBits("FLAGWRD2", "STEERBIT");                 // SET STEERSW
        dvcntset();
    }

    /** DVCNTSET: allow two passes max; GTS if gimbal OK and not APS, else jets. */
    public void dvcntset() {
        s("DVCNTR", 1);
        if (test("FLGWRD10", "APSFLBIT")) {              // BRANCH IF APSFLAG IS SET
            usejets();
            return;
        }
        if ((g("CHAN32") & 0400) == 0) {                 // CHECK GIMBAL FAIL BIT (BIT9)
            usejets();
            return;
        }
        // USEGTS
        s("DAPBOOLS", g("DAPBOOLS") & ~m("USEQRJTS"));
        servout();
    }

    /**
     * LOTHRUST: low thrust. If DVCNTR positive -> DECCNTR (decrement, STOPRATE, USEJETS); else
     * if COMFAIL not active (PHASE4 clear) start it (4.37 spot, PRIO25).
     */
    public void lothrust() {
        quikfaz5();
        int dvcntr = sgn("DVCNTR");
        if (dvcntr > 0) {                                // CCS DVCNTR : positive
            s("DVCNTR1", dvcntr - 1);                    // DECCNTR
            quikfaz5();
            s("DVCNTR", g("DVCNTR1"));
            stoprate();                                  // IBNKCALL STOPRATE: desired rates to zero
            usejets();
            return;
        }
        if (sgn("PHASE4") != 0) {                        // COMFAIL JOB ACTIVE
            servout();
            return;
        }
        s("PHASE4", 037);                                // 4.37 spot for COMFAIL (OCT 00374)
        if (exec != null) {
            exec.novac(PRIO25, this::comfail);
        }
        servout();
    }

    /** Hooks. */
    protected void stoprate() { }
    protected void comfail() { }

    /** USEJETS: DAPBOOLS |= USEQRJTS, then SERVOUT. */
    public void usejets() {
        s("DAPBOOLS", g("DAPBOOLS") | m("USEQRJTS"));
        servout();
    }

    /**
     * SERVOUT: RELINT; 1/ACCS; restore push-list pointer; QUIKFAZ5; exit through AVGEXIT
     * (AVGEND, MUNRVG users etc.) whether or not there is thrust (STEERSW conveys this).
     */
    public void servout() {
        accs();                                          // BANKCALL 1/ACCS
        s("PUSHLOC", g("PRIORITY") & 0777);              // MASK LOW9
        quikfaz5();
        if (avgExit != null) {
            avgExit.run();                               // DCA AVGEXIT ; DXCH Z
        }
    }

    private Runnable avgExit;

    /** Installs the current AVGEXIT target (e.g. P-program steering, or {@link #avgend()}). */
    public void setAvgExit(Runnable r) { this.avgExit = r; }

    /** Hook: 1/ACCS (acceleration conversion). */
    protected void accs() { }

    /**
     * AVGEND: final average-G exit. Sets up free-fall gyro compensation, sets DRIFTFLG, PIPFREE,
     * turns off the test connector bit, group 5 off / group 2 on, turns off R29/R10, resets MUNFLAG
     * and calls AVETOMID, clears V37FLAG and returns to OUTROUTE.
     */
    public void avgend() {
        s("1/PIPADT", g("PIPTIME+1"));
        upflag("FLAGWRD2", "DRIFTFLG");                   // SET DRIFT FLAG
        pipfree();
        s("DSALMOUT", g("DSALMOUT") & ~0400);              // CS BIT9 ; WAND DSALMOUT
        gnufaze5(0);                                      // 2PHSCHNG: group 5 off (5), group 2 on (05022)
        s("PHASE2", 022);
        setBits("FLAGWRD3", "NOR29FLG");                  // SHUT OFF R29 WHEN SERVICER ENDS
        clearBits("FLAGWRD2", "SWANDISP");                // SHUT OFF R10 WHEN SERVICER ENDS
        clearBits("FLAGWRD6", "MUNFLAG");                 // RESET MUNFLAG
        avetomid();
        clearBits("FLAGWRD7", "V37FLAG");
        avertrn();
    }

    /** AVERTRN / OUTGOAVE: return to OUTROUTE. */
    public void avertrn() {
        if (outroute != null) {
            outroute.run();
        }
    }

    private Runnable outroute;
    public void setOutroute(Runnable r) { this.outroute = r; }

    /** Hooks. */
    protected void pipfree() { }
    protected void avetomid() { }

    // ================================================================ SERVIDLE / SERVEXIT

    /**
     * SERVIDLE: disconnect SERVICER from all guidance (AVGEXIT = SERVEXIT), disconnect the DV monitor
     * (set IDLEFLAG), terminate R12 (LRBYBIT), turn off phases 1,2(unless MUNFLAG),3,6; 4.33 spot for
     * GOP00FIX; then WHIMPER (software restart, proceed to GOTOPOOH).
     */
    public void servidle() {
        avgExit = this::serverit;
        setBits("FLAGWRD7", "IDLEFBIT");
        s("FLGWRD11", m("LRBYBIT"));                     // CAF LRBYBIT ; TS FLGWRD11
        negZero("PHASE1");
        if (!test("FLAGWRD6", "MUNFLBIT")) {             // DO NOT TURN OFF PHASE 2 IF MUNFLAG SET
            negZero("PHASE2");
        }
        negZero("PHASE3");
        negZero("PHASE6");
        s("PHASE4", 033);                                 // 4.33 SPOT FOR GOP00FIX
        s("-PHASE4", Agc.fromSigned(~033 & Agc.WORD_MASK));
        whimper();
    }

    /** Hook: WHIMPER (software restart then GOTOPOOH while SERVICER continues). */
    protected void whimper() { if (gotoPooh != null) gotoPooh.run(); }

    /** SERVEXIT: phase change 0035 then end of job. */
    public void serverit() { servexit(); }

    public void servexit() {
        s("PHASE0", 035);                                 // TC PHASCHNG / OCT 00035
        servexit2();
    }

    /** SERVEXIT+2: TCF ENDOFJOB. */
    public void servexit2() { if (exec != null) exec.endOfJob(); }

    // ================================================================ NORMLIZE / COPYCYC

    /**
     * NORMLIZE: first entry to average G. If not MUNFLAG: R = RN1*REFSMMAT*2^6 -> MUNGRAV style; V = VN1;
     * UHYP = unit(V(CSM) x R(CSM)); makes sure group 2 is off, then NORMLIZ2.
     */
    public void normlize() {
        double[][] refsmmat = VecOps.getM(agc, "REFSMMAT");
        double[] rn1 = vec("RN1");
        if (!test("FLAGWRD6", "MUNFLAG")) {
            double[] r = VecOps.scale(VecOps.vxm(rn1, refsmmat), 64.0);     // VSL6, MXV
            setVec("R", r);
            calcgrav();                                                      // via MUNGRAV label (STCALL R)
            double[] v = VecOps.scale(VecOps.vxm(vec("VN1"), refsmmat), 2.0); // VSL1
            setVec("V", v);
            setVec("V(CSM)", v);                                             // STOVL V ... V(CSM) pair
        }
        double[] vcsm = vec("V(CSM)"), rcsm = vec("R(CSM)");
        setVec("UHYP", VecOps.unit(VecOps.cross(vcsm, rcsm)));              // VXV UNIT
        // ASCSPOT: make sure group 2 is off
        negZero("PHASE2");
        normliz2();
    }

    /** NORMLIZ1: CALL CALCGRAV. */
    public void normliz1() { calcgrav(); }

    /** NORMLIZ2: COPYCYC +1 with 18 words (no mass copy); END JOB. */
    public void normliz2() {
        copycycN(EIGHTEEN);
        if (exec != null) exec.endOfJob();
    }

    /** COPYCYC: copy RN1.. -> RN.. (20 words, DEC 20 / OCT24), under INHINT. */
    public void copycyc() { copycycN(COPYCYC_WORDS); }

    /** COPYCYC +1/+2: copy DP pairs RN1+k -> RN+k for k = n (even), n-2 ... 0. */
    public void copycycN(int n) {
        for (int k = n & ~1; k >= 0; k -= 2) {
            s(nameOff("RN", k), g(nameOff("RN1", k)));
            s(nameOff("RN", k + 1), g(nameOff("RN1", k + 1)));
        }
    }

    private static String nameOff(String base, int off) { return off == 0 ? base : base + "+" + off; }

    // ================================================================ PIPA reader

    /**
     * PIPASR: read PIPA counters carefully so the routine is restartable. Result in DELV (high
     * order parts), CDU readings in CDUTEMP, time in PIPTIME1; PGUIDE = PIPTIME1 - PIPTIME.
     */
    public void pipasr() {
        VecOps.setDpRaw(agc, "PIPTIME1", VecOps.getDpRaw(agc, "TIME2"));   // CURRENT TIME POSITIVE VALUE
        pipasr3();
    }

    /** PIPASR+3: initialise TEM* to -0, zero DELV partials, PIPAGE = 0 (reading in progress). */
    public void pipasr3() {
        s("TEMX", Agc.WORD_MASK);
        s("TEMY", Agc.WORD_MASK);
        s("TEMZ", Agc.WORD_MASK);
        s("DELVZ", 0); s("DELVZ+1", 0);
        s("DELVY", 0); s("DELVY+1", 0);
        s("DELVX+1", 0);
        s("PIPAGE", 0);                                  // SHOW PIPA READING IN PROGRESS
        repip1();
    }

    /** REPIP1: X and Y PIPs read (DCS PIPAX; DXCH TEMX ...); PIPAs set to -0 as read. */
    public void repip1() {
        // DCS PIPAX: A = -PIPAX, L = -PIPAY ; DXCH TEMX ; DXCH PIPAX (exchange with counters)
        int px = g("PIPAX"), py = g("PIPAY");
        s("TEMX", ~px & Agc.WORD_MASK);
        s("TEMY", ~py & Agc.WORD_MASK);
        s("PIPAX", Agc.WORD_MASK);
        s("PIPAY", Agc.WORD_MASK);
        s("DELVX", px);
        s("DELVY", py);
        repip3();
    }

    /** REPIP3: repeat process for the Z PIP. */
    public void repip3() {
        int pz = g("PIPAZ");
        s("TEMZ", ~pz & Agc.WORD_MASK);
        s("PIPAZ", Agc.WORD_MASK);
        dodelvz(pz);
    }

    private void dodelvz(int pz) {
        s("DELVZ", pz);
        repip4();
    }

    /**
     * REPIP4: compute guidance period PGUIDE = PIPTIME1 - PIPTIME; read CDUs into CDUTEMP, copy the
     * PIPA readings to PIPATMP.
     */
    public void repip4() {
        VecOps.setDpRaw(agc, "PGUIDE", VecOps.getDpRaw(agc, "PIPTIME1") - VecOps.getDpRaw(agc, "PIPTIME"));
        s("CDUTEMPX", g("CDUX"));
        s("CDUTEMPY", g("CDUY"));
        s("CDUTEMPZ", g("CDUZ"));
        s("PIPATMPX", g("DELVX"));
        s("PIPATMPY", g("DELVY"));
        s("PIPATMPZ", g("DELVZ"));
    }

    /**
     * REREADAC: restart entry. If PIPAGE is positive, reading had not started (READACCS); otherwise
     * continue the interrupted read: Z done -> REPIP4; Y done -> zero Z PIP; else check TEMX.
     */
    public void rereadac() {
        if (sgn("PIPAGE") > 0) {
            readaccs();
            return;
        }
        // return from PIPASR is PIPSDONE
        int delvz = sgn("DELVZ");
        if (delvz != 0) {                                // Z DONE
            repip4();
            pipsdone();
            return;
        }
        int delvy = sgn("DELVY");
        if (delvy == 0) {                                // Y NOT DONE, CHECK X
            chktemx();
            return;
        }
        // Y done: zero Z PIP (LXCH PIPAZ with L = 0)
        int pipaz = g("PIPAZ");
        s("PIPAZ", 0);
        int temz = sgn("TEMZ");
        if (temz != 0 || g("TEMZ") != Agc.WORD_MASK) {   // TEMZ NOT = -0 : contains -PIPAZ value
            dodelvz(Agc.fromSigned(-temz));
        } else {                                          // TEMZ = -0, L HAS ZPIP VALUE
            s("DELVZ", pipaz);
            repip4();
        }
        pipsdone();
    }

    /** CHKTEMX: has TEMX changed? No -> REPIP1; yes -> DELVX = -TEMX, DELVY = -TEMY, zero X/Y PIPs. */
    public void chktemx() {
        if (g("TEMX") == Agc.WORD_MASK || sgn("TEMX") == 0) {
            repip1();
            pipsdone();
            return;
        }
        s("DELVX", ~g("TEMX") & Agc.WORD_MASK);
        s("DELVY", ~g("TEMY") & Agc.WORD_MASK);
        s("PIPAX", 0);
        s("PIPAY", 0);
        repip3();
        pipsdone();
    }

    /** DONEADR: GENADR PIPSDONE. */
    public void doneadr() { pipsdone(); }

    // ================================================================ TMPTOSPT

    /** TMPTOSPT: load the CDUs corresponding to PIPTIME into the CDUSPOT vector. */
    public void tmptospt() {
        s("CDUSPOTY", g("CDUTEMPY"));
        s("CDUSPOTZ", g("CDUTEMPZ"));
        s("CDUSPOTX", g("CDUTEMPX"));
    }

    // ================================================================ LR logic (R12)

    /**
     * LRHTASK: waitlist task set by READACCS during descent braking when altitude is under 25,000 ft.
     * Clears the altitude-measurement-made discrete and starts LRHJOB 50 ms before the next READACCS.
     */
    public void lrhtask() {
        if (!test("FLGWRD11", "LRBYBIT") && test("FLGWRD11", "READLBIT") && !test("FLGWRD11", "NOLRRBIT")) {
            if (exec != null) exec.novac(PRIO32, this::lrhjob);       // LR READ OK
        }
        grp2off();
    }

    /** GRP2OFF: -PHASE2 = -0 ; TCF R10,R11A. */
    public void grp2off() {
        negZero("PHASE2");
        r10r11a();
    }

    /** Hook: R10,R11A (in landing analog displays). */
    protected void r10r11a() { }

    /**
     * HIGATASK: ~6 s before high gate sets HIGATE (BIT11) and LR inhibit (BIT10) flags in FLGWRD11 (PRIO3),
     * starts HIGATJOB (LR repositioning POS1 to POS2) and continues the SERVICER.
     */
    public void higatask() {
        s("FLGWRD11", g("FLGWRD11") | 0600);                    // PRIO3 = bits 8,9 (HIGATE, LR INHIBIT)
        if (exec != null) exec.findVac(PRIO32, this::higatjob);
        contserv();
    }

    /**
     * MUNRETRN: return from MUNRVG. Bypass LR logic if LRBYBIT. Determine which test applies
     * (35 kft, 30 kft, high gate, POS1/POS2) and then UPDATCHK / CONTSERV.
     */
    public void munretrn() {
        if (test("FLGWRD11", "LRBYBIT")) {
            copycyc1();
            return;
        }
        if (!test("FLGWRD11", "READLBIT")) {             // ALT WAS > 35000 FT LAST CYCLE
            k35chk();
            return;
        }
        if (!test("FLGWRD11", "XORFLBIT")) {             // NOT BELOW 30000 FT LAST PASS
            xorchk();
            return;
        }
        hitest();
    }

    /** HITEST: past high gate? */
    public void hitest() {
        if (!test("FLGWRD11", "PSTHIBIT")) {
            higatchk();
            return;
        }
        pos2chk();
    }

    /** POS2CHK: verify the LR is in position 2. */
    public void pos2chk() {
        if ((g("CHAN33") & 0100) == 0) {                 // BIT7 : in POS2
            updatchk();
            return;
        }
        if ((g("CHAN12") & 010000) == 0) {               // BIT13 : not in POS2 nor repositioning
            lrposalm();
            return;
        }
        contserv();                                       // LR BEING REPOSITIONED
    }

    /** HIGATCHK: TTF/8 + RPCRTIME <= 0 -> check QSW; reposition if UXBXP > QSW; else POS1CHK. */
    public void higatchk() {
        if (sgn("TTF/8") + sgn("RPCRTIME") > 0) {        // TTF IS NEGATIVE
            pos1chk();
            return;
        }
        int xnbpip = sgn("XNBPIP");                      // E-bank switch to read UXBXP
        if (-xnbpip + sgn("RPCRTQSW") <= 0) {            // UXBXP > QSW -> REPOSITION
            higatask();
            return;
        }
        pos1chk();
    }

    /** POS1CHK: LR in position 1 (CHAN33 bit 6)? Then UPDATCHK else alarm 511. */
    public void pos1chk() {
        if ((g("CHAN33") & 040) == 0) {
            updatchk();
            return;
        }
        lrposalm();
    }

    /** LRPOSALM: alarm 511 (LR not in proper position), bypass update and continue SERVICER. */
    public void lrposalm() {
        s("ALARM", 0511);
        contserv();
    }

    /** CONTSERV: clear LR measurement-made discretes, set LR lights (R12LITES), then COPYCYC1. */
    public void contserv() {
        s("FLGWRD11", g("FLGWRD11") & ~BITS4_7);
        r12lites();
        copycyc1();
    }

    /** Hook: R12LITES. */
    protected void r12lites() { }

    /**
     * COPYCYC1 / R29?: QUIKFAZ5; if NOR29FLG or READRFLG set, or RR is zeroing / not AUTO, R29 not allowed
     * (R29NODES); if RR repositioning/remoding come back in two seconds (NOR29NOW); else R29.
     */
    public void copycyc1() {
        quikfaz5();
        if ((g("FLAGWRD3") & m("NR29&RDR")) != 0) {
            r29nodes();
            return;
        }
        if ((g("RADMODES") & OCT10002) != 0) {
            r29nodes();
            return;
        }
        if ((g("RADMODES") & PRIO22) != 0) {
            nor29now();
            return;
        }
        r29();
    }

    /** Hook: R29 (rendezvous radar designate). */
    protected void r29() { }

    /** R29NODES: show designation off. */
    public void r29nodes() {
        s("RADMODES", g("RADMODES") & ~m("DESIGBIT"));
        nor29now();
    }

    /**
     * NOR29NOW: interpretive. HCALC = |R1S| - /LAND/ (2^24 m); ALTBITS in bit units;
     * UHZP = downrange half-unit vector UNIT(UNIT/R/ x UHYP); RN1 = R1S*REFSMMAT/16, VN1 = V1S*REFSMMAT*2;
     * altitude rate compensation DALTRATE; then COPYCYC2.
     */
    public void nor29now() {
        double[] r1s = vec("R1S"), v1s = vec("V1S");
        double hcalc = VecOps.abs(r1s) - VecOps.getDp(agc, "/LAND/");
        VecOps.setDp(agc, "HCALC", hcalc);
        VecOps.setDp(agc, "HCALC1", hcalc);
        double altbits = hcalc * VecOps.getDp(agc, "ALTCONV");              // DMPR ALTCONV ; RTB SGNAGREE
        VecOps.setDp(agc, "ALTBITS", altbits);
        double[] unitR = vec("UNIT/R/");
        setVec("UHZP", VecOps.unit(VecOps.cross(unitR, vec("UHYP"))));     // DOWNRANGE HALF-UNIT VECTOR
        double[][] refsmmat = VecOps.getM(agc, "REFSMMAT");
        setVec("RN1", VecOps.scale(VecOps.vxm(r1s, refsmmat), 1.0 / 16));  // VXM, VSR4 (m x 2^-29)
        setVec("VN1", VecOps.scale(VecOps.vxm(v1s, refsmmat), 2.0));       // VXM, VSL1 (m/cs x 2^-7)
        double[] cr = VecOps.cross(unitR, v1s);                             // VXV ABVAL
        double h = VecOps.abs(cr);
        double r = VecOps.abs(r1s);
        double mpac = r != 0 ? 2.0 * h * h / r * VecOps.getDp(agc, "ARCONV1") : 0;   // SL1 DSQ DDV, DMPR ARCONV1
        copycyc2(mpac);
    }

    /**
     * COPYCYC2: update RUNIT for R10 (UNIT/R/ high words), DALTRATE from MPAC, and copy R1S/V1S to R/V.
     */
    public void copycyc2(double mpacAltRateComp) {
        s("RUNIT", g("UNIT/R/"));
        s("RUNIT+1", g("UNIT/R/+2"));
        s("RUNIT+2", g("UNIT/R/+4"));
        s("DALTRATE", (int) Math.round(mpacAltRateComp * 16384) & Agc.WORD_MASK);
        for (int k = 0; k < 6; k += 2) {
            s(nameOff("R", k), g(nameOff("R1S", k)));
            s(nameOff("R", k + 1), g(nameOff("R1S", k + 1)));
            s(nameOff("V", k), g(nameOff("V1S", k)));
            s(nameOff("V", k + 1), g(nameOff("V1S", k + 1)));
        }
        copycycl();
    }

    // ---- ALTCHK

    /**
     * ALTCHK: compare HCALC with the table entry N (ALTCRIT: 25KFT=0, 50KFT=2, 50FT=4, 30KFT=6).
     * Returns true if HCALC > criterion (L+3 return), false if HCALC &lt;= criterion (L+4 return).
     */
    public boolean altchk(int n) {
        double crit = ALTCRIT[n / 2] * ALT_SCALE;
        double hcalc = VecOps.getDp(agc, "HCALC");
        return hcalc - crit > 0;
    }

    /** XORCHK: below 30000 ft? If so inhibit X-axis override (XOVINFLG) and set XORFLG. */
    public void xorchk() {
        if (!altchk(6)) {
            upflag("FLGWRD11", "XOVINFLG");
            upflag("FLGWRD11", "XORFLG");
        }
        hitest();
    }

    /** 35KCHK: below 35,000 ft? Set READLR flag to enable LR reading. */
    public void k35chk() {
        if (!altchk(2)) {
            upflag("FLGWRD11", "READLR");
        }
        contserv();
    }

    // ================================================================ gravity / average G

    /**
     * CALCGRAV (INTERPRETIVE): UNIT/R/ = unit(R); if RTX2 != 0 (lunar) skip oblateness (CALCGRV1); else
     * add J2 and J20 terms to give UNITGOBL; then GDT1/2 = -MUDT,1 / |R|^2 scaled by 2^7 m/cs.
     */
    public void calcgrav() {
        double[] r = vec("R");
        double[] unitR = VecOps.unit(r);
        setVec("UNIT/R/", unitR);
        double rmagsq = VecOps.dot(r, r);
        VecOps.setDp(agc, "34D", rmagsq);
        double[] total = new double[3];
        int rtx2 = sgn("RTX2");
        boolean earth = rtx2 == 0;                       // RTX2 = 0 IF EARTH ORBIT, 2 IF LUNAR
        if (earth) {
            double[] unitz = {0, 0, 1};
            double cosphi = VecOps.dot(unitz, unitR);
            double c = 2 * cosphi;                       // SL1
            double term = DP1_20 - c * c;                // DSQ, BDSU DP1/20
            double resqOverRnsq = VecOps.getDp(agc, "RESQ") / rmagsq;   // (RE/RN)SQ
            double k1 = term * VecOps.getDp(agc, "20J") * resqOverRnsq;
            double k2 = c * VecOps.getDp(agc, "2J") * resqOverRnsq;
            double[] vec = VecOps.add(VecOps.scale(unitR, k1), VecOps.scale(unitz, 2 * k2));
            setVec("UNITGOBL", vec);
            total = VecOps.add(vec, unitR);              // MPAC = UNIT GRAVITY VECTOR
        } else {
            total = unitR;
        }
        // CALCGRV1: normalise RMAGSQ, divide -MU by it
        double mu = VecOps.getDp(agc, "-MUDT");
        double scale = rmagsq != 0 ? mu / rmagsq : 0;
        setVec("GDT1/2", VecOps.scale(total, scale));    // SCALED AT 2^7 M/CS
    }

    /**
     * CALCRVG (INTERPRETIVE): DELVREF = DELV*REFSMMAT*KPIP1*2; RN1 = RN + (VN + (DELV+GDT)/2 ... ) * PGUIDE;
     * recompute gravity at the new position; VN1 = VN + DELVREF/2 + GDT/2 + GDT1/2.
     */
    public void calcrvg() {
        double[][] refsmmat = VecOps.getM(agc, "REFSMMAT");
        double kpip1 = sgn("KPIP1");
        double[] delvref = VecOps.scale(VecOps.vxm(vec("DELV"), refsmmat), 2 * kpip1);  // VXSC VSL1 KPIP1
        setVec("DELVREF", delvref);
        double[] half = VecOps.scale(delvref, 0.5);                                      // VSR1 PUSH
        double[] sum = VecOps.add(half, vec("GDT/2"));                                   // (DV-OLDGDT)/2
        double pguide = VecOps.getDpRaw(agc, "PGUIDE");
        double[] rn1 = VecOps.add(VecOps.scale(VecOps.add(sum, vec("VN")), pguide * 64), vec("RN"));  // SL 6
        setVec("RN1", rn1);                                                              // TEMP STORAGE OF RN (2^29 m)
        setVec("R", rn1);
        calcgrav();
        double[] vn1 = VecOps.add(VecOps.add(sum, vec("GDT1/2")), vec("VN"));            // VAD VAD VAD VN
        setVec("VN1", vn1);                                                              // TEMP STORAGE OF VN (2^7 m/cs)
    }

    /**
     * RVBOTH: special average-G for MUNFLAG with both vehicles. R1S = R(CSM) + (V(CSM) + G(CSM))*PGUIDE/2^11;
     * MUNGRAV; V1S = V(CSM) + G(CSM) + GDT1/2; then store back into G(CSM), R(CSM), V(CSM) with QUIKFAZ5s.
     */
    public void rvboth() {
        double[] g0 = vec("G(CSM)");
        double[] vcsm = vec("V(CSM)");
        double pguide = VecOps.getDpRaw(agc, "PGUIDE");
        double[] r1s = VecOps.add(vec("R(CSM)"), VecOps.scale(VecOps.add(vcsm, g0), pguide * SHIFT11));
        setVec("R1S", r1s);
        mungrav(r1s);
        double[] v1s = VecOps.add(VecOps.add(vcsm, g0), vec("GDT1/2"));
        setVec("V1S", v1s);
        quikfaz5();
        setVec("G(CSM)", vec("GDT1/2"));
        setVec("R(CSM)", r1s);
        setVec("V(CSM)", v1s);
        quikfaz5();
        munrvg();
    }

    /**
     * MUNRVG: special average-G for thrusting programs near a spherical Moon (stable-member frame).
     * R1S = R + (DELV*KPIP2/2... + V)*PGUIDE/2^11 ; V1S = V + DELV + GDT; ABVEL, HDOTDISP, DELVS (lunar rotation
     * correction WM x R1S) and HCALC = |R1S| - /LAND/; then MUNRETRN.
     */
    public void munrvg() {
        double[] delv = VecOps.scale(vec("DELV"), sgn("KPIP2"));       // 2^8 m/cs units
        double[] gdt = vec("GDT/2");
        double[] half = VecOps.add(delv, gdt);                          // (DELV+GDT)/2 units 2^7
        double[] v = vec("V");
        double pguide = VecOps.getDpRaw(agc, "PGUIDE");
        double[] r1s = VecOps.add(VecOps.scale(VecOps.add(half, v), pguide * SHIFT11), vec("R"));
        setVec("R1S", r1s);                                              // STORED AT 2^24 M
        mungrav(r1s);
        double[] v1s = VecOps.add(VecOps.add(delv, gdt), v);
        setVec("V1S", v1s);
        VecOps.setDp(agc, "ABVEL", VecOps.abs(v1s));                     // SPEED FOR LR AND DISPLAYS
        double[] unitR = vec("UNIT/R/");
        VecOps.setDp(agc, "HDOTDISP", 2 * VecOps.dot(unitR, v1s));        // HDOT = V . UNIT(R) x 2^7 m/cs
        double[] delvs = VecOps.scale(VecOps.cross(vec("WM"), r1s), 4);   // LUNAR ROTATION CORRECTION (2^5)
        setVec("DELVS", delvs);
        VecOps.setDp(agc, "HCALC", VecOps.abs(r1s) - VecOps.getDp(agc, "/LAND/"));   // POS OR NEG
        munretrn();
    }

    /** MUNGRAV: UNIT/R/ = unit(R); GDT1/2 = -MUDTMUN/(R.R) * SHIFT11 * UNIT/R/ (1/2 GDT at 2^7 m/cs). */
    public void mungrav(double[] r) {
        double mag2 = VecOps.dot(r, r);
        double[] u = VecOps.unit(r);
        setVec("UNIT/R/", u);
        double k = mag2 != 0 ? VecOps.getDp(agc, "-MUDTMUN") / mag2 * 64 : 0;   // SL 6 BDDV -MUDTMUN
        setVec("GDT1/2", VecOps.scale(u, k * SHIFT11 * 2048.0));
    }

    // ================================================================ LR position / velocity update

    /**
     * UPDATCHK: if LR update inhibited, CONTSERV; if no altitude measurement this cycle, check velocity
     * (VMEASCHK); else POSUPDAT.
     */
    public void updatchk() {
        if (test("FLGWRD11", "NOLRRBIT")) {
            contserv();
            return;
        }
        if (!test("FLGWRD11", "RNGEDBIT")) {
            vmeaschk();
            return;
        }
        posupdat();
    }

    /**
     * POSUPDAT: compute altitude difference DELTAH between the radar slant range projected on UNIT/R/
     * and HCALC; data reasonableness test after high gate (HFAIL on failure); if update not inhibited,
     * apply weighted position correction DELTAR = DELTAH*WH*(1 - H/HMAX)*UNIT/R/.
     */
    public void posupdat() {
        s("PUSHLOC", g("FIXLOC"));                        // SET PUSHLIST TO ZERO
        double[] hbeamsm = VecOps.vxm(vec("HBEAMNB"), VecOps.getM(agc, "XNBPIP"));   // HBEAM SM AT 2^2
        double[] v1s = VecOps.scale(vec("V1S"), 4);        // VSL2 : 2^5 M/CS
        double vAlongBeam = VecOps.dot(VecOps.add(v1s, vec("DELVS")), hbeamsm) * 4;  // 2^7 m/cs
        double mpac = vAlongBeam * VecOps.getDp(agc, "RADSKAL");     // SCALE TO RADAR COUNTS X 5
        if ((g("FLGWRD12") & m("ALTSCBIT")) == 0) {                   // LOW SCALE : rescale
            mpac *= sgn("SKALSKAL");
        }
        double hmeas = VecOps.getDp(agc, "HMEAS") + mpac * 128;      // DAD SL 7D : CORRECT FOR DOPPLER
        double slant = hmeas * VecOps.getDp(agc, "HSCAL");           // SLANT RANGE
        double[] slantVec = VecOps.scale(hbeamsm, slant);            // SLANT RANGE VECTOR 2^23 M
        double alt = VecOps.dot(slantVec, vec("UNIT/R/"));           // ALTITUDE 2^24 M
        double deltah = alt - VecOps.getDp(agc, "HCALC");
        VecOps.setDp(agc, "DELTAH", deltah);
        if (test("FLGWRD11", "PSTHIBIT")) {                          // reasonableness test after HIGATE
            double lim = Math.abs(deltah) - VecOps.getDp(agc, "DELQFIX");
            lim = lim * 8 - VecOps.getDp(agc, "HCALC");              // |DH| - (50 + HCALC/8) at 2^21
            incr("LRLCTR");
            if (lim >= 0) {                                          // DELTA H TOO LARGE
                hfail();
                return;
            }
            downflag("FLGWRD12", "HFLSHFLG");                        // TURN OFF ALT FAIL LAMP
        }
        // NOREASON
        if (test("FLGWRD11", "LRINHBIT")) {
            vmeaschk();                                              // UPDATE INHIBITED -- TEST VELOCITY ANYWAY
            return;
        }
        double hcalc = VecOps.getDp(agc, "HCALC");                   // rescaled to 2^28 m
        int hHigh = (int) (hcalc * 16384 / 16);
        if (hHigh > 16383) {                                         // HCALC > HMAX: bypass
            vmeaschk();
            return;
        }
        int h = Math.abs(hHigh);
        int lrhmax = sgn("LRHMAX");
        if (lrhmax - h <= 0) {                                       // H > HMAX: bypass
            vmeaschk();
            return;
        }
        double w = (double) sgn("LRWH") * (lrhmax - h) / lrhmax;     // WH(1 - H/HMAX)
        double[] gnur = VecOps.add(VecOps.scale(vec("UNIT/R/"), deltah * w * 2), vec("R1S"));
        setVec("GNUR", gnur);
        mungrav(gnur);
        quikfaz5();
        gnurvst(0, gnur);                                            // RUPDATED
        vmeaschk();
    }

    /**
     * VMEASCHK: QUIKFAZ5; if velocity reading available (VELDABIT) go VELUPDAT else VALTCHK.
     */
    public void vmeaschk() {
        quikfaz5();
        if (!test("FLGWRD11", "VELDABIT")) {
            valtchk();
            return;
        }
        velupdat();
    }

    /**
     * VELUPDAT: process LR velocity data. Converts the beam to SM using the LR CDUs (QUICTRIG, *NBSM*),
     * forms VM = VMEAS * VZSCAL, estimates V from the stored V + DELV + G*dt less lunar rotation, and
     * tests |VM - Vest| against 7.5 ft/s + |VM|/8 (VFAIL on failure), then incorporates with weight W.
     */
    public void velupdat() {
        int vsel = sgn("VSELECT");
        // X1 = -6 VSELECT, X2 = -2 VSELECT (index registers), EBANK switch to read LR CDUs
        s("CDUSPOT", g("LRYCDU"));
        s("CDUSPOT+2", g("LRZCDU"));
        s("CDUSPOT+4", g("LRXCDU"));
        quictrig();
        s("PUSHLOC", g("FIXLOC"));
        double[] beamsm = nbsm(vec(vbeamName(vsel)));                // VZBEAMNB,1 -> SM
        double vmeas = VecOps.getDp(agc, "VMEAS") * 4096;            // SL 12D
        double vm = vmeas * vzscal(vsel);                            // VZSCAL,2 : SCALE TO M/CS AT 2^6
        s("MODE", Agc.fromSigned(-1));                               // STORE MODE TO VECTOR
        double[] piptem = {sgn("PIPTEM"), sgn("PIPTEM+1"), sgn("PIPTEM+2")};
        double[] delv = VecOps.scale(piptem, sgn("KPIP1"));          // SCALE DELV TO 2^7 M/CS
        double tu = VecOps.getDpRaw(agc, "LRVTIME");
        double dt = (tu - VecOps.getDpRaw(agc, "PIPTIME")) / (double) SEC2_28;
        double[] gdt = VecOps.scale(vec("GDT/2"), dt * 2);           // G(N-1)(TU - T(N-1))
        double[] vu = VecOps.add(VecOps.add(delv, gdt), vec("V"));
        double[] vrel = VecOps.add(VecOps.scale(vu, 4), vec("DELVS"));      // 2^5 and subtract moon rotation
        double abvm = VecOps.abs(vrel);
        double lim = abvm / 16 + FT7_5;                              // ABS(VM)/8 + 7.5 AT 2^6
        double vest = VecOps.dot(vrel, beamsm);
        double dv = vm - vest;                                       // DELTAV = VMEAS - V(EST)
        incr("LRMCTR");
        if (Math.abs(dv) - lim >= 0) {
            vfail();
            return;
        }
        downflag("FLGWRD12", "VFLSHFLG");                            // TURN OFF VEL FAIL LAMP
        if (test("FLGWRD11", "VXINHBIT")) {                          // VX INHIBIT SET
            downflag("FLGWRD11", "VXINH");                           // RESET VX INHIBIT
            if (vsel == 2) {                                         // X AXIS: BYPASS UPDATE
                endvdat();
                return;
            }
        }
        vupdat(vsel, dv, beamsm);
    }

    private String vbeamName(int vsel) { return vsel == 0 ? "VZBEAMNB" : vsel == 1 ? "VYBEAMNB" : "VXBEAMNB"; }
    private double vzscal(int vsel) { return Agc.toSigned(agc.get("VZSCAL+" + (2 * vsel))) / 16384.0; }

    /** *NBSM*: transform NB vector to SM using XNBPIP-derived matrix (hook for interpreter). */
    protected double[] nbsm(double[] v) { return VecOps.vxm(v, VecOps.getM(agc, "XNBPIP")); }

    /**
     * VUPDAT: if update not inhibited: weight W = WVF for V &lt; VF, W = WV(1 - V/VMAX) up to VMAX, else 0;
     * in P65-P67 (MODREG &lt;= 64) use LRWVFF. GNUV = V1S + W * DELTAV * VBEAMSM. Stored through GNURVST (A=6).
     */
    public void vupdat(int vsel, double dv, double[] beamsm) {
        if (test("FLGWRD11", "LRINHBIT")) {
            valtchk();
            return;
        }
        int abvel = sgn("ABVEL");
        double w;
        if (abvel - sgn("LRVF") <= 0) {
            w = sgn("LRWVFZ+" + vsel);                               // USEVF : constant weight
        } else if (sgn("LRVMAX") - abvel <= 0) {
            w = 0;                                                   // V > VMAX, W = 0
        } else {
            w = (double) sgn("LRWVZ+" + vsel) * (sgn("LRVMAX") - abvel) / sgn("LRVMAX");   // WV(1 - V/VMAX)
        }
        if (sgn("MODREG") - 64 <= 0) {
            w = sgn("LRWVFF");                                       // P65-P67 constant
        }
        double[] gnuv = VecOps.add(VecOps.scale(beamsm, w * dv / 16384.0), vec("V1S"));
        setVec("GNUV", gnuv);
        quikfaz5();
        gnurvst(6, gnuv);
        endvdat();
    }

    /** ENDVDAT = VALTCHK. */
    public void endvdat() { valtchk(); }

    /**
     * VALTCHK: QUIKFAZ5; if READVEL set, read velocity (READV); else if |V| below 2000 ft/s set READVEL
     * and read; else CONTSERV.
     */
    public void valtchk() {
        quikfaz5();
        if (!test("FLGWRD11", "READVBIT")) {
            if (sgn("ABVEL") > 0 && -(sgn("ABVEL")) + (int) Math.round(KFT2_PER_SEC * 16384) <= 0) {
                contserv();                                           // V > 2000 FT/SEC
                return;
            }
            upflag("FLGWRD11", "READVEL");
        }
        readv();
    }

    /** READV: set up the LRVJOB at PRIO32 and CONTSERV. */
    public void readv() {
        if (exec != null) exec.novac(PRIO32, this::lrvjob);
        contserv();
    }

    /** GNURVST: store GNUR (= GNUV) into R1S (offset 0) or V1S (offset 6). */
    public void gnurvst(int offset, double[] v) {
        setVec(offset == 0 ? "R1S" : "V1S", v);
    }

    /** QUIKFAZ5: save the caller's return address in PHSNAME5 (restart protection for phase 5). */
    public void quikfaz5() {
        s("PHSNAME5", g("Q"));
    }

    /**
     * HFAIL: altitude data too far from prediction. If R != 0 and (L - R) &lt; 4, set HFLSHFLG (tracker fail light);
     * R = L; continue with VMEASCHK.
     */
    public void hfail() {
        int r = sgn("LRRCTR");
        if (r != 0 && ((sgn("LRLCTR") - r) & ~3) == 0) {
            upflag("FLGWRD12", "HFLSHFLG");
        }
        s("LRRCTR", g("LRLCTR"));
        vmeaschk();
    }

    /**
     * VFAIL: velocity data too far. If S != 0 and (M - S) &lt; 4 set VFLSHFLG; S = M; if VSELECT == 0 (Z) set VX
     * inhibit (error may be due to cross-lobe lock-up not detected on X); ENDVDAT.
     */
    public void vfail() {
        int sc = sgn("LRSCTR");
        if (sc != 0 && ((sgn("LRMCTR") - sc) & ~3) == 0) {
            upflag("FLGWRD12", "VFLSHFLG");
        }
        s("LRSCTR", g("LRMCTR"));
        if (sgn("VSELECT") == 0) {
            upflag("FLGWRD11", "VXINH");
        }
        endvdat();
    }

    // ================================================================ LR jobs / tasks

    /**
     * LRVJOB: begun when below 15,000 ft. Starts RDGIMS (170 ms) to sample CDUs and PIPAs midway, sequences
     * VSELECT (reset to 2 if zero), initialises the LR velocity read (LRVEL), sleeps (RADSTALL) ~500 ms; on a good
     * return stores VMEAS, LRVTIME/LRXCDU for downlink and sets VELDABIT; on bad return waits 4 seconds.
     */
    public void lrvjob() {
        if (waitlist != null) waitlist.waitlist(sgn("ND1"), this::rdgims);     // 170MS
        int vsel = sgn("VSELECT");
        int idx = (vsel != 0 ? vsel : 2) * 2;                                  // 2 x VSELECT is the VBEAM index
        lrvel(idx);
        boolean good = radstall();
        if (!good) {
            vbad();
            return;
        }
        if (sgn("STILBADV") > 0) {                                             // JUST GOOD -- MUST WAIT 4 SECONDS
            s("STILBADV", g("STILBADV") - 1);
            endlrv();
            return;
        }
        VecOps.setDpRaw(agc, "VMEAS", VecOps.getDpRaw(agc, "SAMPLSUM"));
        VecOps.setDpRaw(agc, "LRVTIMDL", VecOps.getDpRaw(agc, "LRVTIME"));     // FOR DOWNLINK
        s("LRXCDUDL", g("LRXCDU"));
        s("LRYCDUDL", g("LRYCDU"));
        s("LRZCDUDL", g("LRZCDU"));
        setBits("FLGWRD11", "VELDABIT");                                       // VELOCITY MEASUREMENT MADE
        endlrv();
    }

    /** ENDLRV: update VSELECT (reset to 2 if zero) and end job. */
    public void endlrv() {
        int vsel = sgn("VSELECT");
        s("VSELECT", vsel != 0 ? vsel - 1 : 2);
        if (exec != null) exec.endOfJob();
    }

    /** VBAD: STILBADV = 2 (wait 4 seconds). VSTILBAD: store STILBADV. */
    public void vbad() {
        s("STILBADV", 2);
        endlrv();
    }

    /** Hooks for the radar routines (RADAR_LEAD-IN / LRVEL / LRALT / RADSTALL). */
    protected void lrvel(int index) { }
    protected void lralt() { }
    /** RADSTALL: sleeps the job; returns true for a good end, false for a bad end. */
    protected boolean radstall() { return true; }

    /**
     * LRHJOB: begun by LRHTASK below 25,000 ft. Starts the LR altitude measurement (LRALT), sleeps about 95 ms;
     * on a good return stores HMEAS (1.079 ft/bit), time MKTIME and gimbal angles AIG/AMG/AOG for downlink and sets
     * RNGEDBIT; otherwise handles range-scale change or waits 4 seconds.
     */
    public void lrhjob() {
        lralt();
        boolean good = radstall();
        if (!good) {
            hbad();
            return;
        }
        if (sgn("STILBADH") > 0) {
            s("STILBADH", g("STILBADH") - 1);
            if (exec != null) exec.endOfJob();
            return;
        }
        VecOps.setDpRaw(agc, "HMEAS", VecOps.getDpRaw(agc, "SAMPLSUM"));      // LRH DATA 1.079 FT/BIT
        VecOps.setDpRaw(agc, "MKTIME", VecOps.getDpRaw(agc, "PIPTIME1"));      // FOR DOWNLINK
        s("AIG", g("CDUTEMPY"));
        s("AIG+1", g("CDUTEMPZ"));                                             // CDUY,Z = AIG,AMG
        s("AOG", g("CDUTEMPX"));                                               // CDUX = AOG
        setBits("FLGWRD11", "RNGEDBIT");                                       // RANGE MEASUREMENT MADE
        if (exec != null) exec.endOfJob();
    }

    /** HBAD: if bad return was due to a range scale change, reset RNGSCFLG and ignore; else wait 4 seconds. */
    public void hbad() {
        if (test("FLAGWRD5", "RNGSCBIT")) {
            downflag("FLAGWRD5", "RNGSCFLG");
        } else {
            s("STILBADH", 2);
        }
        if (exec != null) exec.endOfJob();
    }

    /**
     * RDGIMS: task set by LRVJOB to sample TIME2/TIME1 (LRVTIME), CDUX/Y (LRXCDU/LRYCDU), CDUZ (LRZCDU)
     * and PIPAX/Y/Z (PIPTEM) about the midpoint of the LR velocity read.
     */
    public void rdgims() {
        s("LRVTIME", g("TIME2"));
        s("LRVTIME+1", g("TIME1"));
        s("LRXCDU", g("CDUX"));
        s("LRYCDU", g("CDUY"));
        s("LRZCDU", g("CDUZ"));
        s("PIPTEM", g("PIPAX"));
        s("PIPTEM+1", g("PIPAY"));
        s("PIPTEM+2", g("PIPAZ"));
        // TC TASKOVER
    }

    /**
     * HIGATJOB: begun ~6 s before high gate. Starts the LR reposition to POS2 (LRPOS2), sleeps; on a good
     * end lowers priority (PRIO23), sets up the POS2 beam transformations (SETPOS2), clears NOLRREAD.
     * On a bad end flashes alarm 523 (PRIOLARM); terminate, proceed (retry) or V32E (terminate R12).
     */
    public void higatjob() {
        lrpos2();
        if (radstall()) {
            posgood();
        } else {
            posalarm();
        }
    }

    /** POSGOOD. */
    public void posgood() {
        if (exec != null) exec.prioChng(PRIO23);
        setpos2();
        downflag("FLGWRD11", "NOLRREAD");                                      // ENABLE LR READING
        if (exec != null) exec.endOfJob();
    }

    /** Hook: LRPOS2 (starts repositioning). */
    protected void lrpos2() { }

    /** POSALARM: alarm 523 with flashing display; terminate (GOTOPOOH), proceed (retry), or end job. */
    public void posalarm() {
        int reply = prioralarm(OCT523);                  // 0 = terminate, 1 = proceed, 2 = V32 recycle/end job
        while (true) {
            if (reply == 0) {
                if (gotoPooh != null) gotoPooh.run();
                return;
            }
            if (reply == 2 || (g("CHAN33") & 0100) == 0) {
                if (reply == 2) {
                    if (exec != null) exec.endOfJob();
                    return;
                }
                posgood();                               // POS2 ACHIEVED
                return;
            }
            reply = prioralarm(OCT523);                  // STILL DIDN'T MAKE IT: REALARM
        }
    }

    /** Hook: PRIOLARM (priority-display alarm): returns 0 terminate, 1 proceed, 2 end job. */
    protected int prioralarm(int code) { return 1; }

    /**
     * SETPOS1: BANKCALLed initialisation: STILBADH/V = 2, L,M,R,S counters = 0, VSELECT = 0, then SETPOS
     * with index 0 (LRALPHA/LRBETA in position 1).
     */
    public void setpos1() {
        s("STILBADH", 2);
        s("STILBADV", 2);
        s("LRLCTR", 0);
        s("LRMCTR", 0);
        s("LRRCTR", 0);
        s("LRSCTR", 0);
        s("VSELECT", 0);
        setpos(0);
    }

    /** SETPOS2: index for POS2. */
    public void setpos2() { setpos(2); }

    /**
     * SETPOS: with index (0 or 2) load LRALPHA/LRBETA (rotation about X and Y), convert UNITY, UNITX and
     * HBEAMANT from the antenna frame to NB (TRG*SMNB, *SMNB*) giving VYBEAMNB, VXBEAMNB, VZBEAMNB (= X x Y, scaled
     * by 2) and HBEAMNB.
     */
    public void setpos(int index) {
        s("CDUSPOT+4", g(nameOff("LRALPHA", index)));    // ROTATION ABOUT X
        s("CDUSPOT", g(nameOff("LRALPHA", index + 1)));  // LRBETA : ROTATION ABOUT Y
        s("CDUSPOT+2", 0);                               // ZERO ROTATION ABOUT Z
        double[] ybeam = smnb(new double[] {0, 1, 0});
        setVec("VYBEAMNB", ybeam);
        double[] xbeam = smnb(new double[] {1, 0, 0});
        setVec("VXBEAMNB", xbeam);
        setVec("VZBEAMNB", VecOps.scale(VecOps.cross(xbeam, ybeam), 2));    // Z = X * Y
        setVec("HBEAMNB", smnb(vec("HBEAMANT")));        // CONVERT TO NB
    }

    /** *SMNB*: SM to NB transform using CDUSPOT-derived trig (hook for the interpreter routine). */
    protected double[] smnb(double[] v) { return v; }
}
