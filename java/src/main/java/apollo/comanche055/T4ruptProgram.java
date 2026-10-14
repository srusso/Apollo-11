package apollo.comanche055;

import apollo.agc.Agc;

/**
 * T4RUPT_PROGRAM.agc - the TIME4 interrupt: DSKY relay output (DSPOUT/CDRVE), PROCEED key monitor,
 * IMU monitoring (IMUMON, TNONTEST, C33TEST, GLOCKMON, TLIM, ITURNON, IMUCAGE, IMUOP, PIPFAIL,
 * DNTMFAST/UPTMFAST, SETISSW, CAGESUB), optics monitoring (OPTMON, ZOPTICS) and optics CDU driving
 * (OPTDRIVE).
 *
 * Interrupt-style control flow: every handler path ends in {@link #resume()} (RESUME) or
 * {@link #taskover()} (TASKOVER for waitlist tasks). Channels are the erasable symbols OUT0,
 * CHAN11, CHAN12, CHAN14, CHAN30, CHAN32, CHAN33, DSALMOUT, accessed through {@link Agc}.
 */
public final class T4ruptProgram {
    // ---- bit masks ----
    private static final int BIT1 = 1, BIT2 = 2, BIT3 = 4, BIT4 = 010, BIT5 = 020, BIT6 = 040, BIT7 = 0100,
        BIT8 = 0200, BIT9 = 0400, BIT10 = 01000, BIT11 = 02000, BIT12 = 04000, BIT13 = 010000,
        BIT14 = 020000, BIT15 = 040000, LOW11 = 03777, HI5 = 076000, POSMAX = 037777;
    // ---- constants ----
    /** RELTAB: packed table, relay word code in upper 4 bits, relay code in lower 5 bits. */
    public static final int[] RELTAB = {
        004025, 010003, 014031, 020033, 024017, 030036, 034034, 040023, 044035, 050037, 054000};
    /** RELTAB11: relay word 12 (condition lamps). */
    public static final int RELTAB11 = 060000;
    /** TABLNTH: length of DSPTAB (decimal 10). */
    public static final int TABLNTH = 012;
    /** 20MRUPT: TIME4 value for 20 ms (DEC 16382). */
    public static final int MS20_RUPT = 037776;
    /** 120MRUPT: TIME4 value for 120 ms (DEC 16372). */
    public static final int MS120_RUPT = 16372;
    public static final int OCT37737 = 037737;
    /** 11,14,9 = OCT 22400. */
    public static final int B11_14_9 = 022400;
    public static final int OC40010 = 040010, OCT54 = 054, OCT75 = 075, OCT272 = 0272, BITS7AND8 = 0300,
        OCT1720 = 01720, OCT740 = 0740, OCT15000 = 015000, OCT77000 = 077000, MINUS_BIT10 = Agc.fromSigned(-01000),
        BIT15_PLUS_6 = BIT15 + BIT6, OCT15 = 015, OCT13 = 013, OCT40 = 040, OCTHIRTY = 030,
        R30_MASK = 076400 /* 30RDMSK: bit 10 not sampled */, R33_MASK = 020000 /* 33RDMSK = PRIO16... */;
    /** 90SECS (DEC 9000 centiseconds), 4SECS, 200MS etc. */
    public static final int SECS90 = 9000, SECS4 = 400, MS200 = 024, MS120 = 014;
    /** -70DEGS, -15DEGS: scaled in half-revolutions. */
    public static final double MINUS_70DEGS = -.38888, MINUS_15DEGS = -.08333;
    /** 20DEGS: DEC 7199. */
    public static final int DEGS20 = 7199;
    /** Optics pulse limits. */
    public static final int MAXPLS = -165, MAXPLS1 = -164;
    private static final int NEG1_HALF = Agc.fromSigned(-(0x2000));   // NEG1/2

    private final Agc agc;
    private final AlarmAndAbort alarms;

    public T4ruptProgram(Agc agc) {
        this.agc = agc;
        this.alarms = new AlarmAndAbort(agc);
    }

    // ---- helpers ----
    private int g(String s) { return agc.get(s); }
    private void s(String n, int v) { agc.set(n, v); }
    private void wor(String chan, int mask) { s(chan, g(chan) | mask); }
    private void wand(String chan, int mask) { s(chan, g(chan) & mask); }
    private void clear(String chan, int bits) { s(chan, g(chan) & ~bits); }
    private boolean bit(String sym, int mask) { return (g(sym) & mask) != 0; }
    private static boolean neg(int w) { return (w & Agc.SIGN_BIT) != 0; }
    private void alarm(int code) { alarms.alarm(code); }
    /** RESUME: restore registers and return to interrupted program. */
    private void resume() { s("RESUME_PENDING", 1); }
    /** TASKOVER: end waitlist task. */
    private void taskover() { s("TASKOVER_PENDING", 1); }
    private void waitlist(int delay, String task) { s("WAITLIST_DELAY", delay); s("WAITLIST_TASK_" + task, 1); }
    private void vardelay(int delay, String task) { s("VARDELAY", delay); s("VARDELAY_TASK_" + task, 1); }

    // ======================= DSKY display output =======================

    /** T4RUPT: interrupt entry. */
    public void t4rupt() {
        s("BANKRUPT", g("A"));
        s("QRUPT", g("Q"));
        int sw = g("DSRUPTSW");
        int sv = Agc.toSigned(sw);
        if (sv < 0) { quikdsp(); return; }                 // CCS: negative
        if (sv > 0) {                                      // NORMT4 +1: count down
            s("RUPTREG1", sv - 1);
            s("DSRUPTSW", sv - 1);
        } else {
            normt4();
            return;
        }
        cdrve();
    }

    /** NORMT4: set RUPTREG1 and DSRUPTSW to 7, then CDRVE. */
    public void normt4() {
        s("RUPTREG1", 7);
        s("DSRUPTSW", 7);
        cdrve();
    }

    /** CDRVE: condition-lamp relay word output (DSPTAB+11D), otherwise normal DSPOUT. */
    public void cdrve() {
        int d = g("DSPTAB+11");
        if (!neg(d)) { dspout(); return; }
        d &= LOW11;
        s("DSPTAB+11", d);
        s("OUT0", Agc.add(d, RELTAB11));
        hang20();
    }

    /** DSPOUT: put out one DSKY relay word if DSKY flag on and display requests exist. */
    public void dspout() {
        if (!neg(g("FLAGWRD5"))) { nodspout(); return; }    // DON'T DISPLAY UNLESS DSKY FLAG ON
        int nout = g("NOUT");
        if (nout != 0 && nout != Agc.WORD_MASK && !neg(nout)) {
            if (dspoutsb(nout)) { hang20(); return; }
        }
        nodspout();                                        // NO DISPLAY REQUESTS
    }

    /**
     * DSPOUTSB: scan DSPTAB (from DSPCNT down, wrapping once) for an entry marked negative (changed),
     * write it to channel 10. Returns true if something was displayed (the 'skip' return).
     */
    public boolean dspoutsb(int nout) {
        s("NOUT", nout);
        boolean firstPass = true;                           // DSRUPTEM = -0 on first pass
        int cnt = g("DSPCNT");
        while (true) {
            for (int i = cnt; i >= 0; i--) {
                int e = g("DSPTAB+" + i);
                if (neg(e)) { cnt = i; dsplay(i, e); return true; }
            }
            if (!firstPass) { s("NOUT", 0); s("DSPCNT", 0); return false; }   // +0 INTO NOUT
            firstPass = false;
            s("DSRUPTEM", 0);
            cnt = TABLNTH;                                   // PASS AGAIN FROM THE TOP
        }
    }

    /** DSPLAY: replace entry positively and write channel 10. */
    private void dsplay(int i, int entry) {
        s("DSPCNT", i);
        int e = Agc.add(entry, 1) & POSMAX;
        s("DSPTAB+" + i, e);
        int low = e & LOW11;                               // REMOVE BITS 12 TO 15
        s("DSRUPTEM", low);
        int relay = RELTAB[Math.min(i, RELTAB.length - 1)] & HI5;   // BITS 12 TO 15 OF RELTAB ENTRY
        s("OUT0", Agc.add(low, relay));                     // WRITE CHANNEL 10
    }

    /** HANG20: schedule the 20 ms hold, then fall into PROCEEDE. */
    public void hang20() {
        s("DSRUPTSW", Agc.add(g("DSRUPTSW"), Agc.fromSigned(-B11_14_9)));   // CS 11,14,9; ADS DSRUPTSW
        s("TIME4", MS20_RUPT);                              // SETTIME4
        proceede();
    }

    /** NODSPOUT: turn off relays and set up the next CDRVE 120 ms later. */
    public void nodspout() {
        s("OUT0", 0);
        s("TIME4", MS120_RUPT);                             // SET FOR NEXT CDRVE
        proceede();
    }

    /**
     * PROCEEDE: monitor the PROCEED pushbutton (CHAN32 bit 14), comparing with IMODES33.
     * ON-transition is processed through PROCKEY at priority CHRPRIO.
     */
    public void proceede() {
        int diff = (g("IMODES33") ^ g("CHAN32")) & BIT14;
        if (diff != 0) {
            int im = g("IMODES33") ^ g("CHAN32");           // LXCH IMODES33 / RXOR LCHAN: update bit
            // IMODES33 = IMODES33 xor (changed bits)
            s("IMODES33", g("IMODES33") ^ (diff));
            if ((g("IMODES33") & BIT14) == 0) {            // WAS ON -- NOW OFF
                t4jump();
                return;
            }
            s("NOVAC_PRIORITY", g("CHRPRIO"));              // WAS OFF -- NOW ON: PROCKEY job
            s("NOVAC_JOB_PROCKEY", 1);
        }
        t4jump();
    }

    /** T4JUMP: once per 0.96 s activity selected by RUPTREG1. */
    public void t4jump() {
        switch (g("RUPTREG1") & 7) {
            case 0, 4 -> opttest();
            case 1, 5 -> optmon();
            case 2, 6 -> imumon();
            default -> resume();
        }
    }

    /** OPTTEST: IBNKCALL OPTDRIVE. */
    public void opttest() { optdrive(); }

    /** QUIKDSP: fast display mode (20 ms). */
    public void quikdsp() {
        if (!bit("DSRUPTSW", BIT14)) { quikoff(); return; }     // WROTE LAST TIME, NOW TURN OFF RELAYS
        int nout = g("NOUT");
        if (nout != 0 && !neg(nout) && dspoutsb(nout)) {
            quikrupt(-BIT14);                                   // GOOD RETURN: CS BIT14
        } else {
            nodspy();                                           // NOUT=0 OR BAD RETURN
        }
    }

    /** QUIKRUPT: ADS DSRUPTSW; reload TIME4 with 20 ms; add BIT9. */
    private void quikrupt(int delta) {
        s("DSRUPTSW", Agc.add(g("DSRUPTSW"), Agc.fromSigned(delta)));
        s("TIME4", MS20_RUPT);
        s("DSRUPTSW", Agc.add(g("DSRUPTSW"), BIT9));
        resume();
    }

    /** NODSPY: relays off, SYNCT4. */
    private void nodspy() {
        s("OUT0", 0);
        synct4();
    }

    /** SYNCT4: add 20 ms to TIME4 and BIT9 to DSRUPTSW; loop if it overflows past zero. */
    public void synct4() {
        s("TIME4", Agc.add(g("TIME4"), MS20_RUPT));
        s("DSRUPTSW", Agc.add(g("DSRUPTSW"), BIT9));
        int sw = Agc.toSigned(g("DSRUPTSW"));
        if (sw > 0) { resume(); return; }
        // +0 : TC SYNCT4 again would loop; the original returns after the second test
        resume();
    }

    /** QUIKOFF: turn off relays and reset DSRUPTSW to send display next pass. */
    public void quikoff() {
        s("OUT0", 0);
        quikrupt(BIT14);
    }

    // ======================= IMU monitoring =======================

    /** IMUMON (every 480 ms): detect changes of IMU status bits in channel 30 and dispatch. */
    public void imumon() {
        int changed = (g("IMODES30") ^ g("CHAN30")) & R30_MASK;
        if (changed == 0) { tnontest(); return; }            // NO CHANGE IN STATUS
        s("IMODES30", g("IMODES30") ^ changed);              // UPDATE IMODES30
        // scan bits 15 down to 9 (bit 15 first = TLIM)
        for (int bitNo = 15; bitNo >= 9; bitNo--) {
            int mask = 1 << (bitNo - 1);
            if ((changed & mask) == 0) continue;
            int now = g("IMODES30") & mask;                  // NEW VALUE OF THIS BIT
            switch (bitNo) {
                case 15 -> tlim(now);
                case 14 -> iturnon(now);
                case 13 -> setissw();                        // IMUFAIL
                case 12 -> setissw();                        // ICDUFAIL
                case 11 -> imucage(now);
                case 9 -> imuop(now);
                default -> { }
            }
        }
        tnontest();                                          // exit via TNONTEST
    }

    /** TNONTEST: honour ISS initialization requests, 2 samples after the first request bit. */
    public void tnontest() {
        if (!bit("IMODES30", BIT7)) { c33test(); return; }   // NO -- EXAMINE CHANNEL 33
        if (bit("IMODES30", BIT8)) { proctnon(); return; }   // REACT AFTER A SECOND SAMPLE
        wor("IMODES30", BIT8);                               // FIRST SAMPLE: SET BIT TO REACT NEXT TIME
        c33test();
    }

    /** PROCTNON: process IMU turn-on requests. */
    public void proctnon() {
        clear("IMODES30", BITS7AND8);
        if (!bit("IMODES30", BIT14)) { oponly(); return; }   // OPERATE ON ONLY
        if (!bit("IMODES30", BIT9)) alarm(0213);             // ISS TURN-ON WITHOUT OPERATE
        cagesub();
        waitlist(SECS90, "ENDTNON");
        c33test();
    }

    /** RETNON: wait another 90 seconds. */
    public void retnon() { vardelay(SECS90, "ENDTNON"); }

    /** ENDTNON: end of 90-second ISS turn-on delay. */
    public void endtnon() {
        boolean failBit = bit("IMODES30", BIT2);
        clear("IMODES30", BIT2);                             // RESET TURN-ON REQUEST FAIL BIT
        if (!failBit) { endtnon2(); return; }                // SEND ISS DELAY COMPLETE
        if (!bit("IMODES30", BIT14)) {                       // turn-on request not present now
            if (!bit("STATE", IMUSEFLG_MASK)) { taskover(); return; }
            s("IMUBAD", 1);                                  // UNSUCCESSFUL TURN-ON (POSTJUMP IMUBAD)
            return;
        }
        retnon();                                            // IF PRESENT, RE-ENTER 90 SEC DELAY
    }

    /** ENDTNON2: send ISS delay complete, turn off NO ATT lamp, zero ICDUs. */
    public void endtnon2() {
        wor("CHAN12", BIT15);
        s("NOATTOFF", 1);
        unz2();
    }

    /** UNZ2: zero ICDUs, remove ZERO and COARSE, wait 10 secs then ISSUP. */
    public void unz2() {
        zeroicdu();
        clear("CHAN12", BIT4 | BIT5);
        vardelay(BIT11, "ISSUP");                            // WAIT 10 SECS FOR CTRS TO FIND GIMBALS
    }

    /** ISSUP: remove caging, enable DAP, set ISS warning, clear delay complete; PFAILOK in 4 secs. */
    public void issup() {
        clear("IMODES30", OCT54);
        clear("IMODES33", BIT6);                             // ENABLE DAP
        setissw();
        clear("CHAN12", BIT15);
        waitlist(SECS4, "PFAILOK");
        taskover();
    }

    private static final int IMUSEFLG_MASK = 0200;           // IMUSEFLG bit in STATE

    /** OPONLY: operate on only; zero ICDUs unless in coarse align or IMU in use. */
    public void oponly() {
        if (bit("CHAN12", BIT4)) { c33test(); return; }      // COARSE ALIGN: MIGHT BE GIMBAL LOCK
        if (bit("STATE", IMUSEFLG_MASK)) { c33test(); return; }
        cagesub2();
        isszero();
    }

    /** ISSZERO: turn off NO ATT, ICDU zero, UNZ2 in 300 ms. */
    public void isszero() {
        s("NOATTOFF", 1);
        wor("CHAN12", BIT5);                                 // ISS CDU ZERO
        zeroicdu();
        waitlist(BIT6, "UNZ2");
        c33test();
    }

    /** ZEROICDU: zero the ICDU counters (CDUX, CDUY, CDUZ). */
    private void zeroicdu() { s("CDUX", 0); s("CDUY", 0); s("CDUZ", 0); }

    /** C33TEST: monitor CHAN33 flip-flop inbits (13 PIPA fail, 12 downlink, 11 uplink too fast). */
    public void c33test() {
        int mask = g("IMODES33") & R33_MASK_ALL;
        int chan33 = g("CHAN33");
        int changed = (mask ^ (chan33 & R33_MASK_ALL));      // WAND CHAN33 resets flip-flops
        if (changed == 0) { glockmon(); return; }
        s("IMODES33", g("IMODES33") ^ changed);
        for (int bitNo = 13; bitNo >= 11; bitNo--) {
            int m = 1 << (bitNo - 1);
            if ((changed & m) == 0) continue;
            int now = g("IMODES33") & m;
            switch (bitNo) {
                case 13 -> pipfail(now);
                case 12 -> dntmfast(now);
                case 11 -> uptmfast(now);
                default -> { }
            }
        }
        glockmon();
    }

    private static final int R33_MASK_ALL = BIT13 | BIT12 | BIT11;

    /** GLOCKMON: gimbal lock monitor from CDUZ (middle gimbal angle). */
    public void glockmon() {
        double mga = Math.abs(Agc.toSigned(g("CDUZ")) / 16384.0);   // half revolutions
        int desired = 0;                                            // desired GIMBAL LOCK lamp bit
        if (mga > 0.38888) {                                         // > 70 degrees
            if (mga > 0.38888 + 0.08333) {                           // > 85 degrees
                if (!bit("CHAN12", BIT4)) {                          // not in coarse align
                    s("SETCOARS", 1);                                // GO INTO COARSE ALIGN
                    waitlist(6, "CA+ECE");                           // ENABLE ISS ERROR COUNTERS IN 60 MS
                }
            }
            desired = BIT6;                                          // NOGIMRUN: turn on lamp
        }
        setglock(desired);
    }

    /** SETGLOCK: compare lamp state in DSPTAB+11D with desired; invert if different. */
    private void setglock(int desired) {
        int cur = g("DSPTAB+11");
        if (((cur ^ desired) & BIT6) == 0) { resume(); return; }    // GLOCKOK
        if ((cur & BIT6) == 0) {                                     // lamp off, want on
            if (bit("IMODES30", BIT6)) { resume(); return; }         // IMU BEING CAGED: don't turn on
        } else if (!lamptest()) { resume(); return; }                // turning off: not during lamp test
        glinvert();
    }

    /** GLINVERT: invert the gimbal lock lamp bit and flag change with BIT15. */
    private void glinvert() {
        int d = g("DSPTAB+11");
        int notBit6 = (~d) & BIT6;
        s("DSPTAB+11", (d & OCT37737) | notBit6 | BIT15);
        resume();
    }

    /** LAMPTEST: returns true (the skip return) if lamp test is NOT in progress (IMODES33 bit 1 = 0). */
    private boolean lamptest() { return !bit("IMODES33", BIT1); }

    /** TLIM: IMU temperature lamp (CHAN11 bit 4) follows CHAN30 bit 15. */
    public void tlim(int now) {
        if (!bit("IMODES30", BIT15)) {                               // TEMP OUT OF LIMITS (0 = out)
            wor("DSALMOUT", BIT4);                                   // TURN ON LAMP
        } else if (lamptest()) {
            clear("DSALMOUT", BIT4);                                 // TURN OFF LAMP
        }
    }

    /** ITURNON: ISS turn-on request changed (CHAN30 bit 14). */
    public void iturnon(int now) {
        if (bit("IMODES30", BIT2)) return;                           // DELAY REQUEST WENT OFF PREMATURELY
        if (!bit("IMODES30", BIT14)) {                               // JUST ON
            iturnon2();
            return;
        }
        if (bit("CHAN12", BIT15)) return;                            // DELAY DISCRETE SENT: ACTION COMPLETE
        wor("IMODES30", BIT2);                                       // REQUEST NOT PRESENT FOR FULL DURATION
        alarm(0207);
    }

    /** ITURNON2: set bit 7 to indicate wait of one sample. */
    public void iturnon2() { wor("IMODES30", BIT7); }

    /** IMUCAGE: IMU cage button; a change to 0 cages the ISS. */
    public void imucage(int now) {
        if (now != 0) { /* going off: no action */ isszeroFromCage(); return; }
        clear("CHAN14", OCT77000);                                   // TERMINATE ICDU, OPTICS, GYRO PULSE TRAINS
        clear("CHAN12", OCT272);                                     // KNOCK DOWN TVC, IMU ERR CNTR, ZERO, COARSE
        clear("DSALMOUT", BIT13);                                    // TURN OFF ENGINE
        cagesub1();
        s("RNDREFDR", 1);                                            // KNOCK DOWN TRACK, REFSMMAT, DRIFT FLAGS
        s("CDUXCMD", 0); s("CDUYCMD", 0); s("CDUZCMD", 0); s("GYROCMD", 0);
        clear("CHAN14", OCT740);                                     // DE-SELECT THE GYROS
    }

    /** ISS cage bit released: ISSZERO as in the original CCS (not-caged branch). */
    private void isszeroFromCage() { isszero(); }

    /** IMUOP: ISS operate discrete changed (CHAN30 bit 9). */
    public void imuop(int now) {
        if (now == 0) { imuop2(); return; }                          // OPERATE JUST ON, WAIT 1 SAMPLE
        wor("IMODES33", BIT6);                                       // DISABLE DAP
        s("RNDREFDR", 1);
        int old = g("STATE");
        s("STATE", old & ~BITS7AND8);                                // KNOCK DOWN RENDEZVOUS, IMUUSE FLAGS
        if ((old & IMUSEFLG_MASK) != 0) alarm(0214);                 // PROGRAM USING IMU WHEN TURNED OFF
    }

    /** IMUOP2. */
    public void imuop2() {
        if (bit("IMODES30", BIT2)) return;                           // FAILED TURN-ON SEQ IN PROGRESS
        iturnon2();
    }

    /** PIPFAIL: CHAN33 bit 13 (PIPA fail) -> IMODES30 bit 10; may cause alarm 00212. */
    public void pipfail(int now) {
        if (now != 0) wor("IMODES30", BIT10); else clear("IMODES30", BIT10);
        setissw();
        if (!bit("IMODES30", BIT1)) return;                          // PIP FAIL LIGHTS ISS WARNING
        if ((g("IMODES30") & OCT1720) != 0) return;
        alarm(0212);
    }

    /** DNTMFAST: downlink too fast (alarm 01105 when bit goes to 0). */
    public void dntmfast(int now) { if (now == 0) alarm(01105); }

    /** UPTMFAST: uplink too fast (alarm 01106 when bit goes to 0). */
    public void uptmfast(int now) { if (now == 0) alarm(01106); }

    /**
     * SETISSW: ISS warning lamp (CHAN11 bit 1) from fail bits 13,12,10 of IMODES30 masked by the
     * inhibit bits 4,3,1. Lamp left on during lamp test. (IMUFAIL and ICDUFAIL are aliases.)
     */
    public void setissw() {
        int inhibit = (g("IMODES30") & OCT15) * BIT10;               // MP BIT10: shift inhibit bits to fail positions
        int fail = (~(g("IMODES30") | inhibit)) & OCT15000;          // 0 INDICATES FAILURE
        if (fail != 0) {                                             // ISSWON
            s("VARALARM_ISSW", fail);
            wor("DSALMOUT", BIT1);
        } else if (lamptest()) {
            clear("DSALMOUT", BIT1);                                 // ISSWOFF
        }
    }

    /** CAGESUB: outbits and internal flags for turn-on or cage. */
    public void cagesub() {
        clear("CHAN12", BIT15_PLUS_6);
        wor("CHAN12", BIT4 | BIT5);                                  // SEND ZERO AND COARSE
        cagesub1();
    }

    /** CAGESUB1: turn on NO ATT lamp. */
    public void cagesub1() {
        s("DSPTAB+11", g("DSPTAB+11") | OC40010);
        cagesub2();
    }

    /** CAGESUB2: set flags indicating caging or turn-on, inhibit ISS warning info, disable DAP. */
    public void cagesub2() {
        s("IMODES30", g("IMODES30") | OCT75);
        wor("IMODES33", BIT6);
    }

    // ======================= Optics monitoring =======================

    /** OPTMON: monitor optics inbits in channels 30 and 33. */
    public void optmon() {
        int change = (g("OPTMODES") ^ g("CHAN30")) & BIT7;
        s("RUPTREG1", change);
        if (change != 0) ocduftst();                                 // OCDU FAIL BIT CHANGED
        optmon33();
    }

    /** 33OPTMON. */
    public void optmon33() {
        int optind = g("OPTIND");
        if (optind == Agc.WORD_MASK || neg(optind)) { resume(); return; }   // BYPASS IF TVC TAKEOVER
        int inchange = (g("OPTMODES") ^ g("CHAN33")) & OCTHIRTY;
        s("RUPTREG1", g("RUPTREG1") + inchange);
        s("OPTMODES", g("OPTMODES") ^ inchange);                     // UPDATE OPTMODES
        int sample = (~g("OPTMODES")) & OCTHIRTY;
        int sw;
        if (sample == 0) sw = 0;                                     // MANUAL
        else sw = (sample & BIT5) != 0 ? 1 : -1;                     // CSC : ZOPTICS
        s("SWSAMPLE", Agc.fromSigned(sw));
        processw();
    }

    /** PROCESSW: branch on previous switch setting DESOPMOD. */
    public void processw() {
        int des = Agc.toSigned(g("DESOPMOD"));
        if (des > 0) cscdes(); else if (des == 0) manudes(); else zoptdes();
    }

    private int sw() { return Agc.toSigned(g("SWSAMPLE")); }

    /** ZOPTDES: previous setting ZOPTICS. */
    public void zoptdes() {
        int sw = sw();
        if (sw > 0) { ztocsc(); return; }                            // NOW AT CSC
        if (sw == 0) { ztoman(); return; }                           // MANUAL
        if (zopfini()) { setdesmd(); return; }                       // ZOPT NOT PROCESSING - NO ACTION
        int cnt = Agc.toSigned(g("ZOPTCNT"));
        if (cnt > 0) { setcnt(cnt - 1); return; }                    // 32 SAMPLE NOT FINISHED
        setzoend();                                                  // 32 SAMPLE WAIT COMPLETED
    }

    /** ZTOMAN: ZOPT to manual. */
    public void ztoman() {
        if (zopfini()) { setdesmd(); return; }
        zopalarm();
    }

    /** ZOPALARM: switch altered while ZOPTICS processing. */
    public void zopalarm() {
        alarm(00116);
        s("WTOPTION", OCT13);
        canzopt();
        setdesmd();
    }

    /** ZTOCSC: ZOPT to CSC. */
    public void ztocsc() {
        if (zopfini()) { mantocsc3(); return; }                      // NO - CHECK RETURN TO COARS OPT
        alarm(00116);
        canzopt();
        mantocsc();
    }

    /**
     * ZOPFINI (labelled ZOPFIN1/2/3): returns true if ZOPTICS is NOT processing (the L+2 return).
     * If the end-ZOPT task is working (OPTMODES bit 1), RESUME and wait one sample.
     */
    private boolean zopfini() {
        if (bit("OPTMODES", BIT1)) { resume(); throw new ResumeSignal(); }
        return !bit("OPTMODES", BIT3);
    }

    /** Used to unwind after a RESUME taken from inside a subroutine. */
    private static final class ResumeSignal extends RuntimeException {
        ResumeSignal() { super(null, null, false, false); }
    }

    /** COARSLOK: returns true if return to coarse optics is needed (OPTMODES bit 9 set). */
    private boolean coarslok() { return !bit("OPTMODES", BIT9); }

    /** CANZOPT: cancel zero optics. */
    public void canzopt() {
        clear("OPTMODES", 06);                                       // ZERO ZOPT PROCESSING BIT - ENABLE OCDUFAIL
        clear("CHAN12", BIT1);                                       // MAKE SURE ZERO OCDU IS OFF
    }

    /** MANUDES: previous setting MANUAL. */
    public void manudes() {
        int sw = sw();
        if (sw > 0) { mantocsc(); return; }
        if (sw == 0) { mantoman(); return; }
        int wt = Agc.toSigned(g("WTOPTION"));                        // ZOPTICS
        if (wt <= 0) { optzero(); return; }                          // ZOPTICS MUST START ANEW
        initzopt();                                                  // 5 SEC RETURN GOOD - CONTINUE
        setdesmd();
    }

    /** MANTOMAN. */
    public void mantoman() {
        int wt = Agc.toSigned(g("WTOPTION"));
        if (wt > 0) s("WTOPTION", wt - 1);                           // DECREMENT RETURN OPTION TIME
        setdesmd();
    }

    /** MANTOCSC: cancel ZOPT return option, check return to coarse optics. */
    public void mantocsc() {
        s("WTOPTION", 0);
        s("ZOPTCNT", 0);
        mantocsc3();
    }

    private void mantocsc3() {
        if (coarslok()) { setdesmd(); return; }                      // NO COARS TASK - NO ACTION
        s("OPTIND", 1);                                              // SET COARS OPT WORKING
        wor("CHAN12", BIT2);                                         // ENABLE OPTICS CDU ERROR COUNTS
        setdesmd();
    }

    /** CSCDES: previous setting CSC. */
    public void cscdes() {
        int sw = sw();
        if (sw > 0) { setdesmd(); return; }                          // STILL AT CSC
        if (sw == 0) { csctoman(); return; }                         // MANUAL
        s("ZOPTCNT", OCT40);                                         // CSCTOZOP: INITIALIZE FOR ZOPT
        initzopt();
        setdesmd();
    }

    /** CSCTOMAN: kill coarse optics if working. */
    public void csctoman() {
        int optind = g("OPTIND");
        if (!neg(optind) && optind != Agc.WORD_MASK && optind != 0) { cancoars(); return; }
        if (optind == 0) { cancoars(); return; }
        setdesmd();
    }

    /** CANCOARS. */
    public void cancoars() {
        s("OPTIND", Agc.fromSigned(-1));
        clear("CHAN12", BIT2);                                       // DISABLE OCDU ERR CNTS
        wor("OPTMODES", BIT9);                                       // SET RETURN-TO-COARS BIT
        setdesmd();
    }

    /** OPTZERO: restart zero optics with 32 sample wait. */
    public void optzero() {
        initzopt();
        setcnt(OCT40);
    }

    /** SETCNT. */
    public void setcnt(int cnt) {
        s("ZOPTCNT", cnt);
        setdesmd();
    }

    /** SETDESMD: current switch indication to DESOPMOD; RESUME. */
    public void setdesmd() {
        s("DESOPMOD", g("SWSAMPLE"));
        resume();
    }

    /** SETZOEND: send zero optics CDU for 200 ms then ENDZOPT. */
    public void setzoend() {
        wor("CHAN12", BIT1);
        waitlist(MS200, "ENDZOPT");
        wor("OPTMODES", BIT1);                                       // SHOW ZOPTICS TASK WORKING
        setdesmd();
    }

    /** ENDZOPT: zero OCDU counters, wait 200 ms, mark zeroed since last fresh start. */
    public void endzopt() {
        zeropcdu();
        clear("CHAN12", BIT1);
        vardelay(MS200, "ENDZOPT2");
    }

    /** Continuation of ENDZOPT after VARDELAY. */
    public void endzopt2() {
        wor("OPTMODES", BIT10);                                      // OPTICS ZEROED SINCE LAST FSTART OR RESTART
        clear("OPTMODES", 07);                                       // ENABLE OCDUFAIL - SHOW OPTICS COMPLETE
        ocduftst();
        taskover();
    }

    /** ZEROPCDU: zero CDUS, set CDUT to -20 degrees, init shaft zone. */
    public void zeropcdu() {
        s("CDUS", 0);
        s("ZONE", 0);
        s("CDUT", Agc.fromSigned(-DEGS20));
    }

    /** INITZOPT: inhibit OCDUFAIL and show optics processing. */
    public void initzopt() {
        s("WTOPTION", 0);
        s("OPTMODES", g("OPTMODES") | 06);
    }

    /** OCDUFTST: set the OCDU fail lamp (DSPTAB+11D bit 8) from CHAN30 bit 7. */
    public void ocduftst() {
        if (!bit("CHAN30", BIT7)) {                                  // OCDUFAIL LIGHT OFF
            if (bit("IMODES33", BIT1)) return;                       // LAMP TEST IN PROGRESS
            setoff(g("DSPTAB+11") & BIT8, 0);
            return;
        }
        if (bit("OPTMODES", BIT2)) return;                           // OCDUFAIL INHIBITED
        setoff(Agc.add(BIT8, g("DSPTAB+11")) & BIT8, BIT8);          // OPFAILON
    }

    private void setoff(int diff, int target) {
        if (diff == 0) return;                                       // NO CHANGE
        int d = g("DSPTAB+11");
        s("DSPTAB+11", ((d ^ BIT8) & POSMAX) + BIT15);               // SHOW ACTION WANTED
    }

    // ======================= Optics CDU driving =======================

    /** OPTDRIVE: shaft stop monitor zone update, then drive optics CDUs toward DESOPTS/DESOPTT. */
    public void optdrive() {
        int cdus = Agc.toSigned(g("CDUS"));
        int abs = Math.abs(cdus);
        if (abs <= 0x2000) {                                          // LESS THAN 45 DEG? (ZONE 0 below 90 deg)
            s("ZONE", 0);
        } else if (g("ZONE") == 0) {                                  // IF ZONE ZERO, CHANGE TO SIGN OF CDUS
            s("ZONE", cdus < 0 ? BIT15 : 1);
        }
        contdrve();
    }

    /** CONTDRVE. */
    public void contdrve() {
        int optind = Agc.toSigned(g("OPTIND"));
        if (optind < 0 || g("OPTIND") == Agc.WORD_MASK) { resume(); return; }   // NO OPT
        if (Agc.toSigned(g("SWSAMPLE")) <= 0) { resume(); return; }              // MANUAL (+0) OR ZERO (-1)
        if (!bit("OPTMODES", BIT10)) alarm(00120);                    // OPTICS NOT ZEROED
        if (!bit("CHAN12", BIT2)) { setbit(); return; }               // CNTS NOT ENABLED
        opt2(1);
    }

    /** OPT2 loop: compute shaft (OPTIND=1) and trunnion (OPTIND=0) commands. */
    private void opt2(int optind) {
        int[] cmd = new int[2];
        for (int ind = 1; ind >= 0; ind--) {
            s("OPTIND", ind);
            if (ind == 0) {                                           // TRUNCMD: limit to 45 degrees
                int diff = Agc.toSigned(g("DESOPTT")) - Agc.toSigned(g("CDUT"));
                if (Math.abs(diff) > 0x2000) {
                    cmd[0] = diff > 0 ? POSMAX : Agc.fromSigned(-POSMAX);
                    s("COMMANDO", cmd[0]);
                    continue;
                }
            }
            int des = ind == 0 ? g("DESOPTT") : g("DESOPTS");
            int cdu = ind == 0 ? g("CDUT") : g("CDUS");
            int diff = Agc.toSigned(des) - Agc.toSigned(cdu);
            int c = (int) (((long) diff * BIT13) >> 14);              // MP BIT13: scale
            c = (c * 2) & Agc.WORD_MASK;                               // DOUBLE
            s("COMMANDO" + (ind == 0 ? "" : "+1"), c);
        }
        shaftStopAvoidance();
    }

    /** SHAFT STOP AVOIDANCE: reverse the shaft command if heading into the stop. */
    private void shaftStopAvoidance() {
        int cdus = Agc.toSigned(g("CDUS"));
        if (Math.abs(cdus) > 0x4000 / 2) {                            // |CDUS| > 90 degrees
            int zone = g("ZONE");
            if (zone != 0 && ((zone ^ g("COMMANDO+1")) & BIT15) == 0) {
                if (Math.abs(Agc.toSigned(g("DESOPTS"))) <= 0x4000 / 2) {  // DESOPTS IN FIRST OR FOURTH QUAD
                    s("COMMANDO+1", (~g("COMMANDO+1")) & Agc.WORD_MASK);   // REVERSE REGULAR COMMAND
                }
            }
        }
        cmdsetup();
    }

    /** CMDSETUP/NEXTOPT: limit pulse counts and send. */
    public void cmdsetup() {
        int send = 0;
        for (int ind = 1; ind >= 0; ind--) {
            s("OPTIND", ind);
            int c = Agc.toSigned(g(ind == 0 ? "COMMANDO" : "COMMANDO+1"));
            int out;
            if (c == 0) {
                out = Agc.WORD_MASK;                                  // MAKE SURE ZERO COMMAND IS -ZERO
            } else {
                int mag = Math.min(Math.abs(c), -MAXPLS);              // LESS THAN MAX PULSE ELSE MAX PULSE
                out = Agc.fromSigned(c < 0 ? -mag : mag);
                send++;                                                // SET SEND INDICATOR
            }
            s(ind == 0 ? "CDUTCMD" : "CDUSCMD", out);
        }
        s("OPTIND", 0);
        if (send > 0) sendocmd(); else resume();
    }

    /** SENDOCMD: send OCDU drive commands (CHAN14 bits 11,12). */
    public void sendocmd() {
        wor("CHAN14", BIT6 * 0 + 03000);                              // 11,12 = PRIO6 field (bits 11,12)
        resume();
    }

    /** SETBIT: enable OCDU error counters; start coarse next time around. */
    public void setbit() {
        wor("CHAN12", BIT2);
        resume();
    }
}
