package apollo.luminary099;

import apollo.agc.Agc;

/**
 * T4RUPT_PROGRAM.agc (Luminary 1A build 099, pages 155-189).
 *
 * The TIME4 interrupt: DSKY relay output (DSPOUT, 20 ms keyboard/display activity), proceed
 * button monitoring, and the 480 ms (0.96 s cycle of 8 passes x 120 ms) activities RCSMONIT,
 * RRAUTCHK (rendezvous radar monitors), IMUMON (channel 30 / IMU status), C33TEST (channel 33),
 * GLOCKMON (gimbal lock monitor), and GPMATRIX / DAPT4S (DAP gimbal-to-body matrix, 4 times a second).
 *
 * Channels are stored in the Agc symbol table as CHAN12, CHAN30, CHAN32, CHAN33, DSALMOUT, OUT0.
 */
public final class T4ruptProgram {
    // ---- constants (fixed-fixed bank)
    /** 100MRUPT = OCT 37766 (DEC 16374). */
    public static final int RUPT_100M = 037766;
    /** 120MRUPT = DEC 16372. */
    public static final int RUPT_120M = 16372;
    /** 20MRUPT = OCT 37776 (DEC 16382). */
    public static final int RUPT_20M = 037776;
    public static final int OCT37737 = 037737;
    public static final int DSRUPT_14_11_9 = 022400; // "14,11,9"
    public static final int LOW11 = 03777;
    public static final int HI5 = 076000;
    public static final int TABLNTH = 012; // length of DSPTAB, decimal 10

    /**
     * RELTAB: packed table. Relay word code in upper 4 bits, relay code in lower 5 bits.
     * RELTAB11 (last entry, 060000) is the relay word for DSPTAB+11D.
     */
    public static final int[] RELTAB = {
        004025, 010003, 014031, 020033, 024017, 030036, 034034, 040023, 044035, 050037, 054000};
    public static final int RELTAB11 = 060000;

    public static final int BIT1 = 1, BIT2 = 2, BIT4 = 8, BIT5 = 16, BIT6 = 32, BIT7 = 64, BIT8 = 128;
    public static final int BIT9 = 0x100, BIT10 = 0x200, BIT11 = 0x400, BIT12 = 0x800, BIT13 = 0x1000;
    public static final int BIT14 = 0x2000, BIT15 = 0x4000;

    public static final int RRINIT = 00102;
    public static final int NEG_70DEGS = Agc.fromSigned((int) Math.round(-.38888 * 16384));
    public static final int NEG_15DEGS = Agc.fromSigned((int) Math.round(-.08333 * 16384));
    public static final int OC40010 = 040010;
    public static final int OCT54 = 054;
    public static final int OCT75 = 075;
    public static final int OCT272 = 0272;
    public static final int BITS7_8 = 0300;
    public static final int OCT1720 = 01720;
    public static final int OCT740 = 0740;
    public static final int OCT15000 = 015000;
    public static final int OCT77000 = 077000;
    public static final int BITS6_15 = 040040;
    public static final int NEG_BIT10 = Agc.fromSigned(-01000);
    public static final int SECS_90 = 9000;
    public static final int RDMSK_30 = 076400; // 30RDMSK (bit 10 not sampled here)
    public static final int RDMSK_33 = 0x2000 | 0x1000 | 0x800; // 33RDMSK = PRIO16 (bits 13,12,11)
    public static final int OCT05776 = 05776;
    public static final int OCT32002 = 032002;
    public static final int OCT20002 = 020002;
    public static final int OCT02100 = 02100; // P20, P22 mask bits

    private final Agc agc;
    private Waitlist waitlist;
    private Executive exec;
    private final RcsFailureMonitor rcsMonitor;
    private final SinglePrecisionSubroutines sp;

    public T4ruptProgram(Agc agc) {
        this.agc = agc;
        this.rcsMonitor = new RcsFailureMonitor(agc);
        this.sp = new SinglePrecisionSubroutines(agc);
    }

    public void setWaitlist(Waitlist w) { this.waitlist = w; }
    public void setExecutive(Executive e) { this.exec = e; }

    private int g(String s) { return agc.get(s); }
    private void set(String s, int v) { agc.set(s, v); }
    private void orInto(String s, int mask) { agc.set(s, agc.get(s) | mask); }
    private void andNot(String s, int mask) { agc.set(s, agc.get(s) & ~mask); }
    private boolean any(String s, int mask) { return (agc.get(s) & mask) != 0; }
    private void alarm(int code) { set("ALARM", code); }
    private void resume() { /* TCF RESUME */ }
    private void wl(int dt, Runnable task) {
        if (waitlist != null) waitlist.waitlist(dt, task);
    }

    // ---------------------------------------------------------------- T4RUPT entry

    /**
     * T4RUPT: saves BANKRUPT/QRUPT (implicit). DSRUPTSW counts 7 down to -0 (an 8-pass cycle);
     * a positive/zero test routes to QUIKDSP or NORMT4. Display activity is generated through DSPOUT.
     */
    public void t4rupt() {
        int dsrupt = Agc.toSigned(g("DSRUPTSW"));
        if (dsrupt > 0) { // CCS DSRUPTSW : positive -> TCF NORMT4 +1 (count down)
            normT4Plus1(dsrupt - 1);
            return;
        }
        if (g("DSRUPTSW") == 0) { // +0 (second case) -> NORMT4
            normT4();
            return;
        }
        // negative: TCF QUIKDSP
        quikDsp();
    }

    /** NORMT4: reload RUPTREG1 and DSRUPTSW with 7, then continue at the DSPOUT logic. */
    public void normT4() {
        set("RUPTREG1", 7);
        set("DSRUPTSW", 7);
        dspOut();
    }

    private void normT4Plus1(int newCount) {
        set("DSRUPTSW", Agc.fromSigned(newCount));
        dspOut();
    }

    /**
     * DSPOUT: puts out displays. If the DSKY flag (FLAGWRD5) is off (or no requests: NOUT=0)
     * NODSPOUT writes 0 to OUT0 and sets TIME4 for a 120 ms interrupt; otherwise DSPOUTSB scans
     * DSPTAB for an entry with negative (pending) sign and sends it (DSPLAY).
     */
    public void dspOut() {
        if (Agc.toSigned(g("FLAGWRD5")) <= 0) { // CCS FLAGWRD5 : DSKY flag
            noDspOut();
            return;
        }
        if (g("NOUT") == 0) {
            noDspOut();
            return;
        }
        dspOutSb();
        hang20();
    }

    /** DSPOUTSB: scan DSPTAB circularly for a negative (to be displayed) entry. Returns true if one was sent. */
    private boolean dspOutSb() {
        int nout = g("NOUT");
        set("DSRUPTEM", Agc.WORD_MASK); // -0 for first pass through DSPTAB
        int cnt = g("DSPCNT") + 1; // XCH DSPCNT ; AD NEG0 : decrement/skip +0
        set("DSPCNT", cnt);
        int pass = 1;
        int i = Agc.toSigned(g("DSPCNT"));
        while (true) {
            i = Agc.toSigned(g("DSPCNT"));
            int entry = g("DSPTAB+" + i);
            if (entry > 0x3FFF || Agc.toSigned(entry) < 0) {
                dsPlay(i, entry);
                return true;
            }
            if (i > 0) { // IF DSPCNT +, TRY AGAIN
                set("DSPCNT", i - 1);
                continue;
            }
            if (pass == 1) { // second pass starting at TABLNTH
                pass = 2;
                set("DSPCNT", TABLNTH);
                continue;
            }
            set("NOUT", 0); // second pass found nothing: +0 into NOUT
            return false;
        }
    }

    /** DSPLAY: replace entry positively, add the relay word code, WRITE OUT0. */
    private void dsPlay(int i, int entry) {
        int pos = Agc.add(entry, 1);
        set("DSPTAB+" + i, pos);
        int data = pos & LOW11;
        set("DSRUPTEM", data);
        int relay = (i < RELTAB.length ? RELTAB[i] : RELTAB11) & HI5;
        set("OUT0", relay + data);
    }

    /** HANG20: add 14,11,9 to DSRUPTSW and set TIME4 for 20 ms (SETTIME4). */
    public void hang20() {
        set("DSRUPTSW", Agc.add(g("DSRUPTSW"), ~DSRUPT_14_11_9 & Agc.WORD_MASK));
        set("TIME4", RUPT_20M);
        proceedE();
    }

    /** CDRVE: handle the DSPTAB+11D (lamp) word: if flagged, mask to LOW11 and write with RELTAB11. */
    public void cdrve() {
        int w = Agc.toSigned(g("DSPTAB+11"));
        if (w > 0) { // CCS DSPTAB+11D positive -> DSPOUT
            dspOut();
            return;
        }
        if (w == 0) {
            dspOut();
            return;
        }
        int v = g("DSPTAB+11") & LOW11;
        set("DSPTAB+11", v);
        set("OUT0", Agc.add(v, RELTAB11));
        hang20();
    }

    /** NODSPOUT: write +0 to OUT0 and set TIME4 for the next 120 ms interval. */
    public void noDspOut() {
        set("OUT0", 0);
        set("TIME4", RUPT_120M);
        proceedE();
    }

    /**
     * PROCEEDE: proceed pushbutton monitor (channel 32 bit 14). Compared with its state in the previous
     * T4RUPT: prev on/now on and prev off/now off: bypass; prev on/now off: update IMODES33; prev
     * off/now on: update IMODES33 and process via PINBALL (job PROCKEY at CHRPRIO).
     */
    public void proceedE() {
        int changed = (g("IMODES33") ^ g("CHAN32")) & BIT14;
        if (changed != 0) {
            set("IMODES33", g("IMODES33") ^ changed); // update IMODES33
            boolean wasOn = (g("IMODES33") & BIT14) == 0; // after update: bit14 = new state
            if ((g("IMODES33") & BIT14) != 0) {
                // was on -> now off: nothing more
            } else {
                // was off -> now on: start job PROCKEY at CHRPRIO
                if (exec != null) exec.novac(g("CHRPRIO"), this::procKey);
            }
        }
        t4Jump();
    }

    private void procKey() { /* PROCKEY lives in PINBALL */ }

    /**
     * T4JUMP: jump to the appropriate once-per-second (0.96 s) activity by RUPTREG1 (7..0):
     * RCSMONIT, RRAUTCHK, IMUMON, DAPT4S repeated twice per cycle.
     */
    public void t4Jump() {
        int idx = g("RUPTREG1") & 3;
        switch (idx) {
            case 0: rcsMonit(); break;
            case 1: rrAutChk(); break;
            case 2: imuMon(); break;
            default: dapT4s(); break;
        }
    }

    private void rcsMonit() {
        rcsMonitor.rcsmon();
    }

    /** QUIKDSP: if the last pass wrote relays (BIT14 of DSRUPTSW clear), turn them off; else send next. */
    public void quikDsp() {
        if ((g("DSRUPTSW") & BIT14) == 0) {
            quikOff();
            return;
        }
        if (g("NOUT") != 0 && dspOutSb()) {
            set("DSRUPTSW", g("DSRUPTSW") & ~BIT14);
            quikRupt();
            return;
        }
        noDspy();
    }

    private void quikRupt() {
        set("TIME4", RUPT_20M);
        set("DSRUPTSW", g("DSRUPTSW") + BIT9);
        resume();
    }

    /** NODSPY. */
    private void noDspy() {
        set("OUT0", 0);
        syncT4();
    }

    /** SYNCT4. */
    private void syncT4() {
        set("TIME4", Agc.add(g("TIME4"), RUPT_20M));
        set("DSRUPTSW", g("DSRUPTSW") + BIT9);
        resume();
    }

    /** QUIKOFF: turn off relays and set the DSRUPTSW BIT14 to send a display next pass. */
    private void quikOff() {
        set("OUT0", 0);
        set("DSRUPTSW", g("DSRUPTSW") | BIT14);
        quikRupt();
    }

    // ---------------------------------------------------------------- IMUMON

    /**
     * IMUMON: entered every 480 ms. Detects changes of IMU status bits in channel 30 and calls
     * the subroutines: TLIM (bit 15), ITURNON (14), IMUFAIL (13), ICDUFAIL (12), IMUCAGE (11), IMUOP (9).
     * The last sampled state is left in IMODES30; each subroutine finds the new bit value in A.
     */
    public void imuMon() {
        int changed = (g("IMODES30") ^ g("CHAN30")) & RDMSK_30;
        if (changed == 0) {
            tnonTest();
            return;
        }
        set("IMODES30", g("IMODES30") ^ changed);
        // bit scan: position 0 is bit 15 (TLIM), then 14, 13, 12, 11, 9 (10 not sampled)
        int[] bits = {BIT15, BIT14, BIT13, BIT12, BIT11, BIT10, BIT9};
        for (int pos = 0; pos < bits.length; pos++) {
            if ((changed & bits[pos]) == 0) continue;
            int newVal = g("IMODES30") & bits[pos]; // select new value of this bit
            switch (pos) {
                case 0: tlim(newVal); break;
                case 1: iTurnOn(newVal); break;
                case 2: setIssW(); break; // IMUFAIL
                case 3: setIssW(); break; // ICDUFAIL
                case 4: imuCage(newVal); break;
                case 6: imuOp(newVal); break;
                default: break;
            }
        }
        tnonTest();
    }

    /** TNONTEST: honour requests for ISS initialisation (turn-on / operate pair) .480 s after either arrives. */
    public void tnonTest() {
        if ((g("IMODES30") & BIT7) == 0) { // no request pending
            c33Test();
            return;
        }
        if ((g("IMODES30") & BIT8) != 0) {
            procTnon(); // react after a second sample
            return;
        }
        orInto("IMODES30", BIT8); // first sample: set bit to react next time
        c33Test();
    }

    /** PROCTNON: process turn-on requests after waiting 1 sample for all signals to arrive. */
    public void procTnon() {
        andNot("IMODES30", BITS7_8);
        if ((g("IMODES30") & BIT14) != 0 && false) {
            return;
        }
        if ((g("IMODES30") & BIT14) == 0) { // bit 14 = 0 : turn-on request present? (CCS on masked bit)
            opOnly();
            return;
        }
        if ((g("IMODES30") & BIT9) != 0) { // IMU OPERATE should be present (bit 9 = 0 means on)
            alarm(0213);
        }
        cageSub();
        wl(SECS_90, this::endTnon);
        c33Test();
    }

    /** RETNON: wait 90 seconds again (VARDELAY). */
    public void retNon() {
        if (waitlist != null) waitlist.varDelay(SECS_90, this::endTnon);
    }

    /** ENDTNON: end of 90 second ISS delay. */
    public void endTnon() {
        int old = g("IMODES30");
        andNot("IMODES30", BIT2); // reset turn-on request fail bit
        if ((old & BIT2) == 0) { // it was off: send ISS delay complete
            endTnon2();
            return;
        }
        if ((g("IMODES30") & BIT14) == 0) { // turn-on request now present: re-enter 90 sec delay
            retNon();
            return;
        }
        if (!any("FLAGWRD0", g("IMUSEBIT"))) { // not waiting
            if (waitlist != null) waitlist.taskOver();
            return;
        }
        set("IMUBAD_CALLED", 1); // unsuccessful turn-on
    }

    /** ENDTNON2: send ISS delay complete (CHAN12 bit 15), turn off NO ATT lamp, zero the ICDUs (UNZ2). */
    public void endTnon2() {
        orInto("CHAN12", BIT15);
        set("NOATTOFF_CALLED", 1);
        unz2();
    }

    /** UNZ2: zero ICDU, remove zero & coarse, wait 10 s for counters to find gimbals, then ISSUP. */
    public void unz2() {
        set("ZEROICDU_CALLED", 1);
        andNot("CHAN12", BIT4 | BIT5);
        if (waitlist != null) waitlist.varDelay(BIT11, this::issUp); // 10.24 s
    }

    /** ISSUP: remove cage flags, enable DAP, start freefall IMUCOMP, set ISS warning, enable PFAIL later. */
    public void issUp() {
        andNot("IMODES30", OCT54);
        andNot("IMODES33", BIT6); // enable DAP
        if (!any("FLAGWRD2", g("DRFTBIT"))) { // DRIFTFLG off: set it and 1/PIPADT
            orInto("FLAGWRD2", g("DRFTBIT"));
            set("1/PIPADT", g("TIME1"));
        }
        setIssW();
        andNot("CHAN12", BIT15); // remove IMU delay complete discrete
        wl(400, this::pfailOk); // don't enable prog alarm on PIP fail for another 4 secs
        if (waitlist != null) waitlist.taskOver();
    }

    private void pfailOk() { /* defined in IMU mode switching routines */ }

    /** OPONLY: only ISS OPERATE present: zero CDUs unless in coarse align or IMU in use. */
    public void opOnly() {
        if (any("CHAN12", BIT4)) { // coarse align: possible gimbal lock
            c33Test();
            return;
        }
        if (any("FLAGWRD0", g("IMUSEBIT"))) {
            c33Test();
            return;
        }
        cageSub2();
        issZero();
    }

    /** ISSZERO: ISS CDU zero (also the IMU cage-off entry). */
    public void issZero() {
        set("NOATTOFF_CALLED", 1);
        orInto("CHAN12", BIT5);
        set("ZEROICDU_CALLED", 1);
        wl(BIT6, this::unz2); // wait 300 ms for AGS to receive signal
        c33Test();
    }

    // ---------------------------------------------------------------- C33TEST

    /**
     * C33TEST: monitors three flip-flop inbits of channel 33 (13 PIPA fail, 12 downlink too fast,
     * 11 uplink too fast). Reads with WAND to reset the flip-flops.
     */
    public void c33Test() {
        int l = g("IMODES33") & RDMSK_33;
        int chan = g("CHAN33") & RDMSK_33;
        andNot("CHAN33", RDMSK_33); // WAND with RDMSK_33 resets flip-flop inputs: result in A
        int changed = (chan ^ l) & RDMSK_33;
        if (changed == 0) {
            glockMon();
            return;
        }
        set("IMODES33", g("IMODES33") ^ changed);
        int[] bits = {BIT13, BIT12, BIT11};
        for (int pos = 0; pos < bits.length; pos++) {
            if ((changed & bits[pos]) == 0) continue;
            int newVal = g("IMODES33") & bits[pos];
            switch (pos) {
                case 0: pipFail(newVal); break;
                case 1: dnTmFast(newVal); break;
                default: upTmFast(newVal); break;
            }
        }
        glockMon();
    }

    // ---------------------------------------------------------------- GLOCKMON

    /**
     * GLOCKMON: monitors CDUZ (middle gimbal angle). |MGA| &lt;= 70 deg normal; 70-85 deg gimbal lock
     * lamp on; &gt; 85 deg ISS put in coarse align and NO ATT lamp turned on.
     */
    public void glockMon() {
        int cdu = Agc.toSigned(g("CDUZ"));
        // CCS CDUZ: magnitude of MGA (CCS gives |A|-1 ; sign selects fold).
        int mag = Math.abs(cdu);
        int a;
        if (cdu >= 0) {
            a = Math.max(0, cdu - 1);
        } else {
            a = Math.max(0, -cdu - 1);
        }
        glockChk(Math.abs(cdu) == 0 ? 0 : mag);
    }

    /** GLOCKCHK. */
    private void glockChk(int mga) {
        int d = Agc.toSigned(Agc.add(Agc.fromSigned(mga), NEG_70DEGS));
        if (d <= 0) { // no lock
            setGlock(0);
            return;
        }
        int d2 = Agc.toSigned(Agc.add(Agc.fromSigned(d), NEG_15DEGS));
        if (d2 > 0 && !any("CHAN12", BIT4)) {
            // > 85 degrees: system should be in coarse align to prevent gimbal runaway
            set("SETCOARS_CALLED", 1);
            wl(6, this::caEce); // enable ISS error counters in 60 ms
        }
        setGlock(BIT6); // NOGIMRUN: turn on gimbal lock lamp
    }

    private void caEce() { /* CA+ECE defined elsewhere */ }

    /** SETGLOCK: bring the DSKY gimbal-lock lamp (DSPTAB+11D bit 6) into agreement with desired state. */
    private void setGlock(int desired) {
        int tab = g("DSPTAB+11");
        if (((tab ^ desired) & BIT6) == 0) {
            resume(); // GLOCKOK
            return;
        }
        if ((tab & BIT6) != 0) { // lamp is on, wants off: turn off unless lamp test in progress
            if (lampTestInProgress()) {
                resume();
                return;
            }
            glInvert();
            return;
        }
        if (any("IMODES30", BIT6)) { // IMU being caged: don't turn on
            resume();
            return;
        }
        glInvert();
    }

    /** GLINVERT: invert the gimbal lock lamp bit and flag the change in DSPTAB+11D (bit 15). */
    private void glInvert() {
        int tab = g("DSPTAB+11");
        int inverted = ~tab & BIT6;
        int changed = inverted | BIT15;
        int old = tab;
        set("DSPTAB+11", changed);
        set("DSPTAB+11", Agc.add(g("DSPTAB+11"), old & OCT37737));
        resume();
    }

    /** LAMPTEST: returns true if a lamp test is in progress (IMODES33 bit 1 = 1). */
    private boolean lampTestInProgress() {
        return any("IMODES33", BIT1);
    }

    // ---------------------------------------------------------------- IMUMON subroutines

    /** TLIM: keep the DSKY TEMP lamp (CHAN11 bit 4) in agreement with the ISS temp signal. */
    public void tlim(int newBit) {
        if (Agc.toSigned(g("IMODES30")) > 0 || g("IMODES30") == 0) {
            // CCS IMODES30 positive or +0: TEMPOK
            if (!lampTestInProgress()) {
                andNot("DSALMOUT", BIT4);
            }
            return;
        }
        orInto("DSALMOUT", BIT4); // turn on lamp
    }

    /** ITURNON: change of ISS turn-on request (bit 14). */
    public void iTurnOn(int newBit) {
        if (any("IMODES30", BIT2)) return; // delay sequence failed: wait for the 90 s to expire
        if ((g("IMODES30") & BIT14) != 0) {
            // request went off
            if (any("CHAN12", BIT15)) return; // delay discrete sent: action complete
            orInto("IMODES30", BIT2); // request not present for full duration
            alarm(0207);
            return;
        }
        iTurnOn2();
    }

    /** ITURNON2: set BIT7 to indicate wait of one sample; reset RR to initial state. */
    public void iTurnOn2() {
        orInto("IMODES30", BIT7);
        set("RADMODES", RRINIT);
    }

    /**
     * IMUCAGE: changes of the IMU cage inbit (bit 11). Nothing happens when the bit goes off;
     * a cage (button pressed) terminates ICDU/RCDU/gyro pulse trains, zeroes counters,
     * de-selects gyros, clears ENGONFLG and turns the engine off.
     */
    public void imuCage(int newBit) {
        if (newBit != 0) { // CCS A positive: goes off ... ISSZERO
            issZero();
            return;
        }
        andNot("CHAN14", OCT77000); // terminate ICDU, RCDU, gyro pulse trains
        andNot("CHAN12", OCT272); // knock down display inertial data, IMU error counter, ZERO ICDU, coarse...
        andNot("FLAGWRD5", g("ENGONBIT"));
        set("DSALMOUT", (g("DSALMOUT") & ~g("PRIO30")) | BIT14); // force bit14=1, bit13=0
        cageSub1();
        set("RNDREFDR_CALLED", 1); // knock down TRACK, REFSMMAT, DRIFT flags
        set("CDUXCMD", 0);
        set("CDUYCMD", 0);
        set("CDUZCMD", 0);
        set("GYROCMD", 0);
        andNot("CHAN14", OCT740); // de-select the gyros
    }

    /** IMUOP: change in ISS operate discrete (bit 9). */
    public void imuOp(int newBit) {
        if (newBit == 0) { // goes to 0: ISS on
            imuOp2();
            return;
        }
        orInto("IMODES33", BIT6); // disable DAP
        set("RNDREFDR_CALLED", 1);
        int old = g("FLAGWRD0");
        set("FLAGWRD0", old & ~BITS7_8);
        if ((old & g("IMUSEFLG")) != 0) { // IMU in use by a program
            alarm(0214);
        }
    }

    /** IMUOP2. */
    private void imuOp2() {
        if (any("IMODES30", BIT2)) return;
        iTurnOn2();
    }

    // ---------------------------------------------------------------- channel 33 handlers

    /** PIPFAIL: set bit 10 of IMODES30 to agree; call SETISSW; alarm 0212 if no ISS warning. */
    public void pipFail(int newBit) {
        int bit10 = newBit != 0 ? BIT10 : 0;
        set("IMODES30", (g("IMODES30") & ~BIT10) | bit10);
        setIssW();
        if ((g("IMODES30") & BIT1) == 0 && (g("IMODES30") & OCT1720) == 0) {
            alarm(0212);
        }
    }

    /** DNTMFAST: downlink too fast alarm 01105 if the bit changed to 0. */
    public void dnTmFast(int newBit) {
        if (newBit == 0) alarm(01105);
    }

    /** UPTMFAST: uplink too fast alarm 01106 if the bit changed to 0. */
    public void upTmFast(int newBit) {
        if (newBit == 0) alarm(01106);
    }

    /**
     * SETISSW: turns the ISS warning lamp (CHAN11 bit 1) on/off from IMODES30 bits 13/12/10
     * (IMU/ICDU/PIPA fail) and 4/3/1 (inhibits). The lamp is left on if lamp test in progress.
     * Alarm codes 777 etc. (VARALARM) show the cause when on.
     */
    public void setIssW() {
        int im = g("IMODES30");
        int fails = im & 0x1000 | im & 0x0800 | im & 0x0200; // bits 13,12,10
        int inhibits = (im & 0x0008) << 9 | (im & 0x0004) << 9 | (im & 0x0001) << 9; // 4,3,1 -> 13,12,10
        boolean failure = (fails & ~inhibits & OCT15000 & 0x7FFF) != 0 || ((~(im | (im << 9)) & OCT15000) != 0 && false);
        // 0 indicates failure after ROR LCHAN: failure present where fail bit is 1 and inhibit bit is 0
        int active = fails & ~(((im & 8) << 9) | ((im & 4) << 9) | ((im & 1) << 9));
        failure = (active & OCT15000) != 0 || (active != 0);
        if (failure) {
            set("VARALARM", active);
            orInto("DSALMOUT", BIT1);
            return;
        }
        if (!any("IMODES33", BIT1)) { // not in lamp test
            andNot("DSALMOUT", BIT1);
        }
    }

    /** CAGESUB: set outbits and flags for system turn-on or cage; disable error counter and delay complete. */
    public void cageSub() {
        andNot("CHAN12", BITS6_15);
        orInto("CHAN12", BIT4 | BIT5); // send zero and coarse
        cageSub1();
    }

    /** CAGESUB1: turn on NO ATT lamp. */
    public void cageSub1() {
        orInto("DSPTAB+11", OC40010 & ~g("DSPTAB+11"));
        cageSub2();
    }

    /** CAGESUB2: set flags indicating caging or turn-on, inhibit ISS warning info, disable DAP auto/hold. */
    public void cageSub2() {
        orInto("IMODES30", OCT75 & ~g("IMODES30"));
        orInto("IMODES33", BIT6);
    }

    // ---------------------------------------------------------------- RR monitors

    /**
     * RRAUTCHK: rendezvous radar inbit monitor. Checks the RR power-on-auto bit (CHAN33 bit 2);
     * on a change updates RADMODES. If power just came on and no program using the RR, sets
     * RRCDUZRO and TURNON (bits 13,1 -> OCT10001) and a 10 ms waitlist call for RRTURNON.
     */
    public void rrAutChk() {
        int changed = (g("RADMODES") ^ g("CHAN33")) & g("AUTOMBIT");
        if (changed == 0) {
            rrCduChk();
            return;
        }
        int updated = (g("RADMODES") ^ changed) & OCT05776;
        set("RADMODES", updated);
        if ((updated & BIT2) == 0) { // off: disable RR CDU error counters
            andNot("CHAN12", BIT2);
            rrCduChk();
            return;
        }
        orInto("RADMODES", 010001);
        wl(1, this::rrTurnOn);
        noRrGMon();
    }

    private void rrTurnOn() { /* RRTURNON lives in the radar routines */ }

    /**
     * RRCDUCHK: RR CDU fail (CHAN30 bit 7). On change, if RR is in AUTO updates RADMODES bit 7 and
     * (if P20 operating) raises alarm 00515; updates the tracker fail lamp via SETTRKF.
     */
    public void rrCduChk() {
        int changed = (g("RADMODES") ^ g("CHAN30")) & g("RCDUFBIT");
        if (changed == 0) {
            rrGiMon();
            return;
        }
        if (!any("RADMODES", g("AUTOMBIT"))) { // not in auto mode
            noRrGMon();
            return;
        }
        set("RADMODES", g("RADMODES") ^ changed);
        if ((g("RADMODES") & g("RCDUFBIT")) == 0) { // RR CDU failed
            if (!any("FLAGWRD0", g("RNDVZBIT"))) {
                alarm(0515);
            }
        }
        set("SETTRKF_CALLED", 1); // update tracker fail lamp (DSPTAB+11D bit 8)
        rrGiMon();
    }

    /** RRGIMON: RR gimbal limit monitor. Calls RRLIMCHK; if outside limits start reposition. */
    public void rrGiMon() {
        if (any("FLAGWRD5", g("NORRMBIT"))) { // no angle monitor flag
            noRrGMon();
            return;
        }
        boolean servicerRunning = !any("FLAGWRD7", g("AVEGFBIT")) ? false : true;
        if (!any("FLAGWRD7", g("AVEGFBIT")) || true) {
            // CS FLAGWRD7 ; MASK AVEGFBIT : CCS positive -> servicer not running, do R25
            if (any("FLAGWRD7", g("AVEGFBIT")) && any("FLAGWRD6", g("MUNFLBIT"))) {
                noRrGMon(); // servicer running and MUNFLAG set: don't do R25
                return;
            }
        }
        if (any("RADMODES", OCT32002)) { // inhibited by remode, zeroing, monitor or RR not auto
            noRrGMon();
            return;
        }
        boolean inLimits = rrLimChk(g("CDUT"));
        if (inLimits) {
            noRrGMon();
            return;
        }
        monRepos();
    }

    /** RRLIMCHK result (defined in radar routines): true if present RR CDU angles are within limits. */
    private boolean rrLimChk(int cdu) {
        return g("RRLIMCHK_RESULT") != 0;
    }

    /** MONREPOS: set reposition flag; disable tracker and error counter; DORREPOS in 20 ms. */
    public void monRepos() {
        orInto("RADMODES", g("REPOSBIT"));
        andNot("CHAN12", OCT20002);
        wl(2, this::dorRepos);
        noRrGMon();
    }

    private void dorRepos() { /* DORREPOS lives in the radar routines */ }

    /** NORRGMON EQUALS DAPT4S: no RR gimbal action; fall through to the DAP matrix. */
    public void noRrGMon() {
        dapT4s();
    }

    // ---------------------------------------------------------------- GPMATRIX (DAPT4S)

    /**
     * GPMATRIX / DAPT4S (J D Addleston, Adams Associates; mod 7 Feb 1968 by P S Weissman).
     * Calculates the single-precision matrix elements used by the LM DAP to transform vectors from
     * gimbal to pilot axes (OIM = XYZ, AOG = CDUX, AIG = CDUY, AMG = CDUZ):
     * <pre>
     *   M_GP = | SIN(MG)            0          1 |
     *          | COS(MG)COS(OG)   SIN(OG)      0 |
     *          | -COS(MG)SIN(OG)  COS(OG)      0 |
     * </pre>
     * Executed four times a second from T4RUPT. Output M11, M21, M31, M22, M32. Exit RESUME.
     */
    public void dapT4s() {
        gpMatrix();
    }

    public void gpMatrix() {
        int m11 = sp.spsin(g("CDUZ")); // SIN(MG), scaled at 1
        set("M11", m11);
        int cosMg = sp.spcos(g("CDUZ")); // COS(MG)
        set("COSMG", cosMg);
        int m22 = sp.spsin(g("CDUX")); // SIN(OG) (also MR22)
        set("M22", m22);
        // -SIN(OG)*COS(MG)
        int m31 = Agc.fromSigned((int) Math.round(-(Agc.toSigned(m22) / 16384.0) * (Agc.toSigned(cosMg) / 16384.0) * 16384.0));
        set("M31", m31);
        int m32 = sp.spcos(g("CDUX")); // COS(OG) (also MR23)
        set("M32", m32);
        // COS(OG)*COS(MG)
        int m21 = Agc.fromSigned((int) Math.round((Agc.toSigned(m32) / 16384.0) * (Agc.toSigned(cosMg) / 16384.0) * 16384.0));
        set("M21", m21);
        resume(); // ENDDAPT4 = RESUME
    }
}
