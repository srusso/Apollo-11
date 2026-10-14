package apollo.luminary099;

import apollo.agc.Agc;

/**
 * BURN_BABY_BURN--MASTER_IGNITION_ROUTINE.agc (Luminary 1A build 099, pages 731-751).
 *
 * "Burn, baby, burn" - the master ignition routine by Peter Adler and Don Eyles, used by the LM
 * programs P12, P40, P42, P61, P63 (and abort). It performs everything between the pre-ignition
 * time check (within 45 s of TIG) and TIG + 26 s when DPS programs throttle up. Program
 * variations are handled by tables of constants and "TCF" entries selected with erasable
 * register WHICH (E4); here each table is an {@link IgnitionTable} of Runnables.
 *
 * Also contains KILLTASK (by Covelli): removes a task from the waitlist by substituting a null
 * task (TASKOVER). The first matching entry (soonest) is removed.
 */
public final class BurnBabyBurn {

    /** One ignition table. Slots follow the octal table offsets (0)..(14) of the original. */
    public static final class IgnitionTable {
        /** (0) verb/noun for the countdown display (VN). */
        public int vn;
        /** (1) ULLGNOT / WANTAPS. */
        public Runnable ullage;
        /** (2) COMFAIL3 / COMFAIL4. */
        public Runnable comfail;
        /** (3) GOCUTOFF / GOPOST / V99RECYC. */
        public Runnable cutoff;
        /** (4) TASKOVER. */
        public Runnable taskOver;
        /** (5) program SPOT routine. */
        public Runnable spot;
        /** (6) ullage time in centiseconds (negative = P41, zero = none). */
        public int ullageTime;
        /** (7) 2CADR of the AVEGEXIT routine (job name). */
        public String avegExit;
        /** (11) DISPCHNG / P40SJUNK / COMMON. */
        public Runnable disp;
        /** (12) WAITABIT / TIGTASK. */
        public Runnable wait;
        /** (13) ignition entry. */
        public Runnable ign;
        /** (14) REP40ALM / P42STAGE. */
        public Runnable alt;
    }

    // ---- Constants for the ignition routine
    public static final int D29_9SEC = 2990;
    public static final int S24_9SEC = 2490;
    public static final int S4_9SEC = 490;
    public static final int P40A_PMD = 00203;
    public static final int V06N61 = 661;
    public static final int VB97DEX = 035; // OCT35
    public static final int CNTDNDEX = 017; // LOW4
    public static final int VB99DEX = 013; // ELEVEN
    public static final int BLANKDEX = 2;
    public static final int LSTLIM = 16;
    public static final int ALARM_VEHICLE_CONFIG = 01706;

    private final Agc agc;
    private Executive exec;
    private Waitlist waitlist;

    public final IgnitionTable p12Table = new IgnitionTable();
    public final IgnitionTable p40Table = new IgnitionTable();
    public final IgnitionTable p41Table = new IgnitionTable();
    public final IgnitionTable p42Table = new IgnitionTable();
    public final IgnitionTable p63Table = new IgnitionTable();
    public final IgnitionTable abrTable = new IgnitionTable();

    /** Currently selected table (erasable register WHICH). */
    private IgnitionTable which = p40Table;

    public BurnBabyBurn(Agc agc) {
        this.agc = agc;
        buildTables();
    }

    public void setExecutive(Executive exec) { this.exec = exec; }
    public void setWaitlist(Waitlist waitlist) { this.waitlist = waitlist; }
    /** Selects the table (the program loads WHICH with the ADRES of its table). */
    public void setWhich(IgnitionTable t) { this.which = t; }

    private int g(String s) { return agc.get(s); }
    private void set(String s, int v) { agc.set(s, v); }

    private void buildTables() {
        // P12TABLE
        p12Table.vn = 674; p12Table.ullage = this::ullgNot; p12Table.comfail = this::comfail3;
        p12Table.cutoff = this::goCutoff; p12Table.taskOver = this::taskOver; p12Table.spot = this::p40Spot;
        p12Table.ullageTime = 0; p12Table.avegExit = "SERVEXIT";
        p12Table.disp = this::dispChng; p12Table.wait = this::waitABit; p12Table.ign = this::p12Ign;
        // P40TABLE
        p40Table.vn = 640; p40Table.ullage = this::ullgNot; p40Table.comfail = this::comfail4;
        p40Table.cutoff = this::goPost; p40Table.taskOver = this::taskOver; p40Table.spot = this::p40Spot;
        p40Table.ullageTime = 2240; p40Table.avegExit = "STEERING";
        p40Table.disp = this::p40SJunk; p40Table.wait = this::waitABit; p40Table.ign = this::p40Ign;
        p40Table.alt = this::rep40Alm;
        // P41TABLE (entered 5 words in: only slots 5,6,7,11,12 exist)
        p41Table.spot = this::p41Spot; p41Table.ullageTime = -1; p41Table.avegExit = "CALCN85";
        p41Table.disp = this::common; p41Table.wait = this::tigTask;
        // P42TABLE
        p42Table.vn = 640; p42Table.ullage = this::wantAps; p42Table.comfail = this::comfail4;
        p42Table.cutoff = this::goPost; p42Table.taskOver = this::taskOver; p42Table.spot = this::p40Spot;
        p42Table.ullageTime = 2640; p42Table.avegExit = "STEERING";
        p42Table.disp = this::p40SJunk; p42Table.wait = this::waitABit; p42Table.ign = this::p42Ign;
        p42Table.alt = this::p42Stage;
        // P63TABLE
        p63Table.vn = 662; p63Table.ullage = this::ullgNot; p63Table.comfail = this::comfail3;
        p63Table.cutoff = this::v99Recyc; p63Table.taskOver = this::taskOver; p63Table.spot = this::p41Spot;
        p63Table.ullageTime = 2240; p63Table.avegExit = "SERVEXIT";
        p63Table.disp = this::dispChng; p63Table.wait = this::waitABit; p63Table.ign = this::p63Ign;
        // ABRTABLE
        abrTable.vn = 663; abrTable.ullage = this::ullgNot; abrTable.comfail = this::comfail3;
        abrTable.cutoff = this::goCutoff; abrTable.taskOver = this::taskOver;
        abrTable.spot = () -> { }; // NOOP (5)
        abrTable.disp = this::dispChng; abrTable.wait = this::waitABit; abrTable.ign = this::abrtIgn;
    }

    // ---- bit helpers on flag words
    private void setBit(String word, int mask) { set(word, g(word) | mask); }
    private void clearBit(String word, int mask) { set(word, g(word) & ~mask); }
    private void upFlag(String flag) { exec_flag(flag, true); }
    private void downFlag(String flag) { exec_flag(flag, false); }
    private void exec_flag(String flag, boolean up) { set("FLAG_" + flag, up ? 1 : 0); }
    private void phasChng(int code) { set("PHASCHNG_LAST", code); } // TC PHASCHNG ; OCT code
    private void twiddle(int dt, Runnable task) {
        if (waitlist != null) waitlist.twiddle(dt, task);
    }
    private void taskOver() { if (waitlist != null) waitlist.taskOver(); }
    private void endOfJob() { if (exec != null) exec.endOfJob(); }

    // ---------------------------------------------------------------- general ignition routines

    /**
     * BURNBABY: group 4 restarts here. Clears DVTOTAL, checks PGNCS/auto via P40AUTO, stores
     * nominal TIG for oblateness compensation (GOBLTIME), switches the engine off (ENGINOF3), then
     * dispatches through (5) of the selected table.
     */
    public void burnBaby() {
        phasChng(04024);
        set("DVTOTAL", 0);
        set("DVTOTAL+1", 0);
        p40Auto();
        bStarNbStar();
    }

    /** B*RNB*B* (the original label has asterisks): store TIG, engine off, vector to (5). */
    public void bStarNbStar() {
        set("GOBLTIME", g("TIG"));
        set("GOBLTIME+1", g("TIG+1"));
        // INHINT ; IBNKCALL ENGINOF3 ; RELINT
        set("ENGINOF3_CALLED", 1);
        which.spot.run();
    }

    /** P40SPOT / P42SPOT / P12SPOT (5): start the countdown clock, then fall into P41SPOT. */
    public void p40Spot() {
        stClok2(-CNTDNDEX);
        p41Spot();
    }

    /**
     * P41SPOT / P63SPOT (5): integrate the CSM (and LM) state vector to TIG-29.9 s with MIDTOAV,
     * reload positions/velocities from the precision vectors, and slip TIG if the integration
     * could not make it in time. Then set up the LONGCALL for TIG-35.
     */
    public void p41Spot() {
        // Interpretive: TDEC1 = TIG - D29.9SEC ; INITCDUW ; if CSM state is available (MUNFLAG)
        // get VATT1/RATT1 via CSMPREC, rotate by REFSMMAT, compute gravity MUNGRAV; MIDTOAV1.
        set("TDEC1", Agc.fromSigned(Agc.toSigned(g("TIG")) - D29_9SEC));
        set("V(CSM)_SET", 1);
        boolean madeIt = g("MIDTOAV_SLIPPED") == 0;
        if (!madeIt) {
            // TIG was slipped: reset TIG to 29.9 s after the time to which we did integrate.
            set("TIG", g("PIPTIME1"));
            set("TIG+1", g("PIPTIME1+1"));
            set("TIG", Agc.add(g("TIG"), D29_9SEC));
        }
        callT35();
    }

    /** CALLT-35: SAVET-30 = DELTA-T until TIG-30; minus 5 s gives TIG-35; LONGCALL TIG-35. */
    private void callT35() {
        set("SAVET-30", g("MPAC"));
        set("SAVET-30", Agc.add(g("SAVET-30"), Agc.fromSigned(-500))); // minus 5SECDP
        if (waitlist != null) waitlist.longCall(g("SAVET-30"), this::tigMinus35, null);
        phasChng(020254); // 4.25SPOT for TIG-35 restart
        if (g("MM") == 63) { // TC CHECKMM ; DEC 63 : P63 can start displaying now
            set("DISPDEX", Agc.fromSigned(-CNTDNDEX));
            set("ABVEL", Math.abs(g("VN1"))); // initialize ABVEL for P63 display
        }
        endOfJob();
    }

    /** TIG-35: five seconds before TIG-30: blank DSKY for 5 s; set up displays restore. */
    public void tigMinus35() {
        twiddle(500, this::tigMinus30);
        phasChng(040154); // 4.15SPOT for TIG-30 restart
        set("DISPDEX", Agc.fromSigned(-BLANKDEX)); // blank DSKY for 5 seconds
        if (which.ullageTime >= 0) { // CS 6 ; BZMF TASKOVER : only P41 (negative ullage) continues
            taskOver();
            return;
        }
        twiddle(S4_9SEC, this::tigMinus30Dot1); // restore display at TIG-30
        // negative ullage time indicates P41: set up job P41BLANK (priority 17, NOVAC)
        if (exec != null) exec.novac(017, this::p41Blank);
        taskOver();
    }

    /** P41BLANK: blank DSKY for five seconds (CLOKJOB is not running during P41). */
    public void p41Blank() {
        set("CLEANDSP", 1); // BANKCALL CLEANDSP
        endOfJob();
    }

    /** TIG-30.1: set up job to restore the display at TIG-30. */
    public void tigMinus30Dot1() {
        if (exec != null) exec.novac(017, this::tigMinus30A);
        taskOver();
    }

    /** TIG-30A: restore display with V16N85B via REGODSP. */
    public void tigMinus30A() {
        set("REGODSP", 1); // CAF V16N85B ; BANKCALL REGODSP (does TCF ENDOFJOB)
    }

    /**
     * TIG-30: schedule TIG-5 at +24.9 s, restart the countdown clock display, set up ullage
     * (ULLGTASK) from table slot (6) and restart protect it (1.3SPOT); then dispatch (1).
     */
    public void tigMinus30() {
        twiddle(S24_9SEC, this::tigMinus5);
        set("DISPDEX", Agc.fromSigned(-CNTDNDEX)); // start up CLOKTASK again
        int ullage = which.ullageTime;
        if (ullage > 0) { // don't set up ullage if DT is negative or zero
            set("SAVET-30", ullage);
            twiddle(ullage, this::ullgTask);
            set("PHASE1", 3); // restart protect ULLGTASK (1.3SPOT): -PHASE1 := (-3, 3)
            set("TBASE1", g("TIME1"));
            which.ullage.run();
            return;
        }
        ullgNot();
    }

    /** WANTAPS (1): for P42 ensure APSFLAG is set (DAP initialised for ascent in 2 s by 1/ACCS). */
    public void wantAps() {
        setBit("FLGWRD10", g("APSFLBIT"));
        ullgNot();
    }

    /** ULLGNOT (1): load AVEGEXIT with the table's 2CADR; set 4.2SPOT restart (TIG-5), then REDO2.17. */
    public void ullgNot() {
        set("AVEGEXIT_JOB", which.avegExit == null ? 0 : which.avegExit.hashCode());
        set("PHASE4", 2); // 4.2SPOT restarts immediately at REDO4.2 and also at TIG-5
        set("TBASE4", g("TIME1"));
        redo217();
    }

    /** REDO2.17: clear out group 2 so LAMBERT can start if needed. */
    public void redo217() {
        set("PHASE2", 0);
        redo42();
    }

    /** REDO4.2: if the SERVICER is going end the task; otherwise PREREAD. */
    public void redo42() {
        if (g("PHASE5") != 0) { // CCS PHASE5: servicer going
            taskOver();
            return;
        }
        set("PREREAD", 1); // POSTJUMP PREREAD, which ends this task
    }

    /** ULLGTASK: comes at TIG-7.5 or TIG-3.5; turns on ullage and clears group 1 restart. */
    public void ullgTask() {
        onUllage();
        phasChng(1);
        taskOver();
    }

    /** TIG-5: five seconds before TIG: group 3 inactive, schedule TIG-0, reset flags; dispatch (11). */
    public void tigMinus5() {
        set("PHASE3", 0);
        twiddle(500, this::tigMinus0);
        downFlag("IGNFLAG"); // reset IGNFLAG and ASTNFLAG for light-up logic
        downFlag("ASTNFLAG");
        which.disp.run();
    }

    /** P40SJUNK (11), P40 and P42: start S40.13 unless it is already in progress, then DISPCHNG. */
    public void p40SJunk() {
        if (g("PHASE3") != 0) { // S40.13 in progress
            dispChng();
            return;
        }
        if (exec != null) exec.findVac(020, this::s4013);
        phasChng(00053); // 3.5SPOT for S40.13
        dispChng();
    }

    private void s4013() { /* S40.13 lives in another file */ }

    /** DISPCHNG (11): display V99 (enable engine request) via DISPDEX. */
    public void dispChng() {
        set("DISPDEX", Agc.fromSigned(-VB99DEX));
        common();
    }

    /** COMMON: restart at TIG-0 (4.7SPOT). */
    public void common() {
        phasChng(040074);
        taskOver();
    }

    /**
     * TIG-0: set IGNFLAG since TIG has arrived. For P63, throttle-up (ZOOMTIME) is measured
     * after nominal ignition. Check ASTNFLAG: if the astronaut has not responded yet, vector to
     * (12) WAITABIT/TIGTASK; otherwise IGNITION.
     */
    public void tigMinus0() {
        setBit("FLAGWRD7", g("IGNFLBIT"));
        if (g("MM") == 63) {
            twiddle(g("ZOOMTIME"), this::p63Zoom);
            phasChng(040033); // 2PHSCHNG: 3.3SPOT for ZOOM, 05014/77777 type C restart
        }
        // IGNYET?
        if ((g("FLAGWRD7") & g("ASTNBIT")) == 0) {
            which.wait.run(); // BZF (12): astronaut has not responded yet
            return;
        }
        ignition();
    }

    /**
     * IGNITION: ensure ENGONFLG is set, turn the engine on via DSALMOUT bit 13, set TEVENT for
     * downlink, update TIG using TGO from S40.13, permit guidance loop displays; dispatch (13).
     */
    public void ignition() {
        setBit("FLAGWRD5", g("ENGONBIT"));
        set("DSALMOUT", (g("DSALMOUT") & ~g("PRIO30")) | 0x1000); // BIT13: engine on
        set("TEVENT", g("TIME2"));
        set("TEVENT+1", g("TIME2+1"));
        set("TIG", Agc.add(g("TGO"), g("TIME2"))); // TIG := TGO + TIME2
        clearBit("FLAGWRD8", g("FLUNDBIT")); // permit guidance loop displays
        which.ign.run();
    }

    /** P63IGN (13): initiate burn displays for P63, then P42IGN path. */
    public void p63Ign() {
        set("AVGEXIT_JOB", "P63DISPS".hashCode()); // DSP2CADR
        set("DISPDEX", 0); // assassinate CLOKTASK
        setBit("FLAGWRD9", g("LETABBIT")); // set flag for P70-P71
        setBit("FLAGWRD7", g("SWANDBIT")); // SWANDISP enables R10
        clearBit("DAPBOOLS", g("PULSES")); // DAP not in minimum-impulse mode
        set("TIG", g("TIME2")); // initialise TIG for P70 and P71
        set("TIG+1", g("TIME2+1"));
        set("WCHPHASE", 0);
        set("WCHPHOLD", 0);
        set("FLPASS0", 2);
        p42Ign();
    }

    /** P40IGN (13): if NOTHRBIT clear schedule P40ZOOM at ZOOMTIME, then P42IGN. */
    public void p40Ign() {
        if ((g("FLAGWRD5") & g("NOTHRBIT")) == 0) {
            p42Ign();
            return;
        }
        if (waitlist != null) waitlist.waitlist(g("ZOOMTIME"), this::p40Zoom);
        phasChng(040033); // 3.3SPOT for ZOOM restart (type C restarts immediately 05014/77777)
        p42Ign();
    }

    /** P12IGN (13): initialise DAP bias acceleration estimates AOSQ, AOSR (EBANK 6), then ABRTIGN. */
    public void p12Ign() {
        set("AOSQ", g("IGNAOSQ"));
        set("AOSR", g("IGNAOSR"));
        abrtIgn();
    }

    /** ABRTIGN (13): kill CLOKTASK, connect ascent guidance (ATMAG) to the servicer, enable R10. */
    public void abrtIgn() {
        set("DISPDEX", 0);
        set("AVGEXIT_JOB", "ATMAG".hashCode());
        setBit("FLAGWRD7", g("SWANDBIT"));
        p42Ign();
    }

    /** P42IGN: ensure powered-flight switching curves are used; check impulse switch; connect DVMON. */
    public void p42Ign() {
        clearBit("DAPBOOLS", g("DRIFTBIT"));
        if ((g("FLAGWRD2") & g("IMPULBIT")) != 0) {
            implBurn();
            return;
        }
        dvMonCon();
    }

    private void implBurn() { set("IMPLBURN", 1); /* defined in another file */ }

    /** DVMONCON: connect DVMON; 4.5SPOT; ullage off half a second after light-up (ULLAGOFF). */
    public void dvMonCon() {
        downFlag("IGNFLAG");
        downFlag("ASTNFLAG");
        downFlag("IDLEFLAG");
        phasChng(040054);
        if (waitlist != null) waitlist.fixDelay(50, this::ullagOff);
    }

    /** ULLAGOFF. */
    public void ullagOff() {
        noUllage();
        waitABit();
    }

    /** WAITABIT (12): kill group 4. */
    public void waitABit() {
        set("PHASE4", 0);
        taskOver();
    }

    /** TIGTASK (12): P41: go to TIGTASK1 which starts TIGNOW (priority 16) and kills group 6. */
    public void tigTask() {
        tigTask1();
    }

    /** TIGTASK1: start job TIGNOW; kill group 6. */
    public void tigTask1() {
        if (exec != null) exec.novac(016, () -> { /* TIGNOW */ });
        phasChng(6);
        taskOver();
    }

    /** P63ZOOM: connect lunar landing guidance (LUNLAND) and flatten out (FLATOUT). */
    public void p63Zoom() {
        set("AVEGEXIT_JOB", "LUNLAND".hashCode());
        set("FLATOUT_CALLED", 1);
        p40ZoomA();
    }

    /** P40ZOOM: command full thrust (THRUST = BIT13) and set channel 14 bit 4. */
    public void p40Zoom() {
        set("THRUST", 0x1000);
        set("CHAN14", g("CHAN14") | 0x08); // WOR CHAN14 with BIT4
        p40ZoomA();
    }

    /** P40ZOOMA / ZOOM: clear 3.x restart protection and end task. */
    public void p40ZoomA() {
        phasChng(3);
        taskOver();
    }

    // ---------------------------------------------------------------- COMFAIL and abort handling

    /** COMFAIL (15): suppress conflicting display, reset DVMON, start the clock, kill group 4. */
    public void comfail() {
        upFlag("IDLEFLAG");
        upFlag("FLUNDISP");
        set("DVCNTR", 4);
        if (g("PHASE6") == 0) {
            stClok1();
        }
        set("DISPDEX", Agc.fromSigned(-VB97DEX));
        phasChng(4);
        endOfJob();
    }

    /** COMFAIL1: INDEX WHICH ; TCF 2 (the (2) slot). */
    public void comfail1() {
        which.comfail.run();
    }

    /** COMFAIL3 (15): kill CLOKTASK using Z; reconnect DV monitor. */
    public void comfail3() {
        set("DISPDEX", 0);
        comfailEnd();
    }

    /** COMFAIL4 (15). */
    public void comfail4() {
        set("DISPDEX", Agc.fromSigned(-CNTDNDEX));
        comfailEnd();
    }

    private void comfailEnd() {
        downFlag("IDLEFLAG"); // reconnect DV monitor
        downFlag("FLUNDISP"); // permit guidance loop displays
        endOfJob();
    }

    /** COMFAIL2: kill ZOOM restart protection, kill ZOOM task, command engine off, back to ullage. */
    public void comfail2() {
        phasChng(3);
        killTask("ZOOM");
        set("ENGINOF4_CALLED", 1);
        upFlag("DRIFTDFL"); // set the drift bit for the DAP
        set("FLAG_AORBTFLG", g("FLAG_AORBTFLG") ^ 1); // INVFLAG: use other RCS system
        upFlag("ULLAGFLG"); // turn on ullage
        twiddle(1, this::tigMinus5);
        endOfJob();
    }

    // ---------------------------------------------------------------- subroutines

    /** NOULLAGE: turn off ullage (clear ULLAGER in DAPBOOLS). Call in a task or inhibited. */
    public void noUllage() {
        clearBit("DAPBOOLS", g("ULLAGER"));
    }

    /** ONULLAGE: turn on ullage (set ULLAGER in DAPBOOLS). */
    public void onUllage() {
        setBit("DAPBOOLS", g("ULLAGER"));
    }

    /** STCLOK1: start the countdown (CLOKTASK/CLOKJOB) with DISPDEX = 0. */
    public void stClok1() {
        stClok2(0);
    }

    /**
     * STCLOK2/STCLOK3: set DISPDEX and start the countdown. Setting DISPDEX positive kills it.
     * Schedules CLOKTASK at the next whole second of TIG-TIME2 (remainder + 2 cs).
     */
    public void stClok2(int dispdex) {
        set("DISPDEX", Agc.fromSigned(dispdex));
        int tig = g("TIG");
        int diff = Agc.toSigned(tig) - Agc.toSigned(g("TIME2")); // TIG - TIME2, positive past 45 s check
        int remainder = Math.floorMod(diff, 100); // DV by 1SEC after masking to LOW5 of the high word
        twiddle(remainder + 2, this::clokTask);
        phasChng(040036); // 6.3SPOT for CLOKTASK (05024, 13000 follow)
    }

    /** CLOKTASK: every second spawn CLOKJOB (priority 27) unless DISPDEX is positive (killed). */
    public void clokTask() {
        set("TBASE6", g("TIME1"));
        if (Agc.toSigned(g("DISPDEX")) > 0) {
            killClok();
            return;
        }
        if (exec != null) exec.novac(027, this::clokJob);
        if (waitlist != null) waitlist.fixDelay(100, this::clokTask);
    }

    /** KILLCLOK: kill restart protection for group 6. */
    public void killClok() {
        set("PHASE6", 0);
        taskOver();
    }

    /**
     * CLOKJOB: TTOGO = TIG - TIME2, then vector through DISPDEX to the display routine selected
     * by (negated) DISPDEX: -35 (V97), -25 (V06N61), -17 (countdown REGODSP), -13 (V99), -2 (blank).
     */
    public void clokJob() {
        set("TTOGO", Agc.fromSigned(Agc.toSigned(g("TIG")) - Agc.toSigned(g("TIME2"))));
        int dispdex = Agc.toSigned(g("DISPDEX"));
        if (dispdex >= 0) { // set positive by a task or higher priority job: do not use as index
            endOfJob();
            return;
        }
        switch (-dispdex) {
            case VB97DEX: v97Paste(); break;
            case 025: v06n61(); break;
            case CNTDNDEX: countdown(); break;
            case VB99DEX: v99Paste(); break;
            case BLANKDEX: blankDsky(); break;
            default: endOfJob(); break;
        }
    }

    /** DISPNOT -35: indicate verb 97 paste, display via CLOCPLAY; responses: terminate/COMFAIL1/COMFAIL2. */
    private void v97Paste() {
        set("NVWORD1", 0xFFFF & Agc.WORD_MASK); // CS ZERO : indicate V97 paste
        set("CLOCPLAY", g("NVWORD+2")); // NVWORD+2 holds V06 and the appropriate noun
        // returns: V34 -> STOPCLOK, V33 -> COMFAIL1, V32 -> COMFAIL2 (determined by crew keystroke)
    }

    /** -25: V06N61 display via REFLASH (reset event timer to agree with TIG); used in P63 via ASTNCLOK. */
    private void v06n61() {
        set("REFLASH", V06N61);
        // V34 -> STOPCLOK ; V33 -> ASTNRETN ; V32 -> redisplay
    }

    /** -17: countdown display from table slot (0) (VN) via REGODSP at one second intervals. */
    private void countdown() {
        set("REGODSP", which.vn);
    }

    /** -13: V99 "please enable engine" display; V34 -> STOPCLOK, V33 -> *PROCEED, V32 -> *ENTER. */
    private void v99Paste() {
        set("NVWORD1", ~0x100 & Agc.WORD_MASK); // CS BIT9 : indicate V99 paste
        set("CLOCPLAY", which.vn);
    }

    /** V99RECYC (3) for P63 equals the V99 recycle entry. */
    public void v99Recyc() {
        v99Paste();
    }

    /** -2: blank DSKY (CLEANDSP) for 5 seconds at TIG-35 to indicate average G is starting. */
    private void blankDsky() {
        set("CLEANDSP", 1);
        endOfJob(); // DISPNOT
    }

    /** STOPCLOK: stop CLOKTASK and turn ullage off on the way to P00 (GOTOPOOH). */
    public void stopClok() {
        nullClok();
        set("GOTOPOOH", 1);
    }

    /** NULLCLOK: inhibit; ullage off, kill ULLGTASK, clear group 1 restart, kill CLOKTASK. */
    public void nullClok() {
        noUllage();
        killTask("ULLGTASK");
        phasChng(1);
        set("DISPDEX", 0);
    }

    /** ASTNRETN: stop displaying but keep running; start job ASTNRET (priority 13). */
    public void astnRetn() {
        phasChng(04024);
        set("DISPDEX", 0);
        if (exec != null) exec.findVac(013, () -> { /* ASTNRET */ });
        endOfJob();
    }

    /** *PROCEED: astronaut approved ignition: set ASTNFLAG, then IGNITE. */
    public void starProceed() {
        upFlag("ASTNFLAG");
        ignite();
    }

    /** *ENTER: dispatch through (3) of the table with interrupts inhibited. */
    public void starEnter() {
        which.cutoff.run();
    }

    /** GOPOST (3): start POSTBURN (priority 12), set DAP for coasting flight, null the clock. */
    public void goPost() {
        if (exec != null) exec.findVac(012, () -> { /* POSTBURN */ });
        set("ALLCOAST_CALLED", 1);
        nullClok();
        phasChng(00134); // 4.13 restart for POSTBURN
        endOfJob();
    }

    /** GOCUTOFF (3): start CUTOFF (priority 17), permit displays, coast DAP, null clock, set 7.24 restart. */
    public void goCutoff() {
        if (exec != null) exec.findVac(017, () -> { /* CUTOFF */ });
        downFlag("FLUNDISP");
        set("ALLCOAST_CALLED", 1);
        nullClok();
        phasChng(07024);
        endOfJob();
    }

    /** IGNITE (2): if IGNFLAG not yet set schedule IGNITION in 10 ms with immediate restart; restore display. */
    public void ignite() {
        if ((g("FLAGWRD7") & g("IGNFLBIT")) == 0) {
            twiddle(1, this::ignition);
            set("PHASE4", 023); // OCT23 immediate restart at IGNITION
        }
        set("DISPDEX", Agc.fromSigned(-CNTDNDEX)); // IGNITE1: restore old display
        endOfJob();
    }

    /** P40ALM: alarm 1706, program selection not consistent with vehicle configuration. */
    public void p40Alm() {
        set("ALARM", ALARM_VEHICLE_CONFIG);
    }

    /** REP40ALM (14): V05N09 flash; V34 terminate, V33 proceed (check P42), V32 redisplay alarm. */
    public void rep40Alm() {
        int response = g("GOFLASH_RESPONSE"); // set by the flash routine: 0 terminate, 1 proceed, 2 redisplay
        if (response == 0) {
            set("GOTOPOOH", 1);
        } else if (response == 2) {
            rep40Alm();
        } else if (which.alt != null) {
            which.alt.run(); // for P42, allow crew to proceed though vehicle is unstaged
        }
    }

    private void p42Stage() { p40Alm(); }

    /**
     * P40AUTO: check PGNCS control and AUTO stabilisation (G+N,AUTO). If not, or if on the
     * descent stage not in AUTO-THROTTLE (CHAN30 bit 5), display V50N25 R1 = 203 (please perform
     * checklist 203) and recycle.
     */
    public void p40Auto() {
        while (true) {
            boolean notPgncsAuto = g("G+N,AUTO") > 0;
            boolean apsStage = (g("FLGWRD10") & g("APSFLBIT")) != 0;
            boolean autoThrottle = (g("CHAN30") & 0x10) == 0;
            if (!notPgncsAuto && (apsStage || autoThrottle)) {
                return; // GOBACK
            }
            set("GOPERF1", P40A_PMD); // V50N25 R1=203: perform checklist 203
            if (g("GOPERF1_RESPONSE") == 0) { // V34 terminate
                set("GOTOPOOH", 1);
                return;
            }
            // otherwise recycle P40A/P
            break;
        }
    }

    // ---------------------------------------------------------------- KILLTASK

    /**
     * KILLTASK: remove a task from the waitlist by substituting TASKOVER (NULLTASK). If the same
     * task is scheduled more than once only the one which occurs first is removed. If the task is
     * not scheduled nothing happens. Leaves interrupts inhibited (caller must RELINT).
     * The LST2 entries are scanned from the head; the original matches GENADR then FBANK.
     */
    public void killTask(String taskName) {
        if (waitlist == null) return;
        Runnable[] lst2 = waitlist.lst2;
        for (int l = 0; l < lst2.length && l < LSTLIM / 2; l++) {
            Runnable r = lst2[l];
            if (r != null && matchesTask(r, taskName)) {
                lst2[l] = waitlist::taskOver; // TCTSKOVR
                return;
            }
        }
    }

    private static boolean matchesTask(Runnable r, String taskName) {
        return r.toString().contains(taskName);
    }
}
