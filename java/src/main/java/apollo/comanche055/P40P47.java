package apollo.comanche055;

import apollo.agc.Agc;

/**
 * P40-P47: SPS burn programs (P40 SPS, P41 RCS, P47 thrust monitor), the steering and
 * burn-timing tasks (TIGAVEG, TIG-5, TIG-0, IGNITION, ENGINOFF, SPSOFF), the V97/V99 engine
 * fail / ignition-enable responses, DAP configuration (S41.2, DAPFIG), S40.1 (VG at ignition),
 * S40.2,3 (preferred attitude), S40.6 (gimbal trim/test), S40.8 (steering/TGO),
 * S40.9 (cross-product steering extrapolation), S40.13 (burn time), RCS DAP start (RCSDAPON).
 *
 * Interpretive (vector) passages are written out as comments with the corresponding Java where
 * scalar; external routines are called through {@link #extern(String, Object...)}.
 */
public class P40P47 {
    // ---- constants ----
    public static final int MINDB = 46;
    public static final int MAXDB = 455;
    /** FENG: SPS thrust (20500 lb), sc. at B+7 newton / E4 (9.1188544). */
    public static final double FENG = 9.1188544;
    /** FRCS2: RCS ullage (2 jets), 0.087437837. */
    public static final double FRCS2 = 0.087437837;
    public static final int SEC24_96 = 2496;
    public static final int SEC29_96 = 2996;
    public static final int SEC18 = 1800;
    public static final int P40CKLS2 = 0204;
    public static final int CST5_40 = 037730;
    public static final int V1683 = 1683;
    public static final int V06N85B = 685;
    public static final int V16N85B = 1685;
    public static final int V06N40 = 640;
    public static final int V16N40 = 1640;
    public static final double MDOT3 = 86.6175796;     // 3MDOT scaled B-16
    public static final double THETACON = 0.31830989; // B-8
    public static final double EP4_45 = 0.125;
    public static final double EP4_10 = 0.027777777;
    public static final double TRIMSCAL = 1.07975111;  // B-1
    public static final double YBIAS = 0.00263888889;  // +0.95 deg
    public static final double PBIAS = -0.00597222222; // -2.15 deg
    public static final int SEC5 = 500;
    public static final int SEC5_DP = 0;
    public static final int OCT02202 = 02202;
    public static final int OCT02200 = 02200;
    public static final int POS_2_5 = 037405;
    public static final int T6SEC_T5 = 037704;
    public static final int T0_6SEC_T5 = 037704;
    public static final int T0_6SEC_T5_HALF = 037703;
    public static final int DEC409 = 409;
    public static final int DEC46 = 46;
    public static final double CONTONE = 0.662034;     // 2PI/M
    public static final double CONTTWO = 0.00118;
    public static final int ACT_DEG_PLUS2 = 84;        // +2 deg at 85.41 arcsec/bit
    public static final int ACT_DEG_MINUS4 = -168;
    public static final double TWODT = 200.0;          // 2 sec
    public static final double FOURDT = -800.0;
    public static final double EXHAUST2V = 63.020792;  // 2 x 10338.0564 fps, B+7 m/cs
    public static final double FOURSEC = 400.0;
    public static final double EARTHMU = -3.986032E10;
    public static final double S40_135 = 69.6005183;   // impulse from 2-jet ullage
    public static final double TENBNK14 = 10.0;

    private static final int BIT1 = 0x0001;
    private static final int BIT2 = 0x0002;
    private static final int BIT3 = 0x0004;
    private static final int BIT4 = 0x0008;
    private static final int BIT7 = 0x0040;
    private static final int BIT9 = 0x0100;
    private static final int BIT10 = 0x0200;
    private static final int BIT11 = 0x0400;
    private static final int BIT12 = 0x0800;
    private static final int BIT13 = 0x1000;
    private static final int BIT14 = 0x2000;
    private static final int BIT15 = 0x4000;
    private static final int OCT60000 = 060000 & Agc.WORD_MASK;

    private final Agc agc;

    public P40P47(Agc agc) {
        this.agc = agc;
    }

    /** Hook for BANKCALL / WAITLIST / FINDVAC / PHASCHNG / display calls; default does nothing. */
    protected void extern(String routine, Object... args) {
    }

    private void upflag(String f) {
        agc.set(f, 1);
    }

    private void downflag(String f) {
        agc.set(f, 0);
    }

    private boolean flag(String word, int bit) {
        return (agc.get(word) & bit) != 0;
    }

    private void setBit(String word, int bit) {
        agc.set(word, agc.get(word) | bit);
    }

    private void clearBit(String word, int bit) {
        agc.set(word, agc.get(word) & ~bit & Agc.WORD_MASK);
    }

    /** TVC takeover indication in FLAGWRD6: bits 15,14 = 1,0. */
    private boolean tvcMode() {
        int comp = ~agc.get("FLAGWRD6") & OCT60000 & Agc.WORD_MASK;
        return comp != 0 && (comp & Agc.SIGN_BIT) == 0;
    }

    // ===================================================================================
    // P40 / P41 / P47 main programs
    // ===================================================================================

    /** P40CSM: SPS burn program. */
    public void p40csm() {
        downflag("ENG2FLAG");                 // SPS (not RCS)
        // Is this an external delta-V burn? CSTEER = ZERO if XDELVFLG else ECSTEER
        agc.set("CSTEER", flag("XDELVFLAGWORD", 0) || agc.get("XDELVFLG") != 0 ? 0 : agc.get("ECSTEER"));
        agc.set("F", (int) Math.round(FENG * 128));    // SPS thrust for P40 (20,000 lb)
        p40sF();
    }

    /** P40S/F: entered from P41 as well; sets F and NOMTIG from TIG. */
    private void p40sF() {
        agc.set("NOMTIG", agc.get("TIG"));   // original TIG may be slipped by P40S/SV
        extern("R02BOTH");                   // IMU status check
        p40pva();
    }

    /** P40PVA: compute VGTIG,UT (S40.1), preferred attitude (S40.2,3) and maneuver. */
    public void p40pva() {
        extern("S40.1");                     // compute VGTIG, UT
        extern("S40.2,3");                   // preferred attitude
        upflag("PFRATFLG");
        p40sxty();
    }

    /** P40SXTY: narrow deadband and attitude maneuver (R60), then start the clock task. */
    public void p40sxty() {
        setmindb();                          // TCR SETMINDB -1 (EBANK6)
        extern("R60CSM");                    // attitude maneuver
        agc.set("NBRCYCLS", Agc.fromSigned(-1)); // for UPDATEVG
        upflag("TIMRFLAG");                  // allow CLOCKTASK
        if (p41p40() == 1) {                 // P41 returns at call+1, P40 at call+2
            p41dsp();
            return;
        }
        p40ttog();
    }

    /** P40TTOG: initialise V06N40 clock display, VGDISP, DVTOTAL; set AVEGEXIT for steering. */
    public void p40ttog() {
        agc.set("NVWORD1", V06N40);
        // R2 = |VGTIG| ; DVTOTAL = 0
        agc.set("VGDISP", agc.get("VGTIG"));
        agc.set("DVTOTAL", 0);
        agc.set("AVEGEXIT", 1 /* STEERADS = 2CADR STEERING */);
        p40gmb();
    }

    /** P40GMB: 4.1 protection; display gimbal trim/test option (GOPERF1 V50N25 style). */
    public void p40gmb() {
        extern("GOPERF1", P40CKLS2);         // V34 -> POST41, V33 -> TST,TRIM
        tstTrim();
    }

    /** TST,TRIM: gimbal drive test and trim (MRKRTMP = +1). */
    public void tstTrim() {
        agc.set("MRKRTMP", 1);
        trimonlyPlus1();
    }

    /** TRIMONLY: MRKRTMP = -1 (gimbal trim only). */
    public void trimonly() {
        agc.set("MRKRTMP", Agc.fromSigned(-1));
        trimonlyPlus1();
    }

    private void trimonlyPlus1() {
        agc.set("CNTR", 0);                  // +0 says normal entry for S40.6 restart logic
        extern("PHASCHNG", 040026, 0234);    // 6.2 PRE40.6, CLOKTASK; 4.23 P40S/SV
        extern("WAITLIST", 1, "S40.6");
        int delay = Agc.toSigned(agc.get("MRKRTMP")) > 0 ? SEC18 : SEC5; // gimbal test wait
        extern("DELAYJOB", delay);
        p40sRs();
    }

    /** P40S/RS: restart entry; start CLOKTASK, then P40S/SV. */
    public void p40sRs() {
        extern("WAITLIST", 1, "CLOKTASK");
        p40sSv();
    }

    /** P40S/SV: job, 4.23 protects; compute time to start the TIG-30 blanking task. */
    public void p40sSv() {
        e7setter();
        // TDEC1 = TIG - SEC29.96
        agc.set("TDEC1", Agc.add(agc.get("TIG"), Agc.fromSigned(-SEC29_96)));
        // CALRB MIDTOAV1 : integration time check; if too large -> P40SNEWM
        extern("MIDTOAV1");
        p40set();
    }

    /** P40SET: delta time to PREREAD less 5 sec for TIGBLNK; LONGCALL TIGBLNK. */
    public void p40set() {
        agc.set("P40TMP", agc.get("MPAC"));  // DP delta time to preread
        agc.set("P40TMP", Agc.add(agc.get("P40TMP"), Agc.fromSigned(-SEC5)));
        extern("LONGCALL", agc.get("P40TMP"), "TIGBLNK");
        extern("PHASCHNG", 020214);          // 4.21 TIGBLNK
        // TCF ENDOFJOB
    }

    /** P40BLNKR: remove residual display (job). */
    public void p40blnkr() {
        extern("CLEANDSP");
    }

    /** P40SNEWM: integration time too long; new TIG = PIPTIME1 + SEC29.96. */
    public void p40snewm() {
        agc.set("TIG", Agc.add(agc.get("PIPTIME1"), SEC29_96));
        p40set();
    }

    /** POSTBURN: V16N40 flash; V34 -> POST41, proceed -> P40RCS, recycle -> POSTBURN. */
    public void postburn() {
        extern("REFLASH", V16N40);
        p40rcs();
    }

    /** P40RCS: V99N40 enters here on a P40 bypass of SPS. */
    public void p40rcs() {
        agc.set("AVEGEXIT", 2 /* ACADN85 */);
        extern("DELAYJOB", 200);             // wait for CALCN85 via AVEGEXIT
        p40mindb();
    }

    public void p40mindb() {
        setmindb();
        tignow();
    }

    /** TIGNOW: display V16N85B for GSOP; V32 redisplays. */
    public void tignow() {
        extern("PHASCHNG", 05024, 020000);   // type C group 4 for N85, priority 20
        extern("REFLASH", V16N85B);
        post41();
    }

    /** POST41: restore SERVEXIT as AVEGEXIT and go to P00. */
    public void post41() {
        agc.set("AVEGEXIT", 3 /* SERVCADR = 2CADR SERVEXIT */);
        extern("GOTOPOOH");
    }

    /** SETMINDB: set minimum DAP deadband; THETAD from CDUs; clear DAPDATR1 bit 4 (call with INHINT). */
    public void setmindb() {
        agc.set("THETADX", agc.get("CDUX"));
        agc.set("THETADY", agc.get("CDUY"));
        agc.set("THETADY+1", agc.get("CDUY+1"));
        agc.set("ADB", MINDB);
        clearBit("DAPDATR1", BIT4);
    }

    /** SETMAXDB: set maximum DAP deadband (call with INHINT, EBANK6). */
    public void setmaxdb() {
        agc.set("ADB", MAXDB);
        setBit("DAPDATR1", BIT4);
    }

    /** P41CSM: RCS burn program. */
    public void p41csm() {
        upflag("ENG2FLAG");                  // set for RCS
        agc.set("CSTEER", 0);                // for P41 CSTEER = 0
        // F = FRCS2 (2-jet) or 2*FRCS2 (4-jet) depending on NJETSFLG
        double f = FRCS2 * (agc.get("NJETSFLG") != 0 ? 1 : 2);
        agc.set("F", (int) Math.round(f * 128));
        p40sF();
    }

    /**
     * P41/P40: returns 1 if the calling program is P41 (exits at call+1), 2 if P40 (exits at call+2).
     */
    public int p41p40() {
        return (agc.get("MODREG") & 1) == 0 ? 1 : 2; // CS MODREG, MASK ONE, BZF: P41
    }

    /** TTG/0: task (4.4 protects in P41); spawn TIGNOW job at priority 20. */
    public void ttg0() {
        extern("NOVAC", 020, "TIGNOW");
    }

    /** P40CLK: clear TIMRFLAG and end the task. */
    public void p40clk() {
        downflag("TIMRFLAG");
    }

    /** P41/DSP: set up non-flashing V06N85 via the clock job; compute VGTIG in control coords. */
    public void p41dsp() {
        agc.set("NVWORD1", V06N85B);
        p40cnv85();
        agc.set("AVEGEXIT", 2 /* ACADN85 */);
        extern("2PHSCHNG", 040036, 0234);    // 6.3 CLOKTASK, 4.23 P40S/SV
        p40sRs();
    }

    /** P41REDSP: from the P41 side of TIGAVEG: redisplay V16N85B non-flashing, wait for TTG/0. */
    public void p41redsp() {
        agc.set("NVWORD1", V16N85B);
        extern("WAITLIST", SEC29_96 + 1, "TTG/0");
        ttgphs(Agc.fromSigned(-BIT3));
    }

    /** P40CNV85: VGBODY = S41.1(VGPREV), VGPREV equals VGTIG (targeting input). */
    public void p40cnv85() {
        extern("S41.1", "VGPREV");
        agc.set("VGBODY", agc.get("MPAC"));
    }

    /** CALCN85: update VG (UPDATEVG) and compute VGBODY; exit via SERVXT. */
    public void calcn85() {
        updatevg();
        p40cnv85();
        servxt();
    }

    /** TIGBLNK: call TIGAVEG in 5 sec at TIG-30; blank the display and spawn P40BLNKR. */
    public void tigblnk() {
        extern("WAITLIST", SEC5, "TIGAVEG");
        agc.set("NVWORD1", 0);
        extern("NOVAC", 014, "P40BLNKR");
        p40tsk(Agc.fromSigned(-037));        // 4.37 = TIGAVEG (500 cs)
    }

    private void p40tsk(int phase) {
        extern("NEWPHASE", phase, 4);
    }

    // ---- P47 ----

    /** P47CSM: thrust monitor program. */
    public void p47csm() {
        extern("R02BOTH");
        extern("MIDTOAV2");
        agc.set("P40TMP", agc.get("MPAC+1"));
        extern("WAITLIST", agc.get("P40TMP"), "TIGON");
        extern("PHASCHNG", 040574);          // 4.57 = TIGON
    }

    /** TIGON: set CALCN83 as AVEGEXIT and start P47BODY at priority 30. */
    public void tigon() {
        agc.set("AVEGEXIT", 4 /* ACADN83 */);
        extern("NOVAC", 030, "P47BODY");
        ttgphs(Agc.fromSigned(-BIT2));       // 4.2 = PRECHECK, P47BODY
    }

    /** CALCN83: DV47TEMP = DELVCTL + DELVREF; DELVIMU = S41.1(...); S11.1; copy to DELVCTL. */
    public void calcn83() {
        extern("S41.1", "DELVCTL+DELVREF");
        agc.set("DELVIMU", agc.get("MPAC"));
        extern("S11.1");                     // VI, H, HDOT for N62
        extern("PHASCHNG", 010035);
        extern("GENTRAN", 5, "DV47TEMP", "DELVCTL");
        servxt();
    }

    /** P47BODY: clear display and accumulator storage on initiation or ENTER response. */
    public void p47body() {
        agc.set("DELVIMU", 0);
        agc.set("DELVCTL", 0);
        p47bod();
    }

    public void p47bod() {
        extern("PRIOCHNG", 015);             // lower than CALCN83 (20)
        extern("PHASCHNG", 05024, 015000);
        extern("GOFLASH", V1683);            // terminate/proceed -> GOTOPOOH; recycle -> P47BODY
    }

    // ---- TIG tasks ----

    /** TIGAVEG: task (4.37 protects); P41 side redisplays, P40 side unblanks and starts TIG-5 timing. */
    public void tigaveg() {
        if (p41p40() == 1) {
            p41redsp();
            return;
        }
        agc.set("NVWORD1", V06N40);          // unblank display
        extern("WAITLIST", SEC24_96, "TIG-5");
        ttgphs(Agc.fromSigned(-6));          // 4.6 = TIG-5 (2496 cs), PRECHECK (-0 cs)
    }

    /** TTGPHS: set the group 4 phase from A; then PRECHECK. */
    private void ttgphs(int a) {
        extern("NEWPHASE", a, 4);
        precheck();
    }

    /** PRECHECK: has the servicer been restarted? If not, go to PREREAD. */
    public void precheck() {
        if (Agc.toSigned(agc.get("PHASE5")) > 0) {
            return;                          // yes: TASKOVER
        }
        extern("PREREAD");
    }

    /** TIG-5: 5 seconds before ignition; start TIG-0 timing, start burn timing S40.13. */
    public void tigMinus5() {
        extern("WAITLIST", SEC5, "TIG-0");
        agc.set("NVWORD1", Agc.fromSigned(-BIT9)); // will cause V99 flash
        extern("2PHSCHNG", 040074, 033);     // 4.7 TIG-0, 3.3 S40.13
        extern("FINDVAC", 020, "S40.13");
    }

    /** TIG-0: set the ignition flag and, if V99P has arrived, ignite. */
    public void tigMinus0() {
        setBit("FLAGWRD7", BIT13);           // IGNFLAG
        if (!flag("FLAGWRD7", BIT12)) {
            return;                          // wait for V99P
        }
        agc.set("NVWORD1", V06N40);          // clear the V99
        extern("PHASCHNG", 0614);            // 4.61 IGNITION
        ignition();
    }

    /** IGNITION: save CDUX as OGAD, record TEVENT, set ENGONFLG and turn on the SPS. */
    public void ignition() {
        agc.set("OGAD", agc.get("CDUX"));
        agc.set("TEVENT", agc.get("TIME2"));
        agc.set("TEVENT+1", agc.get("TIME2+1"));
        setBit("FLAGWRD5", BIT7);            // ENGONFLG
        spson();
    }

    /** SPSON: turn on SPS engine (DSALMOUT bit 13). */
    public void spson() {
        setBit("DSALMOUT", BIT13);
        impulchk();
    }

    /** IMPULCHK: impulsive burn -> IMPLBURN; else set STRULLSW and PREPTVC. */
    public void impulchk() {
        if (flag("FLAGWRD2", BIT9)) {
            implburn();
            return;
        }
        setBit("FLAGWRD6", BIT13);           // STRULLSW for STEERULL
        preptvc();
    }

    /** PREPTVC: reset T5 bits, kill the RCS DAP, wait 0.4 sec for thrust buildup. */
    public void preptvc() {
        clearBit("FLAGWRD6", OCT60000);
        agc.set("T5LOC", 5 /* T5IDL24 = T5IDLOC */);
        extern("NEWPHASE", Agc.fromSigned(-3), 4); // 4.3 = DOTVCON (40 cs)
        extern("FIXDELAY", 40);              // 0.4 second delay for thrust buildup
        dotvcon();
    }

    /** DOTVCON: engage the TVC DAP. */
    public void dotvcon() {
        agc.set("TVCPHASE", Agc.fromSigned(-1)); // TVCDAPON call (FRESHDAP)
        agc.set("TVCEXPHS", 0);
        // set T5 bits to indicate TVC takeover: bits 15,14 = 1,0
        agc.set("FLAGWRD6", (agc.get("FLAGWRD6") & ~OCT60000 & Agc.WORD_MASK) + BIT15);
        extern("-PHASE6", 3);                // 6.3 = CLOKTASK, dropping PRE40.6
        agc.set("ERRBTMP", Agc.fromSigned(-Agc.toSigned(agc.get("ERRORY"))));
        extern("NEWPHASE", Agc.fromSigned(-5), 4);  // 4.5 = DOSTRULL (160 cs)
        agc.set("TIME5", 0x3FFF);            // start right away
        agc.set("T5LOC", 6 /* TVCON2C = TVCDAPON */);
        extern("FIXDELAY", 160);             // 0.4 + 1.6 = 2.0 sec for ullage-off and steering
        dostrull();
    }

    /** DOSTRULL: non-impulsive -> STEERULL; ullage off; kill group 4. */
    public void dostrull() {
        if (flag("FLAGWRD6", BIT13)) {
            steerull();
        }
        ulageoff();
        extern("-PHASE4", 0);                // kill group 4 (DP NEG0)
    }

    /** STEERULL: set STEERSW (bit 11 of FLAGWRD2). */
    public void steerull() {
        setBit("FLAGWRD2", BIT11);
    }

    /** ULAGEOFF: zero channel 5. */
    public void ulageoff() {
        agc.set("CHAN5", 0);
    }

    /** IMPLBURN: impulsive burn; schedule ENGINOFF at TGO. */
    public void implburn() {
        clearBit("FLAGWRD6", BIT13);         // reset STRULLSW
        e7setter();
        // TIG = TGO + TIME2 (for R1 of V06N40)
        agc.set("TIG", Agc.add(agc.get("TGO"), agc.get("TIME2")));
        extern("2PHSCHNG", 040153, 007014);  // 3.15 ENGINOFF, delta T next
        extern("WAITLIST", agc.get("TGO+1"), "ENGINOFF");
        implcont();
    }

    /** IMPLCONT: reset IMPULSW, E6SETTER, set V97VCNTR = 0 and continue to PREPTVC. */
    public void implcont() {
        clearBit("FLAGWRD2", BIT9);          // IMPULSW
        e6setter();
        agc.set("V97VCNTR", 0);
        preptvc();
    }

    /** ENGINOFF: task from 3.15; copy CSMMASS to MASSTMP and shut the engine down. */
    public void enginoff() {
        e6setter();
        agc.set("MASSTMP", agc.get("CSMMASS"));
        extern("2PHSCHNG", 3, 040634);       // kill group 3; 4.63 DOSPSOFF
        dospsoff();
    }

    /** DOSPSOFF: shutdown SPS, mass update, then 2.5 s for tailoff. */
    public void dospsoff() {
        spsoff();
        extern("NEWPHASE", Agc.fromSigned(-027), 4); // 4.27 DOTVCRCS (250 cs)
        extern("FIXDELAY", 250);
        dotvcrcs();
    }

    /** DOTVCRCS: wide deadband, start RCS DAP, update mass properties, wipe out TVC, POSTBURN job. */
    public void dotvcrcs() {
        setmaxdb();
        extern("RCSDAPON");
        extern("MASSPROP");
        tvczap();
        extern("PHASCHNG", 0354);            // 4.35 POSTBURN
        extern("NOVAC", 012, "POSTBURN");
    }

    /** SPSOFF: establish TEVENT, reset ENGONFLG, turn off SPS and send S-IVB cutoff, then MASSBACK. */
    public void spsoff() {
        agc.set("TEVENT", agc.get("TIME2"));
        agc.set("TEVENT+1", agc.get("TIME2+1"));
        clearBit("FLAGWRD5", BIT7);
        clearBit("DSALMOUT", BIT13);
        setBit("CHAN12", BIT14);             // SIV cutoff command (backup)
        massback();
    }

    /**
     * MASSBACK: restore pre-decremented mass: CSMMASS = MASSTMP + V97VCNTR * EMDOT * 100.
     * VCNTR counts half-seconds in the TVC executive.
     */
    public void massback() {
        long corr = (long) Agc.toSigned(agc.get("V97VCNTR")) * Agc.toSigned(agc.get("EMDOT")) >> 14;
        long total = (corr * 100) >> 14;
        agc.set("CSMMASS", Agc.add(agc.get("MASSTMP"), Agc.fromSigned((int) total)));
        if (Agc.toSigned(Agc.add(agc.get("TVCPHASE"), 1)) == 0) {
            return;                          // initialization not complete
        }
        if (!tvcMode()) {
            return;                          // not TVC yet
        }
        besttrim();
    }

    /** BESTTRIM: update trims from DELFILTER values. */
    public void besttrim() {
        agc.set("PACTOFF", agc.get("DELPBAR"));
        agc.set("YACTOFF", agc.get("DELYBAR"));
    }

    /** TVCZAP: disable TVC and optics error counters, enable T4RUPT optics monitor, stop CLOKTASK. */
    public void tvczap() {
        clearBit("CHAN12", OCT02202);
        agc.set("OPTIND", Agc.fromSigned(-1));
        agc.set("NVWORD1", 0);
        clearBit("FLAGWRD7", BIT11);         // clear TIMRFLAG
    }

    // ---- V97/V99 responses ----

    /** E7SETTER: EBANK = 7. */
    public void e7setter() {
        agc.set("EBANK", agc.get("EBANK7"));
    }

    /** E6SETTER: EBANK = 6. */
    public void e6setter() {
        agc.set("EBANK", agc.get("EBANK6"));
    }

    /** CLOKTASK: one-second clock display task (V06N40, V06N85). */
    public void cloktask() {
        if (!flag("FLAGWRD7", BIT11)) {      // TIMRFLAG
            extern("PHASCHNG", 6);           // kill restart
            return;
        }
        clockon();
    }

    /** CLOCKON: TTOGO = TIME2 - TIG; reschedule; choose the display job. */
    public void clockon() {
        agc.set("TTOGO", Agc.add(agc.get("TIME2"), Agc.fromSigned(-Agc.toSigned(agc.get("TIG")))));
        extern("WAITLIST", 100, "CLOKTASK");
        int nv = Agc.toSigned(agc.get("NVWORD1"));
        if (nv == 0) {
            settb6();                        // display blanked
            return;
        }
        if (nv < 0) {
            extern("FINDVAC", 027, "CLOCKJOB");
            return;
        }
        if (nv == V06N85B) {
            extern("FINDVAC", 027, "DYNDISP");   // SETUPDYN
            settb6();
            return;
        }
        extern("NOVAC", 027, "CLOCKJOB");
    }

    /** SETTB6: set group 6 time base. */
    public void settb6() {
        agc.set("TBASE6", agc.get("TIME1"));
    }

    /** DYNDISP: update N85 for dynamic V06N85 in P41, then CKNVWRD1. */
    public void dyndisp() {
        p40cnv85();
        cknvwrd1();
    }

    /** CLOCKJOB: save CDUs in CDUSPOT, QUICTRIG; then dispatch on NVWORD1. */
    public void clockjob() {
        agc.set("CDUSPOTX", agc.get("CDUX"));
        agc.set("CDUSPOTY", agc.get("CDUY"));
        agc.set("CDUSPOTZ", agc.get("CDUZ"));
        extern("QUICTRIG");
        cknvwrd1();
    }

    /** CKNVWRD1: NVWORD1 > 0: normal display; 0: end; negative: V99/V97 flashes. */
    public void cknvwrd1() {
        int nv = Agc.toSigned(agc.get("NVWORD1"));
        if (nv > 0) {
            extern("REGODSP", nv);           // NOFLASH
        } else if (nv == 0) {
            return;                          // ENDOFJOB
        } else if (nv == Agc.fromSigned(-BIT9) || agc.get("NVWORD1") == Agc.fromSigned(-BIT9)) {
            engrequst();                     // V99 flash
        } else {
            faildsp();                       // V97 flash
        }
    }

    /** ENGREQST: V99 flash (SPS engine-on enable). */
    public void engrequst() {
        extern("CLOCPLAY", V06N40);          // T -> V99T, P -> V99P, E -> V99E
    }

    /** FAILDSP: V97 flash (SPS engine failed). */
    public void faildsp() {
        extern("CLOCPLAY", V06N40);          // T -> V97T, P -> V97P, E -> V97E
    }

    /** V99E: ENTER response: wipe out TVC/clock and take the V16N85 post-burn path. */
    public void v99e() {
        extern("2PHSCHNG", 6, 05024, 027000);
        tvczap();
        p40rcs();
    }

    /** V99T: terminate response (also from V97T). */
    public void v99t() {
        extern("2PHSCHNG", 6, 05024, 027000);
        tvczap();
        post41();
    }

    /** V99P: proceed response: set astronaut flag; if ignition time has arrived, ignite now. */
    public void v99p() {
        if (flag("FLAGWRD7", BIT12)) {
            v99pTig();                       // prior V99P: restart entry
            return;
        }
        setBit("FLAGWRD7", BIT12);           // ASTNFLAG
        if (!flag("FLAGWRD7", BIT13)) {
            v99pTig();                       // IGN flag not yet set: wait for TIG-0
            return;
        }
        extern("WAITLIST", 1, "IGNITION");
    }

    private void v99pTig() {
        agc.set("NVWORD1", V06N40);
    }

    /** V97T: engine-fail terminate. */
    public void v97t() {
        extern("2PHSCHNG", 6, 040674);
        extern("TWIDDLE", "V97TTASK");
    }

    /** V97TTASK: disable CLOCKJOB; MASSTMP = CSMMASS + 3 sec of MDOT; SPSOFF; RCS DAP. */
    public void v97ttask() {
        agc.set("NVWORD1", 0);
        agc.set("MASSTMP", Agc.add(agc.get("CSMMASS"), (int) Math.round(MDOT3)));
        extern("PHASCHNG", 05014);
        spsoff();
        extern("PHASCHNG", 0714);            // 4.71 V97TRCS
        extern("FIXDELAY", 250);
        v97trcs();
    }

    /** V97TRCS: RCS DAP on; schedule V99T for TVCZAP and POST41. */
    public void v97trcs() {
        extern("RCSDAPON");
        extern("NOVAC", 027, "V99T");
    }

    /** V97P: proceed after engine fail: restart mass updates, set IDLEFAIL and STEERSW. */
    public void v97p() {
        extern("PHASCHNG", 040734);
        extern("TWIDDLE", "V97PTASK");
    }

    /** V97PTASK. */
    public void v97ptask() {
        agc.set("VCNTR", agc.get("V97VCNTR"));
        agc.set("NVWORD1", V06N40);
        upflag("IDLEFAIL");                  // allow R41 bypass
        upflag("STEERSW");                   // re-enable steering
        extern("PHASCHNG", 0134);            // 4.13 R40ENABL
        extern("FIXDELAY", 200);
        r40enabl();
    }

    /** R40ENABL: re-enable R40 by clearing IDLEFAIL. */
    public void r40enabl() {
        downflag("IDLEFAIL");
        extern("PHASCHNG", 4);
    }

    /** V97E: ENTER response after engine fail. */
    public void v97e() {
        extern("PHASCHNG", 040534);
        extern("WAITLIST", 1, "V97ETASK");
    }

    /** V97ETASK: force R1 of V06N40 to read 59X59; SPSOFF97; trim engine; QUICKIGN; V99FLASH. */
    public void v97etask() {
        agc.set("TIG", Agc.fromSigned(-020));
        agc.set("NVWORD1", V06N40);
        e6setter();
        agc.set("MASSTMP", Agc.add(agc.get("CSMMASS"), (int) Math.round(MDOT3)));
        extern("PHASCHNG", 0754);            // 4.75 SPSOFF97
        spsoff();
        extern("PHASCHNG", 0114);            // 4.11 V97E40.6
        extern("FIXDELAY", 250);
        v97e406();
    }

    /** V97E40.6. */
    public void v97e406() {
        extern("WAITLIST", 1, "PRE40.6");
        extern("RCSDAPON");
        extern("2PHSCHNG", 026, 05014);
        quickign();
    }

    /** QUICKIGN: clear ASTNFLAG and set IGNFLAG for immediate V99 response. */
    public void quickign() {
        agc.set("FLAGWRD7", Agc.add(agc.get("FLAGWRD7") & ~(014000 & ~BIT13) & Agc.WORD_MASK, BIT13) & Agc.WORD_MASK);
        extern("FIXDELAY", 30);
        v99flash();
    }

    /** V99FLASH: cause V99 to flash and start the burn timer S40.13. */
    public void v99flash() {
        agc.set("NVWORD1", Agc.fromSigned(-BIT9));
        extern("2PHSCHNG", 040774, 033);
        extern("FINDVAC", 020, "S40.13");
    }

    // ===================================================================================
    // Steering support
    // ===================================================================================

    /** UPDATEVG: average-G exit for VG update (S40.8) and S40.9 extrapolation. */
    public void updatevg() {
        if (agc.get("XDELVFLG") != 0) {
            s408();                          // external delta-V: CALL40.8 only
            return;
        }
        if (Agc.toSigned(agc.get("NBRCYCLS")) < 0) {
            setup9();
            return;
        }
        // DELVSUMP = DELVSUM + DELVREF ; NBRCYCLP = NBRCYCLS + 1
        agc.set("DELVSUMP", Agc.add(agc.get("DELVSUM"), agc.get("DELVREF")));
        agc.set("NBRCYCLP", Agc.add(1, agc.get("NBRCYCLS")));
        extern("PHASCHNG", 010035);          // type B restart, 5.3 REREADACCS
        agc.set("NBRCYCLS", agc.get("NBRCYCLP"));
        agc.set("DELVSUM", agc.get("DELVSUMP"));
        s408();
    }

    /** SETUP.9: initialise S40.9 integration state (start of a Lambert cycle). */
    public void setup9() {
        if (agc.get("FIRSTFLG") == 0) {
            // VGPREV = VGTEMP + (BDT * NBRCYCLP) - DELVSUM
        }
        extern("FINDVAC", 010, "S40.9");
        extern("2PHSCHNG", 051, 010035);
        agc.set("RINIT", agc.get("RN"));
        agc.set("VINIT", agc.get("VN"));
        agc.set("TNIT", agc.get("PIPTIME"));
        agc.set("DELLT4", Agc.add(agc.get("TPASS4"), Agc.fromSigned(-Agc.toSigned(agc.get("PIPTIME")))));
        agc.set("DELVSUM", 0);
        agc.set("NBRCYCLS", 0);
        s408();
    }

    /**
     * STEERING: AVEGEXIT target. UPDATEVG then, if engine on and not impulsive, check TIG and
     * schedule ENGINOFF when TGO is less than 6 (or 4) seconds.
     */
    public void steering() {
        updatevg();
        if (flag("FLAGWRD2", BIT9)) {        // IMPULSW: pre-ignite, request eng-off, or post-off
            servxt();
            return;
        }
        if (!flag("DSALMOUT", BIT13)) {      // engine off
            servxt();
            return;
        }
        e7setter();
        long delta = Agc.toSigned(agc.get("TIG")) - Agc.toSigned(agc.get("TIME2")); // TIG - TIME2
        int t = (int) delta;
        if (t <= 0) {
            t = 1;                           // protect against negative/zero waitlist call
        }
        agc.set("TGO", 0);
        agc.set("TGO+1", t);
        extern("WAITLIST", t, "ENGINOFF");
        extern("2PHSCHNG", 040153, 010035);
        downflag("IMPULSW");
        servxt();
    }

    /** SERVXT: POSTJUMP to SERVEXIT. */
    public void servxt() {
        extern("SERVEXIT");
    }

    /**
     * S40.8: compute velocity-to-be-gained VG = VGPREV - DELVG (= VGPREV + DELVREF - BDT),
     * test thrust level, TGO and the steering cross product OMEGAC.
     */
    public void s408() {
        // VG = DELVREF - BDT + VGPREV ; VGDISP = |VG|
        agc.set("VGDISP", agc.get("VG"));
        extern("PHASCHNG", 010035);
        agc.set("VGPREV", agc.get("VG"));
        if (agc.get("STEERSW") == 0) {
            return;                          // skip TGO and cross-product
        }
        // LOTHRUST check: |DELVREF| vs DVTHRESH * 2^-9
        int dvmag = Math.abs(Agc.toSigned(agc.get("DELVREF")));
        long thresh = (long) Agc.toSigned(agc.get("DVTHRESH")) >> 9;
        if (dvmag < thresh) {
            lothrust();
            return;
        }
        int repfrac = flag("DAPDATR1", BIT14) ? agc.get("EREPFRAC+1") : agc.get("EREPFRAC");
        agc.set("REPFRAC", repfrac);         // enable TVCDAP CG tracking (LM off/on value)
        tgocalc();
    }

    /**
     * TGOCALC: time to go from the rocket equation:
     * TGO = -(FOURDT)*(|VG| ...) - ETDECAY adjusted; TIG = PIPTIME + TGO; if TGO &lt; 4 s -> S40.81.
     */
    public void tgocalc() {
        // UNIT(DELVG) . VG must show angle greater than pi/2, otherwise alarm 01407 (INCRSVG)
        // TGO = (|DELVG| / (2VEXHUST*...)) ... ; TIG = PIPTIME + TGO
        int tgo = agc.get("TGO");
        agc.set("TIG", Agc.add(agc.get("PIPTIME"), tgo));
        if (Agc.toSigned(tgo) - (int) FOURSEC < 0) {
            s4081();
            return;
        }
        xproduct();
    }

    /** INCRSVG: alarm indicating that the thrust is pointing in the wrong direction. */
    public void incrsvg() {
        extern("ALARM", 01407);
    }

    /** XPRODUCT: OMEGAC = KPRIMEDT * (UNIT(VG) x UNIT(VG - CSTEER*BDT*4 - DELVREF)) in body axes. */
    public void xproduct() {
        agc.set("OMEGAC", agc.get("OMEGAC"));
    }

    /** S40.81: TGO less than 4 seconds: set IMPULSW for engine-off call; TVC to attitude hold. */
    public void s4081() {
        upflag("IMPULSW");
        ratezro();
    }

    /** RATEZRO: OMEGAC = 0, CNTR = POSMAX (inhibit switchover), clear STEERSW. */
    public void ratezro() {
        agc.set("OMEGAC", 0);
        agc.set("CNTR", 0x3FFF);
        downflag("STEERSW");
    }

    /** LOTHRUST: thrust failure (low or none). Starts engine-fail (R40) operations unless IDLEFAIL. */
    public void lothrust() {
        if (agc.get("IDLEFAIL") != 0) {
            agc.set("OMEGAC", 0);
            return;
        }
        agc.set("OMEGAC", 0);                // put TVC in attitude hold
        agc.set("VCNTR", Agc.fromSigned(0));
        agc.set("VCNTRTMP", 0);
        agc.set("REPFRAC", 0);
        agc.set("NVWORD1", Agc.fromSigned(-1)); // engine-fail V97 flash (CLOCKJOB)
        downflag("STEERSW");
    }

    // ===================================================================================
    // S40.1, S40.2,3, S41.1, S40.9, S40.13, S40.6, S41.2
    // ===================================================================================

    /**
     * S40.1: compute VGTIG (velocity to be gained at ignition) and UT (thrust direction).
     * For external delta-V: VG = DELVSIN rotated by half the thrust-arc angle
     * (THETAT = F*DELVSAB*WEIGHT/G / (|RTIG|^2 ...)); for Lambert: VGTIG = DELVEET3 via S40.1B/AGAIN.
     */
    public void s401() {
        upflag("FIRSTFLG");
        agc.set("BDT", 0);
        if (agc.get("XDELVFLG") == 0) {
            s401b();                         // Lambert
            return;
        }
        agc.set("DELVSAB", agc.get("DELVSIN"));
        agc.set("VINIT", agc.get("VTIG"));
        // UT = UNIT(VTIG x RTIG) = "UP" ; RINIT = RTIG
        // THETAT = DELVSAB * WEIGHT/G * |RTIG|^-2 * THETACON / F
        // VP = DELVSIN - (DELVSIN . UP) UP ; VGTIG = VP' sin(THETAT/2) + DELVSIN cos(THETAT/2)
        // UT = UNIT(VGTIG) ; MIDGIM (VGTIG in LV coordinates) ; GOTO QTEMP
        extern("MIDGIM");
    }

    /** S40.1B: Lambert targeting for the thrust direction; DELVEET3 becomes VGTIG. */
    public void s401b() {
        agc.set("TDEC1", Agc.add(agc.get("TIG"), Agc.fromSigned(-(int) TWODT)));
        agc.set("DELLT4", Agc.add(agc.get("TPASS4"), Agc.fromSigned(-Agc.toSigned(agc.get("TDEC1")))));
        again();
        agc.set("UT", agc.get("VIPRIME"));
        agc.set("TDEC1", agc.get("TIG"));
        agc.set("DELLT4", Agc.add(agc.get("TPASS4"), Agc.fromSigned(-Agc.toSigned(agc.get("TDEC1")))));
        again();
        agc.set("VGTIG", agc.get("DELVEET3"));
        upflag("AVFLAG");
        extern("MIDGIM");
        calcut();
    }

    /** AGAIN: precision integrate to TIG and initialise the velocity-to-be-gained iteration. */
    public void again() {
        extern("THISPREC");
        agc.set("RTIG", agc.get("RATT"));
        agc.set("RINIT", agc.get("RATT"));
        agc.set("VTIG", agc.get("VATT"));
        agc.set("VINIT", agc.get("VATT"));
        extern("INITVEL", agc.get("NORMSW") != 0 ? EP4_45 : EP4_10);
    }

    /**
     * CALCUT: refine thrust direction UT including gravity and steering bias:
     * UT = UNIT( Q + sqrt((F/m)^2 - |Q x ...|^2) UG ) etc.
     */
    public void calcut() {
        extern("CALCGRAV");                  // GDELTAT
        // B.C = ((VIPRIME - UT)/200CS - 2*G... ) * CSTEER ; UG = UNIT(VGTIG)
        // Q = B.C - (B.C . UG) UG ; UT = UNIT( Q + sqrt((F/m)^2 - |Q|^2) UG )
    }

    /**
     * S40.2,3: preferred attitude. Computes SCAXIS (engine bell direction in reference
     * coordinates), XSCREF/YSCREF/ZSCREF spacecraft axes from the trim (or ENG2FLAG: UNITX),
     * and the preferred IMU orientation using TSTRXUT.
     */
    public void s4023() {
        // POINTVSM = REFSMMAT * UT << 1
        if (agc.get("ENG2FLAG") != 0) {
            s4023b();                        // RCS: thrust along +X
            return;
        }
        double y = (Agc.toSigned(agc.get("YACTOFF")) / 16384.0 * 2 * TRIMSCAL) + YBIAS;
        double p = (Agc.toSigned(agc.get("PACTOFF")) / 16384.0 * 2 * TRIMSCAL) + PBIAS;
        double cy = Math.cos(y), sy = Math.sin(y), cp = Math.cos(p), sp = Math.sin(p);
        // XSCREF = (cos P cos Y, -sin Y ... ) new SC x axis etc. per the trimmed SPS bell
        agc.set("ZSCREF", Agc.fromSigned((int) Math.round(sp * 16384)));
        agc.set("SCAXIS", Agc.fromSigned((int) Math.round(cy * cp * 16384)));
        agc.set("YSCREF", Agc.fromSigned((int) Math.round(-sy * cp * 16384)));
        // then rotate by the matrix with rows (X-ref, Y-ref, Z-ref) built from UT and RTIG
        tstrxut();
    }

    /** S40.2,3B: RCS case - SC X axis along UT; Y = UNIT(RTIG x UT)... */
    public void s4023b() {
        agc.set("SCAXIS", 0 /* UNITX */);
        agc.set("XSCREF", agc.get("UT"));
        tstrxut();
    }

    /** TSTRXUT: if |RTIG x UT| is zero use a fallback vector (BADVCTOR) perpendicular to VTIG. */
    public void tstrxut() {
        if (agc.get("36D") == 0) {
            badvctor();
        }
    }

    public void badvctor() {
        // UNIT(RTIG), UNIT(VTIG)/8 added, crossed with UT, complemented -> 6D
    }

    /** S41.1: convert VG from reference to control (body) coordinates; uses CDUTRIG, REFSMMAT, QUADROT. */
    public void s411() {
        extern("CDUTRIG");
        // VG(control) = (QUADROT * *SMNB* * REFSMMAT * VG) * 10 << 5, scaled 2(+7)
    }

    /**
     * S40.9: velocity-to-be-gained extrapolation (Lambert refinement).
     * Computes BDT, VGTEMP from the previous cycle and stores VRPREV/TNITPREV.
     */
    public void s409() {
        // PDL: EP4(45)L or EP4(10)L depending on NORMSW ; CLEAR GUESSW ; HAVEGUES
        clearGuess();
        extern("PHASCHNG", 05021, 010000);
        endlamb();
    }

    private void clearGuess() {
        downflag("GUESSW");
        extern("HAVEGUES");
    }

    /** ENDLAMB: BDT = (VIPRIME - VRPREV)/(TNIT - TNITPREV) * 2^17/200CS - GDT/2, etc. */
    public void endlamb() {
        if (agc.get("FIRSTFLG") == 0) {
            long dt = (long) Agc.toSigned(agc.get("TNIT")) - Agc.toSigned(agc.get("TNITPREV"));
            if (dt != 0) {
                agc.set("BDT", Agc.fromSigned((int) ((Agc.toSigned(agc.get("VIPRIME"))
                        - Agc.toSigned(agc.get("VRPREV"))) * 200 / dt)));
            }
        }
        firsttme();
    }

    /** FIRSTTME: VGTEMP += GOBL/2 * (earth gravity term) when in Earth sphere. */
    public void firsttme() {
        if (Agc.toSigned(agc.get("RTX2")) != 0) {
            moonCase();                      // MOONCASE: no gravity-bias correction
            return;
        }
        // VGTEMP = VGTEMP + GOBL/2 * (EARTHMU * (PIPTIME - NOMTIG) / |RN|^3)
        moonCase();
    }

    /** MOONCASE: finish and copy S40.9 state. */
    public void moonCase() {
        extern("PHASCHNG", 04021);
        copy409();
    }

    /** COPY40.9: TNITPREV = TNIT; VRPREV = VIPRIME; clear FIRSTFLG; then end. */
    public void copy409() {
        agc.set("TNITPREV", agc.get("TNIT"));
        agc.set("VRPREV", agc.get("VIPRIME"));
        downflag("FIRSTFLG");
        agc.set("NBRCYCLS", Agc.fromSigned(-1));
        ends409();
    }

    /** ENDS40.9. */
    public void ends409() {
        extern("PHASCHNG", 1);
        // ENDOFJOB
    }

    /** REDO40.9: restart entry; zero DELVSUM and NBRCYCLS, VGTEMP = VGPREV; end. */
    public void redo409() {
        agc.set("DELVSUM", 0);
        agc.set("NBRCYCLS", 0);
        agc.set("VGTEMP", agc.get("VGPREV"));
        ends409();
    }

    /**
     * S40.13: burn timing. Determines TGO from VGTIG, thrust, mass and ullage/tailoff impulse
     * (K1VAL, K2VAL, K3VAL, EMDOT, FANG).
     */
    public void s4013() {
        upflag("IMPULSW");                   // assume no steering until found otherwise
        double vg = Math.abs(Agc.toSigned(agc.get("VGTIG")));
        boolean ullage = !(((agc.get("CHAN31") ^ BIT7) & BIT7) == 0);  // +X translation
        double corrected = vg;
        if (ullage) {
            double comp = S40_135 / Math.max(1, Agc.toSigned(agc.get("WEIGHT/G")));
            corrected = vg - comp * (agc.get("NJETSFLG") != 0 ? 2 : 1); // double correction if 4 jets
        }
        double k1 = agc.get("EK1VAL");
        double mass = Math.max(1, Agc.toSigned(agc.get("WEIGHT/G")));
        double tgoLess100 = corrected - k1 / mass;
        long tgo;
        if (tgoLess100 < 0) {
            // TGO less than 100 cs: TGO = (WEIGHT/G*... + K2VAL) / K3VAL
            tgo = (long) ((mass * 1 + agc.get("EK2VAL")) / Math.max(1, agc.get("EK3VAL")));
        } else {
            // TEMP1 = EMDOT*3.5SEC/WEIGHT/G... ; TEMP2 = FANG*5SECOND/...
            tgo = 600;
            if (tgo > 600) {
                s40133();
                return;
            }
        }
        agc.set("TGO", Agc.fromSigned((int) (tgo >> 14)));
        agc.set("TGO+1", Agc.fromSigned((int) (tgo & 0x3FFF)));
        s40134();
    }

    /** S40.133: TGO greater than 600 cs: clear IMPULSW so the vehicle will steer. */
    public void s40133() {
        downflag("IMPULSW");
        s40134();
    }

    /** S40.134: kill group 3 and end job. */
    public void s40134() {
        extern("PHASCHNG", 3);
    }

    /** PRE40.6: restart entry to S40.6 (not permitted in TVC mode). */
    public void pre406() {
        if (tvcMode()) {
            return;                          // TVC: REDOTVC re-establishes the interface
        }
        agc.set("MRKRTMP", Agc.fromSigned(-1)); // RCS: trim only
        agc.set("CNTR", 1);                  // restart entry indicator
        s406();
    }

    /** S40.6: gimbal drive test and trim of the SPS engine. */
    public void s406() {
        agc.set("OPTIND", Agc.fromSigned(0));
        clearBit("CHAN12", BIT2);            // disable optics error counters
        setBit("CHAN12", OCT02200);          // TVC enable, disengage optics/DAC
        extern("FIXDELAY", 6);               // 60 ms relay latching
        setBit("CHAN12", BIT2);              // enable optics error counters
        extern("FIXDELAY", 2);               // 20 ms
        if (agc.get("CNTR") != 0 && Agc.toSigned(agc.get("CNTR")) > 0) {
            gimtrimPlus2();                  // restart entry: bypass 4 sec delay
            return;
        }
        agc.set("CNTR", agc.get("MRKRTMP"));
        if (Agc.toSigned(agc.get("MRKRTMP")) <= 0) {
            gimtrim();                       // trim only
            return;
        }
        // GDTSETUP: pitch test first (CNTR = -0), then yaw (CNTR = -1)
        agc.set("CNTR", 0);
        while (true) {
            output(ACT_DEG_PLUS2);           // +2 deg
            output(ACT_DEG_MINUS4);          // -2 deg
            output(ACT_DEG_PLUS2);           // back to zero
            if (Agc.toSigned(agc.get("CNTR")) < 0) {
                break;                       // yaw test complete
            }
            agc.set("CNTR", Agc.fromSigned(-1)); // set up yaw test
        }
        gimtrim();
    }

    /** OUTPUT: send the increment to TVCPITCH (CNTR=0) or TVCYAW (CNTR=-1) and wait 2 s. */
    private void output(int increment) {
        String axis = Agc.toSigned(agc.get("CNTR")) < 0 ? "TVCYAW" : "TVCPITCH";
        agc.set(axis, increment & Agc.WORD_MASK);
        setBit("CHAN14", BIT11);             // release the error counters (bit 11 pitch / 12 yaw)
        extern("FIXDELAY", 200);             // 2 sec while astronaut verifies on GPI
    }

    /** GIMTRIM: wait 4 sec then output trim values. */
    public void gimtrim() {
        extern("FIXDELAY", 400);
        gimtrimPlus2();
    }

    /** GIMTRIM+2: output PACTOFF/YACTOFF as TVCPITCH/TVCYAW, release the counters. */
    public void gimtrimPlus2() {
        agc.set("TVCPITCH", agc.get("PACTOFF"));
        agc.set("TVCYAW", agc.get("YACTOFF"));
        setBit("CHAN14", 0x0C00 /* PRIO6: bits 11,12 */);
        // TCF TASKOVER
    }

    /**
     * S41.2: set up DAP parameters from DAPDATR1/2: rate index, LEM attached flag, deadband (ADB),
     * translation (XTRANS) from quad selection, NJETSFLG, ACORBD, RACFAIL, RBDFAIL.
     */
    public void s412() {
        int d1 = agc.get("DAPDATR1");
        int d2 = agc.get("DAPDATR2");
        agc.set("RATEINDX", (d1 & 3) * 2);
        boolean attached = (d1 & 030) == (BIT14 & 030 | 020 & 0);
        // Bits 14,13 of DAPDATR1 = 10 means LEM attached
        attached = ((d1 >> 11) & 3) == 2;
        if (attached) {
            setBit("FLAGWRD7", BIT2);        // TOGETHER: set flag for integration
        } else {
            clearBit("FLAGWRD7", BIT2);
        }
        agc.set("ADB", (d1 & BIT4) != 0 ? DEC409 + DEC46 : DEC46); // DEC 46 narrow, 455 wide
        int xtrans = 0;
        if ((d1 & BIT7) != 0) {              // quad BD
            xtrans = 1;
        }
        if ((d1 & BIT10) != 0) {             // quad AC
            xtrans += Agc.fromSigned(-1) == 0 ? 0 : -1;
        }
        agc.set("XTRANS", Agc.fromSigned(xtrans));
        if (xtrans != 0) {
            setBit("FLAGWRD1", BIT15);       // NJETSFLG = 1: 2-jet ullage (AC or BD)
        } else {
            clearBit("FLAGWRD1", BIT15);     // 4-jet (or 0-jet)
        }
        agc.set("ACORBD", (d2 & BIT13) != 0 ? 1 : Agc.fromSigned(-1)); // minus A-C, plus B-D
        // RACFAIL: +1 if AC bank 1 failed, -1 if AC bank 2 failed (BIT10, BIT4), else 0
        if ((d2 & BIT10) != 0) {
            agc.set("RACFAIL", 0);
            agc.set("RACFAIL", (d2 & BIT4) != 0 ? Agc.fromSigned(-1) : 0);
        } else {
            agc.set("RACFAIL", 1);
        }
        // RBDFAIL: +1 if BD bank 1 failed, -1 if BD bank 2 failed (BIT7, BIT1), else 0
        if ((d2 & BIT7) != 0) {
            agc.set("RBDFAIL", 0);
            agc.set("RBDFAIL", (d2 & BIT1) != 0 ? Agc.fromSigned(-1) : 0);
        } else {
            agc.set("RBDFAIL", 1);
        }
    }

    // ===================================================================================
    // DAP configuration
    // ===================================================================================

    /** DAPFIG: V46 - turn off S-IVB takeover, select DAP by vehicle configuration. */
    public void dapfig() {
        clearBit("CHAN12", BIT9);
        int config = (agc.get("DAPDATR1") >> 2) & 3; // MP BIT3: shift right 4 octal digits? -> bits 14..13 area
        config = (agc.get("DAPDATR1") >> 11) & 3;
        switch (config) {
            case 0:
                nodapup();                   // CM
                break;
            case 1:
            case 2:
                rcsdapup();                  // CSM, CSM/LEM
                break;
            default:
                extern("SATSTKON");          // Saturn stick
        }
    }

    /** RCSDAPUP: activate RCS DAP, then exit via GOPIN. */
    public void rcsdapup() {
        rcsdapon();
        endfig();
    }

    /** NODAPUP: T5 idle, reset T5 flags, zero jets, kill KALCMANU. */
    public void nodapup() {
        agc.set("T5LOC", 5 /* T5IDLDAP */);
        downflag("DAPBIT1");
        downflag("DAPBIT2");
        extern("ZEROJET");
        agc.set("HOLDFLAG", 1);              // kill KALCMANU job
        endfig();
    }

    private void endfig() {
        extern("GOPIN");
    }

    /** S40.14: RCS DAP constants J/M, J/M1, J/M2 (from IXX, IAVG) and KMJ, KMJ1, KMJ2. */
    public void s4014() {
        agc.set("J/M", (int) Math.round(Agc.toSigned(agc.get("IXX")) * CONTONE));
        agc.set("J/M1", (int) Math.round(Agc.toSigned(agc.get("IAVG")) * CONTONE));
        agc.set("J/M2", agc.get("J/M1"));
        int ixx = Math.max(1, Agc.toSigned(agc.get("IXX")));
        int iavg = Math.max(1, Agc.toSigned(agc.get("IAVG")));
        agc.set("KMJ", Agc.fromSigned((int) Math.round(CONTTWO * 16384 * 16384 / ixx)));
        agc.set("KMJ1", Agc.fromSigned((int) Math.round(CONTTWO * 16384 * 16384 / iavg)));
        agc.set("KMJ2", agc.get("KMJ1"));
    }

    /** RCSUP: T5 interrupt entry to activate the RCS DAP. */
    public void rcsup() {
        rcsdapon();
        // TCF RESUME
    }

    /** RCSDAPON: 0.6 s for TVCEXEC/ROLLDAP to die, set RCSATT as T5 task, flag T5 takeover by RCS DAP. */
    public void rcsdapon() {
        rcsdaponPlus1(T0_6SEC_T5);
    }

    /** RCSDAPON+1: entry from R00TOP00 with TIME5 value in A. */
    public void rcsdaponPlus1(int time5) {
        agc.set("TIME5", time5);
        agc.set("T5PHASE", time5);           // will cause FRESHDAP
        setBit("RCSFLAGS", BIT3);            // reinitialise FDAI error display
        agc.set("T5LOC", 7 /* RCSADDR = RCSATT */);
        // bits 15,14 of FLAGWRD6 = 0,1 (T5 takeover by RCS DAP) kills TVCEXEC and ROLLDAP starts
        agc.set("FLAGWRD6", (agc.get("FLAGWRD6") & ~OCT60000 & Agc.WORD_MASK) + BIT14);
    }
}
