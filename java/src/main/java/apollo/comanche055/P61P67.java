package apollo.comanche055;

import apollo.agc.Agc;

/**
 * P61-P67: entry programs. P61 entry preparation, P62 CM/SM separation and attitude
 * maneuver, P63 entry initialization, P64 post .05G, P65 up control, P66 ballistic,
 * P67 final phase; plus S61.1 (state vector / IMU orientation check), S61.2 (entry
 * conic: GMAX, VPRED, GAMMAEI, RTGO, VIO, TTE), FISHCALC, VGAMCALC / PREVGAM, TFF/TRIG,
 * DISPTARG / VRCALC and S62.3 (desired .05G gimbal angles).
 *
 * Interpretive (vector) passages are summarised in comments with the numeric scalar
 * expressions in Java where practical; external routines are called via
 * {@link #extern(String, Object...)}.
 */
public class P61P67 {
    // ---- verb/noun codes ----
    public static final int V06N60 = 660;   // GMAX, VPRED, GAMMAEI
    public static final int V06N61 = 661;   // LAT(SPL), LNG(SPL), HEADSUP
    public static final int V06N63 = 663;   // RTGO, VIO, TTE
    public static final int V06N64 = 664;   // G, VI, R TO SPLSH
    public static final int V06N74 = 674;   // ROLLC, VI, D
    public static final int V16N69 = 1669;  // ROLLC, DL (Q7), VL
    public static final int V06N22 = 622;   // OGA, IGA, MGA
    public static final int V06N66 = 666;   // ROLLC, XRNGERR, DNRNGERR
    public static final int V16N67 = 1667;  // RTOGO, LAT, LONG
    public static final int V05N09 = 509;
    public static final int OCT41 = 33;     // decimal 33 (request separation)

    // ---- fixed constants ----
    public static final int BIT14 = 020000;
    /** C(30)LIM: 1.0 - .5 COS(30). */
    public static final double C30LIM = 0.566985;
    public static final int SECS10 = 1000;        // 10SECS, centiseconds
    public static final int SEC60_DP = 6000;      // 60SECDP, centiseconds
    public static final double M290KFT = 88392.0;           // B-29, meters
    /** KTETA1: 110 * 2PI / 16384 (163.84) B-14. */
    public static final double KTETA1 = 42.1844723;
    public static final double KFT36_PER_S = 109.728;       // B-7 m/cs
    public static final double KFT20_PER_S = 121.92;        // B-7 m/cs (2 x 20 KFT/S)
    public static final double KR1 = -0.026666667;         // -2.4 * 4 / 360
    public static final double MINUS_6_05DEG = -0.016805556;
    public static final double KR2 = 0.54931641;
    public static final double KR3 = 1000.0;                // 100 (10.0), B-14, g's
    public static final double RTRIAL = 6460097.18;         // B-29 meters (RPAD + 264643 ft)
    public static final double M400KFT = 121920.0;          // B-29
    public static final double KR4 = 0.833333333;
    public static final double VEMSCON = -0.0389676;        // -HS D / 2PI B-14
    public static final double DP2_M4 = 2.0 / 16.0;         // DP2(-4) = 2 B-4

    private final Agc agc;

    public P61P67(Agc agc) {
        this.agc = agc;
    }

    /** Hook for BANKCALL / PHASCHNG / interpretive subroutines; default does nothing. */
    protected int extern(String routine, Object... args) {
        return 0;
    }

    private void upflag(String f) {
        agc.set(f, 1);
    }

    private void downflag(String f) {
        agc.set(f, 0);
    }

    // ===================================================================================
    // P61 : ENTRY PREP
    // ===================================================================================

    /** P61: lock out extended verbs, preload HEADSUP, check SV and IMU, display N61. */
    public void p61() {
        agc.set("EXTVBACT", BIT14);           // lock out extended verbs so TFF routines can be used
        agc.set("HEADSUP", Agc.fromSigned(-1)); // preload
        s611();                               // check state vector and IMU orientation
        int rc = extern("GOFLASHR", V06N61);  // LAT(SPL) LNG(SPL) HEADSUP
        while (rc == 2) {                     // ENTER: recycle display
            rc = extern("GOFLASHR", V06N61);
        }
        if (rc == 0) {
            extern("GOTOPOOH");
            return;
        }
        p614();
    }

    /** P61.3: group 4 off, end job. */
    public void p613() {
        extern("PHASCHNG", 00014);
        extern("ENDOFJOB");
    }

    /** P61.4: ROLLC from HEADSUP (positive: 180 deg, lift down; negative: 0, lift up). */
    public void p614() {
        int rollc = Agc.toSigned(agc.get("HEADSUP")) > 0 ? BIT14 : 0;
        agc.set("ROLLC", rollc);
        agc.set("ROLLC+1", 0);
        newrnvn();
    }

    /** NEWRNVN: save time of RN,VN, initialise, compute URONE, VONE, UNI; S61.2; P61.1. */
    public void newrnvn() {
        agc.set("MM", agc.get("PIPTIME"));    // detect update
        extern("STARTEN1");                   // initialise
        // RONE = RN ; URONE = UNIT(RN) ; VONE = VN ; UNI = UNIT(URONE x VN)
        extern("VLOAD", "RN");
        extern("STORE", "RONE");
        extern("UNIT");
        extern("STORE", "URONE");
        extern("VLOAD", "VN");
        extern("STORE", "VONE");
        extern("VXV", "URONE", "UNI");
        // DUMPP61: has a state vector update occurred? (MM - PIPTIME)
        if (Agc.toSigned(agc.get("MM")) - Agc.toSigned(agc.get("PIPTIME")) < 0) {
            extern("CALRB");
            newrnvn();                        // updated... go try again
            return;
        }
        s612();                               // display data for N60 and N63
        p611();
    }

    /** P61.1: clear mark; display V06N60 (GMAX, VPRED, GAMMAEI). */
    public void p611() {
        extern("CLEARMRK");
        int rc = extern("GOFLASH", V06N60);
        if (rc == 0) {
            extern("GOTOPOOH");
            return;
        }
        while (rc == 2) {
            rc = extern("GOFLASH", V06N60);
        }
        p612();
    }

    /** P61.2: correct TTE for time lapse during the display; show N63. */
    public void p612() {
        // TTE = (LOADTIME - MM) + TTE1  (TTE1 is the negative of free fall time)
        int lapse = agc.get("LOADTIME") - agc.get("MM");
        agc.set("TTE", lapse + agc.get("TTE1"));
        int rc = extern("GOFLASH", V06N63);   // RTGO, VIO, TTE
        if (rc == 0) {
            extern("GOTOPOOH");
            return;
        }
        if (rc == 2) {
            p612();                           // REDO
            return;
        }
        // PROCEED falls through to P62 (mode change via V37 automatic)
    }

    // ===================================================================================
    // P62 : CM/SM SEPARATION AND ATTITUDE MANEUVER
    // ===================================================================================

    /** P62: NEWMODEX 62; DNLSTCOD = 1; check SV and IMU; compute .05G gimbal angles; start CM/POSE. */
    public void p62() {
        extern("NEWMODEX", 62);
        agc.set("DNLSTCOD", 1);
        s611();
        agc.set("POSEXIT", 0);                // SSP POSEXIT = P62.3 (no display)
        p623Silent();
        extern("CM/DAPIC");                   // start CM/POSE and body rate calc (returns to P62.2)
        p622();
    }

    private void p623Silent() {
        s623();
    }

    /** P62.2: continue CM/POSE after AVEG; request separation. */
    public void p622() {
        agc.set("AVEGEXIT", agc.get("POSECADR"));   // DCA POSECADR (2CADR CM/POSE)
        int rc = extern("GOPERF1R", OCT41);
        while (rc == 2) {                     // ENTER
            rc = extern("GOPERF1R", OCT41);
        }
        if (rc == 0) {
            extern("GOTOPOOH");
            return;
        }
        if (rc == 3) {
            p613();                           // phase change and ENDOFJOB
            return;
        }
        extern("CM/DAPON");                   // disable RCS DAP, enable entry DAP; attitude hold
        p621();
    }

    /** P62.1: display N61, begin maneuver to entry attitude, start P63. */
    public void p621() {
        int rc = extern("GOFLASH", V06N61);
        while (rc == 2) {
            rc = extern("GOFLASH", V06N61);
        }
        if (rc == 0) {
            p621();                           // TC -3
            return;
        }
        extern("PHASCHNG", 04024);            // use ENTRYVN for display
        agc.set("ROLLC", Agc.toSigned(agc.get("HEADSUP")) > 0 ? BIT14 : 0);
        agc.set("ALFACOM", agc.get("ALFAPAD"));   // ALFACOM = ALFA TRIM, BETACOM = 0
        agc.set("BETACOM", 0);
        agc.set("P63FLAG", 1);                // permits EXDAP2 to change flag to +0
        agc.set("ENTRYVN", V06N22);           // display CDU desired values
        upflag("ENTRYDSP");                   // bit 13 flag 6
        // Go directly to P63 if CMDAPMOD & 1 == 0 (delay task WAKEP62 omitted)
        if ((~agc.get("CMDAPMOD") & 1) == 0 || (agc.get("CMDAPMOD") & 1) == 0) {
            if ((agc.get("CMDAPMOD") & 1) == 0) {
                p631();                       // disable group 4, end job
                return;
            }
        }
        p63();
    }

    /** WAKEP62: wake-up task calling P63 at priority 13. */
    public void wakep62() {
        extern("NOVAC", 13, "P63");
        extern("TASKOVER");
    }

    /** P62.3: compute desired .05G gimbal angles, QPRET = ENDEXIT (entry display, group 5). */
    public void p623() {
        agc.set("QPRET", 1 /* ENDEXIT */);
        s623();
    }

    // ===================================================================================
    // P63 .. P67
    // ===================================================================================

    /** P63: entry initialization; continue at STARTENT after CM/POSE. */
    public void p63() {
        extern("NEWMODEX", 63);
        agc.set("POSEXIT", agc.get("ENTCADR"));       // STARTENT; later changed to SCALEPOP
        agc.set("ENTRYVN", V06N64);           // G, VI, R TO SPLSH
        agc.set("P63FLAG", Agc.fromSigned(-1)); // -1: exo-atmospheric DAP will not call P63 out of sequence
        extern("PHASCHNG", 00004);            // preserve CLEANDSP return on extended verb
        extern("CLEANDSP");                   // flush 'N22' display
        p631();
    }

    /** P63.1: disable group 4; display restarted via ENTRY; ENDOFJOB. */
    public void p631() {
        extern("PHASCHNG", 00004);
        extern("ENDOFJOB");
    }

    /** P64: entered via RTB when .05G is exceeded; N74 displayed via OVERNOUT. */
    public void p64() {
        extern("NEWMODEX", 64);
        agc.set("ENTRYVN", V06N74);           // ROLLC, VI, D
        extern("DANZIG");                     // continue in INITROLL
    }

    /** P65: entered via RTB when range < 25 NM of target. */
    public void p65() {
        extern("NEWMODEX", 65);
        extern("NOVAC", 13, "P65.1");
        extern("2PHSCHNG", 00554, 010035);    // 4.55 spot and servicer
        agc.set("GOTOADDR", 1 /* UPCONTRL */); // change entry mode to UPCONTRL
        extern("REFAZE10");                   // re-establish entry sequencer; continue at UPCONTRL
    }

    /** P65.1: flashing N69 (ROLLC, DL (Q7), VL). */
    public void p651() {
        downflag("ENTRYDSP");
        int rc = extern("GOFLASHR", V16N69);
        if (rc == 0) {
            p651();                           // TC -3 (NODOFLAG is set)
            return;
        }
        if (rc == 2) {
            p651();                           // TC -5
            return;
        }
        if (rc == 3) {
            p613();                           // establish group 4 for display and end job
            return;
        }
        upflag("ENTRYDSP");
        p631();                               // start entry display N06V68 via OVERNOUT
    }

    /** P66: entered via RTB when D < Q7 fps/s. */
    public void p66() {
        extern("NEWMODEX", 66);
        p66end(V06N22);                       // OGA, IGA, MGA; continue at KEP2
    }

    /** P67: entered via RTB. */
    public void p67() {
        extern("NEWMODEX", 67);
        p66end(V06N66);                       // ROLLC, XRNGERR, DNRNGERR
    }

    /** P66END: set ENTRYVN, turn on ENTRYDSP, kill group 4 and continue at PREDICT3. */
    public void p66end(int vn) {
        agc.set("ENTRYVN", vn);
        upflag("ENTRYDSP");
        killgrp4();
    }

    /** KILLGRP4: disable group 4 (called via RTB, returns to caller). */
    public void killgrp4() {
        extern("PHASCHNG", 00004);
        extern("DANZIG");                     // continue at PREDICT3
    }

    /** P67.1: final display N67; TERM -> turn off entry DAP and return to servicer. */
    public void p671() {
        int rc = extern("GOFLASH", V16N67);   // RTOGO, LAT, LONG
        if (rc == 2) {
            p671();                           // REDO
            return;
        }
        // CM/FLAGS &= ~3 : CM/DSTBY, GAMDIFSW off
        agc.set("CM/FLAGS", agc.get("CM/FLAGS") & ~3);
        agc.set("AVEGEXIT", agc.get("SERVCAD2")); // SERVEXIT
        extern("GOTOPOOH");
    }

    /** P67.2: present latitude, longitude, altitude (uses pad radius). */
    public void p672() {
        extern("VLOAD", "RN");
        downflag("ERADFLAG");
        extern("STORE", "ALPHAV");
        agc.set("TIME_OF_RN", agc.get("PIPTIME"));
        downflag("LUNAFLAG");
        extern("LAT-LONG");
        p673();
    }

    /** P67.3: entry exit that omits display. */
    public void p673() {
        extern("SERVNOUT");
    }

    // ===================================================================================
    // S61.1 : state vector and IMU orientation check
    // ===================================================================================

    /** S61.1: save return; R02BOTH; if AVERAGEG off, start via MIDTOAV2 and S61.1C; else S61.1A. */
    public void s611() {
        extern("R02BOTH");
        if (agc.get("AVEGFLAG") != 0) {
            s611a();
            return;
        }
        extern("MIDTOAV2");                   // future state vector as soon as possible; INHINT
        agc.set("S61DT", agc.get("MPAC+1"));
        extern("WAITLIST", "S61DT", "S61.1C");
        extern("PHASCHNG", 040434);
        extern("ENDOFJOB");
    }

    /** S61.1C: waitlist task starts job S61.1A-1 at priority 13; AVERAGEG exit = SERVEXIT. */
    public void s611c() {
        extern("FINDVAC", 13, "S61.1A-1");
        agc.set("AVEGEXIT", agc.get("SERVCAD1"));     // he who starts AVERAGEG must service the exit
        extern("2PHSCHNG", 00454, 00415);
        agc.set("EBANK", agc.get("EBENTRY"));          // set EB = 7 for PREREAD
        extern("PREREAD");                    // does TC TASKOVER
    }

    /** S61.1A: check -YSM orientation: unit(VN x RN) * REFSMMAT, test against 30 degrees. */
    public void s611a() {
        extern("TCDANZIG");                   // turn off OVFIND if on
        extern("VLOAD", "VN");
        extern("VXV", "RN");
        extern("MXV", "REFSMMAT");
        extern("UNIT");
        double cosHalf = agc.get("MPAC+3") / 16384.0;  // COS(THETA)/2
        if (cosHalf >= 0) {
            // cosHalf + C(30)LIM must not overflow
            if (cosHalf + C30LIM >= 1.0) {
                retrn3();
                return;
            }
        } else {
            // S61.1B: do test on -YSM: -cos + C(30)LIM
            if (-cosHalf + C30LIM >= 1.0) {
                retrn2();
                return;
            }
        }
        retrn1();
    }

    /** RETRN3: alarm 1426, IMU unsatisfactory. */
    private void retrn3() {
        extern("ALARM", 01426);
        retrn2a();
    }

    /** RETRN2: alarm 1427, IMU reversed. */
    private void retrn2() {
        extern("ALARM", 01427);
        retrn2a();
    }

    private void retrn2a() {
        extern("GODSPR", V05N09);
        extern("DELAYJOB", SECS10);
        retrn1();
    }

    /** RETRN1: return to caller (60GENRET). */
    private void retrn1() {
        // TC 60GENRET
    }

    // ===================================================================================
    // S61.2 : entry conic
    // ===================================================================================

    /**
     * S61.2: conic entry calculation. If EMSALT &gt; 290 KFT, uses 1/RTMU (earth orbital entry),
     * else 1/RTMUE (lunar entry); computes Fischer radius, TTE, RTGO, VIO, GMAX, VPRED, GAMMAEI.
     */
    public void s612() {
        double emsalt = agc.get("EMSALT");
        double mu = (emsalt - M290KFT) >= 0 ? agc.get("1/RTMU") : agc.get("1/RTMUE");
        agc.set("RTMU", (int) mu);
        callcon();
    }

    /** CALLCON: TFFCONIC; CALCTFF at RTRIAL; FISHCALC iterations; display data. */
    public void callcon() {
        extern("TFFCONIC");                   // fill VAC area with conic parameters
        extern("CALCTFF", RTRIAL);            // saves MPAC in RTERM
        extern("TFF/TRIG");                   // S(THETA) in MPAC
        extern("FISHCALC");                   // Fischer radius in MPAC and ERADM
        extern("CALCTFF", agc.get("ERADM") + agc.get("EMSALT"));
        int tte1 = -agc.get("MPAC");          // negative as in countdown
        agc.set("TTE1", tte1);
        agc.set("TTE", tte1);                 // TTE: time from now to EMSALT + FISCHER
        extern("TFF/TRIG");                   // THETA = range from now to EMSALT
        extern("FISHCALC");
        extern("VRCALC");
        disptarg();
        disptarg();
        agc.set("RTGO", agc.get("MPAC"));
        vgamcalc();                           // MPAC = GAMMA, PDL0 has VGAM
        // VIO = VGAM + (-HS D 180/PI)/GAMMA : predicted velocity at EMSALT
        double gamma = agc.get("MPAC");
        double vgam = agc.get("PDL0");
        agc.set("VIO", (int) (vgam + (gamma != 0 ? VEMSCON / gamma : 0)));
        agc.set("RTERM", agc.get("ERADM") + agc.get("MINPERE"));      // + 300 KFT
        extern("PREVGAM");                    // VGAMCALC with new RTERM
        // VBAR = (V(fps) - 36 KF/S) / 20 F/S ; VBARSQ to PDL0
        double vbar = (agc.get("PDL0") - KFT36_PER_S) / KFT20_PER_S;
        double vbarsq = vbar * vbar;
        double gam = agc.get("PDL2");         // GAM, positive down
        double num = KR1 * gam;               // see source: KR1*GAM + (-6.05 DEG) then KR2
        double den = (KR4 / vbarsq) + DP2_M4;
        double gmax = ((num + MINUS_6_05DEG) * KR2) / den + KR3;
        if (gmax < 0) {
            gmax = 0;                         // HI6ZEROS
        }
        agc.set("GMAX", (int) gmax);          // 100 GMAX (-14)
        extern("CALCTFF", agc.get("ERADM") + M400KFT);  // second iteration for Fischer radius
        extern("TFF/TRIG");
        extern("FISHCALC");
        agc.set("RTERM", agc.get("MPAC") + (int) M400KFT);
        extern("PREVGAM");
        agc.set("GAMMAEI", -agc.get("MPAC+1"));       // conic gamma/360 at 400K ft (hi word)
        agc.set("VPRED", agc.get("PDL0"));    // conic velocity at 400K ft
        extern("P61.1");
    }

    /** LUNENT: lunar-type entries use 1/RTMUE, then CALLCON. */
    public void lunent() {
        agc.set("RTMU", agc.get("1/RTMUE"));
        callcon();
    }

    /**
     * FISHCALC: URPR = UR CDELF + UHOR SDELF, URH = URPR, ALPHAV+4 = .5 SIN(LAT) (UNITW dot URH),
     * then GETERAD returns the Fischer radius in ERADM and MPAC.
     */
    public void fishcalc() {
        extern("VLOAD", "URONE");
        extern("VXV", "UNI");
        extern("VXSC", "SDELF/2");            // SIN(THETA)/2 from PDL+0
        extern("VLOAD", "URONE");
        extern("VXSC", "CDELF/2");            // COS(THETA)/2
        extern("VAD");
        extern("STORE", "URH");
        extern("DOT", "UNITW");               // unit north
        agc.set("ALPHAV+4", agc.get("MPAC") << 1);     // .5 SIN(LAT)
        extern("GETERAD");                    // saves Fischer radius in ERADM and MPAC
    }

    /** PREVGAM: scale RTERM by NR (shift left by -NR) and store in NRTERM; then VGAMCALC. */
    public void prevgam() {
        int nr = agc.get("NR");
        agc.set("NRTERM", agc.get("MPAC") << Math.max(0, -nr));
        vgamcalc();
    }

    /**
     * VGAMCALC: VGAM = sqrt(2 (RMAG - RTERM)/... - V^2/MU); gamma = ACOS(sqrt(P/RTERM)/ (RTERM VGAM/RTMU)).
     * Result: MPAC = GAMMA / 2, PDL0 = VGAM (m/cs). Caller supplies the sign.
     */
    public void vgamcalc() {
        double rmag = agc.get("NRMAG");
        double rterm = agc.get("NRTERM");
        double ratio = 2.0 * (rmag - rterm) / (rmag * rterm);   // 2(RN-RTERM)/(RN RTERM)
        double arg = ratio - agc.get("TFFVSQ");                 // -(V sq/MU)
        double vgam = Math.sqrt(Math.max(arg, 0));             // VGAM / sqrt(MU)
        agc.set("PDL0", (int) (vgam / agc.get("TFF/RTMU")));
        double num = rterm * vgam / agc.get("RTMU");           // RTERM VGAM / RTMU
        double den = Math.sqrt(agc.get("TFFNP") / rterm);       // LC P = H.H/MU
        double cos = Math.max(-1.0, Math.min(1.0, den != 0 ? num / den : 1.0)); // |COS| < 1.0 limiter
        agc.set("MPAC", (int) (Math.acos(cos) / 2));
    }

    /**
     * TFF/TRIG: .5 COS(THETA) = |HIDPHALF - P/(ALFA ...)|; .5 SIN(THETA) = sqrt(1/4 - COS^2) with the
     * sign of DELE/2 (TFFTEM). Returns with .5 SIN(THETA) in MPAC; CDELF/2 stored.
     */
    public void tffTrig() {
        double tffx = agc.get("TFFX") / 2.0;
        double p = (tffx + agc.get("HIDPHALF")) * agc.get("NRMAG") * agc.get("NRTERM");
        double tem = agc.get("TFFTEM");
        double q = tem != 0 ? p / tem : 0;
        double cdelf2 = Math.abs(agc.get("HIDPHALF") - Math.abs(q));
        agc.set("CDELF/2", (int) cdelf2);
        double sdelf2 = Math.sqrt(Math.max(0, agc.get("HIDP1/4") - cdelf2 * cdelf2));
        agc.set("MPAC", (int) (tem < 0 ? -sdelf2 : sdelf2));  // SIGN TFFTEM
    }

    /** DISPTARG: MPAC = TRGO estimate; DTEAROT = KTETA1 * TTE1 - MPAC; EARROT2; VRCALC. */
    public void disptarg() {
        int trgo = agc.get("MPAC");
        agc.set("DTEAROT", (int) (KTETA1 * agc.get("TTE1")) - trgo);
        extern("EARROT2");
        vrcalc();
    }

    /** VRCALC: MPAC = 4 * ACOS(URH . RT) (angle between URH and RT). */
    public void vrcalc() {
        extern("VLOAD", "URH");
        extern("DOT", "RT");
        agc.set("MPAC", agc.get("MPAC") << 2);
        extern("ACOS");
    }

    // ===================================================================================
    // S62.3 : desired .05G gimbal angles
    // ===================================================================================

    /**
     * S62.3: build the entry-attitude body axes in reference coordinates from ALFAPAD (negative alfa
     * trim) and ROLLC about UXA, UYA, UZA, then CALCGA with CPHIFLAG cleared so angles go to CPHI.
     * <pre>
     * YNB = (cos ROLLC * UYA/2 + sin ROLLC * UZA/2) * 2
     * XNB = (cos TRIM * UXA/2 + sin TRIM * (YNB x UXA/2)) * 2 (sign per PDL)
     * ZNB = XNB x YNB
     * </pre>
     */
    public void s623() {
        double alfa = agc.get("ALFAPAD") / 2.0;       // ALFATRIM / 180
        double cosTrim = Math.cos(alfa * Math.PI);
        double sinTrim = Math.sin(alfa * Math.PI);
        double roll = agc.get("ROLLC") / 16384.0 * Math.PI * 2;
        double cosRoll = Math.cos(roll);
        double sinRoll = Math.sin(roll);
        // YNB = UYD (reference coordinates)
        extern("VXSC", "UYA/2", cosRoll);
        extern("VXSC", "UZA/2", sinRoll);
        extern("VAD");
        extern("VSL1");
        extern("STORE", "YNB");
        extern("VXV", "UXA/2");
        extern("VSL1");
        extern("VXSC", sinTrim);
        extern("VXSC", "UXA/2", cosTrim);
        extern("VAD");
        extern("VSL1");
        extern("STORE", "XNB");               // x spacecraft axis (.5 unit)
        extern("VXV", "YNB");
        extern("VSL1");
        extern("STOVL", "ZNB");               // z spacecraft axis, scaled at 2
        extern("STOVL", "XSM", "REFSMMAT");
        extern("STOVL", "YSM", "REFSMMAT+6");
        extern("STORE", "ZSM", "REFSMMAT+12D");
        downflag("CPHIFLAG");                 // CALCGA stores answer in TP CPHI
        extern("CALCGA");                     // returns to original caller via QPRET with 2's comp angles
    }
}
