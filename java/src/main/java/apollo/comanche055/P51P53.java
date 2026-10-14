package apollo.comanche055;

import apollo.agc.Agc;

/**
 * P51-P53: IMU orientation determination (P51), IMU realign (P52), backup IMU orientation (P53),
 * with R51 (fine align), R52 (automatic optics positioning), R53 (sighting mark), R54 (star
 * data test), R55 (gyro torque), R56 (alternate LOS sighting mark), S50 (LOCSAM planet/sun/moon
 * ephemeris), PICAPAR (star selection), S52.2 / S52.3 (gimbal angles / nominal orientation),
 * SR52.1 (optics trunnion/shaft check), CALCSMSC, CAL53A, GYCRS and helpers.
 *
 * Interpretive passages that are vector/matrix arithmetic are expressed through
 * {@link #extern(String, Object...)} calls (named by the original label) with the original
 * operation summarised in comments; control flow is translated to Java.
 */
public class P51P53 {
    // ---- constants (fixed memory) ----
    public static final double RSUBM = 1738090.0;     // moon radius in meters (B-29)
    public static final double RSUBE = 6378166.0;     // earth radius in meters (B-29)
    public static final double FIVE_DEGREES = 0.013888889; // scaled in revs
    public static final double ONE_OVER_C = 0.000042699;   // 1/C, B-1
    public static final double[] ECLIPOL = {0.0, -0.00007896, 0.00018209}; // CSM only, B-1
    public static final int TSIGHT1 = 24000;
    public static final double CSS5 = 0.2490475;      // (COS 5)/4
    public static final double CSSUN = 0.24148;       // (COS 15)/4
    public static final double SIN33 = 0.5376381241;
    public static final double COS33 = 0.8431756920;
    public static final double CSS66 = 0.060480472;   // (COS76)/4
    public static final double CSS6640 = -0.15602587; // (COS76 - COS30)/4
    public static final double CSS33 = 0.197002688;   // (COS(1/2(76))/4
    public static final double[] SHAXIS = {0.5376381241, 0.0, 0.8431766920}; // B-1
    public static final double TRUN38 = 0.66666667;   // 50 degrees in trunnion
    public static final int SEC1_3_DP = 130;
    public static final int MINUS_20DEG_SMN = -7199;
    public static final double DP1_6 = 0.16666666;
    public static final double MPERIOD = 0.047619;    // approx lunar rotation angle in 2 hrs x 16
    public static final double ONE_OVER_SQRT3 = 0.57735021;
    public static final int DEGREE1 = 46;
    public static final int DEG359 = 16338;
    public static final int DEC227 = 227;
    public static final int NEG47 = 077730;
    public static final double ONE_SIXTH = 0.1666667;
    public static final int SEC1_8 = 180;
    public static final int OCT404 = 0404;
    public static final int OCT66 = 066;

    // verb/noun codes
    public static final int V06N34 = 0634;
    public static final int VB06N22 = 0622;
    public static final int V16N20 = 1620;
    public static final int V06N89 = 689;
    public static final int V06N92 = 692;
    public static final int V01N70 = 0170;
    public static final int V06N93 = 693;
    public static final int VB6N5 = 605;
    public static final int V6N22 = 0622;
    public static final int V41K = 4100;
    public static final int V01N71 = 0171;
    public static final int V06N94B = 694;
    public static final int V05N09 = 509;
    public static final int VB53 = 05300;
    public static final int VNPLANV = 688;

    private static final int BIT1 = 1, BIT2 = 2, BIT4 = 8, BIT5 = 020, BIT6 = 040, BIT7 = 0100,
            BIT9 = 0400, BIT10 = 01000, BIT11 = 02000, BIT15 = 040000;

    private final Agc agc;

    public P51P53(Agc agc) {
        this.agc = agc;
    }

    /** Hook for BANKCALL / PHASCHNG / GOFLASH / interpretive subroutines; default does nothing. */
    protected int extern(String routine, Object... args) {
        return 0;
    }

    private void upflag(String f) {
        agc.set(f, 1);
    }

    private void downflag(String f) {
        agc.set(f, 0);
    }

    private boolean stateBit(int word, int bit) {
        return (agc.get(word == 0 ? "STATE" : "STATE" + word) & bit) != 0;
    }

    // ===================================================================================
    // P52 : IMU REALIGN  (P54 shares the entry)
    // ===================================================================================

    /** PROG52 / P54: IMU realign. OPTION2 = 1 PREF, 2 NORM, 3 REF? (1 = LS selection, see P52T). */
    public void prog52() {
        extern("PHASCHNG", 00254);
        downflag("UPDATFLG");                 // bit 7 flag 1
        downflag("TRACKFLG");                 // bit 5 flag 1
        extern("R02BOTH");                    // IMU status check
        boolean pfrat = (agc.get("STATE2") & BIT4) != 0;   // is PFRATFLG set?
        agc.set("OPTION2", pfrat ? BIT1 : BIT2);          // P52A: yes -> 1, no -> 2
        p52b();
    }

    /** P52B: flash option code and orientation code. */
    public void p52b() {
        agc.set("OPTION2", agc.get("OPTION2"));
        int rc = extern("GOPERF4R", BIT1);    // 0 TERMINATE, 1 PROCEED, 2 NEW CODE
        if (rc == 0) {
            extern("GOTOPOOH");
            return;
        }
        if (rc == 2) {
            p52b();                           // new code -- new orientation code input
            return;
        }
        extern("PHASCHNG", 00014);
        switch (agc.get("OPTION2") & 3) {
            case 0:                           // (not reachable) falls to L.S.
            case 1:
                p52t();                       // L.S.
                break;
            case 2:
                p52j();                       // PREF
                break;
            default:
                p52c();                       // REF
        }
        // NOTE: the original dispatch is INDEX A / TC +1 with entries L.S., PREF, NORM, REF
    }

    /** P52T: request time of sighting (V06N34), then compute nominal / LS orientation. */
    public void p52t() {
        agc.set("DSPTEM1", Agc.fromSigned(-0));        // DCA NEG0
        agc.set("DSPTEM1+1", Agc.fromSigned(-0));
        int rc = extern("GOFLASH", V06N34);
        if (rc == 0) {
            extern("GOTOPOOH");
            return;
        }
        if (rc == 2) {
            p52t();                           // recycle
            return;
        }
        if (agc.get("DSPTEM1") == 0 && agc.get("DSPTEM1+1") == 0) {   // BZF: zero -> use TIME2
            agc.set("DSPTEM1", agc.get("TIME2"));
            agc.set("DSPTEM1+1", agc.get("TIME1"));
        }
        if ((agc.get("OPTION2") & BIT2) != 0) {
            // NOM
            extern("S52.3", "DSPTEM1");       // compute nominal IMU orientation
        } else {
            p52Ls();                          // LS
        }
        p52d();
    }

    /** P52D: read vehicle attitude, compute gimbal angles and display (V06N22). */
    public void p52d() {
        s522();
        int rc = extern("GOFLASH", VB06N22);
        if (rc == 0) {
            extern("GOTOPOOH");
            return;
        }
        coarstyp();
    }

    /** P52J: recycle, vehicle has been maneuvered. */
    public void p52j() {
        p52d();
    }

    /** CAL53A return point: set REFSMFLG then P52C. */
    public void cal53ret() {
        upflag("REFSMFLG");
        p52c();
    }

    /** P52C: star selection; ask for sighting (V05N09 alarm path P52I when none available). */
    public void p52c() {
        extern("PHASCHNG", 04024);
        int rc = extern("GOPERF1", 015);      // ALRM15
        if (rc == 0) {
            extern("GOTOPOOH");
            return;
        }
        if (rc == 2) {
            p52f();                           // E: proceed
            return;
        }
        // V33: compute sighting time and sun/moon/earth ephemeris
        agc.set("TSIGHT", agc.get("LOADTIME") + TSIGHT1);
        locsam();
        p52e();
    }

    /** P52E: star selection. */
    public void p52e() {
        int ok = extern("PICAPAR");
        if (ok == 0) {
            p52i();                           // 2 stars not available
        } else {
            p52f();
        }
    }

    /** P52F: two stars available -> R51. */
    public void p52f() {
        r51();
        endp50s();
    }

    /** ENDP50S: EXIT; TC GOTOPOOH. */
    public void endp50s() {
        extern("GOTOPOOH");
    }

    /** P52I: alarm 405 and V05N09 flash. */
    public void p52i() {
        extern("ALARM", 0405);
        int rc = extern("GOFLASH", V05N09);
        if (rc == 0) {
            extern("GOTOPOOH");
        } else if (rc == 1) {
            p52f();                           // PROCEED: do fine align R51
        } else {
            p52c();                           // RECYCLE: vehicle has been maneuvered
        }
    }

    /** COARSTYP: V34 terminate, normal -> P52J, gyro coarse -> GYCRS. */
    public void coarstyp() {
        int rc = extern("GOPERF1", 013);
        if (rc == 0) {
            extern("GOTOPOOH");               // V34
        } else if (rc == 1) {
            p52j();                           // TCF P52J+3: normal
        } else {
            gycrs();
        }
    }

    /** GYCRS: gyro coarse align: torque gyros to the desired orientation. */
    public void gycrs() {
        // XDC = UNIT(XSMD * REFSMMAT); YDC = UNIT(YSMD * REFSMMAT); ZDC = UNIT(ZSMD * REFSMMAT)
        extern("MXV", "XSMD", "REFSMMAT", "XDC");
        extern("MXV", "YSMD", "REFSMMAT", "YDC");
        extern("MXV", "ZSMD", "REFSMMAT", "ZDC");
        extern("CALCGTA");
        downflag("DRIFTFLG");
        downflag("REFSMFLG");
        extern("GODSPR", V16N20);
        extern("IMUPULSE", agc.get("OGC"));   // R55CDR = ECADR OGC
        extern("IMUSTALL");
        extern("CURTAINS");
        extern("PHASCHNG", 04024);
        matmove("XSMD", "REFSMMAT");
        downflag("PFRATFLG");
        upflag("REFSMFLG");
        extern("SET1/PDT");
        agc.set("GCOMP", 0);                  // GCOMP = ZEROVEC
        upflag("DRIFTFLG");
        r51k();
    }

    /** P52LS: landing-site orientation (ERADFLAG, LUNAFLAG set; lat/long -> LLASRD). */
    public void p52Ls() {
        upflag("LUNAFLAG");
        agc.set("TSIGHT", agc.get("DSPTEM1"));
        upflag("ERADFLAG");
        extern("RP-TO-R", "RLS", "TSIGHT");   // 0D = RLS, 6D = TSIGHT
        extern("LAT-LONG", "TSIGHT");
        extern("LLASRD");
        lsdisp();
    }

    /** LSDISP: display landing site (V06N89), then derive X/Y/Z SMD axes. */
    public void lsdisp() {
        int rc = extern("GOFLASH", V06N89);
        if (rc == 0) {
            extern("GOTOPOOH");
            return;
        }
        if (rc == 2) {
            lsdisp();
            return;
        }
        extern("LLASRDA");
        extern("LALOTORV", "TSIGHT");
        // XSMD = UNIT(ALPHAV); TDEC1 = TSIGHT; CSMPREC
        extern("UNIT", "ALPHAV", "XSMD");
        agc.set("TDEC1", agc.get("TSIGHT"));
        extern("CSMPREC");
        // ZSMD = UNIT(UNIT(RATT x VATT) x XSMD)  ; YSMD = UNIT(ZSMD x XSMD)
        extern("VXV", "RATT", "VATT");
        extern("VXV", "XSMD", "ZSMD");
        extern("VXV", "ZSMD", "XSMD", "YSMD");
    }

    // ===================================================================================
    // R52 : automatic optics positioning routine
    // ===================================================================================

    /** R52: STQ SAVQR52 ; clear ADVTRK; common entry for CSM/LEM tracking. */
    public void r52() {
        downflag("ADVTRK");
        r52vrb();
    }

    /** R52VRB. */
    public void r52vrb() {
        agc.set("DESOPTT", agc.get("CDUT"));
        agc.set("DESOPTT+1", agc.get("CDUT+1"));
        agc.set("OPTIND", 0);
        downflag("R53FLAG");
        r52a();
    }

    /** R52A: set TRUNFLAG and TARG1FLG; if TARG1FLG branch R52H (LEM); clear TERMIFLG. */
    public void r52a() {
        upflag("TRUNFLAG");
        boolean targ1 = agc.get("TARG1FLG") != 0;
        if (targ1) {
            r52h();
            return;
        }
        downflag("TERMIFLG");
        r52c();
    }

    /** R52C: optics mode switch test; manual -> R52M. */
    public void r52c() {
        if (Agc.toSigned(agc.get("SWSAMPLE")) <= 0) {
            r52m();                           // MANUAL
            return;
        }
        r52d();
    }

    /** R52D: AGC mode: SR52.1 returns three ways. */
    public void r52d() {
        int rc = sr521();                     // 0 = GR 90 deg, 1 = GR 50 deg, 2 = LS 50 deg
        if (rc == 0) {
            r52l();
        } else if (rc == 1) {
            r52j();
        } else {
            upflag("TRUNFLAG");               // TRUNFLAG bit 4 flag 0
            r52ja();
        }
    }

    /** R52JA: if LEM or R53FLAG set -> R52E; else display V06N92. */
    public void r52ja() {
        boolean lem = stateBit(1, BIT10);
        if (!lem && !stateBit(0, BIT6)) {
            extern("GODSPR", V06N92);
        }
        r52e();
    }

    /** R52E: if OSS in CMC mode, load desired optics angles. */
    public void r52e() {
        if (Agc.toSigned(agc.get("SWSAMPLE")) > 0) {
            if ((agc.get("STATE") & BIT4) == 0) {         // TRUNFLAG clear
                agc.set("DESOPTT", agc.get("PAC"));
                agc.set("DESOPTS", agc.get("SAC"));
            }
        }
        r52f();
    }

    /** R52F: wait 1/2 sec, then loop unless terminated. */
    public void r52f() {
        extern("DELAYJOB", 50);               // .5SEC
        if (stateBit(1, BIT10)) {
            r52ha();                          // LEM
            return;
        }
        if ((agc.get("STATE7") & BIT15) == 0) {           // TERMIFLG not set
            r52c();
            return;
        }
        r52q();
    }

    /** R52Q: return to caller (SAVQR52). */
    public void r52q() {
        // GOTO SAVQR52
    }

    /** R52H: LEM path. */
    public void r52h() {
        r52ha();
    }

    /** R52HA: call R61CSM; track/update flags. */
    public void r52ha() {
        extern("R61CSM");
        if (!stateBit(1, BIT5)) {                         // TRACKFLG clear
            r52q();
            return;
        }
        if (stateBit(1, BIT7)) {                          // UPDATFLG set? CS STATE+1 MASK BIT7
            // CCS A on complement mask: non-zero (flag clear) -> R52SYNC
        }
        if (!stateBit(1, BIT7)) {
            r52sync();
            return;
        }
        if (stateBit(5, BIT10)) {                         // PRFTRKAT = 1
            r52d();
            return;
        }
        r52sync();
    }

    /** R52SYNC: make up for lost time (1.8 sec) and continue. */
    public void r52sync() {
        extern("DELAYJOB", SEC1_8);
        if ((agc.get("STATE7") & BIT15) == 0) {
            r52c();
        } else {
            r52q();
        }
    }

    /** R52J: clear TRUNFLAG, alarm 407, continue at R52JA. */
    public void r52j() {
        downflag("TRUNFLAG");
        extern("ALARM", 0407);
        r52ja();
    }

    /** R52M: manual mode: start R53JOB if R53FLAG not set. */
    public void r52m() {
        if (!stateBit(0, BIT6)) {
            extern("FINDVAC", 24, "R53JOB");  // PRIO24
        }
        r52f();
    }

    /** R53JOB: sighting job (EBANK = SAC). */
    public void r53job() {
        r53();
        endplac();
    }

    /** ENDPLAC: interpreter return to ENDOFJOB. */
    public void endplac() {
        extern("ENDOFJOB");
    }

    /** R52L: optics greater than 90 degrees; alarm 404 via PRIOLARM unless LEM. */
    public void r52l() {
        if (stateBit(1, BIT10)) {
            r52j();                           // LEM
            return;
        }
        int rc = extern("PRIOLARM", OCT404);  // 0 terminate, 1 proceed, 2 no new data, 3 endofjob
        if (rc == 0) {
            term52();
        } else if (rc == 1 || rc == 2) {
            r52f();
        } else {
            extern("ENDOFJOB");
        }
    }

    /** TERM52: kill mark system, clear extended verbs, go to P00. */
    public void term52() {
        extern("CLEARMRK");
        extern("MKRELEAS");
        agc.set("OPTCADR", 0);
        extern("KLEENEX");
        extern("GOTOPOOH");
    }

    /** ADVORB: set up advanced orbit tracking. */
    public void advorb() {
        upflag("ADVTRK");
        upflag("LUNAFLAG");
        upflag("ERADFLAG");
        r52vrb();
    }

    // ===================================================================================
    // S50 / LOCSAM : planet / sun / moon ephemeris
    // ===================================================================================

    /** LOCSAM = S50: LSPOS gives moon (VMOON) and sun (VSUN); CSMCONIC gives RATT; set cosines. */
    public void locsam() {
        extern("LSPOS", "TSIGHT");
        // VMOON = 2D (moon vector), VSUN = vector from LSPOS
        agc.set("TDEC1", agc.get("TSIGHT"));
        extern("CSMCONIC");
        agc.set("S2", 0);
        if (agc.get("MOONCNTR") != 0) {       // TIX,2 test reflects current-body flag
            mooncntr();
        } else {
            eartcntr();
        }
    }

    /** EARTCNTR: earth-centred ephemeris. */
    public void eartcntr() {
        // VMOON = UNIT(VMOON - RATT) ; VEARTH = -UNIT(RATT) ; CEARTH = OCCOS(RSUBE)
        extern("VSU", "VMOON", "RATT");
        extern("UNIT", "VMOON");
        extern("UNIT", "RATT");
        extern("VCOMP", "VEARTH");
        extern("OCCOS", "RSUBE", "CEARTH");
        agc.set("CMOON", (int) Math.round(CSS5 * 16384));
        extern("UNIT", "VSUN");
        endsam();
    }

    /** MOONCNTR: moon-centred ephemeris. */
    public void mooncntr() {
        // VSUN = UNIT(VSUN/8/2 - VMOON); VEARTH = -(VMOON + RATT); VMOON = -UNIT(RATT)
        extern("VSR8", "VMOON");
        extern("VSR1");
        extern("BVSU", "VSUN");
        extern("UNIT", "VSUN");
        extern("VAD", "VMOON", "RATT");
        extern("VCOMP", "VEARTH");
        extern("UNIT", "RATT");
        extern("VCOMP", "VMOON");
        extern("OCCOS", "RSUBM", "CMOON");
        agc.set("CEARTH", (int) Math.round(CSS5 * 16384));
        endsam();
    }

    /** ENDSAM: aberration correction: VEL/C = (ECLIPOL x VATT)... then CSUN. */
    public void endsam() {
        extern("VXV", "ECLIPOL");
        extern("VXSC", "VATT", ONE_OVER_C);
        extern("VAD", "VEL/C");
        agc.set("CSUN", (int) Math.round(CSSUN * 16384));
    }

    /** OCCOS: cosine of occulting body half-angle: COS(ASIN(R/|r|/2)+5 deg)/2. */
    public double occos(double radius, double rmag) {
        double s = radius / rmag / 2.0;
        double asin = Math.asin(Math.min(1.0, Math.max(-1.0, s * 2))) / (2 * Math.PI) + FIVE_DEGREES;
        return Math.cos(asin * 2 * Math.PI) / 2.0;
    }

    // ===================================================================================
    // PICAPAR : star selection
    // ===================================================================================

    /**
     * PICAPAR: choose the best pair of navigation stars from the 37 star catalog (CATLOG), subject
     * to occultation by earth/moon/sun (OCCULT), a 40-66 degree separation and the 33 degree cone about
     * the shaft axis SAX. Returns 0 if 2 stars are not available, 1 if available (QMIN+1).
     */
    public int picapar() {
        extern("CDUTRIG");
        extern("CALCSMSC");
        upflag("VFLAG");
        agc.set("BESTI", 0);
        // SAX = UNIT((XNB*SIN33 + ZNB*COS33) * REFSMMAT) -- shaft axis
        extern("PICSAX", SIN33, COS33);
        int s1 = 6, s2 = 6;
        agc.set("S1", s1);
        agc.set("S2", s2);
        int x1 = 228;                         // 37 X 6 + 6
        agc.set("X1", x1);
        // The pair search is expressed as nested loops in pic1..pic4.
        return picPairSearch(x1);
    }

    private int picPairSearch(int x1) {
        // PIC1: loop major star i (X1 descending by 6); PIC2: occultation test
        for (int i = x1 - 6; i >= 0; i -= 6) {
            if (i == 0) {
                break;
            }
            if (occult(i)) {
                continue;                     // CULTFLAG set -> PIC1
            }
            for (int j = i - 6; j > 0; j -= 6) {          // PIC3/PIC4
                if (occult(j)) {
                    continue;
                }
                double dot = extern("DOT", "CATLOG", i, j) / 16384.0;
                // CSS66 test: separation less than 66 deg
                if (dot - CSS66 < 0) {
                    continue;                 // BMN PIC3
                }
                if (dot - CSS66 + CSS6640 >= 0 + 0 && dot - CSS66 + CSS6640 > 0) {
                    continue;                 // separation more than 40 deg (BPL PIC3)
                }
                // cone test about shaft axis: major star in cone -> PIC1 next major star
                double dotI = extern("DOT", "CATLOG", i, "SAX") / 16384.0;
                if (dotI - CSS33 < 0) {
                    break;                    // BMN PIC1
                }
                double dotJ = extern("DOT", "CATLOG", j, "SAX") / 16384.0;
                if (dotJ - CSS33 >= 0) {
                    strategy(i, j);           // second star also in cone: strategy
                }
            }
        }
        return picend();
    }

    /** STRATGY / STRAT / NEWPAR: remember best pair (BESTI, BESTJ) by minimum star separation. */
    private void strategy(int i, int j) {
        boolean vflag = agc.get("VFLAG") != 0;
        if (vflag) {
            downflag("VFLAG");                // BONCLR VFLAG NEWPAR
            newpar(i, j);
            return;
        }
        // STRAT: exchange and compare; keep pair whose separation is nearer the best
        int dotNew = extern("DOT", "CATLOG", i, j);
        int dotBest = agc.get("PDDOT");
        if (dotNew - dotBest >= 0) {
            newpar(i, j);
        }
    }

    private void newpar(int i, int j) {
        agc.set("BESTI", i);
        agc.set("BESTJ", j);
    }

    /** OCCULT: true (CULTFLAG set) if the star is occulted by earth, moon or sun. */
    private boolean occult(int star) {
        extern("CULTRIX", star);
        boolean culted = false;
        // MXV CULTRIX BVSU CSS: three components tested against zero / sign
        int mp0 = agc.get("MPAC");
        int mp3 = agc.get("MPAC+3");
        int mp5 = agc.get("MPAC+5");
        culted = mp0 == 0 || Agc.toSigned(mp0) < 0 || Agc.toSigned(mp3) < 0 || Agc.toSigned(mp5) < 0;
        agc.set("CULTFLAG", culted ? 1 : 0);
        return culted;
    }

    /** PICEND: return +1 (2 stars found, VFLAG clear) or +0 (none). */
    private int picend() {
        int qmin = agc.get("QMIN");
        if (agc.get("VFLAG") == 0) {
            qmin++;                           // PICGXT: INCR QMIN
        }
        agc.set("QMIN", qmin);
        return agc.get("VFLAG") == 0 ? 1 : 0;
    }

    // ===================================================================================
    // R51 : fine align
    // ===================================================================================

    /** R51: sighting of the two selected (or terminated) stars, then R54 and R55. */
    public void r51() {
        agc.set("STARIND", BIT1);
        agc.set("MARKINDX", BIT1);
        r513();
    }

    /** R51.3: clear target flags, restart protection, load star code from BESTI. */
    public void r513() {
        downflag("TARG2FLG");
        downflag("TARG1FLG");
        extern("PHASCHNG", 05024, 013000);
        int besti = agc.get("BESTI");         // indexed by STARIND
        agc.set("STARCODE", (int) (besti * ONE_SIXTH + 1e-9));  // star code = BESTI/6
        r51dsp();
    }

    /** R51DSP: display V01N70 star code; cheque the code; terminate / proceed / recycle. */
    public void r51dsp() {
        int rc = extern("GOFLASHR", V01N70);
        if (rc == 0) {
            extern("GOTOPOOH");
            return;
        }
        if (rc == 2) {
            r51dsp();
            return;
        }
        if (rc == 3) {                        // BLANKET path: ENDOFJOB
            extern("BLANKET", 6);
            extern("ENDOFJOB");
            return;
        }
        int ck = chkscode();
        if (ck == 0) {
            falton();                         // bad code
            r51dsp();
            return;
        }
        extern("LOADTIME");
        planet();                             // PLANET -> star vector in MPAC
        int starind = agc.get("STARIND");
        agc.set("S1", 0);
        if (starind != 0) {
            // 2nd star
            agc.set("STARSAV2", agc.get("MPAC"));
        } else {
            agc.set("STARSAV1", agc.get("MPAC"));
        }
        if (agc.get("MODREG") == OCT66 - 0) {   // is this P54 (MODREG + 66 octal... zero test)
        }
        if (agc.get("MODREG") + OCT66 == 0 || agc.get("MODREG") == OCT66) {
            r56();
        } else {
            r52();                            // AOP will make calls to sighting
        }
        r51a();
    }

    /** R51A: compute LOS in stable member from mark data (SXTSM), release marks, test next star. */
    public void r51a() {
        sxtsm();
        agc.set("STARSAV2", agc.get("MPAC"));
        extern("MKRELEAS");
        planet();
        int cc = agc.get("STARIND");
        if (cc != 0) {                        // CCS STARIND: first star done, do second
            r514();
            return;
        }
        // STARAD = UNIT(STARSAV2 * REFSMMAT)
        extern("MXV", "REFSMMAT", "STARSAV2", "STARAD");
        // 6D = STARSAV2? 12D = STARSAV1/PLANVEC; STARAD+6 = PLANVEC -> R54
        extern("R54");
        chksdata();
        if (agc.get("FREEFLAG") == 0) {       // BOFF FREEFLAG R51K
            r51k();
            return;
        }
        extern("AXISGEN");
        r55();                                // gyro torque
        downflag("PFRATFLG");
        r51k();
    }

    /** R51K: V16 proceed query (OCT14) then back to ENDP50S or P52C. */
    public void r51k() {
        int rc = extern("GOPERF1", 014);
        if (rc == 0) {
            extern("GOTOPOOH");
        } else if (rc == 1) {                 // V33 proceed
            endp50s();
        } else {
            p52c();
        }
    }

    /** R51.4: first star done; save and loop for second. */
    public void r514() {
        extern("MXV", "REFSMMAT", "STARSAV2", "PLANVEC");
        agc.set("STARSAV1", agc.get("STARSAV2"));
        agc.set("STARIND", 0);
        r513();
    }

    /** R51B: P54 backup, use R56 (alternate LOS sighting). */
    public void r51b() {
        r56();
        r51a();
    }

    // ===================================================================================
    // R55 : gyro torque; R54 : star data test
    // ===================================================================================

    /** R55: compute gyro torque angles, display V06N93, and torque gyros (IMUPULSE). */
    public void r55() {
        extern("CALCGTA");
        pulsem();
    }

    /** PULSEM / R55.1: V06N93 display, then R55.2 torque. */
    public void pulsem() {
        int rc = extern("GOFLASH", V06N93);
        if (rc == 0) {
            extern("GOTOPOOH");
            return;
        }
        if (rc == 1) {
            r55ret();                         // TC R55RET (PROCEED skips torque)
            return;
        }
        r552();
    }

    /** R55.2: torque the gyros. */
    public void r552() {
        extern("PHASCHNG", 00314);
        extern("IMUPULSE", agc.get("OGC"));   // R55CDR = ECADR OGC
        extern("IMUSTALL");
        extern("CURTAINS");
        extern("PHASCHNG", 05024, 013000);
        r55ret();
    }

    /** R55RET: return to caller via QMIN. */
    public void r55ret() {
        // GOTO QMIN
    }

    /** R54 = CHKSDATA: compare angles between the star pair as measured and as computed. */
    public void chksdata() {
        upflag("FREEFLAG");
        // CHKSAB/CHKSB: THETA = ACOS(a.b << 1) for the catalog pair (X1 = STARAD),
        // then for the measured pair (X1 = 6D).
        extern("CHKSB", "STARAD");
        double theta1 = agc.get("THETA");
        downflag("FREEFLAG");                 // BOFF/INVERT on first pass
        extern("CHKSB", 6);
        double theta2 = agc.get("THETA");
        agc.set("18D", (int) theta2);
        chksd(theta1, theta2);
    }

    /** CHKSD: position difference; alarm display if star angle difference is not in agreement. */
    private void chksd(double theta1, double theta2) {
        int diff = (int) Math.abs(theta1 - theta2);
        agc.set("NORMTEM1", diff);
        extern("SGNAGREE", diff);
        upflag("FREEFLAG");
        extern("CLEANDSP", 0);
        int rc = extern("GOFLASH", VB6N5);
        if (rc == 0) {
            extern("GOTOPOOH");
            return;
        }
        // CHKSDA: PROCEED -> return; otherwise clear FREEFLAG and return
        downflag("FREEFLAG");
    }

    // ===================================================================================
    // CAL53A and coarse align helpers
    // ===================================================================================

    /** CAL53A: compute final gimbal angles; if CDU angles within 1 degree skip coarse align. */
    public void cal53a() {
        s522();
        rdcdus();                             // CDU angles to 1..3 of FIXLOC
        for (int x1 = 3; x1 >= 1; x1--) {
            int thetad = agc.get("THETAD+" + (x1 - 1));
            int cdu = agc.get("FIX" + x1);
            int d = Math.abs(Agc.toSigned(thetad >> 1) - Agc.toSigned(cdu >> 1));
            if (d >= DEGREE1 && d <= DEG359) {
                coarfine();                   // difference > 1 degree: do coarse align
                return;
            }
        }
        fineonly();
    }

    /** COARFINE: coarse align the IMU then fine/zero-drift. */
    public void coarfine() {
        extern("PHASCHNG", 04024);
        extern("IMUCOARS");
        extern("IMUSTALL");
        extern("CURTAINS");
        extern("IMUFIN20");
        extern("IMUSTALL");
        extern("CURTAINS");
        extern("SET1/PDT");
        agc.set("GCOMP", 0);                  // ZEROVEC
        upflag("DRIFTFLG");
        fineonly();
    }

    /** FINEONLY: move XSM into REFSMMAT and return to CAL53RET. */
    public void fineonly() {
        matmove("XSM", "REFSMMAT");
        cal53ret();
    }

    /** MATMOVE: transfer a 3x3 matrix (X1 -> X2). */
    public void matmove(String from, String to) {
        extern("MATMOVE", from, to);
    }

    /** RDCDUS: INHINT; read CDUX/Y/Z into FIXLOC+1..3; RELINT. */
    public void rdcdus() {
        agc.set("FIX1", agc.get("CDUX"));
        agc.set("FIX2", agc.get("CDUY"));
        agc.set("FIX3", agc.get("CDUZ"));
    }

    /** CALCSMSC: compute the spacecraft body axes (XNB, YNB, ZNB) in navigation base from CDU trig. */
    public void calcsmsc() {
        double sy = agc.get("SINCDUY"), cy = agc.get("COSCDUY");
        double sz = agc.get("SINCDUZ"), cz = agc.get("COSCDUZ");
        double sx = agc.get("SINCDUX"), cx = agc.get("COSCDUX");
        // XNB = (COSY*COSZ, -SINZ... ) as in the AGC source (scaled B-1, VSL1)
        agc.set("XNB", (int) (-sy * cz));      // see CALCSMSC: DCOMP(SINY*COSZ)
        agc.set("XNB+2", (int) (sz / 2));      // SINCDUZ SR1
        agc.set("XNB+4", (int) (cy * cz));
        double t26 = sx * sz * 2;
        agc.set("26D", (int) t26);
        agc.set("ZNB", (int) (t26 * sy - cx * cy));
        agc.set("ZNB+2", (int) (-sx * cz));
        agc.set("ZNB+4", (int) (-cx * sy + cy * t26));
        extern("VXV", "ZNB", "XNB", "YNB");   // YNB = ZNB x XNB (VSL1)
    }

    // ===================================================================================
    // P51 / P53
    // ===================================================================================

    /** P51/P53: IMU orientation determination. P53 = backup IMU orientation. */
    public void p51() {
        if ((~agc.get("IMODES30") & BIT9 & Agc.WORD_MASK) != 0) {   // IMU on? (bit 9 clear -> OK)
            p51a();
        } else {
            extern("ALARM", 0210);
            extern("GOTOPOOH");
        }
    }

    public void p53() {
        p51();
    }

    /** P51A: zero IMU, then P51AA. */
    public void p51a() {
        extern("R02ZERO");
        p51aa();
    }

    /** P51AA: coarse align to zero gimbal angles, then fine align. */
    public void p51aa() {
        int rc = extern("GOPERF1", 015);      // PRFMSTAQ = OCT15
        if (rc == 0) {
            extern("GOTOPOOH");                // TERM
            return;
        }
        if (rc == 1) {
            p51b();                            // V33
        } else {
            p51Coarse();                       // ENTER: coarse align first
        }
    }

    /** Coarse alignment sequence used when the astronaut recycles (not in V33 path). */
    public void p51Coarse() {
        extern("PHASCHNG", 05024, 013000);
        agc.set("THETAD", 0);
        agc.set("THETAD+1", 0);
        agc.set("THETAD+2", 0);
        extern("GODSPRET", V6N22);
        extern("GODSPRET", V41K);
        extern("IMUCOARS");
        extern("IMUSTALL");
        extern("CURTAINS");
        extern("IMUFIN20");
        extern("IMUSTALL");
        extern("CURTAINS");
        extern("PHASCHNG", 05024, 013000);
        p51aa();                              // coarse align done: recycle for fine
    }

    /** P51B: begin fine alignment: zero gyro compensation, enable T4 compensation. */
    public void p51b() {
        extern("PHASCHNG", 00014);
        agc.set("STARIND", 0);
        extern("SET1/PDT");
        agc.set("GCOMP", 0);                  // ZEROVEC
        upflag("DRIFTFLG");                   // enable T4 compensation
        downflag("TARG2FLG");                 // mark is a star, not a landmark
        agc.set("MARKINDX", BIT1);
        p51c();
    }

    /** P51C: sighting loop (P53 uses R56 backup, P51/P52 use R53). */
    public void p51c() {
        extern("PHASCHNG", 05024, 013000);
        boolean mm53 = extern("CHECKMM", 53) != 0;
        if (mm53) {
            r56();
        } else {
            r53();
        }
        p51c2();
    }

    /** P51C.2: compute LOS in SM from mark data; store star vectors. */
    public void p51c2() {
        sxtsm();
        if (agc.get("STARIND") == 0) {        // P51D: first star
            agc.set("STARSAV1", agc.get("MPAC"));
            extern("PLANET", agc.get("TSIGHT"));
            agc.set("PLANVEC", agc.get("MPAC"));
        } else {
            agc.set("STARSAV2", agc.get("MPAC"));     // downlink
        }
        p51e();
    }

    /** P51E: release marks; second star or finish. */
    public void p51e() {
        extern("PHASCHNG", 05024, 013000);
        extern("MKRELEAS");
        if (agc.get("STARIND") != 0) {
            p51f();                           // star 2 done
            return;
        }
        extern("PHASCHNG", 05024, 013000);
        agc.set("STARIND", BIT1);
        p51c();                               // do second star
    }

    /** P51F: compute star data, check angles (CHKSDATA), generate REFSMMAT (AXISGEN). */
    public void p51f() {
        extern("PHASCHNG", 05024, 013000);
        planet();
        // 12D = star 2 (PLANET), 6D = PLANVEC, STARAD = STARSAV1, STARAD+6 = STARSAV2
        chksdata();
        if (agc.get("FREEFLAG") != 0) {       // BON FREEFLAG P51G
            p51g();
        } else {
            p51aa();
        }
    }

    /** P51G: axes generation, REFSMMAT = XDC, set REFSMFLG. */
    public void p51g() {
        extern("AXISGEN");                    // REFSMMAT comes back in XDC
        downflag("REFSMFLG");
        matmove("XDC", "REFSMMAT");
        upflag("REFSMFLG");
        endp50s();
    }

    /** SET1/PDT: 1/PIPADT = TIME1; then DANZIG. */
    public void set1Pdt() {
        agc.set("1/PIPADT", agc.get("TIME1"));
        extern("DANZIG");
    }

    // ===================================================================================
    // SXTSM, R53, CHKSCODE
    // ===================================================================================

    /** SXTSM: compute line of sight in stable-member coordinates from mark data (MARKSTAT). */
    public void sxtsm() {
        int base = agc.get("MARKSTAT");
        agc.set("TSIGHT", agc.get("MARK" + base));       // time of mark
        // MKDNCDR: ECADR MARKDOWN, MARK2DWN indexed by STARIND; copy mark data for downlink
        extern("MARKDOWN", agc.get("STARIND"));
        extern("SXTNB");                      // LOS from OCDU in MKVAC
        agc.set("S1", base + 2);              // increment to base address of ICDU
        extern("NBSM");                       // transform LOS to SM
    }

    /** R53: sighting mark routine: set R53FLAG and take marks. */
    public void r53() {
        upflag("R53FLAG");
        r53a();
    }

    /** R53A: call SXTMARK for MARKINDX marks, wait on OPTSTALL, check marks done. */
    public void r53a() {
        extern("SXTMARK", agc.get("MARKINDX") & 7);
        extern("OPTSTALL");
        extern("CURTAINS");
        int done = agc.get("QPRET");          // number of marks actually done
        if (Agc.toSigned(done) > 0) {
            r53b();
            return;
        }
        // zero or CCS hole: housekeep VAC area and MARKSTAT, retry
        int ms = agc.get("MARKSTAT");
        agc.set("MARKSTAT", 0);
        if (ms != 0) {
            agc.set("VAC" + ms, 0);
        }
        r53a();
    }

    /** R53B: if P22/P23 skip star code input. */
    public void r53b() {
        boolean mm22 = extern("CHECKMM", 22) != 0;
        if (mm22) {
            r53d();
            return;
        }
        boolean mm23 = extern("CHECKMM", 23) != 0;
        if (!mm23) {
            r53d();
            return;
        }
        r53c1();
    }

    /** R53C1: blank display and ask for star code V01N71. */
    public void r53c1() {
        extern("CLEANDSP", 0);
        r53c();
    }

    /** R53C. */
    public void r53c() {
        int rc = extern("GOFLASH", V01N71);
        if (rc == 0) {
            extern("GOTOPOOH");
        } else if (rc == 2) {
            r53c();                           // recycle
        } else {
            r53z();
        }
    }

    /** R53Z: check star code; compute BESTI = 6 * (STARCODE & ~HIGH9). */
    public void r53z() {
        if (chkscode() == 0) {
            falton();
            r53c();
            return;
        }
        int sc = agc.get("STARCODE") & ~0x7FC0 & Agc.WORD_MASK;   // CS HIGH9 / MASK
        agc.set("BESTI", sc * 6);
        r53d();
    }

    /** R53D / R53OUT: set termination flag for R52 and return. */
    public void r53d() {
        upflag("TERMIFLG");
    }

    /** CHKSCODE: returns 1 if STARCODE is 0..46 (valid), 0 if out of range (error return). */
    public int chkscode() {
        int sc = Agc.toSigned(agc.get("STARCODE"));
        if (sc < 0) {
            sc = -sc;
        }
        return sc <= 46 ? 1 : 0;
    }

    private void falton() {
        extern("FALTON");
    }

    // ===================================================================================
    // S52.2 : gimbal angles; SR52.1 : optics check; ADVTRACK; ROTA; S52.3; R56; PLANET
    // ===================================================================================

    /** S52.2: spacecraft body axes in SM coordinates and gimbal angles (CALCGA). */
    public void s522() {
        extern("CDUTRIG");
        calcsmsc();
        for (int x1 = 18; x1 >= 0; x1 -= 6) {
            // XNB+18D,1 = UNIT(XNB+18D,1 * REFSMMAT)
            extern("MXV", "XNB+" + x1, "REFSMMAT");
        }
        matmove("XSMD", "XSM");
        extern("CALCGA");
    }

    /**
     * SR52.1: optics trunnion / shaft check. Returns 0 if greater than 90 degrees, 1 if greater
     * than 50 degrees, 2 if less than 50 degrees (the three possible returns of the original).
     */
    public int sr521() {
        agc.set("AOPTIME", agc.get("LOADTIME") + SEC1_3_DP);
        if (agc.get("TARG1FLG") != 0) {
            lem52();
        } else if (agc.get("TARG2FLG") != 0) {
            lmk52();
        } else {
            star52();
        }
        return com52();
    }

    /** LEM52: LEMCONIC vector for the LEM; then LMKLMCOM. */
    public void lem52() {
        agc.set("TDEC1", agc.get("AOPTIME"));
        extern("LEMCONIC");
        lmklmcom("RATT");
    }

    /** LMK52: landmark: advanced tracking or LALOTORV. */
    public void lmk52() {
        if (agc.get("ADVTRK") != 0) {
            advtrack();
            return;
        }
        extern("LALOTORV", "AOPTIME");
        lmklmcom("ALPHAV");
    }

    /** LMKLMCOM: STAR = vector; CSMCONIC; LOS = UNIT(STAR - RATT). */
    private void lmklmcom(String vec) {
        extern("STORE", "STAR", vec);
        agc.set("TDEC1", agc.get("AOPTIME"));
        extern("CSMCONIC");
        extern("VSU", "STAR", "RATT");
        extern("UNIT");
    }

    /** STAR52: star LOS from STARSAV1 (STARIND != 0 selects STARSAV1, else STARSAV2). */
    public void star52() {
        extern("STAR52", agc.get("STARIND") != 0 ? "STARSAV1" : "STARSAV2");
    }

    /** COM52: STAR = UNIT(LOS * REFSMMAT); CALCSXA; return code from trunnion tests. */
    public int com52() {
        extern("MXV", "REFSMMAT", "STAR");
        extern("CDUTRIG");
        extern("CALCSXA");
        if (agc.get("CULTFLAG") == 0) {
            return 2 - 2 + trun38();
        }
        return 0;                              // SR52E1: > 90 deg
    }

    private int trun38() {
        int pac = agc.get("PAC");
        if (pac - (int) (TRUN38 * 16384) >= 0) {
            return 1;                          // SR52E2: greater than 50 degrees
        }
        if (pac + MINUS_20DEG_SMN >= 0) {
            return 2;                          // SR52E3: less than 50 degrees
        }
        return 1;
    }

    /** ADVTRACK: advanced tracking: rotate LOS about Moon axis by N/16 periods, then 60 - A. */
    public void advtrack() {
        extern("RP-TO-R", "UNITZ", "LOADTIME"); // moon rotation vector in STAR
        agc.set("AOPTIME", agc.get("LOADTIME"));
        agc.set("TDEC1", agc.get("AOPTIME"));
        extern("CSMCONIC");
        extern("VXV", "VATT", "RATT");        // 24D = -UNIT(V x R) for 2nd rotation
        extern("UNIT");
        extern("VCOMP");                       // LOS = -UNIT(RATT)
        int n = agc.get("LANDMARK") & 7;       // number of advance periods
        int t30 = (int) (((long) n * BIT11) >> 14);   // N/16
        agc.set("30D", t30);
        agc.set("AOPTIME", (int) Math.round(t30 * MPERIOD));
        rota();                                // first rotation
        agc.set("AOPTIME", (int) Math.round(DP1_6 * 16384) - agc.get("AOPTIME"));  // 60 - A
        rota();                                // second rotation
    }

    /** ROTA: rotate LOS by angle AOPTIME about unit vector STAR (Rodrigues formula). */
    public void rota() {
        double a = agc.get("AOPTIME") / 16384.0 * 2 * Math.PI;
        // result = UNIT( 1/2 sinA (UR x LOS) *4 + 1/2 (UR.LOS) UR *4 + 1/2 cosA (LOS - ...) *2 )
        extern("ROTA", Math.sin(a), Math.cos(a));
    }

    /** S52.3: nominal IMU orientation: ZSMD = -UNIT(R), YSMD = UNIT(R x V), XSMD = Y x Z. */
    public void s523(int tdec) {
        agc.set("TDEC1", tdec);
        extern("CSMPREC");
        extern("VCOMP", "RATT");
        extern("UNIT", "ZSMD");
        extern("VXV", "VATT", "RATT");
        extern("UNIT", "YSMD");
        extern("VXV", "YSMD", "ZSMD");
        extern("UNIT", "XSMD");
    }

    /** R56: alternate LOS sighting mark routine (backup for optics failure). */
    public void r56() {
        int rc = extern("GOFLASH", V06N94B);
        while (rc == 2) {                     // ENTER: new angles
            rc = extern("GOFLASH", V06N94B);
        }
        if (rc == 0) {
            extern("GOTOPOOH");               // TERM
            return;
        }
        extern("SXTMARK+2");                   // inhibit ext verb activity and find VAC area
        extern("CLEANDSP", 0);
        r56a1();
    }

    /** R56A1: V53 requests alternate mark; save CDU data into the mark storage area. */
    public void r56a1() {
        int rc = extern("GOMARK2", VB53);
        if (rc == 0) {
            extern("GOTOPOOH");               // V34 terminate
            return;
        }
        if (rc == 2) {
            r56a1();                          // V33: just enter to mark
            return;
        }
        agc.set("SAC", agc.get("MRKBUF1+3"));
        agc.set("PAC", agc.get("MRKBUF1+5"));
        int ms = agc.get("MARKSTAT");
        agc.set("MARK" + ms, agc.get("TIME2"));              // DXCH 0
        agc.set("MARK" + (ms + 1), agc.get("TIME1"));
        agc.set("MARK" + (ms + 2), agc.get("CDUY"));
        agc.set("MARK" + (ms + 3), agc.get("SAC"));
        agc.set("MARK" + (ms + 4), agc.get("CDUZ"));
        agc.set("MARK" + (ms + 5), agc.get("PAC"));
        agc.set("MARK" + (ms + 6), agc.get("CDUX"));
        extern("CLEARMRK");                    // enable extended verbs
        rc = extern("GOPERF1", 016);
        if (rc == 0) {
            extern("GOTOPOOH");
        } else if (rc == 1) {
            extern("R53C1");                   // R56B: proceed, mark completed
        } else {
            r56a1();                           // recycle: do another mark
        }
    }

    /** PLANET: star / planet vector selection by STARCODE. Result in MPAC. */
    public void planet() {
        agc.set("TSIGHT", agc.get("MPAC"));
        locsam();
        // swap VEARTH/VSUN as in the original so CALSAM works with the right body
        int tmp = agc.get("VEARTH");
        agc.set("VEARTH", agc.get("VSUN"));
        agc.set("VSUN", tmp);
        nosam();
    }

    /** NOSAM: BESTI = 6 * (STARCODE & ~HIGH9); planet (0), catalog star (1..45) or sun/earth/moon. */
    public void nosam() {
        int sc = agc.get("STARCODE") & ~0x7FC0 & Agc.WORD_MASK;
        agc.set("BESTI", sc * 6);
        if (sc == 0) {
            int rc = extern("GOFLASH", VNPLANV);
            while (rc == 2) {
                rc = extern("GOFLASH", VNPLANV);
            }
            extern("VXSC", "STARSAV3", ONE_OVER_SQRT3);
            extern("UNIT");
            corplan();
            return;
        }
        notplan(sc);
    }

    private void notplan(int sc) {
        if (sc - DEC227 <= 0 && -sc + DEC227 >= 0 && sc <= DEC227) {
            // catalog star: X1 = BESTI
            agc.set("X1", agc.get("BESTI"));
            extern("VLOAD*", "CATLOG", agc.get("X1"));
            corplan();
            return;
        }
        // CALSAM1: sun / earth / moon vector from STARAD-228D,1
        extern("VLOAD*", "STARAD", agc.get("BESTI") - 228);
        corplan();
    }

    /** CORPLAN: add aberration VEL/C and unitize. */
    private void corplan() {
        extern("VAD", "VEL/C");
        extern("UNIT");
    }

    /** Binary-coded code reference for restart protection. */
    public int version() {
        return 0;
    }
}
