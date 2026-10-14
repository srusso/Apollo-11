package apollo.luminary099;

import apollo.agc.Agc;

/**
 * AOTMARK.agc - Alignment Optical Telescope (AOT) mark taking: program/verb 54/53/52 star-sighting
 * marks (X and Y reticle marks, reject, and averaging of marks), conversion of AOT detent codes
 * and reticle angles into a star line-of-sight vector in navigation base coordinates (OANB),
 * and the surface (R59 / P50S) variant (SURFSTAR).
 *
 * MARKSTAT (erasable): bits 1-9 (LOW9) = base address of the mark storage area in VAC; bit 12 =
 * mark display flag; bit 13 = reject flag; bit 14 = bit that indicates X and Y marks both taken;
 * PRIO3 mask (octal 3 on bits 1-2) = which mark types (X=1, Y=2) are done.
 */
public class AotMark {
    // ---- constants ----
    public static final int LOW9 = 0777;
    public static final int HIGH9 = 077600;
    public static final int BIT2 = 02;
    public static final int BIT3 = 04;
    public static final int BIT4 = 010;
    public static final int BIT5 = 020;
    public static final int BIT6 = 040;
    public static final int BIT8 = 0200;
    public static final int BIT9 = 0400;
    public static final int BIT10 = 01000;
    public static final int BIT11 = 02000;
    public static final int BIT12 = 04000;
    public static final int BIT13 = 010000;
    public static final int BIT14 = 020000;
    public static final int PRIO3 = 03;
    public static final int PRIO15 = 015;
    public static final int PRIO23 = 023;
    public static final int PRIO30 = 030;
    public static final int OCT34 = 034;
    public static final int OCT140 = 0140;
    public static final int NEG7 = -7;
    /** Verb/noun codes: V01N71, V06N87*, V06N71, V06N79* written as verb*100+noun. */
    public static final int V01N71 = 171;
    public static final int V06N87 = 687;
    public static final int V06N71 = 671;
    public static final int V06N79 = 679;
    public static final int MKVB54 = 5471;
    public static final int MKVB53 = 5371;
    public static final int MKVB52 = 5271;
    public static final int MKVB54_STAR = 5471;
    /** Alarm codes. */
    public static final int ALARM_PROG_NOT_ACTIVE = 0105;
    public static final int BAILOUT_NO_VAC = 01207;
    public static final int BAILOUT_EXTVB_BUSY = 01211;
    public static final int ALARM_MARK_NOT_EXPECTED = 0112;
    public static final int ALARM_NO_MARK_DEVICE = 0111;
    public static final int ALARM_KEY_BAD = 0113;
    public static final int ALARM_BAD_MARK = 0114;
    public static final int ALARM_TOO_MANY_MARKS = 0107;
    public static final int ALARM_REJECT_BAD = 0115;
    /** DP1/8 2DEC .125 and ABOUTONE 2DEC .99999999, DP1/12 = DEG30. */
    public static final double DP1_8 = 0.125;
    public static final double ABOUT_ONE = 0.99999999;

    private final Agc agc;

    public AotMark(Agc agc) {
        this.agc = agc;
    }

    /** AOTMARK: entry from V54/V53/V52 etc.; requires MARKSTAT active (>0) else POODOO 00105. */
    public void aotMark() {
        // INHINT
        if (Agc.toSigned(agc.get("MARKSTAT")) > 0) {
            // CCS +: (TC +2 case) mark in use -> abort
            new AlarmAndAbort(agc).poodoo(ALARM_PROG_NOT_ACTIVE);
            return;
        }
        extVbChk();
    }

    /** EXTVBCHK: if extended verb already active (EXTVBACT & 6) bail out, else mark it active. */
    public void extVbChk() {
        if ((agc.get("EXTVBACT") & 06) != 0) {
            mkAbort();
            return;
        }
        agc.set("EXTVBACT", Agc.add(agc.get("EXTVBACT"), BIT2));
        mkVac();
    }

    /** MKVAC: find a free VAC area (VAC1USE..VAC5USE); else BAILOUT1 01207. */
    public void mkVac() {
        for (int i = 1; i <= 5; i++) {
            int use = agc.get("VAC" + i + "USE");
            if (use != 0 && use != Agc.WORD_MASK) {
                mkVacFnd(use);
                return;
            }
        }
        // DXCH BUF2 ; TC BAILOUT1 ; OCT 01207
        new AlarmAndAbort(agc).bailout1();
    }

    /** MKVACFND: MARKSTAT = VAC+2; clear first word; start job GETDAT at priority 15. */
    public void mkVacFnd(int vacUse) {
        int markstat = Agc.add(vacUse, 2);
        agc.set("MARKSTAT", markstat);
        agc.set("VAC_AREA_" + (markstat - 1), 0);     // INDEX MARKSTAT ; TS 0 -1
        // CAF PRIO15 ; FINDVAC 2CADR GETDAT ; RELINT ; TCF SWRETURN
        agc.set("FINDVAC_PRIO", PRIO15);
        agc.set("FINDVAC_JOB", 1);
    }

    /** MKABORT: BAILOUT1 01211 (extended verb already in use). */
    public void mkAbort() {
        new AlarmAndAbort(agc).bailout1();
    }

    /** MKRELEAS: release the mark VAC area, terminate via IBNKCALL GOODEND. */
    public void mkReleas() {
        int ms = agc.get("MARKSTAT");
        agc.set("MARKSTAT", 0);
        int base = ms & LOW9;
        if (base != 0) {
            agc.set("VAC_AREA_" + base, 0);     // INDEX A ; TS 0 : frees the VAC
        }
        // CAF ONE ; IBNKCALL GOODEND
        agc.set("GOODEND_CALLED", 1);
    }

    /** KILLAOT: clear EXTVBACT and exit via GOTOPOOH. */
    public void killAot() {
        agc.set("EXTVBACT", 0);
        // TC GOTOPOOH
    }

    /** GETDAT: set display flag bit12 and request V01N71 (AOT code and detent). */
    public void getDat() {
        agc.set("MARKSTAT", agc.get("MARKSTAT") | (~agc.get("MARKSTAT") & BIT12));
        // GOMARKF V01N71 : terminate -> KILLAOT; proceed -> DODAT; enter -> ENTERDAT(GETDAT)
        agc.set("DISPLAY", V01N71);
    }

    /** DODAT: decode AOTCODE (bits 9-14) into detent XYMARK; 7 = astronaut-entered angles. */
    public void doDat() {
        int detent = ((agc.get("AOTCODE") & HIGH9) * BIT9) >> 14;
        agc.set("XYMARK", detent);
        if (detent == 0 || Agc.toSigned(detent) < 0) {
            getDat();
            return;
        }
        if (detent + NEG7 == 0) {
            code7();
        } else {
            code1to6();
        }
    }

    /** CODE7: V06N87 - azimuth (AZ) and elevation (EL) entered by the astronaut. */
    public void code7() {
        // GOMARKF V06N87: terminate -> KILLAOT; proceed -> continue; enter -> CODE7
        agc.set("FIXLOC+8D", agc.get("AZ"));
        agc.set("FIXLOC+9D", agc.get("AZ+1"));
        coasCode(0);
    }

    /** CODE1TO6: look up AOT elevation (AOTEL) and azimuth (AOTAZ) for detent XYMARK. */
    public void code1to6() {
        int detent = agc.get("XYMARK");
        agc.set("FIXLOC+9D", agc.get("AOTEL+" + (detent - 1)));
        agc.set("FIXLOC+8D", agc.get("AOTAZ+" + (detent - 1)));
        // 10D = AOTAZ+1 - 8D : rotation of the reticle in the detent
        int rot = Agc.add(agc.get("AOTAZ+1"), (~agc.get("FIXLOC+8D")) & Agc.WORD_MASK);
        coasCode(rot);
    }

    /** COASCODE: store 10D and compute reticle-axis vector (OPTAXIS) then go take marks. */
    public void coasCode(int tenD) {
        agc.set("FIXLOC+10D", tenD);
        optAxis();
    }

    /**
     * OPTAXIS: CALL OANB (optical axis in nav base), then the star LOS is rotated about the
     * optical axis by the reticle angle (10D): STARAD+6 = cos(a)*U + sin(a)*(Axis x U)-type
     * combination; then GETMKS.
     */
    public void optAxis() {
        double ang = Agc.toSigned(agc.get("FIXLOC+10D")) * Math.PI / 16384.0;   // SLOAD SR1 10D
        double s = Math.sin(ang);
        double c = Math.cos(ang);
        agc.set("SIN_TMP", (int) (s * 16384));
        agc.set("COS_TMP", (int) (c * 16384));
        // vectors at 12D/18D/24D combined with VXSC/BVSU/VAD; STARAD+6 = LO6ZEROS (clear), then GETMKS
        agc.set("STARAD+6", 0);
        getMks();
    }

    /**
     * OANB: compute optical axis (SCAXIS) in nav base from the shaft (SLOAD 9D) and trunnion (8D)
     * angles via CDULOGIC; also 24D = unit(axis x ...). Returns by GOTO GCTR.
     */
    public void oanb() {
        double shaft = Agc.toSigned(agc.get("FIXLOC+9D")) * 2 * Math.PI / 16384.0;
        double trunnion = Agc.toSigned(agc.get("FIXLOC+8D")) * 2 * Math.PI / 16384.0;
        // PUSH COS ; PDDL SIN ; STADR ; STODL SCAXIS ; 8D ; RTB CDULOGIC ; PUSH COS ; STORE 20D
        agc.set("SCAXIS", (int) (Math.cos(trunnion) * 16384));
        agc.set("SCAXIS+2", (int) (Math.sin(trunnion) * Math.sin(shaft) * 16384 * 2));
        agc.set("SCAXIS+4", (int) (Math.sin(trunnion) * Math.cos(shaft) * 16384 * 2));
        // VXV UNIT SCAXIS -> 24D ; GOTO GCTR
    }

    // ---- SURFSTAR (P50S / R59 bank): surface-star average ----

    /**
     * SURFSTAR: load the CDU spot vector (CDUSPOT) and sextant angles; rotate the star direction
     * about SCAXIS by the angle in 26D, within ABOUTONE/DP1/12 limits; normalise via TRG*NBSM
     * and continue at AVEIT.
     */
    public void surfStar() {
        // VLOAD* 0,1 ; STORE CDUSPOT ; SLOAD* 1,1 ; RTB CDULOGIC ; STORE 24D ; BZE YZCHK
        if (agc.get("FIXLOC+24D") == 0) {
            yzChk();
            return;
        }
        justZy();
    }

    /** JUSTZY: build the star vector from 18D/12D with SCAXIS rotation by 26D. */
    public void justZy() {
        // PUSH COS ; PDDL SIN ; VXSC 18D ; PDDL VXSC 12D ; VSU ; UNIT ; VXV SCAXIS ; UNIT ; PUSH
        // SLOAD* 3,1 ; CDULOGIC ; STORE 26D ; DSU 24D ; DAD ABOUTONE ; DMP DP1/12 ; STORE 26D
        // SIN VXSC VSL1 PDDL 26D ; COS VXSC SCAXIS ; VSL1 VAD
        justOa();
    }

    /** JUSTOA: UNIT, CALL TRG*NBSM, STCALL 24D -> AVEIT. */
    public void justOa() {
        // UNIT ; CALL TRG*NBSM ; STCALL 24D ; AVEIT
        aveIt();
    }

    /** YZCHK: if the Y angle (3,1) is zero use SCAXIS directly (YSZERO), else JUSTZY with 24D. */
    public void yzChk() {
        if (agc.get("CDUSPOT+3") == 0) {
            ySZero();
        } else {
            justZy();
        }
    }

    /** YSZERO: VLOAD SCAXIS ; GOTO JUSTOA. */
    public void ySZero() {
        justOa();
    }

    // ---- Mark taking ----

    /** GETMKS: clear XYMARK and MARKCNTR, keep only base address bits, request V54 (mark). */
    public void getMks() {
        agc.set("XYMARK", 0);
        agc.set("MARKCNTR", 0);
        agc.set("MARKSTAT", agc.get("MARKSTAT") & LOW9);
        passIt(MKVB54_STAR);
    }

    /** PASTIT: GOMARK4 with the verb in A: terminate -> KILLAOT; proceed -> MARKCHEX; enter -> GETDAT. */
    public void passIt(int verb) {
        agc.set("DISPLAY", verb);
    }

    /**
     * MARKCHEX: after PROCEED: set display flag, XYMARK = base; compare mark-type bits (PRIO3 of
     * MARKSTAT) with the channel LCHAN; if all required marks are present average stars
     * (AVESTAR) else count/check marks (CNTCHK).
     */
    public void markChex() {
        int ms = agc.get("MARKSTAT");
        agc.set("MARKSTAT", ms | (~ms & BIT12));
        agc.set("XYMARK", agc.get("MARKSTAT") & LOW9);
        agc.set("MKDEX", 0);
        int l = agc.get("MARKSTAT") & PRIO3;
        agc.set("L", l);
        if (((PRIO3) ^ agc.get("LCHAN")) == 0) {   // RXOR LCHAN ; BZF AVESTAR
            aveStar();
            return;
        }
        cntChk();
    }

    /** CNTCHK: if MARKCNTR is zero raise alarm 111 (MKALARM) else decrement. */
    public void cntChk() {
        int cnt = Agc.toSigned(agc.get("MARKCNTR"));
        if (cnt <= 0) {
            mkAlarm();
            return;
        }
        agc.set("MARKCNTR", cnt - 1);
        aveStar();
    }

    /**
     * AVESTAR: add the next star LOS estimate: MKDEX += BIT12, builds STARAD+6 average from the
     * stored CDU triples (CDUSPOT) with TRG*NBSM, then AVEIT averages with weight 1/n.
     */
    public void aveStar() {
        agc.set("MKDEX", agc.get("MKDEX") + BIT12);
        int cnt = agc.get("MARKCNTR");
        int idx = agc.get("L") - agc.get("XYMARK") - cnt * 6;   // INDEX FIXLOC ; TS X1
        agc.set("X1", idx);
        agc.set("PUSHLOC", agc.get("FIXLOC"));
        // BON SURFFLAG SURFSTAR; else two vectors (12D, 18D) via TRG*NBSM, 24D = unit(12D x 18D)
        if (agc.get("SURFFLAG") != 0) {
            surfStar();
        } else {
            aveIt();
        }
    }

    /**
     * AVEIT: STARAD+6 += (24D * 1/8)/(MKDEX*...) average, store in STARAD+6 and STARSAV2, then
     * loop on MARKCNTR; finally ENDMARKS.
     */
    public void aveIt() {
        // SLOAD MKDEX ; PDVL 24D ; VSR3 V/SC 0 ; STODL 24D ; DSU DP1/8 ; DDV ; VXSC VAD STARAD+6
        agc.set("STARSAV2", agc.get("STARAD+6"));
        if (Agc.toSigned(agc.get("MARKCNTR")) > 0) {     // CCS MARKCNTR
            agc.set("MARKCNTR", agc.get("MARKCNTR") - 1);
            aveStar();
        } else {
            endMarks();
        }
    }

    /** ENDMARKS: waitlist MKRELEAS after 50 ms and end the mark sequence. */
    public void endMarks() {
        // CAF FIVE ; INHINT ; WAITLIST 2CADR MKRELEAS ; TC ENDMARK
        agc.set("WAITLIST_MKRELEAS", 5);
    }

    /** MKALARM: ALARM 111 (mark area not set) then back to GETMKS. */
    public void mkAlarm() {
        new AlarmAndAbort(agc).alarm(ALARM_NO_MARK_DEVICE, agc.get("Q"));
        getMks();
    }

    // ---- MARK interrupt ----

    /**
     * MARKRUPT: Mark button interrupt. Saves CDU Y/Z/X into ITEMP3-5 and TIME2 into ITEMP1/2,
     * reads the NAVKEYIN channel and dispatches on which key (X mark / Y mark / reject).
     */
    public void markRupt() {
        agc.set("ITEMP3", agc.get("CDUY"));
        agc.set("ITEMP4", agc.get("CDUZ"));
        agc.set("ITEMP5", agc.get("CDUX"));
        agc.set("ITEMP1", agc.get("TIME2"));
        agc.set("ITEMP2", agc.get("TIME2+1"));
        agc.set("QRUPT", agc.get("Q"));
        int navkeyin = agc.get("NAVKEYIN");
        if ((navkeyin & OCT34) != 0 ? false : true) {
            // RAND OCT34 == 0 -> some key
            someKey();
            return;
        }
        if ((agc.get("MARKSTAT") & BIT12) != 0) {
            return;     // RESUME (marking disabled while displaying)
        }
        if (Agc.toSigned(agc.get("MARKSTAT")) > 0) {
            findKey();
        } else {
            new AlarmAndAbort(agc).alarm(ALARM_MARK_NOT_EXPECTED, 0);
        }
    }

    /** FINDKEY: reject (BIT5) first, then Y mark (BIT4), then X mark (BIT3). */
    public void findKey() {
        int k = agc.get("NAVKEYIN");
        if ((k & BIT5) != 0) {
            mkRej();
        } else if ((k & BIT4) != 0) {
            yMkRupt();
        } else if ((k & BIT3) != 0) {
            xMkRupt();
        } else {
            someKey();
        }
    }

    /** SOMEKEY: if keys in OCT140 are all off go to DESCBITS; else ALARM 113. */
    public void someKey() {
        if ((agc.get("NAVKEYIN") & OCT140) == 0) {
            // POSTJUMP DESCBITS
            return;
        }
        new AlarmAndAbort(agc).alarm(ALARM_KEY_BAD, 0);
    }

    /** XMKRUPT: X mark: RUPTREG1 = 0, XYMARK = BIT10 (X) flag. */
    public void xMkRupt() {
        agc.set("RUPTREG1", 0);
        markCommon(BIT10);
    }

    /** YMKRUPT: Y mark: RUPTREG1 = 1, XYMARK = BIT11 (Y) flag. */
    public void yMkRupt() {
        agc.set("RUPTREG1", 1);
        markCommon(BIT11);
    }

    private void markCommon(int xy) {
        agc.set("XYMARK", xy);
        if (markTypeIsSurface()) {
            surfStor();
            return;
        }
        if ((agc.get("MARKSTAT") & BIT14) != 0) {
            // both X&Y mark allowed: check mark counter < 5 (else 5MKALARM) and increment
            if (agc.get("MARKCNTR") + 1 - 4 > 0 ) {
                fiveMkAlarm();
                return;
            }
            agc.set("MARKCNTR", agc.get("MARKCNTR") + 1);
            agc.set("MARKSTAT", agc.get("MARKSTAT") & ~PRIO23);
        }
        verifyMk();
    }

    /** VERIFYMK: if this mark type already done ALARM 114 else store (VACSTOR). */
    public void verifyMk() {
        if ((agc.get("XYMARK") & agc.get("MARKSTAT")) == 0) {
            vacStor();
        } else {
            new AlarmAndAbort(agc).alarm(ALARM_BAD_MARK, 0);
        }
    }

    /** 5MKALARM: ALARM 107; surface -> DSPV6N79, else RESUME. */
    public void fiveMkAlarm() {
        new AlarmAndAbort(agc).alarm(ALARM_TOO_MANY_MARKS, 0);
        if (markTypeIsSurface()) {
            dspV6N79();
        }
    }

    /** MKREJ: reject key. Surface -> SURFREJ; else clear last mark (REJECT) or ALARM 115. */
    public void mkRej() {
        if (markTypeIsSurface()) {
            surfRej();
            return;
        }
        if ((agc.get("MARKSTAT") & PRIO3) != 0) {
            reject();
        } else {
            rejAlm();
        }
    }

    /** REJALM: ALARM 115. */
    public void rejAlm() {
        new AlarmAndAbort(agc).alarm(ALARM_REJECT_BAD, 0);
    }

    /** REJECT: clear mark-done bits, toggling BIT13, then RENEWMK/REJECT2 and REMARK. */
    public void reject() {
        int ms = (agc.get("MARKSTAT") & ~PRIO30) + BIT13;
        agc.set("MARKSTAT", ms);
        if ((ms & BIT13) != 0) {
            // REJECT2: clear PRIO3 bits
            renewMk(~PRIO3);
        } else {
            renewMk(~agc.get("XYMARK"));
        }
    }

    /** RENEWMK: MARKSTAT &= mask ; REMARK. */
    public void renewMk(int mask) {
        agc.set("MARKSTAT", agc.get("MARKSTAT") & mask);
        reMark();
    }

    /** SURFREJ: surface reject: decrement MARKCNTR or alarm. */
    public void surfRej() {
        int cnt = Agc.toSigned(agc.get("MARKCNTR"));
        if (cnt > 0) {
            agc.set("MARKCNTR", cnt - 1);
        } else {
            rejAlm();
        }
    }

    /** MARKTYPE: skip-return: true when FLAGWRD8 bit 8 (SURFFLAG) is set (surface mark). */
    public boolean markTypeIsSurface() {
        return (agc.get("FLAGWRD8") & BIT8) != 0;
    }

    /** SURFSTOR: surface store: RUPTREG1 = 0 and add PRIO3 into MARKSTAT; then VACSTOR. */
    public void surfStor() {
        agc.set("RUPTREG1", 0);
        agc.set("MARKSTAT", agc.get("MARKSTAT") + (~agc.get("MARKSTAT") & PRIO3));
        vacStor();
    }

    /**
     * VACSTOR: store mark time (TSIGHT) and the three CDU angles into the VAC area at
     * MARKSTAT base + 6*MARKCNTR (Y at +RUPTREG1 offset), then update MARKSTAT mark-done bits.
     */
    public void vacStor() {
        int base = agc.get("MARKSTAT") & LOW9;
        agc.set("RUPTREG2", base);
        agc.set("TSIGHT", agc.get("ITEMP1"));
        agc.set("TSIGHT+1", agc.get("ITEMP2"));
        int offset = agc.get("MARKCNTR") * 6 + base + agc.get("RUPTREG1");
        agc.set("RUPTREG1", offset);
        agc.set("MKDEX", offset);
        agc.set("VAC_AREA_" + offset, agc.get("ITEMP3"));
        agc.set("VAC_AREA_" + (offset + 2), agc.get("ITEMP4"));
        agc.set("VAC_AREA_" + (offset + 4), agc.get("ITEMP5"));
        if (markTypeIsSurface()) {
            surfJob();
            return;
        }
        int ms = (agc.get("MARKSTAT") & ~(BIT13 + agc.get("XYMARK"))) + agc.get("XYMARK");
        agc.set("MARKSTAT", ms);
        if (((ms & PRIO3) ^ PRIO3) != 0) {
            reMark();
        } else {
            agc.set("MARKSTAT", ms | (~ms & BIT14));
            reMark();
        }
    }

    /** REMARK: MKDEX = (MARKSTAT & PRIO3) * BIT6 scaling, then SURFJOB. */
    public void reMark() {
        agc.set("MKDEX", ((agc.get("MARKSTAT") & PRIO3) * BIT6) >> 14);
        surfJob();
    }

    /** SURFJOB: start job CHANGEVB (priority 15) via NOVAC and RESUME. */
    public void surfJob() {
        agc.set("NOVAC_PRIO", PRIO15);
        agc.set("NOVAC_JOB", 1);    // 2CADR CHANGEVB
    }

    /** CHANGEVB: set the next mark verb: surface -> DSPV6N79; else MKVB5x indexed by MKDEX. */
    public void changeVb() {
        if (markTypeIsSurface()) {
            dspV6N79();
            return;
        }
        int[] vb = {MKVB54, MKVB53, MKVB52};
        passIt(vb[Math.min(agc.get("MKDEX"), 2)]);
    }

    // ---- surface display ----

    /** DSPV6N79: display V06N79 (cursor/spiral), loops until the astronaut keys PROCEED. */
    public void dspV6N79() {
        agc.set("DISPLAY", V06N79);
        // terminate -> KILLAOT, proceed -> SURFEND, enter -> check BIT6 of MPAC -> SURFAGAN else repeat
        if ((agc.get("MPAC") & BIT6) != 0) {
            surfAgan();
        }
    }

    /** SURFEND: set BIT14 of MARKSTAT (done) then SURFAGAN. */
    public void surfEnd() {
        agc.set("MARKSTAT", (agc.get("MARKSTAT") & ~BIT14) + BIT14);
        surfAgan();
    }

    /** SURFAGAN: store CURSOR and SPIRAL at MKDEX+1 / MKDEX+3; continue marking or finish. */
    public void surfAgan() {
        int mk = agc.get("MKDEX");
        agc.set("VAC_AREA_" + (mk + 1), agc.get("CURSOR"));
        agc.set("VAC_AREA_" + (mk + 3), agc.get("SPIRAL"));
        if ((agc.get("MARKSTAT") & BIT14) == 0) {
            markChex();
            return;
        }
        int cnt = agc.get("MARKCNTR") + 1;
        if (5 - cnt <= 0) {
            fiveMkAlarm();
            return;
        }
        agc.set("MARKCNTR", cnt);
        getMks();
    }
}
