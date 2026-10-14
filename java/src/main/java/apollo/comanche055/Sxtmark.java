package apollo.comanche055;

import apollo.agc.Agc;

/**
 * SXTMARK.agc - sextant marking system: SXTMARK (reserve mark system), MKRELEAS, MARKRUPT,
 * MARKCONT, MKREJECT, MKVB51/MKVB50 displays, MARKIT/MARKDIF (rate check) and DIFCHK.
 *
 * Basic-language routines operate on erasable symbols through {@link Agc#get}/{@link Agc#set}.
 * Rupt routines end in RESUME and waitlist tasks in TASKOVER; these are modelled by
 * {@link #resume()} and {@link #taskover()}.
 */
public final class Sxtmark {
    // Bit masks
    private static final int BIT1 = 1, BIT2 = 2, BIT6 = 040, BIT7 = 0100, BIT9 = 0400, BIT10 = 01000,
        BIT12 = 04000, BIT14 = 020000, LOW9 = 0777, LOW5 = 037;
    /** PRIO34 mask: bits 13-14 of MARKSTAT (marks wanted field bits). */
    private static final int PRIO34 = 034000;
    /** 74K: -(BIT12) i.e. decrement marks wanted field. */
    private static final int K74 = 074000;
    private static final int PRIO32 = 032000, PRIO5 = 05000;
    /** VN codes. */
    public static final int V50N25 = 5025, VB51 = 5100;
    public static final int R1D1 = 016;
    /** Alarm / abort codes. */
    public static final int ALARM_MARK_NOT_CALLED = 0122, ALARM_MARK_NOT_WANTED = 0114,
        ALARM_NO_MARK_ACCEPTED = 0112, ALARM_TWO_REJECTS = 0110, ALARM_NO_INBITS = 0113,
        ALARM_RATE_TOO_HIGH = 00121, ABORT_MARK_IN_USE = 01211, ABORT_NO_VAC = 01207;

    private final Agc agc;
    private final AlarmAndAbort alarms;

    public Sxtmark(Agc agc) {
        this.agc = agc;
        this.alarms = new AlarmAndAbort(agc);
    }

    private int g(String s) { return agc.get(s); }
    private void s(String n, int v) { agc.set(n, v); }

    /** CCS result as nonzero/positive test: true if |word| &gt; 0 (word &gt; +0). */
    private static boolean positive(int w) {
        w &= Agc.WORD_MASK;
        return w != 0 && w != Agc.WORD_MASK && (w & Agc.SIGN_BIT) == 0;
    }

    private void resume() { s("RESUME_PENDING", 1); }
    private void taskover() { s("TASKOVER_PENDING", 1); }

    /**
     * SXTMARK: reserve a VAC area for marking and request MKVB51.
     * @param marks number of mark requests (bits 1-3 of A)
     */
    public void sxtmark(int marks) {
        s("INHINT", 1);
        s("RUPTREG1", marks);                           // NUMBER OF MARKS WANTED
        if ((g("EXTVBACT") & 06) != 0) {                // BIT2 = marking in use, BIT3 = extended verb
            mkabort();
            return;
        }
        s("EXTVBACT", g("EXTVBACT") + BIT2);            // SET IT, RESET IN ENDMARK
        markok();
    }

    /** MKABORT: BAILOUT 01211. */
    public void mkabort() { alarms.bailout(ABORT_MARK_IN_USE); }

    /** MARKOK: find a free VAC area (VAC1USE..VAC5USE). */
    public void markok() {
        String[] use = {"VAC1USE", "VAC2USE", "VAC3USE", "VAC4USE", "VAC5USE"};
        for (String u : use) {
            if (positive(g(u))) { mkvacfnd(g(u)); return; }  // CCS: A holds |VACnUSE|-1 (address of VAC area)
        }
        alarms.bailout(ABORT_NO_VAC);                   // OCT 01207
    }

    /** MKVACFND: A = VAC area address - 2 on entry. */
    public void mkvacfnd(int a) {
        int addr = Agc.add(a, 2);
        s("MARKSTAT", addr);                            // ADDRESS OF VAC AREA
        s("QPRET", addr);                               // NEXT AVAILABLE MARK SLOT (QPRET = VAC pointer)
        s("VAC@" + addr + "-1", 0);                     // STORE VAC AREA OCCUPIED (word 1 = +0)
        if (checkmm(53) || checkmm(54)) {               // backup mark routine uses SXTMARK
            swreturn();
            return;
        }
        int wanted = g("RUPTREG1");
        s("MARKSTAT", g("MARKSTAT") + ((BIT12 * wanted) & Agc.WORD_MASK));   // DESIRED NUMBER OF MARKS IN BITS 12-14
        novacMkvb51();
        s("RELINT", 1);
        swreturn();                                     // SAME AS MODEEXIT
    }

    private boolean checkmm(int mm) { return g("MODREG") == mm; }
    private void swreturn() { s("SWRETURN_PENDING", 1); }
    private void novacMkvb51() { s("NOVAC", PRIO32); s("NOVAC_JOB_MKVB51", 1); }
    private void novacMkvb50() { s("NOVAC", PRIO32); s("NOVAC_JOB_MKVB50", 1); }

    /** MKRELEAS: release the mark system. Returns via SWRETURN. */
    public void mkreleas() {
        int ms = g("MARKSTAT");
        s("MARKSTAT", 0);                               // SHOW MARK SYSTEM NOW AVAILABLE
        int vac = ms & LOW9;
        if (vac != 0) s("VAC@" + vac, vac);             // 1ST WORD OF VAC AREA = ITS ADDRESS
        mkrlees();
    }

    /** MKRLEES: kill coarse optics, disable optics error counter. */
    public void mkrlees() {
        s("INHINT", 1);
        s("OPTMODES", g("OPTMODES") & ~BIT9);           // COARSE OPTICS RETURN FLAG
        s("OPTIND", Agc.WORD_MASK - 0);                 // -1 would be 077776; NEGONE
        s("OPTIND", Agc.fromSigned(-1));                // KILL COARSE OPTICS
        s("CHAN12", g("CHAN12") & ~BIT2);               // DISABLE OPTICS ERROR COUNTER
        s("RELINT", 1);
        swreturn();
    }

    /**
     * MARKRUPT: keyrupt2 entry for MARK, MARK REJECT and DSKY keys. Stores CDUs and time.
     */
    public void markrupt() {
        s("MKCDUT", g("CDUT"));
        s("MKCDUS", g("CDUS"));
        s("MKCDUY", g("CDUY"));
        s("MKCDUZ", g("CDUZ"));
        s("MKCDUX", g("CDUX"));
        s("MKT2T1", g("TIME2"));  s("MKT2T1+1", g("TIME1"));
        s("SAMPTIME", g("MKT2T1")); s("SAMPTIME+1", g("MKT2T1+1"));   // RUPT TIME FOR NOUN 65
        s("QRUPT", g("Q"));
        int navkeyin = g("NAVKEYIN");
        if ((navkeyin & BIT6) != 0) { markit(); return; }          // ITS A MARK
        if ((navkeyin & BIT7) != 0) { mkreject(); return; }        // ITS A MARK REJECT
        keycall(navkeyin);
    }

    /** KEYCALL: not a mark; test for key code. */
    public void keycall(int navkeyin) {
        if ((navkeyin & LOW5) != 0) {
            new KeyruptUprupt(agc).keycom(navkeyin & LOW5);        // POSTJUMP CADR KEYCOM
            return;
        }
        new AlarmAndAbort(agc).alarm(ALARM_NO_INBITS);             // ALARM IF NO INBITS (OCT 113)
        resume();
    }

    /**
     * MARKCONT: special R21 mark, special display, or normal mark, selected by STATE+2 bit 14
     * and MARKSTAT.
     */
    public void markcont() {
        if ((g("STATE+2") & BIT14) != 0) { markit1(); return; }    // R21 SPECIAL MARKING
        market();
    }

    /** MARKIT1: R21 - move MRKBUF1 to MRKBUF2 and current mark data to MRKBUF1. */
    public void markit1() {
        gentran("MRKBUF1", "MRKBUF2", 7);
        gentran("MKT2T1", "MRKBUF1", 7);
        taskover();
    }

    /** MARKET: regular marking. */
    public void market() {
        if (positive(g("MARKSTAT"))) { mark2(); return; }          // CCS MARKSTAT: marks called for
        if ((g("EXTVBACT") & BIT2) == 0 || (g("STATE+5") & BIT12) == 0) {   // MARKET3
            new AlarmAndAbort(agc).alarm(ALARM_MARK_NOT_CALLED);
            taskover();
            return;
        }
        s("NOVAC", PRIO5);                                         // CALIBRATION MARK FOR P23: MARKDISP
        s("NOVAC_JOB_MARKDISP", 1);
        gentran("MKT2T1", "MARKDOWN", 7);
        gentran("MKT2T1", "MRKBUF1", 7);                           // FOR SPECIAL DISPLAY IF V59 ACTING
        taskover();
    }

    /** 114ALM: mark not wanted. */
    public void alm114() {
        new AlarmAndAbort(agc).alarm(ALARM_MARK_NOT_WANTED);
        taskover();
    }

    /** MARK2: reduce marks wanted, set reject bit, store mark data in VAC and advance pointer. */
    public void mark2() {
        int a = Agc.add(g("MARKSTAT"), K74);                       // AD 74K: reduce marks wanted
        if (Agc.toSigned(a) <= 0) { alm114(); return; }            // BZMF
        s("MARKSTAT", a);
        s("MARKSTAT", Agc.add(a, (~a) & BIT10));                   // SET BIT10 TO ENABLE REJECT
        s("ITEMP1", g("MARKSTAT") & LOW9);
        int slot = g("QPRET");                                     // pick up mark slot pointer (INDEX)
        s("ITEMP2", slot);
        s("QPRET", Agc.add(slot, 7));                              // INCREMENT POINTER
        vacstor(slot);
    }

    /** VACSTOR: store TIME2/TIME1, CDUY, CDUS, CDUZ, CDUT, CDUX at slot..slot+6. */
    public void vacstor(int slot) {
        s("VAC@" + slot, g("MKT2T1"));
        s("VAC@" + (slot + 1), g("MKT2T1+1"));
        s("VAC@" + (slot + 2), g("MKCDUY"));
        s("VAC@" + (slot + 3), g("MKCDUS"));
        s("VAC@" + (slot + 4), g("MKCDUZ"));
        s("VAC@" + (slot + 5), g("MKCDUT"));
        s("VAC@" + (slot + 6), g("MKCDUX"));
        if ((g("MARKSTAT") & PRIO34) == 0) {                       // ALL MARKS MADE: FLASH VB50
            novacMkvb50();
        }
        taskover();
    }

    /** GENTRAN: block transfer of {@code n} words. */
    private void gentran(String from, String to, int n) {
        for (int i = 0; i < n; i++) {
            s(i == 0 ? to : to + "+" + i, g(i == 0 ? from : from + "+" + i));
        }
    }

    /** MKREJECT: reject a mark. */
    public void mkreject() {
        if ((g("STATE+2") & BIT14) != 0) {                         // R21
            s("MRKBUF1", Agc.fromSigned(-1));                      // -1: FLAG TO R22 SIGNIFYING A REJECTED MARK
            resume();
            return;
        }
        mrkrejct();
    }

    /** MRKREJCT: regular reject. */
    public void mrkrejct() {
        if (!positive(g("MARKSTAT"))) {                            // MARKS NOT BEING ACCEPTED
            new AlarmAndAbort(agc).alarm(ALARM_NO_MARK_ACCEPTED);
            resume();
            return;
        }
        reject2();
    }

    /** REJECT2: clear bit 10; require that a mark was made since last reject. */
    public void reject2() {
        int old = g("MARKSTAT");
        s("MARKSTAT", old & ~BIT10);
        if ((old & BIT10) == 0) {                                  // DONT ACCEPT TWO REJECTS TOGETHER
            new AlarmAndAbort(agc).alarm(ALARM_TWO_REJECTS);
            resume();
            return;
        }
        reject3();
    }

    /** REJECT3: decrement QPRET by 7, increment marks wanted, switch to MKVB51 if more wanted. */
    public void reject3() {
        int ms = g("MARKSTAT") & LOW9;
        s("ITEMP1", ms);
        s("QPRET", Agc.add(g("QPRET"), Agc.fromSigned(-7)));       // NEW POINTER
        int m = Agc.add(BIT12, g("MARKSTAT"));                     // INCREMENT MARKS WANTED
        s("MARKSTAT", m);
        if ((m & PRIO34) != 0) { resume(); return; }
        novacMkvb51();                                             // CHANGE TO VB51: MORE MARKS WANTED
        resume();
    }

    /** MKVB51: flash V51 (request marking); on V34 terminate, V33 proceed, ENTER recycle. */
    public void mkvb51(int response) {
        s("KLEENEX", 1);                                           // CLEAR DISPLAY FOR MARK VERB
        s("GOMARK4", VB51);                                        // DISPLAY MARK VB51
        switch (response) {
            case 0 -> termsxt();                                   // VB34 TERMINATE
            case 1 -> entanswr();                                  // V33 PROCEED - MARKING DONE
            default -> mkvb5x();                                   // ENTER - RECYCLE
        }
    }

    /** TERMSXT: clear mark activity; in P03 set TRM03FLG, else go to TERM52. */
    public void termsxt() {
        s("CLEARMRK", 1);
        if (g("MODREG") == 3) {
            s("TRM03FLG", 1);                                      // TERMP03: UPFLAG TRM03FLG
            entanswr();
        } else {
            s("TERM52", 1);                                        // POSTJUMP CADR TERM52
        }
    }

    /** ENTANSWR: put VAC address in MARKSTAT and number of marks made in QPRET. */
    public void entanswr() {
        s("MARKSTAT", g("MARKSTAT") & LOW9);
        int ms = g("MARKSTAT");
        int diff = Agc.add(~ms & Agc.WORD_MASK, g("QPRET"));       // COM; INDEX MARKSTAT; AD QPRET
        int result;
        if (Agc.toSigned(diff) <= 0) {                             // NO MARKS MADE
            result = diff;
        } else {
            result = Agc.add(diff / BIT12, 1);                     // NUMBER MARKS-1 + 1 (via MP BIT12)
        }
        s("QPRET", result);                                        // JAMIT: store number of marks made
        s("INHINT", 1);
        s("WAITLIST_ENDMARKS", 5);                                 // CAF FIVE; WAITLIST ENDMARKS
        s("ENDMARK", 1);                                           // KNOCKS DOWN MARKING FLAG + ENDOFJOB
    }

    /** ENDMARKS: waitlist task - call GOODEND via IBNKCALL then re-display. */
    public void endmarks() {
        s("GOODEND", 1);
        mkvb5x();
    }

    /** MKVB5X: re-display VB51 if more marks wanted and VB50 if all in. */
    public void mkvb5x() {
        if ((g("MARKSTAT") & PRIO34) != 0) { mkvb51(2); return; }
        mkvb50();
    }

    /** MKVB50: V50 N25 R1=16 request terminate marking. */
    public void mkvb50() {
        s("DSPTEM1", R1D1);
        s("GOMARK4", V50N25);
        mkvb51(2);
    }

    /** MARKIT: channel 16 bit 6 (MARK). Waitlist MARKDIF after CDUCHKWD delay (10 ms if none). */
    public void markit() {
        int w = g("CDUCHKWD");
        int delay = positive(w) ? Agc.add(w, Agc.fromSigned(-1)) : 1;   // DELAY OF CDUCHKWD CS IF PNZ
        s("WAITLIST_MARKDIF", delay);
        resume();
    }

    /** MARKDIF: waitlist task; rate check on CDUX, CDUY, CDUZ, then MARKCONT. */
    public void markdif() {
        if (Agc.toSigned(g("CDUCHKWD")) > 0) {
            s("MKNDX", Agc.fromSigned(-1));
            if (!difchk(g("MKCDUX"), g("CDUX"))) return;
            if (!difchk(g("MKCDUY"), g("CDUY"))) return;
            if (!difchk(g("MKCDUZ"), g("CDUZ"))) return;
        }
        markcont();                                                 // MKACPT
    }

    /**
     * DIFCHK: compares mark CDU with the current CDU; more than 3 bits difference
     * gives alarm 00121 and TASKOVER (mark not accepted). Returns true if acceptable.
     */
    public boolean difchk(int mark, int current) {
        int d = Math.abs(Agc.toSigned(Agc.add(mark, Agc.fromSigned(-Agc.toSigned(current)))));
        if (d - 2 <= 0 || d == 0) return true;                      // NOT GREATER (than 3 bits)
        new AlarmAndAbort(agc).alarm(ALARM_RATE_TOO_HIGH);
        taskover();
        return false;
    }
}
