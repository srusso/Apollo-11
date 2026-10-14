package apollo.comanche055;

import apollo.agc.Agc;

/**
 * KEYRUPT_UPRUPT.agc - keyboard (KEYRUPT1) and uplink (UPRUPT) interrupt handlers.
 * Each interrupt returns to the interrupted program via RESUME.
 */
public final class KeyruptUprupt {
    /** ELRCODE: 'ERROR RESET' uplink code. */
    public static final int ELRCODE = 022;
    /** HI10: OCT 77740 */
    public static final int HI10 = 077740;
    private static final int LOW5 = 037;
    private static final int BIT4 = 010;
    private static final int BIT3 = 04;
    private static final int BIT10 = 01000;
    private static final int BIT15 = 040000;

    private final Agc agc;

    public KeyruptUprupt(Agc agc) { this.agc = agc; }

    /** Hook for channel 15 (MNKEYIN) reads; supplied by the host environment. */
    public int readMnkeyin() { return agc.get("MNKEYIN"); }

    /** KEYRUPT1: save rupt state, read 5-bit key code from MNKEYIN. */
    public void keyrupt1() {
        agc.set("BANKRUPT", agc.get("A"));
        agc.set("QRUPT", agc.get("Q"));
        lodsampt();                                  // TIME IS SNATCHED IN RUPT FOR NOUN 65
        keycom(readMnkeyin() & LOW5);
    }

    /** KEYCOM: store key code, flip bit 15 of FLAGWRD5, fall into ACCEPTUP. */
    public void keycom(int key) {
        agc.set("RUPTREG4", key);
        int f = agc.get("FLAGWRD5");
        // CS FLAGWRD5; MASK BIT15; ADS FLAGWRD5  -> toggles bit 15
        agc.set("FLAGWRD5", f ^ BIT15);
        acceptup();
    }

    /** ACCEPTUP: schedule CHARIN via NOVAC at CHRPRIO with key code left in MPAC. */
    public void acceptup() {
        // CAF CHRPRIO; TC NOVAC; 2CADR CHARIN
        agc.set("NOVAC_PRIORITY", agc.get("CHRPRIO"));
        agc.set("NOVAC_TARGET_CHARIN", 1);
        agc.set("MPAC", agc.get("RUPTREG4"));        // INDEX LOCCTR; TS MPAC
        resume();
    }

    /** UPRUPT: uplink interrupt; checks triple character redundancy of INLINK word. */
    public void uprupt() {
        agc.set("BANKRUPT", agc.get("A"));
        agc.set("QRUPT", agc.get("Q"));
        lodsampt();
        int inlink = agc.get("INLINK");
        agc.set("INLINK", 0);
        agc.set("KEYTEMP1", inlink);
        agc.set("DSALMOUT", agc.get("DSALMOUT") | BIT3);   // UPACT light on
        uprpt1();
    }

    /** UPRPT1: test triple char redundancy. */
    public void uprpt1() {
        int word = agc.get("KEYTEMP1");
        int low = word & LOW5;
        agc.set("KEYTEMP1", low);
        int k2 = (word >> 5) & 0x3FF;                 // shift right 5 (MP BIT10)
        agc.set("KEYTEMP2", k2);
        int mid = k2 & LOW5;
        if (!uptest(Agc.add(mid, HI10))) { tmfail2(); return; }
        int high = (k2 >> 5) & LOW5;
        if (!uptest(Agc.fromSigned(-high) & Agc.WORD_MASK)) { tmfail2(); return; }
        upok();
    }

    /**
     * UPTEST: adds KEYTEMP1 to the argument; a good code gives -0 (all ones), i.e. the
     * TC TMFAIL2 paths of CCS A are taken for non-zero results. Returns true if good.
     */
    private boolean uptest(int a) {
        int sum = Agc.add(a, agc.get("KEYTEMP1"));
        // CCS A: +0 / -0 skips to the "good" return via TC Q; otherwise TMFAIL2
        return sum == 0 || sum == Agc.WORD_MASK;
    }

    /** UPOK: code is good. */
    public void upok() {
        if (agc.get("KEYTEMP1") == ELRCODE) { cluplock(); return; }
        if ((agc.get("FLAGWRD7") & BIT4) != 0) { resume(); return; }   // UPLOCKFL = 1
        acceptup();                                                     // UPLOCKFL = 0
    }

    /** CLUPLOCK: clear UPLOCKFL (bit 4 of FLAGWRD7), then accept code. */
    public void cluplock() {
        agc.set("FLAGWRD7", agc.get("FLAGWRD7") & ~BIT4);
        acceptup();
    }

    /** TMFAIL2: bad code; lock out further uplink activity (UPLOCKFL = 1) until ERROR RESET. */
    public void tmfail2() {
        agc.set("FLAGWRD7", agc.get("FLAGWRD7") | BIT4);
        resume();
    }

    /** LODSAMPT: snatch TIME2/TIME1 into SAMPTIME (for noun 65). */
    private void lodsampt() {
        agc.set("SAMPTIME", agc.get("TIME2"));
        agc.set("SAMPTIME+1", agc.get("TIME1"));
    }

    /** RESUME: restore registers saved at interrupt entry and return to the interrupted program. */
    private void resume() {
        agc.set("RESUME_PENDING", 1);
    }
}
