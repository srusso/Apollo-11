package apollo.luminary099;

import apollo.agc.Agc;

/**
 * Translation of KEYRUPT_UPRUPT.agc (Luminary 1A build 099).
 *
 * KEYRUPT1: DSKY key-press interrupt. UPRUPT: uplink interrupt, with triple-redundancy
 * checking of the 15-bit uplink word. A bad code locks out uplink (UPLOCKFL, bit 4 of
 * FLAGWRD7) until 'ERROR RESET' (code 22 octal) is received. The uplink-activity light is
 * turned off by VBRELDSP, ERROR RESET, or entering P27 via V70-V73.
 */
public class KeyruptUprupt {
    private static final int LOW5 = 037;
    private static final int BIT3 = 04;
    private static final int UPLOCBIT = 010;        // bit 4
    private static final int HI10 = 077740;         // OCT 77740
    private static final int ELRCODE = 022;         // 'ERROR RESET' code

    private final Agc agc;

    public KeyruptUprupt(Agc agc) {
        this.agc = agc;
    }

    /** KEYRUPT1: keyboard interrupt entry. */
    public void keyrupt1() {
        agc.set("BANKRUPT", agc.get("A"));
        agc.set("QRUPT", agc.get("Q"));
        new PinballGameButtonsAndLights(agc).lodsampt(); // time snatched for noun 65
        int key = agc.get("MNKEYIN") & LOW5;             // keys 5M-1M
        keycom(key);
    }

    /** KEYCOM: record the key code, set the DSKY-flag bit and schedule CHARIN. */
    public void keycom(int key) {
        agc.set("RUPTREG4", key);                        // RUPTREG4 = KEYTEMP1
        int flag = agc.get("FLAGWRD5");
        int dskyfbit = agc.get("DSKYFBIT");
        agc.set("FLAGWRD5", flag | dskyfbit);            // CS/MASK/ADS sets the bit
        acceptup();
    }

    /** ACCEPTUP: NOVAC the CHARIN job (priority CHRPRIO), leaving the key code in MPAC. */
    public void acceptup() {
        new Executive(agc).novac(agc.get("CHRPRIO"), () -> new PinballGameButtonsAndLights(agc).charin());
        agc.set("MPAC", agc.get("RUPTREG4"));            // 5-bit key code for CHARIN
        new Waitlist(agc).resume();
    }

    /** UPRUPT: uplink interrupt entry. */
    public void uprupt() {
        agc.set("BANKRUPT", agc.get("A"));
        agc.set("QRUPT", agc.get("Q"));
        new PinballGameButtonsAndLights(agc).lodsampt();
        int inlink = agc.get("INLINK");
        agc.set("INLINK", 0);
        agc.set("KEYTEMP1", inlink);
        agc.set("DSALMOUT", agc.get("DSALMOUT") | BIT3); // turn on UPACT light (channel 11 bit 3)
        uprpt1();
    }

    /** UPRPT1: test for triple character redundancy. */
    public void uprpt1() {
        int word = agc.get("KEYTEMP1");
        int low = word & LOW5;
        agc.set("KEYTEMP1", low);
        int shifted = (word >> 5) & 03777;   // MP BIT10 shifts right 5
        agc.set("KEYTEMP2", shifted);
        int mid = shifted & LOW5;
        if (!uptest(mid + HI10)) return;     // mid must equal complement of low
        int high = (shifted >> 5) & LOW5;
        if (!uptest(~high & Agc.WORD_MASK)) return;
        upck();
    }

    /**
     * UPTEST: add KEYTEMP1 to the argument; the sum must be -0 (all ones) for a good code.
     * Otherwise the code is bad and TMFAIL2 is taken.
     * @return true if the code is good
     */
    public boolean uptest(int a) {
        int sum = Agc.add(a & Agc.WORD_MASK, agc.get("KEYTEMP1"));
        // CCS A: positive -> TMFAIL2; +0 -> second word; -0/negative non-zero -> TMFAIL2; -0 OK
        if (sum == Agc.WORD_MASK) return true;
        tmfail2();
        return false;
    }

    /** UPCK: code is good; ERROR RESET clears UPLOCKFL, other codes accepted only if clear. */
    public void upck() {
        if (agc.get("KEYTEMP1") == ELRCODE) {
            clUplock();
            return;
        }
        if ((agc.get("FLAGWRD7") & UPLOCBIT) != 0) {
            new Waitlist(agc).resume();      // UPLOCKFL = 1: ignore
        } else {
            acceptup();                      // UPLOCKFL = 0: accept
        }
    }

    /** CLUPLOCK: clear UPLOCKFL (bit 4 of FLAGWRD7) and accept the code. */
    public void clUplock() {
        agc.set("FLAGWRD7", agc.get("FLAGWRD7") & ~UPLOCBIT);
        acceptup();
    }

    /** TMFAIL2: bad code; lock out further uplink activity until ERROR RESET. */
    public void tmfail2() {
        agc.set("FLAGWRD7", agc.get("FLAGWRD7") | UPLOCBIT);
        new Waitlist(agc).resume();
    }
}
