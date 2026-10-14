package apollo.comanche055;

import apollo.agc.Agc;

/**
 * ALARM_AND_ABORT.agc (Comanche 055).
 * ALARM, BAILOUT, POODOO, ABORT, CURTAINS, CCSHOLE, VARALARM, PRIOLARM.
 *
 * Return addresses (Q / ALMCADR) are modelled by the caller simply continuing after
 * the call; the "alarm code" is the word that follows the TC ALARM in the original.
 */
public final class AlarmAndAbort {
    public static final int OCT40400 = 0x4100; // 040400 octal
    public static final int OCT77770 = 077770;
    public static final int OCT1103 = 01103;   // CCSHOLE alarm code
    public static final int OCT217 = 0217;     // CURTAINS alarm code
    public static final int OCT14 = 014;
    public static final int NEG2SEC = -200;    // -2SEC DEC -200
    public static final int BIT15 = 0x4000;
    public static final int POSMAX = 037777;

    private final Agc agc;
    private final Executive executive;

    public AlarmAndAbort(Agc agc) {
        this.agc = agc;
        this.executive = new Executive(agc);
    }

    /** ALARM: record alarm code (caller's following word) and set the program alarm light. */
    public void alarm(int code) {
        // INHINT; ALMCADR = caller's return; fetch code
        agc.set("ALMCADR", agc.get("Q"));
        bortent(code);
    }

    /** ALARM2: same, but with return address already in A. */
    public void alarm2(int code) {
        agc.set("ALMCADR", agc.get("A"));
        bortent(code);
    }

    /** BORTENT: L = alarm code; falls into PRIOENT. */
    public void bortent(int code) {
        agc.set("L", code);
        prioent();
    }

    /** PRIOENT: store BBANK + SUPERBNK as second word of ALMCADR, then LARMENT. */
    public void prioent() {
        agc.set("ALMCADR+1", agc.get("BBANK") | agc.get("SUPERBNK"));
        larment();
    }

    /** LARMENT: save return, location and bank of alarm, then fall to CHKFAIL1. */
    public void larment() {
        agc.set("ITEMP1", agc.get("Q"));
        agc.set("LOCALARM", agc.get("LOC"));
        agc.set("BANKALRM", agc.get("BANKSET"));
        chkfail1();
    }

    /** CHKFAIL1: FAILREG holds the first alarm code; light the PROG alarm if it was empty. */
    public void chkfail1() {
        if (agc.get("FAILREG") != 0) {
            chkfail2();
            return;
        }
        agc.set("FAILREG", agc.get("L"));
        proglarm();
    }

    /** CHKFAIL2: second alarm register. */
    public void chkfail2() {
        if (agc.get("FAILREG+1") != 0) {
            fail3();
            return;
        }
        agc.set("FAILREG+1", agc.get("L"));
        multexit();
    }

    /** FAIL3: third alarm register; if all full, flag MULTFAIL. */
    public void fail3() {
        if ((agc.get("FAILREG+2") & POSMAX) != 0) {
            multfail();
            return;
        }
        agc.set("FAILREG+2", agc.get("L"));
        multexit();
    }

    /** PROGLARM: DSPTAB+11D gets PROG alarm bits (OCT40400) turned on. */
    public void proglarm() {
        int dsp = agc.get("DSPTAB+11D");
        agc.set("DSPTAB+11D", Agc.add(dsp, (~dsp) & OCT40400 & Agc.WORD_MASK) & Agc.WORD_MASK);
        multexit();
    }

    /** MULTEXIT: RELINT and return to the user (INDEX A; TC 1 -> skip the code word). */
    public void multexit() {
        agc.set("INHINT", 0); // RELINT
        // return to caller (control returns naturally in Java)
    }

    /** MULTFAIL: more than three alarms; mark FAILREG+2 with bit 15. */
    public void multfail() {
        agc.set("FAILREG+2", Agc.add(agc.get("L"), BIT15));
        multexit();
    }

    /** PRIOLARM (DISPLAYS bank): priority-display alarm for 2CADR user in BUF2. */
    public void priolarm(int code) {
        agc.set("INHINT", 1);
        agc.set("L", code);
        agc.set("ALMCADR", agc.get("BUF2"));
        agc.set("A", agc.get("BUF2+1"));
        prioent(); // TC PRIOENT +1 (L left alone)
    }

    /** BAILOUT: store erasables via VAC5STOR then enter the alarm with the code after the call. */
    public void bailout(int code) {
        agc.set("INHINT", 1);
        agc.set("ALMCADR", agc.get("Q"));
        vac5stor();
        bortent(code);
        // original then falls to WHIMPER via the restart/resume path
        whimper();
    }

    /** WHIMPER: set BRUPT to resume at ENEMA (goes to restart/clean-up). */
    public void whimper() {
        agc.set("INHINT", 1);
        agc.set("BRUPT", Agc.add(2, agc.get("Z")));
        // RESUME; TC POSTJUMP CADR ENEMA
        Fresh.enema(agc);
    }

    /** POODOO: abort with the alarm code that follows; clears average-G flags unless V37 flag set. */
    public void poodoo(int code) {
        agc.set("INHINT", 1);
        agc.set("ALMCADR", agc.get("Q"));
        vac5stor();
        abort2(code);
    }

    /** ABORT2. */
    public void abort2(int code) {
        bortent(code);
        // fall into OCT77770 handling: check V37FLBIT
        if ((agc.get("V37FLBIT") & agc.get("FLAGWRD7")) != 0) {
            whimper(); // average G is on: do BAILOUT-style, not POODOO
            return;
        }
        downflag("STATEFLG");
        downflag("REINTFLG");
        downflag("NODOFLAG");
        mrKlean();
        whimper();
    }

    /** CCSHOLE: a CCS fell into an impossible hole; abort with 1103. */
    public void ccshole() {
        agc.set("INHINT", 1);
        agc.set("ALMCADR", agc.get("Q"));
        vac5stor();
        abort2(OCT1103);
    }

    /** CURTAINS: alarm code 217 and return to user. */
    public void curtains() {
        agc.set("INHINT", 1);
        alarm2(OCT217);
    }

    /** VARALARM: alarm with a user-supplied code in A. */
    public void varalarm(int code) {
        agc.set("INHINT", 1);
        agc.set("L", code);
        agc.set("ALMCADR", agc.get("Q"));
        prioent();
    }

    /** DOALARM = ENDOFJOB. */
    public void doalarm() {
        executive.endofjob();
    }

    /** ABORT = BAILOUT (temporary until ABORT calls out). */
    public void abort(int code) {
        bailout(code);
    }

    private void vac5stor() {
        new Fresh(agc);
        // TC BANKCALL / CADR VAC5STOR
        Fresh.vac5stor(agc);
    }

    private void downflag(String flag) {
        // TC DOWNFLAG / ADRES flag
        agc.set(flag, 0);
    }

    private void mrKlean() {
        // TC BANKCALL / CADR MR.KLEAN
        Fresh.mrKlean(agc);
    }

    /** Stubs for routines owned by other source files. */
    private static final class Fresh {
        Fresh(Agc a) { }
        static void enema(Agc a) { /* FRESH_START_AND_RESTART.agc ENEMA */ }
        static void vac5stor(Agc a) { /* ERASABLE_ASSIGNMENTS VAC5STOR (in INTERPRETER area) */ }
        static void mrKlean(Agc a) { /* MR.KLEAN in PINBALL_GAME_BUTTONS_AND_LIGHTS */ }
    }
}
