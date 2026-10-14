package apollo.luminary099;

import apollo.agc.Agc;

/**
 * ALARM_AND_ABORT.agc - Luminary 099 alarm, bailout and abort routines.
 *
 * Alarm codes are recorded in FAILREG, FAILREG+1, FAILREG+2. The first alarm lights the PROG
 * lamp (DSPTAB+11D). Subsequent alarms are stored in the following registers; a third
 * alarm sets bit 15 of FAILREG+2 (multiple failure indicator).
 */
public class AlarmAndAbort {
    public static final int OCT40400 = 040400;
    public static final int OCT77770 = 077770;
    public static final int OCT14 = 014;
    public static final int OCT217 = 0217;
    public static final int OCT1103 = 01103;
    public static final int OCT35 = 035;
    public static final int BIT15 = 040000;
    public static final int POSMAX = 037777;
    public static final int PROG_LAMP_BITS = 040400;

    private final Agc agc;

    public AlarmAndAbort(Agc agc) {
        this.agc = agc;
    }

    /** ALARM: record alarm code that follows the TC (code passed explicitly) and return. */
    public void alarm(int alarmCode, int callerCadr) {
        agc.set("ALMCADR", callerCadr);
        borTent(alarmCode);
    }

    /** ALARM2: same as ALARM but caller's Q already saved. */
    public void alarm2(int alarmCode, int callerCadr) {
        alarm(alarmCode, callerCadr);
    }

    /** BORTENT: common entry for BAILOUT / POODOO; alarm code held in L. */
    public void borTent(int alarmCodeL) {
        agc.set("L", alarmCodeL);
        prioEnt();
    }

    /** PRIOENT: combine BBANK with SUPERBNK and store as second word of ALMCADR. */
    public void prioEnt() {
        agc.set("ALMCADR+1", agc.get("BBANK") | agc.get("SUPERBNK"));
        larmEnt();
    }

    /** LARMENT: save return address in ITEMP1 then store alarm code in first free FAILREG. */
    public void larmEnt() {
        agc.set("ITEMP1", agc.get("Q"));
        chkFail1();
    }

    /** CHKFAIL1: FAILREG empty? then store code and turn on PROG alarm light. */
    public void chkFail1() {
        if (agc.get("FAILREG") != 0 && agc.get("FAILREG") != Agc.WORD_MASK) {
            chkFail2();
            return;
        }
        agc.set("FAILREG", agc.get("L"));
        progLarm();
    }

    /** CHKFAIL2: try FAILREG+1. */
    public void chkFail2() {
        if (agc.get("FAILREG+1") != 0 && agc.get("FAILREG+1") != Agc.WORD_MASK) {
            fail3();
            return;
        }
        agc.set("FAILREG+1", agc.get("L"));
        multExit();
    }

    /** FAIL3: third alarm; if FAILREG+2 already used, set the multiple-fail flag. */
    public void fail3() {
        if ((agc.get("FAILREG+2") & POSMAX) != 0) {
            multFail();
            return;
        }
        agc.set("FAILREG+2", agc.get("L"));
        multExit();
    }

    /** PROGLARM: turn on the PROG alarm light in DSPTAB+11D. */
    public void progLarm() {
        int dsp = agc.get("DSPTAB+11D");
        int cleared = (~PROG_LAMP_BITS) & Agc.WORD_MASK;
        // CS DSPTAB+11D ; MASK OCT40400 ; ADS DSPTAB+11D
        int add = (~dsp) & PROG_LAMP_BITS & Agc.WORD_MASK;
        agc.set("DSPTAB+11D", Agc.add(dsp, add));
        multExit();
    }

    /** MULTEXIT: re-enable interrupts and return to the saved address (INDEX A / TC 1). */
    public void multExit() {
        agc.set("RELINT", 1);
        // return to caller: address in ITEMP1 (+1 skips the alarm code word)
    }

    /** MULTFAIL: set bit 15 of FAILREG+2 to indicate multiple failures. */
    public void multFail() {
        agc.set("FAILREG+2", Agc.add(agc.get("L"), BIT15));
        multExit();
    }

    /** PRIOLARM: priority-display alarm; saves alarm code and BUF2 2CADR. */
    public void prioLarm(int alarmCode) {
        agc.set("L", alarmCode);
        agc.set("ALMCADR", agc.get("BUF2"));
        // CA BUF2 +1 ; TC PRIOENT +1 (leave L alone)
        agc.set("A", agc.get("BUF2+1"));
        agc.set("ALMCADR+1", agc.get("BUF2+1") | agc.get("SUPERBNK"));
        larmEnt();
    }

    /** -2SEC: constant -200 (centiseconds). Followed by V05N09 priority display. */
    public static final int MINUS_2SEC = -200;

    /** BAILOUT: alarm with abort to restart-able state (code follows TC). */
    public void bailout(int alarmCode) {
        agc.set("ALMCADR", agc.get("Q"));
        borTent(alarmCode);
        // TC BORTENT, then falls to WHIMPER via OCT40400 pseudo-instruction
        whimper();
    }

    /** WHIMPER / ABORT: RESUME into POSTJUMP -> ENEMA (restart the program). */
    public void whimper() {
        agc.set("BRUPT", Agc.add(2, agc.get("Z")));
        // RESUME; TC POSTJUMP; CADR ENEMA
        new Enema(agc).enema();
    }

    /** POODOO: abort with alarm code following TC. */
    public void poodoo(int alarmCode) {
        agc.set("ALMCADR", agc.get("Q"));
        abort2(alarmCode);
    }

    /** ABORT2: store return and enter BORTENT; then fall into the abort sequence. */
    public void abort2(int alarmCode) {
        agc.set("ALMCADR", agc.get("Q"));
        borTent(alarmCode);
        // pseudo-instruction OCT77770 then: CAF OCT35 ; TS L ; COM ; DXCH -PHASE4
        agc.set("L", OCT35);
        goPoodoo();
    }

    /** GOPOODOO: reset flags and shut down programs, then go to POOH or STRTIDLE. */
    public void goPoodoo() {
        // INHINT ; CADR FLAGS via BANKCALL
        flags();
        if ((agc.get("FLAGWRD7") & agc.get("V37FLBIT")) != 0) {
            strtIdle();
            return;
        }
        // Terminate groups 1,3,5,6 then 2,4,1,3,5,6
        // BANKCALL V37KLEAN
        new V37Klean(agc).v37Klean();
        // BANKCALL MR.KLEAN
        new MrKlean(agc).mrKlean();
        whimper();
    }

    /** STRTIDLE: put the servicer into its ground state (SERVIDLE) and proceed to GOTOPOOH. */
    public void strtIdle() {
        agc.set("SUPERSW", agc.get("BBSERVDL"));
        agc.set("SERVIDLE_CALLED", 1);
    }

    /** CCSHOLE: abort with code 01103 (CCS fell into a hole). */
    public void ccsHole() {
        agc.set("ALMCADR", agc.get("Q"));
        abort2(OCT1103);
    }

    /** CURTAINS: alarm 00217 (user program ran into the curtains), then return to user. */
    public void curtains() {
        agc.set("ALMCADR", agc.get("Q"));
        alarm2(OCT217, agc.get("Q"));
    }

    /** BAILOUT1: 2-CADR variant of BAILOUT. */
    public void bailout1() {
        dxchAlmCadr();
        agc.set("ITEMP1", ADR40400);
        bothAbrt();
    }

    /** POODOO1: 2-CADR variant of POODOO. */
    public void poodoo1() {
        dxchAlmCadr();
        agc.set("ITEMP1", ADR77770);
        bothAbrt();
    }

    /** BOTHABRT: store the abort-tail address, load alarm code into L, record. */
    public void bothAbrt() {
        agc.set("L", agc.get("Q"));
        chkFail1();
    }

    /** ALARM1: 2-CADR alarm. */
    public void alarm1() {
        dxchAlmCadr();
        almNCadr();
    }

    /** ALMNCADR: alarm with the code at Q. */
    public void almNCadr() {
        agc.set("L", agc.get("Q"));
        larmEnt();
    }

    private void dxchAlmCadr() {
        int a = agc.get("A");
        int l = agc.get("L");
        agc.set("A", agc.get("ALMCADR"));
        agc.set("L", agc.get("ALMCADR+1"));
        agc.set("ALMCADR", a);
        agc.set("ALMCADR+1", l);
    }

    // Symbolic addresses of "TCF OCT77770" / "TCF OCT40400"
    public static final int ADR77770 = OCT77770;
    public static final int ADR40400 = OCT40400;

    /** DOALARM EQUALS ENDOFJOB. */
    public void doAlarm() {
        new ExecutiveStub(agc).endOfJob();
    }

    /** VARALARM: alarm with variable code supplied in A. */
    public void varAlarm(int alarmCode) {
        agc.set("L", alarmCode);
        agc.set("ALMCADR", agc.get("Q"));
        prioEnt();
        // pseudo-instruction OCT14 ; TC ALMCADR (return to user)
    }

    /**
     * FLAGS: clear STATEFLG, REINTFLG and NODOFLAG
     * (FLAGWRD3 &= ~STATEBIT; FLGWRD10 &= ~REINTBIT; FLAGWRD2 &= ~NODOBIT).
     */
    public void flags() {
        agc.set("FLAGWRD3", agc.get("FLAGWRD3") & ~agc.get("STATEBIT"));
        agc.set("FLGWRD10", agc.get("FLGWRD10") & ~agc.get("REINTBIT"));
        agc.set("FLAGWRD2", agc.get("FLAGWRD2") & ~agc.get("NODOBIT"));
    }

    private static final class V37Klean { V37Klean(Agc a) {} void v37Klean() {} }
    private static final class MrKlean { MrKlean(Agc a) {} void mrKlean() {} }

    /** Placeholder neighbours so the file stands alone. */
    private static final class Enema {
        private final Agc agc;
        Enema(Agc agc) { this.agc = agc; }
        void enema() { /* see Fresh start / Restart */ }
    }

    private static final class ExecutiveStub {
        private final Agc agc;
        ExecutiveStub(Agc agc) { this.agc = agc; }
        void endOfJob() { /* EXECUTIVE.ENDOFJOB */ }
    }
}
