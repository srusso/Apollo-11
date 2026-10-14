package apollo.luminary099;

import apollo.agc.Agc;

/**
 * Translation of UPDATE_PROGRAM.agc (Luminary 1A build 099) - P27, the Update Program.
 *
 * Processes commands and data insertions requested by the ground via uplink, initiated by
 * extended verbs V70 (liftoff time increment), V71 (contiguous block update), V72
 * (scatter update) and V73 (octal clock increment). Accepted only in P00 (MODREG = +-0).
 *
 * Data is collected in UPBUFF (20D words), verified by the ground (V21N02 flashing),
 * then stored. All updates complement VERIFLAG (bit 3 of FLAGWRD7). Restart-protected
 * through restart group 6 (PHASCHNG codes 07026 before verification, 04026 after).
 *
 * Erasable debris: UPBUFF (20D), UPVERB (verb - 70), UPOLDMOD (interrupted major mode),
 * COMPNUMB (components to be sent), UPCOUNT (components received), UPTEMP (scratch /
 * component number to change during verify).
 */
public class UpdateProgram {
    private static final int UP70 = 0, UP71 = 1, UP72 = 2, UP73 = 3;
    private static final int UPLOADNV = 02101;        // VN 2101: V21 N01
    private static final int UPVRFYNV = 02102;        // VN 2102: V21 N02
    private static final int UP21 = 21;               // max components + 1 (MD1)
    private static final int UPDTPHAS = 5;
    private static final int BIT3 = 04;
    private static final int LOW8 = 0377;
    private static final int BIT9 = 0400;
    private static final int PHASE_PRE_VERIFY = 07026;
    private static final int PHASE_STORE = 04026;

    /** Results of a GOXDSPF request. */
    public enum Reply { TERMINATE_V34, PROCEED_V33, DATA }

    private final Agc agc;

    public UpdateProgram(Agc agc) {
        this.agc = agc;
    }

    private int g(String s) { return agc.get(s); }
    private void s(String sym, int v) { agc.set(sym, v); }
    private String ub(int i) { return "UPBUFF+" + i; }

    // ---- entry from the extended verbs -------------------------------------------------

    /** V70UPDAT: comes here on V70E. */
    public void v70updat() { enter(UP70); }

    /** V71UPDAT: comes here on V71E. */
    public void v71updat() { enter(UP71); }

    /** V72UPDAT: comes here on V72E. */
    public void v72updat() { enter(UP72); }

    /** V73UPDAT: comes here on V73E. */
    public void v73updat() { enter(UP73); }

    private void enter(int up) {
        s("UPVERBSV", up);                              // save UPVERB until OK to enter P27
        new PinballGameButtonsAndLights(agc).testxact(); // grab display, else operator error
        if (g("MODREG") != 0 && g("MODREG") != Agc.WORD_MASK) {   // update allowed in P00 / fresh start only
            uperror();
            return;
        }
        s("UPOLDMOD", g("MODREG"));                     // save current major mode
        s("UPVERB", g("UPVERBSV"));
        s("UPCOUNT", 1);
        uppart2();
    }

    /** UPERROR: turn on 'OPERATOR ERROR' light and go to the common exit (UPERROUT +2). */
    public void uperror() {
        uperrout2();
    }

    // ---- P27 proper ---------------------------------------------------------------------

    /** UPPART2: update program part 2. */
    public void uppart2() {
        new FreshStartAndRestart(agc).phaschng(PHASE_PRE_VERIFY, 030000, () -> upout1());   // restart: restore mode and downlist
        s("DNLSTCOD", 1);                               // coast + align downlist
        new PinballGameButtonsAndLights(agc).newmodex(27);   // major mode = 27
        switch (g("UPVERB")) {
            case UP70:
            case UP73:
                s("COMPNUMB", 2);                       // fixed: 2 components
                ohwell2();
                break;
            default:
                ohwell1();                              // V71/V72: ask for II
        }
    }

    /** OHWELL1: request the number of components (II) via flashing V21N01. */
    public void ohwell1() {
        s("MPAC+2", 0);                                 // ECADR of UPBUFF (ADUPBUFF)
        for (;;) {
            Reply r = display(UPLOADNV);
            if (r == Reply.TERMINATE_V34) { upout4(); return; }
            if (r == Reply.PROCEED_V33) continue;       // OHWELL1 +2
            if (ck4v32()) continue;                     // V32: get data again
            int ii = g("UPBUFF");
            if (ii - 2 <= 0) continue;                  // II must be >= 3
            if (UP21 - ii <= 0) continue;               // and <= 20D
            s("COMPNUMB", ii);
            s("UPCOUNT", g("UPCOUNT") + 1);             // INCR UPCOUNT
            ohwell2();
            return;
        }
    }

    /** OHWELL2: request each data component into UPBUFF, then verify. */
    public void ohwell2() {
        s("MPAC+2", 0);                                 // ECADR of UPBUFF-1 + UPCOUNT
        for (;;) {
            Reply r = display(UPLOADNV);
            if (r == Reply.TERMINATE_V34) { upout4(); return; }
            if (r == Reply.PROCEED_V33) continue;
            if (ck4v32()) continue;
            if (g("COMPNUMB") - g("UPCOUNT") <= 0) {
                upverify();                             // all data received
                return;
            }
            s("UPCOUNT", g("UPCOUNT") + 1);
        }
    }

    /** UPVERIFY: display V21N02 for correction (component number index) or verification. */
    public void upverify() {
        for (;;) {
            s("MPAC+2", 0);                             // ECADR of UPTEMP
            Reply r = display(UPVRFYNV);
            if (r == Reply.TERMINATE_V34) { upout4(); return; }
            if (r == Reply.PROCEED_V33) { upstore(); return; }
            if (ck4v32()) continue;
            int idx = Agc.toSigned(g("UPTEMP"));
            if (idx <= 0) continue;                     // not positive nonzero
            if (g("COMPNUMB") - idx + 1 <= 0) continue; // illegal component number
            s("UPCOUNT", idx);                          // user wants to change this buffer location
            ohwell2();
            return;
        }
    }

    private Reply display(int verbNoun) {
        return new PinballGameButtonsAndLights(agc).goxdspf(verbNoun);   // flashing display via BANKCALL
    }

    /** CK4V32: check for a Verb 32 on return from GOXDSPF; true if V32 (data not new). */
    public boolean ck4v32() {
        return (~g("MPAC") & 040) == 0;                 // CS MPAC; MASK BIT6; CCS A
    }

    /** UPOUT4 == UPOUT +1: V34 terminates the update. */
    public void upout4() { upout1(); }

    // ---- store ------------------------------------------------------------------------------

    /** UPSTORE: ground has verified the update; invert VERIFLAG and store the data. */
    public void upstore() {
        s("FLAGWRD7", g("FLAGWRD7") ^ g("VERIFBIT"));   // indicate V33 received (via downlink)
        new FreshStartAndRestart(agc).phaschng(PHASE_STORE);   // redo the store if a restart occurs
        if (g("UPVERB") - 2 <= 0) {
            upfndvac();                                 // V70 - V72 need orbit integration lock
        } else {
            upend73();
        }
    }

    /** UPEND73: V73 performs a DP octal AGC clock increment. */
    public void upend73() {
        copyDp("UPBUFF", "UPBUFF+8");
        if (!timedidl()) {
            new Alarm(agc).falton();                    // operator error
        }
        upout1();
    }

    private void copyDp(String from, String to) {
        s(to, g(from));
        s(to + "+1", g(from + "+1"));
    }

    /** UPFNDVAC: FINDVAC the UPJOB at extended-verb priority. */
    public void upfndvac() {
        new Executive(agc).findvac(g("CHRPRIO"), () -> upjob());
        new Executive(agc).endofjob();
    }

    /** UPJOB: INTSTALL (wait if orbital integration is in progress), then UPWAKE. */
    public void upjob() {
        new Interpreter(agc).intstall(() -> upwake());
    }

    /** UPWAKE: restart protect, set integration restart bit, branch on UPVERB. */
    public void upwake() {
        new FreshStartAndRestart(agc).phaschng(PHASE_STORE);
        new FlagwordAssignments(agc).upflag("REINTFLG");
        switch (g("UPVERB")) {
            case UP70: upend70(); break;
            case UP71: upend71(); break;
            default: upend72(); break;
        }
    }

    // ---- clock increment ------------------------------------------------------------------------

    /**
     * TIMEDIDL: increment the clock (TIME2,TIME1) with the DP word at UPBUFF+8.
     * @return false if the sum overflowed (old clock restored, operator error), true if OK
     */
    public boolean timedidl() {
        long old = dp("TIME2");
        s("TIME2", 0); s("TIME2+1", 0);
        s("UPBUFF+18", Agc.fromSigned((int) (old >> 14)));
        s("UPBUFF+19", Agc.fromSigned((int) (old & 037777)));
        new FreshStartAndRestart(agc).quickPhase6(UPDTPHAS);
        return timedidr();
    }

    /** TIMEDIDR: form the sum; if it overflows, restore the old clock value. */
    public boolean timedidr() {
        long inc = dp("UPBUFF+8");
        s("UPBUFF+8", 0); s("UPBUFF+9", 0);
        long sum = inc + dp("UPBUFF+18");
        if (sum >= (1L << 28) || sum <= -(1L << 28)) {  // overflow
            long oldClock = dp("UPBUFF+18");
            s("UPBUFF+18", 0); s("UPBUFF+19", 0);
            setDp("TIME2", oldClock);                   // restore old value of clock
            new FreshStartAndRestart(agc).phaschng(PHASE_STORE);
            return false;
        }
        new Interpreter(agc).tpagree();                 // force sign agreement
        setDp("TIME2", sum);                            // DAS TIME2
        new FreshStartAndRestart(agc).phaschng(PHASE_STORE);
        return true;
    }

    private long dp(String sym) {
        return ((long) Agc.toSigned(g(sym)) << 14) + Agc.toSigned(g(sym + "+1"));
    }

    private void setDp(String sym, long v) {
        s(sym, Agc.fromSigned((int) (v >> 14)));
        s(sym + "+1", Agc.fromSigned((int) (v & 037777)));
    }

    // ---- V71: contiguous block -------------------------------------------------------------------------

    /**
     * UPEND71: store COMPNUMB-2 words from UPBUFF+2 into erasable starting at the ECADR in
     * UPBUFF+1. Errors if the block would overflow into the next EBANK.
     */
    public void upend71() {
        int ecadr = g("UPBUFF+1");
        s("EBANK", ecadr);                              // set EBANK
        int rel = ecadr & LOW8;
        s("UPTEMP", rel);                               // S-register value of receiving area
        int n = rel - 3 + g("COMPNUMB");
        int remaining;
        if (n == 0) {
            remaining = n;                              // STORLP71 with zero
        } else if (((rel + g("COMPNUMB") - 3) & BIT9) != 0) {
            uperrout();                                 // would overflow into the next EBANK
            return;
        } else {
            remaining = g("COMPNUMB") - 3;
        }
        for (int i = remaining; i >= 0; i--) {          // STORLP71: words minus one down to 0
            int word = g(ub(2 + i));
            int addr = i + g("UPTEMP");
            agc.set("E" + Integer.toOctalString(g("EBANK")) + ":" + addr, word);   // LXCH 1400 (EBANK-relative store)
        }
        upout();
    }

    // ---- V72: scatter update ------------------------------------------------------------------------------

    /** UPEND72: (COMPNUMB-1)/2 data/ECADR pairs; COMPNUMB must be odd. */
    public void upend72() {
        if ((g("COMPNUMB") & 1) == 0) {
            uperrout();                                 // should be odd
            return;
        }
        for (int i = g("COMPNUMB") - 2; i > 0; i -= 2) {   // LDLOOP72
            int word = g(ub(1 + i));                    // next update word
            int ecadr = g(ub(i));                       // ECADR of register to be updated
            s("EBANK", ecadr);
            agc.set("E" + Integer.toOctalString(ecadr >> 8) + ":" + (ecadr & LOW8), word);
        }
        upout();
    }

    // ---- V70: liftoff time increment -----------------------------------------------------------------------------

    /**
     * UPEND70: decrement the AGC clock by the DP delta time in UPBUFF, decrement TETCSM and
     * TETLEM and increment TEPHEM by it (restart-protected copies in UPBUFF+10, +12).
     */
    public void upend70() {
        long delta = dp("UPBUFF");
        setDp("UPBUFF+8", -delta);                      // DCS UPBUFF
        if (!timedidl()) {
            uperrout();
            return;
        }
        setDp("UPBUFF+10", -delta);                     // copy decrementers for restart protection
        setDp("UPBUFF+12", -delta);
        new FreshStartAndRestart(agc).phaschng(PHASE_STORE);
        setDp("TETCSM", dp("TETCSM") + dp("UPBUFF+10")); setDp("UPBUFF+10", 0);   // decrement CSM state vector time
        setDp("TETLEM", dp("TETLEM") + dp("UPBUFF+12")); setDp("UPBUFF+12", 0);   // decrement LEM state vector time
        setDp("TEPHEM+1", dp("TEPHEM+1") + delta);      // increment TP TEPHEM
        new FreshStartAndRestart(agc).phaschng(PHASE_STORE);
        upout();
    }

    // ---- exits ------------------------------------------------------------------------------------------------------------

    /** UPOUT: normal finish of P27; release orbital integration and restore the prior mode. */
    public void upout() {
        new Interpreter(agc).intwakeu();
        upout1();
    }

    /** UPOUT +1: restore prior mode, clear downlist, turn off activity light, kill group 6. */
    public void upout1() {
        new PinballGameButtonsAndLights(agc).newmodex3(g("UPOLDMOD"));
        s("DNLSTCOD", 0);
        upactoff();
        s("-PHASE6", Agc.WORD_MASK);                    // kill group 6 (NEG0)
        s("-PHASE6+1", Agc.WORD_MASK);
        new ServiceRoutines(agc).endext();              // extended verb exit
    }

    /** UPERROUT: turn on 'OPERATOR ERROR' light and go to the common exit. */
    public void uperrout() {
        new Alarm(agc).falton();
        upout();
    }

    /** UPERROUT +2: error exit that does not reset any restart groups. */
    public void uperrout2() {
        new Alarm(agc).falton();
        upactoff();
        new ServiceRoutines(agc).endext();
    }

    /** UPACTOFF: turn off the uplink activity light (bit 3 of channel 11). */
    public void upactoff() {
        s("DSALMOUT", g("DSALMOUT") & ~BIT3);
    }
}
