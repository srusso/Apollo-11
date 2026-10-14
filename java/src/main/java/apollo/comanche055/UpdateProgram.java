package apollo.comanche055;

import apollo.agc.Agc;

/**
 * UPDATE_PROGRAM: P27, the uplink update program. Handles extended verbs V70 (LGC liftoff time /
 * clock + state vector time update), V71 (contiguous block update), V72 (scattered update)
 * and V73 (AGC clock increment). Data arrives through UPBUFF.
 */
public class UpdateProgram {
    public static final int UP70 = 0;
    public static final int UP71 = 1;
    public static final int UP72 = 2;
    public static final int UP73 = 3;
    /** V21N01: request data. */
    public static final int UPLOADNV = 2101;
    /** V21N02: request correction / verification. */
    public static final int UPVRFYNV = 2102;
    /** Maximum number of components + 1 (UP21 = MD1, DEC 21). */
    public static final int UP21 = 21;
    /** UPDTPHAS = FIVE (restart phase code). */
    public static final int UPDTPHAS = 5;

    private static final int BIT1 = 0x0001;
    private static final int BIT2 = 0x0002;
    private static final int BIT3 = 0x0004;
    private static final int BIT6 = 0x0020;
    private static final int BIT8 = 0x0080;
    private static final int BIT9 = 0x0100;
    private static final int LOW8 = 0x00FF;

    private final Agc agc;

    /** Result of a data/verb display response (GOXDSPF returns). */
    public enum Reply { TERMINATE, PROCEED, DATA }

    public UpdateProgram(Agc agc) {
        this.agc = agc;
    }

    // ---- Extended verb bank part (EXTVERBS) ----

    /** V70UPDAT: V70E entry. */
    public void v70updat() {
        v73updatCommon(UP70);
    }

    /** V71UPDAT: V71E entry. */
    public void v71updat() {
        v73updatCommon(UP71);
    }

    /** V72UPDAT: V72E entry. */
    public void v72updat() {
        v73updatCommon(UP72);
    }

    /** V73UPDAT: V73E entry. */
    public void v73updat() {
        v73updatCommon(UP73);
    }

    /** V73UPDAT+1: save UPVERB until it is OK to enter P27, then verify update is allowed. */
    private void v73updatCommon(int upverb) {
        agc.set("UPVERBSV", upverb);
        // TC TESTXACT: grab the display if available, otherwise "BUSY" return
        int modreg = agc.get("MODREG");
        if (modreg != 0 && modreg != Agc.WORD_MASK) { // not +0 (P00) and not -0 (fresh start)
            if (!ckmdmore()) {
                upperror();
                return;
            }
        }
        agc.set("UPOLDMOD", agc.get("MODREG"));
        agc.set("UPVERB", agc.get("UPVERBSV"));
        agc.set("UPCOUNT", 1);
        uppart2();
    }

    /** CKMDMORE: check whether the update is permitted in the current major mode. Returns true if OK. */
    private boolean ckmdmore() {
        boolean lgc = (agc.get("FLAGWRD5") & BIT8) == 0; // CS FLAGWRD5, MASK BIT8
        if (!lgc) {
            // UPERLEM: error - it's the LEM and mode isn't P00
            return false;
        }
        // CS TWO ; MASK MODREG : allow only modes 0,1,2 (i.e. P00, P01, P02)
        if ((agc.get("MODREG") & ~2 & Agc.WORD_MASK) != 0) {
            return false; // UPERCMC
        }
        return true;
    }

    /** UPERROR: operator error exit (UPERROUT+2). */
    private void upperror() {
        uperroutPlus2();
    }

    // ---- Update program part 2 (bank UPDATE2) ----

    /** UPPART2: P27 proper. */
    public void uppart2() {
        // PHASCHNG 07026 : restart group 6 restores old mode & downlist; UPOUT+1 on restart
        agc.set("DNLSTCOD", 1);
        agc.set("MODREG", 27);         // NEWMODEX: major mode 27
        switch (agc.get("UPVERB")) {
            case UP70:
            case UP73:
                agc.set("COMPNUMB", 2);
                ohwell2();
                break;
            default:
                ohwell1();
        }
    }

    /** OHWELL1: ask the user for the number of components parameter II (3..20). */
    private void ohwell1() {
        agc.set("MPAC+2", addressOf("UPBUFF"));
        while (true) {
            Reply r = goxdspf(UPLOADNV);
            if (r == Reply.TERMINATE) {
                upout4();
                return;
            }
            if (r == Reply.PROCEED) {
                continue; // OHWELL1+2
            }
            if (ck4v32()) {
                continue;
            }
            int ii = Agc.toSigned(agc.get("UPBUFF"));
            if (ii - 2 <= 0 || ii > UP21 - 1) {
                continue; // II must be >= 3 and <= 20
            }
            agc.set("COMPNUMB", ii);
            agc.set("UPCOUNT", Agc.add(agc.get("UPCOUNT"), 1));
            ohwell2();
            return;
        }
    }

    /** OHWELL2: receive each of the update components into UPBUFF. */
    private void ohwell2() {
        agc.set("MPAC+2", Agc.add(addressOf("UPBUFF") - 1, agc.get("UPCOUNT")));
        while (true) {
            Reply r = goxdspf(UPLOADNV);
            if (r == Reply.TERMINATE) {
                upout4();
                return;
            }
            if (r == Reply.PROCEED || ck4v32()) {
                continue;
            }
            // Data received: finished when COMPNUMB - UPCOUNT <= 0
            if (Agc.toSigned(agc.get("COMPNUMB")) - Agc.toSigned(agc.get("UPCOUNT")) <= 0) {
                upverify();
                return;
            }
            agc.set("UPCOUNT", Agc.add(agc.get("UPCOUNT"), 1)); // (INCR in display routine)
            agc.set("MPAC+2", Agc.add(addressOf("UPBUFF") - 1, agc.get("UPCOUNT")));
        }
    }

    /** UPVERIFY: flash V21N02 to allow correction or verification of the data. */
    private void upverify() {
        while (true) {
            agc.set("MPAC+2", addressOf("UPTEMP"));
            Reply r = goxdspf(UPVRFYNV);
            if (r == Reply.TERMINATE) {
                upout4();
                return;
            }
            if (r == Reply.PROCEED) {
                upstore();
                return;
            }
            if (ck4v32()) {
                continue;
            }
            int idx = Agc.toSigned(agc.get("UPTEMP"));
            if (idx <= 0 || idx > Agc.toSigned(agc.get("COMPNUMB"))) {
                continue; // illegal component number index
            }
            // Re-enter a corrected component at UPBUFF-1+index
            agc.set("MPAC+2", addressOf("UPBUFF") - 1 + idx);
            ohwell2At();
            return;
        }
    }

    /** OHWELL2+2: request data for the location in MPAC+2 (no recalculation). */
    private void ohwell2At() {
        Reply r = goxdspf(UPLOADNV);
        if (r == Reply.TERMINATE) {
            upout4();
            return;
        }
        if (r == Reply.PROCEED || ck4v32()) {
            ohwell2At();
            return;
        }
        if (Agc.toSigned(agc.get("COMPNUMB")) - Agc.toSigned(agc.get("UPCOUNT")) <= 0) {
            upverify();
        } else {
            ohwell2();
        }
    }

    /** UPOUT4 = UPOUT+1: V34 terminate. */
    private void upout4() {
        upoutPlus1();
    }

    /** CK4V32: returns true if the verb in MPAC is V32 (re-request data); false if it is data. */
    private boolean ck4v32() {
        return isV32(agc.get("MPAC")); // MPAC holds the verb on return
    }

    private boolean isV32(int verb) {
        return verb == 32;
    }

    /** GOXDSPF: external display call (flashing noun/verb, wait for crew/ground response). */
    protected Reply goxdspf(int verbNoun) {
        return Reply.DATA;
    }

    /** Address of an erasable symbol; only used to build ECADR-like values for the display routine. */
    private int addressOf(String symbol) {
        return symbol.hashCode() & LOW8;
    }

    // ---- Store data ----

    /** UPSTORE: ground has verified the update; invert VERIFLAG and perform the update. */
    public void upstore() {
        // INHINT: invert VERIFLAG (bit 3 of FLAGWRD7) to tell the ground via downlink that V33 was received
        agc.set("FLAGWRD7", agc.get("FLAGWRD7") ^ BIT3);
        // PHASCHNG 04026 : redo the data store if a restart occurs
        if (Agc.toSigned(agc.get("UPVERB")) - 2 <= 0) {
            upfndvac();
        } else {
            upend73();
        }
    }

    /** UPEND73: V73 - double precision octal AGC clock increment. */
    public void upend73() {
        // DCA UPBUFF ; DXCH UPBUFF+8 : move the increment where TIMEDIDR will pick it up
        agc.set("UPBUFF+8", agc.get("UPBUFF"));
        agc.set("UPBUFF+9", agc.get("UPBUFF+1"));
        if (!timedidl()) {
            faltonThenUpout();
            return;
        }
        upoutPlus1();
    }

    private void faltonThenUpout() {
        // TC FALTON: turn on OPERATOR ERROR light, then UPOUT+1
        upoutPlus1();
    }

    /** UPFNDVAC: schedule UPJOB (V70-V72 may update a state vector so integration must be locked). */
    public void upfndvac() {
        // CAF CHRPRIO ; TC FINDVAC ; 2CADR UPJOB ; TC ENDOFJOB
        upjob();
    }

    /** UPJOB: wait (INTSTALL) for orbital integration, then do the update. */
    public void upjob() {
        // CALL INTSTALL (job sleeps while Orbital Integration is in progress)
        upwake();
    }

    /** UPWAKE: set REINTFLG (integration restart), then branch on UPVERB. */
    public void upwake() {
        // PHASCHNG 04026 ; UPFLAG REINTFLG
        agc.set("REINTFLG", 1);
        // INHINT
        uppart3();
    }

    /** UPPART3: dispatch to the right update routine. */
    public void uppart3() {
        switch (agc.get("UPVERB")) {
            case UP70:
                upend70();
                break;
            case UP71:
                upend71();
                break;
            case UP72:
                upend72();
                break;
            default:
                upend73();
        }
    }

    // ---- Clock update ----

    /**
     * TIMEDIDL/TIMEDIDR: add the DP increment in UPBUFF+8 to TIME2/TIME1. TIME2,TIME1 are zeroed and
     * saved in UPBUFF+18 so that an overflow can be backed out. Returns false on overflow (error).
     */
    public boolean timedidl() {
        long time = dpGet("TIME2");
        dpSet("UPBUFF+18", time);       // store in case of overflow
        dpSet("TIME2", 0);
        // phase-change record (UPDTPHAS) for restart protection
        return timedidr(time);
    }

    private boolean timedidr(long oldTime) {
        long increment = dpGet("UPBUFF+8");
        dpSet("UPBUFF+8", 0);           // zero it in case of restarts
        long sum = increment + oldTime;
        long max = (1L << 28) - 1;
        if (sum > max || sum < -max) {
            // overflow: restore old clock value and report error
            dpSet("TIME2", oldTime);
            dpSet("UPBUFF+18", 0);
            return false;
        }
        dpSet("TIME2", sum);
        return true;
    }

    private long dpGet(String name) {
        int hi = Agc.toSigned(agc.get(name));
        int lo = Agc.toSigned(agc.get(name + "+1"));
        return ((long) hi << 14) + lo;
    }

    private void dpSet(String name, long v) {
        long mag = Math.abs(v);
        int sign = v < 0 ? -1 : 1;
        agc.set(name, Agc.fromSigned(sign * (int) (mag >> 14)));
        agc.set(name + "+1", Agc.fromSigned(sign * (int) (mag & 0x3FFF)));
    }

    /** UPEND70: V70 - decrement the AGC clock and state vector times, increment TEPHEM. */
    public void upend70() {
        long delta = dpGet("UPBUFF");
        dpSet("UPBUFF+8", -delta);
        if (!timedidl()) {
            uperrout();
            return;
        }
        dpSet("UPBUFF+10", -delta);     // copies for restart protection
        dpSet("UPBUFF+12", -delta);
        dpSet("TETCSM", dpGet("TETCSM") + dpGet("UPBUFF+10"));   // decrement CSM state vector time
        dpSet("TETLEM", dpGet("TETLEM") + dpGet("UPBUFF+12"));   // decrement LEM state vector time
        dpSet("TEPHEM", dpGet("TEPHEM") + delta);                // increment TP TEPHEM
        upout();
    }

    /** UPEND71: V71 - contiguous block store of COMPNUMB-2 words starting at ECADR UPBUFF+1. */
    public void upend71() {
        int ecadr = agc.get("UPBUFF+1");
        agc.set("EBANK", ecadr);
        int start = ecadr & LOW8;
        agc.set("UPTEMP", start);
        int remaining = Agc.toSigned(agc.get("COMPNUMB")) - 3;     // words to store minus one
        if (remaining != 0) {
            // would the block overflow into the next erasable bank?
            int check = start - 3 + Agc.toSigned(agc.get("COMPNUMB"));
            if ((check & BIT9) != 0) {
                uperrout();
                return;
            }
        }
        for (int i = remaining; i >= 0; i--) {
            int word = agc.get("UPBUFF+" + (2 + i));
            agc.set("ERASABLE@" + (agc.get("EBANK") >> 8) + ":" + (start + i), word);
        }
        upout();
    }

    /** UPEND72: V72 - scattered update; pairs of (data word, ECADR) following the first word. */
    public void upend72() {
        int n = Agc.toSigned(agc.get("COMPNUMB"));
        if ((n & BIT1) == 0) {
            uperrout(); // an odd number of components is required
            return;
        }
        int idx = n - 2;
        while (true) {
            int data = agc.get("UPBUFF+" + (idx + 1));
            idx--;
            int ecadr = agc.get("UPBUFF+" + (idx + 1));
            agc.set("EBANK", ecadr);
            agc.set("ERASABLE@" + (ecadr >> 8) + ":" + (ecadr & LOW8), data);
            if (idx <= 0) {
                break;
            }
            idx--;
        }
        upout();
    }

    // ---- Exits ----

    /** UPOUT: release orbital integration (INTWAKEU) and fall into UPOUT+1. */
    public void upout() {
        // TC INTWAKEU
        upoutPlus1();
    }

    /** UPOUT+1: restore prior mode, kill group 6, turn off UPLINK ACTIVITY light, exit extended verb. */
    public void upoutPlus1() {
        agc.set("MODREG", agc.get("UPOLDMOD"));    // NEWMODEX+3
        agc.set("DNLSTCOD", 0);
        upactoff();
        // kill group 6: -PHASE6 = NEG0 ; TC ENDEXT
    }

    /** UPERROUT: operator error light, then common exit. */
    public void uperrout() {
        // TC FALTON
        upout();
    }

    /** UPERROUT+2: operator error light, UPLINK ACTIVITY light off, ENDEXT. */
    public void uperroutPlus2() {
        // TC FALTON
        upactoff();
        // TC ENDEXT
    }

    /** UPACTOFF: clear bit 3 of channel 11 (DSALMOUT). */
    public void upactoff() {
        agc.set("DSALMOUT", agc.get("DSALMOUT") & ~BIT3 & Agc.WORD_MASK);
    }
}
