package apollo.comanche055;

import apollo.agc.Agc;

/**
 * TVCRESTARTS.agc (Comanche055): TVC (thrust vector control) DAP restart package,
 * bank 16 (DAPROLL). Entered by the restart logic (..TCR.. in ..FINCOPY..) to bring the
 * TVC DAP and TVCEXECUTIVE back into a consistent state after a restart.
 */
public final class Tvcrestarts {
    private final Agc agc;

    public Tvcrestarts(Agc agc) {
        this.agc = agc;
    }

    /** RESUME / NOQRSM: return from the interrupt context (restore BANKRUPT/QRUPT and RELINT). */
    private void resume() {
    }

    private static final int BIT2 = 0b10;
    private static final int BIT8 = 0b10000000;
    private static final int BIT11 = 0b10000000000;
    private static final int PRIO6 = 0x0C00; // bits 11,12 (octal 06000)
    private static final int OCT37776 = 037776;
    private static final int NINE = 9;

    /** TVCCADR: table of CADRs (indices 0..3: ENABL2, PCOPY+1, CMDSOUT, YCOPY+1). */
    public static final String[] TVCCADR = {"ENABL2", "PCOPY+1", "CMDSOUT", "YCOPY+1"};
    /** TVCEXADR: OCT 37772 (60 ms in T5) then TVCEXECUTIVE restart points, ordered. */
    public static final int TVCEXADR_T5 = 037772;
    public static final String[] TVCEXADR = {
        "(60MS)", "EXECCOPY+1", "1SHOTCHK", "TEMPSET", "CORSETUP", "CORCOPY+1", "CNTRCOPY"
    };

    /** REDOTVC: TVC restart package. */
    public void redotvc() {
        // LXCH BANKRUPT; EXTEND; QXCH QRUPT  (interrupt context save)
        execphs();
    }

    /** EXECPHS: check TVCEXECUTIVE phase (CCS TVCEXPHS). */
    public void execphs() {
        int phs = Agc.toSigned(agc.get("TVCEXPHS"));
        if (phs > 0) {
            // must restart TVCEXECUTIVE: 9 cs delay forces EXRSTRT to occur before PITCHDAP, after CMDSOUT
            new Waitlist(agc).waitlist(NINE, this::exrstrt);   // 9 cs, 2CADR EXRSTRT
        }
        tvcdaphs();
    }

    /** TVCDAPHS: check bits 15 and 1 of TVCPHASE to see DAP restart location (-1,1,2,3). */
    public void tvcdaphs() {
        int tvcphase = agc.get("TVCPHASE");
        int masked = (~OCT37776 & Agc.WORD_MASK) & tvcphase;   // CS OCT37776; MASK TVCPHASE
        if (Agc.toSigned(masked) != 0) {
            fincopy();            // finish the copycycle first
            return;
        }
        // masked == 0: TVCPHASE is 0 -> just prepare the outcounters; else check for -2
        int notPhase = ~tvcphase & Agc.WORD_MASK;
        if ((notPhase & BIT2) == 0) {   // BZF TRIM/CMD (TVCPHASE = -1)
            trimCmd();
            return;
        }
        enabl1();
    }

    /** ENABL1: TVC enable followed by a 40 ms minimum wait; also optics-DAC-enable. */
    public void enabl1() {
        agc.set("CHAN12", agc.get("CHAN12") | BIT8 | BIT11);   // WOR CHAN12
        agc.set("T5LOC", 0 /* ENABL2 */);                      // T5 will call ENABL2
        agc.set("TIME5", TVCEXADR_T5);                         // 60 ms
        resume();
    }

    /** ENABL2: continue preparation of outcounters; optics error counter enable, 4 ms wait. */
    public void enabl2() {
        agc.set("CHAN12", agc.get("CHAN12") | BIT2);
        agc.set("T5LOC", 2 /* CMDSOUT */);
        agc.set("TIME5", OCT37776);   // 20 ms
        resume();
    }

    /** CMDSOUT: release the counters; send the most recent actuator commands. */
    public void cmdsout() {
        agc.set("TVCPITCH", Agc.fromSigned(Agc.toSigned(agc.get("PCMD"))));  // avoid +0
        agc.set("TVCYAW", Agc.fromSigned(Agc.toSigned(agc.get("YCMD"))));
        agc.set("CHAN14", agc.get("CHAN14") | PRIO6);    // release counters (bits 11,12)
        phschk2();
    }

    /** PHSCHK2: check TVCPHASE again. */
    public void phschk2() {
        int tvcphase = Agc.toSigned(agc.get("TVCPHASE"));
        if (tvcphase > 0) {           // CCS: positive -> CHKSTRK
            chkstrk();
            return;
        }
        if (tvcphase == 0) {
            chkstrk();
            return;
        }
        // negative: A holds diminished absolute value (-2 -> +1, -1 -> +0)
        int dim = Math.abs(tvcphase) - 1;
        if (dim > 0) {
            new TvcInitialize(agc).swichovr();   // IBNKCALL SWICHOVR +5: repeat CSM/LM V46 switch-over
        } else {
            new TvcInitialize(agc).mrclean();    // POSTJUMP MRCLEAN: repeat TVC initialization (no return)
        }
    }

    /** CHKSTRK: check for stroke test in progress. */
    public void chkstrk() {
        int stroker = Agc.toSigned(agc.get("STROKER"));
        if (stroker != 0 && stroker != -0) {
            tstinitj();
            return;
        }
        if (agc.get("STROKER") == Agc.WORD_MASK) {   // -0: prior V68 was done
            tstinitj();
            return;
        }
        chkstrk4();
    }

    private void chkstrk4() {
        // If positive or zero restart at TVCINIT4 (zeros TVCPHASE and so on)
        new TvcInitialize(agc).tvcinit4();   // POSTJUMP TVCINIT4
    }

    /** FINCOPY: re-enter the appropriate copycycle, then prepare the outcounters. */
    public void fincopy() {
        int idx = Agc.toSigned(agc.get("TVCPHASE"));
        iswcall(TVCCADR[idx]);     // TCR ISWCALL
        enabl1();
    }

    /** TRIM/CMD: TVCDAPON initialization not completed; set P,YCMD from PACTOFF,YACTOFF. */
    public void trimCmd() {
        agc.set("PCMD", agc.get("PACTOFF"));
        agc.set("PCMD+1", agc.get("PACTOFF+1"));
        enabl1();
    }

    /** TSTINITJ: disable stroke test (-0 shows prior V68; +0 means new V68 required). */
    public void tstinitj() {
        agc.set("STROKER", 0);
        chkstrk4();
    }

    /** EXRSTRT: TVCEXECUTIVE restarts; go to the appropriate restart point by TVCEXPHS. */
    public void exrstrt() {
        int idx = Agc.toSigned(agc.get("TVCEXPHS"));
        exec(TVCEXADR[idx]);
    }

    /** ISWCALL into the copycycle named by the CADR table entry (routines in the TVC DAP files). */
    private void iswcall(String cadr) {
        // PCOPY +1 / YCOPY +1 / ENABL2 / CMDSOUT
        switch (cadr) {
            case "ENABL2" -> enabl2();
            case "CMDSOUT" -> cmdsout();
            default -> { /* PCOPY+1, YCOPY+1: copycycle in TVC DAP (PITCHDAP / YAWDAP) */ }
        }
    }

    /** TCF 0 indexed by TVCEXADR: TVCEXECUTIVE restart points (EXECCOPY+1, 1SHOTCHK, TEMPSET, CORSETUP, CORCOPY+1, CNTRCOPY). */
    private void exec(String label) {
        // transfers to the named routine in TVC executive (TvcExecutive)
    }
}
