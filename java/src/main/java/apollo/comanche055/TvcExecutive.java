package apollo.comanche055;

import apollo.agc.Agc;

/**
 * TVCEXECUTIVE.agc (Comanche 055 / Colossus 2A, pages 945-950).
 *
 * TVCEXEC, NEEDLEUP, VARGAINS, 1SHOTCHK, REPCHEK, CG.CORR, COPYCYCLES.
 * A self-perpetuating Waitlist task at 1/2 second intervals which prepares the roll DAP
 * (OGANOW, AK, OGAERR), updates needles, mass and variable gains, and performs the
 * one-shot and repetitive CG (trim) corrections of the TMC loop.
 * TVCEXPHS tracks restart phase of the copy cycles.
 */
public class TvcExecutive {
    private final Agc agc;

    /** FCORFRAC: one-shot correction fraction (OCT 10000). */
    public static final int FCORFRAC = 010000;
    public static final int BIT13 = 010000;
    public static final int OCT60000 = 060000;
    public static final int HALF_SEC_CS = 50;

    public TvcExecutive(Agc agc) {
        this.agc = agc;
    }

    private int g(String s) { return agc.get(s); }
    private int gs(String s) { return Agc.toSigned(agc.get(s)); }
    private void s(String s, int v) { agc.set(s, v); }
    private void ss(String s, int v) { agc.set(s, Agc.fromSigned(v)); }

    private boolean engineOn() {
        return (g("DSALMOUT") & BIT13) != 0;      // channel 11 bit 13
    }

    /** CCS result: 0 = >0, 1 = +0, 2 = <0, 3 = -0 ; also returns the abs-1 via side value. */
    private static int ccsBranch(int word) {
        int v = word & Agc.WORD_MASK;
        if (v == 0) return 1;
        if (v == Agc.WORD_MASK) return 3;
        return (v & Agc.SIGN_BIT) != 0 ? 2 : 0;
    }

    private static int ccsDecrement(int word) {
        int v = Agc.toSigned(word);
        int m = Math.abs(v);
        return m == 0 ? 0 : m - 1;
    }

    /** TVCEXEC: waitlist task entry. */
    public void tvcexec() {
        // CS FLAGWRD6 / MASK OCT60000 / BZMF: terminate when bits 15,14 not "10"
        int masked = (~g("FLAGWRD6")) & OCT60000 & Agc.WORD_MASK;
        if (Agc.toSigned(masked) <= 0) {
            tvcexfin();
            return;
        }
        new Waitlist(agc).waitlist(HALF_SEC_CS, this::tvcexec);   // perpetuate TVCEXEC
        rollprep();
    }

    /** ROLLPREP: update roll ladders, FDAI needle (AK) and phase-plane error (OGAERR). */
    private void rollprep() {
        s("OGAPAST", g("OGANOW"));
        s("OGANOW", g("CDUX"));
        int err = Agc.toSigned(g("OGAD")) - Agc.toSigned(g("OGANOW"));
        ss("AK", err);                                       // fly-to OGA error, B-1 revs
        // MP -BIT14 : multiply by -1/2 (scale to B+0 revs), keep high order
        ss("OGAERR", (int) (((long) err * -0x4000L) >> 14));
        new Waitlist(agc).waitlist(3, () -> new TvcRollDap(agc).rolldap());
        needleup();
    }

    /** NEEDLEUP: needles update via IBNKCALL NEEDLER. */
    private void needleup() {
        new InterBankCommunication(agc).ibnkcall(0, () -> new RcsCsmDigitalAutopilot(agc).needler());
        vargains();
    }

    /** VARGAINS: variable gain and mass update, inhibited if engine off. */
    private void vargains() {
        if (!engineOn()) {
            vargainsPlus5();
            return;
        }
        int br = ccsBranch(g("VCNTR"));
        switch (br) {
            case 0:                       // not yet
                s("VCNTRTMP", ccsDecrement(g("VCNTR")));
                s("MASSTMP", g("CSMMASS"));  // protect VCNTR and CSMMASS during impulsive burn
                execcopy();
                break;
            case 1:                       // +0: now
                gainchng();
                break;
            case 2:                       // not used
                s("VCNTRTMP", ccsDecrement(g("VCNTR")));
                s("MASSTMP", g("CSMMASS"));
                execcopy();
                break;
            default:                      // -0: LOTHRUST (S40.8 R40) / V97 R40 ENGFAIL
                vargainsPlus5();
        }
    }

    /** VARGAINS +5: bypass mass/gain updates. */
    private void vargainsPlus5() {
        s("TVCEXPHS", 2);
        oneShotCheck();
    }

    /** GAINCHNG: update IXX, IAVG, IAVG/TLX (FIXCW) and 1/CONACC, VARK (S40.15). */
    private void gainchng() {
        new TvcMassprop(agc).fixcw();
        new TvcInitialize(agc).s4015();
        ss("MASSTMP", gs("CSMMASS") - gs("TENMDOT"));    // KG B+16, mass at next 10 s
        s("VCNTRTMP", 19);                               // reset counter
        execcopy();
    }

    /** EXECCOPY: restart-protected copy cycle of mass and VCNTR. */
    private void execcopy() {
        incrTvcexphs();                                  // (1)
        s("CSMMASS", g("MASSTMP"));
        s("VCNTR", g("VCNTRTMP"));
        s("V97VCNTR", g("VCNTRTMP"));                    // for ENGFAIL (R41) mass updates
        incrTvcexphs();                                  // (2)
        oneShotCheck();
    }

    private void incrTvcexphs() {
        s("TVCEXPHS", Agc.add(g("TVCEXPHS"), 1));
    }

    /** 1SHOTCHK: time for one-shot or repetitive correction? */
    private void oneShotCheck() {
        switch (ccsBranch(g("CNTR"))) {
            case 0:                                      // not yet: count down
                s("CNTRTMP", ccsDecrement(g("CNTR")));
                s("TVCEXPHS", 6);
                cntrcopy();
                break;
            case 1:                                      // now
                oneShotOk();
                break;
            case 2:                                      // one-shot over, on to REPCORR
                s("CNTRTMP", ccsDecrement(g("CNTR")));
                s("TVCEXPHS", 6);
                cntrcopy();
                break;
            default:                                     // -0: one-shot only
                oneShotOk();
        }
    }

    /** REPCHEK: repetitive updates after the one-shot correction. */
    private void repchek() {
        if (gs("REPFRAC") <= 0) {
            tvcexfin();                                  // over and out
            return;
        }
        s("TEMPDAP +1", g("REPFRAC"));
        s("TVCEXPHS", 4);
        corsetup();
    }

    /** 1SHOTOK: engine must be on to permit one-shot. */
    private void oneShotOk() {
        if (!engineOn()) {
            tvcexfin();
            return;
        }
        incrTvcexphs();                                  // (3)
        tempset();
    }

    /** TEMPSET: set up correction fraction. */
    private void tempset() {
        s("TEMPDAP +1", FCORFRAC);
        incrTvcexphs();                                  // (4)
        corsetup();
    }

    /** CORSETUP: double the fraction if LEM is off. */
    private void corsetup() {
        int frac = gs("TEMPDAP +1");
        int tempdap = ((g("DAPDATR1") & BIT13) == 0) ? frac + frac : frac;   // LEM off => 2x
        ss("TEMPDAP", tempdap);
        s("CNTRTMP", Agc.fromSigned(-1));                // CNTR = -1 (one-shot done)
        cgCorr();
    }

    /** CG.CORR: pitch and yaw TMC loop corrections (double precision). */
    private void cgCorr() {
        long tempdap = gs("TEMPDAP");
        correctAxis("PDELOFF", "DELPBAR", "PACTTMP", tempdap);
        correctAxis("YDELOFF", "DELYBAR", "YACTTMP", tempdap);
        corcopy();
    }

    private long getDp(String hi) {
        // double-precision pair hi, hi+1 (stored as consecutive symbols "<hi>" and "<hi> +1")
        return ((long) Agc.toSigned(g(hi)) << 14) + Agc.toSigned(g(hi + " +1"));
    }

    private void setDp(String hi, long v) {
        s(hi, Agc.fromSigned((int) (v >> 14)));
        s(hi + " +1", Agc.fromSigned((int) (v & 0x3FFF)));
    }

    private void correctAxis(String delOff, String delBar, String actTmp, long tempdap) {
        long delOffV = getDp(delOff);
        setDp(actTmp, delOffV);                          // ACTTMP = DELOFF (old)
        long t = -delOffV * 4;                           // DCS, DDOUBL x2
        t += getDp(delBar) * 4;                          // DCA DELBAR, DDOUBL x2, DAS
        long prod = (t >> 14) * tempdap >> 14;           // MP TEMPDAP on high word
        setDp(actTmp, getDp(actTmp) + prod);             // DAS ACTTMP
        s("TTMP1", Agc.fromSigned((int) (t >> 14)));
        s("TTMP1 +1", Agc.fromSigned((int) (t & 0x3FFF)));
    }

    /** CORCOPY: restart-protected copy of trim estimates and trims. */
    private void corcopy() {
        incrTvcexphs();                                  // (5)
        s("PACTOFF", g("PACTTMP"));
        s("PDELOFF", g("PACTTMP"));  s("PDELOFF +1", g("PACTTMP +1"));
        s("YACTOFF", g("YACTTMP"));
        s("YDELOFF", g("YACTTMP"));  s("YDELOFF +1", g("YACTTMP +1"));
        incrTvcexphs();                                  // (6)
        cntrcopy();
    }

    /** CNTRCOPY: update CNTR (restart-safe, follows copy cycle). */
    private void cntrcopy() {
        s("CNTR", g("CNTRTMP"));
        tvcexfin();
    }

    /** TVCEXFIN: reset TVCEXPHS and exit via TASKOVER. */
    private void tvcexfin() {
        s("TVCEXPHS", 0);
        new Waitlist(agc).taskover();
    }
}
