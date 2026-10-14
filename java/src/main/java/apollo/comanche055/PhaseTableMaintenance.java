package apollo.comanche055;

import apollo.agc.Agc;

/**
 * PHASE_TABLE_MAINTENANCE.agc: restart-protection phase table maintenance.
 * Contains NEWMODEX/NEWMODEA (update the major mode register MODREG and display it),
 * CHECKMM, SETUPDSP/DSPMMJOB, and PHASCHNG / 2PHSCHNG which record restart phase
 * information for six groups.
 *
 * PHASCHNG call: TC PHASCHNG followed by OCT word. Word formats:
 * <pre>
 *   TYPE A  TL0 00P PPP PPP GGG  fixed phase
 *   TYPE B  TL1 DAP PPP PPP GGG  fixed phase plus a job
 *   TYPE C  TL0 1AD XXX CJW GGG  variable restart information
 * </pre>
 * G = group (1-7 in the encoding; here groups index 1..6), P = phase, T = set TBASE,
 * L = set LONGBASE, D = priority/delta-time given, A = 2CADR given, C/J/W = longcall/job/waitlist.
 */
public final class PhaseTableMaintenance {
    private final Agc agc;

    public static final int OCT7 = 07, OCT17770 = 017770, OCT14000 = 014000, OCT60000 = 060000;
    public static final int BIT7 = 0100, BIT8 = 0200, BIT12 = 04000, BIT14 = 020000, BIT15 = 040000;
    public static final int PRIO30 = 030000;

    /** PHASE1.. and -PHASE1.. (complement) per group. */
    public final int[] phase = new int[7];
    public final int[] minusPhase = new int[7];
    /** TBASE1.. per group (negative time stamp). */
    public final int[] tbase = new int[7];
    /** PHSPRDT1.. priority or delta time per group. */
    public final int[] phsprdt = new int[7];
    /** PHSNAME1.. restart 2CADR (address, bbcon) per group. */
    public final int[][] phsname = new int[7][2];
    /** LONGBASE two words. */
    public long longbase;

    public PhaseTableMaintenance(Agc agc) {
        this.agc = agc;
    }

    // ------------------------------------------------------------------ mode display

    /** NEWMODEX: update MODREG with the mode word following the caller (mode in fixed). */
    public void newmodex(int modeWord) {
        newmodea(modeWord);
    }

    /** NEWMODEA: entry for mode in A. MODREG = A, then MMDSPLAY. */
    public void newmodea(int a) {
        agc.set("MODREG", a);
        mmdsplay();
    }

    /** MMDSPLAY: display the major mode (BANKJUMP to SETUPDSP). */
    public void mmdsplay() {
        setupdsp();
    }

    /**
     * CHECKMM: returns true if the mode in MODREG equals the mode at caller +1
     * (returns to caller +3 on match, caller +2 otherwise).
     */
    public boolean checkmm(int mode) {
        int diff = Agc.add(Agc.fromSigned(-Agc.toSigned(mode)), agc.get("MODREG"));
        return diff == 0 || diff == Agc.WORD_MASK;
    }

    /**
     * SETUPDSP (bank 14, 10/PHASE): INHINT; save caller's return 2CADR in RUPTREG1; start the
     * DSPMMJOB as a NOVAC job with priority 30 (either a task or a job can come here); RELINT.
     */
    public void setupdsp() {
        // CAF PRIO30; TC NOVAC; 2CADR DSPMMJOB
        dspmmjob();
    }

    /** DSPMMJOB (= DSPMMJB): displays MODREG in the program-number field. */
    public void dspmmjob() {
        // job body lives elsewhere (DSPMMJB)
    }

    // ------------------------------------------------------------------ phase changes

    /** TC PHASCHNG / OCT word: single phase change (TEMPSW2 = 1). */
    public void phaschng(int word) {
        change(word, 0, false, null);
    }

    /** TC 2PHSCHNG / OCT xxxxx (type A) / OCT yyyyy: double phase change. */
    public void phsChng2(int wordA, int wordY) {
        change(wordY, wordA, true, null);
    }

    /**
     * Combined PHASCHNG/2PHSCHNG/PHSCHNG2/ONEORTWO/CON1/CON2/BELOW1..4 logic.
     *
     * @param word   the (second, for 2PHSCHNG) octal phase word
     * @param word2  the first word (type A) for 2PHSCHNG
     * @param extra  optional additional operands: [priorityOrDelta, cadrAddr, cadrBbcon] depending on bits
     */
    private void change(int word, int word2, boolean two, int[] extra) {
        // PHASCHNG: INHINT
        int tempsw2 = two ? (word2 & BIT15) : 1;      // TEMPSW2: 1 = from PHASCHNG; else TBASE flag for group 2
        int tempg2 = 0, tempp2 = 0;
        if (two) {
            int tempP2 = word2;
            tempg2 = (tempP2 & OCT7) * 2;             // MASK OCT7; DOUBLE
            tempp2 = ((tempP2 & OCT17770) >> 3);      // MP BIT12 (shift right 3)
        }
        int tempsw = word;
        // PHSCHNG2 (switched bank)
        int tempg = (tempsw & OCT7) * 2;              // group *2
        int tempp = (tempsw & OCT17770) >> 3;         // phase (7 bits)
        tempsw &= OCT60000;                           // T and L bits only
        boolean typeAorBorC = (word & OCT14000) != 0;
        if (typeAorBorC) {
            // ONEORTWO: type B (bit 14 or 13 set) vs type C
            boolean typeB = (word & 0x2000) != 0 || (word & 0x1000) != 0;
            int tempprio;
            boolean usePrio = typeB ? (word & BIT12) != 0 : (tempp & BIT7) != 0;
            if (!usePrio) {
                tempprio = phsprdt[tempg / 2];       // OLDPRIO: use old priority (or delta T)
            } else {
                tempprio = extra != null && extra.length > 0 ? extra[0] : 0;   // GETPRIO
            }
            if ((tempp & BIT8) != 0 && extra != null && extra.length > 2) {
                // GETNEWNM: 2CADR given
                phsname[tempg / 2][0] = extra[1];
                phsname[tempg / 2][1] = extra[2];
            }
            // CON2: store phase, priority and name
            phase[tempg / 2] = tempp;
            phsprdt[tempg / 2] = tempprio;
        } else {
            // Type A: start storing the phase information
            phase[tempg / 2] = tempp;
        }
        int g = tempg / 2;
        // BELOW1: is it a PHASCHNG or 2PHSCHNG?
        if (tempsw2 != 1 && two) {
            // second group: store complemented phase (DXCH -PHASE1 ...) and set TBASE if flagged
            int g2 = tempg2 / 2;
            phase[g2] = tempp2;
            minusPhase[g2] = ~tempp2 & Agc.WORD_MASK;
            if (tempsw2 != 0) {
                tbase[g2] = Agc.fromSigned(-(agc.get("TIME1")));  // CS TIME1; TS TBASE
            }
        }
        // BELOW2: set TBASE or LONGBASE?
        if ((word & 0x2000) != 0 && (word & 0x4000) == 0) {
            // BELOW3: set LONGBASE only
            longbase = ((long) agc.get("TIME2") << 15) | agc.get("TIME1");
        } else if ((word & 0x4000) != 0) {
            tbase[g] = Agc.fromSigned(-agc.get("TIME1")) ;       // CS TIME1; TS TBASEn
            if ((word & 0x2000) != 0) {
                longbase = ((long) agc.get("TIME2") << 15) | agc.get("TIME1");
            }
        }
        // BELOW4: store the complemented phase
        minusPhase[g] = ~tempp & Agc.WORD_MASK;
        // RELINT; DTCB back to caller
    }

    /** CHECKB (type B test): bit 12 selects getting a new priority (GETPRIO) vs the old one. */
    public boolean checkB(int word) {
        return (word & BIT12) != 0;
    }

    /** ONEORTWO helper: classifies a word as type B (PRIO14 mask bits set) or A/C. */
    public static boolean isTypeB(int word) {
        return (word & OCT14000) != 0;
    }
}
