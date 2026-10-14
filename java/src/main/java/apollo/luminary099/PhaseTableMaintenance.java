package apollo.luminary099;

import apollo.agc.Agc;

/**
 * PHASE_TABLE_MAINTENANCE.agc - restart phase table maintenance (PHASCHNG / 2PHSCHNG) and the
 * major-mode setting routines (NEWMODEX / NEWMODEA / CHECKMM).
 *
 * The restart tables are PHASE1..PHASE6 (and complemented -PHASE1..-PHASE6), TBASE1..6,
 * PHSPRDT1..6 (priorities) and PHSNAME1..6 (2CADR names), each indexed by group number.
 * Group g (1..6) uses index 2*(g-1) in the original (TEMPG = 2*(group)).
 */
public class PhaseTableMaintenance {
    public static final int OCT7 = 07;
    public static final int OCT17770 = 017770;
    public static final int OCT60000 = 060000;
    public static final int OCT14000 = 014000;  // = PRIO14
    public static final int BIT7 = 0100;
    public static final int BIT8 = 0200;
    public static final int BIT12 = 04000;
    public static final int BIT14 = 020000;
    public static final int BIT15 = 040000;
    public static final int BIT14COM = 017777;
    public static final int PRIO30 = 030;

    private final Agc agc;

    public PhaseTableMaintenance(Agc agc) {
        this.agc = agc;
    }

    // ------------------------------------------------------------------
    // Major mode
    // ------------------------------------------------------------------

    /** NEWMODEX: set MODREG to the value in the word following the call (caller passes it). */
    public void newModeX(int mode) {
        newModeA(mode);
    }

    /** NEWMODEA: MODREG = A, then MMDSPLAY. */
    public void newModeA(int mode) {
        agc.set("MODREG", mode);
        mmDisplay();
    }

    /** MMDSPLAY: switch to the display bank to run SETUPDSP (shows the major mode on DSKY). */
    public void mmDisplay() {
        setUpDsp();
    }

    /** CHECKMM: returns true if MODREG equals the supplied major mode (skips on equal). */
    public boolean checkMm(int mode) {
        return Agc.toSigned(agc.get("MODREG")) - mode == 0;
    }

    /** SETUPDSP: schedule job DSPMMJOB at priority 30 (NOVAC) to display the major mode. */
    public void setUpDsp() {
        agc.set("NOVAC_PRIO", PRIO30);
        agc.set("NOVAC_JOB", 1);    // 2CADR DSPMMJOB (= DSPMMJB)
    }

    // ------------------------------------------------------------------
    // Phase change
    // ------------------------------------------------------------------

    /**
     * 2PHSCHNG: change two phase groups at once.
     *
     * @param w1 first word following the call: group number bits (0-7, doubled) in low three
     *           bits, phase in bits 4-14 (MASK OCT17770 / MP BIT12), bit15 flag selects the
     *           second-group time base
     * @param w2 second word: the same format as the PHASCHNG parameter
     */
    public void twoPhsChng(int w1, int w2) {
        agc.set("TEMPP2", w1);
        agc.set("TEMPG2", (w1 & OCT7) << 1);
        int tempp2 = ((w1 & OCT17770) * BIT12) >> 14;
        agc.set("TEMPP2", tempp2);
        agc.set("TEMPSW2", w1 & BIT15);
        agc.set("TEMPSW", w2);
        phasJump();
    }

    /** PHASCHNG: single phase change; parameter is the word following the TC. */
    public void phasChng(int param) {
        phsChngA(param);
    }

    /** PHSCHNGA: entry with TEMPSW = A, TEMPSW2 = 1 (only one group). */
    public void phsChngA(int param) {
        agc.set("TEMPSW", param);
        agc.set("TEMPSW2", 1);
        phasJump();
    }

    /** PHASJUMP: DTCB to PHSCHNG2 (in the PHASETAB bank). */
    public void phasJump() {
        phsChng2();
    }

    /**
     * ONEORTWO: handles the optional priority / name (2CADR) words that follow a PHASCHNG when
     * bits 11-12 of the parameter select a restart-protected job.
     */
    public void oneOrTwo() {
        int a = agc.get("TEMPSW") & OCT14000;
        if (a != 0) {
            checkB();
            return;
        }
        if ((agc.get("TEMPP") & BIT7) != 0) {
            getPrio();
        } else {
            oldPrio();
        }
    }

    /** OLDPRIO: TEMPPR = old priority from PHSPRDT1-2 indexed by group. */
    public void oldPrio() {
        agc.set("TEMPPR", agc.get("PHSPRDT" + (agc.get("TEMPG") / 2)));
        con1();
    }

    /** CON1: if bit 8 of TEMPP set, fetch new job name; else keep none and record BB. */
    public void con1() {
        if ((agc.get("TEMPP") & BIT8) != 0) {
            getNewNm();
            return;
        }
        agc.set("TEMPNM", agc.get("Q"));
        agc.set("TEMPBB", agc.get("BBANK") | agc.get("SUPERBNK"));
        toCon2();
    }

    /** TOCON2: DTCB to CON2 to store priority, name and phase. */
    public void toCon2() {
        con2();
    }

    /** GETPRIO: take new priority from the word after the call, then back into CON1. */
    public void getPrio() {
        agc.set("TEMPPR", agc.get("Q_NEXT_WORD"));
        agc.set("Q", agc.get("Q") + 1);
        con1();
    }

    /** GETNEWNM: take 2CADR name (two words) from after the call. */
    public void getNewNm() {
        agc.set("TEMPNM", agc.get("Q_NEXT_WORD"));
        agc.set("TEMPNM+1", agc.get("Q_NEXT_WORD+1"));
        agc.set("Q", agc.get("Q") + 2);
        toCon2();
    }

    /** CHECKB: bit 12 of TEMPSW selects priority fetch (GETPRIO) or old priority (OLDPRIO). */
    public void checkB() {
        if ((agc.get("TEMPSW") & BIT12) != 0) {
            getPrio();
        } else {
            oldPrio();
        }
    }

    /**
     * PHSCHNG2: the main phase-table update.
     * Sets TEMPG = 2*group (low 3 bits of TEMPSW), TEMPP = phase (bits 4-14), writes PHASEn / -PHASEn
     * with the time-base handling (TBASEn) of BELOW1..BELOW4.
     */
    public void phsChng2() {
        int tempSw = agc.get("TEMPSW");
        int tempG = (tempSw & OCT7) << 1;
        int tempP = ((tempSw & OCT17770) * BIT12) >> 14;
        agc.set("TEMPG", tempG);
        agc.set("TEMPP", tempP);
        int rest = tempSw & OCT60000;
        agc.set("TEMPSW", rest);
        if ((rest & OCT14000) != 0) {
            oneOrTwo();
            return;
        }
        agc.set("PHASE" + (tempG / 2), tempP);          // TS PHASE1 -2 (indexed)
        below1();
    }

    /** BELOW1: second-group (TEMPSW2) handling: PHASE/-PHASE pair and TBASE reset. */
    public void below1() {
        int sw2 = agc.get("TEMPSW2");
        if (Agc.toSigned(sw2) > 0) {
            below2();                                    // CCS > 0
            return;
        }
        if (Agc.toSigned(sw2) == 0) {
            // +0 : fall into the second group update
            int tempP2 = agc.get("TEMPP2");
            int g2 = agc.get("TEMPG2");
            agc.set("-PHASE" + (g2 / 2), Agc.fromSigned(-Agc.toSigned(tempP2)));
            agc.set("PHASE" + (g2 / 2), tempP2);
            agc.set("TBASE" + (g2 / 2), Agc.fromSigned(-Agc.toSigned(agc.get("TIME1"))));
        }
        below2();
    }

    /** BELOW2: if TEMPSW indicates a time-base reset, set TBASE of group; else BELOW3/BELOW4. */
    public void below2() {
        int sw = Agc.toSigned(agc.get("TEMPSW"));
        if (sw > 0) {
            below3();
            return;
        }
        if (sw == 0) {
            below4();
            return;
        }
        int g = agc.get("TEMPG");
        agc.set("TBASE" + (g / 2), Agc.fromSigned(-Agc.toSigned(agc.get("TIME1"))));
        below4();
    }

    /** BELOW3: store TIME2 (double precision) in LONGBASE. */
    public void below3() {
        agc.set("LONGBASE", agc.get("TIME2"));
        agc.set("LONGBASE+1", agc.get("TIME2+1"));
        below4();
    }

    /** BELOW4: write the complemented phase (-PHASEn = ~TEMPP) and return to caller, re-enabling interrupts. */
    public void below4() {
        int g = agc.get("TEMPG");
        agc.set("-PHASE" + (g / 2), (~agc.get("TEMPP")) & Agc.WORD_MASK);
        // RELINT ; DTCB to caller
    }

    /** CON2: store PHASEn, priority and name (PHSNAME), then BELOW1. */
    public void con2() {
        int g = agc.get("TEMPG");
        agc.set("PHASE" + (g / 2), agc.get("TEMPP"));
        agc.set("PHSPRDT" + (g / 2), agc.get("TEMPPR"));
        agc.set("PHSNAME" + (g / 2), agc.get("TEMPNM"));
        agc.set("PHSNAME" + (g / 2) + "+1", agc.get("TEMPNM+1"));
        below1();
    }

    // Aliases (EQUALS) -> erasable temporaries
    public static final String TEMPG = "ITEMP1";
    public static final String TEMPP = "ITEMP2";
    public static final String TEMPNM = "ITEMP3";
    public static final String TEMPBB = "ITEMP4";
    public static final String TEMPSW = "ITEMP5";
    public static final String TEMPSW2 = "ITEMP6";
    public static final String TEMPPR = "RUPTREG1";
    public static final String TEMPG2 = "RUPTREG2";
    public static final String TEMPP2 = "RUPTREG3";
    public static final String TEMPBBCN = "RUPTREG4";
    public static final String BB = "BBANK";
}
