package apollo.comanche055;

import apollo.agc.Agc;

/**
 * JET_SELECTION_LOGIC.agc (bank 21, DAPS4/DAPS5, COUNT 17/DAPJS).
 * <p>
 * RCS jet selection for the Command Module DAP, run from T5RUPT (phase 1) with the T6 program
 * that sequences the jet channel commands. JETSLECT examines the translation hand controller
 * (channel 31), then builds, for each of roll, pitch and yaw, a first channel word (WORD1, jets
 * for rotation plus translation) and second word (WORD2, translation only) and the jet on-times
 * (BLAST, BLAST1, BLAST2, in T6 counts of 1/1600 s). The on-times are sorted and the T6 program
 * (T6START) replaces the words as the times elapse.
 * <p>
 * Words are 15-bit AGC words held in ints. Channels 5, 6, 13, 31 are modelled in the Agc
 * erasable map as CHAN5, CHAN6, CHAN13, CHAN31. "MP BITn" of a positive word is a right shift.
 */
public final class JetSelectionLogic {
    private final Agc agc;

    // ---------------------------------------------------------------- constants
    /** XLNMASK 7700 octal: translation bits of channel 31. */
    public static final int XLNMASK = 07700;
    /** DELTATT3 = 60 ms; DELATT20 = 20 ms (T5 counts, 1 count = 1/1600 s). */
    public static final int DELTATT3 = 16378, DELATT20 = 16382;
    /** Index of translation commands for table look-up. */
    public static final int[] XLNNDX = {0, 3, 6, 0};
    /** Indices for Y/Z translation table (YZTABLE). */
    public static final int[] XLN1NDX = {0, 1, 2, 0};
    public static final int TWELVE = 012, NINE = 9;
    /** Masks for pitch and yaw commands. */
    public static final int PJETS = 01417, YJETS = 06360;
    /** RTABLE masks. */
    public static final int ACRJETS = 03760, BDRJETS = 034017;
    /** YZTABLE masks. */
    public static final int BDZJETS = 03417, ACYJETS = 034360;
    /** 14 ms minimum impulse (T6 counts). */
    public static final int MS14 = 23;
    /** DFTMAX table, indexed by number of jets -3..+3: -.3,-.2,-.1,0,+.1,+.2,+.3 s. */
    public static final int[] DFTMAX = {-480, -320, -160, 0, 160, 320, 480};
    public static final int PLUS_TENTH_SEC = 160, MINUS_TENTH_SEC = -160;
    /** NJET table, indexed -3..+3: -1/3,-1/2,-1,0,+1,+1/2,+1/3 (14-bit fractions). */
    public static final int[] NJET = {-5461, -8192, -16383, 0, 16383, 8192, 5461};

    /** Table for pitch (yaw) commands with X-translation and failures:
     *  ROT 0,+,- x TRANS 0,+,- ; then failure biases 9 (A(B)) and 12 (C(D)). */
    public static final int[] PYTABLE = {
        00, 05125, 05252,
        00231, 02421, 02610,
        00146, 02504, 02442,
        00, 02421, 02442,
        00, 02504, 02610,
    };

    /** Roll table (AC roll in bits 9-11, BD roll in bits 12-14). */
    public static final int[] RTABLE = {
        011000, 022125, 000252,
        011231, 015421, 004610,
        011146, 015504, 004442,
        011000, 015504, 004610,
        011000, 015421, 004442,
    };

    /** Y, Z translation table. */
    public static final int[] YZTABLE = {
        011000, 011231, 011146,
        011000, 004610, 015504,
        011000, 015421, 004442,
    };

    public JetSelectionLogic(Agc agc) {
        this.agc = agc;
    }

    // ---------------------------------------------------------------- helpers
    private int g(String sym) { return Agc.toSigned(agc.get(sym)); }
    private void s(String sym, int v) { agc.set(sym, Agc.fromSigned(v)); }
    /** CCS sign classification of a word: 1 positive, 0 +0/-0, -1 negative. */
    private static int sign(int v) { return Integer.signum(v); }

    /** Table lookup for number of jets index n in -3..3. */
    private static int tab(int[] t, int n) { return t[n + 3]; }

    // ---------------------------------------------------------------- JETSLECT
    /**
     * JETSLECT: entered from T5RUPT. Resets TIME5 to execute phase 1 in 60 ms (DELTATT3 plus
     * T5TIME), resets T6 to 14 ms, enables T6RUPT, examines the translation hand controller on
     * channel 31 and sets the translation indices XNDX1, XNDX2 (X, AC/BD quads), YNDX, ZNDX
     * (1 = +, 2 = -, 3 = none), then continues with the pitch, yaw and roll selection.
     * (An alternate entry sets TIME5 to DELATT20 = 20 ms to assure a T5RUPT.)
     */
    public void jetslect() {
        s("TIME5", DELTATT3 + g("T5TIME"));       // RESET TO EXECUTE PHASE1
        s("TIME6", MS14);                          // RESET T6 TO INITIALIZE THE JET CHANNELS IN 14 MS
        agc.set("CHAN13", agc.get("CHAN13") | 0x4000);   // WOR CHAN13 with NEGMAX: enable T6RUPT

        // EXAMINE THE TRANSLATION HAND CONTROLLER: RXOR CHAN31 then mask
        int a = (XLNMASK ^ agc.get("CHAN31")) & XLNMASK;   // A = XLNMASK XOR CHAN31, masked
        if (a == 0) {
            noxlncmd();
        } else {
            int t5temp = a;
            agc.set("T5TEMP", t5temp);
            int x = (t5temp >> 6) & 3;                      // MP BIT9, MASK THREE
            agc.set("XNDX1", x);                            // AC quad X-translation index
            agc.set("XNDX2", x);                            // BD quad X-translation index
            agc.set("YNDX", (t5temp >> 8) & 3);             // 1 = +, 2 = -, 3 = none
            agc.set("ZNDX", (t5temp >> 10) & 3);
            // LEM attached (DAPDATR1 bit 14): ATTKALMN = -3, else -2
            boolean lem = (agc.get("DAPDATR1") & 020000) != 0;
            s("ATTKALMN", lem ? -3 : -2);
            int xtrans = g("XTRANS");                       // (+, -1, 0)
            if (xtrans > 0) {
                agc.set("XNDX1", 0);                        // using BD-X: zero XNDX1
            } else if (xtrans < 0) {
                agc.set("XNDX2", 0);                        // using AC-X: zero XNDX2
            }
        }
        pword();
    }

    /** NOXLNCMD: zero all requests for translation. */
    private void noxlncmd() {
        agc.set("XNDX1", 0);
        agc.set("XNDX2", 0);
        agc.set("YNDX", 0);
        agc.set("ZNDX", 0);
    }

    // ---------------------------------------------------------------- PITCH
    /** PWORD: pitch commands. PINDEX: 0 none, 1 +pitch, 2 -pitch. Table look-up for PWORD1. */
    private void pword() {
        int tau1 = g("TAU1");
        int pindex = tau1 > 0 ? 1 : (tau1 < 0 ? 2 : 0);
        agc.set("PINDEX", pindex);
        int racfail = g("RACFAIL");                          // flag for real AC quad failures
        int idx;
        if (racfail > 0) idx = NINE;                         // AFAILP: 1-jet operation, ignore X-translation
        else if (racfail < 0) idx = TWELVE;                  // CFAILP
        else idx = XLNNDX[agc.get("XNDX1") & 3];             // no failures: include X translation
        // TABPCOM
        int w = PYTABLE[idx + pindex] & PJETS;
        agc.set("PWORD1", w);
        agc.set("NPJETS", (w * 64) >> 14);                   // MP BIT7: number of pitch jets
        ywordSection();
    }

    // ---------------------------------------------------------------- YAW
    /** YWORD: yaw commands, same scheme as pitch using RBDFAIL and XNDX2. */
    private void ywordSection() {
        int tau2 = g("TAU2");
        int yindex = tau2 > 0 ? 1 : (tau2 < 0 ? 2 : 0);
        agc.set("YINDEX", yindex);
        int rbdfail = g("RBDFAIL");
        int idx;
        if (rbdfail > 0) idx = NINE;                         // BFAILY
        else if (rbdfail < 0) idx = TWELVE;                  // DFAILY
        else idx = XLNNDX[agc.get("XNDX2") & 3];
        // TABYCOM
        int w = PYTABLE[idx + yindex] & YJETS;
        agc.set("YWORD1", w);
        agc.set("NYJETS", (w * 16) >> 14);                   // MP BIT5: number of yaw jets
        rword();
    }

    // ---------------------------------------------------------------- ROLL
    /** RWORD: roll commands. ACORBD >= 0 selects BD roll; negative selects AC roll. */
    private void rword() {
        int tau = g("TAU");
        int rindex = tau > 0 ? 1 : (tau < 0 ? 2 : 0);
        agc.set("RINDEX", rindex);
        if (g("ACORBD") >= 0) {
            bdroll(rindex);
        } else {
            acroll(rindex);
        }
    }

    /** ACROLL: AC roll; Y-translation by the same quads unless a real failure is present. */
    private void acroll(int rindex) {
        int racfail = g("RACFAIL");
        int idx;
        if (racfail > 0) idx = NINE;                          // RAFAIL: 1-jet operation
        else if (racfail < 0) idx = TWELVE;                   // RCFAIL
        else idx = XLNNDX[agc.get("YNDX") & 3];               // RXLNS: include Y translation
        int rword1 = RTABLE[idx + rindex] & ACRJETS;          // TABRCOM
        agc.set("RWORD1", rword1);
        // BDZCHECK: Z-translation on BD jets
        int zndx = g("ZNDX");
        if (zndx > 0) {
            int bias = bdBias();                              // CCS RBDFAIL: 0 / 3 / 6
            int yz = YZTABLE[bias + XLN1NDX[zndx]] & BDZJETS;
            int t5temp = yz + rword1;
            agc.set("T5TEMP", t5temp);
            int nrjets = ((t5temp * 64) >> 14) - 4;           // net no. of +,- roll jets on
            agc.set("NRJETS", nrjets);
            if (nrjets != 0 || g("TAU") == 0) {               // ACRBDZ / TAUCHECK
                agc.set("RWORD1", t5temp);                    // Z translation accepted
                rolltime();
                return;
            }
        }
        // NOBDZ: Z-translation not accepted
        int w = agc.get("RWORD1");
        agc.set("NRJETS", ((w * 64) >> 14) - 2);
        rolltime();
    }

    /** CCS RBDFAIL (or RACFAIL) bias into YZTABLE: 0 none, 3 positive, 6 negative. */
    private int bdBias() {
        int f = g("RBDFAIL");
        return f > 0 ? 3 : (f < 0 ? 6 : 0);
    }

    private int acBias() {
        int f = g("RACFAIL");
        return f > 0 ? 3 : (f < 0 ? 6 : 0);
    }

    /** BDROLL: BD quad selection for roll commands (Z-translation on same quads). */
    private void bdroll(int rindex) {
        int rbdfail = g("RBDFAIL");
        int idx;
        if (rbdfail > 0) idx = NINE;                          // RBFAIL
        else if (rbdfail < 0) idx = TWELVE;                   // RDFAIL
        else idx = XLNNDX[agc.get("ZNDX") & 3];               // RZXLNS
        int rword1 = RTABLE[idx + rindex] & BDRJETS;          // TABRZCMD
        agc.set("RWORD1", rword1);
        // ACYCHECK: any Y-translation (on the AC quads)?
        int yndx = g("YNDX");
        if (yndx != 0) {
            int yz = YZTABLE[acBias() + XLN1NDX[yndx]] & ACYJETS;
            int t5temp = yz + rword1;
            agc.set("T5TEMP", t5temp);
            int nrjets = ((t5temp * 8) >> 14) - 4;            // MP BIT4
            agc.set("NRJETS", nrjets);
            if (nrjets != 0 || g("TAU") == 0) {               // BDRACZ: Y accepted
                agc.set("RWORD1", t5temp);
                rolltime();
                return;
            }
        }
        // NOACY: Y translation not accepted
        int w = agc.get("RWORD1");
        agc.set("NRJETS", ((w * 8) >> 14) - 2);
        rolltime();
    }

    // ---------------------------------------------------------------- jet on-times
    /** ROLLTIME: roll jet on-time calculation. */
    private void rolltime() {
        int tau = g("TAU");
        int nrjets = g("NRJETS");
        if (tau == 0) {
            s("DFT", tab(DFTMAX, nrjets));                    // update DFT even though no roll commands
            rbypass();
            return;
        }
        int blast = (tau * tab(NJET, nrjets)) >> 14;          // MP NJET
        if (blast - PLUS_TENTH_SEC <= 0) {
            // AD14MSR: on-time <= .1 sec: limit minimum on time to 14 ms
            if (blast < MS14) blast = MS14;
            s("BLAST", blast);
            s("DFT", blast * nrjets);                         // LXCH DFT : DFT = BLAST * NRJETS
            s("TAU", 0);                                      // roll commands complete in this cycle
        } else {
            s("DFT", tab(DFTMAX, nrjets));                    // longer than T5 cycle
            s("TAU", tau - tab(DFTMAX, nrjets));              // COM, ADS TAU
            blast = PLUS_TENTH_SEC;                           // limit to .1 sec
            s("BLAST", blast);
        }
        asmblwr();
    }

    /** RBYPASS: no roll commands: RWORD2 = RWORD1, BLAST = 0. */
    private void rbypass() {
        agc.set("RWORD2", agc.get("RWORD1"));
        s("BLAST", 0);
        pitchtim();
    }

    /** ASMBLWR: generation of the second roll (Y,Z) word RWORD2. */
    private void asmblwr() {
        int yndx = g("YNDX");
        agc.set("RWORD2", 0);
        if (yndx > 0) {                                       // ACBD2Y
            if (g("ACORBD") >= 0) {                           // AC2Y: BD roll, Y-trans by AC jets
                int yz = YZTABLE[acBias() + XLN1NDX[yndx]] & ACYJETS;
                agc.set("RWORD2", yz);
                int nrjets = ((yz * 8) >> 14) - 2;            // MP BIT4
                agc.set("NRJETS", nrjets);
                int l = (((g("BLAST") * -1) + PLUS_TENTH_SEC) * nrjets);   // CS BLAST; AD +.1SEC; MP NRJETS
                s("DFT", g("DFT") + l);
            } else if (g("RACFAIL") == 0) {                   // AC roll, AC OK
                int w = RTABLE[XLNNDX[yndx]] & ACRJETS;
                agc.set("RWORD2", w);
            }
        }
        int zndx = g("ZNDX");
        if (zndx > 0) {                                       // ACBD2Z
            if (g("ACORBD") >= 0) {                           // BDF2Z: using BD roll
                if (g("RBDFAIL") == 0) {
                    int w = RTABLE[XLNNDX[zndx]] & BDRJETS;
                    agc.set("RWORD2", agc.get("RWORD2") + w);
                }
            } else {                                          // using AC for roll: can do Z-trans
                int yz = YZTABLE[bdBias() + XLN1NDX[zndx]] & BDZJETS;
                agc.set("RWORD2", agc.get("RWORD2") + yz);
                int nrjets = ((yz * 64) >> 14) - 2;           // MP BIT7
                agc.set("NRJETS", nrjets);
                int l = (((g("BLAST") * -1) + PLUS_TENTH_SEC) * nrjets);
                s("DFT", g("DFT") + l);
            }
        }
        pitchtim();                                           // RWORD2 assembled
    }

    /** PITCHTIM: pitch jet on-time calculation. */
    private void pitchtim() {
        int tau1 = g("TAU1");
        int npjets = g("NPJETS");
        if (tau1 == 0) {
            s("DFT1", 0);                                     // no pitch rotation commands
            // PBYPASS: T6 program will load PWORD2
            agc.set("PWORD2", agc.get("PWORD1"));
            s("BLAST1", 0);
            yawtime();
            return;
        }
        if (tau1 < 0) {                                       // PTAUNEG
            npjets = -npjets;
            s("NPJETS", npjets);
        }
        int blast1 = (tau1 * tab(NJET, npjets)) >> 14;
        if (blast1 - PLUS_TENTH_SEC > 0) {
            s("DFT1", tab(DFTMAX, npjets));                   // greater than .1 sec
            s("TAU1", tau1 - tab(DFTMAX, npjets));
            blast1 = PLUS_TENTH_SEC;                          // limit to .1 sec
        } else {
            if (blast1 < MS14) blast1 = MS14;                 // minimum impulse
            s("DFT1", blast1 * npjets);                       // for rate filter
            s("TAU1", 0);
        }
        s("BLAST1", blast1);
        asmblwp();
    }

    /** ASMBLWP: generation of the second pitch (X-trans) word PWORD2. */
    private void asmblwp() {
        int w;
        if (g("RACFAIL") != 0) {
            w = PYTABLE[0];                                   // FPX2: failure on AC, ignore X-translation
        } else {
            w = PYTABLE[XLNNDX[agc.get("XNDX1") & 3]];
        }
        agc.set("PWORD2", w & PJETS);
        yawtime();
    }

    /** YAWTIME: yaw jet on-time calculation. */
    private void yawtime() {
        int tau2 = g("TAU2");
        int nyjets = g("NYJETS");
        if (tau2 == 0) {
            s("DFT2", 0);                                     // no yaw commands
            agc.set("YWORD2", agc.get("YWORD1"));             // YBYPASS
            s("BLAST2", 0);
            t6setup();
            return;
        }
        if (tau2 < 0) {                                       // YTAUNEG
            nyjets = -nyjets;
            s("NYJETS", nyjets);
        }
        int blast2 = (tau2 * tab(NJET, nyjets)) >> 14;
        if (blast2 - PLUS_TENTH_SEC > 0) {
            s("DFT2", tab(DFTMAX, nyjets));
            s("TAU2", tau2 - tab(DFTMAX, nyjets));
            blast2 = PLUS_TENTH_SEC;
        } else {
            if (blast2 < MS14) blast2 = MS14;
            s("DFT2", blast2 * nyjets);
            s("TAU2", 0);
        }
        s("BLAST2", blast2);
        asmblwy();
    }

    /** ASMBLWY: generation of the second yaw (X-trans) word YWORD2. */
    private void asmblwy() {
        int w;
        if (g("RBDFAIL") != 0) {
            w = PYTABLE[0];                                   // FYX2
        } else {
            w = PYTABLE[XLNNDX[agc.get("XNDX2") & 3]];
        }
        agc.set("YWORD2", w & YJETS);
        t6setup();
    }

    // ---------------------------------------------------------------- sorting
    /**
     * T6SETUP: sort the jet on-times. BLAST/BLAST1/BLAST2 are double words whose low part holds
     * the branch index (0 roll, 4 pitch, 11 yaw). Sorted ascending, then converted to successive
     * differences (CALCDT6) and T5LOC reset to RCSATT; the T6 program is initialised.
     */
    private void t6setup() {
        agc.set("BLAST+1", 0);                                // branch index for roll
        agc.set("BLAST1+1", 4);                               // pitch
        agc.set("BLAST2+1", 11);                              // yaw
        // CS BLAST; AD BLAST1; BZMF DXCHT12 : if BLAST1 <= BLAST exchange (T1 GR T2)
        if (g("BLAST1") - g("BLAST") <= 0) swap("BLAST", "BLAST1");
        // CHECKT23
        if (g("BLAST2") - g("BLAST1") <= 0) {
            swap("BLAST1", "BLAST2");                         // DXCHT23
            if (g("BLAST1") - g("BLAST") <= 0) swap("BLAST", "BLAST1");
        }
        // CALCDT6: differences
        s("BLAST2", g("BLAST2") - g("BLAST1"));
        s("BLAST1", g("BLAST1") - g("BLAST"));
        // reset T5LOC to begin PHASE1 (RCS2CADR = RCSATT)
        agc.set("T5LOC", 1);                                  // placeholder for 2CADR RCSATT
        endjets();
    }

    private void swap(String a, String b) {
        int ah = agc.get(a), al = agc.get(a + "+1");
        agc.set(a, agc.get(b)); agc.set(a + "+1", agc.get(b + "+1"));
        agc.set(b, ah); agc.set(b + "+1", al);
    }

    /** ENDJETS: reset bit 1 of RCSFLAGS (T6 initialisation) and T5PHASE for phase 1, then RESUME. */
    private void endjets() {
        agc.set("RCSFLAGS", agc.get("RCSFLAGS") & ~1);
        agc.set("T5PHASE", Agc.WORD_MASK);                    // CS ZERO: -0
        // TCF RESUME
    }

    // ---------------------------------------------------------------- T6
    /**
     * T6START (bank DAPS5): T6RUPT handler. Ignores a T6RUPT that occurred after TIME6 was reset
     * in T5RUPT; initialises channels 5,6 with WORD1 on the first pass, then for each of
     * BLAST/BLAST1/BLAST2 replaces the channel words (REPLACER/REPLACEP/REPLACEY) as their
     * on-times elapse and re-arms TIME6.
     */
    public void t6start() {
        int t6 = g("TIME6");
        if (t6 != 0) return;                                  // CCS TIME6: wait for next T6RUPT
        int flags = agc.get("RCSFLAGS");
        if ((flags & 1) == 0) {                               // bit 1 is 0: set and initialize channels
            agc.set("RCSFLAGS", flags | 1);
            agc.set("CHAN6", agc.get("RWORD1"));
            agc.set("CHAN5", (agc.get("PWORD1") + agc.get("YWORD1")) & Agc.WORD_MASK);
        }
        // T6RUPTOR
        int b = g("BLAST");
        if (b > 0) { zblast("BLAST"); return; }
        if (b >= 0 || isNegZero(agc.get("BLAST"))) {
            replace("BLAST");                                 // +0 / -0
        }
        // T6L1
        int b1 = g("BLAST1");
        if (b1 > 0) { zblast("BLAST1"); return; }
        if (b1 >= 0 || isNegZero(agc.get("BLAST1"))) replace("BLAST1");
        // T6L2
        int b2 = g("BLAST2");
        if (b2 > 0) { zblast("BLAST2"); return; }
        if (b2 == 0) replace("BLAST2");                       // +0 ; negative: RESUME; -0 replace
        else if (isNegZero(agc.get("BLAST2"))) replace("BLAST2");
    }

    private static boolean isNegZero(int w) { return w == Agc.WORD_MASK; }

    /** REPLACE/REPLACE1/REPLACE2: dispatch on the branch index to REPLACER/REPLACEP/REPLACEY. */
    private void replace(String blast) {
        int idx = agc.get(blast + "+1");
        switch (idx) {
            case 0: replacer(); break;
            case 4: replacep(); break;
            case 11: replacey(); break;
            default: break;
        }
        s(blast, -1);                                         // CS ONE : mark done
    }

    /** REPLACER: channel 6 gets RWORD2. */
    private void replacer() { agc.set("CHAN6", agc.get("RWORD2")); }

    /** REPLACEP: keep the yaw bits in channel 5 and add PWORD2. */
    private void replacep() {
        agc.set("CHAN5", ((agc.get("CHAN5") & YJETS) + agc.get("PWORD2")) & Agc.WORD_MASK);
    }

    /** REPLACEY: keep the pitch bits in channel 5 and add YWORD2. */
    private void replacey() {
        agc.set("CHAN5", ((agc.get("CHAN5") & PJETS) + agc.get("YWORD2")) & Agc.WORD_MASK);
    }

    /** ZBLAST/ZBLAST1/ZBLAST2: zero BLAST, TIME6 = old value, enable T6RUPT (ENABT6). */
    private void zblast(String blast) {
        int old = agc.get(blast);
        agc.set(blast, 0);
        agc.set("TIME6", old);
        agc.set("CHAN13", agc.get("CHAN13") | 0x4000);        // WOR CHAN13 NEGMAX
    }

    /** ENDSLECT. */
    public void endslect() { }
}
