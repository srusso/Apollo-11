package apollo.luminary099;

import apollo.agc.Agc;

/**
 * DOWN_TELEMETRY_PROGRAM (Luminary 099, bank 22, DOWNTELM).
 * Invoked on every DOWNRUPT; sends two 15-bit words (A to channel 34, L to channel 35) per interrupt.
 * Modelled as a state machine: the phase to run on the next DOWNRUPT is held in {@link #dntmGoto}
 * (the AGC keeps a return address in DNTMGOTO).
 */
public class DownTelemetryProgram {
    /** Phases the program may be in on the next DOWNRUPT (DNTMGOTO). */
    public enum Phase { DNPHASE1, DNPHASE2, DNDUMP, DNDUMP1, SENDID_RETURN }

    /** DNADRDCR: OCT 74001, DNADR count and ECADR decrementer. */
    public static final int DNADRDCR = 074001;
    /** MINB1314: minus bits 13 and 14. */
    public static final int MINB1314 = 047777;
    /** MINB12: -1/8 (octal 77767 region), the DNPTR threshold. */
    public static final int MINB12 = 077767;
    public static final int NEGONE = 077776;
    public static final int NEG0 = 077777;
    public static final int BIT7 = 0100;
    public static final int LOW8 = 0377;

    private final Agc agc;
    private Phase dntmGoto = Phase.DNPHASE1;
    private int word1;   // A register to send (channel 34)
    private int word2;   // L register to send (channel 35)
    private boolean wordOrder; // CHAN13 bit 7

    public DownTelemetryProgram(Agc agc) {
        this.agc = agc;
    }

    /** DODOWNTM: entry on DOWNRUPT. Tests the word-order bit, then jumps to the current phase. */
    public void dodowntm() {
        // WOTEST: at the beginning of the list the word order bit is set back to zero
        if (!wordOrder) {
            wordOrder = true;     // WO1: set word order bit to 1 only if it isn't already
            // falls to the phase dispatch below
        } else {
            wordOrder = true;
        }
        switch (dntmGoto) {
            case DNPHASE1: dnphase1(); break;
            case DNPHASE2: dnphase2(); break;
            case DNDUMP: dndump(); break;
            case DNDUMP1: dndump1(); break;
            case SENDID_RETURN: dnphase1(); break;
            default: break;
        }
    }

    /** DNPHASE1: initialise all control words to -1, set next phase to DNPHASE2, start a new list. */
    public void dnphase1() {
        agc.set("SUBLIST", NEGONE);
        agc.set("DNECADR", NEGONE);
        dntmGoto = Phase.DNPHASE2;
        newlist();
    }

    /** DNPHASE2: continue sending data (a fetch of 2 words, sublist or next in list). */
    public void dnphase2() {
        int dnecadr = Agc.toSigned(agc.get("DNECADR"));
        if (dnecadr > 0) {
            fetch2wd();                       // sending of data in progress
            return;
        }
        if (dnecadr == 0) {                   // CCS "+0": fall through (TIME2 case) 
            fetch2wd();
            return;
        }
        if (Agc.toSigned(agc.get("SUBLIST")) > 0) {
            nextinsl();                       // sublist in control
            return;
        }
        chklist();
    }

    /** CHKLIST: negative CTLIST means end of list -> NEWLIST, else NEXTINCL. */
    public void chklist() {
        if (Agc.toSigned(agc.get("CTLIST")) <= 0) {
            newlist();
        } else {
            nextincl();
        }
    }

    /** NEWLIST: initialise CTLIST with the starting address of the list selected by DNLSTCOD; send its ID. */
    public void newlist() {
        agc.set("CTLIST", agc.get("DNTABLE+" + agc.get("DNLSTCOD")));
        sendid(Agc.fromSigned(-Agc.toSigned(agc.get("DNLSTCOD"))), true);
    }

    /** NEXTINCL: fetch next CTLIST entry (DNADR code word) and process it. */
    public void nextincl() {
        int ptr = agc.get("CTLIST");
        int a = agc.get("LIST+" + ptr);
        if (Agc.toSigned(a) > 0) {
            agc.set("CTLIST", Agc.add(ptr, 1));    // point at the next CTLIST word on next entry
        } else {
            // last word of list: CTLIST := -(complement of the DNADR)
            agc.set("CTLIST", Agc.fromSigned(-Math.abs(Agc.toSigned(ptr))));
            a = ~a & Agc.WORD_MASK;
        }
        a = Agc.add(a, 1);
        agc.set("DNECADR", a);
        // TIME2 is sent with no word-order change; compare with -TIME2 address
        if (Agc.toSigned(Agc.add(a, agc.get("MINTIME2"))) > 0) {
            setwo();
        } else {
            setwoPlus1();
        }
    }

    /** SETWO: set word order code to zero, then fall into SETWO+1. */
    public void setwo() {
        wozero();
        setwoPlus1();
    }

    /** SETWO+1: reload DNADR. */
    public void setwoPlus1() {
        setwoPlus2(agc.get("DNECADR"));
    }

    /** SETWO+2: classify the code word as DNADR (regular), DNPTR (pointer) or DNCHAN (channel). */
    public void setwoPlus2(int a) {
        if (Agc.toSigned(Agc.add(a, MINB1314)) <= 0) {
            fetch2wd();                      // regular DNADR
        } else if (Agc.toSigned(Agc.add(Agc.add(a, MINB1314), MINB12)) <= 0) {
            dodnptr();                       // pointer to sublist
        } else {
            dodnchan();                      // channel pair
        }
    }

    /** DODNCHAN: read two I/O channels (DNECADR-based) and send them; DNECADR := -1. */
    public void dodnchan() {
        int ch1 = agc.get("DNECADR");
        int a = agc.get("CHAN+" + (ch1 & 0777));       // first channel read
        int l = a;
        agc.set("DNECADR", NEGONE);
        dntmexit(a, l);
    }

    /** WOZERO: clear the word-order bit in channel 13. */
    public void wozero() {
        wordOrder = false;
        agc.set("CHAN13", agc.get("CHAN13") & ~BIT7);
    }

    /** DODNPTR: pointer to a sublist; negative first entry marks a snapshot sublist. */
    public void dodnptr() {
        int entry = agc.get("DNECADR");
        int first = agc.get("SUBL+" + entry);
        if (Agc.toSigned(first) > 0) {
            dosublst(entry);                 // regular sublist
            return;
        }
        // snapshot sublist: C(DNECADR) -> SUBLIST; collect pairs into DNTMBUFF
        agc.set("SUBLIST", entry);
        int tmindex = 0;
        agc.set("TMINDEX", 0);
        int a = 0;
        for (;;) {
            // SNAPLOOP: pick up two snapshot words from erasable (EBANK from a, relative address in LOW8)
            int hi = agc.get("ERASE+" + (a & LOW8));
            int lo = agc.get("ERASE+" + ((a & LOW8) + 1));
            agc.set("DNTMBUFF+" + tmindex, hi);
            agc.set("DNTMBUFF+" + (tmindex + 1), lo);
            tmindex += 2;
            agc.set("TMINDEX", tmindex);
            // SNAPAGN: next entry of sublist
            agc.set("SUBLIST", Agc.add(agc.get("SUBLIST"), 1));
            a = agc.get("SUBL+" + agc.get("SUBLIST"));
            if (Agc.toSigned(a) <= 0) {
                break;                       // last two words
            }
        }
        // LDNPHAS2: last; set DNECADR and SUBLIST negative, send first two words from buffer
        agc.set("DNECADR", NEGONE);
        agc.set("SUBLIST", NEGONE);
        snapend(agc.get("DNTMBUFF"), agc.get("DNTMBUFF+1"));
    }

    /** SNAPEND: send the first two snapshot words. */
    public void snapend(int a, int l) {
        dntmexit(a, l);
    }

    /** FETCH2WD: fetch 2 words from erasable at DNECADR, then decrement count and ECADR. */
    public void fetch2wd() {
        int ecadr = agc.get("DNECADR");
        int rel = ecadr & LOW8;
        agc.set("DNECADR", Agc.add(ecadr, DNADRDCR));
        dntmexit(agc.get("ERASE+" + rel), agc.get("ERASE+" + (rel + 1)));
    }

    /** DOSUBLST: set the sublist pointer, then take the next entry. */
    public void dosublst(int a) {
        agc.set("SUBLIST", a);
        nextinsl();
    }

    /** NEXTINSL: take next sublist entry; negative entry ends the sublist. */
    public void nextinsl() {
        int a = agc.get("SUBL+" + agc.get("SUBLIST"));
        if (Agc.toSigned(a) > 0) {
            agc.set("SUBLIST", Agc.add(agc.get("SUBLIST"), 1));
        } else {
            agc.set("SUBLIST", NEGONE);
        }
        a = Agc.add(a, 1);
        agc.set("DNECADR", a);
        setwoPlus2(a);
    }

    /** DNTMEXIT: send A to channel 34 (DNTM1) and L to channel 35 (DNTM2), then resume. */
    public void dntmexit(int a, int l) {
        word1 = a;
        word2 = l;
        agc.set("DNTM1", a);
        agc.set("DNTM2", l);
        tmresume();
    }

    /** TMRESUME: exit via RESUME. */
    public void tmresume() {
        // TCF RESUME
    }

    /** DNDUMPI: initialise the downlink erasable dump (+2: skip clearing DUMPLOC). */
    public void dndumpi() {
        agc.set("DUMPLOC", 0);
        dndumpiPlus2();
    }

    /** DNDUMPI+2: send ID/synch bits; next pass goes to DNDUMP1; sends DUMPLOC and TIME1. */
    public void dndumpiPlus2() {
        sendid(agc.get("ERASID"), false);
        dntmGoto = Phase.DNDUMP1;
        dntmexit(agc.get("DUMPLOC"), agc.get("TIME1"));
    }

    /** DNDUMP: advance DUMPLOC by 2; at the start of a new bank, check for end of the N-th complete dump. */
    public void dndump() {
        int dumploc = Agc.add(agc.get("DUMPLOC"), 2);
        agc.set("DUMPLOC", dumploc);
        if ((dumploc & LOW8) != 0) {
            dndump2();
            return;
        }
        int a = dumploc & agc.get("DUMPCNT") & agc.get("PRIO34");
        if (a != 0) {
            dnphase1();                      // resume interrupted downlist
        } else {
            dndumpiPlus2();                  // initialise next bank
        }
    }

    /** DNDUMP1: set DNTMGOTO to DNDUMP for words 3-256 of the bank, then dump. */
    public void dndump1() {
        dntmGoto = Phase.DNDUMP;
        dndump2();
    }

    /** DNDUMP2: send the next pair of erasable registers (masked so editing registers are not altered). */
    public void dndump2() {
        int dumploc = agc.get("DUMPLOC");
        int rel = dumploc & LOW8;
        int l = agc.get("ERASE+" + (rel + 1)) & NEG0;
        int a = agc.get("ERASE+" + rel) & NEG0;
        dntmexit(a, l);
    }

    /**
     * SENDID: entrance used by the erasable dump program ({@code resume} is true when called from the
     * regular downlink at SENDID+3, which keeps DNTMGOTO). Sends ID code and special ID.
     */
    public void sendid(int id, boolean regular) {
        if (!regular) {
            dntmGoto = Phase.DNDUMP1;        // next time go to location after TC SENDID
        }
        wozero();
        dntmexit(id, agc.get("LOWIDCOD"));   // downlist ID code(s)
    }

    public Phase getPhase() { return dntmGoto; }
    public int lastWord1() { return word1; }
    public int lastWord2() { return word2; }
    public boolean wordOrderBit() { return wordOrder; }
}
