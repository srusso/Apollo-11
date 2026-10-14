package apollo.comanche055;

import apollo.agc.Agc;

/**
 * DOWN-TELEMETRY_PROGRAM (Comanche 055): the DOWNRUPT service. Each downrupt sends two
 * words (DNTM1/DNTM2) via channels 34/35. The program is a state machine driven by
 * DNTMGOTO: DNPHASE1 (initialise), DNPHASE2 (list processing), DNDUMP/DNDUMP1 (erasable dump).
 *
 * Erasable control words: DNECADR (= TMINDEX), CTLIST (= LDATALST), SUBLIST (= DNQ),
 * DNLSTCOD, DUMPLOC, DUMPCNT, DNTMGOTO. Word-order code is bit 7 of channel 13.
 */
public final class DownTelemetryProgram {
    private static final int BIT7 = 0100;
    private static final int LOW8 = 0377;
    private static final int NEGONE = Agc.fromSigned(-1);
    private static final int MINB1314 = 047777;
    private static final int MINB12 = 073777; // -1/8
    private static final int DNADRDCR = 074001; // DNADR count and ECADR decrementer
    private static final int PRIO34 = 034000;

    /** State selected by DNTMGOTO. */
    enum Phase { DNPHASE1, DNPHASE2, DNDUMP, DNDUMP1 }

    private final Agc agc;
    private Phase dntmGoto = Phase.DNPHASE1;
    private int chan13;
    private int dntm1;
    private int dntm2;

    public DownTelemetryProgram(Agc agc) {
        this.agc = agc;
    }

    /** DODOWNTM: DOWNRUPT entry. Set word order code to 1, dispatch on DNTMGOTO. */
    public void dodowntm() {
        chan13 |= BIT7;
        switch (dntmGoto) {
            case DNPHASE1 -> dnphase1();
            case DNPHASE2 -> dnphase2();
            case DNDUMP -> dndump();
            case DNDUMP1 -> dndump1();
        }
        // TMRESUME: RESUME
    }

    /** DNPHASE1: initialise control words to -1, next phase is DNPHASE2, start new list. */
    void dnphase1() {
        agc.set("SUBLIST", NEGONE);
        agc.set("DNECADR", NEGONE);
        dntmGoto = Phase.DNPHASE2;
        newlist();
    }

    /** DNPHASE2: continue sending data. */
    void dnphase2() {
        if (Agc.toSigned(agc.get("DNECADR")) > 0) { // CCS DNECADR: sending in progress
            fetch2wd(); // DODNADR
            return;
        }
        if (Agc.toSigned(agc.get("DNECADR")) < 0 || true) {
            // CCS fall-through (-1DNADR TIME2 / TCF +1): check sublist
            if (Agc.toSigned(agc.get("SUBLIST")) > 0) {
                nextinsl();
                return;
            }
        }
        chklist();
    }

    /** CHKLIST: end of current list (CTLIST negative) starts a new list, else next in list. */
    void chklist() {
        if (Agc.toSigned(agc.get("LDATALST")) <= 0) {
            newlist();
        } else {
            nextincl();
        }
    }

    /** NEWLIST: initialise CTLIST with start of the new list selected by DNLSTCOD, send ID. */
    void newlist() {
        agc.set("LDATALST", agc.get("DNTABLE+" + agc.get("DNLSTCOD")));
        int a = Agc.fromSigned(-Agc.toSigned(agc.get("DNLSTCOD"))); // CS DNLSTCOD
        sendid(a);
    }

    /** NEXTINCL: fetch next entry of the control list. */
    void nextincl() {
        int entry = agc.get("CTL@" + agc.get("LDATALST"));
        int a;
        if (Agc.toSigned(entry) > 0) {
            agc.set("LDATALST", agc.get("LDATALST") + 1);
            a = entry;
        } else {
            // last entry: mark CTLIST negative, use complemented DNADR
            agc.set("LDATALST", Agc.fromSigned(-1));
            a = (~entry) & Agc.WORD_MASK;
        }
        a = Agc.add(a, 1); // INCR A
        agc.set("DNECADR", a);
        if (Agc.toSigned(Agc.add(a, agc.get("MINTIME2"))) > 0) {
            setwo1(); // don't set word order code
        } else {
            setwo();
        }
    }

    /** SETWO: set word order code to zero, then classify DNECADR. */
    void setwo() {
        wozero();
        setwo1();
    }

    void setwo1() {
        int a = agc.get("DNECADR");
        if (Agc.toSigned(Agc.add(a, MINB1314)) <= 0) { // regular DNADR
            fetch2wd();
            return;
        }
        if (Agc.toSigned(Agc.add(Agc.add(a, MINB1314), MINB12)) <= 0) {
            dodnptr();
        } else {
            dodnchan();
        }
    }

    /** DODNCHAN: DNECADR names two I/O channels; read both and send. */
    void dodnchan() {
        int ch = agc.get("DNECADR");
        int hi = agc.get("CHAN" + (ch & 077)); // READ -4000 offset
        int lo = agc.get("CHAN" + ((ch & 077) - 1));
        agc.set("DNECADR", NEGONE);
        dntmexit(hi, lo);
    }

    /** WOZERO: set word order code to zero. */
    void wozero() {
        chan13 &= ~BIT7;
    }

    /** DODNPTR: DNECADR points at a sublist; snapshot sublist or regular sublist. */
    void dodnptr() {
        int first = agc.get("SUBL@" + agc.get("DNECADR"));
        if (Agc.toSigned(first) > 0) { // regular sublist (CA DNECADR; DOSUBLST)
            dosublst(agc.get("DNECADR"));
            return;
        }
        // snapshot sublist: copy pairs of erasable words into DNTMBUFF
        agc.set("SUBLIST", agc.get("DNECADR"));
        int tmindex = 0;
        int a = first;
        int hi = 0, lo = 0;
        while (true) { // SNAPLOOP
            hi = agc.get("EADDR@" + a);
            lo = agc.get("EADDR@" + (a + 1));
            agc.set("DNTMBUFF+" + tmindex, hi);
            agc.set("DNTMBUFF+" + (tmindex + 1), lo);
            tmindex += 2;
            agc.set("SUBLIST", agc.get("SUBLIST") + 1); // SNAPAGN
            a = agc.get("SUBL@" + agc.get("SUBLIST"));
            if (Agc.toSigned(a) <= 0) {
                break; // last two words
            }
        }
        dntmGoto = Phase.DNPHASE2; // LDNPHAS2
        agc.set("DNECADR", NEGONE);
        agc.set("SUBLIST", NEGONE);
        dntmexit(hi, lo); // SNAPEND
    }

    /** FETCH2WD: fetch two data words at DNECADR, decrement count/ECADR, send. */
    void fetch2wd() {
        int ecadr = agc.get("DNECADR");
        int hi = agc.get("EADDR@" + ecadr);
        int lo = agc.get("EADDR@" + (ecadr + 1));
        agc.set("DNECADR", Agc.add(ecadr, DNADRDCR));
        dntmexit(hi, lo);
    }

    /** DOSUBLST: set SUBLIST pointer then fall into NEXTINSL. */
    void dosublst(int a) {
        agc.set("SUBLIST", a);
        nextinsl();
    }

    /** NEXTINSL: next entry in sublist; end of sublist resets SUBLIST to -1. */
    void nextinsl() {
        int a = agc.get("SUBL@" + agc.get("SUBLIST"));
        if (Agc.toSigned(a) > 0) {
            agc.set("SUBLIST", agc.get("SUBLIST") + 1);
        } else {
            agc.set("SUBLIST", NEGONE);
        }
        a = Agc.add(a, 1);
        agc.set("DNECADR", a);
        setwo1(); // TCF SETWO +2
    }

    /** DNTMEXIT: write A to DNTM1 and L to DNTM2 (channels 34, 35). */
    void dntmexit(int a, int l) {
        dntm1 = a;
        dntm2 = l;
        agc.set("CHAN34", a);
        agc.set("CHAN35", l); // TMEXITL / TMRESUME
    }

    /** DNDUMPI: initialise downlink erasable dump. */
    void dndumpi() {
        agc.set("DUMPLOC", 0);
        dndumpi2();
    }

    /** DNDUMPI +2: send ID and synch bits, send DUMPLOC and TIME1. */
    void dndumpi2() {
        sendid(0);
        dntmGoto = Phase.DNDUMP1; // LDNDUMP1
        dntmexit(agc.get("DUMPLOC"), agc.get("TIME1"));
    }

    /** DNDUMP: advance DUMPLOC by 2, handle bank change. */
    void dndump() {
        int loc = Agc.add(2, agc.get("DUMPLOC"));
        agc.set("DUMPLOC", loc);
        if ((loc & LOW8) == 0) { // new EBANK
            int a = loc & agc.get("DUMPCNT") & PRIO34;
            if (a != 0) {
                dnphase1(); // dump complete: resume interrupted downlist
            } else {
                dndumpi2();
            }
            return;
        }
        dndump2();
    }

    /** DNDUMP1: set DNTMGOTO for words 3..256 of the current EBANK. */
    void dndump1() {
        dntmGoto = Phase.DNDUMP; // LDNDUMP
        dndump2();
    }

    /** DNDUMP2: pick up a pair of erasable registers and send. */
    void dndump2() {
        int loc = agc.get("DUMPLOC");
        int rel = loc & LOW8;
        int lo = agc.get("EBANK@" + (loc >> 8) + "," + (rel + 1));
        int hi = agc.get("EBANK@" + (loc >> 8) + "," + rel);
        dntmexit(lo, hi);
    }

    /** SENDID: send downlist ID code(s) with word-order code zero. */
    void sendid(int a) {
        wozero();
        int l = agc.get("LOWIDCOD");
        dntmexit(a, l);
    }
}
