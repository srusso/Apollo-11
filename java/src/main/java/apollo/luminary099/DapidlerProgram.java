package apollo.luminary099;

import apollo.agc.Agc;

/**
 * DAP idler program (Luminary 099, $$/DAPID and $$/NEEDL). Runs every 100 ms (T5RUPT) while the LM DAP is
 * not active: starts the DAP when the mode switch and IMU allow, otherwise computes attitude errors and
 * drives the FDAI needles via the IMU CDU error counters.
 *
 * Channels are modelled as erasable symbols "CHAN5", "CHAN6", "CHAN12", "CHAN14", "CHAN30", "CHAN31".
 */
public class DapidlerProgram {
    private final Agc agc;

    // Constants from the fixed-fixed pool and local data
    private static final int BIT3 = 04, BIT4 = 010, BIT5 = 020, BIT6 = 040, BIT10 = 01000, BIT12 = 04000, BIT13 = 010000;
    private static final int BIT13_14 = 030000;
    private static final int POSMAX = 037777;
    private static final int MS100 = 037766;
    /** Bits of RCSFLAGS cleared on initialization (MANFLAG, OCT 03021). */
    public static final int MANFLAG = 03021;
    /** BGIM23: trim gimbal drive enable bits in channel 12. */
    public static final int BGIM23 = 07400;
    /** DAC limits: +/-384 counts max output, 16000 wraps. */
    public static final int DACLIMIT = 384;
    /** One tenth, scaled at 1 (OCT 03146). */
    public static final int ONETENTH = 03146;
    /** DSPLYALT: 100 ms display alternation bit in RCSFLAGS (= BIT4). */
    public static final int DSPLYALT = BIT4;

    public DapidlerProgram(Agc agc) {
        this.agc = agc;
    }

    private int g(String s) { return agc.get(s); }
    private void s(String sym, int v) { agc.set(sym, v); }

    /** CHEKBITS: check mode switch (chan 31 bits 13,14) and IMU usability. */
    public void chekbits() {
        int off = (~g("CHAN31")) & BIT13_14;           // COM; MASK BIT13-14
        if (off == 0) { moreIdle(); return; }          // both set: DAP off, no error display
        int imuOk = (~g("IMODES33")) & BIT6;
        if (imuOk != 0) { jumpDsp(); return; }         // IMU usable: display attitude errors
        s("RCSFLAGS", g("RCSFLAGS") | BIT3);            // IMU not usable: flag for ATT error display init
        shutdown();
    }

    /** CHEKMORE: returns normally if PGNCS has control, else MOREIDLE. (BIT 10 of chan 30, inverted) */
    public boolean chekmore() {
        if ((g("CHAN30") & BIT10) != 0) { moreIdle(); return false; }
        return true;
    }

    /** DAPIDLER: T5RUPT entry. */
    public void dapidler() {
        // LXCH BANKRUPT; QXCH QRUPT (interrupt lead-in continuation)
        if ((g("RCSFLAGS") & BIT13) != 0) {
            checkup();
            return;
        }
        s("RCSFLAGS", g("RCSFLAGS") | BIT13);
        // NOVAC at PRIO27: job to do some initialization and execute 1/ACCS (1/ACCSET)
        new AostaskAndAosjob(agc).oneAccSet();
    }

    /** CHECKUP. */
    public void checkup() {
        chekbits();
    }

    /** Continuation of CHECKUP after CHEKBITS returns: idle unless 1/ACCS done (ACCSOKAY). */
    public void checkupContinue() {
        if ((g("DAPBOOLS") & g("ACCSOKAY")) == 0) { moreIdle(); return; }
        startDap();
    }

    /** STARTDAP: zero the DAP state and install PAXIS as the T5 task. */
    public void startDap() {
        // IBNKCALL ZATTEROR (zero attitude error and desired rates)
        String[] zero = {"TJP", "TJU", "TJV", "OMEGAP", "OMEGAQ", "OMEGAR", "TRAPEDP", "TRAPEDQ", "TRAPEDR",
            "AOSQ", "AOSQ+1", "AOSR", "AOSR+1", "ALPHAQ", "ALPHAR", "NEGUQ", "NEGUR", "AOSQTERM", "AOSRTERM",
            "QACCDOT", "RACCDOT", "ALLOWGTS", "COTROLER", "INGTS", "QGIMTIMR", "RGIMTIMR", "OLDPMIN", "OLDQRMIN",
            "PJETCTR", "UJETCTR", "VJETCTR"};
        for (String z : zero) s(z, 0);
        // CALLGMBL (= BIT5): RCSFLAGS initialization; clear MANFLAG bits
        s("RCSFLAGS", g("RCSFLAGS") & ~MANFLAG & Agc.WORD_MASK);
        s("OLDXFORP", g("CDUX"));                       // DCA CDUX / DXCH OLDXFORP
        s("OLDXFORP+1", g("CDUX+1"));
        s("OLDZFORQ", g("CDUZ"));
        s("RCSFLAGS", g("RCSFLAGS") | BIT12);
        s("SKIPU", 4);
        s("SKIPV", 4);
        s("TIME6", POSMAX);
        s("T6NEXT", POSMAX);
        s("T6FURTHA", POSMAX);
        for (String z : new String[] {"T6NEXT+1", "T6FURTHA+1", "NXT6ADR", "NEXTP", "NEXTU", "NEXTV"}) s(z, 0);
        s("DAPZRUPT", Agc.fromSigned(-10));              // JASK not in progress, initialize negative
        s("NPTRAPS", 2);
        s("NQTRAPS", 2);
        s("NRTRAPS", 2);
        s("T5ADR", paxisAddress());                      // DCA PAXADIDL; DXCH T5ADR
        setTime5();
    }

    private int paxisAddress() { return 1; /* 2CADR PAXIS (EBANK=OMEGAP) */ }

    /** SETTIME5: schedule next T5RUPT in 100 ms, then RESUME. */
    public void setTime5() {
        s("TIME5", MS100);
        // TCF RESUME
    }

    /** MOREIDLE: compute Q,R and P attitude errors then SHUTDOWN. */
    public void moreIdle() {
        // IBNKCALL QERRCALC; IBNKCALL CALCPERR
        shutdown();
    }

    /** SHUTDOWN: reinstall idler, kill jet requests, drop jets and trim gimbal enable. */
    public void shutdown() {
        s("T5ADR", 2);                                   // DCA IDLERADR (2CADR DAPIDLER)
        s("NEXTP", 0);
        s("NEXTU", 0);
        s("NEXTV", 0);
        s("CHAN5", 0);                                   // WRITE CHAN5: jets off
        s("CHAN6", 0);                                   // WRITE CHAN6
        s("CHAN12", g("CHAN12") & ~BGIM23 & Agc.WORD_MASK); // WAND CHAN12 with complement of BGIM23
        setTime5();                                      // return in 100 ms
    }

    /** JUMPDSP: transfer to bank 20 attitude-error display (ALTDSPLY). */
    public void jumpDsp() {
        altDsply();
    }

    /** ALTDSPLY: alternate every 100 ms between needle display and computing/driving. */
    public void altDsply() {
        int flags = g("RCSFLAGS") ^ DSPLYALT;            // invert the alternation bit
        s("RCSFLAGS", flags);
        if ((flags & DSPLYALT) == 0) { needler(); return; }
        if ((g("FLAGWRD0") & g("NEEDLBIT")) != 0) { dsplyTot(); return; }
        // Display DAP attitude errors on the ball: complements of errors go to AK..AK+2
        s("AK+1", ~g("QERROR") & Agc.WORD_MASK);
        s("AK+2", ~g("RERROR") & Agc.WORD_MASK);
        s("AK", ~g("PERROR") & Agc.WORD_MASK);
        retnMore();
    }

    /** DSPLYTOT: total attitude error = (desired - CDU) projected through the M matrix, scaled at pi. */
    public void dsplyTot() {
        int dy = diff(g("CTHETA"), g("CDUY"));           // MSU CDUY
        int dz = diff(g("CPSI"), g("CDUZ"));
        int dx = diff(g("CPHI"), g("CDUX"));
        int q = oversub2(Agc.add(mp(dy, g("M21")), mp(dz, g("M22"))));
        int r = oversub2(Agc.add(mp(dy, g("M31")), mp(dz, g("M32"))));
        int p = oversub2(Agc.add(mp(dy, g("M11")), dx));
        s("AK+1", q);
        s("AK+2", r);
        s("AK", p);
        retnMore();
    }

    /** One's-complement difference (MSU): a - b. */
    private static int diff(int a, int b) {
        return Agc.add(a, (~b) & Agc.WORD_MASK);
    }

    /** MP high-order product of two single-precision fractions. */
    private static int mp(int a, int b) {
        long prod = (long) Agc.toSigned(a) * Agc.toSigned(b);
        return Agc.fromSigned((int) (prod >> 14));
    }

    /** NEEDLER: state machine around enabling the IMU error counters. */
    public void needler() {
        int six = g("RCSFLAGS") & 06;
        if (six == 0) { needles3(); return; }
        if ((six & BIT3) == 0) { needler2(); return; }
        s("CHAN12", g("CHAN12") & ~BIT6 & Agc.WORD_MASK); // disable IMU error counter to zero DACs
        // NEEDLE11: zero inputs and outputs (wait at least 60 ms before enabling counters)
        for (String z : new String[] {"AK", "AK1", "AK2", "EDRIVEX", "EDRIVEY", "EDRIVEZ",
            "CDUXCMD", "CDUYCMD", "CDUZCMD"}) s(z, 0);
        s("RCSFLAGS", (g("RCSFLAGS") & ~06 & Agc.WORD_MASK) + 02);   // reset flags for pass 2
        retnMore();
    }

    /** NEEDLER2: enable IMU error counters (wait 4 ms for relay closure), return to display mode. */
    public void needler2() {
        s("CHAN12", g("CHAN12") | BIT6);
        s("RCSFLAGS", g("RCSFLAGS") & ~06 & Agc.WORD_MASK);
        retnMore();
    }

    /** NEEDLES3: if the error counter is not enabled, re-initialize needler. */
    public void needles3() {
        if ((g("CHAN12") & BIT6) != 0) { needles(); return; }
        s("RCSFLAGS", g("RCSFLAGS") | BIT3);
        retnMore();
    }

    /** NEEDLES: drive the three DACs; inputs rescaled by 0.1, limited to +/-384 counts. */
    public void needles() {
        String[] ak = {"AK", "AK1", "AK2"};
        String[] edrive = {"EDRIVEX", "EDRIVEY", "EDRIVEZ"};
        String[] cmd = {"CDUXCMD", "CDUYCMD", "CDUZCMD"};
        for (int i = 2; i >= 0; i--) {                    // DACLOOP, DINDX = 2,1,0
            int in = Agc.toSigned(g(ak[i]));
            int scaled = (int) (((long) -Agc.toSigned(ONETENTH) * in) >> 14);  // complemented scaled input
            int out = Math.max(-DACLIMIT, Math.min(DACLIMIT, -scaled));        // limit on overflow
            int delta = out - Agc.toSigned(g(edrive[i]));  // new value minus current DAC value
            s(cmd[i], Agc.add(g(cmd[i]), Agc.fromSigned(delta)));
            s(edrive[i], Agc.fromSigned(out));
        }
        s("CHAN14", g("CHAN14") | 070000);                // set DAC activity bits (13,14,15)
        retnMore();
    }

    /** OVERSUB2: return A unchanged, or limited to POSMAX/NEGMAX on overflow. */
    public int oversub2(int a) {
        return a;                                          // 15-bit helper never overflows
    }

    /** RETNMORE: return to CHEKMORE (via DTCB to MORECADR). */
    public void retnMore() {
        chekmore();
    }
}
