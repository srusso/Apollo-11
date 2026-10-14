package apollo.luminary099;

import apollo.agc.Agc;

/**
 * Translation of Q_R-AXIS_RCS_AUTOPILOT.agc (Luminary 1A build 099).
 *
 * LM digital autopilot Q,R-axis (U,V jet axes) RCS control: error calculation, X
 * translation sense, minimum-impulse and rate-command modes, jet selection, the T6
 * jet list (JTLST) and the Q,R to U,V rotation (ROT-TOUV).
 *
 * Conventions: the 15-bit registers are read/written through the shared {@link Agc} by
 * erasable name; two-word quantities use name and name+1. Switch (I/O channel) inputs
 * are read from "CHAN31" etc.; they are inverted in hardware (0 = switch active). The
 * U-axis/V-axis arrays indexed by AXISCTR (0=U,1=V) are modelled as name / name+1.
 */
public class QRAxisRcsAutopilot {
    private static final int BIT1 = 01, BIT2 = 02, BIT5 = 020, BIT6 = 040, BIT7 = 0100,
            BIT8 = 0200, BIT9 = 0400, BIT13 = 010000, BIT15 = 040000;
    private static final int POSMAX = 037777;
    private static final int NEGMAX = 040000;

    // Constants
    public static final int MS60 = 96;                 // 60MS (60 ms in T6 units, "was 96.0")
    public static final int OCT63 = 063;
    public static final int MS14 = 22;                 // 14MS = +TJMINT6
    public static final int TJMINT6 = 22;
    public static final int MINUS_150MS = -240;        // -150MS
    public static final int[] INDEXES = {4, 13};       // axis indices for U,V
    public static final int BIT8_9 = 00600;
    public static final int SCLNORM = 0266;
    public static final int CYCLES40 = 050;            // 40CYCL
    public static final int ONE_TENTH_S = 1;           // 1/10S
    public static final int LINRAT = 46;

    /** Jets: -U, -V, +U, +V. */
    public static final int[] ALLJETS = {00110, 00022, 00204, 00041};
    /** Policy masks: -X, +X, A, B, ALL. */
    public static final int[] TYPEPOLY = {00125, 00252, 00146, 00231, 00377};

    private final Agc agc;

    public QRAxisRcsAutopilot(Agc agc) {
        this.agc = agc;
    }

    // ---- small helpers --------------------------------------------------------------
    private int g(String s) { return agc.get(s); }
    private void s(String sym, int v) { agc.set(sym, v); }
    private static int s16(int w) { return Agc.toSigned(w); }
    private boolean chan31Low(int bit) { return (g("CHAN31") & bit) == 0; }   // inverted bits
    private boolean setIn(String reg, String mask) { return (g(reg) & g(mask)) != 0; }

    /** Mask-and-test for scaled products: (a*b) >> 14, one's-complement truncated. */
    private static int mp(int a, int b) {
        long p = (long) s16(a) * s16(b);
        return Agc.fromSigned((int) (p >> 14));
    }

    // ---- CALLQERR --------------------------------------------------------------------

    /** CALLQERR: compute Q,R errors unless in manual rate command, then pick the control system. */
    public void callqerr() {
        boolean auto = (g("CHAN31") & BIT13) != 0;
        boolean manualRate = (g("DAPBOOLS") & g("OURRCBIT")) != 0;
        if (auto || !manualRate) {
            qerrcalc();                            // else bypass in manual rate command
        }
        qrorgts();
    }

    /** Q,RORGTS: choose control system for this pass (GTS alternates with RCS when docked). */
    public void qrorgts() {
        int c = s16(g("COTROLER"));
        if (c > 0) {
            gotogts();
        } else if (c == 0) {
            // +0 / -0: TRYGTS (GTS if allowed, otherwise RCS)
            trygts();
        } else {
            rcs();
        }
    }

    /** RCS: control via RCS (TRYGTS may branch here); sense translation next. */
    public void rcs() {
        s("COTROLER", 0);
        // Exchange EDOTQ with the rates in U,V space: OMEGAU := rot-to-uv(EDOTQ)
        rotTouv(g("EDOTQ"), g("EDOTQ+1"));
        s("OMEGAU", g("A"));
        s("OMEGAU+1", g("L"));
        senseget();
    }

    // ---- X translation ----------------------------------------------------------------

    /**
     * SENSEGET: determine X-translation sense from bits 7,8 of channel 31 (translation
     * controller), ullage, APS flag and drift bit; sets NEXTU, NEXTV, SENSETYP and
     * ROTINDEX, then proceeds to rotation mode selection.
     */
    public void senseget() {
        if (chan31Low(BIT7)) {                     // +X
            xTranslation(1);
            return;
        }
        if (chan31Low(BIT8)) {                     // -X
            xTranslation(0);
            return;
        }
        if ((g("ULLAGER") & g("DAPBOOLS")) != 0) {   // ullage: +X
            xTranslation(1);
            return;
        }
        s("NEXTU", 0);                             // null translation policies
        s("NEXTV", 0);
        int sense = 0;
        if ((g("DAPBOOLS") & g("DRIFTBIT")) != 0) {   // burning (not drifting)
            if ((g("FLGWRD10") & g("APSFLBIT")) != 0) {
                sense = 2;                         // favour +X jets during APS burn
            }
        }
        s("SENSETYP", sense);
        qrcontrl();
    }

    /** +X0RULGE / -XTRANS: select policy for translation (x=1 for +X, x=0 for -X). */
    private void xTranslation(int x) {
        int rotindex = 4 + x;                      // -XTRANS: AD FOUR
        s("ROTINDEX", rotindex);
        s("SENSETYP", rotindex - 3);               // favour appropriate jets during translation
        int numbert;
        if ((g("DAPBOOLS") & g("ACC4OR2X")) != 0) {
            numbert = 4;                           // TRANS4
        } else if ((g("DAPBOOLS") & g("AORBTRAN")) != 0) {
            numbert = 3;                           // B system: 2 jets
        } else {
            numbert = 2;                           // A system: 2 jets
        }
        s("NUMBERT", numbert);
        selctsub();
        int polytemp = g("POLYTEMP");
        if (s16(polytemp) == 0) {
            new AlarmAndAbort(agc).alarm(02002, 0);   // no policy available because of failures
            polytemp = g("POLYTEMP");
        }
        s("NEXTU", 00314 & polytemp);              // 00314OCT
        s("NEXTV", ~00314 & polytemp & Agc.WORD_MASK);
        qrcontrl();
    }

    // ---- Q,R-axes RCS control mode selection ---------------------------------------------

    /**
     * QRCONTRL: BIT13/CHAN31 set = AUTO (go to ATTSTEER); PULSES = minimum impulse
     * mode; otherwise rate command / attitude hold.
     */
    public void qrcontrl() {
        if ((g("CHAN31") & BIT13) != 0) {          // CCS A on inverted bit: auto
            attsteer();
            return;
        }
        if ((g("PULSES") & g("DAPBOOLS")) == 0) {  // CHKBIT10: attitude hold / rate command
            chekstik();
            return;
        }
        minimumImpulse();
    }

    /** Minimum impulse mode: one hand-controller pulse fires a minimum-impulse jet pair. */
    private void minimumImpulse() {
        new InterBankCommunication(agc).ibnkcall(0, () -> new Zatteror(agc).zatteror());   // ZATTEROR
        s("QERROR", 0);
        s("RERROR", 0);                            // for displays
        int t31 = g("CHAN31");
        s("TEMP31", t31);                          // equals DAPTEMP1
        if (s16(g("OLDQRMIN")) > 0) {              // CHECKIN
            checkin(t31);
            return;
        }
        fireqr(t31);
    }

    /** FIREQR: pick +Q,-Q,+R,-R minimum impulse from channel 31 bits 1,2,5,6. */
    public void fireqr(int t31) {
        if ((t31 & BIT1) == 0) { plusQmin(); return; }
        if ((t31 & BIT2) == 0) { minusQmin(); return; }
        if ((t31 & BIT5) == 0) { plusRmin(); return; }
        if ((t31 & BIT6) == 0) { minusRmin(); return; }
        xtrans();
    }

    /** CHECKIN: record the rotation-switch state (inverted, masked OCT63) in OLDQRMIN. */
    public void checkin(int t31) {
        s("OLDQRMIN", ~t31 & OCT63);
        xtrans();
    }

    public void plusQmin() { s("TJU", MS14); minqr(Agc.fromSigned(-MS14)); }

    public void minusQmin() { s("TJU", Agc.fromSigned(-MS14)); minqr(MS14); }

    public void plusRmin() { minqr(MS14); }

    public void minusRmin() { s("TJU", Agc.fromSigned(-MS14)); minqr(Agc.fromSigned(-MS14)); }

    /** MINQR: store TJV and set up return into MINRTN for both axes. */
    public void minqr(int tjv) {
        s("TJV", tjv);
        s("RETJADR", 1);                           // MINADR -> MINRTN
        s("OLDQRMIN", 1);
        minrtn(1);
    }

    /** MINRTN: per axis (V then U); if docked use 60 ms minimum impulse. */
    public void minrtn(int axis) {
        s("AXISCTR", axis);
        if ((g("DAPBOOLS") & g("CSMDOCKD")) != 0) {
            int tj = s16(g(axis == 0 ? "TJU" : "TJV"));
            s(axis == 0 ? "TJU" : "TJV", Agc.fromSigned(tj > 0 ? MS60 : -MS60));
        }
        s("NUMBERT", ((g("DAPBOOLS") & g("AORBTRAN")) != 0 ? 1 : 0) + 2);
        aftertj();
    }

    // ---- rate command mode -----------------------------------------------------------

    /**
     * CHEKSTIK: rate command / attitude hold. Out of detent goes to RHCACTIV; in detent
     * after manual control goes to DAMPING.
     */
    public void chekstik() {
        s("INGTS", 0);                             // not in GTS when in attitude hold
        s("COTROLER", Agc.fromSigned(-1));         // 1/ACCS will do the nulling drives
        if ((g("CH31TEMP") & BIT15) == 0) {
            rhcactiv();                            // out of detent
            return;
        }
        if ((g("DAPBOOLS") & g("OURRCBIT")) == 0) {
            stillrcs();                            // wasn't under manual control last time
            return;
        }
        s("RCSFLAGS", g("RCSFLAGS") & ~BIT9);
        damping();
    }

    /** DAMPING: zero SAVEHAND then treat as manual command (rate = 0). */
    public void damping() {
        s("SAVEHAND", 0);
        s("SAVEHAND+1", 0);
        rhcactiv();
    }

    /**
     * RHCACTIV: Q,R manual control: WC = A*(B+|D|)*D, where WC is the commanded rate,
     * A the quadratic sensitivity (STIKSENS), B the linear/quadratic factor (LINRAT),
     * |D| the absolute deflection and D the hand controller deflection.
     */
    public void rhcactiv() {
        int d = s16(g("SAVEHAND"));
        int qlast = manualRate(d);
        s("DAPTEMP3", Agc.fromSigned(qlast - s16(g("QLAST")) ));
        s("QLAST", Agc.fromSigned(qlast));
        int d2 = s16(g("SAVEHAND+1"));
        int rlast = manualRate(d2);
        s("DAPTEMP4", Agc.fromSigned(rlast - s16(g("RLAST"))));
        s("RLAST", Agc.fromSigned(rlast));
        s("QRATEDIF", Agc.fromSigned(s16(g("OMEGAQ")) - qlast));
        s("RRATEDIF", Agc.fromSigned(s16(g("OMEGAR")) - rlast));
        enterqr();
    }

    private int manualRate(int deflection) {
        int mag = Math.abs(deflection) * 4 + LINRAT;           // DOUBLE DOUBLE; AD LINRAT
        int prod = s16(mp(Agc.fromSigned(mag), Agc.fromSigned(deflection)));
        return s16(mp(Agc.fromSigned(prod), g("STIKSENS")));    // scaled 45 deg/sec
    }

    /** ENTERQR: transform rate differences to U,V axes; check breakout level. */
    public void enterqr() {
        rotTouv(g("QRATEDIF"), g("RRATEDIF"));
        s("URATEDIF", g("A"));
        s("VRATEDIF", g("L"));
        boolean qExceeds = Math.abs(s16(g("DAPTEMP3"))) + s16(g("-RATEDB")) > 0;
        boolean rExceeds = Math.abs(s16(g("DAPTEMP4"))) + s16(g("-RATEDB")) > 0;
        if (qExceeds || rExceeds) {
            enteruv();                              // breakout level exceeded: direct rate
            return;
        }
        if ((g("RCSFLAGS") & g("QRBIT")) != 0) {
            enteruv();                              // continue direct rate control
            return;
        }
        stillrcs();                                 // pseudo-auto control
    }

    /** ENTERUV: direct rate control; fire jets proportional to rate difference. */
    public void enteruv() {
        s("TCQR", CYCLES40);
        new InterBankCommunication(agc).ibnkcall(0, () -> new Zatteror(agc).zatteror());
        s("DYERROR", 0); s("DYERROR+1", 0);
        s("DZERROR", 0); s("DZERROR+1", 0);
        boolean uOut = Math.abs(s16(g("URATEDIF"))) > s16(g("TARGETDB"));
        boolean vOut = Math.abs(s16(g("VRATEDIF"))) > s16(g("TARGETDB"));
        if (!uOut) {
            if (!vOut) {                            // within the target deadband: pseudo-auto next pass
                s("URATEDIF", 0);
                topseudo();
                return;
            }
        } else if (!vOut) {
            s("VRATEDIF", 0);
        }
        qrtime();
    }

    /** QRTIME: direct rate time check (4 s). */
    public void qrtime() {
        if (s16(g("TCQR")) > 0) {
            s("RCSFLAGS", g("RCSFLAGS") | g("QRBIT"));     // bit 11 = 1
            s("TCQR", g("TCQR"));
            backhand(1);
        } else {
            topseudo();
        }
    }

    /** TOPSEUDO: switch to pseudo-auto control on the next pass. */
    public void topseudo() {
        s("RCSFLAGS", g("RCSFLAGS") & ~g("QRBIT"));         // bit 11 = 0
        backhand(1);
    }

    /**
     * BACKHAND: per axis (V=1, then U=0) compute jet time from rate difference:
     * TJ = RATEDIF / JETACC (with AOS), scaled 10.67 s, limited to 150 ms.
     */
    public void backhand(int axis) {
        for (int ax = axis; ax >= 0; ax--) {
            s("AXISCTR", ax);
            s("NUMBERT", 4);
            if (g(ax == 0 ? "SKIPU" : "SKIPV") != 0 && skipEnter(ax)) {
                continue;
            }
            int rateDif = s16(g(ax == 0 ? "URATEDIF" : "VRATEDIF"));
            int sel = (rateDif > 0 ? 1 : 0) + g("AXISDIFF");
            int inv = s16(g("1/ANET2+" + (sel + 1))) * -1;          // CS 1/ANET2 +1,A
            int tj = s16(mp(Agc.fromSigned(inv), Agc.fromSigned(rateDif))) * 2;
            if (Math.abs(tj) > POSMAX) {
                tj = tj > 0 ? POSMAX : -POSMAX;                      // overflow: limit
            }
            s(ax == 0 ? "TJU" : "TJV", Agc.fromSigned(tj));
            aftertj();
        }
    }

    private boolean skipEnter(int ax) {
        // INDEX SKIPU: if nonzero, count down skip (set to 4) and go to LOOPER
        s(ax == 0 ? "SKIPU" : "SKIPV", 4);
        looper();
        return true;
    }

    // ---- GTS ---------------------------------------------------------------------------

    /**
     * TRYGTS: GTS is tried if USEQRJTS = 0, ALLOWGTS positive and the jets are off
     * (channel 5 empty).
     */
    public void trygts() {
        if ((g("USEQRJTS") & g("DAPBOOLS")) != 0) { rcs(); return; }
        if (s16(g("ALLOWGTS")) <= 0) { rcs(); return; }
        if (g("CHAN5") != 0) { chkingts(); return; }
        gotogts();
    }

    /** GOTOGTS: transfer to the gimbal trim system autopilot (GTS). */
    public void gotogts() {
        new InterBankCommunication(agc).bankjump(0, () -> new Gts(agc).gts());
    }

    /** CHKINGTS: if GTS was controlling, set up a damped nulling drive then use RCS. */
    public void chkingts() {
        if (s16(g("INGTS")) > 0) {
            new InterBankCommunication(agc).ibnkcall(0, () -> new Gts(agc).timegmbl());
            s("INGTS", 0);
        }
        rcs();
    }

    // ---- error calculation ----------------------------------------------------------------

    /**
     * QERRCALC: Q,R-axis attitude errors from CDU minus desired CDU angles, rotated by
     * M21..M32 (scaled pi radians), plus KALCMANU interface errors; results to QERROR
     * and RERROR (used by the displays, RCS and GTS control laws).
     */
    public void qerrcalc() {
        int dy = Agc.fromSigned(s16(g("CDUY")) - s16(g("CDUYD")));
        s("DAPTEMP1", dy);
        int e = mp(dy, g("M21"));
        int dz = Agc.fromSigned(s16(g("CDUZ")) - s16(g("CDUZD")));
        s("DAPTEMP2", dz);
        int q = Agc.add(Agc.add(mp(dz, g("M22")), g("DELQEROR")), e);
        s("QERROR", q);
        rerrcalc();
    }

    /** RERRCALC: R-error calculation. */
    public void rerrcalc() {
        int e = mp(g("DAPTEMP1"), g("M31"));
        int r = Agc.add(Agc.add(mp(g("DAPTEMP2"), g("M32")), g("DELREROR")), e);
        s("RERROR", r);
    }

    // ---- attitude steering --------------------------------------------------------------------

    /** ATTSTEER == STILLRCS: entry for U,V attitude control using RCS. */
    public void attsteer() { stillrcs(); }

    /** STILLRCS: transform (QERROR,RERROR) to UERROR,VERROR then run TJLAW per axis. */
    public void stillrcs() {
        rotTouv(g("QERROR"), g("RERROR"));
        s("UERROR", g("A"));
        s("UERROR+1", g("L"));
        tjlaw();
    }

    /** TJLAW: call TJETLAW (or SPSRCS when docked) for V then U, with skip logic. */
    public void tjlaw() {
        s("RETJADR", 3);                           // TJLAW +3
        for (int ax = 1; ax >= 0; ax--) {
            s("AXISCTR", ax);
            if (g(ax == 0 ? "SKIPU" : "SKIPV") != 0) {          // skip this pass
                s(ax == 0 ? "SKIPU" : "SKIPV", 4);
                continue;
            }
            s("E", g(ax == 0 ? "UERROR" : "UERROR+1"));
            s("EDOT", g(ax == 0 ? "OMEGAU" : "OMEGAU+1"));
            if ((g("DAPBOOLS") & g("CSMDOCKD")) == 0) {
                new TjetLaw(agc).tjetlaw();                       // jet-on time into TJU/TJV
                aftertj();
            } else {
                // docked: if gimbal usable do GTS control on the next pass
                int usable = ~g("DAPBOOLS") & g("USEQRJTS");
                if (usable != 0) {
                    s("COTROLER", usable);
                }
                new InterBankCommunication(agc).ibnkcall(0, () -> new SpsRcs(agc).spsrcs());
                s("NUMBERT", 4);                   // always 2-jet control about U,V
                aftertj();
            }
        }
        closeout();
    }

    // ---- jet selection ------------------------------------------------------------------------------

    /**
     * AFTERTJ: Q,R jet selection logic. Inputs AXISCTR (0,1 for U,V), SNUFFBIT, TJU/TJV
     * (scaled 10.24 s), NUMBERT, RETJADR. Outputs NO.U(V)JETS, channel 5, SKIPU/SKIPV.
     * Times under 14 ms are minimum impulse; over 150 ms use the jet list.
     */
    public void aftertj() {
        boolean snuff = (g("FLAGWRD5") & g("SNUFFBIT")) != 0;
        boolean aps = (g("FLGWRD10") & g("APSFLBIT")) != 0;
        boolean drift = (g("DAPBOOLS") & g("DRIFTBIT")) != 0;
        if (snuff && !aps && !drift) {
            xtrans();                              // inhibit control during DPS burn
            return;
        }
        dorotat();
    }

    /** DOROTAT: rotation about the current axis. */
    public void dorotat() {
        int ax = g("AXISCTR");
        int tj = s16(g(ax == 0 ? "TJU" : "TJV"));
        if (tj == 0) { norotat(); return; }
        int l = tj > 0 ? 2 : 0;
        int abstj = Math.abs(tj) + 1;
        s("ABSTJ", abstj);
        s("ROTINDEX", ax + l);                     // 0 1 2 3 = -U -V +U +V
        if (abstj + MINUS_150MS <= 0) {
            doskip(tj);
            return;
        }
        selctsub();
        int index = INDEXES[ax];
        writep(index, g("POLYTEMP"));
        feedback();
    }

    /** NOROTAT: no rotation; turn off the axis' jets (write zero policy). */
    public void norotat() {
        int ax = g("AXISCTR");
        writep(INDEXES[ax], 0);
        looper();
    }

    /** LOOPER: next axis, or CLOSEOUT when both have been processed. */
    public void looper() {
        int ax = g("AXISCTR");
        if (ax > 0) {
            s("AXISCTR", ax - 1);
            dorotat();
        } else {
            closeout();
        }
    }

    /** DOSKIP: less than 150 ms firing; minimum-impulse / jet list handling. */
    public void doskip(int tj) {
        int abstj = g("ABSTJ");
        if (TJMINT6 - abstj <= 0) {                // longer than 14 ms: normal timed firing
            notmin();
            return;
        }
        int ax = g("AXISCTR");
        s(ax == 0 ? "TJU" : "TJV", Agc.fromSigned(tj > 0 ? TJMINT6 : -TJMINT6));
        if (s16(g("SENSETYP")) > 0) {              // ensure min-impulse not against trans
            s("NUMBERT", g("LOSCALAR") & 1);       // semi-random jet choice
        }
        notmin();
    }

    /** NOTMIN: select jets, write policy for now, place the turn-off in the jet list. */
    public void notmin() {
        selctsub();
        int ax = g("AXISCTR");
        int index = INDEXES[ax];
        s("T6FURTHA+1", index);
        writep(index, g("POLYTEMP"));
        s("T6FURTHA", g("ABSTJ"));
        jtlst();
        s(ax == 0 ? "SKIPU" : "SKIPV", 0);
        feedback();
    }

    /** FEEDBACK: NO.UJETS = 1 or 2 for the rate derivation. */
    public void feedback() {
        int ax = g("AXISCTR");
        s(ax == 0 ? "NO.UJETS" : "NO.VJETS", g("NUMBERT") - 3 <= 0 ? 1 : 2);
        looper();
    }

    /** XTRANS: inhibit control: zero TJU,TJV, clear any pending skip and write translation only. */
    public void xtrans() {
        s("TJU", 0);
        s("TJV", 0);
        int skipU = g("SKIPU");
        s("SKIPU", 4);
        if (skipU != 0) {
            writep(INDEXES[0], g("NEXTU"));        // WRITEU -1
        }
        int skipV = g("SKIPV");
        s("SKIPV", 4);
        if (skipV != 0) {
            writep(INDEXES[1], g("NEXTV"));        // WRITEV -1
        }
        closeout();
    }

    /** WRITEP: write the jet policy for the axis (index 4=U, 13=V) to channel 5 (merged). */
    private void writep(int index, int policy) {
        int mask = index == 4 ? 00314 : 00063;     // jets controlled by this axis' channel bits
        int ch5 = (g("CHAN5") & ~mask) | (policy & mask);
        s("CHAN5", ch5);
    }

    // ---- JTLST ---------------------------------------------------------------------------------------

    /**
     * JTLST: waitlist for T6RUPTs, exactly three entries long. Input is the last member
     * (T6FURTHA = delta time, T6FURTHA+1 = axis index). T6NEXT is the delta time to
     * the next rupt, TIME6 the time of the next rupt, NXT6ADR the axis index (0 P, 4 U, 13 V).
     */
    public void jtlst() {
        int a = Agc.add(~g("T6FURTHA") & Agc.WORD_MASK, g("TIME6"));
        if (s16(a) <= 0) {                         // MIDORLST: -T6FURTHA + TIME6 <= 0
            midorlst(a);
            return;
        }
        int lnext = g("NXT6ADR");
        s("NXT6ADR", g("T6NEXT+1") == 0 ? lnext : lnext);
        // rotate the list: next <- ... ; furtha <- old next
        int oldT6next = g("T6NEXT");
        s("T6NEXT", g("T6FURTHA"));
        s("T6NEXT+1", g("T6FURTHA+1"));
        s("T6FURTHA", oldT6next);
        s("TIME6", g("T6NEXT"));
        turnon();
    }

    /** TURNON: enable T6 (bit 15 of channel 13). */
    public void turnon() {
        s("CHAN13", g("CHAN13") | BIT15);
    }

    /** MIDORLST: insert as middle or last element of the jet list. */
    public void midorlst(int a) {
        int b = Agc.add(a, g("T6NEXT"));
        if (s16(b) <= 0) {                         // LASTCHG
            s("T6FURTHA", Agc.add(~b & Agc.WORD_MASK, Agc.WORD_MASK));
            return;
        }
        s("T6NEXT+1", g("T6NEXT+1"));
        int furtha = g("T6FURTHA");
        s("T6NEXT", Agc.fromSigned(s16(furtha) - s16(g("TIME6"))));
        s("T6FURTHA", furtha);
    }

    // ---- ROT-TOUV ---------------------------------------------------------------------------------------

    /**
     * ROT-TOUV: transform the Q and R components (arguments, nominally in A and L)
     * into the non-orthogonal U,V system, in which RCS jet firings do not cross-couple.
     * U = Q*COEFFQ + R*COEFFR, V = -Q*COEFFQ + R*COEFFR, with overflow limited to
     * POSMAX/NEGMAX. Results are left in A (U) and L (V).
     */
    public void rotTouv(int q, int r) {
        int cq = mp(q, g("COEFFQ"));
        s("ROTEMP2", cq);
        int cr = mp(r, g("COEFFR"));
        int u = s16(cq) + s16(cr);
        int v = -s16(cq) + s16(cr);
        u = Math.max(-POSMAX, Math.min(POSMAX, u));        // overflow limited
        v = Math.max(-POSMAX, Math.min(POSMAX, v));
        s("ROTEMP1", Agc.fromSigned(u));
        s("A", Agc.fromSigned(u));
        s("L", Agc.fromSigned(v));
    }

    /** SELCTSUB: choose jets for ROTINDEX/NUMBERT, falling back through failures. */
    public void selctsub() {
        int rot = g("ROTINDEX");
        int num = g("NUMBERT");
        int poly = ALLJETS[rot & 3] & TYPEPOLY[Math.min(num, 4)];
        s("POLYTEMP", poly);
        if ((poly & g("CH5MASK")) == 0) {
            return;                               // none failed
        }
        num = 3;
        for (;;) {                                // FAILOOP
            s("NUMBERT", num);
            poly = ALLJETS[rot & 3] & TYPEPOLY[num];
            s("POLYTEMP", poly);
            if ((poly & g("CH5MASK")) != 0) {
                // CCS NUMBERT; try the next lower policy
                if (num > 0) { num--; continue; }
                s(g("AXISCTR") == 0 ? "TJU" : "TJV", 0);
                new AlarmAndAbort(agc).alarm(02004, 0);
                norotat();
                return;
            }
            return;
        }
    }

    // ---- CLOSEOUT -----------------------------------------------------------------------------------------------

    /**
     * CLOSEOUT: set the interrupt flip-flop as soon as possible, permitting a return to
     * the interrupted job (ENDJASK restores the DAP rupt registers).
     */
    public void closeout() {
        makerupt(this::endjask);
    }

    /** MAKERUPT (fixed-fixed): EDRUPT to force a rupt to the given continuation. */
    public void makerupt(Runnable target) {
        target.run();
    }

    /** ENDJASK: restore registers and mark the job as finished (NEGMAX in DAPZRUPT). */
    public void endjask() {
        int a = g("DAPARUPT"), l = g("DAPARUPT+1");
        s("DAPARUPT", g("ARUPT")); s("DAPARUPT+1", g("ARUPT+1"));
        s("ARUPT", a); s("ARUPT+1", l);
        int b = g("DAPBQRPT");
        s("DAPBQRPT", g("BRUPT"));
        s("BRUPT", b);
        int q = g("DAPBQRPT+1");
        s("DAPBQRPT+1", g("Q"));
        s("Q", q);
        int z = g("DAPZRUPT");
        s("DAPZRUPT", NEGMAX);                     // negative DAPZRUPT signals JASK is over
        s("DAPZRUPT+1", g("DAPZRUPT+1"));
        s("ZRUPT", z);
        new Waitlist(agc).resume();                  // NOQRSM (WAITLIST.agc): restore and RESUME
    }

    // ---- minimum-impulse helper kept for completeness ---------------------------------------------------------------------

    /** TRANS4: four-jet translation policy. */
    public void trans4() {
        s("NUMBERT", 4);
        selctsub();
    }

    // Referenced collaborators (defined in sibling translations)
    private static final class Zatteror {
        private final Agc agc;
        Zatteror(Agc agc) { this.agc = agc; }
        void zatteror() { new DapInterfaceSubroutines(agc).zatteror(); }
    }
    private static final class Gts {
        private final Agc agc;
        Gts(Agc agc) { this.agc = agc; }
        void gts() { new TrimGimbalControlSystem(agc).gts(); }
        void timegmbl() { new TrimGimbalControlSystem(agc).timegmbl(); }
    }
    private static final class TjetLaw {
        private final Agc agc;
        TjetLaw(Agc agc) { this.agc = agc; }
        void tjetlaw() { new apollo.luminary099.TjetLaw(agc).tjetlaw(); }
    }
    private static final class SpsRcs {
        private final Agc agc;
        SpsRcs(Agc agc) { this.agc = agc; }
        void spsrcs() { new SpsBackUpRcsControl(agc).spsrcs(); }
    }
}
