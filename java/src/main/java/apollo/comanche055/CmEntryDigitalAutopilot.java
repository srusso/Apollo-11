package apollo.comanche055;

import apollo.agc.Agc;

/**
 * CM_ENTRY_DIGITAL_AUTOPILOT.agc (Colossus 2A / Comanche 055).
 *
 * The CM entry DAP: READGYMB (waitlist task reading CDU gimbal angles every 0.1 s),
 * BODYRATE (gimbal-angle differences to body rates), ATTRATES (T5 task: attitude integration,
 * alpha/beta/roll, extra-atmospheric and in-atmosphere jet logic), CM/RCS (2 s roll jet
 * switching logic with phase-plane), JETCALL (waitlist roll-jet timing), CM/FDAI (needles)
 * and CM/DUMPR (telemetry file of body rates). CM/DAPON and CM/DAPIC initialise the DAP.
 *
 * Numeric convention: angles are held as signed fractions scaled so that 16384 = 1.0
 * (a half revolution, "degrees/180"); {@code mp} models the AGC MP instruction (high word of
 * the product); erasable cells are accessed via {@link Agc} with signed values.
 */
public class CmEntryDigitalAutopilot {
    private static final int ONE_F = 16384;          // 1.0 in 15-bit single precision (B-14)
    private static final int HALF = 8192;
    private static final int POS_MAX = 037777;
    private static final int BIT1 = 1, BIT2 = 2, BIT3 = 4, BIT4 = 8, BIT6 = 040, BIT11 = 02000, BIT12 = 04000;

    // ---- constants (end of file) ----
    public static final int NSEC = 2100;             // 65 deg / 3 deg/s
    public static final int D4_S_LIM = 16348;        // 1.0 - 4/180 deg/s
    public static final int YDOTLIM = 16366;         // 1.0 - 2 deg/s dead zone
    public static final int CM_BIAS = 55;            // .6/180 B14
    public static final int YAWLIM = 16055;          // 1.0 - 3.6/180
    public static final int C45LIM = 4799;           // 1.0 - cos(45) = .29289
    public static final int SINTRIM = -5603;         // sin(-20) = -.34202
    public static final int COSTRIM = 15395;         // cos(-20) = .93969
    /** YJETCODE: 0, +Y (00120), -Y (00240) (indexed by tag 0, +1, -1 -> 0,1,2). */
    public static final int[] YJETCODE = {000120, 000000, 000240};
    /** P/RJCODE: 0 ; positive 00005 ; negative 00012. Indexed by [tag]. */
    public static final int[] PRJCODE = {000005, 000000, 000012};
    public static final int MINUS_T_3 = -203;
    public static final double VSQMIN = 0.61050061E-3;
    public static final double D180_8ATT = 0.61813187;
    public static final int TWO_JETT = 400;          // 4SECS: 2*2*100 cs
    public static final int FOUR_JETT = 800;
    public static final int XMIN_360 = 182;
    public static final double MINUS_VM_360K = -0.22222222;
    public static final int XS_360 = 91;
    public static final int BUFLIM = XS_360;
    public static final double KTRCS = 0.5;
    public static final int TWO_T_TCDU = 050;

    private final Agc agc;

    public CmEntryDigitalAutopilot(Agc agc) {
        this.agc = agc;
    }

    /** Hook for external routines (WAITLIST, PHASCHNG, TASKOVER, POSTJUMP, DELAYJOB, NEEDLER, ...). */
    protected void extern(String routine, Object... args) {
    }

    // ---- helpers ----
    private int g(String s) {
        return Agc.toSigned(agc.get(s));
    }

    private void s(String sym, int v) {
        agc.set(sym, Agc.fromSigned(v));
    }

    /** MP: high word of product of two B-14 fractions. */
    private static int mp(int a, int b) {
        return (int) (((long) a * b) / ONE_F);
    }

    private static int spsin(int a) {
        return (int) Math.round(Math.sin(Math.PI * a / ONE_F) * ONE_F);
    }

    private static int spcos(int a) {
        return (int) Math.round(Math.cos(Math.PI * a / ONE_F) * ONE_F);
    }

    /** ANGOVCOR: correct an angle sum (degrees/180) overflowing the 15-bit word by +-360 deg. */
    private static int angovcor(int sum) {
        if (sum > 16383) {
            return sum - 32768;
        }
        if (sum < -16383) {
            return sum + 32768;
        }
        return sum;
    }

    /** FLUSHJET: zero channels 6 (ROLLJETS) and 5 (PYJETS). */
    public void flushjet() {
        agc.set("ROLLJETS", 0);
        agc.set("PYJETS", 0);
    }

    // ===================================================================================
    // READGYMB: CDU read task (every 0.1 s)
    // ===================================================================================

    /** READGYMB: keep restart DT going relative to PIPTIME (group 6); read CDUs when in fine align. */
    public void readgymb() {
        agc.set("CM/GYMDT", agc.get("CM/GYMDT") + 10);
        boolean fineAlign = (agc.get("IMODES33") & BIT6) == 0;
        if (!fineAlign) {
            // not in fine align: GYMDIFSW = 0, quench jets
            agc.set("CM/FLAGS", agc.get("CM/FLAGS") & ~BIT1);
            flushjet();
            cmGymic(0);
            return;
        }
        readgym1();
    }

    /** READGYM1: -DELAOG = AOG(N-1) - AOG(N), likewise for AIG and AMG. */
    private void readgym1() {
        int cduX = g("CDUX");
        s("-DELAOG", angovcor0(g("AOG") - cduX)); s("AOG", cduX);
        int cduY = g("CDUY");
        s("-DELAIG", angovcor0(g("AIG") - cduY)); s("AIG", cduY);
        int cduZ = g("CDUZ");
        s("-DELAMG", angovcor0(g("AMG") - cduZ)); s("AMG", cduZ);
        dobrateTest();
    }

    private static int angovcor0(int v) {
        return angovcor(v);
    }

    /** DOBRATE?: CM/FLAGS bits (GYMDIFSW bit1, CM/DSTBY bit2) select path. */
    private void dobrateTest() {
        int flags = ~agc.get("CM/FLAGS") & 3;      // CS CM/FLAGS ; MASK THREE
        switch (flags) {
            case 0:                                 // both set: normal
                dobrate();
                break;
            case 1:                                 // GYMDIFSW=0, DSTBY=1: first pass
                cmGymic(1);
                break;
            case 2:                                 // NOOP path: standby
            default:
                flushjet();
                extern("PHASCHNG", 00006);           // deactivate DAP group 6
                extern("TASKOVER");
                break;
        }
    }

    /** DOBRATE: JETEM = 1 (do BODYRATE); DOBRATE1: reschedule READGYMB in 0.1 s then BODYRATE. */
    public void dobrate() {
        dobrate1(1);
    }

    private void dobrate1(int jetem) {
        agc.set("JETEM", jetem);
        extern("WAITLIST", 10, "READGYMB");
        if (jetem > 0) {
            bodyrate();
        } else {
            extern("TASKOVER");                      // skip calc on initial pass
        }
    }

    /** CM/GYMIC: GYMDIFSW set (C(A)=1); zero JETAG, OLDELP/Q/R, GAMDOT; skip BODYRATE once. */
    public void cmGymic(int a) {
        agc.set("CM/FLAGS", agc.get("CM/FLAGS") + a);
        agc.set("JETAG", 0);
        agc.set("OLDELP", 0);
        agc.set("OLDELQ", 0);
        agc.set("OLDELR", 0);
        agc.set("GAMDOT", 0);
        dobrate1(0);
    }

    /** RATEAVG: estimate rate with constant acceleration: DELV(est) = DELV + (DELV - OLDELV)/2. */
    private int rateavg(int old) {
        int j = g("JETEM");
        return mp(-old + j, HALF) + j;
    }

    // ===================================================================================
    // CM/DAPON, CM/DAPIC: initialisation
    // ===================================================================================

    /** CM/DAPON: reset DAPBITs, idle T5/T6, set up attitude hold at ALFA/180, ROLL/180, then P62.1. */
    public void cmDapon() {
        agc.set("DAPBIT1", 0);
        agc.set("DAPBIT2", 0);
        agc.set("T5LOC", agc.get("T5IDLOC"));
        agc.set("T6LOC", agc.get("T5IDLOC"));
        flushjet();
        agc.set("DAPDATR1", agc.get("DAPDATR1") & ~070000 & Agc.WORD_MASK);   // config bits = 0 for entry
        while ((agc.get("CM/FLAGS") & BIT11) == 0) {            // NOTYET: wait for GAMDIFSW
            extern("DELAYJOB", 50);                              // .5 SEC
        }
        s("RCSFLAGS", -1);                                       // activate CM/DAP
        s("P63FLAG", -1);
        agc.set("JETAG", 0);
        agc.set("PAXERR1", 0);
        agc.set("ALFACOM", agc.get("ALFA/180"));                 // attitude hold until keyboard HEADSUP
        agc.set("ROLLHOLD", agc.get("ROLL/180"));
        s("ROLLC", mp(g("ROLL/180"), HALF));
        agc.set("CM/FLAGS", agc.get("CM/FLAGS") | BIT12);        // CMDAPARM = 1
        agc.set("FLAGWRD2", agc.get("FLAGWRD2") | BIT1);         // NODOFLAG: no further V37
        extern("POSTJUMP", "P62.1");
    }

    /** CM/DAPIC: initial conditions; schedule READGYMB, init flags, 2PHSCHNG, go to P62.2. */
    public void cmDapic() {
        int dt = -g("PIPTIME+1") + 2 * 16384 + g("TIME1");          // delta time since PIPUP (cs)
        int jetem = dt;
        while (jetem - 5 > 0) {
            jetem -= 203;                                           // AD -CDUT+1 loop
        }
        agc.set("CM/GYMDT", jetem + 1);
        extern("WAITLIST", jetem + 1, "READGYMB");
        agc.set("CM/FLAGS", (~016017 & agc.get("CM/FLAGS")) + 012);  // CM/SWIC1 / CM/SWIC2
        agc.set("BETA/180", 0);
        agc.set("SW/NDX", 1);
        extern("2PHSCHNG", 040116, 005024, 013000);
        extern("POSTJUMP", "P62.2");
    }

    // ===================================================================================
    // BODYRATE
    // ===================================================================================

    /** BODYRATE: Euler-angle differences to body rates P, Q, R (PREL, QREL, RREL) per 0.1 s. */
    public void bodyrate() {
        int cosm = spcos(g("AMG"));
        int sino = spsin(g("AOG"));
        int sinocosm = mp(sino, cosm);
        int coso = spcos(g("AOG"));
        int cosocosm = mp(coso, cosm);
        // QREL
        long q = (long) -g("-DELAMG") * sino + (long) -g("-DELAIG") * cosocosm;
        s("JETEM", (int) (q / ONE_F));
        int oldQ = g("OLDELQ");
        s("OLDELQ", g("JETEM"));
        s("QREL", rateavg(oldQ));
        // RREL
        long r = (long) -g("-DELAMG") * coso + (long) -g("-DELAIG") * sinocosm;
        s("JETEM", (int) (r / ONE_F));
        int oldR = g("OLDELR");
        s("OLDELR", g("JETEM"));
        s("RREL", rateavg(oldR));
        // PREL
        int sinm = spsin(g("AMG"));
        int t = mp(sinm, -g("-DELAIG")) + g("-DELAOG");
        s("JETEM", -t);
        int oldP = g("OLDELP");
        s("OLDELP", g("JETEM"));
        s("PREL", rateavg(oldP));
        // GAMDOT compensation (if >= 0.5 deg/s)
        if (g("GAMDOT") > 0 || (agc.get("GAMDOT") & Agc.SIGN_BIT) != 0) {
            int sr = spsin(g("ROLL/180"));
            int gd = g("GAMDOT");
            int srg = mp(sr, gd);
            s("JETEM+1", srg);
            s("PREL", g("PREL") + mp(srg, SINTRIM));
            s("QREL", g("QREL") + mp(-spcos(g("ROLL/180")), gd));
            s("RREL", g("RREL") + mp(-srg, COSTRIM));
        }
        nogamdut();
    }

    /** NOGAMDUT: if the DAP is armed (CMDAPARM), schedule ATTRATES via T5 in ~10 ms. */
    private void nogamdut() {
        if ((agc.get("CM/FLAGS") & BIT12) == 0) {
            extern("TASKOVER");                      // STBYDUMP: DAP not armed
            return;
        }
        agc.set("TIME5", POS_MAX);
        agc.set("T5LOC", agc.get("ATTRATES"));        // ATDOTCAD
        extern("TASKOVER");
    }

    // ===================================================================================
    // ATTRATES (T5 task)
    // ===================================================================================

    /** ATTRATES: integrate ALFA, BETA, ROLL; decide in-atmosphere or exo-atmospheric jet logic. */
    public void attrates() {
        agc.set("CM/SAVE", (agc.get("SR") * 2) & Agc.WORD_MASK);
        int alfa = angovcor(g("QREL") + g("ALFA/180"));
        s("ALFA/180", alfa);
        int calfa = spcos(alfa);
        s("CALFA", calfa);
        int phidot = mp(calfa, g("PREL"));
        s("BETADOT", mp(calfa, g("RREL")));
        // ordering as in the source: PHIDOT := CA PREL ; BETADOT := CA RREL
        int salfa = spsin(alfa);
        s("SALFA", salfa);
        phidot += mp(salfa, g("RREL"));                   // CB PHIDOT, saved
        s("PHIDOT", phidot);
        int betadot = g("BETADOT") + mp(-salfa, g("PREL"));
        s("BETADOT", betadot);
        int beta = g("BETA/180") + betadot;
        s("BETA/180", beta);
        int cb = spcos(beta);                              // SPSIN of BETA used per source: SIN(beta)
        int sb = spsin(beta);
        alfa = angovcor(alfa + mp(sb, phidot));            // neglect CB in CB PHIDOT
        s("ALFA/180", alfa);
        int ak1 = angovcor(-alfa + g("ALFACOM"));
        s("AK1", ak1);
        s("QAXERR", ak1);
        int roll = angovcor(phidot + g("ROLL/180"));
        s("ROLLTM", roll);
        s("ROLL/180", roll);
        s("RAXERR", -beta + g("BETACOM"));                 // temporary for TM (BETA error)
        if ((agc.get("CM/FLAGS") & BIT3) == 0) {           // .05GSW = 0: less than .05 g
            exdap();
            return;
        }
        // in atmosphere
        s("CMDAPMOD", -1);
        s("AK1", -1);
        s("AK2", -1);
        int yawErr = mp(-g("PREL"), SINTRIM) + g("RREL");
        int tag = dz2dsz(yawErr);
        int jetem = YJETCODE[tagIndex(tag)];
        tag = dz2dsz(g("QREL"));
        exdapin(jetem, tag);
    }

    private static int tagIndex(int tag) {
        return tag == 0 ? 1 : tag > 0 ? 0 : 2;
    }

    /** 2D/SDZ: test whether A (rate) is within 2 deg/s dead zone; returns 0 inside else +-1. */
    private int dz2dsz(int a) {
        int mag = Math.abs(a);
        return (YDOTLIM - 16384 + mag) > 0 ? (a > 0 ? 1 : -1) : 0;
    }

    /** EXDAPIN: combine pitch/roll code with yaw code, write PYJETS, then CM/RCS or FDAI path. */
    private void exdapin(int jetem, int tag) {
        jetem += PRJCODE[tagIndex(tag)];
        s("JETEM", jetem);
        agc.set("PYJETS", jetem);
        int jetag = g("JETAG");
        if (jetag > 0) {
            cmRcs();
        } else if (jetag == 0) {
            cmFdai();
        } else {
            cmFdair();                                    // JETAG=-1 equivalent to CMDAPMOD=+1
        }
    }

    /**
     * BIASEDZ: biased dead zone for extra-atmospheric DAP. rate/180 in A, error/180 in L;
     * returns the error bit tag (0 inside dead zone, +-1 outside). If rate exceeds 4 deg/s the
     * error bit is overridden by the sign of the rate.
     */
    private int biasedz(int rate, int err) {
        int biased = err + (err > 0 ? -CM_BIAS : CM_BIAS);
        int tag = dz3d(biased);
        if (Math.abs(rate) >= 16384 - D4_S_LIM) {            // rate >= 4 deg/s: sign of rate rules
            return rate > 0 ? 1 : -1;
        }
        return tag != 0 ? tag : dz2dsz(rate);
    }

    /** 3DDZ: error dead zone (YAWLIM = 3.6 deg). */
    private int dz3d(int err) {
        return Math.abs(err) > 16384 - YAWLIM ? (err > 0 ? 1 : -1) : 0;
    }

    /** EXDAP: extra-atmospheric (G < .05) DAP. */
    public void exdap() {
        s("CMDAPMOD", 0);
        int calfa = g("CALFA");
        if (Math.abs(calfa) >= 16384 - C45LIM) {              // |CALFA| > .707 branch
            if (calfa > 0) {
                int p63 = g("P63FLAG");
                if (p63 <= 0 && p63 != 0 ? false : p63 == 0) {
                    // single pass: request P62 termination in NSEC
                    extern("PHASCHNG", 040334);
                    s("P63FLAG", -1);
                    extern("WAITLIST", NSEC, "WAKEP62");
                }
            }
            exdap4();
        } else {
            exdap2();
        }
    }

    /** EXDAP4: roll jet interface test between the 0.1 s DAP and 2 s CM/RCS DAP. */
    private void exdap4() {
        if (g("JETAG") < 0) {
            agc.set("ROLLJETS", 0);
            agc.set("JETAG", 0);
        }
        exdap3();
    }

    private void exdap3() {
        int raxerr;
        if (g("CALFA") > 0) {
            raxerr = g("RAXERR");
        } else {
            s("CMDAPMOD", 0);                               // -0 for CM/RCS
            agc.set("CMDAPMOD", Agc.WORD_MASK);
            raxerr = -g("RAXERR");
        }
        exdap1(raxerr);
    }

    /** EXDAP1: yaw FDAI + BETA control (jets) and ALFA control with biased dead zones. */
    private void exdap1(int raxerr) {
        s("RAXERR", raxerr);
        s("AK2", raxerr);
        int rate = (agc.get("CMDAPMOD") == 1 || g("CMDAPMOD") > 0) ? g("BETADOT") : g("RREL");
        int tag = biasedz(rate, raxerr);
        int jetem = YJETCODE[tagIndex(tag)];
        tag = biasedz(g("QREL"), g("QAXERR"));
        exdapin(jetem, tag);
    }

    /** EXDAP2: |CALFA| < cos(45): CMDAPMOD = +1; roll control with yaw jets, beta with P jets. */
    private void exdap2() {
        s("CMDAPMOD", g("CMDAPMOD") + 1);
        s("JETAG", -1);
        if (g("P63FLAG") > 0) {
            s("P63FLAG", 0);
        }
        int pax = g("SALFA") > 0 ? -g("RAXERR") : g("RAXERR");
        s("PAXERR1", mp(pax, HALF));                         // ERROR/360 for FDAI
        int betaRate = g("SALFA") > 0 ? -g("BETADOT") : g("BETADOT");
        int tag = biasedz(betaRate, pax);
        agc.set("ROLLJETS", PRJCODE[tagIndex(tag)]);
        int err = g("ROLLHOLD") - g("ROLL/180");
        s("L", err);
        exdap1(g("SALFA") > 0 ? err : -err);
    }

    // ===================================================================================
    // JETAG setup, CM/RCS
    // ===================================================================================

    /** SETJTAG: save nominal update time for synch; JETAG=1 so CM/RCS runs after next BODYRATE. */
    public void setjtag() {
        s("TUSED", -g("TIME1"));
        s("JETAG", 1);
        extern("PHASCHNG", 00001);
        extern("TASKOVER");
    }

    /** CM/RCS: 2 s roll-jet logic: phase-plane switching on (LCX, V) with coasting and TON1/TOFF/TON2. */
    public void cmRcs() {
        s("JNDX", -1);
        int vt = mp(-TWO_T_TCDU, g("PREL")) + g("L");          // -2 VT/180
        s("-VT/180", vt);
        s("SR", -g("ROLL/180"));
        int lcx;
        if ((agc.get("CM/FLAGS") & BIT4) == 0) {                // LATSW: roll over top
            agc.set("CM/FLAGS", agc.get("CM/FLAGS") | BIT4);
            lcx = g("ROLLC") + g("SR");
            s("LCX/360", lcx);
        } else {
            lcx = (HALF * 2 + g("ROLLC")) + (g("SR") - 2 * HALF);
            s("LCX/360", lcx);
            // take shortest angular path (based on single jet acceleration)
            int calfa = g("CALFA") == 0 ? 1 : g("CALFA");
            int absVt = Math.abs(vt);
            int corr = (int) ((long) mp(-absVt, vt) * (int) Math.round(D180_8ATT * ONE_F) / calfa);
            int sum = (lcx > 0 ? HALF : -HALF) + lcx + corr;
            if (Math.abs(sum) > 16383) {
                lcx += lcx > 0 ? -ONE_F : ONE_F;                 // TRTAGXPI: shift X by -+ 2 pi
            }
            s("LCX/360", lcx);
        }
        compat();
    }

    /** COMPAT: correct for assumed coordinated turn then branch on CMDAPMOD. */
    private void compat() {
        s("LCX/360", mp(g("LCX/360"), g("CALFA")));
        int mode = g("CMDAPMOD");
        if (agc.get("CMDAPMOD") == 0 && mode == 0) {            // positive: all 3 axes done
            dzcall1(0);
            return;
        }
        if (mode < 0 && agc.get("CMDAPMOD") != 0) {
            // G >= .05: continue
        } else {
            s("LCX/360", 0);                                     // CMDAPMOD=-0: damping only
        }
        s("ERRORZ", g("LCX/360"));
        s("PAXERR1", g("LCX/360"));
        s("SR", g("-VT/180"));
        s("-VT/180E", g("-VT/180") / 2);
        s("-VT/180", g("-VT/180") / 2);
        s("VSQ/4API", mp(mp(g("-VT/180"), g("-VT/180")), (int) Math.round(D180_8ATT * ONE_F)));
        whichhalf();
    }

    /** WHICHALF: select half plane relative to the switch curve; reflect LHP into RHP. */
    private void whichhalf() {
        int l = -(2 * g("VSQ/4API")) + BUFLIM;
        l = g("-VT/180") > 0 ? -l : l;
        int t = l + g("LCX/360") + BUFLIM;
        if (t <= 0) {                                            // point in LHP: reflect
            s("-VT/180", -g("-VT/180"));
            s("SR", g("-VT/180"));
            s("LCX/360", -g("LCX/360"));
            s("JNDX", -g("JNDX"));
            dz1();
            return;
        }
        if (-t + 2 * BUFLIM <= 0) {                              // RHP
            dz1();
            return;
        }
        if (-(int) Math.round(VSQMIN * ONE_F) + g("VSQ/4API") <= 0) {
            dzcall();
            return;
        }
        s("JNDX1", -g("JNDX"));
        ovrline1();
    }

    private void dz1() {
        if (-(int) Math.round(VSQMIN * ONE_F) + g("VSQ/4API") <= 0) {
            // DZ2: in dead zone if X - XMIN - VT < 0
            if (-XMIN_360 + g("LCX/360") + g("SR") <= 0) {
                dzcall();
                return;
            }
        }
        maxvtest();
    }

    /** MAXVTEST: compute TON1 (accelerate), TOFF (coast) and TON2 (decelerate) in 4T units. */
    private void maxvtest() {
        s("JNDX1", -g("JNDX"));
        int xd = -XS_360 + g("VSQ/4API") + g("LCX/360");
        s("XD/360", xd);
        int vdt;
        boolean vmLimit = (-(xd + (int) Math.round(MINUS_VM_360K * ONE_F))) <= 0 ? false : false;
        if (xd - (int) Math.round(MINUS_VM_360K * ONE_F) * 0 > 0 && -(xd + (int) Math.round(MINUS_VM_360K * ONE_F)) <= 0) {
            vdt = (int) Math.round(MINUS_VM_360K * ONE_F);        // thrust to VM
            vmLimit = true;
        } else {
            vdt = 2 * mp(xd, HALF);                               // predicted drifting velocity
        }
        s("VDT/180", vdt);
        int ton1 = mp(2 * (vdt + g("-VT/180")), (int) Math.round(D180_8ATT * ONE_F));
        s("TON1", ton1);
        if (ton1 <= 0) {
            ovrline1();
            return;
        }
        if (vmLimit) {                                            // MAXVTIM2: above VM thrust down
            s("JNDX", g("JNDX1"));
            s("TON1", -ton1);
            ovrline2(true);
            return;
        }
        getOn2();
    }

    private void ovrline1() {
        s("VDT/180", -g("-VT/180"));                              // drift at V
        ovrline2(false);
    }

    private void ovrline2(boolean keepTon1) {
        if (!keepTon1) {
            s("TON1", 0);
        } else {
            s("TON1", 0);
        }
        getOn2();
    }

    private void getOn2() {
        int ton2 = 2 * mp(2 * g("VDT/180"), (int) Math.round(D180_8ATT * ONE_F));
        if (ton2 > 0) {
            ton2 = 0;
            s("TON2", 0);
        } else {
            s("TON2", -ton2);
            s("JNDX1", g("JNDX"));
            ton2 = -ton2;
        }
        s("TON2", ton2);
        getoff();
    }

    /** GETOFF: coast time TOFF = ((-VDT/180 - VT) TON1 + (-TON2 VDT) + LCX) / VDT, in 2T units. */
    private void getoff() {
        int vdt = g("VDT/180");
        int toff;
        if (vdt == 0) {
            toff = TWO_JETT;                                      // TOFFOVFL: use 2T
        } else {
            int num = mp(-vdt + g("-VT/180"), g("TON1")) + mp(-g("TON2"), vdt) + g("LCX/360");
            long q = (long) num * ONE_F / vdt;
            toff = Math.abs(q) >= ONE_F ? TWO_JETT : mp((int) q, TWO_JETT);
        }
        timscal(toff);
    }

    /** TIMSCAL: scale TOFF, TON1, TON2 to centiseconds, then TIMETST. */
    private void timscal(int toff) {
        s("TOFF", toff);
        s("TON1", mp(FOUR_JETT, g("TON1")));
        s("TON2", mp(FOUR_JETT, g("TON2")));
        s("JETAG", 0);                                            // roll DAP called
        s("SW/NDX", 1);
        timetst();
    }

    /** TIMETST: account for elapsed time since SETJTAG; trim TON1/TOFF/TON2 sequence. */
    private void timetst() {
        int tused = g("TUSED") + g("TIME1");
        tused += MINUS_T_3;
        s("TUSED", tused);
        int ton1 = g("TON1");
        if (ton1 - 2 > 0) {
            agc.set("T1BITS", PRJCODE[tagIndex(g("JNDX"))]);
            tused += ton1;
            if (tused > 0) {
                timetst3(0);
                return;
            }
        } else {
            s("TON1", -1);
        }
        tofftest(tused);
    }

    private void tofftest(int tused) {
        int toff = g("TOFF");
        if (toff - 2 > 0) {
            tused += toff;
            if (tused > 0) {
                timetst4(0);
                return;
            }
        } else {
            s("TOFF", -1);
        }
        ton2test(tused);
    }

    private void ton2test(int tused) {
        int ton2 = g("TON2");
        if (ton2 - 2 > 0) {
            agc.set("T2BITS", PRJCODE[tagIndex(g("JNDX1"))]);
            tused += ton2;
            if (tused > 0) {
                s("TON2", 0);
                jetcall1();
                return;
            }
        }
        timetst5();
    }

    private void timetst3(int a) {
        s("TON1", a);
        timetst4(-1);
    }

    private void timetst4(int a) {
        s("TOFF", a);
        timetst5();
    }

    private void timetst5() {
        s("TON2", -1);
        jetcall1();
    }

    /** JETCALL1: queue waitlist calls for each pending TON1/TOFF/TON2 interval then JETACTN. */
    private void jetcall1() {
        s("OUTTAG", 0);
        s("NUJET", 0);
        s("TBITS", 0);
        int[] times = {g("TON1"), g("TOFF"), g("TON2")};
        String[] bits = {"T1BITS", "", "T2BITS"};
        int nujet = 0;
        for (int i = 0; i < 3; i++) {
            if (times[i] > 0) {
                nujet += i == 1 ? 0 : agc.get(bits[i]);
                s("NUJET", nujet);
                extern("WAITLIST", times[i] + 1, "JETCALL");
            }
            s(i == 0 ? "TON1" : i == 1 ? "TOFF" : "TON2", -1);
        }
        jetactn();
    }

    /** JETACTN: write the roll jet code now known (NUJET) to ROLLJETS. */
    private void jetactn() {
        agc.set("ROLLJETS", agc.get("NUJET"));
        if (g("OUTTAG") != 0) {
            extern("TASKOVER");
            return;
        }
        cmFdair();                                                // ROLLDUMP
    }

    /** JETCALL: waitlist re-entry; if system disabled (CM/DSTBY) quench roll jets. */
    public void jetcall() {
        s("OUTTAG", BIT2);
        if ((agc.get("CM/FLAGS") & BIT2) == 0) {
            agc.set("NUJET", 0);
            jetactn();
            return;
        }
        jetactn();
    }

    /** DZCALL: in dead zone; save roll angle or roll command, cancel roll jets. */
    private void dzcall() {
        boolean negZero = agc.get("CMDAPMOD") == Agc.WORD_MASK;
        if (negZero) {
            s("ROLLHOLD", angovcor(g("ROLLTM") + 0));
        } else {
            s("ROLLHOLD", angovcor(g("ROLLC") * 2));
        }
        agc.set("ROLLJETS", 0);
        s("VDT/180", 0);
        dzcall1(0);
    }

    private void dzcall1(int a) {
        s("JETAG", a);
        cmFdair();                                                // ROLLDUMP
    }

    /** CM/FDAI: integrate roll error between 2 s updates (each 0.1 s). */
    public void cmFdai() {
        int d = mp(mp(-g("PHIDOT"), g("CALFA")), HALF);
        s("PAXERR1", g("PAXERR1") + d);
        cmFdair();
    }

    /** CM/FDAIR: roll needle: PAXERR = .25 (roll error/180); then CM/DUMPR. */
    public void cmFdair() {
        s("PAXERR", mp(HALF, g("PAXERR1")));
        cmDumpr();
    }

    /** CM/DUMPR: alternate between filing body rates for telemetry and updating needles. */
    public void cmDumpr() {
        int idx = -g("SW/NDX");
        s("SW/NDX", idx);
        if (idx > 0) {
            extern("IBNKCALL", "NEEDLER");
            cmEnd();
            return;
        }
        idx += 3;                                                 // CMTMFILE
        if (idx <= 0) {
            // SAVENDX with idx negative
        } else {
            s("CMTMTIME", g("TIME1"));
            idx = -13;
        }
        s("SW/NDX", idx);
        agc.set("ENDBUF" + (idx - 1), agc.get("PREL"));
        agc.set("ENDBUF" + idx, agc.get("PREL+1"));
        agc.set("ENDBUF" + (idx + 1), agc.get("RREL"));
        cmEnd();
    }

    /** CM/END: restore SR; point T5 back to the idler and RESUME. */
    public void cmEnd() {
        agc.set("SR", agc.get("CM/SAVE"));
        agc.set("T5LOC", agc.get("T5IDLOC"));
        extern("RESUME");
    }
}
