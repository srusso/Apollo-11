package apollo.comanche055;

import apollo.agc.Agc;

/**
 * REENTRY_CONTROL.agc (bank 25/26, REENTRY / REENTRY1): entry initialisation (STARTENT), targeting
 * (TARGETNG: relative velocity, drag, range angle), and the roll-command phases of re-entry:
 * INITROLL, HUNTEST (range prediction RANGER), UPCONTRL, CONSTD/DOWNCNTL, KEP2 (Kepler phase),
 * PREFINAL/PREDICT3 (sub-orbital reference trajectory table VREFER), GLIMITER, LIMITL/D (lateral
 * control) and the display exits OVERNOUT / STEEROFF.
 *
 * The interpretive-language code is rendered with doubles holding the stored (scaled) fraction of
 * each double-precision erasable word, so "scaling" comments in the source still apply to values
 * here. Pushdown-list temporaries become local variables. GOTOADDR selects the current phase.
 */
public class ReentryControl {
    // ---- GOTOADDR phase selectors (address of the roll command equations for the current phase) ----
    public static final int INITROLL = 1;
    public static final int HUNTEST = 2;
    public static final int UPCONTRL = 3;
    public static final int KEP2 = 4;
    public static final int PREDICT3 = 5;
    public static final int PREFINAL = 6;
    public static final int ENDEXIT = 7;
    public static final int P67_2 = 8;
    public static final int SCALEPOP = 9;
    public static final int GETETA = 10;

    // ---- CM/FLAGS bits (STATE +6) ----
    public static final int ENTRYDSP = 1 << 12;   // 92D B13
    public static final int GONEPAST = 1 << 9;    // 95D B10
    public static final int RELVELSW = 1 << 8;    // 96D B9
    public static final int EGSW = 1 << 7;        // 97D B8
    public static final int NOSWITCH = 1 << 6;    // 98D B7
    public static final int HIND = 1 << 5;        // 99D B6
    public static final int INRLSW = 1 << 4;      // 100D B5
    public static final int LATSW = 1 << 3;       // 101D B4
    public static final int G05SW = 1 << 2;       // .05GSW 102D B3
    /** GONEBY: 112D bit 8 of FLAG7 (STATE +7), self-initialising. */
    public static final int GONEBY = 1 << 7;
    /** ENTMASK: initialise all switches to zero except LATSW, ENTRYDSP, GONEPAST. */
    public static final int ENTMASK = 011774;
    /** ENTRYSW: ENTRYDSP B13, GONEPAST B10, LATSW B4. */
    public static final int ENTRYSW = 011010;

    // ---- constants ----
    private static final double NEARONE = 0.999999999;
    public static final double ONE_BIT_DP = 1.0 / (1L << 28);
    /** 500SEC: 2DEC 50000 B-28 (500 seconds in centiseconds, scaled B-28). */
    public static final double SEC500 = 50000.0 / (1L << 28);
    public static final double MAXRNG = ((07577L << 14) | 03565L) / (double) (1L << 28);
    public static final double ONE_TWELFTH = 0.083333;
    public static final double ONE_THIRD = 0.3333333333;
    public static final double ONE_SIXTEENTH = 1.0 / 16;
    public static final double LEWD1 = 0.15;
    public static final double POINT1 = 0.1;
    public static final double POINT2 = 0.2;
    public static final double DLEWD0 = -0.05;
    public static final double GMAX_2 = 0.16;
    public static final double NEAR_ONE_QUARTER = 0.249999;
    public static final double C18 = 0.0097026346;
    public static final double Q7FKDMIN = 0.0080745342;
    public static final double Q3 = 0.167003132;
    public static final double Q5 = 0.326388889;
    public static final double Q6 = 0.0349;
    public static final double Q7F = 0.0074534161;
    public static final double Q19 = 0.5;
    public static final double Q21 = 0.0231481481;
    public static final double Q22 = -0.053333333;
    public static final double VLMIN = 0.34929485;
    public static final double C12 = 0.00684572901;
    public static final double ONE_OVER_KB1 = 0.29411765;
    public static final double NEG_ONE_OVER_KB2 = -0.0057074322 * 16;   // B4
    public static final double VQUIT = 0.019405269;
    public static final double C20 = 0.21739130;
    public static final double C21 = 0.17391304;
    public static final double NM25 = 0.0011574074;
    public static final double K1D = 0.0314453125;
    public static final double K2D = -0.201298418;
    public static final double KVSCALE = 0.81491944;
    public static final double KASCALE = 0.97657358;
    public static final double KTETA = 0.383495203E2 / 16384;
    public static final double KT1 = 0.157788327E2 / 16384;
    public static final double G05 = 0.002;
    public static final double LATBIAS = 0.00003;
    public static final double KWE = 0.120056652 / 2;
    public static final double KACOS = 0.004973592;
    public static final double CHOOK = 1.0 / 64;
    public static final double ONE_24TH = 0.0833333333 / 2;
    public static final double CH1 = 0.32 * 2;
    public static final double KC3 = -0.0247622232;
    public static final double VRCONT = 0.0135836886;
    public static final double HALVE = 0.5;
    public static final double FOURTH = 0.25;
    public static final double ONE_OVER_GMAX = HALVE;
    public static final double TWO_HS = 0.0172786611;
    public static final double TWO_HS_GMXSQ = 0.0000305717;
    public static final double C001 = -0.000625;
    public static final double POINT8 = 0.8;
    public static final double TWO_C1_HS = 0.0215983264;
    public static final double PT1_16 = 0.1 / 16;
    public static final double ONE_OVER_K44 = 0.00260929464;
    public static final double VFINAL = 0.51618016;
    public static final double VFINAL1 = 0.523942273;
    public static final double ONE_OVER_KA1 = 0.30048077;
    public static final double KA2 = 0.008;
    public static final double KA3 = 0.44720497;
    public static final double KA4 = 0.049689441;
    public static final double KALIM = 0.06;
    public static final double Q7MIN = KA4;
    public static final double NEG_HSCALED = -0.55305018;
    public static final double NEG_KSCALE = -0.0312424837;
    public static final double COS15 = 0.965;
    public static final double LATSLOPE = ONE_TWELFTH;

    /** VREFER: 13-point reference velocity table followed by six 13-point functions of V. */
    public static final double[] VREFER = {
        // reference velocity V/51532.3946
        .019288, .040809, .076107, .122156, .165546, .196012, .271945, .309533, .356222, .404192, .448067, .456023, .67918,
        // DRANGE/DA scaled DRDA/(2700/805)
        -.010337, -.016550, -.026935, -.042039, -.058974, -.070721, -.098538, -.107482, -.147762, -.193289, -.602557, -.99999, -.99999,
        // -DRANGE/DRDOT scaled ((2VS/8 2700) DR/DRDOT), B-3
        -.0478599 / 8, -.0683663 / 8, -.1343468 / 8, -.2759846 / 8, -.4731437 / 8, -.6472087 / 8, -1.171693 / 8,
        -1.466382 / 8, -1.905171 / 8, -2.547990 / 8, -4.151220 / 8, -5.813617 / 8, -5.813617 / 8,
        // RDOTREF scaled (8 RDT/2VS), B3
        -.0134001 * 8, -.013947 * 8, -.013462 * 8, -.011813 * 8, -.0095631 * 8, -.00806946 * 8, -.006828 * 8,
        -.00806946 * 8, -.0109791 * 8, -.0151498 * 8, -.0179817 * 8, -.0159061 * 8, -.0159061 * 8,
        // range to go scaled RTOGO/2700
        .0008067, .0032963, .0081852, .017148, .027926, .037, .063296, .077889, .098815, .127519, .186963, .238148, .294185185,
        // -AREF/805
        -.051099, -.074534, -.101242, -.116646, -.122360, -.127081, -.147453, -.155528, -.149565, -.118509, -.034907, -.007950, -.007950,
        // DRANGE/D L/D scaled Y/2700
        .004491, .008081, .016030, .035815, .069422, .104519, .122, .172407, .252852, .363148, .512963, .558519, .558519
    };

    private final Agc agc;
    /** OVFIND: interpreter overflow indicator. */
    private boolean ovf;

    public ReentryControl(Agc agc) {
        this.agc = agc;
    }

    // ---------------- helpers ----------------

    /** Reads a double-precision erasable value as a fraction (upper*2^14+lower)/2^28. */
    private double d(String name) {
        long v = ((long) Agc.toSigned(agc.get(name)) << 14) + Agc.toSigned(agc.get(name + "+1"));
        return v / (double) (1L << 28);
    }

    private void sd(String name, double x) {
        long v = Math.round(x * (1L << 28));
        int sign = v < 0 ? -1 : 1;
        long mag = Math.abs(v);
        agc.set(name, Agc.fromSigned(sign * (int) ((mag >> 14) & 0x3FFF)));
        agc.set(name + "+1", Agc.fromSigned(sign * (int) (mag & 0x3FFF)));
    }

    /** Reads a stored vector (three double-precision components at name, name+2, name+4). */
    private double[] vec(String name) {
        return new double[] {d(name), d(name + "+2"), d(name + "+4")};
    }

    private void svec(String name, double[] v) {
        sd(name, v[0]);
        sd(name + "+2", v[1]);
        sd(name + "+4", v[2]);
    }

    private static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }

    private static double abval(double[] a) {
        return Math.sqrt(dot(a, a));
    }

    private static double[] unit(double[] a) {
        double m = abval(a);
        return m == 0 ? a : new double[] {a[0] / m, a[1] / m, a[2] / m};
    }

    /** SIGN: attach the (negated if operand negative) sign. */
    private static double sign(double mpac, double operand) {
        return operand < 0 ? -mpac : mpac;
    }

    /** Overflow check on a result that must be a fraction; sets OVFIND and clamps. */
    private double chk(double x) {
        if (Math.abs(x) >= 1.0) {
            ovf = true;
            return x < 0 ? -NEARONE : NEARONE;
        }
        return x;
    }

    private double ddv(double a, double b) {
        return b == 0 ? chk(a < 0 ? -2 : 2) : chk(a / b);
    }

    private boolean flag(int mask) {
        return (agc.get("CM/FLAGS") & mask) != 0;
    }

    private void setFlag(int mask) {
        agc.set("CM/FLAGS", agc.get("CM/FLAGS") | mask);
    }

    private void clearFlag(int mask) {
        agc.set("CM/FLAGS", agc.get("CM/FLAGS") & ~mask);
    }

    private boolean goneby() {
        return (agc.get("STATE+7") & GONEBY) != 0;
    }

    private void setGoneby(boolean on) {
        agc.set("STATE+7", on ? agc.get("STATE+7") | GONEBY : agc.get("STATE+7") & ~GONEBY);
    }

    private void gotoAddr(int phase) {
        agc.set("GOTOADDR", phase);
    }

    /** GOTO GOTOADDR: jump to the roll command equations of the current phase. */
    private void dispatch() {
        switch (agc.get("GOTOADDR")) {
            case INITROLL: initroll(); break;
            case HUNTEST: huntest(); break;
            case UPCONTRL: upcontrl(); break;
            case KEP2: kep2(); break;
            case PREDICT3: predict3(); break;
            case PREFINAL: prefinal(); break;
            case ENDEXIT: endexit(); break;
            case P67_2: new P61P67(agc).p672(); break;
            default: break;
        }
    }

    private static double acosRev(double c) {
        return Math.acos(Math.max(-1.0, Math.min(1.0, c))) / (2 * Math.PI);
    }

    // ---------------- ENTRY INITIALIZATION ROUTINE ----------------

    /** STARTENT: MM=63; come here from CM/POSE (restarted in CM/POSE). Initialise switches and constants. */
    public void startent() {
        // CS ENTMASK ; MASK CM/FLAGS ; AD ENTRYSW : all switches zero except LATSW, ENTRYDSP, GONEPAST
        agc.set("CM/FLAGS", Agc.add((~ENTMASK) & agc.get("CM/FLAGS") & Agc.WORD_MASK, ENTRYSW));
        sd("LOD", d("LODPAD"));
        sd("LAD", d("LADPAD"));
        sd("L/DCMINR", d("LAD") * COS15);                   // L/DCMINR = LAD COS(15)
        sd("KLAT", LATSLOPE * d("LAD") / 2);                // KLAT = LAD/24
        sd("Q7", Q7F);                                      // Q7 = Q7F
        sd("FACTOR", NEARONE);                              // 1.0 - 1BIT
        sd("L/D", -sign(d("LAD"), d("HEADSUP")));           // L/D = -LAD SGN(HEADSUP)
        starten1();
        // VN x UNITR, unit, dot RT : LATANG = UNI.RT/4
        double latang = dot(unit(cross(vec("VN"), vec("UNITR"))), vec("RT"));
        sd("LATANG", latang);
        sd("K2ROLL", latang < 0 ? NEARONE : -NEARONE);      // K2ROLL = -SGN(LATANG)
        sd("Q2", d("LAD") * Q21 + Q22);                     // Q2 = -1152 + 500 LAD
        gotoAddr(INITROLL);                                 // selector for initial pass
        agc.set("POSEXIT", SCALEPOP);                       // CM/POSE continues at SCALEPOP
        servnout();                                         // omit initial display, since 1st guess bad
    }

    /**
     * STARTEN1: calculate the initial target vector RTINIT (also RTEAST, RTNORM and RT), all .5 unit
     * in reference coordinates.
     */
    public void starten1() {
        svec("LAT", vec("LAT(SPL)"));                       // target coordinates
        // CLEAR ERADFLAG, LUNAFLAG : do call using pad radius
        agc.set("ERADFLAG", 0);
        agc.set("LUNAFLAG", 0);
        sd("LAT+4", 0);                                     // ALT = 0
        sd("TIME/RTO", d("PIPTIME"));                       // time base of RTINIT
        new LatitudeLongitudeSubroutines(agc).lalotorv(r -> r);   // result in ALPHAV
        svec("RTINIT", unit(vec("ALPHAV")));                // .5 unit target, reference coords
        sd("DTEAROT", SEC500);                              // nominal entry time for P63
        new CsmGeometry(agc).earrot1(d("DTEAROT"));  // get RT
        double dotv = dot(vec("RT"), vec("UNITR"));         // RT/2 . UNITR
        sd("THETAH", acosRev(4 * dotv));                    // range angle /360
        // return to caller via GOTOADDR
    }

    /** SCALEPOP: CALL TARGETNG, then EXIT to REFAZE10. */
    public void scalepop() {
        targetng();
        refaze10();
    }

    /** REFAZE10: PHASCHNG 10035 (SERVICER 5.3 restart), then jump to the current phase. */
    public void refaze10() {
        // TC PHASCHNG ; OCT 10035
        dispatch();                                         // GOTO GOTOADDR
    }

    // ---------------- TARGETING ----------------

    /**
     * TARGETNG: called by P61 (group 4) and ENTRY (group 5). Processes average-G output: relative or
     * inertial velocity, drag, range angle THETAH, target vector RT.
     */
    public void targetng() {
        double[] vel;
        if (!flag(RELVELSW)) {
            vel = vec("VN");
            for (int i = 0; i < 3; i++) {
                vel[i] *= KVSCALE;                          // KVSCALE = (12800/.3048)/2VS; V/2VS
            }
            svec("VEL", vel);                               // GETVEL
        } else {
            vel = vec("-VREL");
            for (int i = 0; i < 3; i++) {
                vel[i] = -vel[i];                           // VCOMP : (VREL) = (V) + KWE UNITR*UNITW
            }
            svec("VEL", vel);
        }
        // GETUNITV
        double vmag = abval(vel);
        svec("UNITV", unit(vel));
        sd("VSQUARE", vmag * vmag);                         // VSQ/4
        sd("LEQ", vmag * vmag - FOURTH);                    // LEQ/4 (4 G-S full scale)
        sd("V", vmag);                                      // V/2VS
        double[] unitr = vec("UNITR");
        sd("RDOT", 2 * dot(vel, unitr));                    // RDOT = V.UNITR
        double dmag = abval(vec("DELV")) * KASCALE * 2;     // PIPA counts in platform coordinates
        sd("D", dmag == 0 ? ONE_BIT_DP : dmag);             // SETMIND if zero; acceleration approximates drag
        svec("UNI", unit(cross(vel, unitr)));               // .5 UNI, reference coordinates
        double dtIn = 0;
        if (!flag(RELVELSW)) {
            dtIn = geteta();
        }
        updatert(dtIn);
    }

    /** GETETA: D = D + D(-RDOT/HS - 2D/V) DT/2; also returns the time-step value (THETAH based) for UPDATERT. */
    private double geteta() {
        double v = d("V");
        double dd = d("D");
        double t = ddv(d("RDOT"), NEG_HSCALED);             // -RDOT/HS
        t = t + ddv(dd * NEG_KSCALE, v);                    // 2D/V term (-KSCALE D / V)
        dd = dd + dd * t;
        sd("D", dd);
        if (flag(EGSW)) {
            return subeta(v);
        }
        return d("THETAH") * KTETA;                         // KTETA = 1000x2PI/(2)E14 163.84
    }

    /** SUBETA: switch from inertial to relative velocity. */
    private double subeta(double v) {
        if (v - FOURTH >= 0) {                              // VMIN = FOURTH
            setFlag(RELVELSW);
        }
        return ddv(d("THETAH") * KT1, v);                   // KT = RE(2 PI)/2 VS 16384 163.84/ 2 VSAT
    }

    /** UPDATERT: elapsed time since RTINIT; predicted target vector RT; THETAH, GONEBY, D L/D calc. */
    private void updatert(double mpac) {
        double dt = mpac - d("TIME/RTO") + d("PIPTIME");    // elapsed time since RTINIT was established
        sd("DTEAROT", dt);
        new CsmGeometry(agc).earrot2();                     // get predicted RT
        double[] rt = vec("RT");
        double[] uni = vec("UNI");
        double[] unitr = vec("UNITR");
        sd("LATANG", dot(rt, uni));                         // LATANG = RT.UNI /4
        setGoneby(false);                                   // have not gone past target
        if (dot(cross(rt, unitr), uni) < 0) {
            setGoneby(true);                                // RT*UNITR.UNI negative: gone past target
        }
        // GETANGLE: THETA = ARCCOS(RT.UNITR)
        double x = dot(rt, unitr) - NEAR_ONE_QUARTER;
        double thetah;
        if (x >= 0) {
            thetah = tinythet(x);                           // to improve accuracy for small angles
        } else {
            thetah = acosRev(4 * dot(rt, unitr));
        }
        sd("THETAH", thetah);                               // THETAH/360
        sd("RTGON67", goneby() ? thetah : -thetah);         // range error: neg if will fall short
        if (d("D") - G05 < 0) {
            clearFlag(G05SW);                               // NO.05G : this way for DAP
        } else {
            setFlag(G05SW);
            double[] dv = vec("DELVREF");
            double p = 2 * dot(dv, vec("UXA/2"));
            p = p * p;
            ovf = false;
            double q = (dot(dv, dv) - p);
            double r = ddv(q, p);
            if (!ovf) {
                sd("L/DCALC", Math.sqrt(Math.abs(r)));      // NOLDCALC skips if overflow
            }
        }
        // GOTO 60GENRET : return to caller
    }

    /** TINYTHET: enter with X-.249; small-angle ARCCOS. */
    private double tinythet(double x) {
        double y = Math.abs(x - ONE_BIT_DP);
        return KACOS * Math.sqrt(y * 8192);
    }

    // ---------------- INITIAL ROLL ----------------

    /** INITROLL: MM = 63, 64. Holds initial roll attitude until KAT exceeded, then until VRCONT. */
    public void initroll() {
        if (flag(INRLSW) || !flag(G05SW)) {
            // BON INRLSW INITRL1 ; BOFF .05GSW LIMITL/D
            if (!flag(INRLSW)) {
                limitLD();
                return;
            }
            initrl1();
            return;
        }
        // MM = 64: KA = KA1 LEQ^3 + KA2
        double leq = d("LEQ");
        double kat = leq * leq * leq * ONE_OVER_KA1 + KA2;
        // P64 : RTB (display ROLLC, VI, RDOT)
        new P61P67(agc).p64();
        sd("KAT", kat);
        if (kat - KALIM >= 0) {
            sd("KAT", KALIM);
        }
        if (d("V") - VFINAL1 < 0) {                         // go to final phase
            clearFlag(GONEPAST);                            // GONEPAST was set to force ROLLC = HEADSUP value
            gotoAddr(KEP2);                                 // idle until D > 0.2 G (no P66 here)
            inrolout();
            return;
        }
        clearFlag(GONEPAST);
        d0eq();
    }

    /** D0EQ: D0 = KA3 LEQ + KA4; initial constant-drag parameters. */
    private void d0eq() {
        double d0 = d("LEQ") * KA3 + KA4;                   // D0/805
        sd("D0", d0);
        ovf = false;
        double c = ddv(C001, d0);                           // (-4/D0) B-8
        sd("C/D0", c);
        sd("L/D", d("LAD"));                                // if V-VFINAL+K(RDOT/V)^3 positive, L/D = -LAD
        ovf = false;
        double r = ddv(d("RDOT"), d("V"));
        double t = r * r * r * ONE_OVER_K44;                // (RDOT/V)^3 / K44
        double w = d("V") - VFINAL + t;
        if (ovf || w < 0) {
            inrolout();                                     // go to LIMITL/D after setting INRLSW
            return;
        }
        sd("L/D", -d("LAD"));
        inrolout();
    }

    /** INROLOUT: end of the pre-.05G path of INITROLL; set INRLSW (restart protection), go to LIMITL/D. */
    private void inrolout() {
        if (!flag(INRLSW)) {
            setFlag(INRLSW);
        }
        limitLD();
    }

    /** KATEST: if KAT - D positive, go to LIMITL/D, else CONSTD. */
    private void katest() {
        if (d("KAT") - d("D") >= 0) {
            limitLD();
        } else {
            constd();
        }
    }

    /** INITRL1: if RDOT + VRCONT positive, fall into HUNTEST. */
    private void initrl1() {
        if (d("RDOT") + VRCONT < 0) {
            katest();
            return;
        }
        forehunt();                                         // initialise HUNTEST
        gotoAddr(HUNTEST);                                  // must go after FOREHUNT for restarts
        huntest();
    }

    // ---------------- HUNT TEST ----------------

    /**
     * HUNTEST: MM = 64. Checks whether the predicted range at nominal L/D is less than the desired
     * range; if not, constant drag controller; if so, control set to UPCONTRL.
     */
    public void huntest() {
        double dd = d("D");
        double rdot = d("RDOT");
        double v = d("V");
        sd("A1", dd);                                       // A1/805 = A1/25G
        double tem1b = rdot < 0 ? d("LAD") : d("LEWD");     // if RDOT neg, TEM1B=LAD, else LEWD
        sd("TEM1B", tem1b);
        // A0CALC
        double v1 = ddv(rdot, tem1b) + v;                   // V1 = V + RDOT/TEM1B
        sd("V1", v1);
        double a0 = ((dd + ddv(ddv(rdot * rdot, tem1b), TWO_C1_HS)) * v1 * v1) / d("VSQUARE");
        sd("A0", a0);                                       // A0 = (V1/V)SQ(D + RDOT SQ/(TEM1B 2 C1 HS))
        if (rdot < 0) {
            sd("A1", a0);                                   // A1/25G
        }
        // V1LEAD: if L/D negative, V1 = V1 - VQUIT
        if (d("L/D") < 0) {
            v1 = v1 - VQUIT;
            sd("V1", v1);
        }
        // HUNTEST1
        double lewd = d("LEWD");
        double alp = a0 * TWO_C1_HS / v1 / v1 / lewd;       // ALP = 2 C1 HS A0/LEWD V1 V1
        sd("ALP", alp);
        double fact1 = v1 / (NEARONE - alp);                // FACT1 / 2VS
        sd("FACT1", fact1);
        double fact2 = (alp - NEARONE) * alp / a0;          // FACT2 (25G)
        sd("FACT2", fact2);
        double q7 = d("Q7");
        double vl = fact1 * (NEARONE - Math.sqrt(Math.max(0, q7 * fact2 + alp)));   // VL=FACT1(1-SQRT(Q7 FACT2+ALP))
        sd("VL", vl);
        sd("GAMMAL1", (v1 - vl) * lewd / vl);               // GAMMAL1 = LEWD (V1-VL)/VL
        if (vl - VLMIN < 0) {
            prefinal();                                     // PREFINAL
            return;
        }
        double vbars = vl * vl;
        sd("VBARS", vbars);                                 // VBARS / 4 VS VS
        if (HALVE - vl < 0) {
            beconstd();                                     // set MODE=HUNTEST, continue in CONSTD
            return;
        }
        double dvl = HALVE - vl;                            // DVL / 2VS
        double vs1 = HALVE;                                 // VS1 = VSAT
        if (HALVE - v1 >= 0) {
            dvl = dvl - (HALVE - v1);                       // DVL = V1 - VL
            vs1 = v1;                                       // VS1 = V1, in this case
        }
        sd("DVL", dvl);
        sd("VS1", vs1);
        // GETDHOOK: DHOOK=((1-VS1/FACT1) SQ -ALP)/FACT2
        double dhook = dhookyq7(vs1);
        sd("DHOOK", dhook);                                 // DHOOK / 25G
        double ahookdv = ddv(dhook / 64, q7) - CHOOK;
        sd("AHOOKDV", ahookdv);
        double gamma1 = d("GAMMAL1");
        double gammal = gamma1 - (ahookdv + ONE_SIXTEENTH) * CH1 * dvl * dvl / dhook / vbars;   // GAMMAL = GAMMAL1-CH1 DVL SQ(1+AHOOK DVL)
        if (gammal < 0) {
            gammal = negama(gammal);
        }
        huntest3(gammal);
    }

    /** HUNTEST3: store GAMMAL, GAMMAL1 = GAMMAL1 + Q19 (GAMMAL-GAMMAL1), then RANGER. */
    private void huntest3(double gammal) {
        sd("GAMMAL", gammal);
        double g1 = d("GAMMAL1");
        sd("GAMMAL1", (gammal - g1) * Q19 + g1);
        ranger(gammal);
    }

    // ---------------- RANGE PREDICTION ----------------

    /** RANGER: range prediction, enter with GAMMAL. Sums ASKEP+ASP1+ASPUP+ASP3+ASPDWN and compares with THETAH. */
    private void ranger(double gammal) {
        double cosg2 = HALVE - gammal * gammal / 4;         // COSG/2 = 1-GAMMAL SQ/2 (truncated series)
        sd("COSG/2", cosg2);
        double vbars = d("VBARS");
        double e4 = Math.sqrt(Math.max(0, (HALVE - vbars) * cosg2 * cosg2 * 4 + ONE_SIXTEENTH));   // E/4 : E = SQRT(1+VBARS...)
        double askep = Math.asin(Math.max(-1, Math.min(1, ddv(vbars * cosg2 * gammal, e4)))) * 2 / (2 * Math.PI);   // ASKEP/2PI
        sd("ASKEP", askep);                                 // ballistic range
        double vl = d("VL");
        double asp1 = Q3 * vl + d("Q2");                    // ASP1 = Q2 + Q3 VL : final phase range
        sd("ASP1", asp1);
        double v1 = d("V1");
        double a0 = d("A0");
        double gam1 = d("GAMMAL1");
        double logArg = ddv(ddv(v1 * v1 * d("Q7"), vbars), a0);
        double minusLog = new CsmGeometry(agc).log(logArg); // return with -LOG in MPAC
        double aspup = C12 * minusLog / gam1;               // ASPUP = -C12 LOG(V1 Q7/VBARS A0)/GAMMAL1
        sd("ASPUP", aspup);                                 // up phase range
        double aspdwn = ddv(KC3 * d("RDOT") * d("V") / a0, d("LAD"));
        sd("ASPDWN", aspdwn);                               // range to pull out
        double asp3 = Q5 * (Q6 - gammal);                   // ASP3 = Q5(Q6-GAMMAL) : gamma correction
        sd("ASP3", asp3);
        sd("ASPS(TM)", askep);                              // save hi-word of ASP's for TM
        ovf = false;
        double asp = asp3 + aspdwn + aspup + asp1 + askep;
        double diff = asp - d("THETAH");                    // DIFF = (ASP-THETAH)/2PI
        if (Math.abs(asp) >= 1.0) {
            tcdanzig();
            return;
        }
        sd("DIFF", diff);
        if (Math.abs(diff) - NM25 < 0) {                    // within 25 NM: go to UP control
            gotoupsy();
            return;
        }
        if (!flag(HIND) && diff >= 0) {
            dconstd();                                      // eventually sets MODE = HUNTEST
            return;
        }
        getdlewd(diff);
    }

    /** GETDLEWD: DLEWD = DLEWD (DIFF/(DIFFOLD-DIFF)). */
    private void getdlewd(double diff) {
        double dlewd = ddv(d("DLEWD") * diff, d("DIFFOLD") - diff);
        lwdstore(dlewd, diff);
    }

    /** LWDSTORE: store DLEWD; if LEWD+DLEWD negative, DLEWD=-LEWD/2; overflow sets LEWD=NEARONE and CONSTD. */
    private void lwdstore(double dlewd, double diff) {
        sd("DLEWD", dlewd);
        double lewd = d("LEWD") + dlewd;
        if (lewd < 0) {
            lewdptr(diff);
            return;
        }
        if (Math.abs(lewd) >= 1.0) {
            lewdovfl();
            return;
        }
        sd("LEWD", lewd);
        sidetrak(dlewd, diff);
    }

    /** LEWDPTR: DLEWD = -LEWD/2, then store. */
    private void lewdptr(double diff) {
        lwdstore(-d("LEWD") / 2, diff);
    }

    /** LEWDOVFL: LEWD = NEARONE; go to DCONSTD (also sets MODE = HUNTEST). */
    private void lewdovfl() {
        sd("LEWD", NEARONE);
        dconstd();
    }

    /** SIDETRAK: sidetrack HUNTEST into group 4 so group 5 (servicer) finishes first. */
    private void sidetrak(double dlewd, double diff) {
        agc.set("EBANK", agc.get("EBENTRY"));
        agc.set("PHSPRDT5", agc.get("PRIO16"));             // drop GRP 5 restart prio to 1 less than GRP 4
        // TC PHASCHNG ; OCT 00474 : restart GRP 4 at PRE-HUNT
        // CA PRIO16 ; TC PRIOCHNG : continue GRP 5 at lower priority than GRP 4 or SERVICER
        gotoAddr(ENDEXIT);                                  // sidetrack next pass until this one done
        sd("DIFFOLD", diff);                                // DIFFOLD / 2 PI
        setFlag(HIND);
        sd("Q7", Q7F);                                      // Q7 / 805 FPSS
        huntest();
    }

    /** NEGAMA: part of HUNTEST; enter with GAMMAL in MPAC; returns GAMMAL (=0 after Q7 recalculation). */
    private double negama(double gammal) {
        double vl = d("VL");
        double lewd = d("LEWD");
        double dvl = d("DVL");
        double t1 = gammal * vl * ONE_THIRD;                // GAMMAL VL /3
        double t2 = lewd * ONE_THIRD;                       // LEWD/3
        double fac = (d("AHOOKDV") + 2.0 / 3) * dvl * CH1 / d("DHOOK") / vl;   // (2/3 + AHOOKDV)(CH1 GS/DHOOK VL)
        double delvl = t1 / (t2 - fac);                     // DEL VL = (GAMMAL VL/3)/(LEWD/3 - DVL (2/3+AHOOKDV)(CH1 GS/DHOOK VL))
        vl = vl + delvl;
        sd("VL", vl);                                       // VL/2 VS
        sd("Q7", dhookyq7(vl));                             // Q7=((1-VL/FACT1)SQ - ALP)/FACT2
        sd("VBARS", vl * vl);                               // VBARS / 4 VS VS
        return 0;                                           // set GAMMAL = 0
    }

    /** DHOOKYQ7: subroutine to calc DHOOK or Q7: ((1-X/FACT1)SQ - ALP)/FACT2. */
    private double dhookyq7(double x) {
        double t = HALVE - ddv(x / 2, d("FACT1"));          // (1 - X/FACT1) scaled
        t = t * 2;
        return ((t * t) - d("ALP")) / d("FACT2");
    }

    /** PRE-HUNT: restart entry in group 4 after HUNTEST was sidetracked. */
    public void preHunt() {
        clearFlag(HIND);
        forehunt();                                         // re-initialise HUNTEST after restart
        huntest();
    }

    /** FOREHUNT: initialise HUNTEST (DIFFOLD=0, DLEWD=DLEWD0, LEWD=LEWD1). */
    private void forehunt() {
        sd("DIFFOLD", 0);
        sd("DLEWD", DLEWD0);
        sd("LEWD", LEWD1);
    }

    /** ADENDEXT: CADR ENDEXIT. */
    public static final int ADENDEXT = ENDEXIT;

    // ---------------- UP CONTROL ----------------

    /** GOTOUPSY: end of HUNTEST. P65 disables HUNTEST group 4 use and sets MODE = UPCONTRL. */
    private void gotoupsy() {
        new P61P67(agc).p65();
        gotoAddr(UPCONTRL);
        upcontrl();
    }

    /** UPCONTRL: MM = 65. Roll control during the super-circular phase. */
    public void upcontrl() {
        double dd = d("D");
        if (dd - C21 >= 0) {
            setFlag(NOSWITCH);                              // suppress lateral switch
        }
        if (d("V") - d("V1") >= 0) {
            downcntl();
            return;
        }
        if (dd - d("Q7") < 0) {
            kep();                                          // IF D-Q7 NEG, GO TO KEP
            return;
        }
        if (d("RDOT") < 0) {
            // VLTEST: if V-VL-C18 negative, EGSW=1, MODE=PREDICT3
            if (d("V") - d("VL") - C18 < 0) {
                prefinal();
                return;
            }
        }
        cont1();
    }

    /** CONT1: if D-A0 positive, L/D = LAD and go to LIMITL/D. */
    private void cont1() {
        double dd = d("D");
        if (dd - d("A0") >= 0) {
            storeLD(d("LAD"));                              // STOREL/D
            return;
        }
        // CONT3: VREF=FACT1(1-SQRT(FACT2 D + ALP))
        double vref = d("FACT1") * (NEARONE - Math.sqrt(Math.max(0, dd * d("FACT2") + d("ALP"))));
        sd("VREF", vref);                                   // VREF / 2VS
        double rdotref = d("LEWD") * (d("V1") - vref);      // RDOTREF = LEWD(V1-VREF)
        if (d("VS1") - vref >= 0) {
            double dv = d("VS1") - vref;                    // DV = VS1 - VREF
            double rdhook = CH1 * (ONE_SIXTEENTH + dv * d("AHOOKDV") / d("DVL")) * dv * dv / d("DHOOK") / vref;
            rdotref = rdotref - rdhook;                     // RDOTREF = RDOTREF - RDHOOK
        }
        sd("RDOTREF", rdotref);
        // CONTINU2
        double factor = d("FACTOR");
        ovf = false;
        if (dd - Q7MIN >= 0) {
            factor = ddv(dd - d("Q7"), d("A1") - d("Q7"));  // FACTOR / 25G
            sd("FACTOR", factor);
        }
        upcntrl3(rdotref, vref, factor);
    }

    /** UPCNTRL3: DELTA L/D = -((RDOT-RDOTREF)F1/KB1 + V-VREF) F1/KB2, nonlinear gain reduction. */
    private void upcntrl3(double rdotref, double vref, double factor) {
        ovf = false;
        double dld = ((d("RDOT") - rdotref) * factor * ONE_OVER_KB1 + d("V") - vref) * factor * NEG_ONE_OVER_KB2;
        if (Math.abs(dld) >= 1.0) {
            gomaxLD(dld);
            return;
        }
        if (Math.abs(dld) - PT1_16 >= 0) {
            dld = sign((Math.abs(dld) - PT1_16) * POINT1 + PT1_16, dld);   // nonlinear circuit for reducing high gain
        }
        double ld = dld * 16 + d("LEWD");                   // NEXT1: SL4, plus LEWD
        negtests(ld);
    }

    /** NEGTESTS: L/D from MPAC; if D-C20 positive, LATSW=0 and if L/D negative, L/D = 0. */
    private void negtests(double ld) {
        if (Math.abs(ld) >= 1.0) {
            gomaxLD(ld);
            return;
        }
        sd("L/D", ld);
        if (d("D") - C20 < 0) {
            limitLD();
            return;
        }
        clearFlag(LATSW);                                   // roll over top, regardless
        if (ld < 0) {
            sd("L/D", 0);
        }
        limitLD();
    }

    /** DCONSTD: save old DIFF for next pass, Q7 = Q7F, then BECONSTD. */
    private void dconstd() {
        sd("DIFFOLD", d("DIFF"));                           // DIFFOLD / 2 PI
        sd("Q7", Q7F);
        beconstd();
    }

    /** BECONSTD: a HUNTEST entry into CONSTD; reset MODE to HUNTEST and deactivate group 4. */
    private void beconstd() {
        gotoAddr(HUNTEST);
        new P61P67(agc).killgrp4();
        constd();
    }

    /** CONSTD: constant drag controller. */
    public void constd() {
        if (ovf) {
            tcdanzig();                                     // BOVB TCDANZIG
            return;
        }
        double leq = d("LEQ");
        double pd1 = leq * d("C/D0");                       // LEQ C/D0 into PDL
        double rdotMinusRef = TWO_HS * d("D0") / d("V") + d("RDOT");   // RDOTREF = -2 HS D0/V
        double pd = rdotMinusRef * K2D + pd1;               // C/D0 LEQ + K2D(RDOT-RDOTREF)
        constd1(d("D0"), pd);                               // D0 /805 is DREF
    }

    /** CONSTD1: enter with DREF in MPAC; L/D = 256 (K1D (D-DREF) + K2D term). */
    private void constd1(double dref, double pd) {
        double ld = ((d("D") - dref) * K1D + pd) * 256;
        negtests(ld);
    }

    /** DOWNCNTL: initial part of UPCONTROL when V exceeds V1. */
    private void downcntl() {
        if (ovf) {
            tcdanzig();                                     // clear OVFIND if on
            return;
        }
        double lad = d("LAD");
        double v = d("V");
        double v1 = d("V1");
        double pdLad = lad / 256;                           // push up LAD
        double rdtrTerm = (v - v1) * lad + d("RDOT");       // RDOT - RDTR with RDTR = LAD(V1-V)
        double pd = rdtrTerm * K2D + pdLad;                 // LAD + K2D(RDOT-RDTR)
        double pd2 = (v1 - v) * (v1 - v) * lad / TWO_C1_HS; // (V1-V)SQ LAD/(2 C1 HS)
        double dref = d("A0") * d("VSQUARE") / (v1 * v1) - pd2;   // DREF = (V/V1)SQ A0 - PD
        constd1(dref, pd);
    }

    // ---------------- BALLISTIC PHASE ----------------

    /** KEP: MM = 66. Display trim gimbal angle values, set GOTOADDR to the Kepler phase. */
    private void kep() {
        new P61P67(agc).p66();
        gotoAddr(KEP2);
        kep2();
    }

    /**
     * KEP2: Kepler phase; monitor drag. Spacecraft trimmed in pitch and yaw to the computed
     * relative velocity; last computed roll angle maintained. Can also start from INITROLL in P64.
     */
    public void kep2() {
        if (Q7FKDMIN - d("D") < 0) {
            prefinal();
            return;
        }
        if (!flag(G05SW)) {
            sd("ROLLC", 0);                                 // set ROLLC & ROLLHOLD = 0
            sd("ROLLHOLD", 0);
        }
        new P61P67(agc).p623();                             // desired gimbal angles at present RN, VN (for N22)
    }

    // ---------------- FINAL PHASE ----------------

    /** PREFINAL: MM = 67. Restart protect GOTOADDR, P67, set EGSW; continue to PREDICT3. */
    public void prefinal() {
        gotoAddr(PREFINAL);
        new P61P67(agc).p67();                              // disables GRP4; displays ROLLC, XRNGERR, DNRNGERR
        setFlag(EGSW);
        gotoAddr(PREDICT3);
        predict3();
    }

    /**
     * PREDICT3: final sub-orbital phase. Roll commands cease when V is less than VQUIT. Uses the
     * VREFER table (13 points of velocity and six 13-point functions) interpolated linearly.
     */
    public void predict3() {
        double v = d("V");
        if (v - VQUIT < 0) {
            steeroff();
            return;
        }
        agc.set("EBANK", agc.get("EBENTRY"));               // precautionary
        int jj = 12;
        while (jj > 0 && VREFER[jj] - v > 0) {              // highest VREF at end of table
            jj--;
        }
        double grad = (v - VREFER[jj]) / (VREFER[jj + 1] - VREFER[jj]);   // GRAD = (V-VREF)/(VK+1 - VK)
        double[] fx = new double[6];
        for (int mm = 5; mm >= 0; mm--) {
            jj += 13;
            fx[mm] = VREFER[jj] + grad * (VREFER[jj + 1] - VREFER[jj]);   // FX = AK + GRAD (AK+1 - AK)
        }
        double f1 = fx[5];
        double f2 = fx[4];
        double rdotref = fx[3];
        double rtogo = fx[2];
        double dref = fx[1];
        double y = fx[0];
        double predang = rtogo + f2 * (rdotref - 8 * d("RDOT")) + f1 * (d("D") + dref);
        agc.set("PREDANG", Agc.fromSigned((int) Math.round(predang * 16384)));
        // L/D = LOD + (THETA- PREDANG)/ Y
        double diffAng = predang / 8 - d("THETAH");
        if (flag(GONEPAST) || goneby()) {
            if (!flag(GONEPAST)) {
                setFlag(GONEPAST);                          // set GONEPAST if GONEBY set & latch in-place
                sd("DNRNGERR", MAXRNG);                     // display = 9999.9 if GONEBY
            }
            goneglad();
            return;
        }
        sd("DNRNGERR", diffAng);                            // = (PREDANG - THETA)/360
        ovf = false;
        double q = ddv(-diffAng, y);                        // FX = DRANGE/D L/D = Y
        q = q * 32;
        if (Math.abs(q) >= 1.0) {
            gomaxLD(q);
            return;
        }
        q = q + d("LOD");
        if (Math.abs(q) >= 1.0) {
            gomaxLD(q);
            return;
        }
        sd("L/D", q);
        glimiter();
    }

    /** GONEGLAD: set L/D = -LAD (any negative number will do), falling into GOMAXL/D. */
    private void goneglad() {
        gomaxLD(-1.0);
    }

    /** GOMAXL/D: L/D = LAD SIGN(MPAC), then GLIMITER. */
    private void gomaxLD(double mpac) {
        sd("L/D", mpac < 0 ? -d("LAD") : d("LAD"));
        glimiter();
    }

    /** GLIMITER: limit L/D to avoid exceeding GMAX. */
    private void glimiter() {
        double dd = d("D");
        if (GMAX_2 - dd >= 0) {
            limitLD();
            return;
        }
        double gmaxMinusD = GMAX_2 + (GMAX_2 - dd);         // GMAX - D
        if (gmaxMinusD < 0) {
            goposlad();
            return;
        }
        double pd = TWO_HS * gmaxMinusD * (d("LEQ") * ONE_OVER_GMAX + d("LAD"));   // 2HS(GMAX-D)(LEQ/GMAX+LAD)
        double xlim = Math.sqrt(Math.max(0, pd + TWO_HS_GMXSQ / d("VSQUARE")));   // XLIM = SQRT(PD+(2HSGMAX/V)SQ)
        if (xlim + d("RDOT") >= 0) {
            limitLD();
            return;
        }
        goposlad();
    }

    /** GOPOSLAD: L/D = LAD. */
    private void goposlad() {
        storeLD(d("LAD"));
    }

    /** STOREL/D: store L/D, then LIMITL/D. */
    private void storeLD(double ld) {
        sd("L/D", ld);
        limitLD();
    }

    /** LIMITL/D: lateral control; computes ROLLC = ACOS(L/D1 / LAD) with sign from K2ROLL. */
    public void limitLD() {
        double ld1 = d("L/D");
        sd("L/D1", ld1);
        if (!flag(GONEPAST)) {                              // no lateral control if past target
            double y = d("KLAT") * d("VSQUARE") + LATBIAS;  // Y = KLAT VSQUARE + LATBIAS
            double latang = d("LATANG");
            double k2roll = d("K2ROLL");
            boolean skipSwitch = false;
            if (Math.abs(ld1) - d("L/DCMINR") >= 0) {       // L350: else go to L353
                if (sign(latang, k2roll) < 0) {
                    // L357: L/D1 = L/DCMINR SIGN(L/D)
                    ld1 = sign(d("L/DCMINR"), ld1);
                    sd("L/D1", ld1);
                    skipSwitch = true;
                } else {
                    y = y / 2;                              // Y = Y/2
                }
            }
            if (!skipSwitch && sign(latang, k2roll) - y >= 0) {   // L353: IF LATANG SIGN(K2ROLL)-Y POS, SWITCH
                if (flag(NOSWITCH)) {
                    clearFlag(NOSWITCH);                    // K2ROLL unchanged
                } else {
                    sd("K2ROLL", -k2roll);                  // K2ROLL = -K2ROLL
                }
            }
        }
        // L355: ROLLC = ACOS((L/D1)/LAD)
        ovf = false;
        double ratio = ddv(ld1, d("LAD"));                  // MPAC set to +-1 if overflow
        if (ovf) {
            ratio = ld1 < 0 ? -1 : 1;
        }
        sd("ROLLC", sign(acosRev(ratio), d("K2ROLL")));
        clearFlag(NOSWITCH);
        endexit();
    }

    /** ENDEXIT: EXIT to OVERNOUT; entry displays are done here. */
    public void endexit() {
        overnout();
    }

    /** OVERNOUT: ENTRYDSP test, display ENTRYVN via REGODSPR (no abort if DISKY in use). */
    public void overnout() {
        if ((agc.get("CM/FLAGS") & ENTRYDSP) != 0) {
            // CA ENTRYVN ; TC BANKCALL ; CADR REGODSPR
            new DisplayInterfaceRoutines(agc).regodspr(agc.get("ENTRYVN"));
        }
        // NODISKY: INHINT ; CCS NEWJOB : protect READACCS GRP 5 if sidetracked (TC CHANG1)
        if (agc.get("NEWJOB") != 0) {
            return;                                         // CHANG1: give up the processor
        }
        servnout();
    }

    /** SERVNOUT: POSTJUMP SERVEXIT, ends the average-G job (also from P67.3). */
    public void servnout() {
        new Servicer207(agc).servexit();
    }

    /** STEEROFF: display when V is less than VQUIT; starts P67.1 as a job and goes to P67.2. */
    private void steeroff() {
        agc.set("EBANK", agc.get("EBENTRY"));               // precautionary
        // CA PRIO16 (2 less than NTRYPRIO) ; NOVAC 2CADR P67.1 : RTOGO LAT LONG display job
        new P61P67(agc).p671();
        // TC 2PHSCHNG ; OCT 00414 ; OCT 10035 : 4.41 restart for P67.1, SERVICER 5.3 restart
        gotoAddr(P67_2);                                    // continue for LAT, LONG this time
        new P61P67(agc).p672();
    }

    /** TCDANZIG: overflow error handler (TIME_OF_FREE_FALL). */
    private void tcdanzig() {
        ovf = false;
    }
}
