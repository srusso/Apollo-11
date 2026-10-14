package apollo.luminary099;

import apollo.agc.Agc;

/**
 * LUNAR_LANDING_GUIDANCE_EQUATIONS.agc (Luminary 099, F2DPS): lunar landing guidance for P63
 * (braking), P64 (approach), P65/P66/P67 (vertical), and the ignition-algorithm sub-entry.
 * Contains the flight sequence tables, LUNLAND / GUILDENSTERN (R13 auto-modes monitor), TTFINCR,
 * landing-site redesignation, RGVGCALC, the TTF/8 computation (ROOTPSRS Newton root finder),
 * QUADGUID, CGCALC, the exit logic, guidance displays, P65/P66 (ROD) guidance, the redesignator
 * trap (PITFALL / REDESMON), the R.O.D. trap and several small helper subroutines.
 *
 * Interpretive code is rendered with doubles holding the stored (scaled) fraction of each
 * double-precision erasable word; pushdown-list temporaries are local variables. WCHPHASE
 * selects the flight phase: -1 IGNALG, 0 BRAKQUAD, 1 APPRQUAD, 2 VERTICAL.
 */
public class LunarLandingGuidanceEquations {
    // ---- WCHPHASE values ----
    public static final int IGNALG = -1;
    public static final int BRAKQUAD = 0;
    public static final int APPRQUAD = 1;
    public static final int VERTICAL = 2;

    /** TARGTDEX: indices for referencing target parameters (IGNALG, BRAKQUAD, APPRQUAD). */
    public static final int[] TARGTDEX = {0, 0, 034};

    // ---- constants ----
    public static final int TTFSCALE = 010000;       // BIT12
    public static final int TSCALINV = 010;          // BIT4
    public static final int NEG_DEC103 = -103;
    public static final int POS_DEC99 = 99;
    public static final double TREDESCL = -.08;
    public static final int DEGS180 = 180;
    public static final double HALF_DEG = .00278;
    public static final double PROJMAX = .42262 / 8;   // SIN(25')/8 to compare with PROJ
    public static final double PROJMIN = .25882 / 8;   // SIN(15')/8 to compare with PROJ
    public static final int V06N63 = 0663;            // P63
    public static final int V06N64 = 0664;            // P64
    public static final int V06N60 = 0660;            // P65, P66, P67
    public static final double HIGHESTF = 4.34546769 / 4096;
    public static final double GSCALE = 100.0 / 2048;
    public static final double D3_8 = .375;
    public static final double D3_4 = .750;
    public static final double DEPRCRIT = -.02 / 2;
    public static final int BIT1H = 1;
    public static final double SHFTFACT = 1.0 / (1 << 17);
    public static final double BIASFACT = 655.36 / (1L << 28);
    /** Redesignator trap bit equivalences (GSOP chapter 4, revision 16 of P64LM). */
    public static final int PLUS_ELBIT = 02;     // BIT2: -PITCH
    public static final int MINUS_ELBIT = 01;    // BIT1: +PITCH
    public static final int PLUS_AZBIT = 020;    // BIT5
    public static final int MINUS_AZBIT = 040;   // BIT6
    public static final int ALL4BITS = 063;
    public static final double AZEACH = .03491;  // 2 degrees
    public static final double ELEACH = .00873;  // 1/2 degree

    private static final int BIT4 = 010;
    private static final int BIT5 = 020;
    private static final int BIT12 = 04000;
    private static final int BIT13 = 010000;

    private final Agc agc;
    private boolean ovf;

    public LunarLandingGuidanceEquations(Agc agc) {
        this.agc = agc;
    }

    // ---------------- helpers ----------------

    private int gs(String s) {
        return Agc.toSigned(agc.get(s));
    }

    private void ss(String s, int v) {
        agc.set(s, Agc.fromSigned(v));
    }

    private double d(String name) {
        long v = ((long) Agc.toSigned(agc.get(name)) << 14) + Agc.toSigned(agc.get(name + "+1"));
        return v / (double) (1L << 28);
    }

    private void sd(String name, double x) {
        long v = Math.round(x * (1L << 28));
        int sign = v < 0 ? -1 : 1;
        long mag = Math.abs(v);
        ss(name, sign * (int) ((mag >> 14) & 0x3FFF));
        ss(name + "+1", sign * (int) (mag & 0x3FFF));
    }

    private double[] vec(String name) {
        return new double[] {d(name), d(name + "+2"), d(name + "+4")};
    }

    private void svec(String name, double[] v) {
        sd(name, v[0]);
        sd(name + "+2", v[1]);
        sd(name + "+4", v[2]);
    }

    /** CG matrix rows are at CG, CG+6 and CG+14 as addressed in the source listing. */
    private static final String[] CG_ROWS = {"CG", "CG+6", "CG+14"};

    private double[] cgRow(int i) {
        return vec(CG_ROWS[i]);
    }

    private double[] mxv(double[] v) {
        return new double[] {dot(cgRow(0), v), dot(cgRow(1), v), dot(cgRow(2), v)};
    }

    private double[] vxm(double[] v) {
        double[] r0 = cgRow(0), r1 = cgRow(1), r2 = cgRow(2);
        return new double[] {
            v[0] * r0[0] + v[1] * r1[0] + v[2] * r2[0],
            v[0] * r0[1] + v[1] * r1[1] + v[2] * r2[1],
            v[0] * r0[2] + v[1] * r1[2] + v[2] * r2[2]};
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

    private static double[] vxsc(double[] a, double s) {
        return new double[] {a[0] * s, a[1] * s, a[2] * s};
    }

    private static double[] vad(double[] a, double[] b) {
        return new double[] {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
    }

    private static double[] vsu(double[] a, double[] b) {
        return new double[] {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    private double chk(double x) {
        if (Math.abs(x) >= 1.0) {
            ovf = true;
            return x < 0 ? -0.999999999 : 0.999999999;
        }
        return x;
    }

    private boolean flag(String word, String bit) {
        return (agc.get(word) & agc.get(bit)) != 0;
    }

    private void upflag(String name) {
        agc.set("FLAGS+" + name, 1);                 // TC UPFLAG ; ADRES flag
    }

    private void downflag(String name) {
        agc.set("FLAGS+" + name, 0);                 // TC DOWNFLAG ; ADRES flag
    }

    private int wch() {
        return gs("WCHPHASE");
    }

    private boolean ch31Bit13Clear() {
        // EXTEND ; RAND CHAN31 with BIT13: zero (attitude hold present) when bit is 0 (inverted discrete)
        return (agc.get("CHAN31") & BIT13) == 0;
    }

    // ---------------- FLIGHT SEQUENCE TABLES (indexed by WCHPHASE) ----------------

    /** NEWPHASE: routines for starting new guidance phases. */
    private void newPhase() {
        switch (wch()) {
            case IGNALG: ttfincr(); break;
            case BRAKQUAD: ttfincr(); break;
            case APPRQUAD: startp64(); break;
            default: p65start(); break;
        }
    }

    /** PREGUIDE: pre-guidance computations. */
    private void preGuide() {
        switch (wch()) {
            case IGNALG: calcrgvg(); break;
            case APPRQUAD: redesig(); break;
            default: rgvgcalc(); break;
        }
    }

    /** WHATGUID: guidance equations. */
    private void whatGuid() {
        if (wch() == VERTICAL) {
            vertguid();
        } else {
            ttf8Cl();
        }
    }

    /** AFTRGUID: post guidance equation computations. */
    private void aftrGuid() {
        if (wch() == VERTICAL) {
            steerQ();
        } else {
            cgcalc();
        }
    }

    /** WHATEXIT: window vector computations (index may be WCHPHOLD). */
    private void whatExit(int phase) {
        switch (phase) {
            case IGNALG: exgsub(); break;
            case BRAKQUAD: exbrak(); break;
            default: exnorm(); break;
        }
    }

    /** WHATDISP: display routines. */
    private void whatDisp() {
        switch (gs("WCHPHOLD")) {
            case BRAKQUAD: p63disps(); break;
            case APPRQUAD: p64disps(); break;
            default: vertdisp(); break;
        }
    }

    /** WHATALM: alarm routine for TTF computation. */
    private void whatAlm() {
        if (wch() == IGNALG) {
            p1406p00();
        } else {
            p1406alm();
        }
    }

    // ---------------- ENTRY POINTS ----------------

    /** ?GUIDSUB: ignition algorithm entry, delivers N passes of quadratic guidance (N = 3). */
    public void queryGuidsub() {
        ss("NGUIDSUB", 2);
        guildretPlus2();
    }

    /** GUIDSUB: on succeeding passes skip TTFINCR. */
    public void guidsub(int a) {
        ss("NGUIDSUB", a);
        calcrgvg();
    }

    /** LUNLAND: normal entry, control comes here from SERVOUT. */
    public void lunland() {
        // PHASCHNG 00035 : group 5, retain only PIPA task
        // PHASCHNG 05023 / 21000 : group 3, protect guidance with prio 21 just higher than SERVICER
        guilden();
    }

    // ---------------- GUILDENSTERN: AUTO-MODES MONITOR (R13) ----------------

    /**
     * GUILDEN: on every appearance or disappearance of the manual throttle discrete select P67 or
     * P66 respectively; on every appearance of the attitude-hold discrete select P66 (unless the
     * current program is P67, in which case there is no change).
     */
    public void guilden() {
        if ((agc.get("CHAN30") & BIT5) != 0) {         // is un-auto-throttle discrete present?
            startp67();
            return;
        }
        if (gs("MODREG") != 67) {                      // P67NOW?: CHECKMM 67
            stabl();
            return;
        }
        startp66();
    }

    /** STARTP66: FASTCHNG, NEWMODEX 66; desired altitude rate = current altitude rate. */
    public void startp66() {
        fastchng();
        agc.set("MODREG", 66);                         // NEWMODEX DEC66
        sd("VDGVERT", d("HDOTDISP"));
        strtp66a();
    }

    /** STRTP66A: initialise ROD bias terms and old PIPA values. */
    public void strtp66a() {
        double[] bias = {gs("PBIASX") / 16384.0, gs("PBIASY") / 16384.0, gs("PBIASZ") / 16384.0};
        upflag("RODFLAG");
        svec("VBIAS", vxsc(bias, BIASFACT));
        double[] temp = {gs("TEMX"), gs("TEMY"), gs("TEMZ")};
        ss("OLDPIPAX", -(int) temp[0]);
        ss("OLDPIPAY", -(int) temp[1]);
        ss("OLDPIPAZ", -(int) temp[2]);
        svec("DELVROD", new double[] {0, 0, 0});
        sd("RODSCAL1", d("RODSCALE"));
        sd("LASTTPIP", d("PIPTIME"));
        ss("FCOLD", 0);
        ss("FWEIGHT", 0);
        ss("FWEIGHT+1", 0);
        vrtstart(0);
    }

    /** VRTSTART: WCHPHASE = 2 ---> VERTICAL: P65, P66, P67. */
    private void vrtstart(int wchvert) {
        ss("WCHVERT", wchvert);
        ss("WCHPHOLD", VERTICAL);
        ss("WCHPHASE", VERTICAL);
        // BANKCALL STOPRATE (temporary)
        downflag("XOVINFLG");                          // permit X-axis override
        downflag("REDFLAG");
        vertguid();
    }

    /** STARTP67: no harm in "starting" P67 over and over; RODCOUNT = 0, WCHVERT = 10. */
    public void startp67() {
        agc.set("MODREG", 67);                         // NEWMODEX DEC 67
        ss("RODCOUNT", 0);
        vrtstart(10);
    }

    /** STABL?: is the un-attitude-hold discrete present? */
    private void stabl() {
        if (!ch31Bit13Clear()) {
            guildret();                                 // yes, all's well
            return;
        }
        // P66NOW?
        if (gs("MODREG") == 66) {
            restartQ();
            return;
        }
        if (gs("RODCOUNT") == 0) {
            guildret();                                 // continue with automatic landing
            return;
        }
        startp66();                                     // ROD switch clicked: switch into ROD mode
    }

    /** RESTART?: has there been a restart? */
    private void restartQ() {
        if (!flag("FLAGWRD1", "RODFLBIT")) {
            strtp66a();                                 // yes: reinitialise but leave VDGVERT as is
            return;
        }
        vertguid();                                     // no: continue with R.O.D.
    }

    // ---------------- INITIALIZATION FOR THIS PASS ----------------

    /** GUILDRET: clear RODCOUNT, then GUILDRET+2. */
    public void guildret() {
        ss("RODCOUNT", 0);
        guildretPlus2();
    }

    /** GUILDRET+2: save TPIP as TPIPOLD, TPIP = PIPTIME1, TTF/8 saved; TTFINCR on first pass else new phase. */
    private void guildretPlus2() {
        sd("TPIPOLD", d("TPIP"));
        fastchng();
        sd("TPIP", d("PIPTIME1"));
        sd("TTF/8TMP", d("TTF/8"));
        if (gs("FLPASS0") != 0) {                       // CCS FLPASS0 : nonzero ---> TTFINCR
            ttfincr();
            return;
        }
        newPhase();                                     // BRSPOT1: INDEX WCHPHASE ; TCF NEWPHASE
    }

    // ---------------- ROUTINES TO START NEW PHASES ----------------

    /** P65START: NEWMODEX 65, WCHVERT = -2, permit X-axis override. */
    private void p65start() {
        agc.set("MODREG", 65);
        ss("WCHVERT", -2);
        downflag("XOVINFLG");
        ttfincr();
    }

    /** STARTP64: NEWMODEX 64, augment TTF/8, enable RUPT10, initialise redesignation flag. */
    private void startp64() {
        agc.set("MODREG", 64);
        ss("TTF/8TMP", gs("TTF/8TMP") + gs("DELTTFAP"));   // augment TTF/8
        agc.set("CHAN13", agc.get("CHAN13") | BIT12);       // enable RUPT10
        downflag("REDFLAG");                                // initialise redesignation flag
        ttfincr();
    }

    // ---------------- TTFINCR ----------------

    /**
     * TTFINCR: TTF/8 = TTF/8 + (TPIP - TPIPOLD)/8; landing site vector updated for lunar rotation
     * LAND = |LAND| UNIT(LAND - LAND(TPIP - TPIPOLD) x WM); slant range to site for display.
     */
    public void ttfincr() {
        double dt = d("TPIP") - d("TPIPOLD");
        dt = dt * 2048;                                  // SLR 11: scales delta time to 2(17) csecs
        double[] land = vec("LAND");
        double[] w = vsu(land, cross(vxsc(land, dt), vec("WM")));
        double[] landtemp = vxsc(unit(w), d("/LAND/") * 2);   // VXSC /LAND/ ; VSL1
        svec("LANDTEMP", landtemp);
        sd("TTF/8TMP", d("TTF/8TMP") + dt / 8);          // incremented TTF/8 in TTF/8TMP
        fastchng();
        sd("TTF/8", d("TTF/8TMP"));
        svec("LAND", vec("LANDTEMP"));
        tdispset();
        fastchng();                                      // since REDESIG may change LANDTEMP
        preGuide();                                      // BRSPOT2
    }

    // ---------------- LANDING SITE PERTURBATION EQUATIONS ----------------

    /** REDESIG: apply pending elevation/azimuth redesignation increments to LAND. */
    private void redesig() {
        if (!flag("FLAGWRD6", "REDFLBIT") || gs("TREDES") == 0) {
            rgvgcalc();                                  // skip redesignation logic
            return;
        }
        // INHINT
        ss("ELINCR", gs("ELINCR1"));
        ss("AZINCR", gs("AZINCR1"));
        fastchng();
        ss("ELINCR1", 0);
        ss("AZINCR1", 0);
        ss("ELINCR+1", 0);
        ss("AZINCR+1", 0);
        double[] land = vec("LAND");
        double[] r = vec("R");
        double[] ul = unit(vsu(land, r));                // unit (LAND - R)
        double[] yn = vec("YNBPIP");
        double[] elTerm = vxsc(cross(yn, ul), -gs("ELINCR") / 16384.0 * 2);   // - ELINCR (YNB * UNIT(LAND - R))
        double[] sum = vad(elTerm, vxsc(yn, gs("AZINCR") / 16384.0));       // + AZINCR YNB
        // resulting vector is 1/2 real size; make sure redesignation is not too close to the horizon
        double[] pd = sum;
        double comp = pd[0];
        if (comp - DEPRCRIT < 0) {
            comp = DEPRCRIT;
        }
        pd = new double[] {comp, pd[1], pd[2]};
        double ratio = (d("LAND") - d("R")) / comp;
        double[] nl = vxsc(unit(vad(vxsc(pd, ratio), r)), d("/LAND/") * 2);
        svec("LANDTEMP", nl);
        fastchng();
        svec("LAND", vec("LANDTEMP"));
        rgvgcalc();                                      // LOOKANGL will be computed at RGVGCALC
    }

    // ---------------- STATE IN GUIDANCE COORDINATES ----------------

    /**
     * CALCRGVG: in IGNALG, compute V from the integration output and trim correction term
     * (VATT1 REFSMMAT + UNFC/2), then RGVGCALC.
     */
    public void calcrgvg() {
        double[] v = vec("VATT1");
        double[] mv = {dot(vec("REFSMMAT"), v), dot(vec("REFSMMAT+6"), v), dot(vec("REFSMMAT+12"), v)};
        svec("V", vad(vxsc(mv, 0.5), vec("UNFC/2")));
        rgvgcalc();
    }

    /**
     * RGVGCALC: ANGTERM = V + R x WM (units 2^9 M/CS); VGU = CG ANGTERM; RGU = CG (R - LAND);
     * VHORIZ = 8 ABVAL(0, VGU2, VGU1); LOOKANGL = ARCSIN(UNIT(R - LAND) . XNBPIP).
     */
    public void rgvgcalc() {
        double[] r = vec("R");
        double[] angterm = vad(vxsc(cross(r, vec("WM")), 0.25), vec("V"));
        svec("ANGTERM", angterm);
        double[] vgu = mxv(angterm);
        svec("VGU", vgu);
        sd("VHORIZ", 8 * Math.sqrt(vgu[1] * vgu[1] + vgu[0] * vgu[0]));   // for display during P65
        double[] rl = vsu(r, vec("LAND"));
        double[] rgu = vxsc(mxv(rl), 2);
        svec("RGU", rgu);
        sd("RANGEDSP", abval(rgu));
        double sinLook = dot(unit(rl), vec("XNBPIP")) / 4;     // SINE(LOOKANGL)/4
        // CA FIXLOC ; TS PUSHLOC : reset pushdown pointer
        double look = Math.asin(Math.max(-1, Math.min(1, sinLook * 4))) / (2 * Math.PI);   // SPARCSIN
        ss("LOOKANGL", (int) Math.round((look + HALF_DEG) * DEGS180 * 16384 / 180));
        whatGuid();                                      // BRSPOT3
    }

    // ---------------- TTF/8 COMPUTATION ----------------

    /**
     * TTF/8CL: loads the cubic in TTF (A(0..3)) into TABLTTF and solves with ROOTPSRS:
     * A(3)=8 JDG2TTF, A(2)=6 ADG2TTF, A(1)=(6 VGU2 + 18 VDG2TTF)/8, A(0)=-24(RGU2-RDG2)/64.
     */
    public void ttf8Cl() {
        intpretx();
        int x1 = gs("X1");
        double[] a = new double[4];
        a[3] = d("JBRFG" + idx(x1));                     // JDG2TTF,1
        a[2] = d("ABRFG" + idx(x1));                     // ADG2TTF,1
        a[1] = d("VGU+4") * D3_4 + d("VBRFG" + idx(x1)); // (6 VGU2 + 18 VDG2TTF)/8
        a[0] = (d("RBRFG+4" + idx(x1)) - d("RGU+4")) * D3_8;   // -24 (RGU2 - RDG2)/64
        for (int i = 0; i < 4; i++) {
            sd("TABLTTF" + (i == 0 ? "" : "+" + (2 * i)), a[i]);
        }
        ss("TABLTTF+10", 0200);                          // BIT8: fractional precision for TTF to table
        double[] result = rootpsrs(a, d("TTF/8"));       // yields TTF/8 in MPAC
        if (result == null) {
            whatAlm();                                   // 1406 alarm (failed to converge)
            return;
        }
        sd("TTF/8", result[0]);                          // corrected TTF/8
        tdispset();
        quadguid();
    }

    private static String idx(int x1) {
        return x1 == 0 ? "" : "+" + x1;
    }

    /**
     * QUADGUID: ACG = ADG + 3/4 (1/4 (RDG - RG)/TTF/8 + VDG + VG) / TTF/8, with lag compensation
     * through LEADTIME: ratio = (TTF+LEADTIME)/TTF, coefficients ratio^2-ratio etc.
     */
    public void quadguid() {
        double ttf8 = d("TTF/8");
        double lead = gs("LEADTIME") / 16384.0;
        // safeguard: force -TTF*LEADTIME >= 0
        double ratio = Math.max(0, -(ttf8 + lead));      // LEADTIME is negative
        ratio = -ratio / ttf8;                           // - ratio of lag-diminished TTF to TTF
        double sq = ratio * ratio - ratio;               // RATIO SQUARED - RATIO
        double cVgu = sq + ratio;                        // coefficient for VGU term (MPAC)
        double cRdg = cVgu + sq;                         // 26D: coefficient for RDG-RGU term
        double cVdg = cRdg + sq;                         // 28D: coefficient for VDG term
        double cAdg = ratio + sq + sq + 1;               // 30D: coefficient for ADG term
        ss("MODE", 0);
        intpretx();
        int x1 = gs("X1");
        double[] vgu = vec("VGU");
        double[] vdg = vec("VBRFG" + idx(x1));
        double[] rdg = vec("RBRFG" + idx(x1));
        double[] adg = vec("ABRFG" + idx(x1));
        double[] rgu = vec("RGU");
        double[] t = vad(vxsc(vgu, cVgu), vxsc(vdg, cVdg));
        double[] pos = vxsc(vsu(rdg, rgu), 1.0 / ttf8);   // (RDG - RGU)/TTF/8
        double[] sum = vad(t, vxsc(pos, cRdg / 4));
        sum = vxsc(sum, D3_4 / ttf8);                    // V/SC TTF/8 ; VXSC 3/4
        double[] acg = vad(sum, vxsc(adg, cAdg));
        afccalc1(acg);
    }

    /** AFCCALC1: VERGUID comes here; UNFC/2 = CG ACG*2 - GDT/2 / GSCALE; then AFCCALC2. */
    public void afccalc1(double[] acg) {
        double[] unfc = vxm(acg);
        unfc = vxsc(unfc, 2);
        unfc = vsu(unfc, vxsc(vec("GDT/2"), 1.0 / GSCALE));   // BVSU STADR: subtract gravity term
        svec("UNFC/2", unfc);                             // UNFC/2 need not be unitized
        afccalc2(abval(unfc));
    }

    /** AFCCALC2: |AFC| for throttle; limit horizontal component to the maximum thrust capability. */
    private void afccalc2(double afc) {
        sd("/AFC/", afc);                                // magnitude of AFC for throttle
        double[] u = vec("UNFC/2");
        double vertical = u[0];
        double outOfPlane = u[1];
        double atotal = HIGHESTF / d("MASS");
        double amax = atotal * atotal - vertical * vertical - outOfPlane * outOfPlane;   // AMAXHORIZ
        if (amax < 0) {
            amax = 0;                                    // AFCCALC3: ZEROVECS
        }
        double horiz = Math.sqrt(amax) + u[2];
        if (horiz >= 0) {
            sd("UNFC/2+4", u[2]);                        // AFCCLEND: leave as is
        } else {
            sd("UNFC/2+4", -horiz);                      // BDSU UNFC/2+4 : limit horizontal component
        }
        afcclend();
    }

    private void afcclend() {
        fastchng();
        ss("WCHPHOLD", wch());                           // prepare for phase switching logic
        ss("FLPASS0", gs("FLPASS0") + 1);                // increment pass counter
        aftrGuid();                                      // BRSPOT4
    }

    // ---------------- GUIDANCE-STABLE MEMBER TRANSFORMATION ----------------

    /**
     * CGCALC: erect CG, rows UNIT(LAND), (LAND x (R-LAND) type) per the GAINBRAK term; when
     * TTF/8 plus TCGFBRAK/TCGFAPPR is not positive, skip (EXTLOGIC).
     */
    public void cgcalc() {
        // CAF EBANK5 ; DCA TCGFBRAK indexed by WCHPHASE/TARGTDEX : time constants for CG update
        int phase = Math.max(0, wch());
        int tcgf = gs("TCGFBRAK+" + (phase == APPRQUAD ? "34" : "0"));
        double ttf8 = d("TTF/8");
        if (ttf8 + tcgf / 16384.0 <= 0 || ttf8 <= 0) {
            extlogic();                                  // TCF EXTLOGIC (non-positive)
            return;
        }
        intpretx();
        int x1 = gs("X1");
        double[] land = vec("LAND");
        double[] row1 = unit(land);                      // first row, CG
        svec("CG", row1);
        double gain = d("GAINBRAK" + idx(x1)) * ttf8;    // TTF/8 * GAINBRAK,1 (numero misterioso)
        double[] w = vad(vxsc(vec("ANGTERM"), gain), land);
        double[] row2 = cross(land, unit(vsu(w, vec("R"))));   // second row
        svec("CG+6", unit(row2));
        double[] row3 = vxsc(cross(row2, row1), 2);      // VXV VSL1
        svec("CG+14", unit(row3));
        extlogic();
    }

    // ---------------- PREPARE TO EXIT ----------------

    /** EXTLOGIC: decide (1) how to exit and (2) whether to switch phases. */
    public void extlogic() {
        int wchphase = wch();
        int tend = gs("TENDBRAK" + (wchphase == APPRQUAD ? "+1" : ""));
        if (tend + gs("TTF/8") <= 0 || wchphase < 0) {   // BZMF WHATEXIT
            whatExit(wchphase);
            return;
        }
        fastchng();
        ss("WCHPHASE", gs("WCHPHOLD") + 1);
        ss("FLPASS0", 0);                                // reset FLPASS0
        whatExit(gs("WCHPHOLD"));
    }

    // ---------------- EXITS ----------------

    /** EXGSUB: return when GUIDSUB is called by the ignition algorithm; computes trim velocity correction. */
    public void exgsub() {
        double[] u = unit(vec("UNFC/2"));
        double zoom = d("ZOOMTIME");
        double[] trim = vxsc(vxsc(u, zoom), d("TRIMACCL"));
        svec("UNFC/2", trim);
        if (gs("NGUIDSUB") > 0) {                        // CCS NGUIDSUB
            guidsub(gs("NGUIDSUB") - 1);
            return;
        }
        if (gs("NIGNLOOP") > 0) {                        // CCS NIGNLOOP
            ddumcalc();                                  // POSTJUMP DDUMCALC
            return;
        }
        alarm(01412);
        ddumcalc();
    }

    /** DDUMCALC (external, ignition algorithm). */
    private void ddumcalc() {
        // POSTJUMP CADR DDUMCALC
    }

    /** EXBRAK: exit used during braking; UNIT(R) is the window pointing vector. */
    public void exbrak() {
        sd("UNWC/2", 0);
        svec("UNWC/2", vec("UNIT/R/"));
        steerQ();
    }

    /**
     * EXNORM: exit at other times; UNWC = PROJ-weighted combination of UNIT(LAND - R) and CG+14 so
     * that the window vector stays within the 15..25 degree window limits.
     */
    public void exnorm() {
        double[] u = unit(vsu(vec("LAND"), vec("R")));
        svec("UNWC/2", u);                               // UNIT(LAND - R) is the tentative choice
        double proj = dot(cross(vec("XNBPIP"), u), cgRow(1));   // PROJ, 1/8 real size
        double c1 = Math.max(0, PROJMAX - proj);         // coefficient for CG+14: 0 if PROJMAX - PROJ negative
        double c2 = Math.max(0, proj - PROJMIN);         // coefficient for UNIT(LAND - R): 0 if PROJ - PROJMIN negative
        double[] cg3 = vec("CG+14");
        double[] unwc = new double[3];
        for (int i = 0; i < 3; i++) {
            unwc[i] = cg3[i] * c1 + u[i] * c2;
        }
        svec("UNWC/2", unwc);
        steerQ();
    }

    /** STEER?: if STEERSW is down there are no outputs (RATESTOP); else EXVERT. */
    public void steerQ() {
        if (!flag("FLAGWRD2", "STEERBIT")) {
            ratestop();
            return;
        }
        exvert();
    }

    /** EXVERT: if overflow anywhere in guidance don't call THROTTLE or FINDCDUW. */
    public void exvert() {
        if (agc.get("OVFIND") != 0) {
            exovflow();
            return;
        }
        gdump1();
    }

    /** EXOVFLOW: sound the alarm non-abortively (01410), then RATESTOP. */
    public void exovflow() {
        alarm(01410);
        ratestop();
    }

    /** RATESTOP: if not in attitude hold, do a STOPRATE; then DISPEXIT. */
    public void ratestop() {
        if (!ch31Bit13Clear()) {                          // not in attitude hold
            // BANKCALL STOPRATE
            new DapInterfaceSubroutines(agc).stoprate();
        }
        dispexit();
    }

    /** GDUMP1: THROTTLE then FINDCDUW; continue to DISPEXIT. */
    public void gdump1() {
        new ThrottleControlRoutines(agc).throttle();
        new FindcduwGuidapInterface(agc).findcduwMinus2(vec("UNFC/2"));   // CALL FINDCDUW -2
        dispexit();
    }

    private void alarm(int code) {
        new AlarmAndAbort(agc).alarm(code, 0);
    }

    // ---------------- GUIDANCE LOOP DISPLAYS ----------------

    /** DISPEXIT: kill group 3 (restored by next guidance cycle); skip display if FLUNDISP set. */
    public void dispexit() {
        agc.set("-PHASE3", Agc.WORD_MASK);                // DCA NEG0 ; DXCH -PHASE3
        dispexitPlus3();
    }

    /** DISPEXIT+3: no display this pass if FLUNDISP is set; else the phase's display. */
    public void dispexitPlus3() {
        if (flag("FLAGWRD8", "FLUNDBIT")) {
            endlljob();
            return;
        }
        whatDisp();
    }

    /** DISPEXIT-2: PHASCHNG 00035 (kill group 5), then P63DISPS. */
    public void dispexitMinus2() {
        p63disps();
    }

    /** P63DISPS: V06N63 display. */
    private void p63disps() {
        dispcomn(V06N63);
    }

    /** DISPCOMN: BANKCALL REGODSPR, then end job. */
    private void dispcomn(int vn) {
        new DisplayInterfaceRoutines(agc).regoDspR(vn);
        endlljob();
    }

    /** ENDLLJOB: ENDOFJOB. */
    private void endlljob() {
        new Executive(agc).endOfJob();
    }

    /** P64DISPS: P64 display; flashing display permits redesignations. */
    private void p64disps() {
        if (gs("TREDES") == 0) {
            redOver();                                    // TREDES reached zero: clear redesignation flag
            return;
        }
        if (flag("FLAGWRD6", "REDFLBIT")) {
            redesOk();                                    // static display
            return;
        }
        // CAF V06N64 ; BANKCALL REFLASHR : terminate / proceed / recycle / (no response -> end)
        new DisplayInterfaceRoutines(agc).reflashR(V06N64);
        int response = gs("REFLASHRC");                   // return slot taken: 1 terminate, 2 proceed, 3 recycle
        if (response == 1) {
            new Executive(agc).endOfJob();                // GOTOPOOH : terminate
        } else if (response == 2) {
            p64ceed();                                    // PROCEED: permit redesignations
        } else if (response == 3) {
            p64disps();                                   // RECYCLE
        } else {
            endlljob();
        }
    }

    /** P64CEED: zero the increments and enable redesignation logic. */
    private void p64ceed() {
        ss("ELINCR1", 0);
        ss("AZINCR1", 0);
        upflag("REDFLAG");
        endlljob();
    }

    /** RED-OVER: clear REDFLAG; fall into REDES-OK. */
    private void redOver() {
        downflag("REDFLAG");
        redesOk();
    }

    /** REDES-OK: static V06N64 display. */
    private void redesOk() {
        dispcomn(V06N64);
    }

    /** VERTDISP: V06N60 display for P65, P66, P67. */
    private void vertdisp() {
        dispcomn(V06N60);
    }

    // ---------------- GUIDANCE FOR P65 ----------------

    /** VERTGUID: positive WCHVERT ---> P67, zero ---> P66, negative ---> P65. */
    public void vertguid() {
        int w = gs("WCHVERT");
        if (w > 0) {
            p67vert();
        } else if (w == 0) {
            p66vert();
        } else {
            p65vert();
        }
    }

    /** P65VERT: ACG = (V2FG - VGU)/TAUVERT. */
    public void p65vert() {
        double[] acg = vxsc(vsu(vec("V2FG"), vec("VGU")), 1.0 / Math.max(d("TAUVERT"), 1e-12));
        afccalc1(acg);
    }

    // ---------------- GUIDANCE FOR P66 ----------------

    /** P66VERT: POSTJUMP P66VERTA. */
    public void p66vert() {
        p66verta();
    }

    /** P67VERT: terminate group 3; VHORCOMP on the current velocity. */
    public void p67vert() {
        // PHASCHNG 00003 : terminate group 3
        vhorcomp(vec("V"));
    }

    /** RODTASK: FINDVAC RODCOMP at priority 22. */
    public void rodtask() {
        // CAF PRIO22 ; FINDVAC 2CADR RODCOMP ; TASKOVER
        rodcomp();
    }

    /** P66VERTA: terminate group 3, schedule RODTASK in 1 second (TWIDDLE). */
    public void p66verta() {
        // PHASCHNG 00003 ; CAF 1SEC ; TWIDDLE ADRES RODTASK
    }

    /**
     * RODCOMP: rate-of-descent computation. Updates desired altitude rate VDGVERT from RODCOUNT
     * clicks, reads PIPA snapshots, integrates the velocity, updates HDOTDISP / HCALC1 for noun 63
     * and the commanded acceleration /AFC/ with force limiting, then calls THROTTLE.
     */
    public void rodcomp() {
        // INHINT
        int rodcount = gs("RODCOUNT");
        ss("RODCOUNT", 0);
        sd("VDGVERT", d("VDGVERT") + rodcount * d("RODSCAL1"));   // update desired altitude rate
        String[] ax = {"X", "Y", "Z"};
        int[] oldNew = new int[3];
        int[] oldOld = new int[3];
        int[] tem = new int[3];
        for (int i = 0; i < 3; i++) {
            oldOld[i] = gs("OLDPIPA" + ax[i]);
            oldNew[i] = gs("PIPA" + ax[i]);
            ss("OLDPIPA" + ax[i], oldNew[i]);                    // set OLDPIPAX,Y,Z = PIPAX,Y,Z
            ss("RUPTREG" + (i + 1), oldOld[i]);                  // RUPTREG1,2,3 = old OLDPIPAX,Y,Z
            tem[i] = gs("TEM" + ax[i]);
        }
        sd("THISTPIP", d("TIME2"));                              // snapshot time of PIPA reading
        double[] mpac = new double[3];
        for (int i = 0; i < 3; i++) {
            mpac[i] = (oldNew[i] + gs("PIPATMP" + ax[i])) / 16384.0;     // PIPA + PIPATMP
            ss("DELVROD" + (i == 0 ? "" : "+" + 2 * i), -oldNew[i] + tem[i] + oldOld[i]);
            ss("TEM" + ax[i], 0);                                // so we know when READACCS changes them
        }
        ss("MODE", -1);
        itrpnt1(mpac);
    }

    /** ITRPNT1: scale the PIPA vector, update velocity, HDOTDISP, HCALC1 and the commanded acceleration. */
    private void itrpnt1(double[] mpac) {
        double kpip1 = d("KPIP1");
        double[] dv = vxsc(mpac, kpip1);                         // M/CS * 2(-7)
        double t = d("THISTPIP") - d("PIPTIME");                 // 30D: time in cs since PIPTIME
        double frac = t / d("4SEC(28)");
        double[] v24 = vad(vad(vxsc(vsu(vec("GDT/2"), vec("VBIAS")), frac * 4), vec("V")), dv);   // updated velocity
        svec("24D", v24);
        double[] r = vec("R");
        double rmag = abval(r);
        double[] u14 = unit(r);
        double hdot = dot(v24, u14) * 2;
        sd("HDOTDISP", hdot);                                    // for noun 63
        sd("HCALC1", rmag + t * 2048 * hdot - d("/LAND/"));      // update HCALC1 for noun 63
        double accel = (d("VDGVERT") - hdot) / d("TAUROD");      // desired altitude acceleration
        double g20 = abval(vec("GDT/2")) / GSCALE / 4;           // 20D
        double aTot = g20 + accel;
        double[] xAxis = cduNbsm(new double[] {1, 0, 0});   // UNITX through CDU*NBSM
        double c22 = dot(xAxis, u14);                            // 22D
        double afc = aTot / c22;
        sd("/AFC/", afc);
        // measured acceleration magnitude
        double[] dvr = vad(vxsc(vec("DELVROD"), kpip1), vec("VBIAS"));
        double meas = abval(dvr);
        double dT = d("THISTPIP") - d("LASTTPIP");
        sd("LASTTPIP", d("THISTPIP"));
        double acc = meas * SHFTFACT / dT;
        double mass = d("MASS");
        double w = acc + d("FWEIGHT") * BIT1H / mass / d("SCALEFAC");
        afc = afc + d("LAG/TAU") * (w - g20 / c22);
        double max = d("MAXFORCE") / mass;
        double min = d("MINFORCE") / mass;
        afc = Math.max(min, Math.min(max, afc));                 // AFCSPOT: limit between MINFORCE and MAXFORCE
        sd("/AFC/", afc);
        // ITRPNT2: MPAC = measured acceleration; THROTTLE+3
        sd("MPAC", acc);
        new ThrottleControlRoutines(agc).throttle();
        vhorcomp(v24);                                           // pick up updated velocity vector
    }

    /** CDU*NBSM: transforms a vector from navigation base to stable member axes (external, not part of this file). */
    private double[] cduNbsm(double[] v) {
        VecOps.setV(agc, "MPAC", v);
        new PoweredFlightSubroutines(agc).cduNbsm();
        return VecOps.getV(agc, "MPAC");
    }

    /** VHORCOMP: VHORIZ = |V - HDOTDISP UNIT(R)| (with DELVS term); display V06N60 avoiding PHASCHNG. */
    private void vhorcomp(double[] v) {
        double[] vv = vad(v, vxsc(vec("DELVS"), 0.25));
        double[] vh = vsu(vv, vxsc(unit(vec("R")), d("HDOTDISP") * 2));
        sd("VHORIZ", abval(vh));
        dispexitPlus3();                                         // BANKCALL DISPEXIT +3
    }

    // ---------------- REDESIGNATOR TRAP ----------------

    /** PITFALL: RUPT10 entry; if not in P64 resume, else start the redesignator monitor. */
    public void pitfall() {
        if (gs("MODREG") != 64) {                       // CHECKMM 64
            return;                                     // RESUME
        }
        ss("ELVIRA", (~agc.get("CHAN31")) & ALL4BITS);  // READ CHAN31 ; COM ; MASK ALL4BITS
        ss("ZERLINA", 2);
        // TWIDDLE REDESMON in 5 (WAITLIST)
    }

    /** PREMON1: diminish ZERLINA, continue monitor. */
    private void premon1(int zerlina) {
        ss("ZERLINA", zerlina);
        premon2();
    }

    /** PREMON2: delay 7 (VARDELAY) and run REDESMON again. */
    private void premon2() {
        // CAF SEVEN ; TC VARDELAY : REDESMON re-entered after delay
    }

    /** REDESMON: redesignator monitor (initiated by PITFALL). */
    public void redesmon() {
        int bits = (~agc.get("CHAN31")) & ALL4BITS;     // READ 31 ; COM ; MASK
        int last = gs("ELVIRA");
        ss("ELVIRA", bits);
        if (bits != 0) {
            premon2();                                  // any bits this pass: continue monitor
            return;
        }
        if (last != 0) {
            countEm(last);                              // bits last pass: count them, reset rupt, terminate
            return;
        }
        if (gs("ZERLINA") > 0) {                        // CCS ZERLINA
            premon1(gs("ZERLINA") - 1);
            return;
        }
        resetrpt();
    }

    /** RESETRPT: reset RUPT10 and terminate. */
    private void resetrpt() {
        agc.set("CHAN13", agc.get("CHAN13") | BIT12);
        // TASKOVER
    }

    /** COUNT'EM: add azimuth / elevation increments from the rotational hand controller bits. */
    private void countEm(int l) {
        if (ch31Bit13Clear()) {                         // in attitude hold: skip redesignation logic
            resetrpt();
            return;
        }
        int az = gs("AZINCR1");
        int el = gs("ELINCR1");
        if ((l & MINUS_AZBIT) != 0) {
            az -= (int) Math.round(AZEACH * 16384);     // -AZ
        }
        if ((l & PLUS_AZBIT) != 0) {
            az += (int) Math.round(AZEACH * 16384);     // +AZ
        }
        if ((l & MINUS_ELBIT) != 0) {
            el -= (int) Math.round(ELEACH * 16384);     // -EL
        }
        if ((l & PLUS_ELBIT) != 0) {
            el += (int) Math.round(ELEACH * 16384);     // +EL
        }
        ss("AZINCR1", az);
        ss("ELINCR1", el);
        resetrpt();
    }

    // ---------------- R.O.D. TRAP ----------------

    /** DESCBITS: from MARKRUPT with bit 7 or 6 of channel 16 in A; bit 7 means -1 rate increment, bit 6 +1. */
    public void descbits(int a) {
        int inc = ((a & 0100) != 0) ? -1 : +1;          // MASK BIT7 ; CCS A ; CS TWO ; AD ONE
        ss("RODCOUNT", gs("RODCOUNT") + inc);
        // RESUME : trap is reset when switch is released
    }

    // ---------------- DOUBLE PRECISION ROOT FINDER (BY ALLAN KLUMPP) ----------------

    /**
     * ROOTPSRS: finds one root of the power series A(N)X^N + ... + A(1)X + A(0) by Newton's method
     * from an initial guess. The precision required of the root (PRECROOT, fraction of the first
     * guess) is read from TABLTTF+10. The derivative coefficient table is I*A(I). Gives up after
     * eight passes (returns null, the caller then takes the alarm return).
     *
     * @return {root, iteration count} or null if it failed to converge
     */
    public double[] rootpsrs(double[] a, double guess) {
        int n = a.length - 1;
        double precroot = gs("TABLTTF+10") / 16384.0;
        double dxcrit = Math.abs(guess * precroot);
        double[] der = new double[n];
        for (int i = 1; i <= n; i++) {
            der[i - 1] = i * a[i];                       // DERCOF = I x A(I)
        }
        double x = guess;
        int mode = 0;
        while (true) {
            double deriv = powrsers(der, x);
            double resid = powrsers(a, x);
            double dx = deriv == 0 ? 0 : resid / deriv;
            x -= dx;                                     // corrected root
            if ((mode & BIT4) != 0 && mode >= 8) {
                return null;                             // BADROOT: give up after eight passes
            }
            mode++;
            if (Math.abs(dx) - dxcrit <= 0) {
                break;
            }
        }
        ss("MPAC+2", mode);                              // SP iteration count
        return new double[] {x, mode};
    }

    /** POWRSERS: evaluates the power series by Horner's rule. */
    private static double powrsers(double[] coef, double x) {
        double r = 0;
        for (int i = coef.length - 1; i >= 0; i--) {
            r = r * x + coef[i];
        }
        return r;
    }

    // ---------------- TRASHY LITTLE SUBROUTINES ----------------

    /** INTPRETX: set X1 on the way to the interpreter: X1 = -TARGTDEX(WCHPHASE). */
    public void intpretx() {
        int w = wch();
        ss("X1", -TARGTDEX[w + 1]);
    }

    /**
     * TDISPSET: TTFDISP = TTF/8 scaled; TREDES becomes zero two passes before TCGFAPPR is reached
     * (TREDES = 99 counting down).
     */
    public void tdispset() {
        double ttf8 = d("TTF/8");
        sd("TTFDISP", ttf8 * TSCALINV);
        int tcgf = gs("TCGFAPPR");
        int x = (int) ((tcgf + (int) Math.round(ttf8 * 16384)) * TREDESCL) + NEG_DEC103;
        int tredes = Math.max(0, Math.min(POS_DEC99, x + POS_DEC99));
        ss("TREDES", tredes);
    }

    /** 1406P00: POODOO 01406 (IGNALG TTF computation failure). */
    public void p1406p00() {
        new AlarmAndAbort(agc).poodoo(01406);
    }

    /** 1406ALM: ALARM 01406, then RATESTOP. */
    public void p1406alm() {
        alarm(01406);
        ratestop();
    }

    /**
     * FASTCHNG: specialised PHASCHNG; stores the phase-table name from the caller's return word
     * (PHSNAME3) in EBANK3 and returns past it.
     */
    public void fastchng() {
        // CA EBANK3 ; XCH EBANK ; DXCH L ; TS PHSNAME3 ; LXCH EBANK ; TC A
    }

    // ---- parameter table indirect addresses ----
    // RDG = RBRFG, VDG = VBRFG, ADG = ABRFG, VDG2TTF = VBRFG*, ADG2TTF = ABRFG*, JDG2TTF = JBRFG*
}
