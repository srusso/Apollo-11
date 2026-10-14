package apollo.luminary099;

import apollo.agc.Agc;

import java.util.HashMap;
import java.util.Map;

/**
 * P51-P53.agc (Luminary 1A build 099, pages 926-983): IMU alignment programs.
 *
 * <ul>
 * <li>PROG52 (P52) IMU realign to preferred / nominal / REFSMMAT / landing-site orientation,
 *     with S52.2 (gimbal angles), S52.3 (nominal orientation), CAL53A (coarse align decision)</li>
 * <li>S50/LOCSAM (sun, earth, moon vectors and occultation cones for R56)</li>
 * <li>R56 / PICAPAR (star-pair selection), R51 (fine align), R52 (automatic optics positioning),
 *     R55 (gyro torque), R54 / CHKSDATA (star data check), GYCOARS, R59 (lunar surface
 *     star acquisition), PLANET, GVDETER (gravity vector determination), GYROTRIM</li>
 * <li>P51 (IMU orientation determination), P57 (lunar surface alignment), IMUCHK, LSORIENT</li>
 * </ul>
 *
 * Interpretive-language vector code is expressed with the small vector helpers below using
 * natural (unscaled) units: vectors are unit vectors / metres, angles are revolutions or radians
 * as noted, and AGC fixed-point scaling constants (e.g. CSS33 = cos50/4) are given in natural
 * units. Erasable vectors/matrices/double-precision words live in a symbol table inside this
 * class; single-precision words go through {@link Agc}. External routines are called through
 * {@link #extern(String, Object...)}. Routines that exit via GOTOPOOH/ENDOFJOB simply return.
 */
public class P51P53 {
    // Return indices for BANKCALLed display/aligner routines: caller code at L+1, L+2, L+3.
    public static final int TERMINATE = 0;   // L+1: TC GOTOPOOH (V34E)
    public static final int PROCEED = 1;     // L+2: V33E
    public static final int ENTER = 2;       // L+3: ENTER / V32E (recycle)

    // ---- constants ----
    /** CSS5: (COS 5)/4. */
    public static final double CSS5 = 0.2490475;
    /** CSSUN: (COS 60)/4 -> cosine used for the sun cone. */
    public static final double CSSUN = 0.125;
    /** 5DEGREES, in revolutions. */
    public static final double FIVE_DEGREES = 0.013888889;
    /** CSS40 / CSS33: (COS 50)/4. */
    public static final double CSS40 = 0.16070;
    public static final double CSS33 = 0.16070;
    /** TSIGHT1: 6 minutes to marking (centiseconds). */
    public static final double TSIGHT1 = 36000;
    /** DEGREE1: 1 deg, CDU/2 scaling. */
    public static final int DEGREE1 = 46;
    /** DEG359: 359 deg, CDU/2 scaling. */
    public static final int DEG359 = 16338;
    public static final double ONE_SIXTH = 0.1666667;
    public static final double DEG30 = 0.083333333;
    public static final double DEG_HALF = 0.00138888;
    public static final int DEG60 = 012525;
    public static final int DEC227 = 227;
    public static final int T_HALF_SEC = -20;
    public static final int DEG42 = 07357;
    public static final int DEG35 = 06211;

    public static final int VN_V06N34 = 634;
    public static final int VN_V06N89 = 689;
    public static final int VN_V16N20 = 1620;
    public static final int VN_V06N93 = 693;
    public static final int VN_V06N05 = 605;
    public static final int VN_V41K = 4100;
    public static final int VN_V01N70 = 170;
    public static final int VN_V06N87 = 687;
    public static final int VN_V06N79 = 679;
    public static final int VN_PLANV = 688;
    public static final int VN_GRAVERR = 604;
    public static final int VN_V05N06 = 506;
    public static final int OCT13 = 013;
    public static final int OCT14 = 014;
    public static final int OCT15 = 015;
    public static final int PHASE_4_24 = 04024;

    private static final int BIT1 = 1, BIT2 = 2, BIT4 = 8, BIT7 = 0x40, BIT9 = 0x100, BIT13 = 0x1000;
    private static final int HIGH9 = 077700;

    private final Agc agc;
    private final Map<String, double[]> vecs = new HashMap<>();

    public P51P53(Agc agc) {
        this.agc = agc;
    }

    // ------------------------------------------------------------------ infrastructure

    /**
     * Hook for BANKCALL / TC to routines in other files (GOFLASH, GOPERF1, PHASCHNG, CDUTRIG,
     * CALCSMSC, IMUCOARS, ...). Returns the return-slot index for multi-return routines; the
     * default simulates the operator pressing PROCEED.
     */
    protected int extern(String routine, Object... args) {
        return PROCEED;
    }

    /** Hook for the star catalogue CATLOG,n (n = star number 1..37) -> unit vector. */
    protected double[] catlog(int star) {
        return new double[3];
    }

    private void upflag(String f) { agc.set(f, 1); }
    private void downflag(String f) { agc.set(f, 0); }
    private boolean flag(String f) { return agc.get(f) != 0; }
    private int get(String s) { return agc.get(s); }
    private void set(String s, int v) { agc.set(s, v); }

    private double[] v(String s) {
        return vecs.computeIfAbsent(s, k -> new double[3]);
    }

    private void sv(String s, double[] x) {
        vecs.put(s, x.clone());
    }

    /** Double-precision scalar kept in vecs as a one-element array. */
    private double d(String s) {
        return vecs.computeIfAbsent(s, k -> new double[1])[0];
    }

    private void sd(String s, double x) {
        vecs.put(s, new double[] {x});
    }

    private static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }

    private static double[] unit(double[] a) {
        double m = Math.sqrt(dot(a, a));
        return m == 0 ? a.clone() : new double[] {a[0] / m, a[1] / m, a[2] / m};
    }

    private static double[] scale(double[] a, double k) {
        return new double[] {a[0] * k, a[1] * k, a[2] * k};
    }

    private static double[] add(double[] a, double[] b) {
        return new double[] {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
    }

    /** MXV: matrix (three row vectors) times vector. */
    private double[] mxv(String m, double[] x) {
        return new double[] {dot(v(m), x), dot(v(m + "+1"), x), dot(v(m + "+2"), x)};
    }

    /** VXM: vector times matrix = sum of rows weighted by the vector components. */
    private double[] vxm(double[] x, String m) {
        return add(add(scale(v(m), x[0]), scale(v(m + "+1"), x[1])), scale(v(m + "+2"), x[2]));
    }

    /** MATMOVE: copy a 3x3 matrix (X1 = source, X2 = destination). */
    public void matMove(String from, String to) {
        sv(to, v(from));
        sv(to + "+1", v(from + "+1"));
        sv(to + "+2", v(from + "+2"));
    }

    // ======================================================================================
    // PROG52 -- IMU realign
    // ======================================================================================

    /** PROG52: align the IMU to preferred / nominal / REFSMMAT / landing-site orientation. */
    public void prog52() {
        extern("R02BOTH");                              // IMU status check
        set("OPTION2", flag("PFRATFLG") ? BIT1 : BIT2); // P52A: preferred if available, else nominal
        int r;
        do {
            set("OPTION1", BIT1);
            r = extern("GOPERF4R", VN_V05N06);          // flash V04N06 option code
            if (r == TERMINATE) { extern("GOTOPOOH"); return; }
        } while (r == ENTER);                           // new code keyed in
        extern("PHASCHNG", 00014);                      // display return
        switch (get("OPTION2") & 3) {
            case 0: opt4(); break;                      // landing site
            case 1: p52d();                             // P52H preferred
                    break;
            case 2: p52t(false); break;                 // nominal
            default: p52f(); break;                     // P52E: REFSMMAT -> straight to R51
        }
    }

    /** OPT4: option 4 displays TLAND then continues at P52T+2. */
    private void opt4() {
        p52t(true);
    }

    /** P52T: ask for alignment time (V06N34); zero or negative key-in means "now" (TIME2). */
    private void p52t(boolean fromOpt4) {
        sd("DSPTEM1", fromOpt4 ? d("TLAND") : -0.0);
        int r;
        do {
            r = extern("GOFLASH", VN_V06N34);
            if (r == TERMINATE) { extern("GOTOPOOH"); return; }
        } while (r == ENTER);
        double t = d("DSPTEM1");
        sd("TALIGN", t > 0 ? t : d("TIME2"));
        p52v();
    }

    /** P52V: option 4 -> landing site orientation (P52LS), otherwise S52.3 nominal orientation. */
    private void p52v() {
        if ((get("OPTION2") & BIT2) != 0) {             // options 2 (nominal) -> compute at P52W
            p52w();
        } else {
            p52ls();                                    // option 4
        }
    }

    /** P52W: nominal IMU orientation at TALIGN (S52.3), then gimbal angles (S52.2). */
    private void p52w() {
        s523(d("TALIGN"));
        p52d();
    }

    /** P52D: S52.2 gimbal angles, display V06N22, then coarse-align type selection. */
    private void p52d() {
        s522();
        int r = extern("GOFLASH", 622);                 // V06N22 display of THETAD
        if (r == TERMINATE) { extern("GOTOPOOH"); return; }
        coarsTyp();
    }

    /** REGCOARS: normal coarse align via CAL53A, then fall into fine align (R51). */
    private void regCoars() {
        cal53a();
        coarsRet();
    }

    /** COARSRET: set REFSMFLG, clear PFRATFLG, then R51. */
    private void coarsRet() {
        upflag("REFSMFLG");
        downflag("PFRATFLG");
        p52f();
    }

    /** P52F: fine align. */
    private void p52f() {
        r51();
        p52out();
    }

    /** P52OUT: exit to GOTOPOOH. */
    private void p52out() {
        extern("GOTOPOOH");
    }

    /** COARSTYP: V50N25 checklist 13: V34 terminate, V33 normal coarse, V32 gyro-torque coarse. */
    private void coarsTyp() {
        int r = extern("GOPERF1", OCT13);
        if (r == TERMINATE) { extern("GOTOPOOH"); return; }
        if (r == PROCEED) { regCoars(); return; }
        // V32: gyro torque coarse alignment. Desired SM axes wrt present SM.
        sv("XDC", unit(mxv("REFSMMAT", v("XSMD"))));
        sv("YDC", unit(mxv("REFSMMAT", v("YSMD"))));
        sv("ZDC", unit(mxv("REFSMMAT", v("ZSMD"))));
        gyCoars();
        p52out();
    }

    /** P52LS: landing-site orientation (option 4). */
    private void p52ls() {
        upflag("LUNAFLAG");
        downflag("ERADFLAG");                           // to pick up RLS
        double[] rls = v("RLS");
        sd("TLAND", d("TALIGN"));                       // jam align time in TLAND for option 4
        sv("ALPHAV", scale(rpToR(rls, d("TALIGN")), 0.25)); // VSR2
        n89Disp(d("TALIGN"));
        sv("XSMD", unit(v("ALPHAV")));
        lsOrient();
        p52d();                                         // now compute gimbal angles
    }

    /** RP-TO-R / R-TO-RP: coordinate transformations between reference and moon-fixed (external). */
    protected double[] rpToR(double[] x, double time) {
        return x.clone();
    }

    protected double[] rToRp(double[] x, double time) {
        return x.clone();
    }

    /** N89DISP: calculate (LAT-LONG) and store landing-site data for the V06N89 display. */
    private void n89Disp(double time) {
        extern("LAT-LONG");                             // sets LAT, LONG, ALT from ALPHAV
        sd("GDT/2+4", time);                            // temp store time
        sd("LANDLONG", d("LONG") / 2);                  // SR1
        sd("LANDALT", d("ALT"));
        sd("LANDLAT", d("LAT"));
    }

    /** LSDISP: display lat, long/2, alt (V06N89) and take back the (possibly loaded) site. */
    public void lsDisp() {
        int r;
        do {
            r = extern("GOFLASH", VN_V06N89);
            if (r == TERMINATE) { extern("GOTOPOOH"); return; }
        } while (r == ENTER);                           // V32 / ENTER: look again / load new
        sd("LONG", d("LANDLONG") * 2);                  // SL1
        sd("ALT", d("LANDALT"));
        sd("LAT", d("LANDLAT"));
        extern("LALOTORV", d("GDT/2+4"));               // RLS B-29 in MPAC and ALPHAV
    }

    // ======================================================================================
    // S50 / LOCSAM -- inputs for PICAPAR and PLANET
    // ======================================================================================

    /**
     * LOCSAM (S50): compute VSUN, VEARTH, VMOON and the cosines of the occultation cones
     * CEARTH, CSUN, CMOON for time TSIGHT. In earth influence (MOONCNTR not taken):
     * VEARTH = -UNIT(RLEM), VMOON = UNIT(R_EM*U_EM - R_L), CMOON = cos 5, CEARTH = cos(5+asin(RE/|RL|)).
     */
    public void locSam(double tsight) {
        sd("TSIGHT", tsight);
        extern("LSPOS");                                // sun and moon positions
        sd("TDEC1", tsight);
        extern("LEMPREC");                              // LM position RATT
        double[] ratt = v("RATT");
        boolean moonCentered = flag("MOONFLAG");       // SSP/TIX,2 selects MOONCNTR
        if (!moonCentered) {
            // EARTCNTR
            double[] vmoon = unit(add(scale(v("VMOON"), d("RSUBEM")), scale(ratt, -1))); // VSL1 VSU
            sv("VMOON", vmoon);
            sv("VEARTH", scale(unit(ratt), -1));
            sd("CEARTH", occCos(d("RSUBE"), ratt));
            sd("CMOON", CSS5);
        } else {
            // MOONCNTR: sun relative to moon, earth direction from EM geometry
            sv("VSUN", unit(add(scale(v("VMOON"), d("ROE")), scale(v("VSUN"), -1))));
            sv("VEARTH", scale(unit(add(scale(v("VMOON"), d("RSUBEM")), ratt)), -1));
            sv("VMOON", scale(unit(ratt), -1));
            sd("CMOON", occCos(d("RSUBM"), ratt));
            sd("CEARTH", CSS5);
        }
        sd("CSUN", CSSUN);
    }

    /** OCCOS: cos(5deg + asin(radius/|r|)); angle scaled in revolutions. */
    private double occCos(double radius, double[] r) {
        double mag = Math.sqrt(dot(r, r));
        double ang = Math.asin(Math.min(1.0, radius / mag)) / (2 * Math.PI) + FIVE_DEGREES;
        return Math.cos(2 * Math.PI * ang) / 4.0;
    }

    // ======================================================================================
    // R56 / PICAPAR -- star pair selection
    // ======================================================================================

    /**
     * PICAPAR (= R56): select the best pair of unocculted stars within 50 deg of the shaft axis
     * and with separation in range. Outputs BESTI, BESTJ (star numbers times 6).
     *
     * @return true for the normal return (pair found), false for the error return (none).
     */
    public boolean picapar() {
        extern("CDUTRIG");
        extern("CALCSMSC");
        upflag("VFLAG");
        set("BESTI", 0);
        set("BESTJ", 0);
        double[] half = add(scale(v("XNB"), 0.5), scale(v("ZNB"), 0.5));
        sv("SAX", unit(vxm(half, "REFSMMAT")));          // shaft axis in reference
        double[] sax = v("SAX");
        for (int i = 37; i >= 1; i--) {                  // PIC1: major star (X1 = 6*i)
            double[] ci = catlog(i);
            if (dot(ci, sax) - CSS33 < 0) continue;      // PIC2: outside field of view
            for (int j = i - 1; j >= 1; j--) {           // PIC3/PIC4: minor star (X2)
                double[] cj = catlog(j);
                if (dot(cj, sax) - CSS33 < 0) continue;
                if (dot(ci, cj) - CSS40 >= 0) continue;  // separation test
                if (occult(ci)) continue;
                if (occult(cj)) continue;
                strategy(i, j, ci, cj);
            }
        }
        return picEnd();
    }

    /** STRATGY / STRAT / NEWPAR: keep the pair with maximum separation. */
    private void strategy(int i, int j, double[] ci, double[] cj) {
        if (flag("VFLAG")) {                             // first pair found
            downflag("VFLAG");
            set("BESTI", i * 6);
            set("BESTJ", j * 6);
            return;
        }
        double[] bi = catlog(get("BESTI") / 6);
        double[] bj = catlog(get("BESTJ") / 6);
        if (dot(ci, cj) - dot(bi, bj) >= 0) return;      // not wider than the best so far
        set("BESTI", i * 6);                             // NEWPAR
        set("BESTJ", j * 6);
    }

    /**
     * OCCULT: sets CULTFLAG when the star lies inside any of the earth/sun/moon exclusion cones
     * (CULTRIX rows = VEARTH, VSUN, VMOON against CEARTH, CSUN, CMOON).
     */
    private boolean occult(double[] star) {
        double[] m = {dot(v("VEARTH"), star), dot(v("VSUN"), star), dot(v("VMOON"), star)};
        double[] c = {d("CEARTH"), d("CSUN"), d("CMOON")};
        boolean culted = (c[0] - m[0]) <= 0 || (c[1] - m[1]) < 0 || (c[2] - m[2]) < 0;
        agc.set("CULTFLAG", culted ? 1 : 0);
        return culted;
    }

    /** PICEND / PICGXT: order the pair so BESTI is the star nearer the shaft axis. */
    private boolean picEnd() {
        if (flag("VFLAG")) return false;                 // PICBXT: no pair in field of view
        int bi = get("BESTI"), bj = get("BESTJ");
        double[] sax = v("SAX");
        if (dot(sax, catlog(bi / 6)) - dot(sax, catlog(bj / 6)) < 0) {   // PICNSWP not taken
            set("BESTI", bj);
            set("BESTJ", bi);
        }
        return true;
    }

    // ======================================================================================
    // R51 -- fine align
    // ======================================================================================

    /** R51: align the stable member to REFSMMAT using two star sightings. */
    public void r51() {
        set("QMAJ", 0);
        extern("PHASCHNG", PHASE_4_24);
        r51c();
    }

    /** R51C: request star sighting (V50N25 checklist 15), compute stars, call R56. */
    private void r51c() {
        int r = extern("GOPERF1", OCT15);
        if (r == TERMINATE) { extern("GOTOPOOH"); return; }
        if (r == ENTER) { r51e(); return; }              // ENTER: use previously selected stars
        locSam(extern("LOADTIME") + TSIGHT1);            // TSIGHT = now + 6 min
        if (picapar()) { r51e(); return; }
        extern("ALARM", 0405);                           // two stars not available
        int fr = extern("GOFLASH", 509);                 // V05N09
        if (fr == TERMINATE) { extern("GOTOPOOH"); return; }
        if (fr == PROCEED) { r51e(); return; }
        r51c();
    }

    /** R51E: start with the first star (STARIND = 0). */
    private void r51e() {
        set("STARIND", 0);
        r513();
    }

    /** R51.3: R52 optics positioning, AOT marking, star data test, gyro torque. */
    private void r513() {
        extern("PHASCHNG", PHASE_4_24);
        r52();
        extern("AOTMARK");
        extern("OPTSTALL");
        // TC CURTAINS only on bad end
        if (get("STARIND") == 0) {
            r514();                                      // first star: store and go for second
            return;
        }
        sv("STARSAV2", v("STARAD+6"));
        extern("PHASCHNG", PHASE_4_24);
        double t = d("TSIGHT");
        sv("STARAD+6", unit(mxv("REFSMMAT", planet(t))));     // 2nd star in SM
        sv("STARAD", unit(mxv("REFSMMAT", v("PLANVEC"))));    // 1st star
        sv("6D", v("STARSAV1"));
        sv("12D", v("STARSAV2"));
        chkSData();                                       // R54
        if (flag("FREEFLAG")) {
            axisGen();
            r55();                                        // gyro torque
            downflag("PFRATFLG");
        }
        r51k();
    }

    /** R51K / R51P63: ask for P63 recycle (V50N25 checklist 14) or return to caller. */
    private void r51k() {
        int r = extern("GOPERF1", OCT14);
        if (r == TERMINATE) { extern("GOTOPOOH"); return; }
        r51c();
    }

    /** R51.4: remember first star (STARSAV1) and its planet vector, then do the second star. */
    private void r514() {
        sv("STARSAV1", v("STARAD+6"));
        sv("PLANVEC", planet(d("TSIGHT")));
        set("STARIND", 1);
        r513();
    }

    // ======================================================================================
    // GYCOARS -- gyro torque coarse alignment
    // ======================================================================================

    /** GYCOARS: compute gyro torque angles, torque the gyros, take DESIRED as REFSMMAT. */
    public void gyCoars() {
        extern("CALCGTA");
        downflag("DRIFTFLG");
        downflag("REFSMFLG");
        extern("GODSPR", VN_V16N20);                     // monitor gimbals
        extern("IMUPULSE", "OGC");
        extern("IMUSTALL");
        extern("PHASCHNG", PHASE_4_24);
        matMove("XSMD", "REFSMMAT");                     // store desired REFSMMAT
        downflag("PFRATFLG");
        upflag("REFSMFLG");
        nCoarse();                                       // set drift, initialise 1/PIPADT
    }

    // ======================================================================================
    // R55 -- gyro torque
    // ======================================================================================

    /** R55: compute and send gyro pulses (V06N93 display with proceed). */
    public void r55() {
        extern("CALCGTA");
        int r = extern("GOFLASH", VN_V06N93);
        if (r == TERMINATE) { extern("GOTOPOOH"); return; }
        if (r == PROCEED) {                              // R55.2
            extern("PHASCHNG", 00214);
            extern("IMUPULSE", "OGC");
            extern("IMUSTALL");
            extern("PHASCHNG", PHASE_4_24);
        }
        // R55RET: return to caller (QMIN)
    }

    // ======================================================================================
    // R54 -- CHKSDATA: check validity of star sighting pair
    // ======================================================================================

    /**
     * CHKSDATA (R54): compare the angle between the two observed stars (STARAD, STARAD+6)
     * with the angle between the two catalogue stars (6D, 12D); display the difference (V06N05).
     * FREEFLAG set on exit means PROCEED; reset means the astronaut wants to recycle.
     */
    public void chkSData() {
        upflag("FREEFLAG");
        double a1 = Math.acos(clamp(dot(v("STARAD"), v("STARAD+6")) * 2)) / (2 * Math.PI) ;   // SL1 ACOS (revs)
        downflag("FREEFLAG");                            // 2nd pass flag
        double a2 = Math.acos(clamp(dot(v("6D"), v("12D")) * 2)) / (2 * Math.PI);
        sd("THETA", a2);
        sd("NORMTEM1", Math.abs(a1 - a2));               // ABS, SGNAGREE
        upflag("FREEFLAG");
        int r = extern("GOFLASH", VN_V06N05);
        if (r == TERMINATE) { extern("GOTOPOOH"); return; }
        downflag("FREEFLAG");                            // CHKSDA: proceed -> clear, return
    }

    private static double clamp(double x) {
        return Math.max(-1.0, Math.min(1.0, x));
    }

    // ======================================================================================
    // CAL53A -- coarse align decision
    // ======================================================================================

    /**
     * CAL53A: make a final computation of gimbal angles; if any desired angle differs from the
     * present CDU angle by more than 1 degree (and less than 359), coarse align; fine align only.
     */
    public void cal53a() {
        s522();
        int[] cdu = rdCdus();
        boolean coarse = false;
        for (int k = 0; k < 3; k++) {
            int diff = Math.abs(get("THETAD" + (k == 0 ? "" : "+" + k)) / 2 - cdu[k] / 2);
            if (diff >= DEGREE1 && diff <= DEG359) {
                coarse = true;
                break;
            }
        }
        if (coarse) {
            extern("PHASCHNG", PHASE_4_24);
            coarse();
            nCoarse();
        }
        matMove("XSM", "REFSMMAT");                      // FINEONLY: store the new orientation
    }

    /** RDCDUS: read the three CDUs (inhibited). */
    private int[] rdCdus() {
        return new int[] {get("CDUX"), get("CDUY"), get("CDUZ")};
    }

    // ======================================================================================
    // P51 -- IMU orientation determination
    // ======================================================================================

    /** P51: determine the inertial orientation of the IMU from two star sightings. */
    public void p51() {
        extern("IMUCHK");                                // alarm 210 if ISS off
        p512();
    }

    private void p512() {
        int r = extern("GOPERF1", OCT15);
        if (r == TERMINATE) { extern("GOTOPOOH"); return; }
        if (r == PROCEED) { p51b(); return; }            // V33
        extern("PHASCHNG", PHASE_4_24);
        set("THETAD", 0);                                // zero the gimbals
        set("THETAD+1", 0);
        set("THETAD+2", 0);
        extern("GODSPRET", 622);                         // V06N22
        extern("GODSPRET", VN_V41K);                     // coarse align verb 41
        coarse();
        extern("PHASCHNG", PHASE_4_24);
        p512();
    }

    private void p51b() {
        extern("PHASCHNG", 00014);
        nCoarse();
        set("STARIND", 0);                               // star 1 or 2
        p51c();
    }

    /** P51C: R53 marking, then planet vector; second star; then R54 and AXISGEN. */
    private void p51c() {
        extern("PHASCHNG", PHASE_4_24);
        extern("AOTMARK");                               // R53
        extern("AOTSTALL");
        if (get("STARIND") == 0) {
            sv("STARSAV1", v("STARAD+6"));
        }
        extern("PHASCHNG", PHASE_4_24);
        if (get("STARIND") == 0) {
            extern("PHASCHNG", PHASE_4_24);
            sv("PLANVEC", planet(d("TSIGHT")));
            set("STARIND", 1);
            p51c();                                      // do second star
            return;
        }
        extern("PHASCHNG", PHASE_4_24);                  // P51E
        sv("12D", planet(d("TSIGHT")));
        sv("6D", v("PLANVEC"));
        sv("STARAD", v("STARSAV1"));
        sv("STARAD+6", v("STARSAV2"));
        chkSData();
        if (!flag("FREEFLAG")) {
            p512();                                      // TC P51 +2
            return;
        }
        axisGen();                                       // P51G: REFSMMAT comes back in XDC
        matMove("XDC", "REFSMMAT");
        upflag("REFSMFLG");
        extern("GOTOPOOH");                              // finis
    }

    /** COARSE: coarse align with IMUCOARS then IMUFINE, waiting for each. */
    public void coarse() {
        extern("IMUCOARS");
        extern("IMUSTALL");
        extern("IMUFINE");
        extern("IMUSTALL");
    }

    /** NCOARSE: after coarse alignment set drift flag and initialise 1/PIPADT, PIPA counters. */
    public void nCoarse() {
        set("1/PIPADT", get("TIME1"));
        set("PIPAX", Agc.WORD_MASK);                     // CS ZERO = -0
        set("PIPAY", Agc.WORD_MASK);
        set("PIPAZ", Agc.WORD_MASK);
        sv("GCOMP", new double[3]);
        upflag("DRIFTFLG");
    }

    // ======================================================================================
    // S52.2, S52.3
    // ======================================================================================

    /**
     * S52.2: compute the gimbal angles for the desired SM (XSMD, YSMD, ZSMD) and the present
     * vehicle attitude. Outputs OGC, IGC, MGC and THETAD,+1,+2.
     */
    public void s522() {
        extern("CDUTRIG");
        extern("CALCSMSC");                              // XNB, YNB, ZNB in present SM
        for (int k = 0; k < 3; k++) {                    // S52.2A: NB axes in REF
            String n = k == 0 ? "XNB" : k == 1 ? "YNB" : "ZNB";
            sv(n, unit(vxm(v(n), "REFSMMAT")));
        }
        matMove("XSMD", "XSM");
        extern("CALCGA");
    }

    /**
     * S52.3: nominal IMU orientation at the time in MPAC:
     * XSMD = UNIT(R), YSMD = UNIT(V x R), ZSMD = UNIT(XSMD x YSMD).
     */
    public void s523(double time) {
        sd("TDEC1", time);
        extern("LEMCONIC");                              // RATT, VATT at TDEC1
        sv("XSMD", unit(v("RATT")));
        sv("YSMD", unit(cross(v("VATT"), v("RATT"))));
        sv("ZSMD", unit(cross(v("XSMD"), v("YSMD"))));
    }

    // ======================================================================================
    // R52 -- automatic optics positioning
    // ======================================================================================

    /** AOTAZ -1,n : AOT azimuth CDU word for detent n (1..6); defined in the AOT mark file. */
    protected int aotAz(int detent) {
        return 0;
    }

    /** OANB: compute the AOT optic axis in NB coordinates from AZ, EL (words 8D, 9D) -> SCAXIS. */
    protected void oanb(int az, int el) {
        extern("OANB", az, el);
    }

    /**
     * R52: point the AOT optics axis at a navigation star chosen by the alignment programs.
     * Input BESTI/BESTJ (star codes x 6). Output STARCODE: star in bits 1-6, detent in bits 7-9.
     */
    public void r52() {
        int starcode = get("BESTI") / 6 + 0x80;          // 1/6TH scaling, detent position 2
        set("STARCODE", starcode);
        int r;
        do {                                             // R52A
            r = extern("GOFLASH", VN_V01N70);            // display STARCODE
            if (r == TERMINATE) { extern("GOTOPOOH"); return; }
        } while (r == ENTER);                            // select new star code -> recycle
        downflag("3AXISFLG");                            // allow VECPOINT
        int detent = (get("STARCODE") & HIGH9) >> 6;
        int az, el;
        if (detent == 0 || detent == 7) {                // COAS calibration / sighting
            r = getAzEl();
            if (r < 0) return;
            az = get("AZ");
            el = get("EL");
        } else {
            az = aotAz(detent);
            el = BIT13;                                  // 45 deg elevation
        }
        oanb(az, el);                                    // SCAXIS in NB
        sv("POINTVSM", unit(mxv("REFSMMAT", planet(extern("LOADTIME")))));
        extern("R60LEM");                                // torque LEM optic axis to star LOS
        if ((get("STARCODE") & HIGH9) == 0) {            // COAS calibration code 0: recycle
            r52();
        }
    }

    /** GETAZEL: key in AZ and EL (V06N87); returns -1 if terminated. */
    private int getAzEl() {
        int r;
        do {
            r = extern("GOFLASH", VN_V06N87);
            if (r == TERMINATE) { extern("GOTOPOOH"); return -1; }
        } while (r == ENTER);
        return 0;
    }

    // ======================================================================================
    // R59 -- lunar surface star acquisition
    // ======================================================================================

    /**
     * R59: for the selected catalogue star find the AOT detent position where the star is in the
     * field of view and display CURSOR, SPIRAL and position code (V06N79), then AOTMARK.
     */
    public void r59() {
        if (!flag("REFSMFLG")) { r59Out(); return; }     // no REFSMMAT: bypass acquisition
        int r = extern("GOFLASH", VN_V01N70);            // star code
        if (r == TERMINATE) { extern("GOTOPOOH"); return; }
        if (r != PROCEED) { r59(); return; }
        int star6 = ((get("AOTCODE") & ~HIGH9) & Agc.WORD_MASK) * 6;
        set("BESTI", star6);                             // BESTI[STARIND]
        if (star6 == 0 || DEC227 - star6 <= 0) { r59Out(); return; }   // not a catalogue star
        double[] starNb = unit(extern3("CDU*SMNB", mxv("REFSMMAT", catlog(star6 / 6))));
        sv("STAR", starNb);
        int poscode = 1;
        for (;;) {                                       // INCAZ
            int az = aotAz(poscode);
            set("QMIN", az);
            oanb(az, BIT13);                             // SCAXIS wrt NB
            double sep = Math.acos(clamp(dot(starNb, v("SCAXIS")) * 2)) / (2 * Math.PI);  // revs
            if (sep - DEG30 <= 0) {                      // star inside the AOT field of view
                double cursor, spiral;
                if (sep - DEG_HALF < 0) {                // ZSPCR: star on the optic axis
                    cursor = 0;
                    spiral = 0;
                } else {
                    double sepSpiral = sep * 12;         // 12 x angle (revs)
                    double[] oa = v("SCAXIS");
                    double[] c1 = unit(cross(oa, new double[] {1, 0, 0}));        // OA x UNITX
                    double[] c2 = unit(scale(cross(oa, c1), 1));                  // OA x (OA x UNITX)
                    double[] oaStar = unit(cross(oa, starNb));
                    double thet = Math.acos(clamp(dot(oaStar, c1) * 2)) / (2 * Math.PI);
                    if (dot(oaStar, c2) < 0) thet = 1.0 - thet;   // 360 - THET
                    double azRevs = az / 32768.0;
                    cursor = (thet + azRevs) % 1.0;      // YROT + AZ(N), 1STO2S
                    spiral = (sepSpiral + cursor) % 1.0; // 12(SEP) + YROT
                }
                sd("CURSOR", cursor);
                sd("SPIRAL", spiral);
                set("POSCODE", poscode);
                break;
            }
            poscode++;                                   // NXAX
            if (7 - poscode <= 0) {
                extern("ALARM", 0404);                   // star can't be located in AOT field
                int fr = extern("GOFLASH", 509);         // V05N09
                if (fr == TERMINATE) { extern("GOTOPOOH"); return; }
                if (fr == PROCEED) { r59Out(); return; } // go without acquire
                r59();                                   // recycle, try another star
                return;
            }
        }
        r = extern("GOFLASH", VN_V06N79);                // CURSOR, SPIRAL, POSCODE
        if (r == TERMINATE) { extern("GOTOPOOH"); return; }
        if (r != PROCEED) { r59(); return; }
        // R59E: detent code from POSCODE into bits 7-9 of AOTCODE
        set("AOTCODE", (get("AOTCODE") & ~HIGH9 & Agc.WORD_MASK) + ((get("POSCODE") & 7) * BIT7));
        r59Out();
    }

    private double[] extern3(String routine, double[] in) {
        return in;                                       // CDU*SMNB: SM -> NB via present CDUs (external)
    }

    /** R59OUT: AOTMARK for the sighting, then return to 1STAR / 2STAR logic (R59RET). */
    private void r59Out() {
        extern("AOTMARK");
        extern("AOTSTALL");                              // sleep until sighting is done
        r59Ret();
    }

    // ======================================================================================
    // PLANET
    // ======================================================================================

    /**
     * PLANET: reference vector of the sighted body at the given time. Stars are fetched from the
     * catalogue; sun/earth/moon are computed by LOCSAM; "planet" codes are keyed in via V06N88.
     */
    public double[] planet(double tsight) {
        sd("TSIGHT", tsight);
        sv("STARAD", new double[3]);
        int n = (get("AOTCODE") & ~HIGH9 & Agc.WORD_MASK) * 6;   // star code x 6
        set("BESTI", n);                                 // BESTI[STARIND]
        if (n == 0) {                                    // code 0: planet vector from DSKY
            int r;
            do {
                r = extern("GOFLASH", VN_PLANV);
            } while (r != PROCEED);
            return unit(v("STARAD"));
        }
        if (DEC227 - n > 0) {                            // catalogue star
            return catlog(n / 6);
        }
        locSam(tsight);                                  // CALSAM
        // after the VEARTH/VSUN exchange the vectors are ordered sun (228), earth (234), moon (240)
        switch (n / 6 - 38) {
            case 0: return v("VSUN");
            case 1: return v("VEARTH");
            default: return v("VMOON");
        }
    }

    // ======================================================================================
    // GVDETER -- gravity vector determination (lunar surface alignment)
    // ======================================================================================

    /**
     * GVDETER: find the direction of lunar gravity by reading the PIPAs with the IMU at two
     * orientations (about 180 deg apart about G) and averaging. Output STARSAV1 = GSAV = UNIT(G).
     */
    public void gvDeter() {
        set("THETAD", DEG42);
        set("THETAD+1", Agc.fromSigned(-DEG42));
        set("THETAD+2", DEG35);
        downflag("REFSMFLG");
        lung();                                          // first orientation: G in NB -> GOUT
        // Rotate SM 180 deg about G: XSM = 2(X X ) - I in G coordinates, XNB = NBSM(I)
        double[] g = v("STAR");
        double[][] xm = new double[3][];
        double[][] xn = new double[3][];
        for (int k = 0; k < 3; k++) {
            double[] unitK = new double[3];
            unitK[k] = 1;
            xn[k] = nbSm(unitK);                         // *NBSM* of the unit axes
            xm[k] = new double[3];
            for (int c = 0; c < 3; c++) {
                xm[k][c] = 2 * g[k] * g[c] - (k == c ? 1 : 0);   // outer product term minus identity
            }
        }
        for (int k = 0; k < 3; k++) {
            sv("XNB" + (k == 0 ? "" : "+" + k), xn[k]);
            sv("XSM" + (k == 0 ? "" : "+" + k), xm[k]);
        }
        extern("CALCGA");                                // gimbal angles for the 180 deg turn
        sv("STARAD+12", scale(v("GOUT"), 0.5));
        lung();                                          // second orientation
        sv("STARSAV1", unit(add(scale(v("GOUT"), 0.5), v("STARAD+12"))));
        sd("DSPTEM1", Math.acos(clamp(dot(v("STARSAV1"), v("GSAV")) * 2)) / (2 * Math.PI));
        downflag("FREEFLAG");                            // clear in case of recycle
        int r = extern("GOFLASH", VN_GRAVERR);           // V06N04: angular difference
        if (r == TERMINATE) { extern("GOTOPOOH"); return; }
        if (r != PROCEED) upflag("FREEFLAG");            // V32: recycle, store gravity again
        extern("PHASCHNG", PHASE_4_24);
        sv("GSAV", v("STARSAV1"));
        if (flag("FREEFLAG")) {
            gvDeter();
        } else {
            attChk();                                    // exit from GVDETER
        }
    }

    protected double[] nbSm(double[] x) {
        return x.clone();                                // *NBSM*: NB -> SM using SINCDU/COSCDU
    }

    /**
     * LUNG: coarse and fine align to THETAD, then accumulate 2-second PIPA readings (GCTR cycles)
     * into GACC; result unit vector in NB coordinates -> STAR and GOUT.
     */
    private void lung() {
        sv("GACC", new double[3]);
        extern("PHASCHNG", PHASE_4_24);
        coarse();
        set("GCTR", T_HALF_SEC & Agc.WORD_MASK);
        set("1/PIPADT", 0x7FFF & 037777);                // PRIO31 jam
        extern("GCOMPZER");
        extern("PHASCHNG", PHASE_4_24);
        extern("PIPSRINE");                              // initialise PIPAs (data discarded)
        for (;;) {                                       // GREED: 2 sec task GRABGRAV then ADDGRAV
            extern("TWIDDLE", 200, "GRABGRAV");
            extern("PIPSRINE");
            extern("1/PIPA");
            set("GCTR", Agc.add(get("GCTR"), 1));
            sv("GACC", add(v("DELV"), v("GACC")));       // accumulate G vector
            if (Agc.toSigned(get("GCTR")) >= 0) break;
        }
        sv("STAR", unit(v("GACC")));
        extern("CDUTRIG");
        sv("GOUT", nbSm(v("STAR")));                     // *SMNB*: transform and store output
        extern("PHASCHNG", PHASE_4_24);
    }

    // ======================================================================================
    // GYROTRIM
    // ======================================================================================

    /**
     * GYROTRIM: compute and send gyro commands that make the CDUs attain THETAD,+1,+2, assuming
     * the vehicle attitude stays fixed.
     */
    public void gyroTrim() {
        sv("CDUSPOT", new double[] {get("THETAD"), get("THETAD+1"), get("THETAD+2")});  // VDEF
        sv("STARAD", nbSm(new double[] {1, 0, 0}));      // TRG*NBSM(XUNIT)
        sv("STARAD+6", nbSm(new double[] {0, 1, 0}));    // *NBSM*(YUNIT)
        extern("CDUTRIG");
        extern("CALCSMSC");
        sv("6D", v("XNB"));
        sv("12D", v("YNB"));
        axisGen();
        extern("CALCGTA");
        justTrim();
    }

    /** JUSTTRIM: fine-align mode, send gyro pulses, wait. */
    private void justTrim() {
        extern("IMUFINE");
        extern("IMUSTALL");
        extern("IMUPULSE", "OGC");
        extern("IMUSTALL");
    }

    /** AXISGEN: generate the REFSMMAT-like axis triad XDC,YDC,ZDC from vector pairs (external). */
    protected void axisGen() {
        extern("AXISGEN");
    }

    // ======================================================================================
    // Star sightings for P57 (1STAR / 2STARS)
    // ======================================================================================

    /** 2STARS: STARIND = 0 for the first star. */
    public void twoStars() {
        set("STARIND", 0);
        starSighting();
    }

    /** 1STAR: STARIND = 1 (second star / only star). */
    public void oneStar() {
        set("STARIND", 1);
        starSighting();
    }

    private void starSighting() {
        extern("PHASCHNG", PHASE_4_24);
        r59();                                           // star acquire and AOTMARK
    }

    /** R59RET: back from the surface marking; store catalogue vector (VEC1/VEC2) and go on. */
    public void r59Ret() {
        if (get("STARIND") != 0) {                       // second star marked
            extern("PHASCHNG", PHASE_4_24);
            sv("VEC2", planet(d("TSIGHT")));             // 2nd catalogue vector (ref)
            surfLine();
            return;
        }
        sv("STARSAV1", v("STARAD+6"));                   // ASTAR: 1st observed star (SM)
        sv("VEC1", planet(d("TSIGHT")));                 // 1st catalogue vector (ref)
        oneStar();                                       // go get 2nd star sighting
    }

    // ======================================================================================
    // SURFLINE -- fine or coarse align of the IMU
    // ======================================================================================

    /**
     * SURFLINE: express catalogue (VEC1, VEC2) vectors in the desired SM and compare with the
     * observed ones (STARSAV1, STARSAV2); then do the alignment (R54 data check, AXISGEN, gyro
     * torque angles, optional >5 deg coarse alignment).
     */
    public void surfLine() {
        for (int k = 0; k < 2; k++) {                    // WRTDESIR (X2 = 12, 6)
            String c = k == 0 ? "VEC1" : "VEC2";
            String o = k == 0 ? "STARSAV1" : "STARSAV2";
            sv("STARAD+" + (6 * k + 12) , unit(mxv("XSMD", v(c))));
            sv("OBS" + k, v(o));
        }
        sv("6D", v("STARAD+12"));
        sv("12D", v("STARAD+18"));
        if (!flag("INITALGN")) {                         // DOALIGN: skip R54 on initial pass
            chkSData();
            if (!flag("FREEFLAG")) { p57Post(); return; }   // astronaut does not like the data
        }
        axisGen();                                       // INITBY: desired orientation wrt present SM
        extern("CALCGTA");                               // OGC, IGC, MGC
        if ((get("FLAGWRD8") & get("INITABIT")) == 0 && !flag("INITALGN")) {
            int r = extern("GOFLASH", VN_V06N93);        // display gyro torque angles
            if (r == TERMINATE) { extern("GOTOPOOH"); return; }
            if (r != PROCEED) { p57Post1(); return; }    // V32: recycle, maybe re-align
        }
        fiveDegTest();
    }

    /** 5DEGTEST: more than 5 degrees -> COATRIM coarse, else gyro torque only (JUSTTRIM). */
    private void fiveDegTest() {
        double[] ogc = {get("OGC"), get("IGC"), get("MGC")};
        boolean big = false;
        for (double a : ogc) {
            if (Math.abs(Agc.toSigned((int) a)) / 16384.0 > FIVE_DEGREES * 2) big = true;
        }
        if (big) {
            coaTrim();
        } else {
            justTrim();
            surfDisp();
        }
    }

    /** COATRIM: coarse and fine align when the gyro torque angles exceed 5 degrees. */
    private void coaTrim() {
        matMove("XDC", "XSM");
        extern("CDUTRIG");
        extern("CALCSMSC");
        extern("CALCGA");
        if (!flag("INITALGN")) {
            int r;
            do {
                r = extern("GOFLASH", 622);              // V06N22 final gimbal angles
                if (r == TERMINATE) { extern("GOTOPOOH"); return; }
            } while (r == ENTER);
            extern("PHASCHNG", PHASE_4_24);
        }
        coarse();                                        // CORSIT
        nCoarse();
        gyroTrim();
        surfDisp();
    }

    /** SURFDISP: record the new REFSMMAT; finish (option 0) or re-cycle. */
    private void surfDisp() {
        extern("PHASCHNG", PHASE_4_24);
        matMove("XSMD", "REFSMMAT");
        upflag("REFSMFLG");
        if (get("OPTION2") == 0) { p57Post1(); return; }   // CCS OPTION2: zero -> P57POST+1
        if ((get("FLAGWRD8") & get("INITABIT")) != 0) { p57Jump(); return; }   // B2F8: initial flag set
        refMf();                                         // attitude vectors in MF (YNBSAV, ZNBSAV)
        p57Post();
    }

    /** P57POST: V50N25 checklist 14, then compute landing site if aligned by option 2. */
    private void p57Post() {
        int r = extern("GOPERF1", OCT14);
        if (r == TERMINATE) { extern("GOTOPOOH"); return; }
        p57Post1x(r);
    }

    private void p57Post1() {
        p57Post1x(PROCEED);
    }

    private void p57Post1x(int r) {
        if (get("OPTION2") != BIT2) { extern("GOTOPOOH"); return; }   // not aligned by option 2
        extern("PHASCHNG", PHASE_4_24);
        // G(REF) = (REFSMMAT)T (NBSM) GNB; we are on the moon so set LUNAFLAG
        double[] gRef = vxm(sm(v("GSAV")), "REFSMMAT");
        upflag("LUNAFLAG");
        double rlsMag = Math.sqrt(dot(v("RLS"), v("RLS")));
        sv("ALPHAV", scale(gRef, rlsMag));
        downflag("ERADFLAG");
        double t = extern("LOADTIME");
        n89Disp(t);                                      // LS lat/long/alt and RLS back
        sv("RN", v("ALPHAV"));                           // RN = RLS = LM position
        sd("PIPTIME", d("GDT/2+4"));                     // LM state time
        sv("RLS", rToRp(scale(v("RN"), 4), d("GDT/2+4"))); // RLS in moon-fixed coordinates
        extern("GOTOPOOH");                              // exit P57
    }

    /** CDU*NBSM: NB -> SM through present CDU angles (external). */
    private double[] sm(double[] nb) {
        return nbSm(nb);
    }

    // ======================================================================================
    // P57 -- lunar surface IMU alignment
    // ======================================================================================

    /** P57: lunar surface alignment; first select an alignment option. */
    public void p57() {
        extern("IMUCHK");                                // alarm 210 and exit if IMU off
        set("OPTION2", 3);                               // jam REFSMMAT option 3 for initial display
        p57Opt();
    }

    private void p57Opt() {
        int r = extern("GOPERF4R", VN_V05N06);           // flash V04N06 alignment code
        if (r == TERMINATE) { extern("GOTOPOOH"); return; }
        if (r != PROCEED) { p57Opt(); return; }
        extern("PHASCHNG", 00014);
        alignOpt();
    }

    private void alignOpt() {
        switch (get("OPTION2") & 3) {
            case 0: tDisp(); break;                      // option 4: LS orientation
            case 1: packOptn(); break;                   // option 1: preferred
            case 2: p57Opt(); break;                     // option 2 invalid in P57, recycle
            default:
                matMove("REFSMMAT", "XSMD");             // option 3: jam REFSMMAT into XSMD
                packOptn();
        }
    }

    /** TDISP / P57A: display TIG (ascent time) as the alignment time (V06N34). */
    private void tDisp() {
        sd("DSPTEM1", d("TIG"));
        int r;
        do {
            r = extern("GOFLASH", VN_V06N34);
            if (r == TERMINATE) { extern("GOTOPOOH"); return; }
        } while (r == ENTER);                            // P57AA: VB32 recycle
        double now = extern("LOADTIME");
        double key = d("DSPTEM1");
        if (key == 0 || key - now <= 0) {                // P57C: zero/past key-in -> current time
            sd("TALIGN", now);
        } else {
            sd("TIG", key);
            sd("TALIGN", key);
        }
        sd("TDEC1", d("TALIGN"));                        // P57D
        extern("LEMPREC");                               // compute desired IMU orientation
        sv("XSMD", unit(v("RATT")));
        lsOrient();
        packOptn();
    }

    /** PACKOPTN / DSPOPTN: pack option and flag bits for the V05N06 display. */
    private void packOptn() {
        set("OPTION1+1", 0);
        set("OPTION1+2", 0);
        if (flag("REFSMFLG")) set("OPTION1+2", get("OPTION1+2") + BIT7);
        if (flag("ATTFLG")) set("OPTION1+2", get("OPTION1+2") + BIT4);
        set("OPTION1", BIT4);                            // 00010 for the checklist
        dspOptn();
    }

    private void dspOptn() {
        int r;
        do {
            r = extern("GOFLASH", VN_V05N06);
            if (r == TERMINATE) { extern("GOTOPOOH"); return; }
        } while (r == ENTER);                            // V32: recycle
        if (flag("REFSMFLG")) { getLmAtt(); return; }    // set: compute LM attitude
        if (flag("ATTFLG") || (get("OPTION2") & BIT2) != 0) { byLmAtt(); return; }
        extern("ALARM", 0701);                           // option inconsistent with flags
        r = extern("GOFLASH", 509);                      // V05N09
        if (r == TERMINATE) { extern("GOTOPOOH"); return; }
        dspOptn();                                       // proceed or recycle -> option display
    }

    /** MFREF: transform VEC1,2 from moon fixed to reference coordinates. */
    public void mfRef() {
        double t = extern("LOADTIME");
        sd("TSIGHT", t);
        sv("VEC1", rpToR(v("VEC1"), t));
        sv("VEC2", rpToR(v("VEC2"), t));
    }

    /** REFMF: compute LM attitude in moon-fixed coordinates (YNBSAV, ZNBSAV); sets ATTFLAG. */
    public void refMf() {
        extern("CDUTRIG");
        double t = extern("LOADTIME");
        sd("TSIGHT", t);
        extern("CALCSMSC");                              // YNB, ZNB in SM
        sv("YNBSAV", rToRp(unit(vxm(v("YNB"), "REFSMMAT")), t));
        sv("ZNBSAV", rToRp(unit(vxm(v("ZNB"), "REFSMMAT")), t));
        upflag("ATTFLAG");
    }

    private void getLmAtt() {
        refMf();
        byLmAtt();
    }

    /** BYLMATT: set INITALGN; options 1 or 3 -> gravity determination else attitude check. */
    private void byLmAtt() {
        upflag("INITALGN");
        if ((BIT1 & get("OPTION2")) != 0) {
            gvDeter();                                   // options 1 or 3 get gravity
        } else {
            attChk();
        }
    }

    /** ATTCHK: exit from GVDETER; use stored attitude if ATTFLG is set. */
    public void attChk() {
        extern("PHASCHNG", PHASE_4_24);
        if (flag("ATTFLG")) { p57Opt0(); return; }
        p57Jump();
    }

    /** P57JUMP: branch on option code. */
    private void p57Jump() {
        extern("PHASCHNG", PHASE_4_24);
        downflag("INITALGN");
        switch (get("OPTION2") & 3) {
            case 0: p57Opt0(); break;
            case 1: p57Opt1(); break;
            case 2: twoStars(); break;                   // P57OPT2: sight on 2 stars
            default: p57Opt3(); break;
        }
    }

    /** OPTION 0: two attitude vectors (Y and Z axes) from the stored attitude. */
    private void p57Opt0() {
        sv("VEC1", v("YNBSAV"));
        sv("VEC2", v("ZNBSAV"));
        extern("CDUTRIG");
        extern("CALCSMSC");
        sameType(v("YNB"), v("ZNB"));
    }

    /** OPTION 1: landing site vector and Z attitude vector. */
    private void p57Opt1() {
        sv("VEC1", unit(v("RLS")));
        sv("VEC2", v("ZNBSAV"));
        extern("CDUTRIG");
        extern("CALCSMSC");
        sameType(sm(v("GSAV")), v("ZNB"));
    }

    /** SAMETYP: observed vectors STARSAV1/2 in present SM, transform VEC1,2 MF -> REF, SURFLINE. */
    private void sameType(double[] obs1, double[] obs2) {
        sv("STARSAV1", obs1);
        sv("STARSAV2", obs2);
        mfRef();
        surfLine();
    }

    /** OPTION 3: landing site vector and one star sighting. */
    private void p57Opt3() {
        sv("VEC1", unit(v("RLS")));
        sv("VEC2", v("VEC1"));                           // dummy VEC2 for 2nd catalogue star
        sv("STARSAV1", sm(v("GSAV")));
        mfRef();
        oneStar();                                       // 1STAR gets VEC2; STARSAV2 -> SURFLINE
    }

    // ======================================================================================
    // IMUCHK and LSORIENT
    // ======================================================================================

    /** IMUCHK: verify (IMODES30 bit 9) the IMU is on; alarm 210 and terminate otherwise. */
    public boolean imuChk() {
        if ((get("IMODES30") & BIT9) == 0) {             // bit 9 clear = IMU on
            upflag("IMUSE");
            return true;
        }
        extern("ALARM", 0210);
        extern("GOTOPOOH");
        return false;
    }

    /**
     * LSORIENT: landing-site orientation. With XSMD = UNIT(RLS), ZSMD = UNIT(VRECTCSM x XSMD ...);
     * (RRECTCSM x VRECTCSM) x XSMD, YSMD = UNIT(ZSMD x XSMD).
     */
    public void lsOrient() {
        double[] z = unit(cross(cross(v("RRECTCSM"), v("VRECTCSM")), v("XSMD")));
        sv("ZSMD", z);
        sv("YSMD", unit(cross(z, v("XSMD"))));
    }

    /** R51P63: R51 as used by P63 - request fine align (V50N25 checklist 14), then R51C; returns to QMAJ. */
    public void r51p63() {
        int r = extern("GOPERF1", 014);
        if (r == TERMINATE) { extern("GOTOPOOH"); return; }
        r51c();
    }
}
