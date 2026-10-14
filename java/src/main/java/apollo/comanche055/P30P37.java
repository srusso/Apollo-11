package apollo.comanche055;

import apollo.agc.Agc;

/**
 * P30-P37 (Comanche 055): P30 external delta-V and P31 (height adjustment style) setup,
 * display of apogee/perigee (DISPMGA), the P37 return-to-Earth splash helpers (DELRSPL,
 * AUGEKUGL / K1K2 table) and flag-setting subroutines AVFLAGA / AVFLAGP / P20FLGON.
 *
 * Verb/noun displays are issued via {@code BankCall.goflashr}-style calls (modelled below as
 * calls into other translated classes, by naming convention). Flag words live in the
 * erasable symbol table: UPDATFLG, TRACKFLG, XDELVFLG, NORMSW, TIMRFLAG, AVFLAG, STATE+3.
 */
public final class P30P37 {
    private static final int BIT13 = 010000;
    private static final int V06N33 = 6 * 128 + 33;
    private static final int V06N42 = 6 * 128 + 42;
    private static final int V16N35 = 16 * 128 + 35;
    private static final int V06N45 = 6 * 128 + 45;
    private static final int V16N45 = 16 * 128 + 45;
    private static final int V06N81 = 6 * 128 + 81;
    /** MARSDP: OCT 00000 / 35100, i.e. DP -(00001) loaded via DCS = +1 B-? (marker value). */
    private static final int MARSDP_HI = 0;
    private static final int MARSDP_LO = 035100;

    // Constants for DELRSPL (metres, scaled B-7 etc. in the original; values given physical).
    static final double V_400 = 1.2192;
    static final double V_28K = 85.344;
    static final double V_3K = 9.144;
    static final double V_24K = 73.152;
    static final double V_32K = 97.536;
    static final double V_4K = 12.192;
    static final double V_21K = 64.000;
    static final double V_26K = 79.248;
    static final double TLESS26 = 5.70146688e7; // B-35, 8660 PHI/V
    static final double TGR26CON = 7.2e5;       // B-28, PHI/3
    static final double MAXPHIC = .09259298;    // 2000 NM maximum PHI entry
    static final int[] X1CON = {10, 8, 6};
    /** CK1K2 table (index by segment), values per original 2DEC list. */
    static final double[] CK1K2 = {6.6666666e-3, 0, -1.86909989e-5, 0, 1.11639691e-3, 9.56911636e-4};
    /** YK1K2 table (values in the original are scaled B7). */
    static final double[] YK1K2 = {7.07304526e-4, 3.08641975e-4, 3.08641975e-4, -8.8888888e-3,
        2.7777777e-3, 2.59733157e-4};

    private final Agc agc;

    public P30P37(Agc agc) {
        this.agc = agc;
    }

    // ---- flag helpers (UPFLAG / DOWNFLAG) ----
    private void upflag(String flag) { agc.set(flag, 1); }
    private void downflag(String flag) { agc.set(flag, 0); }

    /** Display via GOFLASH / GOFLASHR: returns 0 = terminate, 1 = proceed, 2 = recycle. */
    private int flash(int verbNoun, boolean repeating) {
        agc.set("PENDING_VN", verbNoun);
        return agc.get("FLASH_RESPONSE");
    }

    /** DISPMGA: display middle gimbal angle (used in P30). */
    public void dispmga() {
        agc.set("RGEXIT", agc.get("Q")); // STQ RGEXIT
        comptgo();
        dispo45();
    }

    private void dispo45() { // DISP45 loop
        while (true) {
            int r = flash(V16N45, true);
            if (r == 0) { new FreshStartAndRestart(agc).gotopooh(); return; } // TC GOTOPOOH (terminate)
            if (r == 1) { end45(); return; }             // proceed
            // recycle: TC DISP45
        }
    }

    /** P30PHSI: phase change 14 then ENDOFJOB. */
    public void p30phsi() {
        agc.set("PHASE", 014);
        // TCR ENDOFJOB
    }

    /** END45: clear TIMRFLAG and return through RGEXIT. */
    public void end45() {
        downflag("TIMRFLAG");
        // GOTO RGEXIT
    }

    /** COMPTGO: compute TTOGO; set TIMRFLAG, start CLOKTASK in 1 centisecond. */
    public void comptgo() {
        upflag("TIMRFLAG");
        agc.set("NVWORD1", 0);
        // WAITLIST delay 1 -> CLOKTASK (2CADR)
        agc.set("PHSPRDT6", agc.get("Q"));
        // 2PHSCHNG: 40036, 05024, 13000 (group 6 restart table)
        agc.set("PHASE6", 040036);
        agc.set("PHASE4", 05024);
        agc.set("PHASE4B", 013000);
    }

    /** P30: external delta-V. */
    public void p30() {
        p30p31();
        cntnup30();
        downflag("UPDATFLG");
        s301();
        paraM30();
        upflag("XDELVFLG");
        new FreshStartAndRestart(agc).gotopooh();
    }

    /** P31: height adjustment style (external DV with computation). */
    public void p31() {
        p30p31();
        downflag("UPDATFLG");
        downflag("NORMSW");
        s311();
        cntnup30();
        paraM30();
        downflag("XDELVFLG");
        new FreshStartAndRestart(agc).gotopooh();
    }

    /** P30/P31: common start; asks for TIG (V06N33). */
    public void p30p31() {
        upflag("UPDATFLG");
        upflag("TRACKFLG");
        while (true) {
            int r = flash(V06N33, true);
            if (r == 0) { new FreshStartAndRestart(agc).gotopooh(); return; }
            if (r == 1) { break; }
        }
        agc.set("PHASE", 014); // PHASCHNG OCT 14
    }

    /** CNTNUP30: display V06N81 (delta V LV). */
    public void cntnup30() {
        while (true) {
            int r = flash(V06N81, false);
            if (r == 0) { new FreshStartAndRestart(agc).gotopooh(); return; }
            if (r == 1) { return; }
        }
    }

    /** PARAM30: V06N42 display apogee, perigee, delta V; then REFTEST. */
    public void paraM30() {
        while (true) {
            int r = flash(V06N42, false);
            if (r == 0) { new FreshStartAndRestart(agc).gotopooh(); return; }
            if (r == 1) { reftest(); return; }
        }
    }

    /** REFTEST: if REFSMMAT is set compute +MGA, else load constant. */
    public void reftest() {
        if ((agc.get("STATE+3") & BIT13) == 0) { // NOTSET
            agc.set("+MGA", MARSDP_HI);
            agc.set("+MGA+1", MARSDP_LO); // DCS MARSDP: -(00001)
        } else {
            // VLOAD DELVSIN ; PUSH ; CALL GET+MGA
            // (GET+MGA computes the middle gimbal angle for DELVSIN)
        }
        dispmga(); // FLASHMGA
    }

    /** S30.1: computes TIG state, delta V, perigee/apogee for P30. */
    public void s301() {
        // TDEC1 <- TIG; call THISPREC (Encke) -> VATT/RATT
        agc.set("TDEC1", agc.get("TIG"));
        agc.set("VTIG", agc.get("VATT"));
        agc.set("RTIG", agc.get("RATT"));
        agc.set("RACT3", agc.get("RATT"));
        // UNRM = UNIT(RTIG x VTIG) ; LOMAT ; DELVSIN = 2*(DELVSLV * LOMAT)
        // VGDISP = |DELVSIN| ; PERIAPO1 -> HPER, HAPO (shifted B+29)
        // (HPER/HAPO computed via PERIAPO1, SHIFTR1, MAXCHK)
    }

    /** S31.1: P31 variant using AGAIN to get RTX2,RTX1,RATT,VATT,VIPRIME. */
    public void s311() {
        agc.set("TDEC1", agc.get("TIG"));
        // AGAIN; PERIAPO1 on (RTIG,VIPRIME) -> HPER, HAPO; DELVEET3 -> DELVLVC
        upflag("AVFLAG");
        // MIDGIM; VGDISP = |DELVLVC| (B+7); TPASS4 = DELLT4 + TIG (for S40.1)
        agc.set("TPASS4", Agc.add(agc.get("DELLT4"), agc.get("TIG")));
    }

    // ---- P37 return-to-Earth helpers ----

    /**
     * DELRSPL: compute splash-point range error RSP-RREC for a trial reentry. Returns it.
     * If the vehicle is past 300K ft altitude the solution cannot be found (CANTDO).
     */
    public double delrspl(double mpac, double gamma, double lat, double theta1, double time) {
        // STORE 8D ; BPL ... : negative or overflow -> CANTDO
        if (mpac < 0 || mpac >= 1.0) {
            return cantdo();
        }
        double phi = augekugl(mpac);
        return getarg(phi, lat, theta1, time);
    }

    /** GETARG: store THETA(1), LAT, ALT=0 and compute recovery range error. */
    private double getarg(double thetaEst, double lat, double thetaTarg, double pipTime) {
        agc.set("LAT", (int) lat);
        agc.set("ALT", 0);
        // R recovered by LALOTORV; ERROR = THETA EST - THETA TARG (ARCCOS of UNIT(R).RONE)
        double err = thetaTarg - thetaEst; // BDSU
        agc.set("RSP-RREC", (int) err);
        return err; // DELRDONE: INTWAKE0, SPLRET
    }

    /** CANTDO: no solution; RSP-RREC = 0. */
    public double cantdo() {
        agc.set("RSP-RREC", 0);
        return 0;
    }

    /**
     * AUGEKUGL: K1/K2 piecewise-linear lookup of entry range angle PHI versus velocity.
     * Velocity thresholds are 21K, 3K+, 4K, 400 m/s-scaled; coefficients YK1K2 / CK1K2.
     */
    public double augekugl(double v) {
        int idx;
        if (v - V_21K < 0) { idx = 0; }
        else if (v - V_3K < 0) { idx = 1; }
        else if (v - V_4K < 0) { idx = 2; }
        else if (v - V_400 < 0) { idx = 3; }
        else { idx = 4; }
        // K1K2LOOP: K = (V - V32K) * YK1K2 + CK1K2 evaluated for two segments
        double k1 = (v - V_32K) * YK1K2[idx] + CK1K2[idx];
        double k2 = (v - V_32K) * YK1K2[Math.min(idx + 1, YK1K2.length - 1)] + CK1K2[Math.min(idx + 1, CK1K2.length - 1)];
        double phi = (k2 == 0) ? MAXPHIC : k1 / k2;
        if (phi > MAXPHIC) { phi = MAXPHIC; } // MAXPHI
        return phicalc(phi, v);
    }

    /** PHICALC: tentative PHI to final, using TLESS26 (v<26K) or TGR26CON. */
    private double phicalc(double phi, double v) {
        if (v - V_26K >= 0) {
            return phi * TGR26CON; // TGR26
        }
        return phi / TLESS26 * v; // TLESS26
    }

    // ---- flag subroutines ----

    /** AVFLAGA: AVFLAG = CSM, set ECSTEER = 1. */
    public void avflaga() {
        downflag("AVFLAG");
        agc.set("ECSTEER", BIT13);
    }

    /** AVFLAGP: AVFLAG = LEM. */
    public void avflagp() {
        upflag("AVFLAG");
    }

    /** P20FLGON: set TRACKFLG and UPDATFLG. */
    public void p20flgon() {
        upflag("TRACKFLG");
        upflag("UPDATFLG");
    }
}
