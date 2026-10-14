package apollo.comanche055;

import apollo.agc.Agc;

/**
 * TPI_SEARCH.agc: S17.1 and S17.2 (TPI search) and the P17/P77 display routine.
 * <p>
 * The TPI search determines the minimum total velocity transfer trajectory from a given
 * TPI maneuver time within the constraint of a safe pericenter (sum of impulsive velocities
 * for TPI and TPF). S17.1 extrapolates both state vectors to TPI time and computes the
 * relative phase angle, altitude difference and the search sector (sign of altitude
 * difference). S17.2 computes angular limits, uses TIMETHET for search start/end times and
 * iterates with Lambert (INITVEL) until solution criteria are met (or no solution exists:
 * ALARUMS, alarm 00124).
 * <p>
 * Interpretive vector arithmetic is done with doubles in natural units; scale factors are
 * recorded in comments.
 */
public final class TpiSearch {
    private final Agc agc;

    // ---- constants (bank 36, P17S1)
    /** HPE: Earth's minimum pericenter altitude 85 n.m. (meters). */
    public static final double HPE = 157420.0;
    /** HPL: Moon's minimum pericenter altitude 35000 ft (meters). */
    public static final double HPL = 10668.0213;
    public static final double CDSEC = 40000;
    public static final double CLSEC = 15000;
    public static final double PIINVERS = 0.3183098862;
    /** 70 degrees. */
    public static final double SEC1THET = 0.1944444444;
    /** 330 degrees. */
    public static final double SEC2THET = 0.9166666667;
    public static final double MANYFEET = -1.0 / 4;
    /** LIMVEL: 2 fps in m/s. */
    public static final double LIMVEL = 0.6096e-2;
    /** DFTMOON: 500 feet. */
    public static final double DFTMOON = 0.1524e3;
    public static final double DP_MINUS_002 = 0.002;
    public static final double DPPOSMAX = 1.0 - Math.pow(2, -28);

    // ---- display verb/noun codes
    public static final String V06N37 = "0637";
    public static final String V06N72 = "0672";
    public static final String V06N58 = "0658";
    public static final String V06N55 = "0655";
    public static final String V05N09 = "0509";
    /** Alarm code: no safe pericenter in this sector. */
    public static final int ALARM_NO_SAFE_PERICENTER = 0124;

    /** External services used by the routines (conic extrapolation, Lambert, etc.). */
    public interface Services {
        /** Advance the passive (LEM) vehicle to time t; returns {RATT, VATT}. */
        double[][] lemconic(double t);
        /** Advance the active (CSM) vehicle to time t; returns {RATT, VATT}. */
        double[][] csmconic(double t);
        /** TIMETHET: time to travel from RVEC/VVEC through angle given by SNTH/CSTH; returns T. */
        double timethet(double[] rvec, double[] vvec, double snth, double csth);
        /** INITVEL (Lambert): returns {VTPRIME, VIPRIME, DELVEET3}. */
        double[][] initvel(double[] rinit, double[] vinit, double[] rtarg, double dellt4);
        /** PERIAPO: pericenter altitude for RVEC/VVEC (meters). */
        double periapo(double[] rvec, double[] vvec);
    }

    // ---- state (erasable symbols)
    public double[] ract3 = new double[3], vact3 = new double[3];
    public double[] rpass3 = new double[3], vpass3 = new double[3];
    public double[] e2 = new double[3];
    public double delhite, thetzero, it, snth, csth, tf, tfo, tfi, thetl, hp, hpo, delvee, delveo;
    public double deltee, hpermin, ttpi, relDelv, magvtpi, centang, nn1;
    public int xrs;
    public boolean kflag, avflag, itswich, rvsw, updatflg;
    public double p21onenn = 1;
    public double[] vtprime = new double[3], viprime = new double[3];

    private Services svc;
    private int option2;

    public TpiSearch(Agc agc) {
        this.agc = agc;
    }

    public void setServices(Services s) { this.svc = s; }

    /**
     * S17.1: extrapolate both vehicles to TTPI, compute altitude difference DELHITE, central
     * angle THETZERO, search sector flag KFLAG (OFF = active vehicle lower, + ; ON = -) and
     * XRS (index: +=active ahead, -=active behind).
     */
    public void s17_1() {
        double[][] l = svc.lemconic(ttpi);                // passive vehicle to TPI
        rpass3 = l[0]; vpass3 = l[1];                      // LEMSTORE
        double[][] c = svc.csmconic(ttpi);                // active vehicle to TPI
        ract3 = c[0]; vact3 = c[1];                        // CSMSTORE
        double ra = norm(ract3), rp = norm(rpass3);
        // |RP| - |RA| : KFLAG off = + ; on = -
        double dh = rp - ra;
        kflag = dh < 0;
        delhite = dh;
        e2 = unit(cross(rpass3, vpass3));                  // orbit normal, saved for S17.2
        // project RP into plane of RA: angle between RA and RPA
        double[] rpa = unit(sub(rpass3, scale(e2, dot(rpass3, e2))));
        double ang = Math.acos(clamp(dot(rpa, unit(ract3)))) / (2 * Math.PI);   // revolutions
        double sign = dot(cross(ract3, rpa), e2);          // active ahead / behind
        thetzero = sign >= 0 ? ang : -ang;                 // CENTRAL ANGLE
        xrs = sign >= 0 ? 0 : 2;                           // index saved for further use
    }

    /**
     * S17.2: compute search-sector limits, search loop on the Lambert subroutine; normal exit via
     * POSTTPI, error exit via ALARUMS.
     * @return true on normal exit, false if no safe pericenter (ALARUMS)
     */
    public boolean s17_2() {
        // sector limits: sin/cos of the angle limits, with .002 rad added
        double sinLim = Math.sqrt(Math.abs(dot(unit(ract3), e2))) * 2 + DP_MINUS_002;
        if (kflag) sinLim = -sinLim;
        // PHI(I) = -(THETZERO + K2 IT) ; PHI(0) = -(THETZERO + K5 IT)
        double piRev = sinLim * PIINVERS;
        it = piRev - thetzero;
        double phi0 = it / 2 + it + thetzero;               // SR1, DAD, DAD
        rvsw = true;
        snth = Math.sin(phi0 * 2 * Math.PI);
        csth = Math.cos(phi0 * 2 * Math.PI);
        if (csth < 0) csth = -csth;
        double[] rvec = scale(rpass3, 1.0 / (1 << xrs));
        double[] vvec = scale(vpass3, 1.0 / (1 << xrs));
        tf = svc.timethet(rvec, vvec, snth, csth);          // search start time
        tfo = tf;
        snth = Math.sin(it * 2 * Math.PI);
        csth = Math.abs(Math.cos(it * 2 * Math.PI));
        double tEnd = svc.timethet(rvec, vvec, snth, csth); // search end time
        // initialise loop
        itswich = false;
        tfi = tEnd;
        delvee = DPPOSMAX;
        hp = MANYFEET;
        thetl = kflag ? SEC2THET : SEC1THET;                // 330 or 70 degrees
        return bis(rvec, vvec);
    }

    /** BIS: loop head. */
    private boolean bis(double[] rvec, double[] vvec) {
        while (true) {
            // get 4-quadrant THETA via ARCTRIG on (SNTH, CSTH); put between 0,1
            double theta = Math.atan2(snth / 2, csth / 2) / (2 * Math.PI);
            if (theta < 0) theta += DPPOSMAX;
            double t1 = thetl - theta;
            if (Math.signum(tf - tfi) * Math.abs(t1) < 0 && tf - tfi < 0) {
                return rngetest();                          // time must have a stop
            }
            // CONCAUL: advance passive vehicle for target vector
            double[][] tgt = avflag ? svc.csmconic(ttpi + tf) : svc.lemconic(ttpi + tf);
            double[] vpass4 = tgt[1], rpass4 = tgt[0];       // JUNCT3
            double dellt4 = tf;
            hpo = hp;
            delveo = delvee;
            double[][] lam = svc.initvel(ract3, vact3, rpass4, dellt4);
            double[] vtp = lam[0];
            vtprime = vtp; viprime = lam[1];
            double[] delveet3 = lam[2];
            relDelv = norm(sub(vtprime, vpass4));            // /V2 - VP(TPI+TF)/
            magvtpi = norm(delveet3);                         // /V1 - VA/
            delvee = magvtpi + relDelv;
            hp = svc.periapo(ract3, viprime);                 // PERIAPO
            if (itswich) return enden(vpass4, rpass4);        // interpolation: acceptance forced
            if (hpermin - hp >= 0 && hp - hpermin < 0 || hp < hpermin) {
                // pericenter not safe: HALFSAFE logic or step
                double hpDiff = hp - hpo;                     // (HPLIM-HPO)-(HPLIM-HP)
                if (hpDiff < 0 && hpo < hpermin && hp >= hpermin) {
                    interp();
                } else if (hpDiff < 0) {
                    return alarums();                         // getting worse: sound the alarm
                } else {
                    deltee = CDSEC;                           // junct1
                    if (!kflag) deltee = -deltee;
                }
            } else {
                // HALFSAFE: pericenter is safe
                if (Math.abs(delvee - delveo) - LIMVEL < 0) return enden(vpass4, rpass4);
                double dh = hpermin - hpo;
                if (dh < 0) {
                    if (deltee < 0 || true) {
                        deltee = -deltee / 2;
                    }
                } else {
                    interp();
                }
            }
            tf += deltee;                                     // JUNCT2: recycle
        }
    }

    /** INTERP: interpolate time step to hit the pericenter limit (secant), sets ITSWICH. */
    private void interp() {
        itswich = true;
        double dHp = hp - hpo;
        double num = (hpermin - hp) + DFTMOON;
        if (dHp != 0) {
            deltee = deltee * (num / dHp);
        }
    }

    /** RNGETEST: time ran out; assume solution if pericenter is safe, else ALARUMS. */
    private boolean rngetest() {
        if (hp - hpermin < 0) return alarums();
        tf = tf - deltee;                                     // time of solution
        return enden(vpass4Last(), rpass4Last());
    }

    private double[] vpass4Last() { return vpass3; }
    private double[] rpass4Last() { return rpass3; }

    /** ENDEN: finalise: sign of RELDELV, central angle, and NN1 (pericenter code). */
    private boolean enden(double[] vpass4, double[] rpass4) {
        double sg2 = dot(vtprime, rpass4);
        relDelv = Math.copySign(relDelv, sg2);                // SIGN(RELDELV) = SIGN(SG2)
        transang(vpass4, rpass4);                             // central angle
        double sg1 = dot(ract3, viprime);
        double dectwo = 2;
        double n;
        if (sg1 * relDelv >= 0) {                             // USEKAY
            n = kflag ? dectwo - p21onenn : dectwo;
        } else {
            n = -dectwo;
            n = (n * relDelv >= 0) ? n : -n;
            n = kflag ? n - p21onenn : n;
        }
        nn1 = n;
        // STCALL POSTTPI via QTEMP
        return true;
    }

    /**
     * TRANSANG: central angle of travel of the passive vehicle during the transfer:
     * CENTANG = sqrt(MU/ASUBP***) * TF in revolutions, with ASUBP*** = R/(2 - R V**2/MU) cubed.
     */
    public void transang(double[] vpass4, double[] rpass4) {
        double mu = 3.986032e14;                              // MUTABLE (Earth; Moon uses 4.902778e12)
        double r = norm(rpass4), v2 = dot(vpass4, vpass4);
        double a = r / (2 - r * v2 / mu);                     // semi-major axis
        centang = Math.sqrt(mu / (a * a * a)) * tf / (2 * Math.PI);
    }

    /** ALARUMS: set UPDATFLG; alarm 00124 (no safe pericenter); display V05N09. */
    public boolean alarums() {
        updatflg = true;
        agc.set("ALARM", ALARM_NO_SAFE_PERICENTER);
        return false;
    }

    // ------------------------------------------------------------------ P17 / P77 display

    /** P17: AVFLAG = CSM, set track and update flags. */
    public void p17() {
        avflag = true;                                         // AVFLAGA
        p17_1();
    }

    /** P77: AVFLAG = LEM. */
    public void p77() {
        avflag = false;                                        // AVFLAGP
        p17_1();
    }

    /** P17.1: set update flag, display TTPI (V06N37), run S17.1, display phi, delta H, option K. */
    public void p17_1() {
        updatflg = true;                                       // P20FLGON
        // CAF V06N37 / VNPOOH (display TTPI time)
        updatflg = false;
        s17_1();
        updatflg = true;
        option2 = kflag ? 1 : 2;                               // DELTA H = 2 (K positive), 1 (K negative)
        // CAF V06N72: display PHI, DELTA H, search option K (VNCOMP17)
        updatflg = false;
        kflag = true;
        if (option2 - (int) p21onenn == 0) kflag = false;      // KFLAG on for option 1, off for option 2
        double hpl = (xrs + 1) == 0 ? HPL : HPE;               // moon vs earth minimum pericenter
        hpermin = hpl;
        if (!s17_2()) {
            return;                                            // ALARUMS handled
        }
        updatflg = true;
        p17_3();
    }

    /**
     * P17.3: display DELTA VTPI, DELTA VTPF and H (V06N58) then pericenter code and central
     * angle (V06N55). Terminate/end -> GOTOPOOH, recycle -> P17.1, proceed -> blank R2, ENDOFJOB.
     */
    public void p17_3() {
        // CAF V06N58 ; VNCOMP17
        // CAF V06N55 ; BANKCALL GOFLASHR
        // responses: terminate/end -> GOTOPOOH ; recycle -> P17.1 ; proceed -> CAF TWO, BLANKET, ENDOFJOB
    }

    /**
     * VNCOMP17: display verb/noun via GOFLASH; terminate illegal redisplay; PROCEED returns to
     * caller; RECYCLE (V32) with new TPI time or search option goes to P17.1 when MPAC==BIT6.
     */
    public boolean vncomp17(String verbNoun) {
        agc.set("VERBNOUN", Integer.parseInt(verbNoun, 8));
        return true;
    }

    // ------------------------------------------------------------------ vector helpers
    private static double dot(double[] a, double[] b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
    private static double norm(double[] a) { return Math.sqrt(dot(a, a)); }
    private static double[] unit(double[] a) { double m = norm(a); return m == 0 ? a.clone() : scale(a, 1 / m); }
    private static double[] scale(double[] a, double s) { return new double[] {a[0]*s, a[1]*s, a[2]*s}; }
    private static double[] sub(double[] a, double[] b) { return new double[] {a[0]-b[0], a[1]-b[1], a[2]-b[2]}; }
    private static double clamp(double x) { return Math.max(-1, Math.min(1, x)); }
    private static double[] cross(double[] a, double[] b) {
        return new double[] {a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]};
    }
}
