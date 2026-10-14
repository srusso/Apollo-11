package apollo.comanche055;

import apollo.agc.Agc;

/**
 * CM_BODY_ATTITUDE.agc (bank 35, BODYATT, COUNT 37/CMBAT).
 * Computes CM body attitude (roll, beta, alpha) and flight-path angle gamma relative to
 * the relative-velocity vector at PIPA time (CM/POSE), and updates them (CM/ATUP).
 * Angles are stored as -(angle/180)/2 style scaled values (one's-complement revolutions).
 */
public final class CmBodyAttitude {
    private final Agc agc;

    /** -KVSCALE: -12800/(2 VS .3048) */
    public static final double MINUS_KVSCALE = -0.81491944;
    /** TCDU = .1 SEC. */
    public static final double TCDU = 0.1;
    /** SPVQUIT: 1000/2VS, used to hold UYA stable at the end of terminal phase. */
    public static final double SPVQUIT = 0.019405;
    /** REPOSADR 2CADR REDOPOSE: restart address. */
    public static final String REPOSADR = "REDOPOSE";

    public CmBodyAttitude(Agc agc) {
        this.agc = agc;
    }

    /** Output of CM/POSE: unit vectors in reference coordinates and angles (revolutions). */
    public static final class Pose {
        public double[] uxa = new double[3], uya = new double[3], uza = new double[3];
        public double[] ubx = new double[3], uby = new double[3], ubz = new double[3];
        public double[] minusVrel = new double[3];
        public double vrelMag;
        /** -(ROLL/180)/2, -(BETA/180)/2, -(ALFA/180)/2, -(GAMA/180)/2 : stored in MPAC as pushdown 6D..10D. */
        public double roll, beta, alfa, gama;
    }

    private double[] oldUya = {0, 1, 0};

    /**
     * CM/POSE: come here via AVEGEXIT. Builds the trajectory triad (UXA, UYA, UZA) from
     * relative velocity, builds the body triad (UBX, UBY, UBZ) from CDU angles at PIP time
     * (AOG/PIP, AIG/PIP, AMG/PIP) and REFSMMAT, then extracts roll/beta/alpha/gamma.
     *
     * @param vn      velocity (2^-7 m/cs), @param unitw earth rate unit vector, @param unitr unit radius
     * @param kwe     earth rate constant, @param cdu {AOG,AIG,AMG} at PIP time, in revolutions
     */
    public Pose cmPose(double[] vn, double[] unitw, double[] unitr, double kwe,
                       double[] cdu, double[][] refsmmat, double kvscale) {
        Pose p = new Pose();
        // VREL = V - WE*R  (VXSC -KVSCALE; VXV UNITR; VXSC KWE; VAD)
        double[] vrel = new double[3];
        double[] wxr = cross(unitw, unitr);
        for (int i = 0; i < 3; i++) vrel[i] = (vn[i] * -kvscale) + (wxr[i] * kwe);
        for (int i = 0; i < 3; i++) p.minusVrel[i] = vrel[i];    // STORE -VREL
        double mag = norm(vrel);
        p.vrelMag = mag;                                          // ABVAL(-VREL) to X1
        p.uxa = unit(p.minusVrel);                                // UXA/2 = -UVREL
        double[] uya = unit(neg(cross(p.uxa, unitr)));
        // UN is held stable at end of terminal phase: if V - VQUIT is positive save UYA
        if (mag * 1.0 - SPVQUIT > 0) {
            oldUya = uya.clone();
        } else {
            uya = oldUya.clone();
        }
        p.uya = uya;
        p.uza = neg(cross(uya, p.uxa));                           // trajectory triad completion

        // CM/TRIO: CDU angles at PIPUP time
        double aog = cdu[0] * 2 * Math.PI, aig = cdu[1] * 2 * Math.PI, amg = cdu[2] * 2 * Math.PI;
        double ci = Math.cos(aig), si = Math.sin(aig);
        double cm = Math.cos(amg), sm = Math.sin(amg);
        double co = Math.cos(aog), so = Math.sin(aog);
        // YB = (-CO*SM*CI + SO*SI, CO*CM, CO*SM*SI + SO*CI)  (platform coordinates)
        double[] ybPlat = {-sm * ci * co + so * si, cm * co, co * sm * si + so * ci};
        // XB = (CM*CI, SM, -CM*SI)
        double[] xbPlat = {cm * ci, sm, -cm * si};
        p.uby = vxm(refsmmat, ybPlat);
        p.ubx = vxm(refsmmat, xbPlat);
        p.ubz = cross(p.ubx, p.uby);                              // ZB = XB x YB

        // UL = unit(-UVREL x UBY)
        double[] ul = unit(cross(p.uxa, p.uby));
        double cosTh = dot(ul, p.uza);                            // COS(ROLL)
        double sinTh = dot(ul, p.uya);                            // -SIN(ROLL)
        double rollRev = arctrig(sinTh, cosTh);                   // ARCTRIG
        p.roll = rollRev;
        p.beta = Math.asin(clamp(dot(p.uby, p.uxa))) / (2 * Math.PI);  // -(BETA/180)/2 sign kept per original
        p.alfa = arctrig(dot(ul, p.ubx), dot(ul, p.ubz));
        p.gama = Math.acos(clamp(dot(unitr, p.uza))) / (2 * Math.PI);
        return p;
    }

    /** ARCTRIG helper: angle in revolutions from sine and cosine. */
    private static double arctrig(double sin, double cos) {
        return Math.atan2(sin, cos) / (2 * Math.PI);
    }

    /**
     * CM/ATUP: basic subroutine to update attitude angles. Differences gamma to get GAMDOT
     * (ignored if &lt;= 0.5 deg/sec), then corrects Euler angles for accrued increment since
     * PIPUP: R = R_EUL + R(now) - R(PIPUP), written to TEMPROLL/TEMPALFA/TEMPBETA, followed
     * by a phase change (REPOSADR) and REDOPOSE which copies them to ROLL/180, ALFA/180, BETA/180.
     */
    public void cmAtup() {
        // CMTR1: GAMA update
        int oldGama = agc.get("GAMA");
        int newGama = Agc.fromSigned(-Agc.toSigned(agc.get("MPAC10D")));   // CS 10D : GAMA/180
        agc.set("GAMA", newGama);
        // INHINT: must remain inhibited until update of body angles so GAMDIFSW is valid
        final int BIT11 = 0x400;
        int flags = agc.get("CM/FLAGS");
        boolean noGamdot = false;
        if ((flags & BIT11) == 0) {
            // GAMDIFSW=0: do not calc GAMA dot until one difference formed; set switch
            agc.set("CM/FLAGS", flags | BIT11);
            noGamdot = true;
        } else {
            int diff = Agc.toSigned(newGama) - Agc.toSigned(oldGama);
            long prod = (long) diff * Agc.toSigned(agc.get("TCDU"));      // MP TCDU
            int gamdot = (int) (prod >> 14);
            agc.set("GAMDOT", Agc.fromSigned(gamdot));
            // ignore GAMDOT if magnitude <= .5 deg/sec (FIVE counts)
            if (Math.abs(gamdot) - 5 <= 0) noGamdot = true;
        }
        if (noGamdot) agc.set("GAMDOT", 0);                              // NOGAMDOT: tag +0

        // Corrected Euler angles. MPAC holds (R EUL/180)/2 for roll, beta, alfa.
        agc.set("TEMPROLL", correct(agc.get("MPAC"), agc.get("ROLL/PIP"), agc.get("ROLL/180")));
        agc.set("TEMPALFA", correct(agc.get("MPAC+2"), agc.get("ALFA/PIP"), agc.get("ALFA/180")));
        // CMTR2: BETA (overflow not expected)
        int b = Agc.add(Agc.add(doubleWord(Agc.fromSigned(-Agc.toSigned(agc.get("MPAC+1")))),
                Agc.fromSigned(-Agc.toSigned(agc.get("BETA/PIP")))), agc.get("BETA/180"));
        agc.set("TEMPBETA", b);
        // PHASCHNG with REPOSADR (restart address REDOPOSE)
        redopose();
    }

    private static int correct(int mpac, int pip, int cur) {
        int v = corangov(doubleWord(Agc.fromSigned(-Agc.toSigned(mpac))));   // CS MPAC; DOUBLE
        v = Agc.add(v, Agc.fromSigned(-Agc.toSigned(pip)));                  // SU xxx/PIP
        v = Agc.add(v, cur);                                                 // AD xxx/180
        return corangov(v);
    }

    private static int doubleWord(int w) {
        return Agc.fromSigned(Agc.toSigned(w) * 2);   // DOUBLE (possible overflow corrected by CORANGOV)
    }

    /** CORANGOV: correct angle for overflow (a single overflow wraps by one revolution). */
    private static int corangov(int v) {
        int s = Agc.toSigned(v);
        if (s > 0x3FFF) s -= 0x8000;
        if (s < -0x3FFF) s += 0x8000;
        return Agc.fromSigned(s);
    }

    /** REDOPOSE: re-starts come here. Copies TEMP angles into the working angles, RELINT. */
    public void redopose() {
        agc.set("ROLL/180", agc.get("TEMPROLL"));
        agc.set("ALFA/180", agc.get("TEMPALFA"));
        agc.set("BETA/180", agc.get("TEMPBETA"));
        // RELINT; TC INTPRET (cannot TC DANZIG after PHASCHNG)
        cmPose3();
    }

    /** CM/POSE3: return from CM/ATUP (restart); VMAGI = |VN| for display, then POSEXIT. */
    public void cmPose3() {
        // VLOAD ABVAL VN (2^-7 m/cs); STORE VMAGI ; GOTO POSEXIT
        // (POSEXIT is ENDEXIT, STARTENT or SCALEPOP depending on caller)
    }

    // --- vector helpers ---
    private static double clamp(double x) { return Math.max(-1, Math.min(1, x)); }
    private static double dot(double[] a, double[] b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
    private static double norm(double[] a) { return Math.sqrt(dot(a, a)); }
    private static double[] unit(double[] a) { double m = norm(a); return m == 0 ? a.clone() : new double[] {a[0]/m, a[1]/m, a[2]/m}; }
    private static double[] neg(double[] a) { return new double[] {-a[0], -a[1], -a[2]}; }
    private static double[] cross(double[] a, double[] b) {
        return new double[] {a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]};
    }
    /** VXM: v * M (row vector times matrix). */
    private static double[] vxm(double[][] m, double[] v) {
        double[] o = new double[3];
        for (int j = 0; j < 3; j++) o[j] = v[0]*m[0][j] + v[1]*m[1][j] + v[2]*m[2][j];
        return o;
    }
}
