package apollo.luminary099;

import apollo.agc.Agc;

/**
 * LEM_GEOMETRY.agc (Luminary 1A build 099, pages 320-325).
 * State-vector update (conic + deviations) for LM/CSM and rendezvous-radar (RR) geometry.
 * Interpretive code is modelled with double[3] vectors; erasable vectors live in the
 * Agc symbol table as three consecutive words under the names X, X+1, X+2.
 */
public final class LemGeometry {
    private final Agc agc;

    public LemGeometry(Agc agc) {
        this.agc = agc;
    }

    private static double[] vsl(double[] v, int n) {
        double[] r = new double[3];
        double f = Math.pow(2, n);
        for (int i = 0; i < 3; i++) r[i] = v[i] * f;
        return r;
    }

    private static double[] vad(double[] a, double[] b) {
        return new double[] {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
    }

    private double[] getVec(String name) {
        double[] v = new double[3];
        v[0] = Agc.toSigned(agc.get(name));
        v[1] = Agc.toSigned(agc.get(name + "+1"));
        v[2] = Agc.toSigned(agc.get(name + "+2"));
        return v;
    }

    private void setVec(String name, double[] v) {
        agc.set(name, Agc.fromSigned((int) v[0]));
        agc.set(name + "+1", Agc.fromSigned((int) v[1]));
        agc.set(name + "+2", Agc.fromSigned((int) v[2]));
    }

    /**
     * SVDWN2: compute actual LM/CSM state vector as conic plus deviations; store in RN, VN, PIPTIME.
     * Input scaling selected by index register X2 (2: pos B27/vel B5; 0: pos B29/vel B7).
     * Returns immediately (RVQ) if AVEMIDSW is set (W-matrix integration under way).
     */
    public void svdwn2(int x2) {
        if (agc.get("AVEMIDSW") != 0) { // BOF RVQ AVEMIDSW
            return;
        }
        double[] r = vad(vsl(getVec("TDELTAV"), -7 + x2), vsl(getVec("RCV"), x2));
        // note: vsl* with 0 -7,2 means shift count (-7 + X2) as used by the original
        setVec("RN", vsl(r, 0));
        double[] v = vad(vsl(getVec("TNUV"), -4 + x2), getVec("VCV"));
        setVec("VN", vsl(v, x2));
        agc.set("PIPTIME", agc.get("TET")); // STODL VN / TET ; STORE PIPTIME
        agc.set("PIPTIME+1", agc.get("TET+1"));
    }

    /** SVDWN1: same as SVDWN2 but results go to R-OTHER, V-OTHER (for downlink). */
    public void svdwn1(int x2) {
        double[] r = vad(vsl(getVec("TDELTAV"), -7 + x2), vsl(getVec("RCV"), x2));
        setVec("R-OTHER", r);
        double[] v = vad(vsl(getVec("TNUV"), -4 + x2), getVec("VCV"));
        setVec("V-OTHER", vsl(v, x2));
    }

    /**
     * RRANGLES: given a half-unit target vector (nav-base coordinates) in {@code los}, find both
     * RR gimbal orientations. Mode 1: |trunnion| &lt; 90 deg, arbitrary shaft; mode 2 follows from
     * S2 = 180 + S1, T2 = 180 - T1. Angles are in revolutions (1.0 = 360 deg, so half = 180 deg).
     * Results left in MODEA (current mode) and MODEB (mode after remode) as {trunnion, shaft} pairs.
     */
    public double[][] rrAngles(double[] los, boolean antennaMode2) {
        double[] v32 = los.clone(); // STORE 32D
        // Mode 1 trunnion: asin of -Y component (half unit => *2), expressed in revolutions.
        double t1 = Math.asin(-2.0 * v32[1]) / (2 * Math.PI);
        double t2 = 0.5 - t1; // BDSU LODPHALF: mode 2 trunnion
        // Project vector into X-Z plane and unit it. Overflow => target along Y (LUNDESCH).
        double px = v32[0], pz = v32[2];
        double norm = Math.hypot(px, pz);
        if (norm < 1e-12) {
            lunDesch();
            return null;
        }
        // ARCTRIG on (sinTh, cosTh) gives mode 1 shaft angle in revolutions.
        double s1 = Math.atan2(px, pz) / (2 * Math.PI);
        if (s1 < 0) s1 += 1.0;
        double s2 = s1 + 0.5; // mode 2 shaft: 180 + S1
        if (s2 >= 1.0) s2 -= 1.0;
        double[] modeA = {t1, s1}; // 2V1STO2S: convert to CDU format
        double[] modeB = {t2, s2};
        // CS RADMODES / MASK ANTENBIT: swap if RR in mode 2.
        if (antennaMode2) {
            return new double[][] {modeB, modeA};
        }
        agc.set("MODEA", Agc.fromSigned((int) (t1 * 16384)));
        agc.set("MODEA+1", Agc.fromSigned((int) (s1 * 16384)));
        agc.set("MODEB", Agc.fromSigned((int) (t2 * 16384)));
        agc.set("MODEB+1", Agc.fromSigned((int) (s2 * 16384)));
        return new double[][] {modeA, modeB}; // GOTO S2
    }

    private void lunDesch() {
        // Target vector along Y: call for maneuver unless on lunar surface.
        new Object(); // BOVB LUNDESCH (defined in another file)
    }

    /**
     * RRNB: given RR trunnion T and shaft S (revolutions) find the half-unit line of sight in nav-base
     * axes: 0.5 * (sin(S)cos(T), -sin(T), cos(S)cos(T)). Result is also left in 32D (X), 34D (Y), 36D (Z).
     */
    public double[] rrnb(double trunnionRev, double shaftRev) {
        double t = trunnionRev * 2 * Math.PI;
        double s = shaftRev * 2 * Math.PI;
        double[] los = {
            0.5 * Math.sin(s) * Math.cos(t),
            -0.5 * Math.sin(t),
            0.5 * Math.cos(s) * Math.cos(t)};
        agc.set("32D", Agc.fromSigned((int) (los[0] * 16384)));
        agc.set("34D", Agc.fromSigned((int) (los[1] * 16384)));
        agc.set("36D", Agc.fromSigned((int) (los[2] * 16384)));
        return los;
    }

    /** RRNB (entry from TANGNB): reads trunnion/shaft from TANGNB, TANGNB+1 via CDULOGIC. */
    public double[] rrnbFromTangnb() {
        return rrnb(Agc.toSigned(agc.get("TANGNB")) / 16384.0, Agc.toSigned(agc.get("TANGNB+1")) / 16384.0);
    }

    /** RRNBMPAC: entry requiring trunnion in MPAC and shaft in MPAC+1. */
    public double[] rrnbMpac() {
        return rrnb(Agc.toSigned(agc.get("MPAC")) / 16384.0, Agc.toSigned(agc.get("MPAC+1")) / 16384.0);
    }
}
