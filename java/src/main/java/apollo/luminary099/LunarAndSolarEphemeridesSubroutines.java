package apollo.luminary099;

import apollo.agc.Agc;

/**
 * LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc (Luminary 099, pages 984-987).
 *
 * LSPOS (alias LUNPOS): unit position vectors of the Sun and Moon in the basic reference system.
 * The Sun is located by its ecliptic longitude LOS = LOS0 + LOSR*T - (C term); the Moon by LOM, LOS and
 * LON (mean longitude, ecliptic longitude, node longitude), where each sinusoidal "fudge" term is
 * (X0^2 + X1^2)^(1/2) * SIN(2 pi T/period + PHI) from the VAL67 table (AMOD/AARG, BMOD/BARG, CMOD/CARG).
 * The angles are rotated through the KONMAT matrix (K1..K4: cos OBL, sin OBL sin IM, sin OBL, cos OBL sin IM).
 *
 * Angles are in revolutions and T is in days (original scaling: T at B9 days, vectors at B-1).
 * Erasable inputs (RATESP, VAL67, KONMAT, TEPHEM) are provided through {@link #load}.
 * Outputs VSUN and VMOON are returned from {@link #lspos(long)} and stored in the Agc symbol table.
 */
public final class LunarAndSolarEphemeridesSubroutines {
    /** CSTODAY: centiseconds per day (8,640,000, B-33). */
    public static final double CSTODAY = 8_640_000.0;

    private final Agc agc;

    /** VAL67 rows (offset by X1 = 0, -6, -12): {amplitude, phi (rev), frequency (rev/day)} for A, B, C terms. */
    private final double[][] val67 = new double[3][3];
    /** RATESP: LOMR, LOSR, LONR (rev/day) followed by LOMO, LOSO, LONO (rev). */
    private final double[] ratesp = new double[6];
    /** KONMAT: the 3x3 matrix K1,K2,K3,K4 with the zero constant at KONMAT+2 (row-major). */
    private final double[][] konmat = new double[3][3];
    private double tephem;   // centiseconds, TEPHEM

    private final double[] stmp = new double[3];  // STMP: LOM, LOS, LON
    private double gtmp;                           // GTMP
    private double timep;                          // TIMEP: T in days

    public double[] vsun = new double[3];
    public double[] vmoon = new double[3];

    public LunarAndSolarEphemeridesSubroutines(Agc agc) {
        this.agc = agc;
    }

    /** Loads the yearly constants (normally from the ground load). */
    public void load(double[][] val67, double[] ratesp, double[][] konmat, double tephem) {
        for (int i = 0; i < 3; i++) {
            System.arraycopy(val67[i], 0, this.val67[i], 0, 3);
            System.arraycopy(konmat[i], 0, this.konmat[i], 0, 3);
        }
        System.arraycopy(ratesp, 0, this.ratesp, 0, 6);
        this.tephem = tephem;
    }

    /** LUNPOS EQUALS LSPOS. */
    public void lunpos(long timeCs) { lspos(timeCs); }

    /** LUNVEL: RVQ (to fool integration). */
    public void lunvel() { }

    /**
     * LSPOS: TIMEP = (TIME + TEPHEM)/CSTODAY (days); computes three angles (pass X2 = 0, -2, -4 over
     * RATESP), then the Moon vector in VMOON and the Sun vector in VSUN (both unit vectors).
     */
    public void lspos(long timeCs) {
        timep = (timeCs + tephem) / CSTODAY;       // TAD TEPHEM, DDV CSTODAY
        boolean freeflag = false;                   // CLEAR FREEFLAG
        int x1 = 0;                                 // AXT,1 0
        for (int x2 = 0; x2 < 3; x2++) {            // AXT,2 0 ; INCR,2 -2 each pass (POSITA .. POSITD)
            gtmp = 0;                               // POSITA: DLOAD KONMAT+2 (zero), STORE GTMP
            if (x2 < 2) {
                int terms = (x2 == 0) ? 2 : 1;      // BOFSET FREEFLAG loops POSITB: two terms the first time
                for (int t = 0; t < terms; t++) {
                    double[] v = val67[(-x1) / 6];
                    positb(v);
                    x1 -= 6;                        // INCR,1 -6
                }
                freeflag = true;
            }
            // POSITE (third pass): GTMP = 0 then POSITD
            // POSITD: STMP,2 = T*RATESP,2 << 5 + RATESP+6,2 - GTMP
            stmp[x2] = timep * ratesp[x2] + ratesp[x2 + 3] - gtmp;
        }
        positf();
    }

    /** POSITB: GTMP += amp * SIN(T*freq + PHI) for one VAL67 row {amp, phi, freq}. */
    private void positb(double[] row) {
        double arg = timep * row[2] + row[1];
        gtmp += row[0] * Math.sin(2 * Math.PI * arg);
    }

    /**
     * POSITF (3rd pass): Moon M = KONMAT x (SIN(LOM-LON), SIN LOM, COS LOM), unit; then Sun
     * S = KONMAT x (0, SIN LOS, COS LOS), unit.
     */
    private void positf() {
        double lom = stmp[0], los = stmp[1], lon = stmp[2];
        double[] m = {Math.sin(2 * Math.PI * (lom - lon)), Math.sin(2 * Math.PI * lom), Math.cos(2 * Math.PI * lom)};
        vmoon = VecOps.unit(VecOps.mxv(konmat, m));
        double[] s = {0, Math.sin(2 * Math.PI * los), Math.cos(2 * Math.PI * los)};
        vsun = VecOps.unit(VecOps.mxv(konmat, s));
        VecOps.setV(agc, "VMOON", VecOps.scale(vmoon, 0.5));   // SCALED B-1
        VecOps.setV(agc, "VSUN", VecOps.scale(vsun, 0.5));
    }
}
