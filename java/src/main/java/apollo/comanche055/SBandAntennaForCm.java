package apollo.comanche055;

import apollo.agc.Agc;

/**
 * S-BAND_ANTENNA_FOR_CM.agc (R05, V64E): computes the S-band antenna pointing angles
 * (pitch RHOSB and yaw GAMMASB) from the spacecraft position relative to the Earth
 * or Moon, expressed in navigation base coordinates. Angles are in revolutions.
 */
public final class SBandAntennaForCm {
    private final Agc agc;

    /** V06N51 display: angles. */
    public static final String V06N51 = "0651";
    /** Pushdown list offsets (EQUALS). */
    public static final int RCM = 2, UR = 8, URP = 14, YAWANG = 20, PITCHANG = 22;
    public static final double DPPOSMAX = 1.0 - Math.pow(2, -28);
    public static final double HIDP1_4 = 0.25;

    public SBandAntennaForCm(Agc agc) {
        this.agc = agc;
    }

    /** Unit vectors used by the routine. */
    private static final double[] HIUNITX = {1, 0, 0}, HIUNITY = {0, 1, 0}, HIUNITZ = {0, 0, 1};

    /**
     * SBANDANT: V 64 E entry. Checks IMU on and aligned (R02BOTH), gets the current time and
     * CDU sines/cosines, integrates to TDEC1 (CSMCONIC), then finds the vehicle position
     * relative to the Earth/Moon and computes pitch/yaw.
     *
     * @param ratt   CSM position at TDEC1 (reference system)
     * @param moonPos position of the moon at the original time (used if the reference origin is the Moon)
     * @param refsmmat 3x3 reference to stable member matrix
     * @param smnb   3x3 stable member to nav base matrix (from *SMNB* with YAWANG=0)
     * @param originMoon true if X2 (origin of reference system) is the Moon (2), false for Earth (0)
     * @return {RHOSB, GAMMASB} in revolutions
     */
    public double[] sbandant(double[] ratt, double[] moonPos, double[][] refsmmat,
                             double[][] smnb, boolean originMoon) {
        // TC BANKCALL / CADR R02BOTH ; RTB LOADTIME ; CALL CDUTRIG ; STCALL TDEC1 / CSMCONIC
        double[] rcm = ratt.clone();               // STORE RCM (move RATT to prevent wipeout)
        double[] r;
        if (originMoon) {
            // LUNPOS(TAT); R = -(REM + RCM): negative of S/C position vector
            r = new double[3];
            for (int i = 0; i < 3; i++) r[i] = -(moonPos[i] + rcm[i]);
        } else {
            // EISOI: EARTH, R = -RCM
            r = new double[3];
            for (int i = 0; i < 3; i++) r[i] = -rcm[i];
        }
        // MXV REFSMMAT : RCS to stable member; then *SMNB* with YAWANG zero
        double[] sm = mxv(refsmmat, r);
        double[] nb = mxv(smnb, sm);               // STORE R: nav base coordinates
        double[] ur = unit(nb);                    // UNIT; STORE UR
        // URP = unit( R - (R.UZ)UZ ) : projection into the CM XY-plane
        double dz = dot(nb, HIUNITZ);
        double[] urpRaw = {nb[0] - dz * HIUNITZ[0], nb[1] - dz * HIUNITZ[1], nb[2] - dz * HIUNITZ[2]};
        double yaw;
        double mag = Math.sqrt(dot(urpRaw, urpRaw));
        if (mag == 0) {
            // NOADJUST via BOV: null vector, yaw left as pushed (0)
            yaw = 0;
        } else {
            double[] urp = unit(urpRaw);
            yaw = Math.acos(dot(urp, HIUNITX)) / (2 * Math.PI);   // revolutions
            if (dot(urp, HIUNITY) < 0) {                          // 181-360 degrees
                yaw = DPPOSMAX - yaw;                              // 2 PI minus yaw angle
            }
        }
        // Pitch angle = ACOS(UR.UZ) - PI/2 (revolutions)
        double pitch = Math.acos(dot(ur, HIUNITZ)) / (2 * Math.PI) - HIDP1_4;
        agc.set("RHOSB", Agc.fromSigned((int) Math.round(pitch * 16384)));
        agc.set("GAMMASB", Agc.fromSigned((int) Math.round(yaw * 16384)));
        return new double[] {pitch, yaw};
    }

    /**
     * Display loop after computing angles (EXIT section): if V64 flag (EXTVBACT bit 5) is
     * still on, display V06N51 via GOMARKFR; on terminate/proceed turn B5 off, on recycle
     * (ENDOFJOB) wait a minimum time (DELAYJOB) and recompute from SBANDANT +2.
     */
    public void displayLoop() {
        final int BIT5 = 0x10;
        if ((agc.get("EXTVBACT") & BIT5) == 0) {
            return;                                 // BZF ENDEXT : we have been answered
        }
        // CAF V06N51; BANKCALL GOMARKFR -> returns: terminate / proceed (B5OFF) / recycle
        // On recycle: CAF BIT3 / BLANKET (blank R3); CAF BIT1 / DELAYJOB; TCF SBANDANT +2
    }

    private static double dot(double[] a, double[] b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
    private static double[] unit(double[] a) {
        double m = Math.sqrt(dot(a, a));
        return m == 0 ? a.clone() : new double[] {a[0]/m, a[1]/m, a[2]/m};
    }
    private static double[] mxv(double[][] m, double[] v) {
        double[] o = new double[3];
        for (int i = 0; i < 3; i++) o[i] = m[i][0]*v[0] + m[i][1]*v[1] + m[i][2]*v[2];
        return o;
    }
}
