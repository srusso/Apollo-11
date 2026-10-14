package apollo.luminary099;

import apollo.agc.Agc;

/**
 * LATITUDE_LONGITUDE_SUBROUTINES (Luminary 099): conversion between a position vector R (meters,
 * reference frame) and geodetic latitude / longitude / altitude over the Earth or Moon.
 *
 * <p>LAT-LONG converts R (ALPHAV) at time (6-7D) into LAT, LONG (revolutions) and ALT (meters);
 * LALOTORV does the reverse. Moon uses a spherical model (GAMMA = 1); the Earth uses the
 * oblate ellipsoid with GAMMA = B^2/A^2.
 */
public class LatitudeLongitudeSubroutines {
    /** B**2 scaled B-51 (square of Earth's polar radius, 2DEC .0179450689 B-51 -> m^2). */
    public static final double B2XSC = 0.0179450689;
    /** B2/A2: GAMMA for Earth, B-1. */
    public static final double B2_A2 = 0.9933064884;
    /** EE = 1 - B**2/A**2. */
    public static final double EE = 6.6935116e-3;
    /** 2^51, scale factor of B2XSC. */
    private static final double TWO_51 = 2251799813685248.0;

    /** Services provided by other parts of the system (conic/coordinate transforms and constants). */
    public interface Env {
        /** R-TO-RP: convert reference-frame vector to the planet-fixed frame at the given time. */
        double[] rToRp(double[] r, double time, boolean moon);

        /** RP-TO-R: convert planet-fixed unit vector back to the reference frame. */
        double[] rpToR(double[] rp, double time, boolean moon);

        /** Fixed Earth radius ERAD, meters. */
        double erad();

        /** Computed lunar radius 504RM, meters. */
        double rm504();

        /** Landing site vector RLS, (meters). */
        double[] rls();
    }

    private final Agc agc;
    private Env env;

    // Erasable state corresponding to the AGC symbols.
    public double[] alphaV = new double[3];
    public double alphaM;
    public double lat;   // LAT, revolutions
    public double lon;   // LONG, revolutions in -1/2..1/2
    public double alt;   // ALT, meters
    public double eradM; // ERADM, meters
    public double gamRp; // GAMRP (= location 8D)
    public boolean lunaFlag;
    public boolean eradFlag;

    public LatitudeLongitudeSubroutines(Agc agc) {
        this.agc = agc;
    }

    public void setEnv(Env env) {
        this.env = env;
    }

    private static double[] unit(double[] a) {
        double n = Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        return n == 0 ? new double[3] : new double[] {a[0] / n, a[1] / n, a[2] / n};
    }

    /**
     * LAT-LONG: R vector (ALPHAV, meters) and time -> LAT, LONG (revolutions) and ALT (meters).
     */
    public void latLong(double time) {
        alphaM = Math.sqrt(alphaV[0] * alphaV[0] + alphaV[1] * alphaV[1] + alphaV[2] * alphaV[2]);
        double[] rp = env.rToRp(alphaV, time, lunaFlag); // R-TO-RP (non-zero MPAC selects Moon)
        double[] u = unit(rp);                           // UNIT RP
        alphaV = u;                                      // U2 = SINL for SETRE below
        setGamma();
        setRe();                                         // RE in meters
        double cosLat = Math.sqrt(u[0] * u[0] + u[1] * u[1]) * gamRp; // COS(LAT)
        double sinLat = u[2];                                           // SIN(LAT)
        lat = arctan(sinLat, cosLat);                    // LAT, revolutions
        lon = arctan(u[1], u[0]);                        // LONG, revolutions in -1/2..1/2
        alt = alphaM - eradM;                            // ALT = R - RE
    }

    /** LALOTORV: LAT, LONG, ALT (and time) to R vector in ALPHAV (meters). */
    public void laLoToRv(double time) {
        setGamma();
        double latRad = lat * 2 * Math.PI;
        double lonRad = lon * 2 * Math.PI;
        double[] rp = {Math.cos(lonRad) * Math.cos(latRad), Math.sin(lonRad) * Math.cos(latRad),
                gamRp * Math.sin(latRad)};
        double[] u = unit(rp);        // unit RP for the RP-TO-R subroutine
        alphaV = u;                   // ALPHAV +4 = SINL for SETRE
        setRe();                      // RE meters
        double[] ur = env.rpToR(u, time, lunaFlag); // unit R vector
        double scale = eradM + alt;   // (RE + ALT) UNIT R, meters
        alphaV = new double[] {ur[0] * scale, ur[1] * scale, ur[2] * scale};
    }

    /** GETERAD: Earth radius at latitude: ERADM = sqrt(B^2 / (1 - EE cos^2(L))) with SIN(L) = ALPHAV +4. */
    public void getErad() {
        double sin2 = alphaV[2] * alphaV[2];
        double cos2 = 1 - sin2;
        double b2 = B2XSC * TWO_51;
        eradM = Math.sqrt(b2 / (1 - EE * cos2));
    }

    /**
     * ARCTAN: angle in revolutions (-1/2..1/2) of the point (cos, sin); zero when both are zero
     * (ATAN = 0/0 sets THETA = 0). Handles the quadrant using the sign of COSTH (NEGCOS).
     */
    public static double arctan(double sin, double cos) {
        if (sin * sin + cos * cos == 0) {
            return 0; // ARCTANXX: ATAN=0/0 set THETA=0
        }
        return Math.atan2(sin, cos) / (2 * Math.PI);
    }

    /** SETGAMMA: GAMRP = B2/A2 for Earth, 1 for Moon. */
    public void setGamma() {
        gamRp = lunaFlag ? 1.0 : B2_A2;
    }

    /**
     * SETRE: planet radius ERADM in meters. Earth: fixed ERAD (ERADFLAG=0) or computed at latitude
     * (ERADFLAG=1). Moon: |RLS| (ERADFLAG=0) or computed 504RM (ERADFLAG=1).
     */
    public void setRe() {
        if (lunaFlag) {
            if (eradFlag) {
                eradM = env.rm504();
            } else {
                double[] r = env.rls();
                eradM = Math.sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
            }
            return;
        }
        if (!eradFlag) {
            eradM = env.erad();
        } else {
            getErad();
        }
    }
}
