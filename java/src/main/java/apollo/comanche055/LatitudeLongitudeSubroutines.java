package apollo.comanche055;

import apollo.agc.Agc;

/**
 * LATITUDE_LONGITUDE_SUBROUTINES.agc (bank 30, LATLONG, COUNT 13/LT-LG).
 * LAT-LONG converts a radius vector (ALPHAV, meters B-29) at a given time to latitude,
 * longitude (revolutions) and altitude; LALOTORV does the inverse. Supporting routines:
 * GETERAD, ARCTAN, SETGAMMA, SETRE. LUNAFLAG=0 Earth, 1 Moon; ERADFLAG=1 computes the
 * Earth radius from the Fischer ellipsoid, 0 uses fixed radius.
 *
 * Values are held as doubles in natural units (meters, revolutions) instead of scaled
 * double-precision words.
 */
public final class LatitudeLongitudeSubroutines {
    private final Agc agc;

    // Constants computed with A=6378166, B=6356784 meters.
    /** B**2 scaled B-51. */
    public static final double B2XSC = 0.0179450689;
    /** B**2/A**2 (GAMMA for the Earth), scaled B-1. */
    public static final double B2_A2 = 0.9933064884 * 2;
    /** EE = 1 - B**2/A**2, scaled B-0. */
    public static final double EE = 6.6935116e-3;
    /** ERAD: pad radius, meters. */
    public static final double ERAD = 6373338;
    /** 504RM: Moon radius, meters. */
    public static final double RM504 = 1738090;

    // Working state (ALPHAV, ALPHAM, GAMRP, ERADM, LAT, LONG, ALT)
    public double[] alphav = new double[3];
    public double alphaM, gamrp, eradm, lat, lon, alt, theta;
    public boolean lunaflag, eradflag;
    public double[] rls = new double[3];

    public LatitudeLongitudeSubroutines(Agc agc) {
        this.agc = agc;
    }

    /**
     * LAT-LONG: convert R (ALPHAV) at time to LAT, LONG, ALT.
     * R-TO-RP (rotation from reference to planet-fixed frame) is supplied by the caller.
     * @param rToRp function applying R-TO-RP to the position vector
     */
    public void latLong(java.util.function.UnaryOperator<double[]> rToRp) {
        alphaM = norm(alphav);                       // ABS value of R for ALT formula
        double[] rp = rToRp.apply(alphav.clone());   // CALL R-TO-RP
        double[] u = unit(rp);                       // UNIT RP
        alphav = u;                                  // U2 = SIN L for SETRE
        setGamma();
        setRe();
        // COS(LAT) = sqrt(x^2+y^2)*GAMRP ; SIN(LAT) = z
        double cosLat = Math.sqrt(alphav[0] * alphav[0] + alphav[1] * alphav[1]) * gamrp;
        lat = arctan(alphav[2], cosLat);
        lon = arctan(alphav[1], alphav[0]);          // range -1/2 to 1/2 revs
        alt = alphaM - eradm;                        // ALT = R - RE
    }

    /**
     * LALOTORV: convert LAT, LONG, ALT to the radius vector in ALPHAV.
     * @param rpToR function applying RP-TO-R to the unit vector (planet-fixed to reference)
     */
    public void lalotorv(java.util.function.UnaryOperator<double[]> rpToR) {
        setGamma();
        double latR = lat * 2 * Math.PI, lonR = lon * 2 * Math.PI;
        double[] rp = {Math.cos(lonR) * Math.cos(latR), Math.sin(lonR) * Math.cos(latR), gamrp / 2 * Math.sin(latR) * 2};
        double[] u = unit(rp);
        alphav = u.clone();
        alphav[2] = u[2];                            // ALPHAV+4 = SIN L for SETRE
        setRe();
        double[] ur = unit(rpToR.apply(u));          // CALL RP-TO-R
        double r = eradm + alt;                      // (RE + ALT)
        alphav = new double[] {ur[0] * r, ur[1] * r, ur[2] * r};
    }

    /** GETERAD: Earth radius from 1/2 SIN LAT in ALPHAV+4; result in ERADM. */
    public double geterad() {
        double sin2 = alphav[2] * alphav[2];
        double cos2 = 1 - sin2;
        double d = 1 - EE * cos2;                    // (1 - EE*COS**2)
        eradm = Math.sqrt(B2XSC / d);                // sqrt(B**2 / (1 - e**2 cos**2 L))
        return eradm;
    }

    /**
     * ARCTAN: theta (revolutions, -1/2..+1/2) from SINTH and COSTH. 0/0 gives 0.
     */
    public double arctan(double sinth, double costh) {
        if (sinth * sinth + costh * costh == 0) {   // ARCTANXX: ATAN = 0/0, THETA = 0
            theta = 0;
            return theta;
        }
        double t = Math.atan2(sinth, costh) / (2 * Math.PI);
        theta = t;
        return theta;
    }

    /** SETGAMMA: GAMRP = B**2/A**2 for Earth (B-1), 1 for Moon (B-1). */
    public void setGamma() {
        gamrp = lunaflag ? 1.0 : B2_A2 / 2;
    }

    /**
     * SETRE: ERADM = Moon radius for Moon; for Earth either ERAD (fixed) or computed from the
     * Fischer ellipsoid (GETERAD). For Moon with ERADFLAG=0 RLS magnitude is used (TSTRLSRM).
     */
    public void setRe() {
        if (lunaflag) {
            if (eradflag) {
                eradm = RM504;
            } else {
                eradm = norm(rls);                   // R0 = |RLS| (scaled B-27 to B-29)
            }
        } else if (eradflag) {
            geterad();
        } else {
            eradm = ERAD;
        }
    }

    private static double norm(double[] a) { return Math.sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]); }
    private static double[] unit(double[] a) {
        double m = norm(a);
        return m == 0 ? a.clone() : new double[] {a[0]/m, a[1]/m, a[2]/m};
    }
}
