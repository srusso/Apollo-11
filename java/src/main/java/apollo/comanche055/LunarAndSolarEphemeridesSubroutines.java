package apollo.comanche055;

import apollo.agc.Agc;

/**
 * LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc
 *
 * Position/velocity of the Sun and Moon relative to the Earth at a specified ground elapsed time.
 * The Moon is a ninth-degree vector polynomial (VECOEM, ten coefficient vectors, descending order)
 * valid over a 15 day interval centred on TIMEMO. The Sun is a rotation of RESO about the
 * orbit normal (RESO x VESO) at rate OMEGAES.
 *
 * Vectors are double[3] in physical units; the original scaling is given in each method's comment.
 * Erasable inputs: TEPHEM, TIMEMO, VECOEM (10 x 3), RESO(3), VESO(3), OMEGAES.
 * Interpreter-style outputs (MPAC vector, 2D of VAC) are returned in {@link Result}.
 */
public final class LunarAndSolarEphemeridesSubroutines {
    private static final double NINEB4 = 9.0;  // 2DEC 9.0 B-4
    private static final double ONEB4 = 1.0;   // 2DEC 1.0 B-4

    /** Result of LSPOS: sun position in VAC+2D, moon position in MPAC. */
    public static final class Result {
        public double[] sun;   // 2D of VAC: metres x B-38
        public double[] moon;  // MPAC: metres x B-29
    }

    private final Agc agc;
    private final double[][] vecoem = new double[10][3];
    private final double[] reso = new double[3];
    private final double[] veso = new double[3];
    private double omegaes;
    private double tephem;
    private double timemo;

    public LunarAndSolarEphemeridesSubroutines(Agc agc) {
        this.agc = agc;
    }

    /** Loads the prelaunch erasable data load (normally done by the ground load). */
    public void load(double[][] vecoem, double[] reso, double[] veso, double omegaes, double tephem, double timemo) {
        for (int i = 0; i < 10; i++) {
            System.arraycopy(vecoem[i], 0, this.vecoem[i], 0, 3);
        }
        System.arraycopy(reso, 0, this.reso, 0, 3);
        System.arraycopy(veso, 0, this.veso, 0, 3);
        this.omegaes = omegaes;
        this.tephem = tephem;
        this.timemo = timemo;
    }

    /** LSPOS: positions of both Sun (in VAC 2D) and Moon (in MPAC). */
    public Result lspos(double time) {
        Result r = new Result();
        double t = lstime(time);
        r.sun = res(t);       // AXT,2 RESA
        r.moon = rem(t);      // AXT,1 REM
        return r;
    }

    /** LUNPOS: position vector of the Moon, metres x B-29. */
    public double[] lunpos(double time) {
        return rem(lstime(time));
    }

    /** LUNVEL: velocity vector of the Moon, metres/cs x B-7. */
    public double[] lunvel(double time) {
        return vem(lstime(time));
    }

    /** SOLPOS: position vector of the Sun, metres x B-38. */
    public double[] solpos(double time) {
        return res(lstime(time));
    }

    /**
     * LSTIME: time argument for the polynomial: (GET - TEPHEM - TIMEMO) after SR 14D / SL 16D scaling.
     * S1 is set to 6D (vector step) in the original.
     */
    private double lstime(double time) {
        return time - tephem - timemo;
    }

    /** RES: Sun position = cos(w t) RESO + sin(w t) |RESO| (unit(RESO) x unit(VESO)) x unit(RESO)-plane tangent. */
    private double[] res(double t) {
        double angle = 2.0 * Math.PI * omegaes * t;   // OMEGAES in rev/cs
        double c = Math.cos(angle);
        double s = Math.sin(angle);
        double rmag = norm(reso);
        double[] ur = scale(reso, 1.0 / rmag);
        double[] uv = scale(veso, 1.0 / norm(veso));
        double[] w = cross(ur, uv);                    // orbit normal
        double[] tangent = scale(cross(w, reso), 1.0); // W x RESO
        double[] out = new double[3];
        for (int i = 0; i < 3; i++) {
            out[i] = c * reso[i] + s * tangent[i];     // VXSC / VAD
        }
        return out;
    }

    /** REM: Horner evaluation of the lunar position polynomial (loop X1 = 54D step -6D). */
    private double[] rem(double t) {
        double[] acc = vecoem[0].clone();
        for (int k = 1; k < 10; k++) {                 // REMA: VXSC 0D ; VAD VECOEM+60D,1 ; TIX,1
            for (int i = 0; i < 3; i++) {
                acc[i] = acc[i] * t + vecoem[k][i];
            }
        }
        return acc;                                    // metres x B-29 in MPAC
    }

    /** VEM: derivative of the polynomial, with coefficients 9,8,...,1 (NINEB4 reduced by ONEB4 each step). */
    private double[] vem(double t) {
        double factor = NINEB4;
        double[] acc = scale(vecoem[0], factor);       // 9 * c0
        for (int k = 1; k < 9; k++) {                  // VEMA loop
            factor -= ONEB4;
            for (int i = 0; i < 3; i++) {
                acc[i] = acc[i] * t + factor * vecoem[k][i];
            }
        }
        return acc;                                    // metres/cs x B-7 in MPAC
    }

    private static double norm(double[] v) {
        return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }

    private static double[] scale(double[] v, double k) {
        return new double[] {v[0] * k, v[1] * k, v[2] * k};
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[] {
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]};
    }
}
