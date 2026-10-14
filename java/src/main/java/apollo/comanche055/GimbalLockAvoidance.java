package apollo.comanche055;

import apollo.agc.Agc;

/**
 * GIMBAL_LOCK_AVOIDANCE.agc (Comanche 055, pp. 412-413).
 * Detecting gimbal lock / computing the maneuver rate-vector bias (WCALC).
 */
public class GimbalLockAvoidance {
    private final Agc agc;

    public GimbalLockAvoidance(Agc agc) {
        this.agc = agc;
    }

    /** ARATE: maneuver rates .05, .2, .5, 2 deg/sec (scaled). */
    public static final double[] ARATE = {0.0022222222, 0.0088888889, 0.0222222222, 0.0888888889};
    /** ANGLTIME: maneuver angle to maneuver time = 100B-19. */
    public static final double ANGLTIME = 0.000190735;
    /** QUADROT: rotation matrix from S/C axes to control axes (X rot = -7.25 deg), row major. */
    public static final double[] QUADROT = {
        0.1, 0, 0,
        0, 0.099200, -0.012620,
        0, 0.012620, 0.099200};
    /** BIASCALE = (450/180)(1/0.6)(1/16384). */
    public static final double BIASCALE = 0.0002543132;

    /** Interpretive-level result of WCALC. */
    public double[] brate = new double[3];
    public double[] biastemp = new double[3];
    public double tm;

    /** LOCSKIRT / WCALC: compute BRATE, TM and BIASTEMP for the selected rate index. */
    public void wcalc(double[] cof, double am, int rateIndx, double[][] delcomp) {
        double arate = ARATE[rateIndx];
        // CALL DELCOMP: incremental rotation matrix for 1 sec rotation about COF (SR4 scaled)
        // DLOAD* ARATE,1 ; VXSC COF ; MXV QUADROT ; STODL BRATE
        double[] v = {arate * cof[0], arate * cof[1], arate * cof[2]};
        for (int i = 0; i < 3; i++) {
            brate[i] = QUADROT[3 * i] * v[0] + QUADROT[3 * i + 1] * v[1] + QUADROT[3 * i + 2] * v[2];
        }
        // DMP ANGLTIME ; DDV* ARATE,1 ; SR 5 ; STOVL TM
        tm = am * ANGLTIME / arate / 32.0;
        // VXSC BIASCALE ; STORE BIASTEMP (attitude error bias to prevent overshoot)
        for (int i = 0; i < 3; i++) {
            biastemp[i] = brate[i] * BIASCALE;
        }
        // SETGO CALCMAN2 / NEWANGL +1: state switch CALCMAN2 (43D): off = bypass starting procedure,
        // on = start maneuver.
    }
}
