package apollo.luminary099;

import apollo.agc.Agc;

/**
 * GIMBAL_LOCK_AVOIDANCE (Luminary 099).
 *
 * Free-fall attitude maneuver rate selection (W-CALC) used by KALCMAN3 when gimbal lock
 * must be avoided. Located in bank 15 at KALCMON1.
 */
public final class GimbalLockAvoidance {
    private final Agc agc;

    public GimbalLockAvoidance(Agc agc) {
        this.agc = agc;
    }

    /** The four selectable free fall maneuver rates (RATEINDX = 0,2,4,6), scaled at 22.5 deg/sec. */
    public static final double[] ARATE = {
        0.0088888888, // 0.2 deg/sec
        0.0222222222, // 0.5 deg/sec
        0.0888888888, // 2.0 deg/sec
        0.4444444444, // 10.0 deg/sec
    };

    /** Fudge factor (1008-19) converting maneuver angle to maneuver time. */
    public static final double ANGLTIME = 0.0001907349;

    /** Result of WCALC: component maneuver rates (45 deg/sec scaling) and execution time (scaled as T2). */
    public static final class Result {
        public double[] brate = new double[3];
        public double tm;
    }

    /**
     * NOGIMLOC (alias LOCSKIRT): set CALCMAN3, then fall into WCALC.
     * @return true if CALCMAN2 is ON (start the maneuver via NEWANGL+1), false to continue (CALCMAN2).
     */
    public boolean nogimloc(double[] cof, double am, Result out) {
        // SET CALCMAN3
        setFlag("CALCMAN3");
        return wcalc(cof, am, out);
    }

    /** WCALC: choose the maneuver rate from the list of four and compute BRATE and TM. */
    public boolean wcalc(double[] cof, double am, Result out) {
        int index = Agc.toSigned(agc.get("RATEINDX")) / 2; // LXC,1 RATEINDX ; list entries are DP
        index = Math.max(0, Math.min(3, index));
        double arate = ARATE[index];
        // SR4 CALL DELCOMP : incremental rotation matrix DEL for 1 sec rotation about COF
        new DelcompHook(agc).delcomp(arate / 16.0, cof);
        // DLOAD* ARATE,1 ; VXSC COF ; STODL BRATE -- component maneuver rates 45 deg/sec
        for (int i = 0; i < 3; i++) {
            out.brate[i] = arate * cof[i];
        }
        // DLOAD AM ; DMP DDV* ANGLTIME ARATE,1 ; SR 5 ; STORE TM (maneuver time, scaled as T2)
        out.tm = am * ANGLTIME / arate / 32.0;
        // SETGO CALCMAN2 / NEWANGL +1 : 0 (OFF) = continue maneuver, 1 (ON) = start maneuver
        setFlag("CALCMAN2");
        return true;
    }

    private void setFlag(String name) {
        agc.set(name, 1);
    }

    /** Placeholder wrapper for the shared DELCOMP interpretive subroutine (defined elsewhere). */
    private static final class DelcompHook {
        private final Agc agc;

        DelcompHook(Agc agc) {
            this.agc = agc;
        }

        void delcomp(double angle, double[] axis) {
            // implemented by the translation of the file containing DELCOMP
        }
    }
}
