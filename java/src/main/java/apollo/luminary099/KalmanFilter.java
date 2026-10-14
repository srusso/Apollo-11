package apollo.luminary099;

import apollo.agc.Agc;

/**
 * KALMAN_FILTER.agc - LM DAP rate filter (RATELOOP): converts jet-on times (TJP) into
 * a torque estimate, accumulates the DOWNTORK downlist registers, and finally computes
 * the jet-acceleration rates JETRATER / JETRATEQ.
 */
public class KalmanFilter {
    private final Agc agc;

    /** -100MST6: DEC -160. */
    public static final int MINUS_100MST6 = -160;
    /** ELEVEN constant used for scaling (10.24 plus). */
    public static final int ELEVEN = 11;

    public KalmanFilter(Agc agc) {
        this.agc = agc;
    }

    private static int s(int w) { return Agc.toSigned(w); }

    /** RATELOOP: loop over the three axis indices (DAPTEMP6 = 2,1,0). */
    public void rateLoop() {
        for (int idx = 2; idx >= 0; idx--) {
            agc.set("DAPTEMP6", idx);
            int q = idx * 2;               // DOUBLE; TS Q (downlist offset)
            int tjp = s(agc.get("TJP") ); // indexed by idx in the original (TJP + idx)
            int a;
            if (tjp == 0) {
                // CCS fall-through to LOOPRATE with A = 0 torque factor
                a = loopRate(idx, 0);
            } else {
                int mag = Math.abs(tjp);
                if (mag - 160 <= 0) {
                    a = smallTju(idx);      // SMALLTJU
                } else {
                    // subtract (or add) 100 ms from TJP, saturate torque at +/-0.1
                    int step = tjp > 0 ? MINUS_100MST6 : -MINUS_100MST6;
                    agc.set("TJP", Agc.fromSigned(tjp + step));
                    a = loopRate(idx, tjp > 0 ? -1 : 1);
                }
            }
            storTork(q, a);                 // STORTORK / NEGTORK
        }
        roTorque();                         // ROTORQUE
    }

    /** LOOPRATE: scale by NO.PJETS, store signed torque in DAPTEMP1+idx. */
    private int loopRate(int idx, int sign) {
        int pjets = s(agc.get("NO.PJETS"));
        int product = sign * pjets;
        agc.set("DAPTEMP1", Agc.fromSigned(product)); // signed torque at 1 jet-sec for filter
        return product >> 4;                           // rescale (MP BIT10): one bit ~ 2 jet-msec
    }

    /** SMALLTJU: TJP is small; clear it and use ELEVEN as the torque factor. */
    private int smallTju(int idx) {
        agc.set("TJP", 0);
        return loopRate(idx, ELEVEN);
    }

    /** STORTORK / NEGTORK: increment downlist register (not initialized; overflows). */
    private void storTork(int q, int a) {
        agc.set("DOWNTORK", Agc.add(agc.get("DOWNTORK"), Agc.fromSigned(a)));
    }

    /** ROTORQUE: compute JETRATER and JETRATEQ, then TCF BACKP. */
    public void roTorque() {
        int t2 = s(agc.get("DAPTEMP2"));
        int t3 = s(agc.get("DAPTEMP3"));
        agc.set("JETRATER", Agc.fromSigned(((t2 + t3) * s(agc.get("1JACCR"))) >> 14));
        agc.set("JETRATEQ", Agc.fromSigned(((t2 - t3) * s(agc.get("1JACCQ"))) >> 14));
        // TCF BACKP -> continue in the DAP (not translated here)
    }
}
