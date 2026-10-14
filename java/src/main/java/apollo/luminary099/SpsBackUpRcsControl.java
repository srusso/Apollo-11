package apollo.luminary099;

import apollo.agc.Agc;

/**
 * SPS_BACK-UP_RCS_CONTROL.agc (Luminary 099, pp. 1507-1510).
 * SPSRCS: phase-plane control of all RCS jets in the docked configuration, per axis
 * (P, U, V; axis selected by AXISCTR = -1, 0, +1). Author: E. M. Oshika (AC Electronics).
 *
 * Steps: (1) jet sense test; (2) rate dead band test; (3) outer rate limit test;
 * (4) coast zone test; (5) inhibition logic using the per-axis counters.
 * Inputs: E, EDOT, TJP/TJU/TJV. Outputs: TJP/TJU/TJV. Called via IBNKCALL SPSRCE with
 * interrupts inhibited.
 */
public class SpsBackUpRcsControl {
    private final Agc agc;

    /** RATELIM2: 1.125 deg/sec. */
    public static final int RATELIM2 = 0x1A2;      // OCT 00632
    /** RATELIM1 = CALLCODE = 00032, corresponds to 1.73 deg/sec. */
    public static final int RATELIM1 = 0x1A;
    /** RATEDB1 = TBUILDFX = 00045, corresponds to 0.101 deg/sec. */
    public static final int RATEDB1 = 0x25;
    /** HALF = BIT14. */
    public static final int HALF = 0x2000;
    /** DRIFTBIT = BIT8 of DAPBOOLS. */
    public static final int DRIFTBIT = 0x80;
    /** Jet inhibition counter values, indexed by axis + 1 (P: 4, U and V: 10). */
    public static final int[] UTIME = {4, 10, 10};

    public SpsBackUpRcsControl(Agc agc) {
        this.agc = agc;
    }

    private static int s(int w) { return Agc.toSigned(w); }

    private int axis() { return s(agc.get("AXISCTR")); }

    private String tj() { return "TJU" + (axis() >= 0 ? "+" : "") + axis(); }

    private String ctr() { return "UJETCTR" + (axis() >= 0 ? "+" : "") + axis(); }

    private int oldSense() { return s(agc.get("DAPTREG1")); } // OLDSENSE

    private void setOldSense(int v) { agc.set("DAPTREG1", Agc.fromSigned(v)); }

    /** High-order word of the fractional product of two single-precision words (MP). */
    private static int mp(int a, int b) { return (int) (((long) a * b) / 16384); }

    /** SPSRCS: jet sense test. */
    public void spsrcs() {
        int tju = s(agc.get(tj()));
        if (tju > 0) {
            plusFire();
        } else if (tju < 0) {
            negFire();
        } else {
            setOldSense(0);          // jets off
            spsStart();
        }
    }

    /** NEGFIRE: jets firing negatively. */
    private void negFire() {
        setOldSense(-1);
        rateDeadBand(s(agc.get("EDOT")));
    }

    /** PLUSFIRE: jets firing positively. */
    private void plusFire() {
        setOldSense(1);
        rateDeadBand(-s(agc.get("EDOT")));
    }

    /** Rate dead band test: if DRIFTBIT is 0 use a 0.10 deg/sec target rate. */
    private void rateDeadBand(int signedRate) {
        int target = ((~agc.get("DAPBOOLS")) & DRIFTBIT) != 0 ? RATEDB1 : 0;
        if (signedRate + target <= 0) {
            spsStart();
        } else {
            inhibitionLogic();      // POSTHRST +3: leave jets on
        }
    }

    /** SPSSTART: outer rate limit test, then coast zone test. */
    private void spsStart() {
        int edot = s(agc.get("EDOT"));
        int hi = mp(edot, RATELIM1);
        if (hi > 0) {
            negThrust();            // outer rate limit exceeded
            return;
        }
        if (hi < 0) {
            posThrust();            // outer rate limit exceeded (negative rate)
            return;
        }
        // Coast zone test: E + EDOT, scaled by the pad-loaded deadband DKDB.
        int prod = mp(edot + s(agc.get("E")), s(agc.get("DKDB")));
        if (prod == 0) {
            tjZero();
        } else if (prod < 0) {
            // Below the line: positive thrust unless EDOT >= RATELIM2.
            if (edot - RATELIM2 <= 0) {
                posThrust();
            } else {
                tjZero();
            }
        } else {
            // Above the line: negative thrust unless EDOT <= -RATELIM2.
            if (edot + RATELIM2 <= 0) {
                tjZero();
            } else {
                negThrust();
            }
        }
    }

    /** NEGTHRST: jet time = -HALF. */
    private void negThrust() { storeTj(-HALF); }

    /** POSTHRST: jet time = +HALF. */
    private void posThrust() { storeTj(HALF); }

    /** TJZERO: jet time = 0. */
    private void tjZero() { storeTj(0); }

    private void storeTj(int v) {
        agc.set(tj(), Agc.fromSigned(v));
        inhibitionLogic();
    }

    /** Inhibition logic (CCS OLDSENSE onward). */
    private void inhibitionLogic() {
        int old = oldSense();
        if (old == 0) {
            ctrCheck();
        } else if (old > 0) {
            negCheck(s(agc.get(tj())));       // POSCHECK
        } else {
            negCheck(-s(agc.get(tj())));      // NEGCHECK
        }
    }

    /** NEGCHECK / POSCHECK: return if jet time has the same sign as OLDSENSE. */
    private void negCheck(int signedTj) {
        if (signedTj > 0) {
            return;                            // TC Q
        }
        setCtr();                              // jets commanded off or reversed
    }

    /** SETCTR: set inhibition counter, zero jet time and return. */
    private void setCtr() {
        agc.set(ctr(), UTIME[axis() + 1]);
        zapTj();
    }

    /** ZAPTJ: set jet time to zero. */
    private void zapTj() { agc.set(tj(), 0); }

    /** CTRCHECK: while the inhibition counter is positive, inhibit firings. */
    private void ctrCheck() {
        if (s(agc.get(ctr())) > 0) {
            zapTj();
        }
    }
}
