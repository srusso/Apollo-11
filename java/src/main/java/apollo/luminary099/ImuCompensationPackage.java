package apollo.luminary099;

import apollo.agc.Agc;

/**
 * IMU_COMPENSATION_PACKAGE.agc - PIPA scale-factor/bias compensation (1/PIPA), gyro drift and
 * acceleration-dependent compensation (IRIGCOMP, NBDONLY, LASTBIAS), and gyro torquing (1/GYRO).
 *
 * GCOMP holds 3 DP gyro-pulse accumulators (GCOMP+0/+1 X, +2/+3 Y, +4/+5 Z), scaled 2^14
 * gyro pulses. Axis index idx = 0,2,4 in the AGC corresponds to X,Y,Z here as 0,1,2.
 */
public class ImuCompensationPackage {
    private final Agc agc;

    /** COMPCHK: DEC -1. */
    public static final int COMPCHK = -1;
    private static final int BIT4 = 0x8, BIT6 = 0x20, BIT8 = 0x80, BIT9 = 0x100, BIT10 = 0x200;

    private static final String[] DELV = {"DELVX", "DELVY", "DELVZ"};
    private static final String[] NBD = {"NBDX", "NBDY", "NBDZ"};
    private static final String[] PIPASCF = {"PIPASCF", "PIPASCF+1", "PIPASCF+2"};
    private static final String[] PIPABIAS = {"PIPABIAS", "PIPABIAS+1", "PIPABIAS+2"};

    public ImuCompensationPackage(Agc agc) {
        this.agc = agc;
    }

    private static int s(int w) { return Agc.toSigned(w); }

    private String gcomp(int axis) { return axis == 0 ? "GCOMP" : "GCOMP+" + (2 * axis); }

    /** 1/PIPA: compensate PIPA readings for scale factor error and bias. */
    public void onePipa() {
        // GCOMPSW negative bypasses compensation (go to IRIG1)
        if (s(agc.get("GCOMPSW")) < 0) {
            irig1();
            return;
        }
        for (int axis = 2; axis >= 0; axis--) {        // PIPAZ, PIPAY, PIPAX
            String delv = DELV[axis];
            int[] pm = VecOps.mp(s(agc.get(PIPASCF[axis])), s(agc.get(delv))); // (PIPA pulses)*2(+5)
            int major = pm[0];
            int[] lo = VecOps.mp(pm[1], BIT6);          // scale 2(+9), shift right 9
            agc.set(delv + "+1", Agc.fromSigned(lo[0]));// fractional PIPA pulses scaled 2(+14)
            int[] hi = VecOps.mp(major, BIT6);
            VecOps.das(agc, delv, hi[0], hi[1]);        // (PIPAI) + (PIPAI)(SFE)
            int[] b = VecOps.mp(-s(agc.get(PIPABIAS[axis])), s(agc.get("1/PIPADT")));
            int[] b2 = VecOps.mp(b[0], BIT4);           // scale 2(+11), shift right 11
            VecOps.das(agc, delv, b2[0], b2[1]);        // - (BIAS)(DELTAT)
        }
    }

    /** IRIGCOMP: compensate gyro drift and acceleration terms; send torquing if needed. */
    public void irigComp(int dummy) {
        agc.set("GCOMPSW", 0);                           // commands 2 pulses or less
        irigX();
        driftSub(0, -s(agc.get("NBDX")));                // -(NBDX)(DELTAT)
        irigY();
        driftSub(1, -s(agc.get("NBDY")));
        irigZ();
        driftSub(2, s(agc.get("NBDZ")));
        if (s(agc.get("GCOMPSW")) > 0) {
            // CA PRIO21 ; TC NOVAC ; 2CADR 1/GYRO
            oneGyro();
        }
        irig1();
    }

    /** IRIG1: restore caller's EBANK and return (SWRETURN). */
    public void irig1() {
        agc.set("EBANK", agc.get("MODE"));
    }

    /** IRIGX: acceleration-dependent drift for the X gyro: -(ADIAX)(PIPAX) + (ADSRAX)(PIPAY). */
    public void irigX() {
        gcompSub(0, -s(agc.get("DELVX")), s(agc.get("ADIAX")), agc.get("DELVX+1"));
        gcompSub(0, -s(agc.get("DELVY")), -s(agc.get("ADSRAX")), agc.get("DELVY+1"));
    }

    /** IRIGY: -(ADIAY)(PIPAY) + (ADSRAY)(PIPAZ). */
    public void irigY() {
        gcompSub(1, -s(agc.get("DELVY")), s(agc.get("ADIAY")), agc.get("DELVY+1"));
        gcompSub(1, -s(agc.get("DELVZ")), -s(agc.get("ADSRAY")), agc.get("DELVZ+1"));
    }

    /** IRIGZ: -(ADSRAZ)(PIPAY) - (ADIAZ)(PIPAZ). */
    public void irigZ() {
        gcompSub(2, -s(agc.get("DELVY")), s(agc.get("ADSRAZ")), agc.get("DELVY+1"));
        gcompSub(2, -s(agc.get("DELVZ")), s(agc.get("ADIAZ")), agc.get("DELVZ+1"));
    }

    /**
     * GCOMPSUB: multiply the PIPA pulses (major in mpac, minor given) by coefficient
     * (GYRO pulses/PIPA pulse x 2(-6)) and add to GCOMP of the axis.
     */
    private void gcompSub(int axis, int mpacMajor, int coeff, int mpacMinorWord) {
        int minor = -Math.abs(0) + (mpacMajor < 0 ? -s(mpacMinorWord) : s(mpacMinorWord));
        int[] a = VecOps.mp(coeff, mpacMajor);           // (gyro pulses) x 2(+8)
        long vbuf = (long) a[0] * VecOps.HALF_RANGE + a[1];
        int[] m = VecOps.mp(coeff, minor);
        vbuf += m[0];
        int hiPart = (int) (vbuf / VecOps.HALF_RANGE);
        int loPart = (int) (vbuf % VecOps.HALF_RANGE);
        int[] h = VecOps.mp(hiPart, BIT9);               // scale 2(+6)
        VecOps.das(agc, gcomp(axis), h[0], h[1]);
        int[] l = VecOps.mp(loPart, BIT9);
        VecOps.das(agc, gcomp(axis), 0, l[0]);
    }

    /** DRIFTSUB: add -(NBD)(DELTAT) for an axis into GCOMP; check magnitude (DRFTSUB2). */
    public void driftSub(int axis, int nbd) {
        int[] p = VecOps.mp(nbd, s(agc.get("1/PIPADT"))); // (gyro pulses) x 2(+3)
        int[] hi = VecOps.mp(p[0], BIT4);                 // scale 2(+11)
        VecOps.das(agc, gcomp(axis), hi[0], hi[1]);
        int[] lo = VecOps.mp(p[1], BIT4);
        VecOps.das(agc, gcomp(axis), 0, lo[0]);
        drftSub2(axis);
    }

    /** DRFTSUB2: set GCOMPSW positive if any gyro command exceeds 2 pulses. */
    private void drftSub2(int axis) {
        int major = s(agc.get(gcomp(axis)));
        if (major != 0) {                                 // 1 pulse or greater
            if (Math.abs(major) + COMPCHK > 0) {          // greater than 2 pulses
                agc.set("GCOMPSW", Math.abs(major) - 1);  // positive
            }
        }
    }

    /** 1/GYRO: scale gyro commands and send them via IMUPULSE, then wait for completion. */
    public void oneGyro() {
        for (int axis = 2; axis >= 0; axis--) {
            String g = gcomp(axis);
            int[] f = VecOps.mp(s(agc.get(g + "+1")), BIT8);        // fractional, shift right 7
            agc.set(g + "+1", Agc.fromSigned(f[0]));
            int pulses = s(agc.get(g));
            agc.set(g, 0);
            int[] m = VecOps.mp(pulses, BIT8);
            VecOps.das(agc, g, m[0], m[1]);
        }
        // CAF LGCOMP ; BANKCALL IMUPULSE ; BANKCALL IMUSTALL ; TCF ENDOFJOB
    }

    /** GCOMP1: rescale the fractional pulse remainders (shift minor part left 7). */
    public void gcomp1() {
        for (int axis = 2; axis >= 0; axis--) {
            String g = gcomp(axis) + "+1";
            int[] r = VecOps.mp(s(agc.get(g)), BIT8);
            agc.set(g, Agc.fromSigned(r[1]));
        }
    }

    /** NBDONLY: gyro bias (and, on the surface, acceleration terms) compensation by elapsed time. */
    public void nbdOnly() {
        if (s(agc.get("GCOMPSW")) < 0) {
            return;                                       // ENDOFJOB
        }
        if (agc.get("FLAGWRD2") == 0) {                   // pre-read T3RUPT may coincide
            return;
        }
        int tem1 = agc.get("FLAGWRD8") & BIT8;            // surface flag
        agc.set("TEM1", tem1);
        if (tem1 != 0) {
            // IBNKCALL PIPASR+3 : read PIPAs without scaling
        }
        int now = agc.get("TIME1");
        int prev = agc.get("1/PIPADT");
        agc.set("1/PIPADT", now);
        int dt = (now - prev) & Agc.WORD_MASK;
        nbd2(dt);
    }

    /** LASTBIAS: final bias compensation using PIPTIME. */
    public void lastBias() {
        // BANKCALL PIPUSE1
        if (s(agc.get("GCOMPSW")) < 0) {
            return;
        }
        agc.set("TEM1", agc.get("FLAGWRD8") & 0x80);      // SURFFBIT
        int prev = agc.get("1/PIPADT");
        agc.set("1/PIPADT", 0x31F);                       // PRIO31: 2 seconds
        nbd2(Agc.add(Agc.fromSigned(-s(prev)), agc.get("PIPTIME+1")));
    }

    /** NBD2/NBD3: scale DELTAT and apply bias (and optionally acceleration) compensation. */
    private void nbd2(int dt) {
        int[] scaled = VecOps.mp(s(dt), BIT10);           // shift right 5
        agc.set("VBUF+2", Agc.fromSigned(scaled[0]));
        agc.set("VBUF+3", Agc.fromSigned(scaled[1]));
        agc.set("GCOMPSW", 0);
        boolean accel = s(agc.get("TEM1")) > 0;
        for (int axis = 0; axis < 3; axis++) {
            if (accel) {
                if (axis == 0) irigX(); else if (axis == 1) irigY(); else irigZ();
            }
            int sign = axis == 2 ? -1 : 1;
            int nbd = s(agc.get(NBD[axis]));
            fBiasSub(axis, (axis == 1 ? nbd : -nbd) * sign * (axis == 2 ? -1 : 1));
        }
        if (s(agc.get("GCOMPSW")) > 0) {
            oneGyro();                                    // gyro commands > 2 pulses
        }
        // else ENDOFJOB
    }

    /** FBIASSUB: NBD * DELTAT (scaled CS 2(+19)) added to GCOMP of the axis. */
    private void fBiasSub(int axis, int nbd) {
        int dtMajor = s(agc.get("VBUF+2"));
        int dtMinor = s(agc.get("VBUF+3"));
        int[] hi = VecOps.mp(nbd, dtMajor);
        VecOps.das(agc, gcomp(axis), hi[0], hi[1]);
        int[] lo = VecOps.mp(nbd, dtMinor);
        VecOps.das(agc, gcomp(axis), 0, lo[0]);
        drftSub2(axis);
    }

    /** GCOMPZER: zero GCOMP and GCOMPSW before the first call to 1/PIPA. */
    public void gcompZer() {
        agc.set("MODE", agc.get("EBANK"));
        agc.set("GCOMPSW", 0);
        for (int i = 0; i < 6; i++) {
            agc.set(i == 0 ? "GCOMP" : "GCOMP+" + i, 0);
        }
        irig1();
    }
}
