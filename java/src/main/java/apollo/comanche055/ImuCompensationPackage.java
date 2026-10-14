package apollo.comanche055;

import apollo.agc.Agc;

/**
 * IMU_COMPENSATION_PACKAGE.agc
 *
 * PIPA scale-factor/bias compensation of accumulated delta-V (1/PIPA) and IRIG gyro drift
 * compensation (IRIGCOMP, 1/GYRO, NBDONLY, LASTBIAS).
 *
 * Double precision erasable quantities are two consecutive words (major at NAME, minor at NAME+1),
 * each 14 magnitude bits plus sign, so the double value is major*2^14 + minor in one's-complement
 * arithmetic; the helpers dpGet/dpSet model this.
 */
public final class ImuCompensationPackage {
    private static final int POSMAX = 037777;
    private static final String[] AXIS = {"X", "Y", "Z"};
    private static final int PRIO21 = 021000;
    private static final int PRIO31 = 031000;

    private final Agc agc;

    public ImuCompensationPackage(Agc agc) {
        this.agc = agc;
    }

    // ----- double-precision helpers -----
    private long dpGet(String sym) {
        return ((long) Agc.toSigned(agc.get(sym)) << 14) + Agc.toSigned(agc.get(sym + "+1"));
    }

    private void dpSet(String sym, long v) {
        agc.set(sym, Agc.fromSigned((int) (v >> 14)));
        agc.set(sym + "+1", Agc.fromSigned((int) (v & 0x3FFF)));
    }

    private void dpAdd(String sym, long v) {
        dpSet(sym, dpGet(sym) + v);
    }

    /** Saturating single-precision product scaled as the AGC MP instruction followed by a shift. */
    private static long mp(int a, int b) {
        return (long) a * b; // 28-bit product (before >>14 to obtain high word)
    }

    /**
     * 1/PIPA: compensate DELVX/Y/Z for PIPA scale factor error and bias. Entry saves the caller's
     * EBANK in MODE (restored at IRIG1) and is bypassed when GCOMPSW is negative.
     */
    public void onePipa() {
        agc.set("MODE", agc.get("EBANK"));
        agc.set("EBANK", 0 /* LGCOMP */);
        int gsw = Agc.toSigned(agc.get("GCOMPSW"));
        if (gsw < 0) {
            irig1();                                  // bypass
            return;
        }
        for (int i = 4; i >= 2; i -= 1) {              // PIPAZ, PIPAY, PIPAX (buf+2 = 4, then CCS counts down)
            int axis = (i == 4) ? 2 : (i == 3 ? 1 : 0);
            int scf = Agc.toSigned(agc.get("PIPASCF+" + axis));    // (ppm) x 2^-9
            int delvMajor = Agc.toSigned(agc.get("DELV" + AXIS[axis]));
            long prod = mp(scf, delvMajor);            // MP DELVX
            int major = (int) (prod >> 14);
            int minor = (int) (prod & 0x3FFF);
            // fractional pulses scaled 2^14 -> DELVX+1, major part -> added to DELVX
            long frac = (long) minor * 0x20 >> 14;     // MP BIT6 (shift right 9)
            agc.set("DELV" + AXIS[axis] + "+1", Agc.fromSigned((int) frac));
            long majorScaled = ((long) major * 0x20) >> 14;
            dpAdd("DELV" + AXIS[axis], majorScaled << 14);
            // bias: -(PIPABIAS)(DELTAT)
            int bias = -Agc.toSigned(agc.get("PIPABIAS+" + axis));
            long b = (mp(bias, Agc.toSigned(agc.get("1/PIPADT"))) >> 14);
            dpAdd("DELV" + AXIS[axis], b);
        }
        irigcomp();
    }

    /** IRIGCOMP: gyro compensation terms for each axis (ADIA, ADSRA, NBD). */
    public void irigcomp() {
        agc.set("GCOMPSW", 0);                         // commands 2 pulses or less
        // IRIGX
        gcompsub(0, -Agc.toSigned(agc.get("ADIAX")), "DELVX");
        gcompsub(0, Agc.toSigned(agc.get("ADSRAX")), "DELVY");
        driftsub(0, -Agc.toSigned(agc.get("NBDX")));
        // IRIGY
        gcompsub(1, -Agc.toSigned(agc.get("ADIAY")), "DELVY");
        gcompsub(1, Agc.toSigned(agc.get("ADSRAY")), "DELVZ");
        driftsub(1, -Agc.toSigned(agc.get("NBDY")));
        // IRIGZ
        gcompsub(2, -Agc.toSigned(agc.get("ADSRAZ")), "DELVY");
        gcompsub(2, -Agc.toSigned(agc.get("ADIAZ")), "DELVZ");
        driftsub(2, Agc.toSigned(agc.get("NBDZ")));
        if (Agc.toSigned(agc.get("GCOMPSW")) > 0) {    // CCS GCOMPSW: more than 2 pulses
            // NOVAC at PRIO21 for 1/GYRO
            agc.set("NOVAC_PRIORITY", PRIO21);
            oneGyro();
        } else {
            irig1();
        }
    }

    /** IRIG1: restore EBANK of the caller and SWRETURN. */
    public void irig1() {
        agc.set("EBANK", agc.get("MODE"));
        // TCF SWRETURN
    }

    /** GCOMPSUB: GCOMP[axis] += coef * (-DELV) with coefficient scaled (gyro pulses/PIPA pulse) x 2^-3. */
    private void gcompsub(int axis, int coef, String delv) {
        long pipa = -dpGet(delv);                       // DCS: (PIPA pulses) x 2^14
        long product = pipa * coef;                     // x 2^-3 -> gyro pulses x 2^11
        dpAdd("GCOMP+" + (2 * axis), (product >> 3) );  // MP BIT12: scale 2^3
    }

    /** DRIFTSUB: GCOMP[axis] += NBD * DELTAT, then DRFTSUB2 magnitude test. */
    private void driftsub(int axis, int nbd) {
        long dt = Agc.toSigned(agc.get("1/PIPADT"));    // (cs) x 2^8
        long product = (long) nbd * dt;                 // gyro pulses x 2^3
        dpAdd("GCOMP+" + (2 * axis), product >> 11 << 0);
        drftsub2(axis);
    }

    /** DRFTSUB2: set GCOMPSW positive if any gyro command exceeds 2 pulses. */
    private void drftsub2(int axis) {
        int major = Agc.toSigned(agc.get("GCOMP+" + (2 * axis)));
        if (Math.abs(major) > 2) {                      // CCS then MASK NEGONE, CCS
            agc.set("GCOMPSW", 1);
        }
    }

    /** 1/GYRO: rescale GCOMP for IMUPULSE, call the torquing routine and wait for pulses to leave. */
    public void oneGyro() {
        for (int i = 2; i >= 0; i--) {                  // PIPAZ, PIPAY, PIPAX
            int frac = Agc.toSigned(agc.get("GCOMP+" + (2 * i + 1)));
            agc.set("GCOMP+" + (2 * i + 1), Agc.fromSigned(frac >> 7)); // MP BIT8: shift right 7
            int gyro = Agc.toSigned(agc.get("GCOMP+" + (2 * i)));
            agc.set("GCOMP+" + (2 * i), 0);
            long scaled = (long) gyro >> 7;
            dpAdd("GCOMP+" + (2 * i), scaled);          // add to fractional pulses
        }
        // CAF LGCOMP / TC BANKCALL / CADR IMUPULSE
        new ImuModeSwitchingRoutines(agc).imupulse(agc.get("LGCOMP"));
        new ImuModeSwitchingRoutines(agc).imustall();   // wait for pulses to get out
        new Executive(agc).endofjob();                  // TEMPORARY
    }

    /** GCOMP1: rescale GCOMP+1 fractional parts (shift minor part left 7). */
    public void gcomp1() {
        for (int i = 2; i >= 0; i--) {
            int minor = Agc.toSigned(agc.get("GCOMP+" + (2 * i + 1)));
            agc.set("GCOMP+" + (2 * i + 1), Agc.fromSigned((minor << 7) & Agc.WORD_MASK));
        }
    }

    /** V06N30S: display VN 0630 then end the job. */
    public void v06n30s() {
        // VN 0630 (verb 06 noun 30)
        new Executive(agc).endofjob();
    }

    /** NBDONLY: gyro bias (NBD) drift compensation only over the elapsed time since the last pass. */
    public void nbdonly() {
        if (Agc.toSigned(agc.get("GCOMPSW")) < 0) {     // CCS: bypass if negative
            new Executive(agc).endofjob();
            return;
        }
        // CCS FLAGWRD2: only a negative word (PREREAD flag set) lets the routine continue
        if ((agc.get("FLAGWRD2") & Agc.SIGN_BIT) == 0) {
            new Executive(agc).endofjob();
        }
        int now = agc.get("TIME1");
        int prev = agc.get("1/PIPADT");
        agc.set("1/PIPADT", now);
        nbd2(now, prev);
    }

    /** NBD2: compute elapsed time (cs x 2^14) allowing one TIME1 overflow, then apply NBD. */
    private void nbd2(int now, int prev) {
        int diff = Agc.toSigned(Agc.add(prev & Agc.WORD_MASK, ~now & Agc.WORD_MASK));
        int dt = diff;
        if (dt == 0) {                                  // difference = -0: elapsed time zero
            new Executive(agc).endofjob();
            return;
        }
        if (dt < 0) {                                   // TIME1 overflow: absolute difference
            dt = Agc.toSigned(Agc.add(~dt & Agc.WORD_MASK, POSMAX));
        }
        nbd3(dt);
    }

    /** NBD3: DELTAT scaled (cs) x 2^19 into MPAC/VBUF, compensate each axis. */
    private void nbd3(int dt) {
        long deltat = (long) dt >> 5;                   // MP BIT10: shift right 5
        agc.set("GCOMPSW", 0);
        fbiassub(0, -Agc.toSigned(agc.get("NBDX")), deltat);
        fbiassub(1, Agc.toSigned(agc.get("NBDY")), deltat);
        fbiassub(2, -Agc.toSigned(agc.get("NBDZ")), deltat);
        if (Agc.toSigned(agc.get("GCOMPSW")) > 0) {
            oneGyro();
        } else {
            new Executive(agc).endofjob();
        }
    }

    /** FBIASSUB: GCOMP[axis] += NBD * DELTAT, then check magnitude (DRFTSUB2). */
    private void fbiassub(int axis, int nbd, long deltat) {
        dpAdd("GCOMP+" + (2 * axis), ((long) nbd * deltat) >> 14);
        drftsub2(axis);
    }

    /** LASTBIAS: PIPUSE, then NBD compensation over time since the PIPA1 time (PIPTIME1+1). */
    public void lastbias() {
        new ImuModeSwitchingRoutines(agc).pipuse();
        if (Agc.toSigned(agc.get("GCOMPSW")) < 0) {     // bypass if negative
            new Executive(agc).endofjob();
            return;
        }
        int prev = agc.get("1/PIPADT");
        agc.set("1/PIPADT", PRIO31);                    // 2 seconds scaled (cs) x 2^8
        int now = agc.get("PIPTIME1+1");                // time at PIPA1 = 0
        nbd2(now, prev);
    }
}
