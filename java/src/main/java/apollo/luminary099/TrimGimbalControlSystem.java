package apollo.luminary099;

import apollo.agc.Agc;

/**
 * TRIM_GIMBAL_CONTROL_SYSTEM.agc (Luminary 1A build 099, pages 1472-1484).
 * Descent-engine trim gimbal (GTS) control law, gimbal drive timing (TIMEGMBL), channel 12
 * writer (ACDT+C12) and the GTSQRT square-root subroutine with ROOTCYCL Newton iteration.
 *
 * Drive-setting algorithm:
 * <pre>
 *   DEL     = SGN(OMEGA + ALPHA*ABS(ALPHA)/(2K))
 *   NEGUSUM = ERROR*K + ALPHA*(DEL*OMEGA + ALPHA^2/(3K)) + DEL*K^(1/2)*(DEL*OMEGA + ALPHA^2/(2K))^(3/2)
 *   DRIVE   = -SGN(NEGUSUM)
 * </pre>
 * Erasable names (AOSQ, KQ, QDIFF, NEGUQ, ...) are accessed through {@link Agc}. Q axis is
 * index 0, R axis index 2 (QRCNTR), as in the original storage layout (words are accessed as
 * NAME and NAME+2 etc.). Quantities in the control law are modelled as doubles in physical units
 * after unscaling, while control flow and sign conventions follow the original.
 */
public final class TrimGimbalControlSystem {
    private static final int BIT1 = 1, BIT9 = 0x100, BIT10 = 0x200, BIT11 = 0x400, BIT12 = 0x800;
    /** BGIM: mask of channel 12 gimbal drive bits 9-12. */
    public static final int BGIM = 0x0F00; // OCTAL 07400 (bits 9-12 in 15-bit word)
    /** OCT23146 = decimal .6 (fraction). */
    public static final int OCT23146 = 023146;
    /** OCT00240 = decimal 10/1024. */
    public static final int OCT00240 = 0240;
    public static final double NEG_1_3 = -0.33333;
    public static final int ROOTHALF = 026501;
    public static final int OCT11276 = 011276;
    /** GMBLBITA / GMBLBITB: channel 12 gimbal bits, indexed by axis. */
    public static final int GMBLBITA = 01400;
    public static final int GMBLBITB = 06000;

    private final Agc agc;

    public TrimGimbalControlSystem(Agc agc) {
        this.agc = agc;
    }

    private int g(String s) { return Agc.toSigned(agc.get(s)); }
    private void s(String sym, int v) { agc.set(sym, Agc.fromSigned(v)); }
    private static String idx(String base, int axis) { return axis == 0 ? base : base + "+" + axis; }

    /**
     * GTS: entry to the gimbal trim system for attitude errors, rates and accelerations.
     * Entered when descent engine and DAP are on and trim-gimbal control was active (or the Q,R
     * RCS autopilot found a coast zone with small offset acceleration).
     */
    public void gts() {
        s("COTROLER", -1); // next DAP pass through RCS control
        s("SKIPU", 4);
        s("SKIPV", 4);
        s("INGTS", 2); // GTS control indicator positive
        s("QGIMTIMR", 2); // 200 ms timers to avoid runaway/interference by nulling
        s("RGIMTIMR", 2);
        int sr = agc.get("SR");
        s("SAVESR", sr << 1); // CA SR ; AD A  (compensate for right shift due to editing)
        // GTSGO+ON: R axis first, then Q axis (QRCNTR counts 2 -> 0)
        for (int axis = 2; axis >= 0; axis -= 2) {
            goQTrimG(axis);
        }
        agc.set("SR", agc.get("SAVESR")); // restore SR
        goClose();
    }

    /** GOQTRIMG: process one axis (QRCNTR = 0 for Q, 2 for R). */
    private void goQTrimG(int qrcntr) {
        s("QRCNTR", qrcntr);
        // AOS scaled at PI/2; rescale to PI/4 and use full scale for larger estimates.
        double aos = g(idx("AOSQ", 2 * qrcntr)) / 16384.0; // AOSQ, AOSR are double precision, 2 words apart
        double alpha = aos;
        double omega = g(qrcntr == 0 ? "EDOTQ" : "EDOTQ+1") / 16384.0; // rate cells adjacent
        double k = g(idx("KQ", qrcntr / 2)) / 16384.0;
        s("KCENTRAL", g(idx("KQ", qrcntr / 2)));
        if (k == 0) {
            // CONTROL AUTHORITY ZERO: avoid driving engine bell to the stops.
            posDrive(qrcntr, 0.0, true);
            return;
        }
        double error = g(idx("QDIFF", qrcntr)) / 16384.0; // theta (error) scaled at PI
        double k2theta = k * error; // ALGORTHM: first term of NEGUSUM

        // ALPHA^2/(2K), limiting the quotient to avoid overflow (HUGEQUOT).
        double a2;
        if (k - alpha * alpha / 2.0 <= 0) {
            a2 = 0.99994; // POSMAX
        } else {
            a2 = (alpha * alpha / 2.0) / k;
        }
        // FUNCTION = OMEGA + ALPHA*ABS(ALPHA)/(2K)
        double function = omega + (alpha > 0 ? a2 : -a2);
        int del = function > 0 ? 1 : (function == 0 && g("FUNCTION+1") > 0 ? 1 : -1);
        s("DEL", del);
        double k2 = del * omega; // replace OMEGA by DEL*OMEGA (DEFUNCT: zero stays zero)
        k2 += a2; // FUNCT2: DEL*OMEGA + ALPHA^2/(2K)
        // FUNCT3: -ALPHA^2/(6K) + (DEL*OMEGA + ALPHA^2/(2K)) = DEL*OMEGA + ALPHA^2/(3K)
        double a3 = a2 * NEG_1_3 + k2;
        k2theta += a3 * alpha; // two terms of NEGUSUM accumulated
        // GETROOT: K*(DEL*OMEGA + ALPHA^2/(2K))
        double fn = k2 * k;
        double root = gtsqrt(fn); // RSTOFGTS
        double prod = k2 * root; // PRODUCT: (DEL*OMEGA + A^2/2K)^(3/2) * K^(1/2)
        double term = del > 0 ? prod : -prod;
        k2theta += term;
        // NEGUSUM
        posDrive(qrcntr, k2theta, false);
    }

    /** NEGUSUM / NEGDRIVE / POSDRIVE: set new drive from sign of NEGUSUM. */
    private void posDrive(int qrcntr, double negusum, boolean zeroAuthority) {
        int newNegu;
        if (zeroAuthority) {
            newNegu = -1; // POSDRIVE +1 : CS BIT1 path
        } else if (negusum > 0) {
            newNegu = 1; // NEGDRIVE: CA BIT1
        } else if (negusum < 0) {
            newNegu = -1; // POSDRIVE: CS BIT1
        } else {
            newNegu = 0; // zero NEGUSUM stops drive
        }
        String neg = idx("NEGUQ", qrcntr);
        int old = g(neg);
        s(neg, newNegu);
        boolean reversal = old * newNegu < 0;
        if (reversal) {
            // REVERSAL: a zero-drive pause is needed; clear ACCDOT and the gimbal channel bits.
            s(idx("QACCDOT", qrcntr), 0);
            int bit = qrcntr == 0 ? GMBLBITA : GMBLBITB;
            agc.set("CHAN12", agc.get("CHAN12") & ~bit);
        }
        if (old * newNegu <= 0) {
            // ZEROLOUP: request ACDT+C12 call.
            agc.set("RCSFLAGS", agc.get("RCSFLAGS") | agc.get("CALLGMBL"));
        }
        // LOUPE: loop continues in gts().
    }

    /** GOCLOSE: terminate the job (DTCB to CLOSEOUT). */
    private void goClose() {
        // 2CADR CLOSEOUT (EBANK= AOSQ)
        new Object(); // CLOSEOUT handled by the Executive
    }

    /**
     * ACDT+C12: sets bits 9-12 of channel 12 from NEGUQ/NEGUR (negatives of the desired
     * acceleration changes) and sets Q(R)ACCDOT. Must be called with interrupt inhibited.
     */
    public void acdtC12() {
        // CS NEGUQ ; MP ACCDOTQ ; LXCH QACCDOT : QACCDOT = -NEGUQ * ACCDOTQ
        s("QACCDOT", -g("NEGUQ") * g("ACCDOTQ"));
        s("RACCDOT", -g("NEGUQ+2") * g("ACCDOTQ+2"));
        int chnl12 = g("NEGUQ") > 0 ? BIT10 : BIT9; // CCS NEGUQ: positive -> BIT10 else BIT9
        chnl12 += g("NEGUQ+2") > 0 ? BIT12 : BIT11;
        agc.set("ITEMP6", chnl12);
        int ch12 = agc.get("CHAN12") & ~BGIM; // CS BGIM ; RAND CHAN12
        agc.set("CHAN12", ch12 + chnl12); // WRITE CHAN12
        // Turn off request for ACDT+C12 execution.
        agc.set("RCSFLAGS", agc.get("RCSFLAGS") & ~agc.get("CALLGMBL"));
    }

    /**
     * TIMEGMBL: compute drive time needed for the trim gimbal to null offset angular acceleration
     * about Q and R. Uses .4*AOS scaled at PI/8; drive time = ABS(ALPHA/ACCDOT). A zero ALPHA,
     * ACCDOT or quotient turns off the drive. NEGUQ(R) = +1/0/-1; Q(R)GIMTIMR terminates drive.
     * Must be called with interrupt inhibited.
     */
    public void timegmbl() {
        s("ALLOWGTS", 1);
        for (int axis = 2; axis >= 0; axis -= 2) { // R axis first, then TIMQGMBL for Q
            timAxis(axis);
        }
        acdtC12(); // IBNKCALL ACDT+C12 turns off channel bits, sets Q(R)ACCDOTs
    }

    /** TIMQGMBL body for one axis; DONEYET falls through to the next. */
    private void timAxis(int qrndxer) {
        s("ITEMP6", qrndxer);
        int accdot = g(idx("ACCDOTQ", qrndxer));
        if (accdot <= 0) { tgOffNow(qrndxer); return; }
        s("ITEMP3", accdot); // NZACCDOT
        int negAos = -g(idx("AOSQ", 2 * qrndxer)); // AOS is D.P.; high word used
        if (negAos == 0) { tgOffNow(qrndxer); return; }
        // -1.6*AOS at PI/2 = -.4*AOS at PI/8
        int scaled = negAos + (int) Math.round(negAos * 0.6);
        if (Math.abs(scaled) > 0x3FFF) {
            // overflow: start drive without waitlist, with direction -SGN(AOS)
            s(idx("NEGUQ", qrndxer), negAos > 0 ? 1 : -1);
            notAllow(qrndxer);
            return;
        }
        int negu;
        int itemp2;
        if (scaled > 0) { // POSALPH path: ALPHA negative
            itemp2 = scaled;
            negu = 1;
        } else {
            itemp2 = -scaled;
            negu = -1;
        }
        // stored: ITEMP2 = -ABS(.4*AOS) at PI/8, negu = SGN(ALPHA)-like sign convention
        itemp2 = -Math.abs(scaled);
        s(idx("NEGUQ", qrndxer), scaled > 0 ? 1 : -1);
        int twoAccdot = accdot * 2 / 4 * 2; // MP BIT12: 2*ACCDOT scaled at PI/8
        if (twoAccdot + itemp2 <= 0) { // drive time more than two seconds?
            notAllow(qrndxer);
            return;
        }
        int quotient = (int) (((long) -itemp2 * OCT00240 * 2) / Math.max(1, accdot)); // DV NZACCDOT
        if (quotient == 0) { tgOffNow(qrndxer); return; }
        s(idx("QGIMTIMR", qrndxer / 2), quotient); // DRIVEON
    }

    private void tgOffNow(int qrndxer) {
        s(idx("NEGUQ", qrndxer), 0);
    }

    /** NOTALLOW: drive time more than 2 s; no waitlist call, GTS attitude-rate control disallowed. */
    private void notAllow(int qrndxer) {
        s(idx("QGIMTIMR", qrndxer / 2), 025); // OCT31
        s("ALLOWGTS", 0);
    }

    /**
     * GTSQRT: square root of a double precision value (FUNCTION, FUNCTION+1) returning a
     * single precision root of the 14 most significant bits; SHFTFLAG gets the binary exponent.
     * Zero or negative arguments yield zero. Modelled here on a real value.
     */
    public double gtsqrt(double arg) {
        if (arg <= 0) {
            s("SHFTFLAG", 0);
            return 0;
        }
        // Scale arg down by powers of 4 until it is below 1/4... (SCALLOOP / MULBUSH), then Newton.
        int index = 0;
        double scaled = arg;
        while (scaled < 0.25 && index < 12) {
            scaled *= 4;
            index += 2;
        }
        s("ININDEX", index);
        double halfarg = scaled / 2.0;
        double x = ((int) (scaled * 16384) & 0x2000) != 0 ? ROOTHALF / 32768.0 * 2 : 0.99994;
        x = Math.min(x, 0.99994);
        for (int i = 0; i < 3; i++) {
            x = rootCycl(x, halfarg);
        }
        return x * Math.pow(2, -index / 2.0);
    }

    /** ROOTCYCL: one Newton iteration: X' = (X + ARG/X)/2, with ARG/2 given. */
    public static double rootCycl(double x, double halfArg) {
        return x / 2.0 + halfArg / x;
    }
}
