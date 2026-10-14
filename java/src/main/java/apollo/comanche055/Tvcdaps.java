package apollo.comanche055;

import apollo.agc.Agc;

/**
 * TVCDAPS: Command Module SPS thrust-vector-control digital autopilot. Self-perpetuating T5
 * tasks (PITCHDAP and YAWDAP, each calling the other every half DAP sample) generate the pitch
 * and yaw SPS gimbal actuator commands from body-axis rate commands (cross-product steering),
 * using CDU rate derivation, gimbal/body transformation, error integration with limiting, the
 * generalised 6th-order CSM or CSM/LM filter, an output limiter, and the CG-offset tracker.
 *
 * Double-precision quantities are held as a long in units of 2^-28 (upper word * 2^14 + lower
 * word), as in the sibling translations; single words are signed 14-bit fractions (2^-14).
 */
public class Tvcdaps {
    /** ACTSAT: actuator limit (6 deg), scaled at 1 ASCREV. */
    public static final int ACTSAT = 253;
    /** 1/ACTSAT: reciprocal of ACTSAT. */
    public static final double ONE_OVER_ACTSAT = 0.0039525692;
    /** ERRLIM = BIT13: filter input limit, B-3 revs (45 deg). */
    public static final int ERRLIM = 010000;
    /** 1/ERRLIM = BIT3. */
    public static final int ONE_OVER_ERRLIM = 04;
    /** 1/RTLIM: .004715(CDUDIF) = 0 if CDU difference is below 2.33 degrees per sample. */
    public static final double ONE_OVER_RTLIM = 0.004715;
    /** 1-E(-AT) (OCT 00243). */
    public static final int ONE_MINUS_E_NEG_AT = 000243;
    /** E(-AT) (OCT 37535). */
    public static final int E_NEG_AT = 037535;
    private static final int BIT3 = 04;
    private static final int BIT11 = 02000;
    private static final int BIT12 = 04000;
    private static final int BIT13 = 010000;
    private static final int BIT14 = 020000;

    private final Agc agc;

    public Tvcdaps(Agc agc) {
        this.agc = agc;
    }

    // ---- helpers: single words, double words, arithmetic ----

    private int gs(String s) {
        return Agc.toSigned(agc.get(s));
    }

    private void ss(String s, int v) {
        agc.set(s, Agc.fromSigned(v));
    }

    private long dp(String name) {
        return ((long) gs(name) << 14) + gs(name + "+1");
    }

    private void setDp(String name, long v) {
        int sign = v < 0 ? -1 : 1;
        long mag = Math.abs(v);
        ss(name, sign * (int) (mag >> 14));
        ss(name + "+1", sign * (int) (mag & 0x3FFF));
    }

    /** DAS: double-precision add into memory. */
    private void das(String name, long v) {
        setDp(name, dp(name) + v);
    }

    /** DXCH: exchange a double word between two erasable locations (copy src to dst). */
    private void copy(String dst, String src) {
        setDp(dst, dp(src));
    }

    /** EXTEND MP: product of two single words as a double word (2^-28 units). */
    private static long mp(int a, int b) {
        return (long) a * b;
    }

    /** MP of a double word (hi in A, lo through L) with a single constant, summed as the source does. */
    private static long mpDp(long dpv, int k) {
        long sign = dpv < 0 ? -1 : 1;
        long mag = Math.abs(dpv);
        long hi = mag >> 14;
        long lo = mag & 0x3FFF;
        return sign * (hi * k + (lo * k) / 16384);
    }

    /** MP then DDOUBL (twice the product, double precision). */
    private static long dmul2(int a, int b) {
        return 2L * a * b;
    }

    /** Upper word of a double-precision value (as picked up in A). */
    private static int hi(long v) {
        return (int) (v / 16384);
    }

    private void setBit14(int mask) {
        agc.set("CHAN14", agc.get("CHAN14") | mask);
    }

    private int n10(int off) {
        return gs("N10+" + off);
    }

    private boolean lemOn() {
        return (agc.get("DAPDATR1") & BIT14) != 0;
    }

    private boolean lemOnBit13() {
        return (agc.get("DAPDATR1") & BIT13) != 0;
    }

    /** HACK (TVCSTROKETEST): stroke test waveform generation. */
    private void hack() {
        new TvcStrokeTest(agc).hack(0);  // Q = return address (modelled as a call)
    }

    private void stroker() {
        if (gs("STROKER") != 0) {   // CCS STROKER: test-start or test-in-progress
            hack();
        }
    }

    /** RESUME: end of the T5 task. */
    private void resume() {
        // return from interrupt (TCF RESUME)
    }

    /** T5 call markers: GENADR PITCHDAP / DAPINIT / YAWDAP (upper words of the T5 2CADRs). */
    public static final int PITCHT5 = 1;
    public static final int DAPT5 = 2;
    public static final int YAWT5 = 3;

    // ======================= PITCH DAP =======================

    /** PITCHDAP: pitch TVC DAP, T5 entry (normal, or via DAPINIT). */
    public void pitchdap() {
        agc.set("T5LOC", YAWT5);           // set up T5 call for yaw autopilot
        agc.set("TIME5", agc.get("T5TVCDT"));
        pstroker();
        pcduDots();
        pintegrl();
    }

    /** PSTROKER: check for stroke test (STRKFLG). */
    private void pstroker() {
        stroker();
    }

    /** PCDUDOTS: compute CDUYDOT and CDUZDOT (used by pitch and yaw). */
    private void pcduDots() {
        int oldY = agc.get("PCDUYPST");
        agc.set("PCDUYPST", agc.get("CDUY"));
        ss("MCDUYDOT", rlimTest(cduDiff(oldY, agc.get("PCDUYPST"))));   // minus, sc. 1/2TVCDT revs/sec
        int oldZ = agc.get("PCDUZPST");
        agc.set("PCDUZPST", agc.get("CDUZ"));
        ss("MCDUZDOT", rlimTest(cduDiff(oldZ, agc.get("PCDUZPST"))));
    }

    /** MSU: old minus new, one's complement modulo 2 pi. */
    private static int cduDiff(int oldWord, int newWord) {
        return Agc.toSigned(Agc.add(oldWord, (~newWord) & Agc.WORD_MASK));
    }

    /** RLIMTEST: zero the rate if it exceeds 2.33 deg in one sample period. */
    private int rlimTest(int diff) {
        return ((int) (Math.abs(diff) * ONE_OVER_RTLIM) != 0) ? 0 : diff;
    }

    /** PINTEGRL: integral of body-axis pitch-rate error, sc. B-1 revs; prepares OMEGAYB. */
    private void pintegrl() {
        setDp("ERRBTMP", dp("PERRB"));
        das("ERRBTMP", dp("OMEGAYC"));
        long omegayb = dmul2(-gs("COSCDUZ"), gs("COSCDUX"));
        omegayb = dmul2(hi(omegayb), gs("MCDUYDOT"));
        omegayb += dmul2(-gs("MCDUZDOT"), gs("SINCDUX"));
        setDp("OMEGAYB", omegayb);        // sc. 1/2TVCDT revs
        das("ERRBTMP", -omegayb);          // sign change, integrate
        perorlim();
    }

    /** PERORLIM: pitch body-axis-error input limiter. */
    private void perorlim() {
        errorlim();
        pforward();
    }

    /** PFORWARD: prepare filter storage for the pitch channel, compute the forward filter. */
    private void pforward() {
        copy("TMP1", "PTMP1");
        copy("TMP3", "PTMP3");
        copy("TMP5", "PTMP5");
        fwdfltr();
        poffset();
    }

    /** POFFSET: add the pitch offset (CG tracker), then limit. */
    private void poffset() {
        das("CMDTMP", dp("PDELOFF"));      // scaled B+0 ASCREV
        actlim();                          // PACLIM
        pout();
    }

    /** POUT: incremental pitch command to the optics error counter, count release. */
    private void pout() {
        ss("TVCPITCH", gs("TVCPITCH") + gs("CMDTMP") - gs("PCMD"));
        setBit14(BIT11);                   // TVCPITCH count release
        pprecomp();
    }

    /** PPRECOMP: prepare filter storage for pitch and precompute for the next pass. */
    private void pprecomp() {
        copy("TMP2", "PTMP2");
        copy("TMP4", "PTMP4");
        copy("TMP6", "PTMP6");
        precomp();
        delbarp();
    }

    /** DELBARP: update the pitch offset-tracker filter. */
    private void delbarp() {
        offsetTracker("DELPBAR");
        pcopy();                           // PCOPYCYC
        resume();                          // PDAPEND
    }

    /** Offset tracker: DELBRTMP = DELxBAR*E(-AT) + CMDTMP*(1-E(-AT)). */
    private void offsetTracker(String delBar) {
        long t = mpDp(dp(delBar), E_NEG_AT);
        t += mp(gs("CMDTMP"), ONE_MINUS_E_NEG_AT);
        setDp("DELBRTMP", t);
    }

    /** PCOPY: pitch copycycle (called via pitch DAP or the TVC restart package). */
    public void pcopy() {
        ss("TVCPHASE", gs("TVCPHASE") + 1);     // restart-protect (1)
        copy("PTMP1", "TMP1");
        copy("PTMP2", "TMP2");
        copy("PTMP3", "TMP3");
        copy("PTMP4", "TMP4");
        copy("PTMP5", "TMP5");
        copy("PTMP6", "TMP6");
        pmisc();
    }

    /** PMISC: pitch-rate-error integrator, actuator command, offset tracker. */
    private void pmisc() {
        agc.set("AK1", agc.get("ERRBTMP"));     // for pitch needles, sc. B-1 revs
        copy("PERRB", "ERRBTMP");
        agc.set("PCMD", agc.get("CMDTMP"));
        copy("DELPBAR", "DELBRTMP");
        ss("TVCPHASE", gs("TVCPHASE") + 1);     // pitch copycycle completed (2)
    }

    // ======================= YAW DAP =======================

    /** YAWDAP: yaw TVC DAP, T5 entry. */
    public void yawdap() {
        agc.set("T5LOC", PITCHT5);
        agc.set("TIME5", agc.get("T5TVCDT"));
        stroker();                          // YSTROKER; uses body rates from PITCHDAP (PCDUDOTS)
        yintegrl();
    }

    /** YINTEGRL: integral of body-axis yaw-rate error; prepares OMEGAZB. */
    private void yintegrl() {
        setDp("ERRBTMP", dp("YERRB"));
        das("ERRBTMP", dp("OMEGAZC"));
        long omegazb = dmul2(gs("COSCDUZ"), gs("SINCDUX"));
        omegazb = dmul2(hi(omegazb), gs("MCDUYDOT"));
        omegazb += dmul2(-gs("MCDUZDOT"), gs("COSCDUX"));
        setDp("OMEGAZB", omegazb);         // sc. 1/2TVCDT revs
        das("ERRBTMP", -omegazb);
        yerorlim();
    }

    /** YERORLIM: yaw body-axis-error input limiter. */
    private void yerorlim() {
        errorlim();
        yforward();
    }

    /** YFORWARD: prepare filter storage for yaw, compute the forward filter. */
    private void yforward() {
        copy("TMP1", "YTMP1");
        copy("TMP3", "YTMP3");
        copy("TMP5", "YTMP5");
        fwdfltr();
        yoffset();
    }

    /** YOFFSET: add the yaw offset, round and limit (YACLIM). */
    private void yoffset() {
        das("CMDTMP", dp("YDELOFF"));
        actlim();
        yout();
    }

    /** YOUT: incremental yaw command to the error counter, count release. */
    private void yout() {
        ss("TVCYAW", gs("TVCYAW") + gs("CMDTMP") - gs("YCMD"));
        setBit14(BIT12);                   // TVCYAW count release
        yprecomp();
    }

    /** YPRECOMP: prepare filter storage for yaw and precompute. */
    private void yprecomp() {
        copy("TMP2", "YTMP2");
        copy("TMP4", "YTMP4");
        copy("TMP6", "YTMP6");
        precomp();
        delbary();
    }

    /** DELBARY: update the yaw offset-tracker filter. */
    private void delbary() {
        offsetTracker("DELYBAR");
        ycopy();                           // YCOPYCYC
        resume();                          // YDAPEND
    }

    /** YCOPY: yaw copycycle (called via yaw DAP or the TVC restart package). */
    public void ycopy() {
        ss("TVCPHASE", gs("TVCPHASE") + 1);     // restart-protect (3)
        copy("YTMP1", "TMP1");
        copy("YTMP2", "TMP2");
        copy("YTMP3", "TMP3");
        copy("YTMP4", "TMP4");
        copy("YTMP5", "TMP5");
        copy("YTMP6", "TMP6");
        ymisc();
    }

    /** YMISC: yaw-rate-error integrator, command and offset tracker; resets TVCPHASE. */
    private void ymisc() {
        agc.set("AK2", agc.get("ERRBTMP"));     // for yaw needles, sc. B-1 revs
        copy("YERRB", "ERRBTMP");
        agc.set("YCMD", agc.get("CMDTMP"));
        copy("DELYBAR", "DELBRTMP");
        agc.set("TVCPHASE", 0);                 // yaw copycycle completed
    }

    // ======================= COMMON SUBROUTINES =======================

    /** DAPINIT: T5 entry from TVCINT4; schedule PITCHDAP in TVCDT secs, store initial CDUs. */
    public void dapinit() {
        // T5TVCDT = POSMAX - TVCDT/2 + 1; time to PITCHDAP is TVCDT seconds
        agc.set("TIME5", Agc.add(Agc.add(Agc.add(Agc.fromSigned(-1), agc.get("T5TVCDT")), 040000), agc.get("T5TVCDT")));
        agc.set("T5LOC", PITCHT5);
        agc.set("PCDUYPST", agc.get("CDUY"));
        agc.set("PCDUZPST", agc.get("CDUZ"));
        // TCF NOQRSM
    }

    /** ERRORLIM: body-axis-error input limiter (checks and writes upper word only). */
    private void errorlim() {
        int e = gs("ERRBTMP");
        if (e * ONE_OVER_ERRLIM / 16384 != 0) {
            ss("ERRBTMP", e > 0 ? ERRLIM : -ERRLIM);
        }
    }

    /** ACTLIM: round the low word into CMDTMP and limit the actuator command. */
    private void actlim() {
        int cmd = gs("CMDTMP");
        int twice = 2 * gs("CMDTMP+1");
        if (twice > 16383) {
            cmd += 1;          // overflow carried into the round-up
        } else if (twice < -16383) {
            cmd -= 1;
        }
        if ((int) (cmd * ONE_OVER_ACTSAT) != 0) {
            cmd = cmd > 0 ? ACTSAT : -ACTSAT;
        }
        ss("CMDTMP", cmd);
    }

    /** FWDFLTR: filter computation of the present output (cascades 1-3, then OPTVARK). */
    private void fwdfltr() {
        for (String z : new String[] {"DAP1", "DAP2", "DAP3", "CMDTMP", "DELBRTMP"}) {
            setDp(z, 0);
        }
        // 1DAPCAS: N10
        setDp("DAP1", mpDp(dp("ERRBTMP"), n10(0)) + dp("TMP1"));
        // 2DAPCAS: N20
        setDp("DAP2", mpDp(dp("DAP1"), n10(5)) + dp("TMP3"));
        if (!lemOn()) {
            setDp("DAP3", dp("DAP2"));     // LEM off
        } else {
            // 3DAPCAS: N30
            setDp("DAP3", mpDp(dp("DAP2"), n10(10)) + dp("TMP5"));
        }
        optvark();
    }

    /** OPTVARK: variable gain package (also sign change in the forward loop). */
    private void optvark() {
        long out = mpDp(-dp("DAP3"), gs("VARK"));     // sc. 1/(8 ASCREV)
        out *= 4;                                       // DDOUBL twice: sc. B+1 ASCREVS
        setDp("CMDTMP", out);
    }

    /** PRECOMP: filter precomputations for the next pass (cascades 1-3). */
    private void precomp() {
        cascade("ERRBTMP", "DAP1", "TMP1", "TMP2", 1, 3, 2, 4);     // first cascade: N11/2 D11/2 N12 D12
        cascade("DAP1", "DAP2", "TMP3", "TMP4", 6, 8, 7, 9);        // second cascade: N21/2 D21/2 N22 D22
        if (!lemOnBit13()) {
            return;                                                   // exit if LEM off
        }
        cascade("DAP2", "DAP3", "TMP5", "TMP6", 11, 13, 12, 14);    // third cascade: N31/2 D31/2 N32 D32
    }

    /** One filter cascade precomputation. */
    private void cascade(String in, String out, String tmpA, String tmpB, int n1, int d1, int n2, int d2) {
        long sum = mpDp(dp(in), n10(n1)) + mpDp(-dp(out), n10(d1));
        long a = dp(tmpB) + 2 * sum;
        setDp(tmpA, a);
        long b = mpDp(dp(in), n10(n2)) + mpDp(-dp(out), n10(d2));
        setDp(tmpB, b);
    }
}
