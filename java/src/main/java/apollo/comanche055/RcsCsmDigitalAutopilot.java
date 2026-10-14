package apollo.comanche055;

import apollo.agc.Agc;

/**
 * RCS-CSM_DIGITAL_AUTOPILOT.agc (Comanche 055) - phases 1 and 2 of the CSM RCS digital autopilot, run from
 * T5RUPT. Phase 1 (REDORCS/RCSATT) reads the rate-filter, updates attitude and displays FDAI needles;
 * phase 2 (T5PHASE2) reads the hand controller (ROT/translation) and sets up jet selection (T6PROG).
 *
 * T5PHASE: positive = FRESHDAP (start up), 0 = phase 2, -0 -> REDAP, negative = phase 1.
 * Erasables are accessed through the Agc symbol table by name; hardware channels are named CHAN5 / CHAN6 /
 * CHAN12 / CHAN13 / CHAN14 / CHAN31 symbols.
 */
public class RcsCsmDigitalAutopilot {
    private final Agc agc;

    // Constants
    public static final int DELTATT = 0x3FF8;      // OCT 37770: 80 ms (TIME5)
    public static final int DELTATT2 = 0x3FFE;     // OCT 37776: 20 ms (TIME5)
    public static final int ONESEK = 16284;        // 1 sec (TIME5)
    public static final int BITS4_5 = 030;
    public static final int DELAY200 = 16364;      // 200 ms
    public static final int MINUS_384 = -384;
    public static final int DACLIMIT = 384;        // output limit (also 16000 octal-decimal pair in source)
    public static final int NO_T5VAR = 36;         // locations to zero (minus one) in FRESHDAP
    public static final double SLOPE_INIT = 0.24;  // slope of 0.6/sec
    public static final int RCSINIT = 04;
    public static final int RCSINITB = 020004;
    public static final int T5WAIT60 = 16378;      // 6 cs
    public static final int PLUS_14MS = 23;
    public static final int OCT01760 = 01760;      // forced firing bits mask
    public static final int OCT01400 = 01400;      // roll forced firing mask
    public static final int OCT00060 = 060;        // pitch forced firing mask
    public static final int OCT00300 = 0300;       // yaw forced firing mask
    public static final int MANROT = 077;
    public static final int OCT16000 = 016000;     // rate damping flags mask
    /** MANTABLE: commanded manual rates for RATEINDX 0,2,4,6 (+/-). */
    public static final double[] MANTABLE = {
        0.0071111, -0.0071111, 0.028444, -0.028444, 0.071111, -0.071111, 0.284444, -0.284444};
    /** FREETAU: acceleration command (in cs-ish) for RMANNDX/PMANNDX/YMANNDX = 0,1,2,3. */
    public static final int[] FREETAU = {0, 480, -480, 0};
    /** Gains for translation / 2 deg/sec maneuvers (preceding GAIN1 in the source, indexed by ATTKALMN-?). */
    public static final double[] GAIN_TRANS = {0.2112, 0.8400, 0.2112};
    /** GAIN1: Kalman filter gains for initialization of attitude rates. */
    public static final double[] GAIN1 = {
        0.0640, 0.3180, 0.3452, 0.3774, 0.4161, 0.4634, 0.5223, 0.5970, 0.6933, 0.8151, 0.9342,
        0.0174, 0.3600, 0.0174};
    /** GAIN2: scaled 10. */
    public static final double[] GAIN2 = {
        0.0016, 0.0454, 0.0545, 0.0666, 0.0832, 0.1069, 0.1422, 0.1985, 0.2955, 0.4817, 0.8683};

    public RcsCsmDigitalAutopilot(Agc agc) {
        this.agc = agc;
    }

    private int g(String s) { return agc.get(s); }
    private void s(String sym, int v) { agc.set(sym, v); }
    private int sg(String s) { return Agc.toSigned(agc.get(s)); }
    private static boolean bit(int word, int n) { return (word & (1 << (n - 1))) != 0; }
    private boolean isZeroOrNeg(int word) { return word == 0 || word == Agc.WORD_MASK || bit(word, 15); }

    /** REDORCS: restart of autopilot on a T5 rupt; reset T5PHASE negative unless positive; hook T5LOC. */
    public void redorcs() {
        int t5 = g("T5PHASE");
        if (isZeroOrNeg(t5)) { // BZMF
            s("T5PHASE", Agc.fromSigned(-1));
        }
        s("T5LOC", g("RCSATT")); // DXCH T5LOC <- RCSLOC (2CADR RCSATT)
        rcsatt(true);
    }

    /** RCSATT: T5RUPT entry. skipSave true on entry from REDORCS ("RCSATT +1"). */
    public void rcsatt(boolean skipSave) {
        boolean enabled = !bit(g("CHAN31"), 15); // bit15 = 0 if IMU power on and S/C CONT in CMC
        if (!enabled) {
            // set NORATE flag (RCSFLAGS bit14), HOLDFLAG +POSMAX, zero error signals
            s("RCSFLAGS", g("RCSFLAGS") | 0x2000);
            s("HOLDFLAG", Agc.WORD_MASK >> 1);
            s("ERRORX", 0);
            s("ERRORY", 0);
            s("ERRORZ", 0);
            if (bit(g("CHAN31"), 14)) {
                // not in free mode: schedule FRESHDAP reinit in 100 ms and zero jets
                s("T5PHASE", 0);
                s("TIME5", 037766);
                zerojet();
                kmatrix();
                return;
            }
            // in free mode: fall to SETT5
        }
        sett5();
    }

    /** SETT5: dispatch on T5PHASE: + FRESHDAP, 0 PHASE2, -0 REDAP, - phase 1. */
    private void sett5() {
        int t5 = sg("T5PHASE");
        int w = g("T5PHASE");
        if (t5 > 0) {
            freshdap();
        } else if (w == 0) {
            t5phase2();
        } else if (w == Agc.WORD_MASK) {
            redap();
        } else {
            phase1();
        }
    }

    /** Phase 1 body after SETT5 (T5PHASE negative): reset for phase 2 in 20 ms, test IMU, derive rates. */
    private void phase1() {
        s("T5PHASE", g("T5PHASE")); // TS T5PHASE (overflow-free copy)
        s("T5TIME", g("TIME5"));    // compensate delays in T5
        s("TIME5", DELTATT2);       // T5RUPT in 20 ms for phase 2
        boolean imuNotOk = bit(g("IMODES33"), 6);
        if (!imuNotOk) {
            ratefilt();
        } else {
            freechk();
        }
    }

    /** FREECHK: no attitude reference - stop auto steering, free control only. */
    private void freechk() {
        s("RCSFLAGS", g("RCSFLAGS") | 0x2000); // rates not initialized (bit14)
        s("HOLDFLAG", 0x2000);
        if (!bit(g("CHAN31"), 14)) {
            kresume1();
        } else {
            reinit();
        }
    }

    /**
     * RATEFILT: rate derivation. For each axis: DRHO = (1-gain1)*DRHO - 0.1*ADOT (quarter-scaled), then body
     * rate changes from CDU deltas via AMGB matrix are added to DRHO and MERROR, then ADOT is updated as
     * ADOT + 0.16*0.1*DRHO*gain2 + KMJ*DFT.
     */
    private void ratefilt() {
        if ((g("RCSFLAGS") & 0x2000) != 0) { // rate filter not initialized: skip rate derivation
            kmatrix();
            return;
        }
        int k = Math.max(0, Math.min(g("ATTKALMN"), GAIN1.length - 1));
        for (int axis = 2; axis >= 0; axis--) {
            String drho = "DRHO" + axis;
            double d = Agc.toSigned(g(drho)) * (1 - GAIN1[k]);
            d += -0.25 * Agc.toSigned(g("ADOT" + axis)); // -(.1)ADOT via quarter
            s(drho, Agc.fromSigned((int) d));
        }
        // measured body rates: DELRHO = AMGB * (CDU - RHO)
        int dx = Agc.toSigned(g("CDUX")) - Agc.toSigned(g("RHO"));
        int dy = Agc.toSigned(g("CDUY")) - Agc.toSigned(g("RHO1"));
        int dz = Agc.toSigned(g("CDUZ")) - Agc.toSigned(g("RHO2"));
        s("RHO", g("CDUX"));
        s("RHO1", g("CDUY"));
        s("RHO2", g("CDUZ"));
        double a1 = fr("AMGB1"), a4 = fr("AMGB4"), a7 = fr("AMGB7");
        double a5 = fr("AMGB5"), a8 = fr("AMGB8");
        double tx = dx + a1 * dy;
        double ty = a4 * dy + a5 * dz;
        double tz = a7 * dy + a8 * dz;
        double[] delta = {tx, ty, tz};
        for (int axis = 2; axis >= 0; axis--) {
            String drho = "DRHO" + axis;
            s(drho, Agc.fromSigned((int) (Agc.toSigned(g(drho)) + delta[axis])));
            s("MERROR" + axis, Agc.fromSigned((int) (Agc.toSigned(g("MERROR" + axis)) + delta[axis])));
            double adot = Agc.toSigned(g("ADOT" + axis))
                + 4 * Agc.toSigned(g(drho)) * GAIN2[Math.max(0, Math.min(k, GAIN2.length - 1))]
                + fr("KMJ" + axis) * fr("DFT" + axis);
            s("ADOT" + axis, Agc.fromSigned((int) adot));
        }
        kmatrix();
    }

    private double fr(String sym) {
        return Agc.toSigned(g(sym)) / 16384.0;
    }

    /** KMATRIX: once per second (ATTSEC counter) schedule AMBGUPDT; then hold/auto steering via DCDUINCR. */
    private void kmatrix() {
        int attsec = g("ATTSEC") & 017;
        if (attsec == 0) {
            // CAF PRIO34; NOVAC AMBGUPDT (update transformation gimbal -> body)
            attsec = 9;
        } else {
            attsec = attsec - 1;
        }
        s("ATTSEC", attsec);
        if (sg("HOLDFLAG") > 0) {
            dcduincr();
        } else {
            dacndls();
        }
    }

    /** DCDUINCR: THETAD = CDUD + DELCDU per axis (automatic steering, HOLDFLAG minus). */
    private void dcduincr() {
        String[] ax = {"X", "Y", "Z"};
        for (int i = 2; i >= 0; i--) {
            int sum = Agc.add(g("CDU" + ax[i] + "D"), g("DELCDU" + ax[i]));
            s("THETAD" + ax[i], sum);
            s("CDU" + ax[i] + "D", sum);
        }
        dacndls();
    }

    /** DACNDLS: alternate between FDAIDSP1 and FDAIDSP2 using RCSFLAGS bit4. */
    private void dacndls() {
        if (bit(g("RCSFLAGS"), 4)) {
            fdaidsp2();
        } else {
            fdaidsp1();
        }
    }

    private void fdaidsp1() {
        s("RCSFLAGS", g("RCSFLAGS") | 0x8);
        needler();
        kresume1();
    }

    private void kresume1() {
        // RESUME: end phase 1
    }

    /** NEEDLER: drive FDAI needles via IMU error counters unless in coarse align. */
    private void needler() {
        if (bit(g("CHAN12"), 4)) { // coarse align enabled: do not use the IMU error counters
            s("RCSFLAGS", g("RCSFLAGS") | 0x4);
            return;
        }
        int f = g("RCSFLAGS") & 06;
        if (f == 0) { // NEEDLES3 neither bit: check counter enabled
            if (!bit(g("CHAN12"), 6)) {
                needler(); // recycle
                return;
            }
            needles();
            return;
        }
        if ((f & 04) != 0) { // first pass (BIT3 = 1)
            s("CHAN12", g("CHAN12") & ~(1 << 5)); // disable IMU error counter to zero DACs
            // NEEDLE11: zero inputs, display registers and out counters
            for (String n : new String[] {"AK", "AK1", "AK2", "EDRIVEX", "EDRIVEY", "EDRIVEZ",
                "CDUXCMD", "CDUYCMD", "CDUZCMD"}) {
                s(n, Agc.WORD_MASK);
            }
            s("RCSFLAGS", (g("RCSFLAGS") & ~06) | 02); // pass 2
        } else { // NEEDLER2: enable IMU error counters, wait >= 4 ms for relay closure
            s("CHAN12", g("CHAN12") | (1 << 5));
            s("RCSFLAGS", g("RCSFLAGS") & ~06);
        }
    }

    /** NEEDLES: output (AK/4 - already-displayed) to each DAC, clamped to +/-DACLIMIT (384). */
    private void needles() {
        String[] ak = {"AK", "AK1", "AK2"};
        String[] ed = {"EDRIVEX", "EDRIVEY", "EDRIVEZ"};
        String[] cmd = {"CDUXCMD", "CDUYCMD", "CDUZCMD"};
        for (int i = 2; i >= 0; i--) {
            int v = -sg(ak[i]) / 4;
            v = Math.max(-DACLIMIT, Math.min(DACLIMIT, v));
            int delta = v - sg(ed[i]);
            s(cmd[i], Agc.add(g(cmd[i]), Agc.fromSigned(delta)));
            s(ed[i], Agc.fromSigned(v));
        }
        s("CHAN14", g("CHAN14") | 0x7000); // set DAC activity bits 13,14,15
    }

    /** REINIT: reinitialise DAP in 200 ms (.....TILT). */
    private void reinit() {
        s("TIME5", DELAY200);
        s("T5PHASE", DELAY200);
        // RESUME
    }

    /** FRESHDAP: reset HOLDFLAG to stop automatic steering, then fall into REDAP. */
    private void freshdap() {
        s("HOLDFLAG", 1);
        redap();
    }

    /** REDAP: decode DAP data (S41.2, S40.14), zero T5 variables, init filter, schedule AMBGUPDT. */
    private void redap() {
        // IBNKCALL S41.2 (deadbands, rates, quad management); IBNKCALL S40.14 (inertia -> gains)
        for (int i = NO_T5VAR; i >= 0; i--) {
            s("WBODY+" + i, 0);
        }
        zerojet();
        s("CHANTEMP", Agc.WORD_MASK);
        s("CH31TEMP", Agc.WORD_MASK);
        s("SLOPE", Agc.fromSigned((int) (SLOPE_INIT * 16384)));
        s("T5TIME", 4);
        s("ATTKALMN", 11);
        s("RHO", g("CDUX"));
        s("RHO1", g("CDUY"));
        s("RHO2", g("CDUZ"));
        s("T5PHASE", 0);
        if (!bit(g("IMODES33"), 6)) { // IMU in fine align
            // NOVAC AMBGUPDT
            rcsswit(RCSINIT);
        } else {
            s("ATTKALMN", 0);
            rcsswit(RCSINITB); // free control only, no rate filter start
        }
    }

    /** RCSSWIT: set RCSFLAGS and schedule next T5RUPT in 60 ms. */
    private void rcsswit(int flags) {
        s("RCSFLAGS", flags);
        s("TIME5", T5WAIT60);
        // RESUME
    }

    /** ZEROJET: zero the jet-select words and schedule T6RUPT in 14 ms to shut off jets. */
    public void zerojet() {
        for (int i = 11; i >= 0; i--) {
            s("RWORD1+" + i, 0);
        }
        s("BLAST1+1", 4);
        s("BLAST2+1", 11);
        s("RCSFLAGS", g("RCSFLAGS") & ~1);
        s("T6LOC", g("T6START"));
        s("TIME6", PLUS_14MS);
        s("CHAN13", g("CHAN13") | 0x4000);
    }

    /** T5PHASE2: hand-controller examination, forced firing bits, manual rate commands. */
    public void t5phase2() {
        int ak = sg("ATTKALMN");
        if (ak > 0) {
            kalupdt(ak);
            return;
        }
        int time5 = g("TIME5");
        s("TIME5", DELTATT2); // phase 3 (jet selection) in 20 ms
        s("T5TIME", g("T5TIME") + time5);
        s("ATTKALMN", (g("RCSFLAGS") & 0x4000) != 0 ? Agc.fromSigned(-1) : 0); // high-rate auto maneuver
        s("RCSFLAGS", g("RCSFLAGS") & ~OCT01760); // reset forced-firing bits
        int ch31 = g("CHAN31");
        int changed = (g("CH31TEMP") ^ ch31) & MANROT;
        if (changed != 0) {
            s("CH31TEMP", ch31);
            s("RCSFLAGS", g("RCSFLAGS") + ((ch31 & 077) << 4)); // bits 6-1 -> 10-5
            s("RCSFLAGS", g("RCSFLAGS") | OCT16000 & ~g("RCSFLAGS") | g("RCSFLAGS")); // rate-damping flags
        }
        int man = ~g("CH31TEMP") & MANROT;
        if (man == 0) {
            ahfnorot();
            return;
        }
        s("HOLDFLAG", man);
        stickchk(man);
        s("FLAGWRD1", g("FLAGWRD1") | 0x2000); // STIKFLAG
        if (!bit(g("CHAN31"), 14)) { // free function
            freefunc();
            return;
        }
        if ((g("RCSFLAGS") & 0x2000) != 0) { // rate filter not initialized
            reinit();
            return;
        }
        if (sg("RATEINDX") - 5 > 0) {
            s("ATTKALMN", Agc.fromSigned(-1)); // high-rate manual maneuver
        }
        for (int sp = 2; sp >= 0; sp--) {
            setwbody(sp);
        }
        jets();
    }

    /** SETWBODY for one axis: RMANNDX 0 none, 1 plus, 2 minus; sets WBODY and MERROR. */
    private void setwbody(int sp) {
        String[] ndx = {"RMANNDX", "PMANNDX", "YMANNDX"};
        String[] ax = {"X", "Y", "Z"};
        int dir = g(ndx[sp]);
        boolean damped = (g("RCSFLAGS") & OCT16000) == 0;
        if (dir == 0) { // NORATE
            s("WBODY" + ax[sp], 0);
            if (!damped) {
                s("MERROR" + ax[sp], 0); // ZEROER
            }
        } else {
            int q = dir + sg("RATEINDX");
            double w = MANTABLE[q - 1] / 64.0; // times 2^-6
            s("WBODY" + ax[sp], Agc.fromSigned((int) (w * 16384)));
            if (damped) { // MERUPDAT: MERROR += measured - commanded (WBODY * .1 sec)
                s("MERROR" + ax[sp], Agc.add(g("MERROR" + ax[sp]),
                    Agc.fromSigned((int) (-MANTABLE[q - 1] / 256.0 * 16384))));
            } else {
                s("MERROR" + ax[sp], 0);
            }
        }
        s("ERROR" + ax[sp], g("MERROR" + ax[sp])); // SPNDXCHK: high order word
    }

    /** STICKCHK: decode RHC axes into PMANNDX / YMANNDX / RMANNDX (2 bits each). */
    private void stickchk(int word) {
        s("PMANNDX", word & 3);
        s("YMANNDX", (word >> 2) & 3);
        s("RMANNDX", (word >> 4) & 3);
    }

    /** FREEFUNC: acceleration commands TAU, TAU1, TAU2 from FREETAU, then T6PROGM. */
    private void freefunc() {
        s("TAU", Agc.fromSigned(FREETAU[g("RMANNDX") & 3]));
        s("TAU1", Agc.fromSigned(FREETAU[g("PMANNDX") & 3]));
        s("TAU2", Agc.fromSigned(FREETAU[g("YMANNDX") & 3]));
        t6progm();
    }

    /** T6PROGM: for manual rotations zero ERRORX/Y/Z, then T6PROG. */
    private void t6progm() {
        s("ERRORX", 0);
        s("ERRORY", 0);
        s("ERRORZ", 0);
        t6prog();
    }

    /** AHFNOROT: no manual commands - continue with automatic steering (defined in the jet-selection file). */
    private void ahfnorot() {
        // TCF AHFNOROT (RCS-CSM jet logic in a neighbouring file)
    }

    private void jets() {
        // TCF JETS (jet selection logic, neighbouring file)
    }

    private void t6prog() {
        // TCF T6PROG (jet timing logic, neighbouring file)
    }

    /** KALUPDT: initialisation of attitude rates using Kalman filter; schedules phase 1 in 1 sec - 80 ms. */
    private void kalupdt(int ak) {
        s("ATTKALMN", ak);
        s("TIME5", DELTATT + g("T5TIME"));
        s("T5PHASE", Agc.WORD_MASK); // KRESUME2: reset for phase 1 (-0)
        // RESUME
    }

    /** FDAIDSP2: display autopilot following error or total attitude error on the FDAI needles. */
    public void fdaidsp2() {
        s("RCSFLAGS", g("RCSFLAGS") & ~0x8);
        if (!bit(g("FLAGWRD0"), 9)) { // OFF: display autopilot following error
            s("AK", Agc.fromSigned(-sg("ERRORX")));
            s("AK1", Agc.fromSigned(-sg("ERRORY")));
            s("AK2", Agc.fromSigned(-sg("ERRORZ")));
            return;
        }
        fdaitotl();
    }

    /** FDAITOTL: total attitude error w.r.t. N22 (CTHETA/CPHI) or N17 (CPHIX). */
    private void fdaitotl() {
        int wtemp0, wtemp1, cphi;
        if (!bit(g("FLAGWRD9"), 6)) { // WRTN17
            wtemp0 = g("CPHIX+1");
            wtemp1 = g("CPHIX+2");
            cphi = g("CPHIX");
        } else { // WRTN22
            wtemp0 = g("CTHETA");
            wtemp1 = g("CTHETA+1");
            cphi = g("CPHI");
        }
        // GETAKS
        double ex = Agc.toSigned(cphi) - Agc.toSigned(g("CDUX"));
        double ey = Agc.toSigned(wtemp0) - Agc.toSigned(g("CDUY"));
        double ez = Agc.toSigned(wtemp1) - Agc.toSigned(g("CDUZ"));
        s("AK", Agc.fromSigned((int) (ex + fr("AMGB1") * ey)));
        s("AK1", Agc.fromSigned((int) (fr("AMGB4") * ey + fr("AMGB5") * ez)));
        s("AK2", Agc.fromSigned((int) (fr("AMGB7") * ey + fr("AMGB8") * ez)));
        // RESUME: end phase 1 of RCS DAP
    }
}
