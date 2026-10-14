package apollo.comanche055;

import apollo.agc.Agc;

/**
 * TVCINITIALIZE (Comanche055, pp 937-944): TVC DAP initialization and startup (TVCDAPON),
 * gain computation (S40.15), CSM/LM switch-over (SWICHOVR) and filter coefficient loading.
 * Called by DOTVCON in P40 via T5; exits with RESUME.
 */
public final class TvcInitialize {
    // Filter coefficient tables (N10... sets)
    /** CSM only filter coefficients: N10,N11/2,N12,D11/2,D12,N20,N21/2,N22,D22/2,D22. */
    public static final double[] CSMN10 = {
        .99999, -.2549, .0588, -.7620, .7450, .99999, -.4852, 0, -.2692, 0};
    /** Low bandwidth filter coefficients (CSM/LM). */
    public static final double[] LBN10 = {
        .99999, -.3285, -.3301, -.9101, .8460, .03125, 0, 0, -.9101, .8460,
        .5000, -.47115, .4749, -.9558, .9372};

    public static final int NZERO = 51;       // must be odd for MRCLEAN
    public static final int NZEROJR = 23;     // must be odd for MCLEANJR
    public static final double ATTLIM = 0.00833;      // initial attitude error limit (1.5 deg)
    public static final double INV_ATTLIM = 0.007325; // 1/ATTLIM label: 0 if error < 1.5 deg
    public static final int[] TCORR = {5, 0}; // CSM; CSM/LM
    public static final double FKPRIMDT = .0102;      // CSM/LM (LB), scaled at PI/8
    public static final double FREPFRAC = .0375;      // B-2
    public static final int NINETEEN = 19;
    public static final double TWO_PI_OVER_M = .00331017; // B+8
    public static final double ONETHOU = 1000;        // B-13

    private final Agc agc;

    public TvcInitialize(Agc agc) {
        this.agc = agc;
    }

    /** TVCDAPON/MRCLEAN: zero TVC DAP erasables from OMEGAYC on, set up T5 to run TVCINIT1. */
    public void tvcdapon() {
        mrclean();
    }

    public void mrclean() {
        // zero NZERO+1 words starting at OMEGAYC by double-word pairs
        for (int cntr = NZERO; cntr > 0; cntr -= 2) {
            agc.set("OMEGAYC+" + cntr, 0);
            agc.set("OMEGAYC+" + (cntr - 1), 0);
        }
        // T5LOC = INITLOC2 (2CADR TVCINIT1); TIME5 = POSMAX
        agc.set("T5LOC", 1);                 // refers to tvcinit1()
        agc.set("TIME5", 037777);
        // ENDMRC: TCF RESUME
    }

    /** TVCINIT1: T5 entry. Update masses/gains via MASSPROP, set filter coefficients. */
    public void tvcinit1() {
        // IBNKCALL MASSPROP: updates IXX, IAVG/TLX
        int tenmdot = (int) (agc.get("EMDOT") * 1000 / 8192.0);  // 10-sec mass loss B+16 KG
        agc.set("TENMDOT", tenmdot);
        agc.set("MASSTMP", agc.get("CSMMASS") - tenmdot);        // decrement for first 10 sec of burn
        int cntr = (agc.get("DAPDATR1") & 020000) != 0 ? 1 : 0;  // BIT14: LEM-ON(1)/OFF(0)
        agc.set("CNTR", cntr);
        agc.set("COEFFADR", 0);                                  // CSMCFADR (CSM only) / HBCFADR (CSM/LM)
        loadcoef(cntr == 0 ? CSMN10 : null);
        agc.set("KTLX/I", agc.get(cntr == 0 ? "EKTLX/I" : "EKTLX/I+1"));
        s4015();
        tvcinit2();
    }

    /** TVCINIT2: filter period, KPRIMEDT, T5TVCDT, REPFRAC, correction time. */
    public void tvcinit2() {
        int cntr = agc.get("CNTR");
        int kprimedt = (cntr == 0) ? 2 : 4;          // BIT2 (CSM 40 ms filter) / BIT3 for CSM/LM 80 ms
        agc.set("KPRIMEDT", cntr == 0 ? 2 : 4);
        agc.set("T5TVCDT", (037777 - kprimedt + 1) & Agc.WORD_MASK);
        agc.set("FLAGWRD9", agc.get("FLAGWRD9") & ~040000);      // reset SWTOVER flag (bit 15)
        long k = (long) agc.get(cntr == 0 ? "EKPRIME" : "EKPRIME+1") * agc.get("KPRIMEDT");
        agc.set("KPRIMEDT", (int) (k >> 14));
        agc.set("KPRIMEDT+1", (int) (k & 0x3FFF));
        agc.set("REPFRAC", agc.get(cntr == 0 ? "EREPFRAC" : "EREPFRAC+1"));
        agc.set("CNTR", TCORR[cntr]);                // one-shot correction time
        agc.set("STRKTIME", Agc.fromSigned(-1));     // prevent stroke test until called
        agc.set("VCNTR", NINETEEN);                  // variable-gain updates in 10 s
        agc.set("V97VCNTR", NINETEEN);               // for ENGFAIL (R41)
        tvcinit3();
    }

    /** TVCINIT3: trim values to trim trackers, output trackers, offset updates. */
    public void tvcinit3() {
        agc.set("PDELOFF", agc.get("PACTOFF"));
        agc.set("PCMD", agc.get("PACTOFF"));
        agc.set("DELPBAR", agc.get("PACTOFF"));
        agc.set("YDELOFF", agc.get("YACTOFF"));
        agc.set("YCMD", agc.get("YACTOFF"));
        agc.set("DELYBAR", agc.get("YACTOFF"));
        attinit();
    }

    /** ATTINIT: limit initial RCS attitude errors to 1.5 degrees (CSM only). */
    public void attinit() {
        if ((agc.get("DAPDATR1") & 010000) != 0) {       // BIT13 set: CSM/LM => bypass
            needlein();
            return;
        }
        for (int ttmp = 1; ttmp >= 0; ttmp--) {
            int err = Agc.toSigned(agc.get("ERRBTMP+" + ttmp));
            double scaled = INV_ATTLIM * err / 16384.0;
            if ((int) scaled != 0) {                    // |ERROR| > 1.5 deg
                int lim = (int) (ATTLIM * 16384);
                agc.set("ERRBTMP+" + ttmp, Agc.fromSigned(err > 0 ? lim : -lim));
            }
        }
        agc.set("PERRB", agc.get("ERRBTMP"));
        agc.set("YERRB", agc.get("ERRBTMP+1"));
        needlein();
    }

    /** NEEDLEIN: set RCSFLAGS bit 3 (init pass) and call NEEDLER; then TVCINIT4. */
    public void needlein() {
        agc.set("RCSFLAGS", agc.get("RCSFLAGS") | 04);
        // IBNKCALL NEEDLER
        tvcinit4();
    }

    /** TVCINIT4 (possible restart entry): TVCPHASE = 0, prepare roll DAP, call TVCEXEC and DAPINIT. */
    public void tvcinit4() {
        agc.set("TVCPHASE", 0);
        agc.set("OGANOW", agc.get("CDUX"));
        boolean engineOff = (agc.get("DSALMOUT") & 010000) == 0;  // BIT13 of DSALMOUT: engine on
        if (!engineOff) {
            agc.set("CSMMASS", agc.get("MASSTMP"));     // COPYCYCLE
        }
        // CAF .5SEC; WAITLIST 2CADR TVCEXEC
        // T5LOC = DAPINIT5 (2CADR DAPINIT); TIME5 = T5TVCDT
        agc.set("TIME5", agc.get("T5TVCDT"));
        // ENDTVCIN: TCF RESUME
    }

    /** PRESWTCH: entry from V46; SWICHOVR then POSTJUMP to PINBRNCH. */
    public void preswtch() {
        swichovr();
        // TC POSTJUMP / CADR PINBRNCH (does a RELINT)
    }

    /** SWICHOVR: switch from high to low bandwidth gains/coefficients (CSM/LM). */
    public void swichovr() {
        int phasetmp = agc.get("TVCPHASE");
        agc.set("PHASETMP", phasetmp);
        agc.set("TVCPHASE", Agc.fromSigned(-2));        // indicates switch-over to restart logic
        // RESTART ENTRY at SWICHOVR+5 (RTRNLOC saved)
        for (int i = NZEROJR; i >= 0; i--) {            // MCLEANJR: zero filter storage PTMP1-1 ...
            agc.set("PTMP1+" + (i - 1), 0);
        }
        agc.set("FLAGWRD9", agc.get("FLAGWRD9") | 040000);       // set switchover flag for downlink
        agc.set("KTLX/I", agc.get("EKTLX/I+2"));        // low bandwidth gains - DAP
        s4015Entry7();
        agc.set("KPRIMEDT", (int) (FKPRIMDT * 16384));  // steering
        agc.set("REPFRAC", (int) (FREPFRAC * 16384));   // TMC loop
        agc.set("PDELOFF", agc.get("DELPBAR"));         // update trim estimates
        agc.set("PDELOFF+1", agc.get("DELPBAR+1"));
        agc.set("YDELOFF", agc.get("DELYBAR"));
        agc.set("YDELOFF+1", agc.get("DELYBAR+1"));
        agc.set("COEFFADR", 2);                         // LBCFADR
        loadcoef(LBN10);
        agc.set("TVCPHASE", phasetmp);                  // restore TVCPHASE
        // TC RTRNLOC: back to PRESWTCH or TVCRESTARTS
    }

    /** LOADCOEF: load DAP filter coefficients (15 words) from the table at COEFFADR into N10... */
    public void loadcoef(double[] table) {
        if (table == null) {
            // CSM/LM high bandwidth: erasable pad loads HBN10 (already in erasable)
            return;
        }
        for (int i = 0; i < table.length; i++) {
            agc.set("N10+" + i, Agc.fromSigned((int) Math.round(table[i] * 16384)));
        }
    }

    /** S40.15: gain computations 1/CONACC and VARK (from TVCINITIALIZE and TVCEXEC). */
    public void s4015() {
        double conacc = agc.get("IXX") * TWO_PI_OVER_M * 8;      // DDOUBL x3; scaled B+9 sec-sq/rev
        agc.set("1/CONACC", (int) conacc);
        s4015Entry7();
    }

    /** S40.15 +7: entry from CSM/LM V46 switch-over. VARK = KTLX/I * IAVG/TLX * 4. */
    public void s4015Entry7() {
        agc.set("VARK", (int) ((long) agc.get("KTLX/I") * agc.get("IAVG/TLX") * 4 >> 14));
    }
}
