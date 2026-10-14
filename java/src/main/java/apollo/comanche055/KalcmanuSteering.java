package apollo.comanche055;

import apollo.agc.Agc;

/**
 * KALCMANU_STEERING.agc (Comanche 055).
 * Generation of steering commands for digital autopilot free-fall maneuvers.
 * New commands are generated every one second during the maneuver (job NEWDELHI),
 * and applied every tenth of a second by the DAP via DELCDUX/Y/Z increments.
 */
public final class KalcmanuSteering {
    public static final int PRIO26 = 026;   // priority of NEWDELHI job
    public static final int ONE_SEC = 100;  // 1SEC = 100 centiseconds
    public static final int BIT15 = 0x4000;

    private final Agc agc;
    private final Executive executive;
    private final Interpreter interp;

    public KalcmanuSteering(Agc agc) {
        this.agc = agc;
        this.executive = new Executive(agc);
        this.interp = new Interpreter(agc);
    }

    /** NEWDELHI: job run each second to update commanded CDU angles. */
    public void newdelhi() {
        // CS HOLDFLAG; BZMF NOGO-2: terminate if maneuver interrupted by astronaut
        if (Agc.toSigned(~agc.get("HOLDFLAG") & Agc.WORD_MASK) <= 0) {
            Anglfind.of(agc).nogom2(); // NOGO -2
            return;
        }
        newangl();
    }

    /** NEWANGL: compute the new S/C-to-stable-member matrix and the new desired CDU angles. */
    public void newangl() {
        // AXC,1 MIS ; AXC,2 DEL ; CALL MXM3
        interp.axc(1, "MIS");
        interp.axc(2, "DEL");
        interp.call(() -> { Anglfind af = Anglfind.of(agc); af.mis = Anglfind.mxm3(af.mis, af.del); });
        // VLOAD STADR ; STOVL MIS+12D ; STADR ; STOVL MIS+6D ; STADR ; STORE MIS
        interp.vload(null);
        interp.stadr();
        interp.stovl("MIS+12D", null);
        interp.stadr();
        interp.stovl("MIS+6D", null);
        interp.stadr();
        interp.store("MIS");
        // AXC,1 MIS ; CALL DCMTOCDU : new CDU angles from the matrix
        interp.axc(1, "MIS");
        interp.call(() -> { Anglfind af = Anglfind.of(agc); af.cphi = af.dcmtocdu(af.mis); });
        // RTB V1STO2S ; STORE NCDU
        interp.rtb(() -> new Rt8OpCodes(agc).v1sto2s());
        interp.store("NCDU");
        // BONCLR CALCMAN2 / MANUSTAT: start maneuver on first pass (flag test, then EXIT)
        if (interp.bonclr("CALCMAN2")) {
            manustat();
        }
        agc.set("A", 2);
        incrdcdu(2);
    }

    /**
     * INCRDCDU: loop over three axes; compute the CDU angle increment
     * (NCDU - BCDU) * QUADROT and store as DELCDU{X,Y,Z}; advance BCDU and CDU{X,Y,Z}D.
     */
    public void incrdcdu(int axis) {
        for (int k = axis; k >= 0; k--) {
            agc.set("KSPNDX", k);
            int kd = 2 * k;
            agc.set("KDPNDX", kd);
            int ncdu = agc.get(idx("NCDU", k));
            int diff = Agc.add(ncdu, ~agc.get(idx("BCDU", k)) & Agc.WORD_MASK); // MSU
            // EXTEND MP QUADROT : double-precision product, high word to A, low to L
            long prod = (long) Agc.toSigned(diff) * Agc.toSigned(agc.get("QUADROT"));
            int hi = (int) (prod >> 14);
            int lo = (int) (prod & 0x3FFF);
            String[] delcdu = {"DELCDUX", "DELCDUY", "DELCDUZ"};
            agc.set(delcdu[k], Agc.fromSigned(hi));
            agc.set(delcdu[k] + "+1", Agc.fromSigned(lo));
            agc.set(idx("BCDU", k), ncdu);
            String[] cdud = {"CDUXD", "CDUYD", "CDUZD"};
            agc.set(cdud[k], ncdu);
        }
        agc.set("INHINT", 0); // RELINT
        tmanuchk();
    }

    private static String idx(String base, int k) {
        return k == 0 ? base : base + "+" + k;
    }

    /** TMANUCHK: compare present time with the time to terminate the maneuver. */
    public void tmanuchk() {
        int r = timechk();
        if (r == 0) {
            contmanu(); // POSTJUMP CONTMANU : maneuver continues
            return;
        }
        // maneuver ends in less than 1 second: schedule MANUSTOP in 1 centisecond
        new Waitlist(agc).waitlist(1, () -> manustop());
        agc.set("INHINT", 0);
        executive.endofjob();
    }

    /**
     * TIMECHK: returns 0 if (TM - TIME2) &gt; 1 second (continue, first return),
     * 1 if the maneuver should be stopped within a second (later return).
     */
    public int timechk() {
        long time2 = ((long) Agc.toSigned(agc.get("TIME2")) << 14) | (agc.get("TIME2+1") & 0x3FFF);
        long tm = ((long) Agc.toSigned(agc.get("TM")) << 14) | (agc.get("TM+1") & 0x3FFF);
        long remain = tm - time2;
        if (remain > ONE_SEC && remain >= 0) {
            return 0;
        }
        return 1;
    }

    /** MANUSTAT: initialisation routine for automatic maneuvers. */
    public void manustat() {
        // TM = TIME2 + TM ; then TM -= 1 sec (so TM = (TM+T0) - 1)
        int tmLo = agc.get("TM+1") + agc.get("TIME2+1");
        agc.set("TM+1", tmLo);
        agc.set("TM", Agc.add(agc.get("TM"), agc.get("TIME2")));
        agc.set("TM+1", Agc.add(agc.get("TM+1"), Agc.fromSigned(-ONE_SEC)));
        agc.set("INHINT", 1);
        agc.set("HOLDFLAG", Agc.fromSigned(-1)); // enable autopilot to perform automatic maneuvers
        // CS RATEINDX; AD SIX; BZMF HIGHGAIN : high rate if RATEINDX >= 6
        int t = Agc.add(~agc.get("RATEINDX") & Agc.WORD_MASK, 6);
        if (Agc.toSigned(t) <= 0) {
            highgain();
        }
        agc.set("BIAS", agc.get("BIASTEMP+1"));
        agc.set("BIAS1", agc.get("BIASTEMP+3"));
        agc.set("BIAS2", agc.get("BIASTEMP+5"));
        agc.set("NEXTIME", Agc.add(agc.get("TIME1"), ONE_SEC));
        // POSTJUMP CADR INCRDCDU-1 : enter loop with A = 2
        incrdcdu(2);
    }

    /** HIGHGAIN: set high-rate flag (bit 15 of RCSFLAGS) and load maneuver rates. */
    public void highgain() {
        int rcs = agc.get("RCSFLAGS");
        agc.set("RCSFLAGS", Agc.add(rcs, ~rcs & BIT15 & Agc.WORD_MASK));
        agc.set("WBODY", agc.get("BRATE"));
        agc.set("WBODY+1", agc.get("BRATE+1"));
        agc.set("WBODY1", agc.get("BRATE+2"));
        agc.set("WBODY1+1", agc.get("BRATE+3"));
        agc.set("WBODY2", agc.get("BRATE+4"));
        agc.set("WBODY2+1", agc.get("BRATE+5"));
    }

    /** CONTMANU: continue with the update process; schedule UPDTCALL via waitlist. */
    public void contmanu() {
        agc.set("INHINT", 1);
        int dt = Agc.add(~agc.get("TIME1") & Agc.WORD_MASK, agc.get("NEXTIME"));
        int delay = Agc.toSigned(dt);
        if (delay <= 0) {
            delay = delay + 037777 + 1; // MANUCALL wrap-around via NEGMAX
        } else {
            delay = delay + 1;
        }
        new Waitlist(agc).waitlist(delay, () -> updtcall());
        agc.set("INHINT", 0);
        agc.set("NEXTIME", Agc.add(agc.get("NEXTIME"), ONE_SEC));
        executive.endofjob();
    }

    /** UPDTCALL: waitlist task; calls for an update of steering commands (FINDVAC NEWDELHI). */
    public void updtcall() {
        executive.findvac(PRIO26, this::newdelhi); // 2CADR NEWDELHI
        new Waitlist(agc).taskover();
    }

    /** MANUSTOP: routine for terminating automatic maneuvers. */
    public void manustop() {
        stopyz();
        loadyz();
        endroll();
    }

    /** ENDROLL. */
    public void endroll() {
        agc.set("CDUXD", agc.get("CPHI")); // commanded outer gimbal
        stoprate();
        endmanu();
    }

    /** ENDMANU: restore user priority, clear ATTCADR and return to the caller of GOMANUR. */
    public void endmanu() {
        agc.set("NEWPRIO", agc.get("ATTPRIO"));
        agc.set("ATTCADR", 0);
        agc.set("ATTCADR+1", 0);
        executive.spvac(agc.get("ATTPRIO"), agc.get("ATTCADR"), agc.get("ATTCADR+1"));
        new Waitlist(agc).taskover();
    }

    /** STOPRATE: zero roll incremental angles, rate and bias; reset high-rate flag. */
    public void stoprate() {
        agc.set("DELCDUX", 0);
        agc.set("DELCDUX+1", 0);
        agc.set("WBODY", 0);
        agc.set("WBODY+1", 0);
        agc.set("BIAS", 0);
        agc.set("RCSFLAGS", agc.get("RCSFLAGS") & ~BIT15 & Agc.WORD_MASK);
        stopyz();
    }

    /** STOPYZ: zero pitch and yaw increments, rates and biases. */
    public void stopyz() {
        for (String s : new String[] {"DELCDUY", "DELCDUY+1", "DELCDUZ", "DELCDUZ+1",
                "WBODY1", "WBODY1+1", "WBODY2", "WBODY2+1", "BIAS1", "BIAS2"}) {
            agc.set(s, 0);
        }
    }

    /** ZEROERROR: set CDU desired equal to present CDU angles. */
    public void zeroerror() {
        agc.set("CDUXD", agc.get("CDUX"));
        agc.set("CDUYD", agc.get("CDUY"));
        agc.set("CDUZD", agc.get("CDUZ"));
    }

    /** LOADCDUD: store terminal angles into command angles. */
    public void loadcdud() {
        agc.set("CDUXD", agc.get("CPHI"));
        loadyz();
    }

    /** LOADYZ. */
    public void loadyz() {
        agc.set("CDUYD", agc.get("CTHETA"));
        agc.set("CDUZD", agc.get("CPSI"));
    }
}
