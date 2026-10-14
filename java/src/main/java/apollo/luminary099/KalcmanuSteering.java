package apollo.luminary099;

import apollo.agc.Agc;

/**
 * KALCMANU_STEERING.agc - steering of the attitude maneuver (KALCMANU): every second
 * (waitlist UPDTCALL -> NEWDELHI) the desired CDU angles are recomputed from the rotated
 * matrix and the CDU error increments (DELDCDU), rate commands (OMEGAPD/QD/RD) and
 * bias-corrected rate errors are updated.
 */
public class KalcmanuSteering {
    /** DT/TAU: DEC .1 (fraction of the increment applied per 0.1 sec step). */
    public static final double DT_TAU = 0.1;
    /** ONESEK: DEC 0 / DEC 100 : one second in centiseconds. */
    public static final int ONESEK_HI = 0;
    public static final int ONESEK_LO = 100;
    /** BIASCALE: OCT 75777. */
    public static final int BIASCALE = 075777;
    public static final int PRIO26 = 026;

    private final Agc agc;

    public KalcmanuSteering(Agc agc) {
        this.agc = agc;
    }

    /** NEWDELHI: job entered every second to compute new target CDU angles. */
    public void newDelhi() {
        // BANKCALL ISITAUTO ; if A != 0 (not in auto mode) -> NOGO-2
        if (agc.get("A") != 0) {
            new AttitudeManeuverRoutine(agc).noGo();
            return;
        }
        newAngl();
    }

    /** NEWANGL: MIS = MIS(matrix from KEL x MIS), convert to CDU angles NCDU. */
    public void newAngl() {
        // interpretive: AXC,1 MIS ; AXC,2 KEL ; CALL MXM3 -> new matrix in MIS (3 rows stored at MIS, MIS+6D, MIS+12D)
        // AXC,1 MIS ; CALL DCMTOCDU ; RTB V1STO2S ; STORE NCDU
        // BONCLR CALCMAN2 MANUSTAT: first pass -> go to MANUSTAT
        if (agc.get("CALCMAN2_FLAG") == 0) {
            manuStat();
            return;
        }
        agc.set("SPNDX", 2);
        incrDcdu();
    }

    /**
     * INCRDCDU: for each axis (SPNDX=2,1,0) compute DELDCDU = (BCDU - NCDU) * DT/TAU,
     * then BCDU = NCDU and CDU?D = NCDU (the new desired CDU angle).
     */
    public void incrDcdu() {
        for (int i = agc.get("SPNDX"); i >= 0; i--) {
            int bcdu = agc.get("BCDU+" + i);
            int ncdu = agc.get("NCDU+" + i);
            // CA BCDU ; MSU NCDU  : difference (one's complement modular)
            int diff = Agc.add(bcdu, (~ncdu) & Agc.WORD_MASK);
            int dSigned = Agc.toSigned(diff);
            // MP DT/TAU, take the high part; CCS A / AD ONE / COM rounding toward zero
            int prod = (int) (dSigned * DT_TAU);
            agc.set("DELDCDU+" + i, Agc.fromSigned(prod));
            agc.set("BCDU+" + i, ncdu);       // XCH BCDU
            agc.set("CDUXD+" + i, ncdu);      // TS CDUXD (CDUXD, CDUYD, CDUZD)
            agc.set("SPNDX", i);
        }
        tManuChk();
    }

    /** TMANUCHK: if time is up (TIMECHK skip) continue maneuver; else stop. */
    public void tManuChk() {
        if (timeCheck()) {
            contManu();
        } else {
            manuStal();
        }
    }

    /** MANUSTAL: schedule MANUSTOP on the waitlist (1 sec) and end job. */
    public void manuStal() {
        // INHINT ; WAITLIST MANUSTOP in A=ONE ticks
        agc.set("WAITLIST_MANUSTOP", 1);
    }

    /**
     * TIMECHK: compare elapsed time (TIME2 - TM) with the maneuver duration.
     * Returns true when the maneuver is not yet complete (second return of the skip-return).
     */
    public boolean timeCheck() {
        long time2 = ((long) agc.get("TIME2") << 14) | agc.get("TIME2+1");
        long tm = ((long) agc.get("TM") << 14) | agc.get("TM+1");
        long ttemp = tm - time2;
        agc.set("TTEMP", (int) (ttemp >> 14));
        agc.set("TTEMP+1", (int) (ttemp & 0x3FFF));
        if (ttemp > 0) {
            return false;      // TC Q (first return)
        }
        // elapsed: in last second? ONESEK+1 + low word
        int low = (int) (ttemp & 0x3FFF);
        int manuoff = low + ONESEK_LO;
        return manuoff > 0;
    }

    /** MANUSTAT: initialise the maneuver: TM = TIME2 - 1 second, set rate biases. */
    public void manuStat() {
        long time2 = ((long) agc.get("TIME2") << 14) | agc.get("TIME2+1");
        long tm = time2 - ONESEK_LO;
        agc.set("TM", (int) (tm >> 14));
        agc.set("TM+1", (int) (tm & 0x3FFF));
        rateBias();
    }

    /**
     * RATEBIAS: for each axis compute OMEGAxD = BRATE and DELxEROR = rate bias error
     * = (|BRATE| * BIASCALE * BRATE) / 1JACC.
     */
    public void rateBias() {
        for (int k = 2; k >= 0; k--) {
            int brate = Agc.toSigned(agc.get("BRATE+" + k));
            agc.set("OMEGAPD+" + k, agc.get("BRATE+" + k));
            int mag = Math.abs(brate);
            long p = ((long) mag * Agc.toSigned(BIASCALE)) >> 14;
            long q = (p * brate) >> 14;
            int jacc = Agc.toSigned(agc.get("1JACC"));
            int err = jacc == 0 ? 0 : (int) (q / jacc);
            agc.set("DELPEROR+" + k, Agc.fromSigned(err));
            agc.set("KSPNDX", k);
        }
        agc.set("NEXTIME", Agc.add(agc.get("TIME1"), ONESEK_LO));
        agc.set("SPNDX", 2);
        incrDcdu();
    }

    /** CONTMANU: if NEXTIME is not yet reached, wait; schedule UPDTCALL after 1 second. */
    public void contManu() {
        int diff = Agc.add((~agc.get("TIME1")) & Agc.WORD_MASK, agc.get("NEXTIME"));
        if (Agc.toSigned(diff) <= 0) {
            diff = Agc.add(diff, 040000 /* NEGMAX-ish wrap */);
        }
        manuCall();
    }

    /** MANUCALL: waitlist UPDTCALL, advance NEXTIME by one second. */
    public void manuCall() {
        // INHINT ; WAITLIST UPDTCALL
        agc.set("NEXTIME", Agc.add(agc.get("NEXTIME"), ONESEK_LO));
    }

    /** UPDTCALL: waitlist task that schedules job NEWDELHI at priority 26. */
    public void updtCall() {
        // FINDVAC 2CADR NEWDELHI at PRIO26 ; TASKOVER
        agc.set("FINDVAC_PRIO", PRIO26);
    }

    /** MANUSTOP: zero all rate and error terms and fall into ENDROLL. */
    public void manuStop() {
        for (String s : new String[] {"DELDCDU2", "OMEGARD", "DELREROR", "DELDCDU1",
                "OMEGAQD", "DELQEROR"}) {
            agc.set(s, 0);
        }
        agc.set("CDUZD", agc.get("CPSI"));
        agc.set("CDUYD", agc.get("CTHETA"));
        endRoll();
    }

    /** ENDROLL: final roll-angle command and zero of roll-axis rate and error terms. */
    public void endRoll() {
        agc.set("CDUXD", agc.get("CPHI"));
        agc.set("OMEGAPD", 0);
        agc.set("DELDCDU", 0);
        agc.set("DELPEROR", 0);
        goodManu();
    }

    /** GOODMANU: restore the attitude-priority job (ATTCADR) via SPVAC and finish the task. */
    public void goodManu() {
        agc.set("NEWPRIO", agc.get("ATTPRIO"));
        agc.set("ATTCADR", 0);
        // TC SPVAC ; TC TASKOVER
    }
}
