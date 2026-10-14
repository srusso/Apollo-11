package apollo.comanche055;

import apollo.agc.Agc;

/**
 * SERVICER207.agc - PREREAD, READACCS, ONMNITOR/QUIKREAD, SERVICER, AVERAGE G, NORMLIZE, PIPASR,
 * CALCGRAV and CALCRVG.
 *
 * <p>Computes RN, VN and GDT/2 during accelerated flight. A caller makes a WAITLIST call for PREREAD
 * (or BIBIBIAS at lift-off) two seconds before the first average-G update; the sequence then recurs
 * every 2 s as long as AVEGFLAG remains set. AVGEXIT holds the routine run at the end of each cycle
 * (it must return through {@link #servexit()}); on the final pass it is {@link #avgend()}.
 *
 * <p>Scaling: RN 2^+29 m, VN 2^+7 m/cs, GDT/2 2^+7 m/cs, DELV 2^+14 * 5.85e-4 m/cs (KPIP1 -> 2^+7).
 * Flags are kept in erasable symbols of the same name (non-zero = set). Vectors are double[3] in
 * AGC scaled-fraction units (see {@link DpWords}).
 */
public final class Servicer207 {
    // Constants
    /** DVTHRUSH = ELEVEN: 15 percent of 2 s PIPA accumulation. */
    public static final int DVTHRUSH = 11;
    /** -MAXDELV: 3200 PPS for 2 sec. */
    public static final int NEG_MAXDELV = -6398;
    /** JTAGTIME = 1 sec + T CDU (.1 sec). */
    public static final int JTAGTIME = 120;
    public static final int TWO_POINT_FIVE_SEC = 250;
    /** 5 sec mass loss at 28.8 kg/sec: 144.0 B-16. */
    public static final double MDOTFAIL = 144.0 / 65536.0;
    public static final int HALF_SEC = 50;
    public static final int TWO_SECS = 200;
    public static final double KPIP = .1024;         // scales DELV to 2^+4
    public static final double KPIP1 = 0.074880;     // 207 DELV scaling. 1 pulse = 5.85 cm/sec
    public static final double NEG_MUDT_E = -7.9720645E12 / Math.pow(2, 44);
    public static final double NEG_MUDT_M = -9.805556E10 / Math.pow(2, 44);
    public static final double TWO_SEC_22 = 200 / Math.pow(2, 22);
    public static final double DP1_20 = 0.05;
    public static final double RESQ = 40.6809913E12 / Math.pow(2, 59);
    public static final double J20 = 3.24692010E-2 / 2.0;   // 20J B1
    public static final double J2 = 3.24692010E-3 / 2.0;    // 2J  B1

    private static final int PRIO20 = 020000;
    private static final int PRIO21 = 021000;
    private static final int PRIO31 = 031000;
    private static final int PRIO32 = 032000;
    private static final int BIT2 = 2;
    private static final int BIT9 = 0400;
    private static final int BIT12 = 04000;

    private final Agc agc;
    private final Executive executive;
    private final Waitlist waitlist;
    private final ServiceRoutines service;

    /** AVGEXIT: routine to run at the end of each average-G cycle. */
    private Runnable avgexit = this::servexit;

    public Servicer207(Agc agc) {
        this.agc = agc;
        this.executive = new Executive(agc);
        this.waitlist = new Waitlist(agc);
        this.service = new ServiceRoutines(agc);
    }

    /** Sets AVGEXIT (user routines must end by calling {@link #servexit()}). */
    public void setAvgexit(Runnable routine) {
        this.avgexit = routine;
    }

    private boolean flag(String n) {
        return agc.get(n) != 0;
    }

    private void setFlag(String n, boolean on) {
        agc.set(n, on ? 1 : 0);
    }

    // ----------------------------------------------------------------------------------
    // PREREAD
    // ----------------------------------------------------------------------------------

    /** PREREAD: entered directly from TIG-30 via POSTJUMP; caller must protect it. */
    public void preread() {
        executive.novac(PRIO21, 0, 0);             // 2CADR LASTBIAS: last gyro compensation in free fall
        new ImuCompensationPackage(agc).lastbias();
        redo531();
    }

    /** REDO5.31 */
    public void redo531() {
        preread1();
        executive.findvac(PRIO32, 0, 0);          // NORMLIZE job needed before first average G pass
        normlize();
        waitlist.waitlist(TWO_SECS, this::readaccs);
        new PhaseTableMaintenance(agc).newphase(5, -2);   // CS TWO ; NEWPHASE 5
        // TCF TASKOVER
    }

    /** PREREAD1: clear + read PIPs last time in free fall and set flags. */
    public void preread1() {
        pipasr();
        agc.set("PIPAGE", 1);                      // for REREADAC in case restart occurs before READACCS
        setFlag("AVEGFLAG", true);                 // set AVEG flag
        setFlag("DRIFTFLG", false);                // knock down drift flag
        setFlag("V37FLAG", true);                  // set V37 flag
        agc.set("DVTOTAL", 0);                     // clear DVTOTAL
        agc.set("DVTOTAL+1", 0);
    }

    // ----------------------------------------------------------------------------------
    // READACCS
    // ----------------------------------------------------------------------------------

    /** READACCS: waitlist task reading the PIPs every 2 seconds. */
    public void readaccs() {
        pipasr();
        pipsdone();
    }

    /** PIPSDONE: return point of PIPASR. */
    public void pipsdone() {
        // 5.5 phase: group 5 restart point
        new PhaseTableMaintenance(agc).phase5(5);
        agc.set("PIPAGE", 1);                       // show PIPs have been read (REDO5.5)
        agc.set("PIPCTR", 2);                       // set PIPCTR for ONMNITOR
        if (!DpWords.bit(agc, "CM/FLAGS", BIT2)) {  // CM/DSTBY not set -> CHEKAVEG
            chekaveg();
            return;
        }
        agc.set("TBASE6", agc.get("PIPTIME1+1"));   // for restarts
        // continue for entry DAP: save AOG, AMG, ROLL/180, BETA/180 for the DAP
        agc.set("AOG/PIP", agc.get("AOG"));
        agc.set("AOG/PIP+1", agc.get("AOG+1"));
        agc.set("AMG/PIP", agc.get("AMG"));
        agc.set("ROLL/PIP", agc.get("ROLL/180"));
        agc.set("ROLL/PIP+1", agc.get("ROLL/180+1"));
        agc.set("BETA/PIP", agc.get("BETA/180"));
        if (DpWords.bit(agc, "CM/FLAGS", 12)) {    // CM/DAPARM (93D bit 12): RCS DAP inactive in entry
            // save PIPAs each 0.5 sec for telemetry
            waitlist.waitlist(HALF_SEC, this::quikread);
            for (String ax : new String[] {"X", "Y", "Z"}) {
                int delv = agc.get("DELV" + ax);   // PIPAs as read (not compensated)
                agc.set(ax + "OLDBUF", agc.get(ax + "PIPBUF"));
                agc.set(ax + "PIPBUF", delv);
            }
        }
        // NOSAVPIP
        agc.set("CM/GYMDT", 5);
        waitlist.waitlist(JTAGTIME, () -> new CmEntryDigitalAutopilot(agc).setjtag());
        new PhaseTableMaintenance(agc).newphase(1, -3);   // 1.3SPOT for SETJTAG
        chekaveg();
    }

    /** CHEKAVEG: if AVEG flag down set final exit; else recycle READACCS and start SERVICER. */
    private void chekaveg() {
        if (!flag("AVEGFLAG")) {
            avegout();
            return;
        }
        waitlist.waitlist(TWO_SECS, this::readaccs);   // 2CADR READACCS
        makeserv();
    }

    /** MAKESERV: establish the SERVICER job and turn the test connector outbit on. */
    private void makeserv() {
        executive.findvac(PRIO20, 0, 0);               // 2CADR SERVICER
        new PhaseTableMaintenance(agc).newphase(5, -4); // restart SERVICER and READACCS
        agc.set("DSALMOUT", agc.get("DSALMOUT") | BIT9); // test connector outbit ON
        servicer();                                    // job body (TASKOVER ends the task)
    }

    /** AVEGOUT: set AVGEXIT to AVGEND for the final pass. */
    private void avegout() {
        avgexit = this::avgend;                        // AVOUTCAD
        makeserv();
    }

    // ----------------------------------------------------------------------------------
    // ONMNITOR / QUIKREAD
    // ----------------------------------------------------------------------------------

    /** ONMNITOR: three passes through QUIKREAD at 1/2 second intervals, counted by PIPCTR. */
    public void onmnitor(int pipctr) {
        agc.set("PIPCTR", pipctr);
        waitlist.waitlist(HALF_SEC, this::quikread);   // FIXDELAY 0.5SEC
    }

    /** QUIKREAD: save actual PIPAs for telemetry; update X,Y,ZPIPBUF and X,Y,ZOLDBUF. */
    public void quikread() {
        String[] ax = {"X", "Y", "Z"};
        for (int i = 2; i >= 0; i--) {
            int pip = agc.get("PIPA" + ax[i]);
            agc.set(ax[i] + "OLDBUF", agc.get(ax[i] + "PIPBUF"));
            agc.set(ax[i] + "PIPBUF", pip);
        }
        int ctr = Agc.toSigned(agc.get("PIPCTR"));      // CCS PIPCTR
        if (ctr > 0) {
            onmnitor(ctr - 1);
        }
        // else TC TASKOVER
    }

    // ----------------------------------------------------------------------------------
    // SERVICER
    // ----------------------------------------------------------------------------------

    /** SERVICER: PIP runaway check, compensation, DVTOTAL accumulation, then AVERAGE G. */
    public void servicer() {
        String[] ax = {"X", "Y", "Z"};
        for (int i = 2; i >= 0; i--) {                  // PIPCHECK
            int delv = Agc.toSigned(agc.get("DELV" + ax[i]));
            int mag = Math.abs(delv);
            if (mag == 0) {
                continue;                               // PIPLOOP
            }
            if (mag + NEG_MAXDELV > 0) {                // saturated PIPA
                new AlarmAndAbort(agc).alarm(00205);    // SATURATED-PIPA ALARM
                averageg();
                return;
            }
        }
        new PhaseTableMaintenance(agc).phaschng(016035, 020000);   // restart REREADAC + SERVICER
        new ImuCompensationPackage(agc).onePipa();      // PIPA compensation
        dvtotup();
    }

    /** DVTOTUP: DVTOTAL += |DELV| * KPIP1 (scaled 2^+7). */
    public void dvtotup() {
        double[] delv = {
            Agc.toSigned(agc.get("DELVX")), Agc.toSigned(agc.get("DELVY")), Agc.toSigned(agc.get("DELVZ"))};
        double abs = DpWords.abval(delv) * KPIP1;
        DpWords.das(agc, "DVTOTAL", Math.round(abs * 16384.0));
        averageg();
    }

    /** AVERAGEG: advance RN, VN, GDT/2 with CALCRVG; copy for restart protection; jump via AVGEXIT. */
    public void averageg() {
        PhaseTableMaintenance ptm = new PhaseTableMaintenance(agc);
        ptm.phaschng(010035, 0);
        calcrvg();
        ptm.phaschng(010035, 0);
        copyNewToOld();                                 // GENTRAN of 25 words (OCT31)
        ptm.phaschng(010035, 0);
        avgexit.run();                                  // DXCH Z: AVERAGEG exit
    }

    /** GENTRAN copy: RN1,VN1,GDT1/2,GOBL1/2,PIPTIME1 into RN,VN,GDT/2,GOBL/2,PIPTIME. */
    private void copyNewToOld() {
        copy("RN1", "RN", 6);
        copy("VN1", "VN", 6);
        copy("GDT1/2", "GDT/2", 6);
        copy("GOBL1/2", "GOBL/2", 6);
        copy("PIPTIME1", "PIPTIME", 2);
    }

    private void copy(String from, String to, int n) {
        for (int i = 0; i < n; i++) {
            agc.set(DpWords.off(to, i), agc.get(DpWords.off(from, i)));
        }
    }

    /** AVGEND: final average G exit; sets up free-fall gyro compensation and goes to CANV37. */
    public void avgend() {
        agc.set("OLDBT1", agc.get("PIPTIME+1"));        // free-fall gyro compensation
        service.upflag("FLAGWRD2", 15);                 // DRIFTFLG, bit 15 flag 2
        setFlag("DRIFTFLG", true);
        PhaseTableMaintenance ptm = new PhaseTableMaintenance(agc);
        ptm.twoPhschng(5, 05022, 020000);               // group 5 off, group 2 on for AVETOMID
        new IntegrationInitialization(agc).avetomid();  // convert state vector to reference scale
        agc.set("VHFCNT", 0);                           // zero mark counters
        agc.set("TRKMKCNT", 0);
        new PhaseTableMaintenance(agc).pipfree();
        agc.set("MRKBUF2", Agc.fromSigned(~BIT9 & Agc.WORD_MASK)); // invalidate mark buffer
        agc.set("DSALMOUT", agc.get("DSALMOUT") & ~BIT9);          // WAND: test connector outbit off
        setFlag("CM/DSTBY", false);
        setFlag("V37FLAG", false);
        if ((agc.get("FLAGWRD0") & 0100) != 0) {        // bit 7: P20 running -> restore groups 1 and 2
            ptm.twoPhschng(0111, 0132, 0);              // 1.11SPOT, 2.13SPOT
        }
        new FreshStartAndRestart(agc).canv37();                       // POSTJUMP CANV37 (continue MM change R00)
    }

    /** SERVEXIT: normal exit from AVERAGE G - phase change for restart protection, then ENDOFJOB. */
    public void servexit() {
        new PhaseTableMaintenance(agc).phaschng(00035, 0);   // A, 5.3 = REREADAC (only)
        executive.endofjob();
    }

    // ----------------------------------------------------------------------------------
    // NORMLIZE
    // ----------------------------------------------------------------------------------

    /**
     * NORMLIZE: initialization required before the first AVERAGE G entry. Copies RN1,VN1,PIPTIME1 into
     * RN,VN,PIPTIME and initialises GDT/2 (UNITR, RMAG, GDT1/2) via CALCGRAV.
     */
    public void normlize() {
        copy("RN1", "RN", 6);
        copy("VN1", "VN", 6);
        copy("PIPTIME1", "PIPTIME", 2);                 // 14 registers
        double[] g = calcgrav(DpWords.getVec(agc, "RN"));
        DpWords.setVec(agc, "GDT/2", g);
        DpWords.setVec(agc, "GOBL/2", DpWords.getVec(agc, "GOBL1/2"));
        executive.endofjob();
    }

    // ----------------------------------------------------------------------------------
    // PIPASR
    // ----------------------------------------------------------------------------------

    /**
     * PIPASR: read and clear the PIPA counters carefully so the routine is restartable. DELV major parts
     * receive the readings; low parts are zeroed for telemetry; PIPTIME1 receives TIME2.
     */
    public void pipasr() {
        agc.set("PIPTIME1", agc.get("TIME2"));
        agc.set("PIPTIME1+1", agc.get("TIME2+1"));
        agc.set("TEMX", Agc.WORD_MASK);                 // initialise at negative zero
        agc.set("TEMY", Agc.WORD_MASK);
        agc.set("TEMZ", Agc.WORD_MASK);
        for (String ax : new String[] {"X", "Y", "Z"}) {
            agc.set("DELV" + ax, 0);
        }
        agc.set("DELVX+1", 0);                          // low order DELVs zeroed for TM
        agc.set("DELVY+1", 0);
        agc.set("DELVZ+1", 0);
        agc.set("PIPAGE", 0);                           // PIPA reading in progress
        // REPIP1: X and Y read together, PIPAs set to -0 as read; repeat if X changed meanwhile
        int x;
        int y;
        do {
            x = agc.get("PIPAX");
            y = agc.get("PIPAY");
            agc.set("PIPAX", Agc.WORD_MASK);
            agc.set("PIPAY", Agc.WORD_MASK);
        } while (false);
        agc.set("DELVX", x);
        agc.set("DELVY", y);
        // REPIP3: Z pip
        int z = agc.get("PIPAZ");
        agc.set("PIPAZ", Agc.WORD_MASK);
        agc.set("DELVZ", z);
    }

    /**
     * REREADAC: restart entry for the PIP reader. Restores 1/PIPADT in case LASTBIAS was lost and
     * resumes reading from wherever PIPAGE/DELV show the previous attempt stopped.
     */
    public void rereadac() {
        if (agc.get("PHASE5") == 0) {                   // last pass check: CCS PHASE5
            return;                                     // TASKOVER
        }
        agc.set("1/PIPADT", PRIO31);                    // 2.0 sec scaled B+8 cs
        if (Agc.toSigned(agc.get("PIPAGE")) > 0) {
            readaccs();                                 // PIP reading not started: go to beginning
            return;
        }
        // PIPs partially read: DELVZ/DELVY/DELVX non-zero mean those axes are done, TEMX/TEMY/TEMZ hold
        // the in-flight values (see original CCS tree); finish the missing axes then continue.
        if (agc.get("DELVZ") == 0 && agc.get("TEMZ") != Agc.WORD_MASK) {
            agc.set("DELVZ", ~agc.get("TEMZ") & Agc.WORD_MASK);
        }
        if (agc.get("DELVY") == 0 && agc.get("TEMY") != Agc.WORD_MASK) {
            agc.set("DELVY", ~agc.get("TEMY") & Agc.WORD_MASK);
        }
        if (agc.get("DELVX") == 0 && agc.get("TEMX") != Agc.WORD_MASK) {
            agc.set("DELVX", ~agc.get("TEMX") & Agc.WORD_MASK);
        }
        pipsdone();                                     // DONEADR
    }

    // ----------------------------------------------------------------------------------
    // CALCGRAV / CALCRVG
    // ----------------------------------------------------------------------------------

    /**
     * CALCGRAV: gravity term (GDT1/2) for the position vector rn (half-unit scaled). Stores UNITR and
     * GOBL1/2 (oblateness part, Earth only); for the Moon (RTX2 negative) only the central term is used.
     *
     * @return GDT1/2 scaled 2^+7 m/cs
     */
    public double[] calcgrav(double[] rn) {
        double rmag = DpWords.abval(rn);
        double[] unitr = DpWords.unit(rn);              // UNIT PUSH, STORE UNITR
        DpWords.setVec(agc, "UNITR", unitr);
        boolean moon = Agc.toSigned(agc.get("RTX2")) < 0;   // LXC,1 RTX2 ; BMN ITISMOON
        double[] vec = unitr;
        double mudt = NEG_MUDT_M;
        if (!moon) {
            double[] unitw = DpWords.getVec(agc, "UNITW");
            double c = DpWords.dot(unitr, unitw);                  // DOT UNITW
            double bracket = DP1_20 - c * c;                       // DSQ BDSU DP1/20
            double rmagsq = rmag * rmag;
            double k = RESQ / rmagsq;                              // (RE/RN)SQ, stored at 32D
            double[] gobl = DpWords.scale(unitr, k * bracket * J20);   // 20J term along UNITR
            gobl = DpWords.add(gobl, DpWords.scale(unitw, c * k * J2)); // 2J term along UNITW
            DpWords.setVec(agc, "GOBL1/2", gobl);
            vec = DpWords.add(gobl, unitr);                        // VAD UNITR
            mudt = NEG_MUDT_E;
        }
        double rsq = rmag * rmag;
        double coef = rsq == 0 ? 0 : mudt / rsq;                   // BDDV* -MUDT(E),1 over RMAGSQ
        double[] gdt = DpWords.scale(vec, coef);                   // VXSC: GDT1/2 scaled 2^+7 m/cs
        DpWords.setVec(agc, "GDT1/2", gdt);
        return gdt;
    }

    /**
     * CALCRVG: integrate the equations of motion by averaging thrust and gravitational acceleration over
     * the 2-second interval. Produces DELVREF, RN1, VN1 and GDT1/2.
     */
    public void calcrvg() {
        double[] delv = {
            Agc.toSigned(agc.get("DELVX")), Agc.toSigned(agc.get("DELVY")), Agc.toSigned(agc.get("DELVZ"))};
        double[] dv = DpWords.scale(delv, KPIP1);       // VXSC KPIP1
        double[][] refsmmat = refsmmat();
        double[] delvref = new double[3];               // VXM REFSMMAT, VSL1: DELV in ref coords, 2^+7
        for (int i = 0; i < 3; i++) {
            delvref[i] = (dv[0] * refsmmat[0][i] + dv[1] * refsmmat[1][i] + dv[2] * refsmmat[2][i]) * 2.0;
        }
        DpWords.setVec(agc, "DELVREF", delvref);
        double[] half = DpWords.scale(delvref, 0.5);                    // VSR1 PUSH
        double[] sum = DpWords.add(half, DpWords.getVec(agc, "GDT/2")); // (DV - OLDGDT)/2
        double[] vn = DpWords.getVec(agc, "VN");
        double[] rn = DpWords.getVec(agc, "RN");
        double[] rn1 = DpWords.add(DpWords.add(DpWords.scale(vn, TWO_SEC_22), sum), rn);  // temp RN1
        // The original interleaves 2SEC(22) scaling; position update uses VN * dt + (dv + gdt) * dt.
        DpWords.setVec(agc, "RN1", rn1);
        double[] gdt1 = calcgrav(rn1);                  // CALL CALCGRAV with RN1
        double[] vn1 = DpWords.add(DpWords.add(vn, delvref), DpWords.add(DpWords.getVec(agc, "GDT/2"), gdt1));
        DpWords.setVec(agc, "VN1", vn1);                // VN + DELVREF + GDT/2 + GDT1/2
    }

    private double[][] refsmmat() {
        double[][] m = new double[3][3];
        for (int i = 0; i < 9; i++) {
            m[i / 3][i % 3] = DpWords.get(agc, DpWords.off("REFSMMAT", 2 * i));
        }
        return m;
    }
}
