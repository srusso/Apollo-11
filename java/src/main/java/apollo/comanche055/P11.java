package apollo.comanche055;

import apollo.agc.Agc;

/**
 * P11: Earth orbit insertion monitor. Started at liftoff; zeroes the clock, updates TEPHEM,
 * initialises average-G, sets up the launch REFSMMAT and runs the FDAI attitude error
 * display (ATERJOB, with Saturn pitch/roll polynomial and S-IVB takeover/stick handling),
 * and the V06N62 velocity/altitude-rate/altitude display (VHHDOT). Also contains the
 * Saturn stick takeover (V46) routines SATSTKON, REDOSAT and SATSTICK.
 */
public class P11 {
    /** -ERTHRAT: minus earth rate, rad/sec x 2^18 scaling. */
    public static final double NEG_ERTHRAT = -7.292115138E-7;
    /** REP11S: restart table entry (2OCT 7776600011). */
    public static final long REP11S = 0x7FFF6000_11L;
    /** REP11SA: restart table entry (2OCT 7776400013). */
    public static final long REP11SA = 0x7FFF4000_13L;
    /** V06N62 display: VI, HDOT, H. */
    public static final int V06N62 = 0662;
    /** POWDNCOD = THREE: powered flight downlist code. */
    public static final int POWDNCOD = 3;
    /** STIKBITS: manual rotation bits mask (OCT 00077). */
    public static final int STIKBITS = 077;
    /** 100MST5: 100 ms on T5 clock (DEC 16374). */
    public static final int MS100_T5 = 16374;

    private static final int BIT1 = 0x0001;
    private static final int BIT3 = 0x0004;
    private static final int BIT9 = 0x0100;
    private static final int BIT10 = 0x0200;
    private static final int OCT60000 = 060000 & Agc.WORD_MASK;

    private final Agc agc;

    public P11(Agc agc) {
        this.agc = agc;
    }

    private long dp(String name) {
        return ((long) Agc.toSigned(agc.get(name)) << 14) + Agc.toSigned(agc.get(name + "+1"));
    }

    private void setDp(String name, long v) {
        int sign = v < 0 ? -1 : 1;
        long mag = Math.abs(v);
        agc.set(name, Agc.fromSigned(sign * (int) (mag >> 14)));
        agc.set(name + "+1", Agc.fromSigned(sign * (int) (mag & 0x3FFF)));
    }

    /** P11: entry at liftoff (from the liftoff discrete). Directs restarts to REP11, zeroes the clock. */
    public void p11() {
        agc.set("EBANK", agc.get("EBANK3"));
        // DXCH -PHASE3 <- REP11S : direct restarts to REP11
        agc.set("LIFTTEMP", 0);
        agc.set("-PHASE5", Agc.WORD_MASK); // inactive group 5 (prelaunch protection)
        p11Plus7();
    }

    /** P11+7: record liftoff time in TLIFTOFF, save the clock in LIFTTEMP and zero TIME2/TIME1. */
    public void p11Plus7() {
        setDp("TLIFTOFF", REP11SA);
        setDp("LIFTTEMP", dp("TIME2"));  // for restarts
        setDp("TIME2", 0);
        rep11aMinus2();
    }

    /** REP11A-2: TLIFTOFF <- LIFTTEMP (saved clock). */
    private void rep11aMinus2() {
        setDp("TLIFTOFF", dp("LIFTTEMP"));
        rep11aMinus1();
    }

    /** REP11A-1: reset phase 3. */
    private void rep11aMinus1() {
        setDp("-PHASE3", REP11S);
        rep11a();
    }

    /** REP11A: update TEPHEM by the liftoff time and continue initialisation. */
    public void rep11a() {
        // INHINT ; TEPHEM1 <- TEPHEM + TLIFTOFF (triple precision with overflow correction)
        setDp("TEPHEM1+1", dp("TEPHEM+1") + dp("TLIFTOFF"));
        agc.set("TEPHEM1", Agc.add(agc.get("TEPHEM"), 0));
        // PHASCHNG 05023 / 22000
        setDp("TEPHEM", dp("TEPHEM1"));
        agc.set("TEPHEM+2", agc.get("TEPHEM1+2"));
        agc.set("EBANK", agc.get("EBDVCNT"));
        // IBNKCALL PREREAD1 : zero PIPs and initialise average-G
        // PHASCHNG 05023 / 22000 : continue here on restart
        // WAITLIST .5 SEC -> ATERTASK (start attitude error display)
        agc.set("MODREG", 11);          // NEWMODEX: display MM 11
        agc.set("NODOP01", 1);          // UPFLAG NODOP01
        agc.set("DNLSTCOD", POWDNCOD);  // switch to powered flight downlist
        // BANKCALL CLEANDSP : clear DSKY in case of V75
        // 2PHSCHNG 40514 / 00073 : protect ATERTASK
        agc.set("EBANK", agc.get("EBQPLACE"));
        agc.set("QPLACES", 0 /* P11XIT = GENADR P11OUT : exit from PROUT in EARTHR */);
        // INTERPRETIVE: ERCOMP <- ERCOMP + 2*(XSM * THETAN) ; TLIFTOFF ; GOTO EARTHR+3 (returns to PROUT)
        // After EARTHR: p11Out()
    }

    /** P11OUT / MATRXJOB: store gimbal angles, build the launch REFSMMAT and start average G. */
    public void matrxjob() {
        // Store DP gimbal angles for attitude error display after liftoff
        setDp("OGC", (long) Agc.toSigned(agc.get("CDUX")) << 14);
        setDp("IGC", (long) Agc.toSigned(agc.get("CDUY")) << 14);
        setDp("MGC", (long) Agc.toSigned(agc.get("CDUZ")) << 14);
        // OGC scaled B-1 (VSR1)
        agc.set("RTX2", 0);   // zero for earth
        // LAT/LONG/ALT <- (PGNCSALT, PADLONG, LATITUDE); TIME = 0; CLEAR ERADFLAG
        // RN1 <- LALOTORV (position vector in reference coordinates); GETDOWN gives down direction
        // REFSMMAT+12 (UNITZ) <- UNIT(-GRAV) ; VN1 <- (EARTHRATE x RN1) << 4
        // EAST <- UNIT(REF3 x UNITW) ; SOUTH <- EAST x REF3
        // REFSMMAT (REF1) <- UNIT(SIN(LAUNCHAZ)*EAST - COS(LAUNCHAZ)*SOUTH)
        // REFSMMAT+6 (REF2) <- -UNIT(REF1 x REF3)
        // SATRLRT <- SIGN(SATRLRT, (1/2rev - LAUNCHAZ + AZIMUTH)) : roll rate sign for FDAI
        agc.set("REFSMFLG", 1);   // REFSMMAT known
        // PHASCHNG 04023 ; AVGEXIT <- P11SCADR (VHHDOT)
        agc.set("1/PIPADT", agc.get("PRIO31")); // 2 seconds at 2(+8)
        agc.set("EBANK", agc.get("EBANK6"));
        // INHINT; TBASE5 = -0; schedule READACCS 2 sec after liftoff (DT = 2SECS - TIME1, min 1)
        int dt = Agc.add(Agc.fromSigned(-Agc.toSigned(agc.get("TIME1"))), agc.get("2SECS"));
        if (Agc.toSigned(dt) <= 0) {
            dt = 1;
        }
        agc.set("TBASE5", Agc.fromSigned(0));
        // WAITLIST dt -> READACCS
        // 2PHSCHNG 00003 / 00025 : turn off group 3, protect NORMLIZE and READACCS
        // POSTJUMP NORMLIZE : do NORMLIZE and ENDOFJOB
    }

    /** REP11: restart entry for P11. */
    public void rep11() {
        // INHINT
        if (agc.get("PHASE5") != 0 && agc.get("PHASE5") != Agc.WORD_MASK) {
            return; // CCS PHASE5 positive: TC ENDOFJOB
        }
        int lt = Agc.toSigned(agc.get("LIFTTEMP"));
        if (lt == 0 && agc.get("LIFTTEMP") == 0) {
            // CCS cases: (positive) P11+7, (+0) / (-0) fall on through
        }
        if (Agc.toSigned(agc.get("LIFTTEMP")) > 0) {
            p11Plus7();
            return;
        }
        if (Agc.toSigned(dpHigh("TLIFTOFF")) <= 0 && dpHigh("TLIFTOFF") != 0) {
            // CS TLIFTOFF; BZMF -> ENDREP11 when TLIFTOFF >= 0
        }
        if (Agc.toSigned(agc.get("TLIFTOFF")) >= 0) {
            endrep11();
            return;
        }
        if (Agc.toSigned(agc.get("TIME2")) != 0) {
            // T2,T1 nonzero: not yet zeroed; go and do it
            p11Plus7();
            return;
        }
        // T2,T1 zeroed: set TLIFTOFF from LIFTTEMP
        rep11aMinus2();
    }

    private int dpHigh(String name) {
        return agc.get(name);
    }

    /** ENDREP11. */
    public void endrep11() {
        setDp("TLIFTOFF", REP11SA);
        rep11aMinus1();
    }

    /** VHHDOT: average-G exit; S11.1 computes VMAGI, HDOT, ALTI; then display V06N62. */
    public void vhhdot() {
        s111();
        // PHASCHNG 00035 ; CAF V06N62 ; BANKCALL REGODSP : VI, HDOT, H
    }

    /** ATERTASK: waitlist task 0.33 s after liftoff; creates ATERJOB and initialises needles. */
    public void atertask() {
        // FINDVAC PRIO1 -> ATERJOB
        agc.set("RCSFLAGS", Agc.add(agc.get("RCSFLAGS") & ~BIT3 & Agc.WORD_MASK, BIT3)); // NEEDLER init pass
        // IBNKCALL NEEDLER
        agc.set("SATSW", BIT1);   // set SW for display
        // TC TASKOVER
    }

    /** ATERJOB: attitude error display job (cycles about every 0.56 s). */
    public void aterjob() {
        if ((agc.get("FLAGWRD6") & OCT60000) != 0) {
            return; // Saturn stick on - kill job
        }
        boolean panelControl = (agc.get("CHAN30") & BIT10) == 0; // RAND CHAN30; BZF STRSAT
        if (panelControl) {
            strsat();
            return;
        }
        int satsw = Agc.toSigned(agc.get("SATSW"));
        if (satsw > 0) {
            attdisp();
        } else {
            atreset();
        }
    }

    /** ATTDISP: compute pitch/roll program angles and the CDU-based attitude errors. */
    public void attdisp() {
        int t = Agc.toSigned(Agc.add(agc.get("TIME1"), Agc.fromSigned(-Agc.toSigned(agc.get("RPSTART")))));
        if (t <= 0) {
            nopoly();   // attitude hold before pitch/roll start
            return;
        }
        agc.set("MPAC", Agc.fromSigned(t));
        agc.set("SPOLYARG", Agc.fromSigned(t));
        if (Agc.toSigned(Agc.add(Agc.fromSigned(t), agc.get("POLYSTOP"))) > 0) {
            nopoly();
            return;
        }
        if (Agc.toSigned(agc.get("TIME2")) <= 0) {
            nopoly();
            return;
        }
        // POWRSERS: evaluate pitch polynomial (COEFPOLY=POLYLOC, order POLYNUM), scaled 32 revs
        // PITCH(rev) = 32 * PITCH(32rev); P(rad) = 2*pi*PITCH(rev)
        // 14D = -.5*cos(P), 10D = .5*sin(P), 12D = 0
        // ROLL = (TIME1 - RPSTART) * SATRLRT  - 1/2rev - LAUNCHAZ - AZIMUTH ; RLTST ends rollout
        // 22D=-.5cosR cosP, 18D=-.5cosR, 26D=-.5cosR sinP, 24D=.5sinR, 16D=-.5sinR cosP, 20D=-.5sinR sinP
        // XDC = .5*UNIT(sinP, 0, -cosP) ; YDC = .5*UNIT(-sinR cosP, -cosR, -sinR sinP)
        // ZDC = .5*UNIT(-cosR cosP, sinR, -cosR sinP) ; CALCGTA
        nopolym();
    }

    /** NOPOLY: attitude hold; continue at NOPOLYM using current attitude. */
    public void nopoly() {
        nopolym();
    }

    /** NOPOLYM: compute delta-CDU from stored gimbal angles and the AK needle values. */
    public void nopolym() {
        // CDUTRIG ; DELTACDU: DPHI = OGC - CDUX, DTHETA = IGC - CDUY, DPSI = MGC - CDUZ (V1STO2S, DELSTOR)
        delstor();
        attdisp2();
    }

    /** ATTDISP2: convert delta-CDU to body attitude errors AK, AK1, AK2. */
    private void attdisp2() {
        // AK  = .5*(DPHI + DTHETA*sin(CDUZ))
        // AK1 = .5*(DTHETA*cos(CDUX)*cos(CDUZ) + DPSI*sin(CDUX))
        // AK2 = .5*(-DTHETA*sin(CDUX)*cos(CDUZ) + DPSI*cos(CDUX))
        int satsw = Agc.toSigned(agc.get("SATSW"));
        if (satsw > 0) {
            satout();       // display only
        } else if (satsw == 0) {
            akload();       // store bias
        } else {
            steersat();     // steer launch vehicle
        }
    }

    /** STEERSAT: AK <- AK + BIASAK, then rescale for Saturn steering. */
    public void steersat() {
        agc.set("AK", Agc.add(agc.get("AK"), agc.get("BIASAK")));
        agc.set("AK1", Agc.add(agc.get("AK1"), agc.get("BIASAK+1")));
        agc.set("AK2", Agc.add(agc.get("AK2"), agc.get("BIASAK+2")));
        agc.set("AK", rescales(agc.get("AK")));
        agc.set("AK1", rescales(agc.get("AK1")));
        agc.set("AK2", rescales(agc.get("AK2")));
        satout();
    }

    /** SATOUT: output needles (NEEDLER), delay 0.25 s, and cycle again. */
    public void satout() {
        // BANKCALL NEEDLER
        atterSet();
    }

    private void atterSet() {
        // ATERSET: CAF OCT31 ; DELAYJOB (about .56 s per cycle); TC ATERJOB (recurse as a job loop)
    }

    /** AKLOAD: store (complemented) AK as bias, set SATSW to steer, then steer. */
    public void akload() {
        agc.set("BIASAK", Agc.fromSigned(-Agc.toSigned(agc.get("AK"))));
        agc.set("BIASAK+1", Agc.fromSigned(-Agc.toSigned(agc.get("AK1"))));
        agc.set("BIASAK+2", Agc.fromSigned(-Agc.toSigned(agc.get("AK2"))));
        agc.set("SATSW", Agc.fromSigned(-1));
        steersat();
    }

    /** STRSAT: Saturn panel control; if needler initialised continue display, else reset. */
    public void strsat() {
        if (Agc.toSigned(agc.get("SATSW")) <= 0) {
            attdisp();
        } else {
            atreset();
        }
    }

    /** ATRESET: reinitialise the needler and wait for the IMU error counters to zero. */
    public void atreset() {
        agc.set("RCSFLAGS", Agc.add(agc.get("RCSFLAGS") & ~BIT3 & Agc.WORD_MASK, BIT3));
        // BANKCALL NEEDLER ; CAF REVCNT (6 = 60 ms) ; DELAYJOB
        int satsw = Agc.toSigned(agc.get("SATSW"));
        if (satsw > 0) {
            takeon();
            return;
        }
        agc.set("SATSW", BIT1);                          // display on
        agc.set("CHAN12", agc.get("CHAN12") & ~BIT9 & Agc.WORD_MASK); // disable S-IVB takeover
        satout();
    }

    /** TAKEON: enable S-IVB takeover. */
    public void takeon() {
        agc.set("CHAN12", agc.get("CHAN12") | BIT9);
        agc.set("SATSW", 0);        // needler initialised
        satout();
    }

    /** S11.1: VMAGI = |VN|; HDOT = UNIT(RN).VN; ALTI = |RN| - |RPAD| (or minus RLS on the Moon). */
    public void s111() {
        // VLOAD ABVAL VN ; STOVL VMAGI ; RN ; UNIT DOT VN ; SL1 ; STODL HDOT ; RPAD
        // BOF AMOONFLG EARTHALT : ABVAL(RLS)>>2 for lunar ; BDSU 36D ; STORE ALTI
        // 'RVQ' return
    }

    /** DELSTOR: store delta-CDU into PUSHLIST locations 0,2,4 (BOOSTEMP - MPAC). */
    public void delstor() {
        String[] mp = {"MPAC", "MPAC+1", "MPAC+2"};
        for (int i = 0; i < 3; i++) {
            String bt = i == 0 ? "BOOSTEMP" : "BOOSTEMP+" + i;
            int v = Agc.add(agc.get(bt), Agc.fromSigned(-Agc.toSigned(agc.get(mp[i]))));
            agc.set("FIXLOC+" + (2 * i), v);
        }
    }

    /** RLTST: determine whether roll-out is complete; if so zero the roll contribution. */
    public void rltst() {
        long prod = (long) Agc.toSigned(agc.get("MPAC")) * Agc.toSigned(agc.get("SATRLRT+1"));
        if (prod <= 0) {
            return; // unlike signs: still rolling
        }
        // roll-out completed: MPAC <- MBDYTCTL+2 (zero out roll contribution)
        agc.set("MPAC", agc.get("MBDYTCTL+2"));
    }

    /** RESCALES: rescale AK words for the new Saturn hardware (x SATSCALE x 4). */
    public int rescales(int ak) {
        long prod = (long) Agc.toSigned(ak) * Agc.toSigned(agc.get("SATSCALE"));
        return Agc.fromSigned((int) ((prod >> 14) * 4));
    }

    // ---- Saturn stick takeover (V46) ----

    /** SATSTKON: set up T5 for Saturn stick control. */
    public void satstkon() {
        agc.set("T5LOC", 0 /* REDOSAT 2CADR */);
        agc.set("TIME5", 0x3FFF);                    // POSMAX
        agc.set("FLAGWRD6", agc.get("FLAGWRD6") | OCT60000); // RELTAB11: bits 15,14 on
        // IBNKCALL ZEROJET: zero jet channels in 14 ms, leave T6 disabled
        // GOPIN : exit via V46 path
    }

    /** REDOSAT: also entered on restarts; reinitialise needler, enable takeover, start T5 cycle. */
    public void redosat() {
        agc.set("RCSFLAGS", Agc.add(agc.get("RCSFLAGS") & ~BIT3 & Agc.WORD_MASK, BIT3));
        // IBNKCALL NEEDLER
        agc.set("CHAN12", agc.get("CHAN12") | BIT9);   // S-IVB takeover enable
        agc.set("T5LOC", 0 /* SATSTICK 2CADR */);
        agc.set("TIME5", MS100_T5);
        // TCF RESUME
    }

    /** SATSTICK: T5 interrupt each 100 ms; reads stick bits and sets Saturn attitude rates. */
    public void satstick() {
        agc.set("TIME5", MS100_T5);
        int bits = (agc.get("CHAN31") ^ STIKBITS) & STIKBITS;
        agc.set("STIKTMP", bits);
        // IBNKCALL STICKCHK : set rate indices (RMANNDX, PMANNDX, YMANNDX)
        agc.set("AK", agc.get("SATRATE+" + agc.get("RMANNDX")));    // roll
        agc.set("AK1", agc.get("SATRATE+" + agc.get("PMANNDX")));   // pitch
        agc.set("AK2", agc.get("SATRATE+" + agc.get("YMANNDX")));   // yaw
        // IBNKCALL NEEDLER (Saturn interface and FDAI display) ; TCF RESUME
    }
}
