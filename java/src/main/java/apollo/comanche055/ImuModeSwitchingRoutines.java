package apollo.comanche055;

import apollo.agc.Agc;

/**
 * IMU_MODE_SWITCHING_ROUTINES.agc (Comanche055): IMU zeroing, coarse align, fine align mode
 * switches, PIPA use/free, IRIG gyro torquing (IMUPULSE), the general mode-switch termination
 * (GOODEND/BADEND) and stall routines, GOMANUR, R02 (IMU status check), P06 (standby) and POSTAND.
 *
 * <p>Hardware channels are modelled as erasable symbols "CHAN12", "CHAN13", "CHAN14" in the shared
 * {@link Agc}. AGC instructions WOR/WAND are the helpers {@link #wor}/{@link #wand}. Wherever the
 * AGC code skips its return address (CAGETSTQ) a boolean is returned: true = the skipped (IMU not
 * being caged) path. Tasks waitlisted via 2CADR become {@link Runnable}s.
 */
public final class ImuModeSwitchingRoutines {
    private final Agc agc;

    public ImuModeSwitchingRoutines(Agc agc) {
        this.agc = agc;
    }

    // ---- bit masks ----
    private static final int BIT1 = 1, BIT4 = 1 << 3, BIT5 = 1 << 4, BIT6 = 1 << 5, BIT8 = 1 << 7,
        BIT9 = 1 << 8, BIT10 = 1 << 9, BIT11 = 1 << 10, BIT13 = 1 << 12, BIT14 = 1 << 13, BIT15 = 1 << 14;
    private static final int BITS3_4 = 014;          // BITS3&4 = OCT14
    private static final int BITS4_6 = 050;          // BITS4&6 = OCT50
    private static final int BITS4_5 = 030;          // BITS4-5 = BITS4&5
    private static final int SUPER011 = 060;
    private static final int OCT40010 = 040010;
    private static final int LOW5 = 037, LOW7 = 0177, LOW8 = 0377, SEVEN = 7;
    private static final int OCT13_14_15 = 070000;   // 13,14,15
    private static final int GDESELCT = 01700;       // turn off select and activity bits
    private static final int OCT220 = 0220;
    private static final int OCT62 = 50;             // .5SEC
    private static final int NEG0 = 0x7FFF;

    // ---- constants ----
    /** SPSCODE = BIT9. */
    public static final int SPSCODE = BIT9;
    /** -COMMAX: COMMAX = max number of pulses allowed minus one. */
    public static final int MINUS_COMMAX = -191;
    /** -COMMAX-: -192. */
    public static final int MINUS_COMMAX_MINUS = -192;
    /** 600MS (centiseconds). */
    public static final int MS600 = 60;
    /** COARSTOL: 2 degrees scaled at half-revolutions. */
    public static final double COARSTOL = -.01111;
    /** -GYROMIN: OCT -176, may be adjusted to specify minimum command (16 gyro pulses). */
    public static final int MINUS_GYROMIN = -0176;
    /** GYROFRAC: 2DEC .215 B-21. */
    public static final double GYROFRAC = .215 / (1 << 21) * 2;
    /** IMUSEFLG = BIT8 (interpreter switch 7). */
    public static final int IMUSEFLG = BIT8;
    /** DEC51. */
    public static final int DEC51 = 51;

    // device indexes for mode-switch stalls
    private static final int IMU = 0, OPTICS_AOT = 1, RADAR = 2;

    // ---- channel helpers ----
    private int ch(String name) {
        return agc.get(name);
    }

    private void wor(String name, int mask) {
        agc.set(name, agc.get(name) | mask);
    }

    private void wand(String name, int mask) {
        agc.set(name, agc.get(name) & mask);
    }

    /** CS X; MASK Y; (result = ~X & Y) */
    private static int csMask(int x, int y) {
        return (~x) & y & Agc.WORD_MASK;
    }

    // ---- fixed-fixed ----

    /** ZEROICDU: zero ICDU counters. */
    public void zeroicdu() {
        agc.set("CDUX", 0);
        agc.set("CDUY", 0);
        agc.set("CDUZ", 0);
    }

    // ---- IMU zeroing ----

    /** IMUZERO: routine to zero ICDUs. */
    public void imuzero() {
        // INHINT
        // Do not zero CDUs if IMU in gimbal lock and coarse align (gimbal runaway protection)
        int a = csMask(agc.get("DSPTAB+11"), BITS4_6);
        if (a == 0) {
            new AlarmAndAbort(agc).alarm(0206);
            cagetstjFail();           // TCF CAGETSTJ +4: immediate failure
            return;
        }
        if (!cagetstj()) {
            return;
        }
        // DO ALL THE WORK.
        // disable DAP auto and hold modes (bit 5 for ground)
        agc.set("IMODES33", agc.get("IMODES33") + csMaskAdd(agc.get("IMODES33"), SUPER011));
        // inhibit ICDUFAIL and IMUFAIL (in case we just came out of coarse align)
        agc.set("IMODES30", agc.get("IMODES30") + csMaskAdd(agc.get("IMODES30"), BITS3_4));
        wand("CHAN12", ~BITS4_6 & Agc.WORD_MASK);    // zero encode with coarse and error counter disabled
        noattoff();                                   // turn off NO ATT lamp
        wor("CHAN12", BIT5);
        zeroicdu();
        new Waitlist(agc).waitlist(BIT6, this::imuzero2);   // wait 320 ms for AGS to receive its pulse train
        // see if IMU operating and alarm if not
        if ((agc.get("IMODES30") & BIT9) == 0) {
            new AlarmAndAbort(agc).alarm(0210);
        }
        modeexit();
    }

    /** ADS idiom: CS IMODES; MASK mask; ADS IMODES = add the bits that were not set (set the bits in mask). */
    private int csMaskAdd(int modes, int mask) {
        return csMask(modes, mask);
    }

    /** MODEEXIT: general mode-switching exit (RELINT; TCF SWRETURN). */
    public void modeexit() {
        // RELINT; TCF SWRETURN
        new InterBankCommunication(agc).swreturn();
    }

    /** IMUZERO2: waitlist task, 320 ms after the zero encode. */
    public void imuzero2() {
        if (!cagetest()) {
            return;
        }
        zeroicdu();                                        // zero CDUX, CDUY, CDUZ
        wand("CHAN12", ~BIT5 & Agc.WORD_MASK);             // remove zero discrete
        new Waitlist(agc).vardelay(BIT11, this::imuzero3); // wait 10 secs for counters to find gimbals
    }

    /** IMUZERO3. */
    public void imuzero3() {
        if (!cagetest()) {
            return;
        }
        agc.set("IMODES30", csMask(BITS3_4, agc.get("IMODES30")));   // remove IMUFAIL and ICDUFAIL inhibit
        agc.set("IMODES33", csMask(SUPER011, agc.get("IMODES33")));  // enable DAP auto and hold modes
        // IBNKCALL SETISSW: set ISS warning if either of above are present
        endimu();
    }

    // ---- IMU coarse align ----

    /** IMUCOARS: IMU coarse align mode. */
    public void imucoars() {
        // INHINT
        if (!cagetstj()) {
            return;
        }
        setcoars();
        new Waitlist(agc).waitlist(6, this::coars);
        modeexit();
    }

    private static final String[] THETAD = {"THETAD", "THETAD+1", "THETAD+2"};
    private static final String[] CDU = {"CDUX", "CDUY", "CDUZ"};
    private static final String[] COMMAND = {"COMMAND", "COMMAND+1", "COMMAND+2"};
    private static final String[] CDUCMD = {"CDUXCMD", "CDUYCMD", "CDUZCMD"};

    /** COARS: first coarse-align task. */
    public void coars() {
        if (!cagetest()) {
            return;
        }
        wor("CHAN12", BIT6);                          // enable all three ISS CDU error counters
        for (int idx = 2; idx >= 0; idx--) {          // COARS1: CDU indicator
            agc.set("CDUIND", idx);
            // compute THETAD - THETAA in 1's complement form, shifted right 2 and rounded
            int diff = Agc.toSigned(agc.get(THETAD[idx])) - Agc.toSigned(agc.get(CDU[idx]));
            agc.set(COMMAND[idx], Agc.fromSigned(diff >= 0 ? (diff + 2) >> 2 : -((-diff + 2) >> 2)));
        }
        new Waitlist(agc).vardelay(2, this::coars2);  // minimum of 4 ms wait
    }

    /** COARS2: compute pulses per CDU, limited to COMMAX per pass. */
    public void coars2() {
        if (!cagetest()) {
            return;
        }
        int itemp1 = 0;                               // pulses-to-go flag
        for (int idx = 2; idx >= 0; idx--) {
            agc.set("CDUIND", idx);
            int cmd = Agc.toSigned(agc.get(COMMAND[idx]));
            int out;
            if (cmd > 0) {                            // COMPOS
                int rem = cmd + MINUS_COMMAX;
                if (rem <= 0) {                       // COMZERO case: fits in one burst
                    agc.set(COMMAND[idx], 0);
                    out = cmd;
                } else {
                    agc.set(COMMAND[idx], Agc.fromSigned(rem));   // reduce command by max pulses allowed
                    out = -MINUS_COMMAX_MINUS;                    // CS -COMMAX-
                }
                itemp1++;
                agc.set(CDUCMD[idx], Agc.fromSigned(out));
            } else if (cmd < 0) {                     // COMNEG
                int rem = -cmd + MINUS_COMMAX;
                if (rem <= 0) {
                    agc.set(COMMAND[idx], 0);
                    out = cmd;
                } else {
                    agc.set(COMMAND[idx], Agc.fromSigned(-rem));
                    out = MINUS_COMMAX_MINUS;
                }
                itemp1++;
                agc.set(CDUCMD[idx], Agc.fromSigned(out));
            } else {
                agc.set(CDUCMD[idx], NEG0);           // zero command: -0 (NEXTCDU +1 path)
            }
        }
        if (itemp1 != 0) {
            sendpuls();                               // pulses to go out
            return;
        }
        new Waitlist(agc).fixdelay(150, this::chkcors0);   // wait for gimbals to settle
    }

    /** SENDPULS: send the pulse trains and delay 600 ms before the next COARS2 pass. */
    public void sendpuls() {
        wor("CHAN14", OCT13_14_15);
        new Waitlist(agc).vardelay(MS600, this::coars2);
    }

    private void chkcors0() {
        chkcors(2);
    }

    /** CHKCORS: at end of command, check gimbals are within 2 degrees of THETAD. */
    public void chkcors(int idx) {
        for (int i = idx; i >= 0; i--) {
            int err = Agc.toSigned(agc.get(CDU[i])) - Agc.toSigned(agc.get(THETAD[i]));
            if (err != 0) {
                // COARSERR: AD COARSTOL; BZMF CORSCHK2 -- error within tolerance continues
                double rev = Math.abs(err) / 16384.0;   // half-revolutions
                if (rev + COARSTOL > 0) {
                    new AlarmAndAbort(agc).alarm(0211);  // coarse align error
                    imubad();
                    return;
                }
            }
        }
        endimu();                                       // end of coarse alignment
    }

    /** CA+ECE: enable all three ISS CDU error counters (waitlist task). */
    public void caEce() {
        wor("CHAN12", BIT6);
        // TC TASKOVER
    }

    /** SETCOARS: put the ISS in coarse align (bypass if already in coarse align). */
    public void setcoars() {
        if ((ch("CHAN12") & BIT4) != 0) {
            return;
        }
        wand("CHAN12", ~BIT6 & Agc.WORD_MASK);       // clear ISS error counters
        wand("CHAN14", ~BIT10 & Agc.WORD_MASK);      // knock down gyro activity
        agc.set("GYROCMD", NEG0);                    // CS ZERO; TS GYROCMD
        wor("CHAN12", BIT4);                         // put ISS in coarse align
        agc.set("DSPTAB+11", agc.get("DSPTAB+11") + csMask(agc.get("DSPTAB+11"), OCT40010));  // NO ATT lamp on
        agc.set("IMODES33", agc.get("IMODES33") | BIT6);   // disable DAP auto and hold modes
        agc.set("IMODES30", agc.get("IMODES30") | BIT4);   // disable IMUFAIL
        rndrefdr();
    }

    /** RNDREFDR: knock down the track, drift and REFSMMAT flags. */
    public void rndrefdr() {
        agc.set("FLAGWRD1", csMask(BIT5, agc.get("FLAGWRD1")));    // track flag
        agc.set("FLAGWRD2", csMask(BIT15, agc.get("FLAGWRD2")));   // drift flag
        agc.set("FLAGWRD3", csMask(BIT13, agc.get("FLAGWRD3")));   // REFSMMAT flag
    }

    // ---- IMU fine align ----

    /** IMUFINE: IMU fine align mode switch. */
    public void imufine() {
        // INHINT
        if (!cagetstj()) {
            return;
        }
        wand("CHAN12", ~BITS4_5 & Agc.WORD_MASK);                  // reset zero and coarse
        agc.set("IMODES33", csMask(BIT6, agc.get("IMODES33")));    // ensure DAP auto and hold enabled
        noattoff();
        // IMU fail was inhibited during the presumably preceding coarse align; leave on first 5 secs
        Waitlist w = new Waitlist(agc);
        w.waitlist(BIT10, this::ifailok);
        w.waitlist(200, this::imufined);          // 2SECS
        modeexit();
    }

    /** IMUFINED. */
    public void imufined() {
        if (!cagetest()) {
            return;
        }
        endimu();
    }

    /** IFAILOK: enable IMU fail unless IMU being caged. */
    public void ifailok() {
        if (!cagetstq()) {
            return;                               // being caged: TASKOVER
        }
        if ((ch("CHAN12") & BIT4) != 0) {
            return;                               // someone has gone into coarse align
        }
        agc.set("IMODES30", agc.get("IMODES30") + csMask(agc.get("IMODES30"), BIT13));   // reset IMUFAIL
        pfailok2(BIT4);
    }

    private void pfailok2(int clearMask) {
        agc.set("IMODES30", csMask(clearMask, agc.get("IMODES30")));
        // IBNKCALL SETISSW: ISS warning light may come on now that the inhibit was removed
    }

    /** PFAILOK: enable PIP fail program alarm. */
    public void pfailok() {
        if (!cagetstq()) {
            return;
        }
        agc.set("IMODES30", agc.get("IMODES30") + csMask(agc.get("IMODES30"), BIT10));   // reset IMU and PIPA fail bits
        agc.set("IMODES33", agc.get("IMODES33") + csMask(agc.get("IMODES33"), BIT13));
        pfailok2(BIT5);
    }

    /** NOATTOFF: subroutine to turn off the NO ATT lamp. */
    public void noattoff() {
        agc.set("DSPTAB+11", Agc.add(csMask(OCT40010, agc.get("DSPTAB+11")), BIT15));
    }

    // ---- PIPA use ----

    /** PIPUSE: initiate program use of the PIPAs (zeroes PIPAX..Z). */
    public void pipuse() {
        agc.set("PIPAX", NEG0);
        agc.set("PIPAY", NEG0);
        agc.set("PIPAZ", NEG0);
        pipuse1();
    }

    /** PIPUSE1. */
    public void pipuse1() {
        if (!cagetstq()) {
            new InterBankCommunication(agc).swreturn();   // IMU caged: do not enable PIPA fail
            return;
        }
        // INHINT
        agc.set("IMODES30", csMask(BIT1, agc.get("IMODES30")));   // if PIPA fails from now on, light ISS warning
        pipfree2();
    }

    /** PIPFREE2: ISS warning might come on now (or go off on PIPFREE). */
    public void pipfree2() {
        // IBNKCALL SETISSW
        modeexit();
    }

    /** PIPFREE: program done with PIPAs; do not light ISS warning. */
    public void pipfree() {
        // INHINT
        agc.set("IMODES30", agc.get("IMODES30") + csMask(agc.get("IMODES30"), BIT1));
        if ((agc.get("IMODES30") & BIT10) == 0) {          // PIP fail on: do program alarm and reset ISS warning
            new AlarmAndAbort(agc).alarm(0212);
            pipfree2();
            return;
        }
        modeexit();
    }

    // ---- IMUPULSE gyro torquing ----

    /**
     * IMUPULSE: torque the IRIGs according to double precision inputs in the six registers
     * beginning at the ECADR arriving in A. Minimum size of any pulse train is 16 pulses (.25 CDU
     * counts). The unsent portion is left intact in the input command registers.
     */
    public void imupulse(int ecadr) {
        agc.set("MPAC+5", ecadr);                       // save arriving ECADR
        if (!cagetstj()) {
            return;                                     // do not proceed if IMU being caged
        }
        if (Agc.toSigned(agc.get("LGYRO")) != 0) {
            gyrobusy();                                 // gyros busy: sleep
            return;
        }
        agc.set("MPAC+2", 0);
        wor("CHAN14", BIT6);                            // enable the power supply
        gwake2(4);
    }

    /** GWAKE2. */
    public void gwake2(int dt) {
        new Waitlist(agc).waitlist(dt, this::strtgyro);
        // set up EBANK, saving caller's EBANK for restoration on return
        int caller = agc.get("EBANK");
        agc.set("EBANK", agc.get("MPAC+5"));
        agc.set("MPAC+5", caller);
        agc.set("LGYRO", agc.get("EBANK"));             // reserves gyros
        int itemp1 = agc.get("LGYRO") & LOW8;
        agc.set("ITEMP1", itemp1);
        for (int i = 2; i >= 0; i--) {                  // GYROAGRE: force sign agreement on inputs
            // DCA 1400+2*i : DP command pair, then TPAGREE (sign agreement) and store back
            String hi = "E1400+" + (itemp1 + 2 * i);
            String lo = "E1400+" + (itemp1 + 2 * i + 1);
            int h = Agc.toSigned(agc.get(hi));
            int l = Agc.toSigned(agc.get(lo));
            if (h > 0 && l < 0) { l += 0x4000; h -= 1; }
            else if (h < 0 && l > 0) { l -= 0x4000; h += 1; }
            agc.set(hi, Agc.fromSigned(h));
            agc.set(lo, Agc.fromSigned(l));
        }
        agc.set("EBANK", agc.get("MPAC+5"));            // restore caller's EBANK
        modeexit();
    }

    /** GYROBUSY: sleep until the gyros are free (only one torquing job at a time). */
    public void gyrobusy() {
        // save return 2FCADR (BUF2) in MPAC
        regsleep();
    }

    /** REGSLEEP: CAF LGWAKE; TCF JOBSLEEP. */
    public void regsleep() {
        new Executive(agc).jobsleep(0 /* LGWAKE = CADR GWAKE */);
    }

    /** GWAKE: when awakened, see if gyros still busy. */
    public void gwake() {
        if (Agc.toSigned(agc.get("LGYRO")) != 0) {
            regsleep();                                 // sleep some more
            return;
        }
        agc.set("MPAC+2", 0);
        // restore SWRETURN info (DXCH BUF2)
        gwake2(1);
    }

    // ---- gyro waitlist tasks ----

    /** STRTGYRO: de-select last gyro then start the next. */
    public void strtgyro() {
        wand("CHAN14", ~GDESELCT & Agc.WORD_MASK);
        if (!cagetest()) {
            return;
        }
        strtgyr2();
    }

    /** STRTGYR2: jump on phase counter in LGYRO bits 13-14. */
    public void strtgyr2() {
        int phase = (agc.get("LGYRO") >> 11) & 3;       // MP BIT4 -> bits 13-14
        switch (phase) {
            case 0 -> gselect(0202);                    // do Y gyro
            case 1 -> gselect(0302);                    // do Z gyro
            case 2 -> {                                 // do X gyro (GSELECT -2: regress LGYRO for X)
                agc.set("LGYRO", Agc.add(agc.get("LGYRO"), Agc.fromSigned(-4)));
                gselect(0100);
            }
            default -> {                                // =3: done
                agc.set("LGYRO", 0);
                new Executive(agc).jobwake(0 /* LGWAKE */);   // wake a possibly sleeping job
                noreset();
            }
        }
    }

    /** NORESET: do not reset power supply. */
    public void noreset() {
        imufined();
    }

    /** GSELECT: select gyro (packed word: select bits + increment to LGYRO). */
    public void gselect(int packed) {
        int itemp4 = packed;
        agc.set("ITEMP4", itemp4);
        agc.set("LGYRO", Agc.add(agc.get("LGYRO"), (itemp4 & SEVEN) + BIT13));
        agc.set("EBANK", agc.get("LGYRO"));
        int itemp1 = agc.get("LGYRO") & LOW8;
        agc.set("ITEMP1", itemp1);
        itemp4 = csMask(SEVEN, itemp4);
        agc.set("ITEMP4", itemp4);
        // move DP command to RUPTREGs for testing
        String hi = "E1400+" + itemp1, lo = "E1400+" + (itemp1 + 1);
        agc.set("RUPTREG1", agc.get(hi));
        agc.set("RUPTREG2", agc.get(lo));
        int r1 = Agc.toSigned(agc.get("RUPTREG1"));
        int r2 = Agc.toSigned(agc.get("RUPTREG2"));
        if (r1 > 0) {
            maj(true);
        } else if (r1 < 0) {
            maj(false);
        } else if (r2 > 0) {
            if (r2 + MINUS_GYROMIN <= 0) strtgyr2(); else maj(true);   // MIN+
        } else if (r2 < 0) {
            if (-r2 + MINUS_GYROMIN <= 0) strtgyr2(); else maj(false); // MIN-
        } else {
            strtgyr2();
        }
    }

    /** MAJ+ / MAJ-: definite positive/negative output; GMERGE; torque pulses via GYROCMD and CHAN14. */
    private void maj(boolean positive) {
        int r1 = Agc.toSigned(agc.get("RUPTREG1"));
        int r2 = Agc.toSigned(agc.get("RUPTREG2"));
        int itemp4 = agc.get("ITEMP4");
        // add rounding fraction, select torque polarity
        long cmd = ((long) Math.abs(r1) << 14) | Math.abs(r2);
        wor("CHAN14", positive ? itemp4 : (itemp4 + BIT9));
        // pulses: command in gyro pulses (units of the DP word >> 7)
        int total = (int) (cmd >> 7);
        int itemp1 = agc.get("ITEMP1");
        String hi = "E1400+" + itemp1, lo = "E1400+" + (itemp1 + 1);
        if (total > 16383) {
            // LONGGYRO: initial command out plus N augments of 8192
            agc.set("GYROCMD", Agc.fromSigned(BIT14 + (total & 0x1FFF)));
            agc.set(hi, Agc.fromSigned((total >> 13) - 1));
            new Waitlist(agc).waitlist(Math.max(1, total / 100), this::aug8192);
            gyroexit();
            return;
        }
        agc.set(hi, 0);
        agc.set(lo, 0);
        lastseg(total);
    }

    /** LASTSEG: output the last segment; schedule STRTGYRO when the train is out. */
    private void lastseg(int cmd) {
        agc.set("GYROCMD", Agc.fromSigned(cmd));
        int dt = (cmd >> 6) + 3;                        // WAITLIST DT: pulse train time + truncation uncertainty
        new Waitlist(agc).waitlist(dt, this::strtgyro);
        gyroexit();
    }

    /** GYROEXIT: set the gyro activity bit. */
    public void gyroexit() {
        wor("CHAN14", BIT10);
        // TCF TASKOVER
    }

    /** 8192AUG: add 8192 pulses to GYROCMD. */
    public void aug8192() {
        if (!cagetest()) {
            return;
        }
        if ((ch("CHAN12") & BIT4) != 0) {
            imubad();
            return;
        }
        int itemp1 = agc.get("LGYRO") & LOW8;
        String hi = "E1400+" + itemp1;
        int remaining = Agc.toSigned(agc.get(hi));
        if (remaining > 0) {                            // AUG2: more to come
            agc.set(hi, Agc.fromSigned(remaining - 1));
            agc.set("GYROCMD", Agc.add(agc.get("GYROCMD"), BIT14));
            new Waitlist(agc).waitlist(Math.max(1, 8192 / 100 - 3), this::aug8192);   // AUG3: compute dt
            return;
        }
        agc.set("GYROCMD", Agc.add(agc.get("GYROCMD"), BIT14));
        lastseg(Agc.toSigned(agc.get("GYROCMD")));
    }

    // ---- ENDIMU and cage tests ----

    /** ENDIMU: mode is bad if cage has occurred or ISS warning is on. */
    public void endimu() {
        if ((ch("DSALMOUT") & BIT1) != 0) {
            imubad();
            return;
        }
        imugood();
    }

    /** IMUGOOD: TCF GOODEND with C(A) = 0. */
    public void imugood() {
        goodend(IMU);
    }

    /** IMUBAD: CAF ZERO; TCF BADEND. */
    public void imubad() {
        badend(IMU);
    }

    /** CAGETEST: terminate IMU mode switch if the IMU has been caged. Returns true to continue. */
    public boolean cagetest() {
        if ((agc.get("IMODES30") & BIT6) != 0) {
            imubad();
            return false;
        }
        return true;
    }

    /** CAGETSTQ: returns true (skips return address) if the IMU is NOT being caged. */
    public boolean cagetstq() {
        return (csMask(agc.get("IMODES30"), BIT6)) == 0 ? false : true;
    }

    /** CAGETSTJ: if IMU is being caged, set IMUCADR to -0 (operation complete but failed). Returns true if not caged. */
    public boolean cagetstj() {
        if (csMask(agc.get("IMODES30"), BIT6) != 0) {
            return true;
        }
        cagetstjFail();
        return false;
    }

    private void cagetstjFail() {
        agc.set("IMUCADR", NEG0);
        modeexit();
    }

    // ---- GOODEND / BADEND / STALL ----

    /** BADEND: unsuccessful end of an I/O operation; device index 0 = IMU, 1 = optics. */
    public void badend(int device) {
        agc.set("RUPTREG2", device);
        endMode(NEG0);                        // CS ZERO for failure
    }

    /** GOODEND: successful end; device index 0 = IMU, 1 = optics. */
    public void goodend(int device) {
        agc.set("RUPTREG2", device);
        endMode(0x7FFE);                      // CS ONE for success (-1)
    }

    private void endMode(int resultCode) {
        agc.set("RUPTREG3", resultCode);
        String mc = modecadrOf(agc.get("RUPTREG2"));
        int m = Agc.toSigned(agc.get(mc));
        if (m == 0 && agc.get(mc) == 0) {
            agc.set(mc, resultCode);          // ENDMODE: -0 = complete but unsuccessful, -1 = complete and successful
            return;                           // TASKOVER
        }
        if (agc.get(mc) == NEG0 || agc.get(mc) == 0x7FFE) {
            agc.set(mc, resultCode);
            return;
        }
        agc.set(mc, 0);                       // wake sleeping program
        new Executive(agc).jobwake(m);
        // advance LOC if successful
        if (resultCode == 0x7FFE) {
            agc.set("LOC", Agc.add(agc.get("LOC"), 1));
        }
    }

    private static String modecadrOf(int device) {
        return device == 0 ? "MODECADR" : "MODECADR+" + device;
    }

    /** AOTSTALL: stall for AOT. */
    public void aotstall() {
        stall(OPTICS_AOT);
    }

    /** RADSTALL: stall for radar. */
    public void radstall() {
        stall(RADAR);
    }

    /** OPTSTALL = AOTSTALL. */
    public void optstall() {
        aotstall();
    }

    /** IMUSTALL: stall for IMU. */
    public void imustall() {
        stall(IMU);
    }

    /**
     * STALL: general stalling routine. Using programs wait here for I/O completion. MODECADR of the
     * device: +0 incomplete (job put to sleep), -1 complete good (return to L+3), -0 complete failed
     * (clears MODECADR and returns at L+2), other (positive CADR) = two programs using the same
     * device (POODOO 1210).
     */
    public void stall(int device) {
        // INHINT
        agc.set("RUPTREG2", device);
        String mc = modecadrOf(device);
        int raw = agc.get(mc);
        if (raw == NEG0) {                       // complete failed: reset to +0
            agc.set(mc, 0);
            modeexit();
        } else if (raw == 0) {                   // operation incomplete: sleep
            agc.set(mc, new InterBankCommunication(agc).makecadr());
            new Executive(agc).jobsleep();
        } else if (raw == 0x7FFE) {              // complete and good (-1): increment return, set CADR = +0
            agc.set("BUF2", Agc.add(agc.get("BUF2"), 1));
            agc.set(mc, 0);
            modeexit();
        } else {
            new AlarmAndAbort(agc).poodoo(01210); // two programs using the same device
        }
    }

    /** GOMANUR: start KALCMANU if free; else POODOO 1210 (two trying to use same device). */
    public void gomanur() {
        if (agc.get("ATTCADR") != 0) {
            new AlarmAndAbort(agc).poodoo(01210);
            return;
        }
        // DXCH ATTCADR: save final return for KALCMAN3
        agc.set("ATTCADR", agc.get("BUF2"));
        agc.set("ATTCADR+1", agc.get("BUF2+1"));
        agc.set("ATTCADR+1", agc.get("ATTCADR+1") + (agc.get("BBANK") & SEVEN));
        agc.set("ATTPRIO", agc.get("PRIORITY") & 037000);   // save user's priority (PRIO37)
        agc.set("EBANK", agc.get("BCDU"));                  // KALEBCON = ECADR BCDU: set EBANK for KALCMAN3
        new Anglfind(agc).kalcman3();                        // POSTJUMP KALCMAN3
    }

    // ---- R02: IMU status check ----

    /** R02BOTH: check IMU on and aligned to an orientation known by the CMC; else alarm and go to P00H. */
    public void r02both() {
        if ((agc.get("STATE+3") & BIT13) != 0) {            // REFSMFLG
            r02zero();
            return;
        }
        int code = OCT220;
        if ((agc.get("IMODES30") & BIT9) == 0) {            // ISS not initialized
            code = Agc.add(~BIT4 & Agc.WORD_MASK, OCT220);  // send IMU alarm code 210
        }
        // TC VARALARM; TC GOTOPOOH
        new AlarmAndAbort(agc).priolarm(code);
    }

    /** R02ZERO: set IMUSE flag and return. */
    public void r02zero() {
        agc.set("STATE+IMUSE", 1);                          // UPFLAG IMUSE
        new InterBankCommunication(agc).swreturn();
    }

    // ---- P06: standby ----

    /** P06: transfer the ISS/CMC from operate to standby; sets NODOV37 (NODOFLAG). */
    public void p06() {
        agc.set("NODOFLAG", 1);                             // UPFLAG NODOFLAG
        prestand();
    }

    /** PRESTAND: snapshot the scaler and TIME1/TIME2 and prepare for standby. */
    public void prestand() {
        // INHINT
        agc.set("TIME2SAV", agc.get("TIME2"));              // snapshot TIME1 TIME2 (DP)
        agc.set("TIME2SAV+1", agc.get("TIME1"));
        if (!scalprep()) {
            prestand();                                     // T1, T2, scaler not compatible
            return;
        }
        agc.set("SCALSAVE", agc.get("MPAC"));               // store scaler
        agc.set("SCALSAVE+1", agc.get("MPAC+1"));
        // INHINT; BANKCALL RNDREFDR: REFSMMAT, drift, track flags down
        rndrefdr();
        agc.set("IMUSE", 0);                                // IMUSE down
        agc.set("RNDVZFLG", 0);                             // RNDVZFLG down
        wor("CHAN13", BIT11);                               // set standby enable bit
        // PHASCHNG: set restart to POSTAND when standby recovers (phase 07024, 20000)
        // BANKCALL GOPERF1 with OCT62 (.5 sec); the three TCFs after it re-enter PRESTAND area
        // GOPERF1 returns to L+1/L+2/L+3: -3 = PRESTAND, -4,-5 = loop
    }

    /**
     * SCALPREP: the low 5 bits of the scaler read 10000 for the first interval after a T1 increment.
     * Returns false (return to L+1: re-read data) if that interval is detected, true (L+2) otherwise.
     * Adds 5 ms to the scaler reading and truncates it to 10 ms; result left in MPAC, MPAC+1.
     */
    public boolean scalprep() {
        // TC FINETIME +1 : read scaler into MPAC, MPAC+1
        int hi = agc.get("MPAC");
        int lo = agc.get("MPAC+1");
        lo = Agc.add(lo, BIT5);                             // add 5 ms to the scaler reading
        if (lo > 0x3FFF) { /* DAS carry into high word is handled by Agc.add */ }
        int low5 = lo & LOW5;
        agc.set("MPAC+1", lo & ~LOW5 & Agc.WORD_MASK);      // set low 5 bits to zero
        agc.set("MPAC", hi);
        return low5 != 0;                                   // 0 -> first interval after T1 increment
    }

    /** POSTAND: recover time after standby. */
    public void postand() {
        wand("CHAN13", ~BIT11 & Agc.WORD_MASK);             // clear standby enable bit
        // INHINT
        agc.set("TIME2", 0);                                // clear TIME1 TIME2
        agc.set("TIME1", 0);
        if (!scalprep()) {
            postand();                                      // T1,T2,scaler not compatible: retry
            return;
        }
        // DP difference of post-standby scaler minus pre-standby scaler, shifted right 5
        long post = ((long) agc.get("MPAC") << 14) | agc.get("MPAC+1");
        long pre = ((long) agc.get("SCALSAVE") << 14) | agc.get("SCALSAVE+1");
        long diff = (post - pre) >> 5;
        if (diff < 0) {
            diff += (long) BIT10 << 14;                     // scaler overflowed: add bit 10 to high diff
        }
        long saved = ((long) agc.get("TIME2SAV") << 14) | agc.get("TIME2SAV+1");
        long updated = diff + saved;                        // updated value for T1,T2
        agc.set("TIME2", (int) ((updated >> 14) & Agc.WORD_MASK));
        agc.set("TIME1", (int) (updated & 0x3FFF));
        agc.set("NODOFLAG", 0);                             // clear NODOFLAG
        // TC GOTOPOOH
    }
}
