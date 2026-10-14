package apollo.luminary099;

import apollo.agc.Agc;

/**
 * IMU_MODE_SWITCHING_ROUTINES.agc - IMU zeroing, coarse align, fine align, PIPA use/free,
 * gyro torquing (IMUPULSE / STRTGYRO), mode-switch termination (GOODEND/BADEND), the STALL
 * routines, GOMANUR, R02 (IMU status check) and P06 (standby / POSTAND).
 *
 * I/O channels are agc symbols CHAN5, CHAN12, CHAN13, CHAN14, CHAN6; bit n is {@code 1 << (n-1)}.
 * Waitlist tasks are passed as Runnables to {@link #waitlist}. The device index is
 * 0 = IMU, 1 = AOT/optics, 2 = radar; MODECADR holds +0 (incomplete), -1 (good), -0 (failed).
 */
public class ImuModeSwitchingRoutines {
    private final Agc agc;

    // constants
    public static final int COMMAX_MINUS = -191;          // -COMMAX: max pulses allowed minus one
    public static final int COMMAX_MINUS_ONE_MORE = -192; // -COMMAX-
    public static final int MS600 = 60;                   // 600MS (centiseconds)
    public static final int COARSTOL_COUNTS = -(int) Math.round(0.01111 * 16384); // -2 degrees at half revs
    public static final int BITS_3_4 = 0x0C;              // OCT14
    public static final int BITS_4_6 = 0x28;              // OCT50
    public static final int BITS_4_5 = 0x18;              // OCT30
    public static final int OCT40010 = 0x4008;
    public static final int GDESELCT = 0x3C0;             // OCT 1700: turn off select and activity bits
    public static final int GYROMIN_NEG = -126;           // -GYROMIN: OCT -176 (minimum command)
    public static final int OCT220 = 0x90;
    public static final int IMUSEFLG = 0x80;              // BIT8: interpreter switch 7
    public static final int OCT62 = 50;                   // .5SEC
    /** GYROFRAC: 2DEC .215 B-21, fractional-pulse adjustment per commanded pulse train. */
    public static final double GYROFRAC = 0.215 / (1 << 21);
    /** DSPTAB+11D lamp word. */
    private static final String DSPTAB11 = "DSPTAB+11D";

    // alarm codes
    public static final int ALARM_GIMBAL_LOCK_ZERO = 0206;
    public static final int ALARM_IMU_NOT_OPERATING = 0210;
    public static final int ALARM_COARSE_ALIGN_ERROR = 0211;
    public static final int ALARM_PIP_FAIL = 0212;

    // task queue used by the waitlist model
    private final java.util.ArrayDeque<Runnable> pending = new java.util.ArrayDeque<>();

    public ImuModeSwitchingRoutines(Agc agc) {
        this.agc = agc;
    }

    private static int bit(int n) { return 1 << (n - 1); }
    private int g(String s) { return agc.get(s); }
    private void or(String s, int m) { agc.set(s, agc.get(s) | m); }
    private void andNot(String s, int m) { agc.set(s, agc.get(s) & ~m); }
    private void waitlist(int dt, Runnable task) { pending.add(task); agc.set("WAITDT", dt); }
    private void alarm(int code) { agc.set("ALMCODE", code); }

    /** Runs pending waitlist tasks (test driver). */
    public void runTasks() {
        while (!pending.isEmpty()) {
            pending.poll().run();
        }
    }

    // ===== fixed-fixed =====

    /** ZEROICDU: zero the CDU counters. */
    public void zeroIcdu() {
        agc.set("CDUX", 0);
        agc.set("CDUY", 0);
        agc.set("CDUZ", 0);
    }

    // ===== IMU zeroing =====

    /** IMUZERO: zero the ICDUs (protected against gimbal lock in coarse align). */
    public void imuZero() {
        if ((~g(DSPTAB11) & BITS_4_6 & Agc.WORD_MASK) == 0) {
            alarm(ALARM_GIMBAL_LOCK_ZERO);              // IMUZEROA not taken: gimbal-lock/coarse-align
            cagetStJ(true);                             // immediate failure (CAGETSTJ +4)
            return;
        }
        if (cagetStJ(false)) return;
        // disable DAP auto/hold modes, inhibit ICDUFAIL and IMUFAIL
        or("IMODES33", bit(5) | bit(7) | bit(8) | bit(9)); // SUPER011 mask bits
        or("IMODES30", BITS_3_4);
        andNot("CHAN12", BITS_4_6);                     // zero encode with coarse/error counter disabled
        noAttOff();
        or("CHAN12", bit(5));                           // ZERO discrete
        zeroIcdu();
        waitlist(bit(6), this::imuZero2);               // wait 320 ms for AGS pulse train
        if ((g("IMODES30") & bit(9)) == 0) {            // IMU operating?
            alarm(ALARM_IMU_NOT_OPERATING);
        }
        modeExit();
    }

    /** MODEEXIT: general mode-switching exit: RELINT; TCF SWRETURN. */
    public void modeExit() {
        // RELINT ; return to caller (SWRETURN)
    }

    /** IMUZERO2: waitlist task, 320 ms after zeroing started. */
    public void imuZero2() {
        if (cageTest()) return;
        zeroIcdu();
        andNot("CHAN12", bit(5));                       // remove ZERO discrete
        varDelay(bit(11), this::imuZero3);              // wait 10 s for counters to find gimbals
    }

    private void varDelay(int dt, Runnable then) { waitlist(dt, then); }

    /** IMUZERO3: re-enable failure monitors and DAP modes. */
    public void imuZero3() {
        if (cageTest()) return;
        andNot("IMODES30", BITS_3_4);                   // remove IMUFAIL and ICDUFAIL inhibit
        andNot("IMODES33", bit(5) | bit(7) | bit(8) | bit(9));
        new SetIssw(agc).setIssw();                     // IBNKCALL SETISSW
        endImu();
    }

    // ===== coarse align =====

    /** IMUCOARS: IMU coarse align mode switch. */
    public void imuCoars() {
        if (cagetStJ(false)) return;
        setCoars();
        waitlist(6, this::coars);
        modeExit();
    }

    /** COARS: compute pulse commands (THETAD - THETAA) for each CDU, enable error counters. */
    public void coars() {
        if (cageTest()) return;
        or("CHAN12", bit(6));                           // enable all three ISS CDU error counters
        String[] cdu = {"CDUX", "CDUY", "CDUZ"};
        String[] thd = {"THETAD", "THETAD+1", "THETAD+2"};
        String[] cmd = {"COMMAND", "COMMAND+1", "COMMAND+2"};
        for (int i = 2; i >= 0; i--) {                  // COARS1
            int diff = Agc.add(g(thd[i]), Agc.fromSigned(-Agc.toSigned(g(cdu[i])))); // MSU
            int shifted = Agc.toSigned(diff) / 4;       // MP BIT13: shift right 2 (rounded via L)
            int rem = Agc.toSigned(diff) % 4;
            int rounded = shifted + (Math.abs(rem * 2) >= 4 ? Integer.signum(rem) : 0);
            agc.set(cmd[i], Agc.fromSigned(rounded * 2)); // DOUBLE: pulses (in CDU counts x 2)
        }
        varDelay(2, this::coars2);                      // minimum 4 ms wait
    }

    /** COARS2: send up to COMMAX pulses per axis; repeat until the command is exhausted. */
    public void coars2() {
        if (cageTest()) return;
        int pulsesOut = 0;
        String[] cmd = {"COMMAND", "COMMAND+1", "COMMAND+2"};
        String[] out = {"CDUXCMD", "CDUYCMD", "CDUZCMD"};
        for (int i = 2; i >= 0; i--) {
            int c = Agc.toSigned(g(cmd[i]));
            int sent;
            if (c > 0) {                                // COMPOS
                if (c + COMMAX_MINUS <= 0) { sent = c; c = 0; }             // COMZERO
                else { sent = -COMMAX_MINUS_ONE_MORE; c += COMMAX_MINUS; }
            } else if (c < 0) {                         // COMNEG
                if (-c + COMMAX_MINUS <= 0) { sent = c; c = 0; }
                else { sent = COMMAX_MINUS_ONE_MORE; c += -COMMAX_MINUS; }
            } else { sent = 0; }
            agc.set(cmd[i], Agc.fromSigned(c));
            agc.set(out[i], Agc.fromSigned(sent));      // NEXTCDU: set up command register
            if (sent != 0) pulsesOut++;
        }
        if (pulsesOut > 0) {                            // SENDPULS
            or("CHAN14", bit(13) | bit(14) | bit(15));  // 13,14,15
            varDelay(MS600, this::coars2);              // then to VARDELAY, TCF COARS2 -1
            return;
        }
        fixDelay(150, this::chkCors);                   // wait for gimbals to settle
    }

    private void fixDelay(int dt, Runnable then) { waitlist(dt, then); }

    /** CHKCORS: gimbals must be within 2 degrees of THETAD. */
    public void chkCors() {
        String[] cdu = {"CDUX", "CDUY", "CDUZ"};
        String[] thd = {"THETAD", "THETAD+1", "THETAD+2"};
        for (int i = 2; i >= 0; i--) {
            int err = Math.abs(Agc.toSigned(Agc.add(g(cdu[i]), Agc.fromSigned(-Agc.toSigned(g(thd[i]))))));
            if (err + COARSTOL_COUNTS > 0) {            // COARSERR
                alarm(ALARM_COARSE_ALIGN_ERROR);
                imuBad();
                return;
            }
        }
        endImu();                                       // end of coarse alignment
    }

    /** CA+ECE: enable all three ISS CDU error counters (waitlist task). */
    public void caEce() {
        or("CHAN12", bit(6));
    }

    /** SETCOARS: put the ISS into coarse align (no-op if already there). */
    public void setCoars() {
        if ((g("CHAN12") & bit(4)) != 0) return;        // already in coarse align
        andNot("CHAN12", bit(6));                       // clear ISS error counters
        andNot("CHAN14", bit(10));                      // knock down gyro activity
        agc.set("GYROCMD", Agc.WORD_MASK);              // CS ZERO: -0
        or("CHAN12", bit(4));                           // ISS in coarse align
        or(DSPTAB11, OCT40010);                         // turn on NO ATT lamp
        or("IMODES33", bit(6));                         // disable DAP auto and hold
        or("IMODES30", bit(4));                         // disable IMUFAIL
        rndRefDr();
    }

    /** RNDREFDR: clear TRACK, DRIFT and REFSMMAT flags. */
    public void rndRefDr() {
        andNot("FLAGWRD1", bit(7));                     // TRACKBIT
        andNot("FLAGWRD2", bit(9));                     // DRFTBIT
        andNot("FLAGWRD3", bit(13));                    // REFSMBIT
    }

    // ===== fine align =====

    /** IMUFINE: IMU fine align mode switch (also IMUFIN20). */
    public void imuFine() {
        if (cagetStJ(false)) return;
        andNot("CHAN12", BITS_4_5);                     // reset ZERO and COARSE
        andNot("IMODES33", bit(6));                     // ensure DAP auto and hold enabled
        noAttOff();
        waitlist(bit(10), this::ifailOk);               // keep IMUFAIL inhibit for 5 s
        waitlist(200, this::imuFined);                  // 2SECS
        modeExit();
    }

    /** IMUFINED: waitlist task; check for caging then signal completion. */
    public void imuFined() {
        if (cageTest()) return;
        endImu();
    }

    /** IFAILOK: enable IMUFAIL unless the IMU is being caged or in coarse align. */
    public void ifailOk() {
        if (cagetStQ()) return;                         // being caged: TASKOVER
        if ((g("CHAN12") & bit(4)) != 0) return;        // someone is in coarse align
        or("IMODES30", bit(13));                        // CS IMODES30 MASK BIT13 ADS: reset IMUFAIL
        pfailOk2(bit(4));
    }

    private void pfailOk2(int clear) {
        andNot("IMODES30", clear);
        new SetIssw(agc).setIssw();                     // ISS warning may come on now
    }

    /** PFAILOK: enable the PIP FAIL program alarm; reset IMU and PIPA fail bits. */
    public void pfailOk() {
        if (cagetStQ()) return;
        or("IMODES30", bit(10));
        or("IMODES33", bit(13));
        pfailOk2(bit(5));
    }

    /** NOATTOFF: turn off the NO ATT lamp. */
    public void noAttOff() {
        agc.set(DSPTAB11, ((~OCT40010 & g(DSPTAB11)) + bit(15)) & Agc.WORD_MASK);
    }

    // ===== PIPA use =====

    /** PIPUSE: zero PIPAX/Y/Z then PIPUSE1. */
    public void pipUse() {
        agc.set("PIPAX", Agc.WORD_MASK);
        agc.set("PIPAY", Agc.WORD_MASK);
        agc.set("PIPAZ", Agc.WORD_MASK);
        pipUse1();
    }

    /** PIPUSE1: from now until PIPFREE, a PIPA failure lights the ISS warning. */
    public void pipUse1() {
        if (cagetStQ()) return;
        andNot("IMODES30", bit(1));
        pipFree2();
    }

    private void pipFree2() {
        new SetIssw(agc).setIssw();
        modeExit();
    }

    /** PIPFREE: program done with the PIPAs. */
    public void pipFree() {
        or("IMODES30", bit(1));
        if ((g("IMODES30") & bit(10)) != 0) {           // PIP fail on: alarm and reset ISS warning
            alarm(ALARM_PIP_FAIL);
            pipFree2();
            return;
        }
        modeExit();
    }

    // ===== gyro torquing =====

    private int lgyro() { return g("LGYRO"); }

    /**
     * IMUPULSE: torque the IRIGs per the six DP words at the ecadr given. Minimum train is 16 pulses
     * (.25 CDU counts); unsent portion remains in the command registers.
     * @param ecadr base symbol of the 6 command words (X,Y,Z each DP)
     */
    public void imuPulse(String ecadr) {
        if (cagetStJ(false)) return;
        if (lgyro() != 0) {
            gyroBusy(ecadr);                            // another job is torquing: sleep
            return;
        }
        gWake2(ecadr);
    }

    private String gyroCommands;

    private void gWake2(String ecadr) {
        or("CHAN14", bit(6));                           // enable the power supply
        waitlist(4, this::strtGyro);
        gyroCommands = ecadr;
        agc.set("LGYRO", 1);                            // reserves gyros (EBANK of command registers)
        // force sign agreement on inputs (TPAGREE) for X, Y, Z
        for (int i = 0; i < 3; i++) {
            String hi = ecadr + (i == 0 ? "" : "+" + (2 * i));
            long v = VecOps.getDpRaw(agc, hi);
            VecOps.setDpRaw(agc, hi, v);
        }
        modeExit();
    }

    /** GYROBUSY / REGSLEEP / GWAKE: only one job may torque at a time. */
    private void gyroBusy(String ecadr) {
        pending.add(() -> gWake(ecadr));
    }

    /** GWAKE: when awakened, check whether the gyros are still busy. */
    public void gWake(String ecadr) {
        if (lgyro() != 0) {
            gyroBusy(ecadr);                            // REGSLEEP
            return;
        }
        agc.set("LGYRO", 0);
        gWake2(ecadr);
    }

    /** STRTGYRO: waitlist task: deselect last gyro, then do the next. */
    public void strtGyro() {
        andNot("CHAN14", GDESELCT);
        if (cageTest()) return;
        strtGyr2();
    }

    /** STRTGYR2: phase counter in bits 13-14 of LGYRO selects Y(0), Z(1), X(2) or done (3). */
    public void strtGyr2() {
        int phase = (lgyro() >> 12) & 3;
        switch (phase) {
            case 0: gSelect(0x082, 1); break;           // OCT 00202: Y gyro
            case 1: gSelect(0x0C2, 2); break;           // OCT 00302: Z gyro
            case 2:                                     // X gyro: special entry regresses LGYRO
                agc.set("LGYRO", Agc.add(lgyro(), Agc.fromSigned(-4)));
                gSelect(0x040, 0);                      // OCT 00100
                break;
            default:                                    // =3: DONE
                agc.set("LGYRO", 0);
                // wake a possible sleeping job (JOBWAKE LGWAKE)
                break;
        }
    }

    /** NORESET: TCF IMUFINED. */
    public void noReset() {
        imuFined();
    }

    /** GSELECT: pick the gyro, examine the command and start a pulse train. */
    private void gSelect(int packed, int axis) {
        int sel = packed & ~7;
        agc.set("LGYRO", lgyro() + (packed & 7) + bit(13));
        String cmd = gyroCommands + (axis == 0 ? "" : "+" + (2 * axis));
        long v = VecOps.getDpRaw(agc, cmd);
        if (v == 0) {                                   // zero: go on to the next gyro
            strtGyr2();
            return;
        }
        int ruptreg1 = (int) (v / VecOps.HALF_RANGE);
        int ruptreg2 = (int) (v % VecOps.HALF_RANGE);
        boolean positive = v > 0;
        if (positive && ruptreg1 == 0 && ruptreg2 + GYROMIN_NEG <= 0) { strtGyr2(); return; } // MIN+
        if (!positive && ruptreg1 == 0 && -ruptreg2 + GYROMIN_NEG <= 0) { strtGyr2(); return; } // MIN-
        // MAJ+ / MAJ-: add GYROFRAC rounding; negative commands select the negative sign bit (BIT9)
        long mag = Math.abs(v) + Math.round(GYROFRAC * (1L << 28));
        or("CHAN14", sel | (positive ? 0 : bit(9)));
        long pulses = mag >> 14;                        // GMERGE: scale to gyro pulses
        long remaining = pulses - 16383;
        if (remaining > 0) {                            // LONGGYRO: more than one pulse train
            VecOps.setDpRaw(agc, cmd, (positive ? 1 : -1) * (remaining << 14));
            agc.set("GYROCMD", Agc.fromSigned(16383));
            aug3(16383);
            return;
        }
        VecOps.setDpRaw(agc, cmd, 0);
        lastSeg((int) pulses);
    }

    /** LASTSEG: output the last segment; schedule the next gyro. */
    private void lastSeg(int pulses) {
        agc.set("GYROCMD", Agc.fromSigned(pulses));
        waitlist(pulses / 64 + 3, this::strtGyro);       // MP BIT10 + 3 (truncation/phase uncertainty)
        gyroExit();
    }

    private void gyroExit() {
        or("CHAN14", bit(10));                          // activity bit starts the train
    }

    private void aug3(int pulses) {
        waitlist(pulses / 64 - 3, this::aug8192);
        gyroExit();
    }

    /** 8192AUG: add 8192 pulses to GYROCMD while a long command remains. */
    public void aug8192() {
        if (cageTest()) return;
        if ((g("CHAN12") & bit(4)) != 0) {              // coarse align going: bad
            imuBad();
            return;
        }
        long remaining = VecOps.getDpRaw(agc, gyroCommands) >> 14;
        agc.set("GYROCMD", Agc.add(g("GYROCMD"), bit(14)));
        if (remaining > 0) {                            // AUG2: more to come
            aug3(16383);
        } else {
            lastSeg(Agc.toSigned(g("GYROCMD")));        // last aug: LASTSEG +1
        }
    }

    // ===== completion =====

    /** ENDIMU: mode is bad if cage occurred or ISS warning is on. */
    public void endImu() {
        if ((g("DSALMOUT") & bit(1)) != 0) {
            imuBad();
        } else {
            imuGood();
        }
    }

    public void imuGood() { goodEnd(0); }

    public void imuBad() { badEnd(0); }

    /** CAGETEST: terminates the mode switch if the IMU has been caged. @return true if terminated. */
    public boolean cageTest() {
        if ((g("IMODES30") & bit(6)) != 0) {
            imuBad();
            return true;
        }
        return false;
    }

    /** CAGETSTQ: @return true if the IMU is NOT being caged (skip-return form). */
    public boolean cagetStQ() {
        return (g("IMODES30") & bit(6)) == 0;
    }

    /**
     * CAGETSTJ: if the IMU is being caged, set IMUCADR to -0 and return via MODEEXIT.
     * @param force true for the "immediate failure" entry (CAGETSTJ +4)
     * @return true if the caller must abandon its operation
     */
    public boolean cagetStJ(boolean force) {
        if (!force && (g("IMODES30") & bit(6)) == 0) return false;
        agc.set("IMUCADR", Agc.WORD_MASK);              // -0: operation complete but failed
        modeExit();
        return true;
    }

    /** BADEND: unsuccessful end for device index (0 = IMU, 1 = optics). */
    public void badEnd(int device) {
        finish(device, false);
    }

    /** GOODEND: successful end for device index. */
    public void goodEnd(int device) {
        finish(device, true);
    }

    private void finish(int device, boolean success) {
        String modecadr = device == 0 ? "MODECADR" : "MODECADR+" + device;
        int m = g(modecadr);
        if (m != 0 && m != Agc.WORD_MASK) {             // program asleep: wake it
            agc.set(modecadr, 0);
            if (success) {
                agc.set("LOC", g("LOC") + 1);           // advance LOC if successful
            }
            // TC JOBWAKE ; TCF TASKOVER
            return;
        }
        // ENDMODE: -0 = complete but failed, -1 = complete and successful
        agc.set(modecadr, success ? Agc.fromSigned(-1) : Agc.WORD_MASK);
    }

    // ===== STALL routines =====

    public void aotStall() { stall(1); }
    public void radStall() { stall(2); }
    public void optStall() { stall(1); }
    public void imuStall() { stall(0); }

    /**
     * STALL: wait for I/O completion. MODECADR: +0 incomplete (sleep), -1 good (return L+3),
     * -0 failed (return L+2), otherwise abort 1210 (two programs using the same device).
     */
    public void stall(int device) {
        String modecadr = device == 0 ? "MODECADR" : "MODECADR+" + device;
        int m = Agc.toSigned(g(modecadr));
        boolean negZero = g(modecadr) == Agc.WORD_MASK;
        if (negZero) {
            agc.set(modecadr, 0);                       // MG2: reset to +0
            modeExit();
        } else if (m == 0) {
            agc.set(modecadr, 1);                       // MODESLP: record caller CADR; JOBSLEEP
        } else if (m == -1) {
            agc.set("BUF2", g("BUF2") + 1);             // MODEGOOD: increment return address
            agc.set(modecadr, 0);
            modeExit();
        } else {
            alarm(01210);                               // MODABORT -> BAILOUT1
            throw new IllegalStateException("BAILOUT 1210");
        }
    }

    /** GOMANUR: start KALCMANU (KALCMAN3) from a mode-switching caller. */
    public void goManur() {
        if (g("ATTCADR") != 0) {
            alarm(01210);                               // two trying to use the same device
            return;
        }
        agc.set("ATTCADR", g("BUF2"));                  // save final return for KALCMAN3
        agc.set("ATTPRIO", g("PRIORITY") & 0x1F);       // save user's priority (PRIO37)
        agc.set("EBANK", g("BCDU"));                    // KALEBCON
        // TC POSTJUMP ; CADR KALCMAN3
    }

    // ===== R02 =====

    /** R02BOTH: IMU status check - IMU on and aligned to a known orientation. */
    public void r02Both() {
        if ((g("FLAGWRD3") & bit(13)) == 0) {           // REFSMBIT clear
            // A = IMODES30 bit 9 set ? OCT220 : OCT220 - BIT4 (IMU alarm code 210)
            int code = (g("IMODES30") & bit(9)) != 0 ? OCT220 : OCT220 - bit(4);
            alarm(code);                                // TC VARALARM ; TC GOTOPOOH
            return;
        }
        // R02ZERO: UPFLAG IMUSE ; return
        new ServiceRoutines(agc).upFlag("FLAGWRD3", 12);
    }

    // ===== P06 standby =====

    /** P06: transfer the ISS/CMC from operate to standby. */
    public void p06() {
        new ServiceRoutines(agc).upFlag("FLAGWRD8", 1);  // NODOFLAG
        preStand();
    }

    /** PRESTAND: snapshot TIME1/TIME2 and the scaler, drop flags, set STANDBY enable. */
    public void preStand() {
        long time = VecOps.getDpRaw(agc, "TIME2");
        VecOps.setDpRaw(agc, "TIME2SAV", time);
        long scaler = scalPrep();
        VecOps.setDpRaw(agc, "SCALSAVE", scaler);
        rndRefDr();
        ServiceRoutines sr = new ServiceRoutines(agc);
        sr.downFlag("FLAGWRD3", 12);                    // IMUSE down
        sr.downFlag("FLAGWRD3", 11);                    // RNDVZFLG down
        or("CHAN13", bit(11));                          // set STANDBY enable bit
        // PHASCHNG: restart to POSTAND ; GOPERF1 (OCT62 = .5 SEC)
    }

    /** SCALPREP: scaler reading adjusted 5 ms and truncated to 10 ms; DP result. */
    private long scalPrep() {
        long scaler = VecOps.getDpRaw(agc, "SCALER") + bit(5);
        return scaler & ~0x1FL;                          // zero low 5 bits
    }

    /** POSTAND: recover time after STANDBY by adding the elapsed scaler difference. */
    public void postAnd() {
        andNot("CHAN13", bit(11));                      // clear STANDBY enable bit
        VecOps.setDpRaw(agc, "TIME2", 0);
        long scaler = scalPrep();
        long diff = scaler - VecOps.getDpRaw(agc, "SCALSAVE");
        diff >>= 5;                                      // align bits with TIME1/TIME2
        if (diff < 0) {
            diff += bit(10) * VecOps.HALF_RANGE;        // scaler overflowed between pre/post
        }
        diff += VecOps.getDpRaw(agc, "TIME2SAV");
        VecOps.setDpRaw(agc, "TIME2", VecOps.getDpRaw(agc, "TIME2") + diff);
        new ServiceRoutines(agc).downFlag("FLAGWRD8", 1); // clear NODOFLAG
        // TC GOTOPOOH
    }

    /** Forwarding stub for the ISS-warning routine (SETISSW lives in another file). */
    private static final class SetIssw { /* delegates to T4rupt SETISSW */
        private final Agc agc;
        SetIssw(Agc agc) { this.agc = agc; }
        void setIssw() { new T4RuptProgram(agc).setIssw(); }
    }
}
