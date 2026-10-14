package apollo.luminary099;

import apollo.agc.Agc;

/**
 * LANDING_ANALOG_DISPLAYS (Luminary 099): routine R10, drives the LM altitude / altitude-rate
 * meter and the forward / lateral velocity pointers during descent. Runs as a T4 (waitlist)
 * task four times a second. All words are signed integers (see {@link Agc#toSigned}).
 */
public class LandingAnalogDisplays {
    /** BITS8/7: inertial data and interleave flags (OCT 00300). */
    public static final int BITS8_7 = 0300;
    private static final int BIT2 = 02, BIT3 = 04, BIT4 = 010, BIT6 = 040, BIT7 = 0100,
            BIT8 = 0200, BIT10 = 01000, BIT15 = 040000;
    private static final int POSMAX = 037777;
    private static final int NEG0 = 077777;

    private final Agc agc;
    /** Hook for channel writes (WOR/WAND/RAND on I/O channels) and executive services. */
    public interface Io {
        int readChannel(int chan);
        void writeChannel(int chan, int value);
        void taskOver();
        void varDelay(int count);
        void twiddle(Runnable r, int bit);
    }
    private Io io;

    public LandingAnalogDisplays(Agc agc) {
        this.agc = agc;
    }

    public void setIo(Io io) {
        this.io = io;
    }

    private int g(String s) {
        return Agc.toSigned(agc.get(s));
    }

    private void s(String s, int v) {
        agc.set(s, Agc.fromSigned(v));
    }

    private static int mp(int a, int b) {
        return (int) (((long) a * b) / 16384);
    }

    private void wor(int ch, int mask) {
        io.writeChannel(ch, io.readChannel(ch) | mask);
    }

    private void wand(int ch, int mask) {
        io.writeChannel(ch, io.readChannel(ch) & mask);
    }

    /** LANDISP: T4 entry, update TBASE2/PIPCTR1, select ALTITUDE or ALTRATE output. */
    public void landisp() {
        // LXCH PIPCTR1; CS TIME1; DXCH TBASE2  (simultaneous update of TBASE2 and PIPCTR)
        s("TBASE2", -g("TIME1"));
        if ((agc.get("FLAGWRD7") & agc.get("SWANDBIT")) == 0) {
            dispRset(); // landing analog displays flag not set
            return;
        }
        if ((agc.get("IMODES33") & BIT7) != 0) {
            altout();
        } else {
            altrout();
        }
    }

    /** ALTROUT: output altitude rate. Returns after the ALTM register is loaded. */
    public void altrout() {
        if (!disindat()) {
            return;
        }
        agc.set("IMODES33", agc.get("IMODES33") | BIT7); // alternate ALTRATE with ALT
        wor(014, BIT2); // CHAN14 bit2: ALTRATE (1) / ALTITUDE (0)
        arcomp();
    }

    /** ARCOMP: ALTRATE = RUNIT . VVECT in M/CS * 2(-6), converted to bit units and compensated. */
    public void arcomp() {
        int sum = mp(g("RUNIT"), g("VVECT"));
        sum += mp(g("RUNIT+1"), g("VVECT+1"));
        sum += mp(g("RUNIT+2"), g("VVECT+2"));
        int rate = mp(g("ARCONV"), sum) * 4; // MP; DDOUBL; DDOUBL: bit units * 2(-14)
        rate += mp(g("DALTRATE"), g("DT"));
        s("ALTRATE", rate);
        int out = rate;
        if (rate >= 0) { // positive or zero: set sign bit = 1 (do not send +0)
            out = Agc.add(Agc.fromSigned(rate), BIT15);
            dataout(out);
            return;
        }
        dataout(Agc.fromSigned(out));
    }

    /** DATAOUT: load ALTM and activate the meter (CHAN14 bit3). */
    public void dataout(int bits) {
        agc.set("ALTM", bits);
        wor(014, BIT3);
        io.taskOver();
    }

    /** ALTOUT: output altitude, extrapolating old data with altitude rate between new samples. */
    public void altout() {
        if (!disindat()) {
            return;
        }
        agc.set("IMODES33", agc.get("IMODES33") & ~BIT7);
        wand(014, ~BIT2);
        int altbits = g("ALTBITS");
        if (altbits < 0) { // -1: old altitude data is to be extrapolated
            oldData(0, false);
            return;
        }
        // New altitude data: ALTSAVE <- ALTBITS (new sample), ALTBITS <- -1 for next pass.
        int newHi = altbits;
        int newLo = g("ALTBITS+1");
        s("ALTBITS", -1);
        s("ALTBITS+1", -1);
        s("ALTSAVE", newHi);
        s("ALTSAVE+1", newLo);
        int dt = g("DT") / BIT10; // rescale DT*2(-14) to *2(-9) time in cs
        oldData(mp(dt, g("ARTOA2")), true);
    }

    /** OLDDATA: extrapolate ALTSAVE with ALTRATE; negative altitude is forced to zero (ZERODATA). */
    private void oldData(int scale, boolean scaled) {
        int ext = mp(scaled ? scale : g("ARTOA"), g("ALTRATE")) * 2;
        int lo = g("ALTSAVE+1") + ext;
        int hi = g("ALTSAVE");
        if (lo > POSMAX) {
            lo -= (POSMAX + 1);
            hi += 1;
        } else if (lo < 0) {
            lo += (POSMAX + 1);
            hi -= 1;
        }
        if (hi < 0) { // ZERODATA: no negative altitudes
            hi = 0;
            lo = 0;
        }
        s("ALTSAVE", hi);
        s("ALTSAVE+1", lo);
        newData();
    }

    /** NEWDATA: produce 15-bit unsigned output from ALTSAVE / ALTSAVE+1. */
    private void newData() {
        int lo = g("ALTSAVE+1");
        if (lo < 0) {
            lo = 0;
            s("ALTSAVE+1", 0);
        }
        int bits = lo + (g("ALTSAVE") > 0 ? BIT15 : 0);
        dataout(bits & Agc.WORD_MASK);
    }

    /**
     * DISINDAT: check the mode select switch (CHAN30 bit 6) and DIDFLAG. Returns true when the
     * caller should proceed to produce output (SPEEDRUN completed); false when the routine exits.
     */
    public boolean disindat() {
        if ((io.readChannel(030) & BIT6) != 0) {
            dispRset(); // not in PGNCS: no inertial data
            return false;
        }
        if ((agc.get("FLAGWRD1") & agc.get("DIDFLBIT")) != 0) {
            speedrun();
            return true;
        }
        // Initialization: set DIDFLAG, display ALTRATE first
        agc.set("FLAGWRD1", agc.get("FLAGWRD1") | agc.get("DIDFLBIT"));
        agc.set("IMODES33", agc.get("IMODES33") & ~BIT7);
        if ((agc.get("FLAGWRD0") & agc.get("R10FLBIT")) == 0) {
            io.taskOver(); // not in descent trajectory
            return false;
        }
        wor(012, BIT8); // display inertial data outbit
        for (String sym : new String[] {"TRAKLATV", "TRAKFWDV", "LATVMETR", "FORVMETR"}) {
            agc.set(sym, 0);
        }
        io.twiddle(this::intlze, BIT4);
        io.taskOver();
        return false;
    }

    /** INTLZE: enable RR error counter and set the inertial data flag. */
    public void intlze() {
        wor(012, BIT2);
        agc.set("IMODES33", agc.get("IMODES33") | BIT8);
        io.taskOver();
    }

    /** SPEEDRUN: update velocity vector VVECT = VN + DELV + G(T-TN), then the pointer displays. */
    public void speedrun() {
        int dt = -g("PIPTIME+1") + g("TIME1");
        s("DT", dt);
        int oneSec = g("1SEC");
        String[] gdt = {"GDT/2", "GDT/2+2", "GDT/2+4"};
        String[] v = {"V", "V+2", "V+4"};
        String[] vv = {"VVECT", "VVECT+1", "VVECT+2"};
        String[] pip = {"PIPAX", "PIPAY", "PIPAZ"};
        String[] tmp = {"PIPATMPX", "PIPATMPY", "PIPATMPZ"};
        for (int i = 0; i < 3; i++) {
            int gt = (int) (((long) g(gdt[i]) * 4 * dt) / oneSec); // G(T-TN) M/CS * 2(-5)
            int val = gt + g(v[i]) * 4; // + VN rescaled from 2(-7) to 2(-5)
            val += mp(g(pip[i]) + g(tmp[i]), g("KPIP1(5)"));
            s(vv[i], val);
        }
        io.varDelay(BIT3); // pause 40 ms to let other interrupts in
        if ((agc.get("FLAGWRD0") & agc.get("R10FLBIT")) == 0) {
            return; // not descent: back to ALTROUT +1 / ALTOUT +1
        }
        int i1 = g("DELVS") + g("VVECT");
        int i2 = g("DELVS+2") + g("VVECT+1");
        int i3 = g("DELVS+4") + g("VVECT+2");
        s("ITEMP1", i1);
        s("ITEMP2", i2);
        s("ITEMP3", i3);
        s("VHY", 2 * (mp(i1, g("UHYP")) + mp(i2, g("UHYP+2")) + mp(i3, g("UHYP+4"))));
        s("VHZ", 2 * (mp(i1, g("UHZP")) + mp(i2, g("UHZP+2")) + mp(i3, g("UHZP+4"))));
        get22over32();
    }

    /** GET22/32 + LATFWDV: lateral and forward velocity in bit units. */
    private void get22over32() {
        int m22 = g("M22"); // COS(AOG) from GPMATRIX (EBANK6)
        int m32 = g("M32"); // SIN(AOG)
        int vhy = g("VHY");
        int vhz = g("VHZ");
        int lat = mp(m32, vhy) + mp(m22, vhz);
        s("LATVEL", 2 * mp(g("VELCONV"), lat));
        int fwd = mp(m32, vhz) - mp(m22, vhy);
        s("FORVEL", 2 * mp(g("VELCONV"), fwd));
        s("ITEMP6", -g("MAXVBITS"));
        vmonitor(1);
        vmonitor(0);
        wor(014, 040); // BITSET = PRIO6: drive the X-pointer display
    }

    /**
     * VMONITOR: forward (i=1) and lateral (i=0) velocity landing analog display monitor.
     * Tracks the meter (LATVMETR/FORVMETR) toward the velocity, limited to +-MAXVBITS per pass.
     */
    public void vmonitor(int i) {
        String velS = i == 0 ? "LATVEL" : "FORVEL";
        String metS = i == 0 ? "LATVMETR" : "FORVMETR";
        String trkS = i == 0 ? "TRAKLATV" : "TRAKFWDV";
        String cmdS = i == 0 ? "CDUTCMD" : "CDUTCMD+1";
        int v = g(velS);
        int m = g(metS);
        int t = g(trkS);
        int max = g("MAXVBITS");
        int r; // pulses to send (RUPTREG3,i)
        int newTrack = 0;
        boolean overLimit = v > 0 ? (max - v <= 0) : (v < 0 && v + max <= 0);
        if (!overLimit) {
            r = lvlimits(v, m, t, max); // LVLIMITS
        } else {
            // CHKLASTY: continue/saturate according to last output direction
            boolean lastOkPath;
            int a = 0;
            boolean zero = false;
            int direction = 0; // 0 = decide by LASTOK
            if (m > 0) {
                direction = v <= 0 ? 1 : 0; // LASTPOSY +5
                if (v <= 0) {
                    a = -max;
                    zero = true;
                }
            } else if (m < 0) {
                if (-v <= 0) { // LASTNEGY +4
                    a = max;
                    zero = true;
                }
            }
            if (zero) {
                r = a;
            } else {
                // LASTOK
                if (t > 0) { // LASTPOSY
                    if (v <= 0) {
                        r = -max;
                    } else {
                        r = max - m;
                        newTrack = 1; // POSVMAXY
                    }
                } else if (t < 0) { // LASTNEGY
                    if (v <= 0) {
                        r = -(m + max);
                        newTrack = -1; // NEGVMAXY
                    } else {
                        r = max;
                    }
                } else if (v <= 0) { // NEGVMAXY
                    r = -(m + max);
                    newTrack = -1;
                } else { // POSVMAXY
                    r = max - m;
                    newTrack = 1;
                }
            }
        }
        // ZEROLSTY
        s(trkS, newTrack);
        agc.set("RUPTREG3+" + i, Agc.fromSigned(r));
        agc.set(cmdS, r == 0 ? NEG0 : Agc.fromSigned(r)); // avoids +0 DINC hardware malfunction
        s(metS, m + r);
    }

    /** LVLIMITS: velocity within limits; returns pulse count r. */
    private int lvlimits(int v, int m, int t, int max) {
        int maxNeg = -max; // ITEMP6
        if (t > 0 || t < 0) {
            // LATVPOS: v>=0 -> LVMINLM else -m ; LATVNEG: v<=0 -> LVMINLM else -m
            if (t > 0 ? -v <= 0 : v <= 0) {
                return v - m;
            }
            return -m;
        }
        if (-m <= 0) { // m >= 0
            if (-v <= 0) {
                return v - m; // LVMINLM
            }
            if (-v + maxNeg + m <= 0) {
                return v - m;
            }
            return maxNeg;
        }
        // NEGLMLV: m < 0
        if (v <= 0) {
            return v - m;
        }
        if (-(max + m) + v <= 0) {
            return v - m;
        }
        return max;
    }

    /** DISPRSET: reset analog displays (disable RR error counter / inertial data bit) and flags. */
    public void dispRset() {
        if ((agc.get("FLAGWRD0") & agc.get("R10FLBIT")) != 0) {
            int mask = ((agc.get("IMODES33") & BIT8) != 0) ? BIT2 : 0;
            mask += BIT8;
            wand(012, ~mask & Agc.WORD_MASK);
        }
        // ABORTON: reset inertial data and interleave flags, reset DIDFLAG
        agc.set("IMODES33", agc.get("IMODES33") & ~BITS8_7);
        agc.set("FLAGWRD1", agc.get("FLAGWRD1") & ~agc.get("DIDFLBIT"));
        io.taskOver();
    }
}
