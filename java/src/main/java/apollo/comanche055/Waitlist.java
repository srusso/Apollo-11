package apollo.comanche055;

import apollo.agc.Agc;

import java.util.Map;
import java.util.WeakHashMap;

/**
 * WAITLIST.agc (Comanche 055, pages 1221-1235): WAITLIST, TWIDDLE, FIXDELAY, VARDELAY,
 * T3RUPT dispatch, TASKOVER, LONGCALL (LONGCYCL, GETCADR) and the SVCT3 drift task.
 *
 * A task (a 2CADR in the original) is modelled as a {@link Runnable}. The delta-time list
 * LST1[0..7] is kept in the Agc erasable table as the symbols "LST1", "LST1 +1" ... with the
 * same encoding as the original:
 *   C(TIME3)   = 16384 - (T1 - T) centiseconds
 *   C(LST1 +k) = -(T(k+2) - T(k+1)) + 1
 * while the 2CADR list LST2 (9 entries) is a Java array of Runnables (null = ENDTASK).
 */
public class Waitlist {
    private final Agc agc;

    public static final int POSMAX = 037777;
    /** NEG1/2: initial LST1 filler. */
    public static final int NEG_HALF = 050000 | 0x0FFF;   // -0.5 in one's complement (0x5FFF)
    public static final int MAX_DT = 16250;
    public static final int ABORT_OVERFLOW = 01203;       // WAITLIST OVERFLOW - TOO MANY TASKS
    public static final int ABORT_BAD_DT = 01204;         // WAITLIST CALL WITH ZERO OR NEG DT
    private static final int BIT8 = 0200;
    private static final int BIT14 = 020000;

    /** Per-machine task list (LST2) and Q-saved state. */
    private static final class State {
        final Runnable[] lst2 = new Runnable[9];
        Runnable longTask;
        long longTime;           // LONGTIME, LONGTIME+1 (DP centiseconds)
        Runnable longExit;
    }

    private static final Map<Agc, State> STATES = new WeakHashMap<>();

    public Waitlist(Agc agc) {
        this.agc = agc;
        synchronized (STATES) {
            STATES.computeIfAbsent(agc, a -> {
                State st = new State();
                for (int i = 0; i < 8; i++) {
                    a.set(lst1(i), NEG_HALF);
                }
                return st;
            });
        }
    }

    private State st() {
        synchronized (STATES) {
            return STATES.get(agc);
        }
    }

    private static String lst1(int i) {
        return i == 0 ? "LST1" : "LST1 +" + i;
    }

    /** Count of tasks currently scheduled. */
    private int taskCount() {
        int n = 0;
        for (Runnable r : st().lst2) {
            if (r != null) n++;
        }
        return n;
    }

    /**
     * WAITLIST: schedule {@code task} to start in {@code dt} centiseconds (1 .. 16250).
     * Aborts with 1204 if dt is zero or negative, 1203 if there are already nine tasks.
     */
    public void waitlist(int dt, Runnable task) {
        // INHINT
        if (dt <= 0) {
            throw new IllegalStateException("POODOO 1204: WAITLIST call with zero or negative dt");
        }
        if (dt > MAX_DT) {
            dt = MAX_DT;
        }
        wait2(dt, task);
        // RELINT; return to caller
    }

    /** TWIDDLE: like WAITLIST but the task is in the caller's own bank (no BBCON needed). */
    public void twiddle(int dt, Runnable task) {
        waitlist(dt, task);
    }

    /**
     * WAIT2: begin task insertion. Computes TD (delay relative to T1 using TIME3) and
     * walks the delta list (WTLST5 / WTLST2 / WTLST4).
     */
    private void wait2(int dt, Runnable task) {
        State s = st();
        if (taskCount() >= 9) {
            throw new IllegalStateException("BAILOUT 1203: WAITLIST overflow - too many tasks");
        }
        int time3 = agc.get("TIME3");
        // Absolute times (centiseconds from now): T1 = 16384 - TIME3 (if TIME3 overflow
        // occurred earlier the original corrects by the T-T1+1.0 form; handled by the clamp).
        long t = 16384L - time3;
        if (t < 0) t = 0;
        long[] times = new long[9];
        int n = taskCount();
        if (n > 0) {
            times[0] = t;
            for (int i = 1; i < n; i++) {
                times[i] = times[i - 1] + (1 - Agc.toSigned(agc.get(lst1(i - 1))));
            }
        }
        // Insert TD in order (WTLST5 scan: after tasks whose time is <= TD).
        int pos = 0;
        while (pos < n && times[pos] <= dt) {
            pos++;
        }
        for (int i = n; i > pos; i--) {
            s.lst2[i] = s.lst2[i - 1];
            times[i] = times[i - 1];
        }
        s.lst2[pos] = task;
        times[pos] = dt;
        n++;
        // Rewrite TIME3 (when new task is first) and LST1 deltas.
        agc.set("TIME3", (int) (16384 - times[0]) & Agc.WORD_MASK);
        for (int i = 0; i < 8; i++) {
            if (i < n - 1) {
                agc.set(lst1(i), Agc.fromSigned((int) (1 - (times[i + 1] - times[i]))));
            } else {
                agc.set(lst1(i), NEG_HALF);
            }
        }
    }

    /** FIXDELAY: delay the current task for {@code dt} cs then continue with {@code next}. */
    public void fixdelay(int dt, Runnable next) {
        waitlist(dt, next);
        taskover();
    }

    /** VARDELAY: as FIXDELAY but dt arrives in A. */
    public void vardelay(int dtInA, Runnable next) {
        fixdelay(dtInA, next);
    }

    /**
     * SVCT3: drift-compensation task. If the drift flag (FLAGWRD2) is set and the IMU is
     * not in use by someone else, start NBDONLY (compensate NBD coefficients only) with
     * priority 35; otherwise delay 500 cs (at most twice) and test again.
     */
    public void svct3() {
        int flagwrd2Branch = ccs(agc.get("FLAGWRD2"));
        if (flagwrd2Branch == 0 || flagwrd2Branch == 2) {   // CCS: >0 or <0 -> TASKOVER
            taskover();
            return;
        }
        if (flagwrd2Branch == 3) {                          // -0: fall to IMUCADR test
            // (TCF +1 follows the second TASKOVER; +0 and -0 both reach here)
        }
        int imu = ccs(agc.get("IMUCADR"));
        if (imu == 1) {                                     // +0: nobody else in
            enableNbdonly();
        } else {
            svct3x();
        }
    }

    private void enableNbdonly() {
        // CAF PRIO35 ; TC NOVAC ; 2CADR NBDONLY
        new Executive(agc).novac(035, () -> new ImuCompensationPackage(agc).nbdonly());
        taskover();
    }

    /** SVCT3X: delay max of 2 times (500 cs) for IMU zero, then check drift flag again. */
    private void svct3x() {
        fixdelay(500, this::svct3);
    }

    /**
     * T3RUPT: Time3 interrupt. Moves up LST1, resets TIME3 and dispatches the first
     * task in LST2 (T3RUPT2). The task ends via TASKOVER.
     */
    public void t3rupt() {
        t3rupt2();
    }

    private void t3rupt2() {
        State s = st();
        int oldFirst = agc.get(lst1(0));
        for (int i = 0; i < 7; i++) {
            agc.set(lst1(i), agc.get(lst1(i + 1)));
        }
        agc.set(lst1(7), NEG_HALF);
        agc.set("TIME3", Agc.add(agc.get("TIME3"), Agc.add(oldFirst, POSMAX)));
        boolean again = false;   // RUPTAGN = +1 only if the ADS TIME3 overflowed again
        Runnable task = s.lst2[0];
        System.arraycopy(s.lst2, 1, s.lst2, 0, 8);
        s.lst2[8] = null;
        agc.set("RUPTAGN", again ? 1 : Agc.WORD_MASK);       // -0 => resume, +1 => redo
        if (task != null) {
            task.run();                                      // DTCB to the task's 2CADR
        }
    }

    /**
     * TASKOVER: return after a task. If RUPTAGN is +1 another task is due and T3RUPT2
     * runs again; otherwise the interrupted program resumes (RESUME).
     */
    public void taskover() {
        if (ccs(agc.get("RUPTAGN")) == 0) {
            t3rupt2();
        }
        // else RESUME: restore SUPERBNK, QRUPT, BBANK, ARUPT/LRUPT; RELINT; RESUME
    }

    private static int ccs(int word) {
        int v = word & Agc.WORD_MASK;
        if (v == 0) return 1;
        if (v == Agc.WORD_MASK) return 3;
        return (v & Agc.SIGN_BIT) != 0 ? 2 : 0;
    }

    // ------------------------------------------------------------------ LONGCALL

    /**
     * LONGCALL: schedule {@code task} after a long DP delta time {@code deltaCs}
     * (TIME2,TIME1 scaling: centiseconds). The delay is split into cycles of about
     * 1.25 minutes (BIT14 = 16384 cs) by the waitlist task LONGCYCL.
     */
    public void longcall(long deltaCs, Runnable task) {
        State s = st();
        s.longTime = deltaCs;
        s.longTask = task;
        s.longExit = null;
        longcyc();
    }

    /** LONGCYCL: take about 1.25 minutes off LONGTIME, rescheduling until the remainder fits. */
    public void longcyc() {
        State s = st();
        s.longTime -= BIT14;
        if (s.longTime > 0) {
            muchtime();
        } else {
            lasttime();
        }
    }

    /** LASTTIME: recover correct delta T and WAITLIST the real task via GETCADR. */
    private void lasttime() {
        State s = st();
        long dt = s.longTime + BIT14;
        waitlist((int) dt, this::getcadr);
        longrtrn();
    }

    /** MUCHTIME: more than one cycle left; wait BIT14 and run LONGCYCL again. */
    private void muchtime() {
        waitlist(BIT14, this::longcyc);
        longrtrn();
    }

    /** LONGRTRN: first exit returns to the caller, later ones go to TASKOVER (TSKOVCDR). */
    private void longrtrn() {
        // Only the first exit is to the caller of LONGCALL; the rest end the task.
        if (st().longExit != null) {
            taskover();
        }
    }

    /** GETCADR: transfer control to the LONGCALL task. */
    public void getcadr() {
        Runnable r = st().longTask;
        if (r != null) {
            r.run();
        }
    }
}
