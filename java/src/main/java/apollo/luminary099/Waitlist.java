package apollo.luminary099;

import apollo.agc.Agc;

/**
 * WAITLIST.agc (Luminary 099, pp. 1117-1132).
 * WAITLIST, TWIDDLE, FIXDELAY, VARDELAY, T3RUPT/TASKOVER and LONGCALL.
 *
 * Task list model: TIME3 = 16384 - (T1 - T) centiseconds; LST1[i] = -(T(i+2) - T(i+1)) + 1;
 * LST2 holds 2CADRs of up to 9 tasks (empty slots hold ENDTASK, i.e. SVCT3).
 * Here the tasks are held as Runnables in {@link #lst2} and the delays as signed ints in
 * {@link #lst1} with the same meaning.
 */
public class Waitlist {
    private final Agc agc;

    /** Maximum number of tasks (LST2 has nine 2CADR slots). */
    public static final int MAX_TASKS = 9;
    /** Waitlist overflow alarm code (WTABORT / FILLED): OCT 01203. */
    public static final int ALARM_WAITLIST_OVERFLOW = 01203;
    /** Bad delta time alarm (WAITPOOH / LONGPOOH): OCT 01204. */
    public static final int ALARM_BAD_DT = 01204;
    /** Delta-time value (10000 octal) that LONGCYCL strips per pass: DPBIT14 (BIT14 = 8192 cs). */
    public static final int DPBIT14 = 0x2000;
    /** Delay used by SVCT3X when IMU zero is in progress: DEC 500. */
    public static final int SVCT3_RETRY_DELAY = 500;
    /** NEG1/2 initial LST1 value, meaning 81.91 s for ENDTASK. */
    public static final int NEG_HALF = -0x2000;

    /** LST1 (8 entries). */
    public final int[] lst1 = new int[8];
    /** LST2 (9 task slots). Null means ENDTASK (SVCT3). */
    public final Runnable[] lst2 = new Runnable[MAX_TASKS];
    /** TIME3 as 16384 - (T1 - T), kept as a signed centisecond count. */
    private int time3 = 0;

    // LONGCALL state: LONGTIME (DP), LONGCADR
    private long longTime;
    private Runnable longCadr;
    private Runnable longExit;

    public Waitlist(Agc agc) {
        this.agc = agc;
        java.util.Arrays.fill(lst1, NEG_HALF);
    }

    private static int s(int w) { return Agc.toSigned(w); }

    /** WAITLIST: schedule {@code task} to start in {@code deltaT} centiseconds (1..16250). */
    public void waitlist(int deltaT, Runnable task) {
        // INHINT; save return in WAITEXIT; pick up the 2CADR; switch to WAIT2.
        wait2(deltaT, task);
    }

    /** TWIDDLE: as WAITLIST for a task in the same E/F bank as the caller. */
    public void twiddle(int deltaT, Runnable task) {
        waitlist(deltaT, task);
    }

    /**
     * WAIT2: begin task insertion. Times are worked in centiseconds relative to now (T):
     * task 1 is due at T1 - T, task n+1 at (T(n) - T) + (-LST1[n-1] - 1). The new task
     * (due TD - T = dt) goes in front of the first later task (WTLST4 shift, or WTLST5/WTLST2
     * walk down the list), and the following interval is rewritten as -(Tn+1 - TD) + 1.
     */
    private void wait2(int dt, Runnable task) {
        if (dt <= 0) {
            waitPooh();                      // WAITPOOH: TC POODOO1, OCT 1204
            return;
        }
        int due = Math.max(0, 16384 - s(agc.get("TIME3"))); // T1 - T
        if (lst2[0] == null) {
            due = Integer.MAX_VALUE / 4;     // empty list: only ENDTASK pending
        }
        // Absolute due times of the pending tasks.
        int[] t = new int[MAX_TASKS];
        int n = 0;
        int cur = due;
        for (int i = 0; i < MAX_TASKS && lst2[i] != null; i++) {
            t[i] = cur;
            cur += -lst1[Math.min(i, lst1.length - 1)] - 1;
            n++;
        }
        if (n >= MAX_TASKS) {
            filled();                        // WTABORT / FILLED: no room in the inn
            return;
        }
        int pos = 0;
        while (pos < n && t[pos] <= dt) {
            pos++;                           // WTLST5 walk: first task due after TD
        }
        // Shift tasks up (WTLST4) and insert.
        for (int i = n; i > pos; i--) {
            lst2[i] = lst2[i - 1];
        }
        lst2[pos] = task;
        n++;
        if (pos == 0) {
            agc.set("TIME3", Agc.fromSigned(16384 - dt)); // 1.0 - DELTA T
        }
        // Rebuild the LST1 intervals: -(T(i+1) - Ti) + 1.
        int[] nt = new int[n];
        for (int i = 0, j = 0; i < n; i++) {
            nt[i] = (i == pos) ? dt : t[j++];
        }
        for (int i = 0; i < lst1.length; i++) {
            lst1[i] = (i + 1 < n) ? -(nt[i + 1] - nt[i]) + 1 : NEG_HALF;
        }
        // LVWTLIST: DXCH WAITEXIT; AD TWO; DTCB - return to caller +2.
    }

    /** FILLED: no room in the inn - BAILOUT1 with alarm 1203. */
    private void filled() {
        throw new IllegalStateException("WAITLIST overflow, alarm " + Integer.toOctalString(ALARM_WAITLIST_OVERFLOW));
    }

    /** WAITPOOH: bad delta time - POODOO1 with alarm 1204. */
    private void waitPooh() {
        throw new IllegalStateException("WAITLIST bad DT, alarm " + Integer.toOctalString(ALARM_BAD_DT));
    }

    /** FIXDELAY: wait DT (centiseconds, constant after the TC) then return to the caller. */
    public void fixDelay(int dt, Runnable continuation) {
        waitlist(dt, continuation);          // task terminates; continuation runs after DT
    }

    /** VARDELAY: as FIXDELAY with the DT arriving in A. */
    public void varDelay(int dt, Runnable continuation) {
        waitlist(dt, continuation);
    }

    /** ENDTASK / SVCT3: dummy end-of-list task; checks the drift flag and calls NBDONLY. */
    public void svct3() {
        int flagwrd2 = s(agc.get("FLAGWRD2"));
        if (flagwrd2 <= 0 && flagwrd2 >= 0) {
            taskOver();                      // CCS FLAGWRD2: drift flag clear (zero)
            return;
        }
        if (flagwrd2 < 0) {
            taskOver();
            return;
        }
        ckImuse();
    }

    /** CKIMUSE: do not do NBDONLY if someone else is in IMUSTALL. */
    private void ckImuse() {
        if (s(agc.get("IMUCADR")) != 0) {
            svct3x();
            return;
        }
        // CAF PRIO35; TC NOVAC; 2CADR NBDONLY - enabled every 81.93 seconds.
        taskOver();
    }

    /** SVCT3X: delay max of 2 times (5 seconds) for IMUZERO, then check the flag again. */
    private void svct3x() {
        fixDelay(SVCT3_RETRY_DELAY, this::svct3);
    }

    /**
     * T3RUPT: dispatch the first waitlisted task. LST1 and LST2 move up one slot, an
     * ENDTASK with NEG1/2 enters at the bottom, TIME3 is reset from the new first interval.
     */
    public void t3Rupt() {
        // T3RUPT2: move up LST1, entering NEG1/2 at the bottom.
        int dropped = lst1[0];
        for (int i = 0; i < lst1.length - 1; i++) {
            lst1[i] = lst1[i + 1];
        }
        lst1[lst1.length - 1] = NEG_HALF;
        // AD POSMAX / ADS TIME3: TIME3 = 1.0 - (T2 - T) - using list 1 so T3 won't tick.
        agc.set("TIME3", Agc.fromSigned(s(agc.get("TIME3")) + Agc.toSigned(0x3FFF) + dropped));
        // Move up LST2, entering ENDTASK at the bottom, and dispatch the first task.
        Runnable task = lst2[0];
        for (int i = 0; i < MAX_TASKS - 1; i++) {
            lst2[i] = lst2[i + 1];
        }
        lst2[MAX_TASKS - 1] = null;
        if (task == null) {
            task = this::svct3;
        }
        task.run();                           // DTCB into the task; tasks end with TC TASKOVER
    }

    /** TASKOVER: if time3 has overflowed again dispatch the next due task, else RESUME. */
    public void taskOver() {
        // CCS RUPTAGN: +1 -> T3RUPT2 again; -0 -> restore SUPERBNK and RESUME.
        int ruptagn = s(agc.get("RUPTAGN"));
        if (ruptagn > 0) {
            agc.set("RUPTAGN", 0);
            t3Rupt();
        } else {
            resume();
        }
    }

    /** RESUME: restore QRUPT, BBANK, ARUPT/LRUPT; RELINT; RESUME. */
    public void resume() {
        // EXTEND QXCH QRUPT / CA BANKRUPT / XCH BBANK / DXCH ARUPT / RELINT / RESUME
    }

    // ---------------------------------------------------------------- LONGCALL

    /**
     * LONGCALL: schedule {@code task} after a DP delta time in centiseconds (A,L as
     * TIME2,TIME1 - up to many minutes). Long delays are broken into LONGCYCL passes of
     * about 1.25 minutes (DPBIT14 centiseconds).
     */
    public void longCall(long deltaTime, Runnable task, Runnable caller) {
        longTime = deltaTime;                // DXCH LONGTIME
        longCadr = task;                     // DXCH LONGCADR
        longExit = caller;                   // LONGEXIT / LONGEXIT +1
        lngCall2();
    }

    /** LNGCALL2: check for a legitimate delta time. */
    private void lngCall2() {
        if (longTime <= 0) {
            longPooh();
            return;
        }
        longCycl();
    }

    /** LONGCYCL: take about 1.25 minutes off LONGTIME; schedule GETCADR when the rest fits. */
    public void longCycl() {
        longTime -= DPBIT14;
        if (longTime > 0) {
            muchTime();
        } else {
            lastTime();
        }
    }

    /** LASTTIME: restore the correct delta T for WAITLIST and schedule GETCADR. */
    private void lastTime() {
        int dt = (int) (longTime + DPBIT14);
        waitlist(dt, this::getCadr);
        longRtrn();
    }

    /** MUCHTIME: more than 1.25 minutes left; set up another LONGCYCL pass. */
    private void muchTime() {
        waitlist(DPBIT14, this::longCycl);
        longRtrn();
    }

    /** LONGRTRN: the first exit is to the caller of LONGCALL; later exits go to TASKOVER. */
    private void longRtrn() {
        Runnable exit = longExit;
        longExit = this::taskOver;
        exit.run();
    }

    /** GETCADR: transfer control to the task given to LONGCALL. */
    public void getCadr() {
        longCadr.run();
    }

    /** LONGPOOH: bad delta time - POODOO1 with alarm 1204. */
    private void longPooh() {
        throw new IllegalStateException("LONGCALL bad DT, alarm " + Integer.toOctalString(ALARM_BAD_DT));
    }
}
