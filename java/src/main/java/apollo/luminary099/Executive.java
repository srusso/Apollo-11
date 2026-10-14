package apollo.luminary099;

import apollo.agc.Agc;

/**
 * EXECUTIVE.agc - the AGC cooperative priority scheduler.
 *
 * Seven core sets (each 12 erasable registers: MPAC area, LOC/BBANK, PRIORITY, PUSHLOC...) and five
 * VAC areas hold jobs. A job's priority register is -0 when the core set is free, positive when
 * the job is active and negative when it is asleep. Entry points: NOVAC (no VAC), FINDVAC (needs a
 * VAC area), SPVAC, CHANG1/CHANG2 (voluntary suspension), JOBSLEEP / JOBWAKE, PRIOCHNG,
 * ENDOFJOB. Idle processing is DUMMYJOB (green activity light off, SELFCHK).
 *
 * In this Java model a job is a {@link Runnable}; "NEWJOB" is the index of the core set to run next.
 */
public class Executive {
    /** NO.CORES: seven core sets. */
    public static final int NO_CORES = 7;
    /** COREINC: 12 registers per core set. */
    public static final int CORE_INC = 12;
    public static final int VAC_AREAS = 5;
    /** Priority words carry the VAC address in the low nine bits. */
    public static final int LOW9 = 0x1FF;
    /** BAILOUT codes. */
    public static final int ALARM_NO_VAC = 01201;
    public static final int ALARM_NO_CORE = 01202;
    public static final int NEG100 = 077677;

    /** One executive core set. */
    public static final class CoreSet {
        /** PRIORITY: 0 means free (-0); positive = active; negative = asleep. */
        public int priority = 0;
        public boolean free = true;
        /** LOC: the job (Runnable) and its wake key (FCADR) for JOBWAKE matching. */
        public Runnable job;
        public Object wakeKey;
        /** MPAC (multi-purpose accumulator) area, 8 words. */
        public final int[] mpac = new int[8];
        /** VAC area index (1..5) or 0 when none (NOVAC). */
        public int vac;
        public boolean interpretive;
    }

    private final Agc agc;
    private final CoreSet[] cores = new CoreSet[NO_CORES];
    private final boolean[] vacInUse = new boolean[VAC_AREAS];
    /** NEWJOB: index of the core set that should run next; -1 => +0 (active job already set). */
    private int newJob = -1;
    private int running = 0;
    private boolean activityLight;

    public Executive(Agc agc) {
        this.agc = agc;
        for (int i = 0; i < NO_CORES; i++) {
            cores[i] = new CoreSet();
        }
    }

    // --- job entry ---

    /** NOVAC: enter a job request needing no VAC area. */
    public void novac(int priority, Runnable job) {
        novac2(priority, job, 0);
    }

    /** FINDVAC: enter a job needing a VAC area (all interpretive jobs). */
    public void findVac(int priority, Runnable job) {
        int vac = 0;
        for (int i = 0; i < VAC_AREAS; i++) {          // CCS VACiUSE ... VACFOUND
            if (!vacInUse[i]) { vac = i + 1; break; }
        }
        if (vac == 0) {
            bailout(ALARM_NO_VAC);                      // OCT 1201: no VAC areas
            return;
        }
        vacInUse[vac - 1] = true;                       // reserve it
        novac2(priority, job, vac);
    }

    /** SPVAC: FINDVAC with priority already in NEWPRIO (caller has INHINTed). */
    public void spVac(int priority, Runnable job) {
        findVac(priority, job);
    }

    /** NOVAC2: find a free core set and install the job (CORFOUND / SETLOC). */
    private void novac2(int priority, Runnable job, int vac) {
        int idx = -1;
        for (int i = 0; i < NO_CORES; i++) {            // NOVAC3 / NEXTCORE scan
            if (cores[i].free) { idx = i; break; }
        }
        if (idx < 0) {
            bailout(ALARM_NO_CORE);                     // OCT 1202: no core sets
            return;
        }
        CoreSet c = cores[idx];
        c.free = false;
        c.priority = priority;
        c.job = job;
        c.vac = vac;
        c.interpretive = vac != 0;
        // SETLOC: compare with the highest active priority to decide if NEWJOB changes
        if (newJob < 0 || priority > cores[newJob].priority || cores[newJob].free) {
            if (idx != running || cores[running].free) {
                newJob = idx;
            }
        }
        agc.set("NEWJOB", newJob + 1);
        agc.set("LOCCTR", idx * CORE_INC);
    }

    // --- job control ---

    /** CHANG1 / CHANG2: voluntarily suspend the current job so a higher priority one may run. */
    public void chang1() {
        chanJob();
    }

    public void chang2() {
        cores[running].interpretive = true;             // negative LOC marks an interpretive job
        chanJob();
    }

    /** JOBSLEEP: put the current job to sleep until JOBWAKE with the same key. */
    public void jobSleep(Object wakeKey, Runnable resume) {
        CoreSet c = cores[running];
        c.wakeKey = wakeKey;
        c.job = resume;
        c.priority = -Math.abs(c.priority);             // JOBSLP1: negative priority => asleep
        ejScan(-1);
    }

    /** JOBWAKE: wake the job asleep on wakeKey; returns false if none found (LOCCTR = -1). */
    public boolean jobWake(Object wakeKey) {
        for (int i = 0; i < NO_CORES; i++) {            // JOBWAKE4 / WAKETEST
            CoreSet c = cores[i];
            if (!c.free && c.priority < 0 && wakeKey.equals(c.wakeKey)) {
                c.priority = -c.priority;               // re-complement priority: awake
                if (newJob < 0 || c.priority > cores[newJob].priority) {
                    newJob = i;
                }
                agc.set("LOCCTR", i * CORE_INC);
                return true;
            }
        }
        agc.set("LOCCTR", Agc.fromSigned(-1));
        return false;
    }

    /** PRIOCHNG: change the priority of the running job (PRIOCH2 / EJSCAN). */
    public void prioChng(int newPriority) {
        CoreSet c = cores[running];
        c.priority = newPriority;                       // low nine bits (VAC address) preserved in the original
        int best = highestActive();
        if (best >= 0 && best != running && cores[best].priority > c.priority) {
            newJob = best;
            chanJob();
        }
    }

    /** ENDOFJOB / ENDJOB1: release the core set and VAC area and dispatch the next job. */
    public void endOfJob() {
        CoreSet c = cores[running];
        if (c.vac != 0) {
            vacInUse[c.vac - 1] = false;                // free the VAC area
        }
        c.free = true;
        c.priority = 0;
        c.job = null;
        c.wakeKey = null;
        c.vac = 0;
        ejScan(-1);
    }

    /** EJSCAN: find the job of highest active priority; DUMMYJOB if none. */
    private void ejScan(int ignored) {
        int best = highestActive();
        if (best < 0) {
            dummyJob();
        } else {
            newJob = best;
            chanJob();
        }
    }

    private int highestActive() {
        int best = -1;
        for (int i = 0; i < NO_CORES; i++) {            // EJ1 / EJ2
            CoreSet c = cores[i];
            if (!c.free && c.priority > 0 && (best < 0 || c.priority > cores[best].priority)) {
                best = i;
            }
        }
        return best;
    }

    /** CHANJOB: swap in the core set named by NEWJOB (MPAC areas are exchanged by the original). */
    private void chanJob() {
        if (newJob < 0) {
            return;
        }
        running = newJob;
        newJob = -1;
        activityLight = true;
        Runnable job = cores[running].job;
        if (job != null) {
            job.run();                                  // DTCB / INTRSM
        }
    }

    /** DUMMYJOB: idle; activity light off; keep checking NEWJOB (ADVAN); SELFCHK would run here. */
    public void dummyJob() {
        newJob = -1;
        activityLight = false;                          // WAND DSALMOUT clears the activity bit
    }

    /** NUCHANG2 / NUDIRECT: dispatch a waiting job after idling. */
    public void advan() {
        if (newJob >= 0) {
            activityLight = true;                       // WOR DSALMOUT
            chanJob();
        }
    }

    /** Runs all pending jobs until idle (test driver for the model). */
    public void runUntilIdle() {
        advan();
        while (newJob >= 0) {
            advan();
        }
    }

    public boolean isActivityLightOn() {
        return activityLight;
    }

    /** SUPDXCHZ: transfer to a location in the super bank (no-op in the Java model). */
    public void supDxchz(Runnable target) {
        target.run();
    }

    private void bailout(int code) {
        agc.set("BAILOUT", code);
        throw new IllegalStateException("BAILOUT " + Integer.toOctalString(code));
    }
}
