package apollo.comanche055;

import apollo.agc.Agc;

/**
 * EXECUTIVE.agc (Comanche 055): the AGC job scheduler.
 *
 * There are seven core sets (LOCCTR = 0, 12, ..., 72) of twelve registers each
 * (LOC, BANKSET, PUSHLOC, PRIORITY, MPAC area ...), and five VAC areas
 * (VAC1USE..VAC5USE). Core-set registers are addressed as symbol "NAME+offset"
 * relative to core set 0; the offset is LOCCTR.
 * Cross-file jumps (INTRSM, BAILOUT, CCSHOLE, SELFCHK) are calls to the other classes.
 */
public final class Executive {
    public static final int NO_CORES = 6;
    public static final int COREINC = 12;
    public static final int LOW9 = 0x1FF;
    public static final int LOW7 = 0x7F;
    public static final int FBANKMSK = 0x7C00;
    public static final int K2 = 02000;
    public static final int NEG100 = 077677;
    /** FAKEPRET = LOC(MPAC+6) - LOC(QPRET); MPAC-36D in the source. */
    public static final String FAKEPRET = "MPAC-36D";
    public static final int OCT1201 = 01201; // no VAC areas
    public static final int OCT1202 = 01202; // no core sets

    private final Agc agc;
    private final AlarmAndAbort alarms;

    public Executive(Agc agc) {
        this.agc = agc;
        this.alarms = new AlarmAndAbort(agc);
    }

    private static String reg(String base, int off) {
        return off == 0 ? base : base + "+" + off;
    }

    private int core(String base, int locctr) {
        return agc.get(reg(base, locctr));
    }

    private void setCore(String base, int locctr, int v) {
        agc.set(reg(base, locctr), v);
    }

    /** CCS semantic: returns -1 for +0 skip (value &gt;0 -&gt; 1, +0 -&gt; 2, &lt;0 -&gt; 3, -0 -&gt; 4). */
    static int ccs(int word) {
        int w = word & Agc.WORD_MASK;
        if (w == 0) return 2;               // +0
        if (w == Agc.WORD_MASK) return 4;   // -0
        return (w & Agc.SIGN_BIT) != 0 ? 3 : 1;
    }

    /** Jobs started with a Java entry point (the Java analogue of the 2CADR following the call). */
    private static final java.util.ArrayDeque<Runnable> STARTED_JOBS = new java.util.ArrayDeque<>();

    /** Removes the next job entry started by novac/findvac/spvac (used by the dispatcher). */
    public static Runnable pollStartedJob() {
        return STARTED_JOBS.poll();
    }

    /** NOVAC with the job entry given as a Java Runnable (replaces the 2CADR). */
    public void novac(int priority, Runnable job) {
        STARTED_JOBS.add(job);
        novac(priority, 0, 0);
    }

    /** FINDVAC with the job entry given as a Java Runnable (replaces the 2CADR). */
    public void findvac(int priority, Runnable job) {
        STARTED_JOBS.add(job);
        findvac(priority, 0, 0);
    }

    /** SPVAC with the job entry given as a Java Runnable (replaces the 2CADR). */
    public void spvac(int priority, Runnable job) {
        STARTED_JOBS.add(job);
        spvac(priority, 0, 0);
    }

    /** JOBSLEEP with no explicit wake address (the caller's continuation is the wake point). */
    public void jobsleep() {
        jobsleep(agc.get("Q"));
    }

    /** Alias of {@link #endofjob()}. */
    public void endOfJob() {
        endofjob();
    }

    /** NOVAC: start a job needing no VAC area. A = priority, 2CADR follows. */
    public void novac(int priority, int jobCadr, int jobBbcon) {
        agc.set("INHINT", 1);
        agc.set("NEWPRIO", Agc.add(priority, agc.get(FAKEPRET)));
        agc.set("NEWLOC", jobCadr);
        agc.set("NEWLOC+1", jobBbcon);
        agc.set("EXECTEM1", agc.get("FBANK"));
        novac2();
    }

    /** FINDVAC: start a job with a VAC area. */
    public void findvac(int priority, int jobCadr, int jobBbcon) {
        agc.set("INHINT", 1);
        agc.set("NEWPRIO", priority);
        agc.set("NEWLOC", jobCadr);
        agc.set("NEWLOC+1", jobBbcon);
        findvac2();
    }

    /** SPVAC: like FINDVAC with Q adjusted by -2 (two-word CADR returned). */
    public void spvac(int priority, int jobCadr, int jobBbcon) {
        agc.set("Q", Agc.add(agc.get("Q"), Agc.fromSigned(-2)));
        findvac(priority, jobCadr, jobBbcon);
    }

    /** CHANG1: basic-style job change (voluntary). */
    public void chang1() {
        int q = agc.get("Q");
        agc.set("L", q);
        agc.set("BBANK", agc.get("BBANK")); // CAF EXECBANK; XCH BBANK
        chanjob();
    }

    /** CHANG2: interpretive job change; negative LOC marks an interpretive job. */
    public void chang2() {
        agc.set("L", Agc.fromSigned(-Agc.toSigned(agc.get("LOC"))));
        chang2plus2();
    }

    public void chang2plus2() {
        agc.set("BBANK", agc.get("BBANK"));
        chanjobMinus1();
    }

    /** JOBSLEEP: put the current job to sleep; A holds the wake address. */
    public void jobsleep(int wakeLoc) {
        agc.set("LOC", wakeLoc);
        jobslp1();
    }

    /** JOBWAKE: wake a sleeping job whose LOC matches NEWLOC. */
    public void jobwake(int wakeCadr) {
        agc.set("INHINT", 1);
        agc.set("NEWLOC", wakeCadr);
        agc.set("Q", Agc.add(agc.get("Q"), Agc.fromSigned(-2)));
        jobwake2();
    }

    /** PRIOCHNG: change the priority of the running job. */
    public void priochng(int newPrio) {
        agc.set("INHINT", 1);
        agc.set("NEWPRIO", newPrio);
        agc.set("BANKSET", agc.get("BBANK"));
        prioch2(agc.get("Q"));
    }

    /** ENDOFJOB: terminate the current job. */
    public void endofjob() {
        endjob1();
    }

    /** ENDFIND: return to the caller after job entry is complete. */
    public void endfind() {
        agc.set("FBANK", agc.get("EXECTEM1"));
        // TCF Q+2: return to caller past the 2CADR
    }

    /** FINDVAC2: locate a free VAC area (VAC1USE..VAC5USE are CCS-tested). */
    public void findvac2() {
        agc.set("EXECTEM1", agc.get("FBANK"));
        for (int i = 1; i <= 5; i++) {
            String use = "VAC" + i + "USE";
            int c = ccs(agc.get(use));
            if (c == 1 || c == 3) { // not +0 or -0 : free area; A holds |value|-1
                vacfound(use, i);
                return;
            }
        }
        alarms.bailout(OCT1201);
    }

    /** VACFOUND: reserve the VAC area, store its address in the low 9 bits of NEWPRIO. */
    public void vacfound(String use, int i) {
        int a = Agc.add(Math.max(agc.get(use), 0), 2);
        agc.set(use, 0);
        agc.set("NEWPRIO", Agc.add(agc.get("NEWPRIO"), a));
        novac2();
    }

    /** NOVAC2: search for a free core set. */
    public void novac2() {
        agc.set("LOCCTR", 0);
        novac3(NO_CORES);
    }

    /** NOVAC3. */
    public void novac3(int count) {
        agc.set("EXECTEM2", count);
        int locctr = agc.get("LOCCTR");
        int c = ccs(core("PRIORITY", locctr));
        if (c == 4 || c == 2) { // -0 (or +0): core set available
            corfound();
        } else {
            nextcore();
        }
    }

    /** CORFOUND: load priority and pushdown pointer for the new job. */
    public void corfound() {
        int locctr = agc.get("LOCCTR");
        setCore("PRIORITY", locctr, agc.get("NEWPRIO"));
        setCore("PUSHLOC", locctr, agc.get("NEWPRIO") & LOW9);
        if (locctr != 0) {
            setloc();
            return;
        }
        agc.set("OVFIND", 0);
        agc.set("FIXLOC", agc.get("PUSHLOC"));
        spectest();
    }

    /** SPECTEST: if core set 0 loaded, check NEWJOB. */
    public void spectest() {
        int c = ccs(agc.get("NEWJOB"));
        if (c == 1) {
            setloc(); // active jobs waiting: awaken unchanged job
            return;
        }
        if (c != 2) {
            alarms.ccshole();
        }
        agc.set("NEWJOB", 0);
        agc.set("LOC", agc.get("NEWLOC"));
        agc.set("LOC+1", agc.get("NEWLOC+1"));
        endfind();
    }

    /** SETLOC: set up location registers for the job and signal a switch if it is higher priority. */
    public void setloc() {
        int locctr = agc.get("LOCCTR");
        setCore("LOC", locctr, agc.get("NEWLOC"));
        setCore("LOC+1", locctr, agc.get("NEWLOC+1"));
        int diff = Agc.add(~core("PRIORITY", agc.get("NEWJOB")) & Agc.WORD_MASK, agc.get("NEWPRIO"));
        if (Agc.toSigned(diff) > 0) { // new job has higher priority
            agc.set("NEWJOB", locctr);
        }
        endfind();
    }

    /** NEXTCORE: advance to the next core set. */
    public void nextcore() {
        agc.set("LOCCTR", Agc.add(agc.get("LOCCTR"), COREINC));
        int c = ccs(agc.get("EXECTEM2"));
        if (c == 1) {
            novac3(Agc.add(agc.get("EXECTEM2"), Agc.fromSigned(-1)));
            return;
        }
        alarms.bailout(OCT1202); // no core sets
    }

    /** CHANJOB-2: LXCH LOC / CAE BANKSET fall-through entry. */
    public void chanjobMinus2() {
        int t = agc.get("LOC");
        agc.set("LOC", agc.get("L"));
        agc.set("L", t);
        chanjobMinus1();
    }

    /** CHANJOB-1. */
    public void chanjobMinus1() {
        agc.set("A", agc.get("BANKSET"));
        chanjob();
    }

    /** CHANJOB: swap the running job with NEWJOB's core set (LOC, BANKSET, MPAC, PUSHLOC/PRIORITY). */
    public void chanjob() {
        agc.set("INHINT", 1);
        agc.set("L", agc.get("BBANK") | agc.get("SUPERBNK"));
        chanjob4();
    }

    /** CHANJOB+4. */
    public void chanjob4() {
        int nj = agc.get("NEWJOB");
        // swap LOC / BANKSET with the new core set
        swap("LOC", 0, nj);
        swap("LOC+1", 0, nj);
        agc.set("SUPERBNK", agc.get("BANKSET"));
        // swap MPAC area (MPAC .. MPAC+7)
        for (int i = 0; i < 8; i++) {
            swap(i == 0 ? "MPAC" : "MPAC+" + i, 0, nj);
        }
        // OVFIND: make PUSHLOC negative if overflow indicator was set
        int ovf = agc.get("OVFIND");
        agc.set("OVFIND", 0);
        if (ovf != 0) {
            agc.set("PUSHLOC", Agc.fromSigned(-Agc.toSigned(agc.get("PUSHLOC"))));
        }
        swap("PUSHLOC", 0, nj);
        swap("PRIORITY", 0, nj);
        agc.set("FIXLOC", agc.get("PRIORITY") & LOW9);
        // set overflow indicator according to the sign of the new PUSHLOC
        if (ccs(agc.get("PUSHLOC")) == 3 || ccs(agc.get("PUSHLOC")) == 4) {
            agc.set("PUSHLOC", Agc.fromSigned(-Agc.toSigned(agc.get("PUSHLOC"))));
            agc.set("OVFIND", 1);
            agc.set("NEWJOB", 1);
        } else {
            agc.set("NEWJOB", 0);
        }
        endprchg();
    }

    private void swap(String sym, int offA, int offB) {
        int t = agc.get(reg(sym, offA));
        agc.set(reg(sym, offA), agc.get(reg(sym, offB)));
        agc.set(reg(sym, offB), t);
    }

    /** ENDPRCHG: RELINT and dispatch the job (DTCB for basic; INTRSM for interpretive). */
    public void endprchg() {
        agc.set("INHINT", 0);
        int loc = Agc.toSigned(agc.get("LOC"));
        if (loc > 0) {
            // basic job: DTCB dispatches to LOC/LOC+1
            return;
        }
        agc.set("LOC", Agc.fromSigned(-loc)); // COM; AD ONE
        new Interpreter(agc).intrsm();
    }

    /** JOBSLP1: mark job asleep by complementing its priority. */
    public void jobslp1() {
        agc.set("INHINT", 1);
        agc.set("PRIORITY", ~agc.get("PRIORITY") & Agc.WORD_MASK);
        agc.set("BANKSET", (agc.get("BBANK") & LOW7) | agc.get("SUPERBNK"));
        jobslp2(Agc.WORD_MASK); // CS ZERO : -0
    }

    /** JOBSLP2. */
    public void jobslp2(int negHighest) {
        agc.set("BUF+1", negHighest);
        ejscan();
    }

    /** NUCHANG2: a new job requires a change-job. */
    public void nuchang2() {
        agc.set("INHINT", 1);
        if (ccs(agc.get("NEWJOB")) != 1) {
            agc.set("INHINT", 0);
            advan(2); // wake via NUDIRECT (very rare)
            return;
        }
        agc.set("DSALMOUT", agc.get("DSALMOUT") | 2); // activity light on
        chanjob4();
    }

    /** JOBWAKE2: scan core sets for a sleeping job whose CADR matches NEWLOC. */
    public void jobwake2() {
        agc.set("EXECTEM1", agc.get("FBANK"));
        agc.set("LOCCTR", 0);
        jobwake4(NO_CORES);
    }

    /** JOBWAKE4. */
    public void jobwake4(int count) {
        agc.set("EXECTEM2", count);
        int c = ccs(core("PRIORITY", agc.get("LOCCTR")));
        if (c == 1) { // active: check next
            jobwake3();
        } else {
            waketest();
        }
    }

    /** JOBWAKE3: advance to next core set; exit if sleeping job not found. */
    public void jobwake3() {
        agc.set("LOCCTR", Agc.add(agc.get("LOCCTR"), COREINC));
        if (ccs(agc.get("EXECTEM2")) == 1) {
            jobwake4(Agc.add(agc.get("EXECTEM2"), Agc.fromSigned(-1)));
            return;
        }
        agc.set("LOCCTR", Agc.fromSigned(-1)); // not found
        endfind();
    }

    /** WAKETEST: compare the core set's LOC against NEWLOC. */
    public void waketest() {
        int locctr = agc.get("LOCCTR");
        if (core("LOC", locctr) != agc.get("NEWLOC")) {
            jobwake3();
            return;
        }
        int pri = ~core("PRIORITY", locctr) & Agc.WORD_MASK;
        agc.set("NEWPRIO", pri);
        setCore("PRIORITY", locctr, pri);
        // build 2CADR from CADR in NEWLOC and bank in BANKSET
        int newloc = (agc.get("NEWLOC") & ~FBANKMSK & Agc.WORD_MASK) + K2;
        agc.set("NEWLOC+1", (agc.get("NEWLOC") & FBANKMSK) + core("BANKSET", locctr));
        agc.set("NEWLOC", newloc);
        if (locctr != 0) {
            setloc();
        } else {
            spectest();
        }
    }

    /** PRIOCH2. */
    public void prioch2(int q) {
        agc.set("LOC", q);
        agc.set("BUF", 0);
        agc.set("PRIORITY", (agc.get("PRIORITY") & LOW9) + agc.get("NEWPRIO"));
        jobslp2(~agc.get("PRIORITY") & Agc.WORD_MASK);
    }

    /** ENDJOB1: free the core set (-0 priority) and VAC area, then scan. */
    public void endjob1() {
        agc.set("INHINT", 1);
        agc.set("BUF+1", Agc.WORD_MASK);
        int pri = agc.get("PRIORITY");
        agc.set("PRIORITY", 0);
        int l = pri & LOW9;
        agc.set("L", l);
        int diff = Agc.add(~agc.get(FAKEPRET) & Agc.WORD_MASK, l);
        if (Agc.toSigned(diff) > 0) {
            // VAC area: release it by storing its base back in VACnUSE
            agc.set("VACUSE@" + (l - 1), 0);
        }
        ejscan();
    }

    /** EJSCAN: find the highest-priority active job across the 7 core sets. */
    public void ejscan() {
        // EJ1/EJ2 track the most-negative (i.e. highest) priority in BUF+1
        int bestPrio = 0;
        int bestSet = -1;
        for (int cs = 0; cs < 7; cs++) {
            int p = Agc.toSigned(core("PRIORITY", cs * COREINC));
            if (p > bestPrio) {
                bestPrio = p;
                bestSet = cs * COREINC;
            }
        }
        if (bestSet < 0) {
            dummyjob();
            return;
        }
        agc.set("BUF+1", Agc.fromSigned(-bestPrio));
        if (agc.get("BUF") == 0) {
            endprchg(); // PRIOCHNG and priority is still highest
            return;
        }
        agc.set("NEWJOB", bestSet);
        chanjobMinus2();
    }

    /** DUMMYJOB: nothing to run; idle with the activity light off. */
    public void dummyjob() {
        agc.set("NEWJOB", Agc.WORD_MASK); // -0
        agc.set("INHINT", 0);
        agc.set("DSALMOUT", agc.get("DSALMOUT") & ~2);
        advan(0);
    }

    /** ADVAN: idle loop; run the self-check when no new job is waiting. */
    public void advan(int mode) {
        if (ccs(agc.get("NEWJOB")) == 1) {
            nuchang2();
            return;
        }
        if (mode == 2) {
            nudirect();
            return;
        }
        // SELFCHK as a pseudo job
        agc.set("L", agc.get("SELFRET"));
        supdxchz(true);
    }

    /** NUDIRECT: new job already in position; turn activity light on and dispatch. */
    public void nudirect() {
        agc.set("DSALMOUT", agc.get("DSALMOUT") | 2);
        supdxchz(false);
    }

    /** SUPDXCHZ / +1: set SUPERBNK and BBANK, then dispatch to L. */
    public void supdxchz(boolean selfCheck) {
        if (selfCheck) {
            new SelfCheck(agc).selfchk();
        }
    }
}
