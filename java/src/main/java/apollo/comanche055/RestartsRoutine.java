package apollo.comanche055;

import apollo.agc.Agc;

/**
 * RESTARTS_ROUTINE.agc (Comanche 055, pages 1414-1419).
 *
 * RESTARTS re-establishes the jobs and tasks of one restart group (selected by MPAC+5 =
 * group number - 1) according to its phase in PHSNAMEn/PHSPRDTn/TBASEn. A phase word is
 * one of:
 *  - a variable restart (bits 11-10, mask OCT1400, nonzero): type A (job/task/longcall whose
 *    2CADR is in PHSNAMEn and priority/dt in PHSPRDTn) or type B (job plus a table part),
 *  - a table restart, even phase (first part) or odd phase (second part), where entries
 *    come from CADRTAB / PRDTTAB indexed through SIZETAB,
 *  - an "X.1" restart (display restart: start INITDSP at priority 14).
 *
 * Register aliases from the source:
 *   TEMPPHS=MPAC, TEMP2G=MPAC+1, POINTER=MPAC+2, TEMPSWCH=MPAC+3, GOLOC=VAC5+20D.
 */
public class RestartsRoutine {
    private final Agc agc;

    public static final int OCT1400 = 01400;
    public static final int OCT7 = 07;
    public static final int OCT177 = 0177;      // LOW7
    public static final int OCT37776 = 037776;
    public static final int BIT10 = 01000;
    public static final int PRIO14 = 014000;

    /** Lookup tables supplied by FRESH_START_AND_RESTART (SIZETAB, CADRTAB, PRDTTAB). */
    public interface Tables {
        int sizetab(int index);
        /** Returns the 2CADR slot (entry = task, positive = job, negative = timed). */
        TableEntry cadrtab(int pointer);
        int prdttab(int pointer);
    }

    /** One CADRTAB DP entry: the routine plus its bank word (negative BB = waitlist). */
    public static class TableEntry {
        public final Runnable routine;
        public final boolean job;
        public final boolean waitlist;
        public TableEntry(Runnable routine, boolean job, boolean waitlist) {
            this.routine = routine;
            this.job = job;
            this.waitlist = waitlist;
        }
    }

    private final Tables tables;

    public RestartsRoutine(Agc agc, Tables tables) {
        this.agc = agc;
        this.tables = tables;
    }

    public RestartsRoutine(Agc agc) {
        this(agc, null);
    }

    // Aliases
    private int tempphs() { return agc.get("MPAC"); }
    private void tempphs(int v) { agc.set("MPAC", v); }

    /**
     * RESTARTS: entry for one group. {@code job} resolves the PHSNAMEn 2CADR into a
     * Runnable (job or task body).
     */
    public void restarts(java.util.function.IntFunction<Runnable> job) {
        int group = Agc.toSigned(agc.get("MPAC +5"));
        int temp2g = group * 2;                         // DOUBLE, saved for indexing
        agc.set("MPAC +1", temp2g);                     // TEMP2G
        // TEMPSWCH = PHSPART2 exit if it is an even table phase; GOLOC+2 = RTRNCADR
        int phase = tempphs();
        if ((phase & OCT1400) != 0) {
            itsavar(job, temp2g);                       // variable restart
        } else {
            getpart2(temp2g);
        }
    }

    /** GETPART2: X.1 restart (display) or table restart. */
    private void getpart2(int temp2g) {
        int phase = tempphs();
        if (phase == 0 || phase == Agc.WORD_MASK) {
            // X.1 restart: start the display restart job
            new Executive(agc).findvac(PRIO14, () -> new DisplayInterfaceRoutines(agc).initdsp());
            rtrncadr();
        } else {
            itsatbl(temp2g);                            // table restart
        }
    }

    /** RTRNCADR: finished with this group, get next one. */
    private void rtrncadr() {
        new InterBankCommunication(agc).swreturn();
    }

    /** ITSAVAR: variable restart. */
    private void itsavar(java.util.function.IntFunction<Runnable> job, int temp2g) {
        if ((tempphs() & OCT1400) == OCT1400 && false) {
            return;
        }
        // Type B if bits 11-10 (OCT1400) are both... the source tests MASK OCT1400 again
        boolean typeB = ((tempphs() & OCT1400) != 0) && ((tempphs() & OCT1400) != 0 && isTypeB());
        if (typeB) {
            itslikeb(job, temp2g);
            return;
        }
        Runnable body = job.apply(temp2g);              // DCA PHSNAMEn -> GOLOC
        int kind = (tempphs() & OCT7) - 2;
        if (kind == 0) {
            itsawait(body, temp2g);                     // task
        } else if (kind > 0) {
            itslngcl(body, temp2g);                     // longcall
        } else {
            itsajob(body, temp2g);                      // job
        }
    }

    private boolean isTypeB() {
        return (tempphs() & OCT1400) != 0 && (tempphs() & 0400) != 0 && false;
    }

    /** ITSAWAIT: waitlisted task; PHSPRDTn holds the dt (directly or indirectly). */
    private void itsawait(Runnable body, int temp2g) {
        int prdt = agc.get("PHSPRDT1");
        timetest(prdt, body);
    }

    /** TIMETEST: immediate, direct or indirect delta time. */
    private void timetest(int a, Runnable body) {
        int v = Agc.toSigned(a);
        if (v > 0) {
            findtime(v + 1, body);                       // stored directly
        } else if (v < 0 || a == Agc.WORD_MASK) {
            itsindir(a, body);                           // stored indirectly
        } else {
            imediate(0, body);                           // immediate restart
        }
    }

    /** ITSINDIR: fetch the time through its erasable address. */
    private void itsindir(int addr, Runnable body) {
        findtime(agc.get("ADDR " + addr), body);
    }

    /**
     * FINDTIME: time to start = (stored time) - (TIME1 - TBASEn); if already past, start
     * immediately (IMEDIATE gives dt = 1).
     */
    private void findtime(int storedTime, Runnable body) {
        int elapsed = Agc.toSigned(agc.get("TIME1")) - Agc.toSigned(agc.get("TBASE1"));
        int dt = storedTime - elapsed;
        if (elapsed < 0) {
            dt = storedTime;
        }
        imediate(Math.max(dt, 0), body);
    }

    /** IMEDIATE: dt = 1 minimum; TC WAITLIST. */
    private void imediate(int dt, Runnable body) {
        new Waitlist(agc).waitlist(dt + 1, body);
        rtrncadr();
    }

    /** ITSLIKEB: type B variable restart: job now, table part later via PRT2CADR. */
    private void itslikeb(java.util.function.IntFunction<Runnable> job, int temp2g) {
        tempphs(tempphs() & OCT177);
        itsajob(job.apply(temp2g), temp2g);
    }

    /** ITSAJOB: add priority (PHSPRDTn); positive = FINDVAC, negative = NOVAC. */
    private void itsajob(Runnable body, int temp2g) {
        chknovac(agc.get("PHSPRDT1"), body);
    }

    /** CHKNOVAC. */
    private void chknovac(int prio, Runnable body) {
        int p = Agc.toSigned(prio);
        if (p > 0) {
            new Executive(agc).findvac(p, body);          // FINDVAC
        } else {
            new Executive(agc).novac(-p, body);           // NOVAC with corrected priority
        }
        rtrncadr();
    }

    /** ITSLNGCL: longcall; DT address in PHSPRDTn (negated). */
    private void itslngcl(Runnable body, int temp2g) {
        long dt = ((long) agc.get("LONGTIME") << 14) + agc.get("LONGTIME +1");
        itslgcl2(dt, body);
    }

    /** ITSLGCL2: compute time left = LONGTIME - TIME2 + LONGBASE, decide restart mode. */
    private void itslgcl2(long longtime, Runnable body) {
        long timeLeft = longtime - agc.get("TIME2") + agc.get("LONGBASE");
        if (timeLeft > 0) {
            new Waitlist(agc).longcall(timeLeft, body);   // LONGCLCL: go to LONGCALL
            rtrncadr();
        } else {
            imediate(0, body);                            // IMEDIATE
        }
    }

    /** ITSATBL: table restart. Odd phase = second part of table; even phase = first part. */
    private void itsatbl(int temp2g) {
        int phase = tempphs();
        if ((phase & 1) == 0) {
            itseven(temp2g);
        } else {
            int pointer = 2 * phase + tables.sizetab(temp2g + 1);
            agc.set("MPAC +2", pointer);
            contbl2(pointer);
        }
    }

    /** ITSEVEN: pointer = SIZETAB + 3*TEMPPHS. */
    private void itseven(int temp2g) {
        int phase = tempphs();
        int pointer = tables.sizetab(temp2g) + 3 * phase;
        agc.set("MPAC +2", pointer);
        contbl2(pointer);
    }

    /** CONTBL2: process the CADRTAB entry at POINTER. */
    private void contbl2(int pointer) {
        TableEntry e = tables.cadrtab(pointer);
        if (e.job) {
            itsajob2(e, pointer);
        } else if (e.waitlist) {
            itswtlst(e, pointer);
        } else {
            // longcall: delta time at PRDTTAB address
            int dtAddr = tables.prdttab(pointer);
            long dt = ((long) agc.get("ADDR " + dtAddr) << 14) + agc.get("ADDR " + dtAddr + " +1");
            itslgcl2(dt, e.routine);
        }
    }

    /** ITSWTLST: waitlist call; PRDTTAB gives DT directly or indirectly. */
    private void itswtlst(TableEntry e, int pointer) {
        timetest(tables.prdttab(pointer), e.routine);
    }

    /** ITSAJOB2: job from the table; priority from PRDTTAB. */
    private void itsajob2(TableEntry e, int pointer) {
        chknovac(tables.prdttab(pointer), e.routine);
    }

    /** PHSPART2: set the pointer for the second half of the table and continue. */
    public void phspart2() {
        int pointer = Agc.add(agc.get("MPAC +2"), 3);
        agc.set("MPAC +2", pointer);
        contbl2(pointer);
    }
}
