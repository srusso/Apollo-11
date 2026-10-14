package apollo.luminary099;

import apollo.agc.Agc;

/**
 * Translation of RESTARTS_ROUTINE.agc (Luminary 1A build 099).
 *
 * RESTARTS re-establishes jobs, waitlist tasks and long calls from the restart tables
 * (phase table PHSNAME/PHSPRDT/TBASE, plus the CADRTAB/PRDTTAB/SIZETAB tables) after a
 * hardware restart. It is entered once per restart group with the group number minus 1
 * in MPAC+5.
 *
 * Scratch aliases (EQUALS in the source): TEMPPHS=MPAC, TEMP2G=MPAC+1, POINTER=MPAC+2,
 * TEMPSWCH=MPAC+3, GOLOC=VAC5+20D.
 *
 * The "exit" cell (GOLOC+2) is modelled by the {@link #exit} field: the continuation to
 * run after the current group entry has been restarted.
 */
public class RestartsRoutine {
    private static final int OCT1400 = 01400;
    private static final int OCT7 = 07;
    private static final int OCT177 = 0177;       // LOW7
    private static final int BIT10 = 0400;
    private static final int THREE = 3;

    private final Agc agc;
    private Runnable exit = () -> { };            // GOLOC +2 (what to do next)
    private Runnable swchExit = () -> { };        // TEMPSWCH

    public RestartsRoutine(Agc agc) {
        this.agc = agc;
    }

    // ---- aliases -----------------------------------------------------------------
    private int tempphs() { return agc.get("MPAC"); }
    private void tempphs(int v) { agc.set("MPAC", v); }
    private int temp2g() { return agc.get("MPAC+1"); }
    private int pointer() { return agc.get("MPAC+2"); }
    private void pointer(int v) { agc.set("MPAC+2", v); }

    /** RESTARTS: entry for one restart group; MPAC+5 holds group number minus 1. */
    public void restarts() {
        agc.set("MPAC+1", 2 * agc.get("MPAC+5"));   // TEMP2G: doubled for indexing
        swchExit = this::phspart2;                    // exit if it is an even table phase
        exit = this::rtrncadr;                        // assume next group follows
        if ((tempphs() & OCT1400) != 0) {
            itsavar();                                // variable restart
        } else {
            getpart2();
        }
    }

    /** GETPART2: X.1 (display) restart or table restart. */
    public void getpart2() {
        if (tempphs() != 0) {
            itsatbl();                                // table restart
            return;
        }
        // X.1 restart: start the display restart job
        new Executive(agc).findvac(agc.get("PRIO14"), () -> new PinballGameButtonsAndLights(agc).initdsp());
        rtrncadr();
    }

    /** RTRNCADR: finished with this group; get the next one. */
    public void rtrncadr() {
        new InterBankCommunication(agc).swreturn(agc.get("FBANK"));
    }

    /** ITSAVAR: variable restart (type A or B). */
    public void itsavar() {
        if ((tempphs() & OCT1400) != 0 && isTypeB()) {
            itslikeb();
            return;
        }
        storeJobCadr();
        int kind = (tempphs() & OCT7) - 2;
        if (kind > 0) {
            itslngcl();                               // a LONGCALL
        } else if (kind == 0) {
            itsawait();                               // a task via WAITLIST
        } else {
            itsajob();
        }
    }

    private boolean isTypeB() {
        return (tempphs() & OCT1400) != 0;
    }

    private void storeJobCadr() {
        // DCA PHSNAME1,TEMP2G -> GOLOC,GOLOC+1
        agc.set("GOLOC", agc.get("PHSNAME1+" + temp2g()));
        agc.set("GOLOC+1", agc.get("PHSNAME1+" + (temp2g() + 1)));
    }

    /** ITSAWAIT: set up a WAITLIST call; time stored directly in PHSPRDT. */
    public void itsawait() {
        agc.set("GOLOC-1", agc.get("WTLTCADR"));
        timetest(agc.get("PHSPRDT1+" + temp2g()));
    }

    /** TIMETEST: positive = direct time, negative = indirect, zero = immediate. */
    public void timetest(int a) {
        int s = Agc.toSigned(a);
        if (s > 0) {
            findtime(s);
        } else if (s < 0) {
            itsindir(s);
        } else {
            imediate(1);
        }
    }

    /** ITSINDIR (fixed-fixed): fetch the delta time indirectly using the task's EBANK. */
    public void itsindir(int addr) {
        findtime(agc.get("ERASABLE+" + (-addr)));
    }

    /** FINDTIME: compute time remaining relative to TBASE1 and TIME1. */
    public void findtime(int dt) {
        int negDt = ~dt & Agc.WORD_MASK;                      // COM
        int elapsed = Agc.add(agc.get("TBASE1+" + temp2g()) ^ Agc.WORD_MASK,
                Agc.WORD_MASK ^ agc.get("TIME1"));            // CS TBASE1; SU TIME1
        int diff = Agc.toSigned(elapsed);
        int adj = diff > 0 ? diff : -diff - 1;
        int t = Agc.add(Agc.fromSigned(adj + 037776 + 1), negDt);
        int remaining = Agc.toSigned(t) > 0 ? 0 : Agc.toSigned(t);
        imediate(remaining + 1);
    }

    /** IMEDIATE: ADD ONE and transfer to the waitlist/findvac/novac routine at GOLOC-1. */
    public void imediate(int a) {
        agc.set("A", a);
        new Waitlist(agc).waitlist(a, () -> exit.run());
    }

    /** ITSLIKEB: type B restart, job part first, table part afterwards. */
    public void itslikeb() {
        swchExit = this::rtrncadr;
        exit = this::getpart2;
        tempphs(tempphs() & OCT177);
        storeJobCadr();
        itsajob();
    }

    /** ITSAJOB: add the priority and go via CHKNOVAC. */
    public void itsajob() {
        chknovac(agc.get("PHSPRDT1+" + temp2g()));
    }

    /** CHKNOVAC: positive priority means FINDVAC, negative means NOVAC. */
    public void chknovac(int prio) {
        int p = Agc.toSigned(prio);
        Executive ex = new Executive(agc);
        Runnable job = () -> {
            // 2CADR in GOLOC,GOLOC+1 starts the job
            ex.startJob(agc.get("GOLOC"), agc.get("GOLOC+1"));
            exit.run();
        };
        if (p > 0) {
            ex.findvac(p, job);
        } else {
            ex.novac(-p, job);
        }
    }

    /** ITSATBL: table restart; odd phases use the second half of the table. */
    public void itsatbl() {
        int phase = tempphs();
        agc.set("CYR", phase);
        if ((phase & 1) == 0) {
            itseven();
            return;
        }
        exit = this::rtrncadr;
        pointer(2 * phase + agc.get("SIZETAB+" + (temp2g() + 1)));
        contbl2();
    }

    /** CONTBL2: examine what is in the table (job, waitlist or longcall). */
    public void contbl2() {
        int cadr = agc.get("CADRTAB+" + pointer());
        agc.set("GOLOC+1", agc.get("CADRTAB+" + (pointer() + 1)));   // BB information
        if (Agc.toSigned(cadr) > 0) {
            itsajob2(cadr);
            return;
        }
        agc.set("GOLOC", cadr + 1);
        agc.set("GOLOC-1", agc.get("WTLTCADR"));
        if ((agc.get("GOLOC+1") & BIT10) != 0) {
            itswtlst();
        } else {
            itslgcl1(agc.get("PRDTTAB+" + pointer()));
        }
    }

    /** ITSLGCL1: fetch the DP delta time for a longcall (switching EBANK as required). */
    public void itslgcl1(int address) {
        agc.set("L", agc.get("ERASABLE+" + (address + 1)));
        agc.set("A", agc.get("ERASABLE+" + address));
        itslgcl2();
    }

    /** ITSLGCL2: compute the time left on the longcall and restart it. */
    public void itslgcl2() {
        int timeLeft = Agc.toSigned(agc.get("LONGTIME")) * 037777 + Agc.toSigned(agc.get("LONGTIME+1"));
        timeLeft -= Agc.toSigned(agc.get("TIME2")) * 037777 + Agc.toSigned(agc.get("TIME1"));
        timeLeft += Agc.toSigned(agc.get("LONGBASE")) * 037777 + Agc.toSigned(agc.get("LONGBASE+1"));
        if (timeLeft > 0) {
            longclcl();
        } else {
            imediate(1);
        }
    }

    /** LONGCLCL: go to LONGCALL with the remaining time in A,L. */
    public void longclcl() {
        agc.set("GOLOC-1", agc.get("LGCLCADR"));
        new Waitlist(agc).longcall(agc.get("LONGTIME"), agc.get("LONGTIME+1"), () -> exit.run());
    }

    /** ITSLNGCL: a longcall; DT address is negative in PHSPRDT. */
    public void itslngcl() {
        agc.set("GOLOC-1", agc.get("WTLTCADR"));
        itslgcl1(-Agc.toSigned(agc.get("PHSPRDT1+" + temp2g())));
    }

    /** ITSWTLST: waitlist call from a table; correct the BBCON and test how the time is stored. */
    public void itswtlst() {
        agc.set("GOLOC+1", ~agc.get("GOLOC+1") & Agc.WORD_MASK);
        timetest(agc.get("PRDTTAB+" + pointer()));
    }

    /** ITSAJOB2: job from a table; store CADR, add priority and go via CHKNOVAC. */
    public void itsajob2(int cadr) {
        agc.set("GOLOC", cadr);
        chknovac(agc.get("PRDTTAB+" + pointer()));
    }

    /** ITSEVEN: even table phase; set up pointer into the first half of the table. */
    public void itseven() {
        exit = swchExit;
        pointer(agc.get("SIZETAB+" + temp2g()) + 3 * tempphs());
        contbl2();
    }

    /** PHSPART2: second half of an even table; step the pointer and finish the group. */
    public void phspart2() {
        pointer(pointer() + THREE);
        exit = this::rtrncadr;
        contbl2();
    }
}
