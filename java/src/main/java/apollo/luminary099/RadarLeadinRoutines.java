package apollo.luminary099;

import apollo.agc.Agc;

/**
 * RADAR_LEADIN_ROUTINES (Luminary 099, bank 25, RRLEADIN): cyclic radar-data sampling
 * (RADSAMP / DORSAMP) used by the radar self-test (R77-style) logic.
 */
public class RadarLeadinRoutines {
    public static final int PRIO25 = 025;
    /** RDRLOCS: radar routines indexed by RTSTDEX. */
    public static final String[] RDRLOCS = {
        "RRRANGE",  // 0
        "RRRDOT",   // 1
        "LRVELX",   // 2
        "LRVELY",   // 3
        "LRVELZ",   // 4
        "LRALT"     // 5
    };

    private final Agc agc;

    public RadarLeadinRoutines(Agc agc) {
        this.agc = agc;
    }

    /** RADSAMP: waitlist task, repeats every RSAMPDT; +0 terminates the test. Starts DORSAMP as a prio-25 job. */
    public void radsamp() {
        int rsampdt = Agc.toSigned(agc.get("RSAMPDT"));
        if (rsampdt == 0) {
            return;                  // TASKOVER
        }
        // TC WAITLIST (2CADR RADSAMP) - reschedule self after RSAMPDT
        // CAF PRIO25; TC NOVAC (2CADR DORSAMP)
        dorsamp();
        // RTSTDEX = RTSTLOC/2 + RTSTBASE (0 for RR, 2 for LR)
        int rtstdex = (Agc.toSigned(agc.get("RTSTLOC")) >> 1) + Agc.toSigned(agc.get("RTSTBASE"));
        agc.set("RTSTDEX", Agc.fromSigned(rtstdex));
        // TCF TASKOVER
    }

    /** DORSAMP: select the radar channel, wait for data, accept bad data but count failure. */
    public void dorsamp() {
        varadar();
        // TC BANKCALL / CADR RADSTALL: wait for radar read
        agc.set("RFAILCNT", Agc.add(agc.get("RFAILCNT"), 1));   // advance fail counter, but accept bad data
        dorsamp2();
    }

    /** DORSAMP2: store the sample unless in R77, and cycle RTSTLOC. */
    public void dorsamp2() {
        boolean r77 = (agc.get("FLAGWRD5") & agc.get("R77FLBIT")) != 0;   // don't update RSTACK in R77
        if (!r77) {
            int loc = Agc.toSigned(agc.get("RTSTLOC"));
            agc.set("RSTACK+" + loc, agc.get("SAMPLSUM"));
            agc.set("RSTACK+" + (loc + 1), agc.get("SAMPLSUM+1"));
        }
        int loc = Agc.toSigned(agc.get("RTSTLOC"));
        if (loc - Agc.toSigned(agc.get("RTSTMAX")) != 0) {
            agc.set("RTSTLOC", Agc.fromSigned(loc + 2));       // storage is DP
        } else {
            agc.set("RTSTLOC", 0);
        }
        // TCF ENDOFJOB
    }

    /** VARADAR: select the variable radar routine via RDRLOCS[RTSTDEX] and SWCALL it with A=1 in BUF2. */
    public String varadar() {
        agc.set("BUF2", 1);
        return RDRLOCS[agc.get("RTSTDEX")];
    }
}
