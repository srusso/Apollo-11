package apollo.luminary099;

import apollo.agc.Agc;

/**
 * DAP_INTERFACE_SUBROUTINES (Luminary 099): deadband restoration, rate/error zeroing
 * and coast set-up used by the descent/ascent DAP interface.
 */
public class DapInterfaceSubroutines {
    /** 0.3 degree scaled at 45. */
    public static final int NARROWDB = 0155;
    /** 5.0 degrees scaled at 45. */
    public static final int WIDEDB = 03434;
    /** 1.0 degree scaled at 45 (DEC .02222 * 2^14 ~ 364). */
    public static final int POWERDB = (int) Math.round(0.02222 * 16384);

    private final Agc agc;

    public DapInterfaceSubroutines(Agc agc) {
        this.agc = agc;
    }

    /** RESTORDB: restore the crew-selected deadband (DBSELECT bit of DAPBOOLS) and queue 1/ACCJOB. */
    public void restordb() {
        if ((agc.get("DAPBOOLS") & agc.get("DBSELECT")) == 0) {
            setmindb();
        } else {
            setmaxdb();
        }
    }

    /** SETMAXDB: set 5 degree deadband, then CALLACCS. */
    public void setmaxdb() {
        agc.set("DB", WIDEDB);
        callaccs();
    }

    /** SETMINDB: set 0.3 degree deadband (TCF SETMAXDB +1). */
    public void setmindb() {
        agc.set("DB", NARROWDB);
        callaccs();
    }

    /** CALLACCS: schedule job 1/ACCJOB at priority 27 (NOVAC) and return to caller. */
    public void callaccs() {
        // NOVAC with PRIO27, 2CADR 1/ACCJOB (EBANK= AOSQ)
        // Executive NOVAC call is modelled as a call to 1/ACCJOB.
        agc.set("JOBREQ_1ACCJOB_PRIO", 027);
    }

    /** PFLITEDB: powered flight deadband of 1.0 degree after zeroing errors/rates. */
    public void pfliteDb() {
        zatteror();
        agc.set("DB", POWERDB);
        callaccs();
    }

    /** ZATTEROR: copy CDUs into CDU desired registers then zero rates and errors. */
    public void zatteror() {
        agc.set("CDUXD", agc.get("CDUX"));
        agc.set("CDUYD", agc.get("CDUY"));
        agc.set("CDUZD", agc.get("CDUZ"));
        stoprate3();
    }

    /** STOPRATE: zero commanded rates and attitude errors. */
    public void stoprate() {
        stoprate3();
    }

    /** STOPRATE +3. */
    private void stoprate3() {
        for (String s : new String[] {"OMEGAPD", "OMEGAQD", "OMEGARD", "DELCDUX", "DELCDUY",
                "DELCDUZ", "DELPEROR", "DELQEROR", "DELREROR"}) {
            agc.set(s, 0);
        }
    }

    /** ALLCOAST: clear rate interface, zero jet-on-time terms, set DRIFTBIT, restore deadband. */
    public void allcoast() {
        stoprate();
        for (String s : new String[] {"AOSQ", "AOSQ+1", "AOSR", "AOSR+1", "ALPHAQ", "ALPHAR",
                "AOSQTERM", "AOSRTERM"}) {
            agc.set(s, 0);
        }
        // CS DAPBOOLS; MASK DRIFTBIT; ADS DAPBOOLS  -> set DRIFTBIT
        agc.set("DAPBOOLS", agc.get("DAPBOOLS") | agc.get("DRIFTBIT"));
        // TC RESTORDB +1: restore crew-selected deadband
        restordb();
    }
}
