package apollo.luminary099;

import apollo.agc.Agc;

/**
 * Interrupt lead-ins (fixed-fixed, SETLOC 4000, $$/RUPTS). Each vector saves A/L/Q state in ARUPT and
 * transfers to the handler with the appropriate BBANK (bank+EBANK) setting. Here each vector is a method
 * that dispatches to the handler class by naming convention.
 */
public class InterruptLeadIns {
    private final Agc agc;

    /** Vector numbers (offset = 4 * n from location 4000). */
    public static final int GO = 0, T6RUPT = 1, T5RUPT = 2, T3RUPT = 3, T4RUPT = 4,
            KEYRUPT1 = 5, KEYRUPT2 = 6, UPRUPT = 7, DOWNRUPT = 8, RADAR_RUPT = 9, RUPT10 = 10;

    /** BBCON constants naming handler/EBANK pairs. */
    public static final String GOBB = "GOPROG";
    public static final String T6ADR = "DOT6RUPT";     // 2CADR, EBANK=PERROR
    public static final String T3RPTBB = "T3RUPT";     // EBANK=LST1
    public static final String KEYRPTBB = "KEYRUPT1";  // EBANK=KEYTEMP1
    public static final String MKRUPTBB = "MARKRUPT";  // EBANK=AOTAZ
    public static final String UPRPTBB = KEYRPTBB;     // UPRUPT shares KEYRPTBB
    public static final String DWNRPTBB = "DODOWNTM";  // EBANK=DNTMBUFF
    public static final String RDRPTBB = "RADAREAD";   // EBANK=RADMODES
    public static final String T4RPTBB = "T4RUPT";     // EBANK=M11
    public static final String RUPT10BB = "PITFALL";   // EBANK=ELVIRA

    public InterruptLeadIns(Agc agc) {
        this.agc = agc;
    }

    /** Saves interrupted A,L (DXCH ARUPT) before dispatch; INHINT is implied on entry. */
    private void enter() { /* INHINT; DXCH ARUPT */ }

    /** Location 4000: GO (restart) -- INHINT; BBANK = GOBB; TCF GOPROG. */
    public void go() { new FreshStartAndRestart(agc).goProg(); }

    /** T6RUPT: DXCH ARUPT; DCA T6ADR; DTCB. */
    public void t6rupt() { enter(); new T6RuptPrograms(agc).doT6Rupt(); }

    /** T5RUPT (autopilot): DXCH ARUPT; DCA T5ADR; DTCB. */
    public void t5rupt() { enter(); new DapidlerProgram(agc).dapidler(); }

    /** T3RUPT. */
    public void t3rupt() { enter(); new Waitlist(agc).t3Rupt(); }

    /** T4RUPT. */
    public void t4rupt() { enter(); new T4ruptProgram(agc).t4rupt(); }

    /** KEYRUPT1 (DSKY keyboard). */
    public void keyrupt1() { enter(); new KeyruptUprupt(agc).keyrupt1(); }

    /** KEYRUPT2 = MARKRUPT. */
    public void markrupt() { enter(); new AotMark(agc).markRupt(); }

    /** UPRUPT (uplink). */
    public void uprupt() { enter(); new KeyruptUprupt(agc).uprupt(); }

    /** DOWNRUPT. */
    public void downrupt() { enter(); new DownTelemetryProgram(agc).dodowntm(); }

    /** RADAR RUPT. */
    public void radarRupt() { enter(); new P20P25(agc).radaread(); }

    /** RUPT10, used only by landing guidance (PITFALL). */
    public void rupt10() { enter(); new LunarLandingGuidanceEquations(agc).pitfall(); }
}
