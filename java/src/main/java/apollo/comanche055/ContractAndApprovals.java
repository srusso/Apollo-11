package apollo.comanche055;

import apollo.agc.Agc;

/**
 * CONTRACT_AND_APPROVALS.agc - Comanche 055 (Colossus 2A) title page.
 *
 * Pure documentation: "Assemble revision 055 of AGC program Comanche by NASA 2021113-051,
 * 10:28 APR. 1, 1969". Prepared by MIT Instrumentation Laboratory under NASA contract NAS 9-4065
 * (DSR project 55-23870) for the Command Module as specified in report R-577.
 */
public final class ContractAndApprovals {
    public static final String PROGRAM = "COMANCHE 055 (COLOSSUS 2A)";
    public static final String REVISION = "055";
    public static final String NASA_PART_NUMBER = "2021113-051";
    public static final String ASSEMBLY_DATE = "APR. 1, 1969";
    public static final String CONTRACT = "NAS 9-4065";
    public static final String REPORT = "R-577";
    public static final String SUBMITTED_DATE = "28 MAR 69";

    /** Submitted and approved by (all dated 28 MAR 69). */
    public static final String[][] APPROVALS = {
        {"SUBMITTED", "MARGARET H. HAMILTON", "COLOSSUS PROGRAMMING LEADER"},
        {"APPROVED", "DANIEL J. LICKLY", "DIRECTOR, MISSION PROGRAM DEVELOPMENT"},
        {"APPROVED", "FRED H. MARTIN", "COLOSSUS PROJECT MANAGER"},
        {"APPROVED", "NORMAN E. SEARS", "DIRECTOR, MISSION DEVELOPMENT"},
        {"APPROVED", "RICHARD H. BATTIN", "DIRECTOR, MISSION DEVELOPMENT"},
        {"APPROVED", "DAVID G. HOAG", "DIRECTOR, APOLLO GUIDANCE AND NAVIGATION PROGRAM"},
        {"APPROVED", "RALPH R. RAGAN", "DEPUTY DIRECTOR, INSTRUMENTATION LABORATORY"},
    };

    private final Agc agc;

    public ContractAndApprovals(Agc agc) {
        this.agc = agc;
    }
}
