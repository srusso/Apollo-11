package apollo.luminary099;

/**
 * RESTART_TABLES.agc (Luminary 1A build 099, pages 238-243).
 *
 * Restart tables, one pair per restart group (EVEN tables hold two three-word entries, ODD tables
 * one). Each entry is a PRDTTAB word followed by a 2CADR (CADRTAB, CADRTAB+1):
 * <ul>
 * <li>Job: PRDTTAB holds the priority (positive = FINDVAC, negative = NOVAC) and the 2CADR the job.</li>
 * <li>Waitlist task: 2CADR stored negatively ("-2CADR"); PRDTTAB positive = delta time (centiseconds),
 *     negative = -GENADR of an erasable DP cell holding the delta time, 77777 (-0) = restart immediately.</li>
 * <li>LONGCALL: GENADR of the DP delta-time register, then -GENADR and BBCON of the long task, then a
 *     priority OCT and a 2CADR for the follow-on job.</li>
 * </ul>
 * Each row below is {opcode, operand}; "EBANK=" rows record the erasable bank declaration.
 */
public final class RestartTables {
    private RestartTables() {}

    /** PRDTTAB relative location (12000 octal). */
    public static final int PRDTTAB = 012000;
    /** CADRTAB relative location (12001 octal); this and the next hold the restart 2CADR. */
    public static final int CADRTAB = 012001;

    /** SIZETAB: table of offsets (TC 1.2SPOT -12006 for even phase groups, -12004 for odd). */
    public static final String[] SIZETAB = {
        "1.2SPOT -12006", "1.3SPOT -12004", "2.2SPOT -12006", "2.3SPOT -12004",
        "3.2SPOT -12006", "3.3SPOT -12004", "4.2SPOT -12006", "4.3SPOT -12004",
        "5.2SPOT -12006", "5.3SPOT -12004", "6.2SPOT -12006", "6.3SPOT -12004"};

    /** 1.2SPOT. */
    public static final String[][] SPOT_1_2 = {
        {"OCT", "21000"},
        {"EBANK=", "STATE"},
        {"2CADR", "ENDOFJOB"},
        {"DEC", "100"},
        {"EBANK=", "STATE"},
        {"2CADR", "TASKOVER"}};

    /** 1.3SPOT. */
    public static final String[][] SPOT_1_3 = {
        {"-GENADR", "SAVET-30"},
        {"EBANK=", "DVCNTR"},
        {"-2CADR", "ULLGTASK"}};

    /** 2.2SPOT EQUALS 1.2SPOT. */
    public static final String[][] SPOT_2_2 = SPOT_1_2;

    /** 2.3SPOT. */
    public static final String[][] SPOT_2_3 = {
        {"GENADR", "600SECS"},
        {"-GENADR", "STATEINT"},
        {"EBANK=", "RRECTCSM"},
        {"BBCON", "STATEINT"}};

    /** 2.5SPOT. */
    public static final String[][] SPOT_2_5 = {
        {"OCT", "05000"},
        {"EBANK=", "RRECTCSM"},
        {"2CADR", "STATINT1"}};

    /** 2.7SPOT. */
    public static final String[][] SPOT_2_7 = {
        {"DEC", "1500"},
        {"EBANK=", "LOSCOUNT"},
        {"-2CADR", "P20LEMC1"}};

    /** 2.11SPOT. */
    public static final String[][] SPOT_2_11 = {
        {"OCT", "14000"},
        {"EBANK=", "P21TIME"},
        {"2CADR", "P25LEM1"}};

    /** 2.13SPOT. */
    public static final String[][] SPOT_2_13 = {
        {"OCT", "10000"},
        {"EBANK=", "LOSCOUNT"},
        {"2CADR", "RELINUS"}};

    /** 2.15SPOT. */
    public static final String[][] SPOT_2_15 = {
        {"OCT", "26000"},
        {"EBANK=", "LOSCOUNT"},
        {"2CADR", "R22RSTRT"}};

    /** 2.17SPOT. */
    public static final String[][] SPOT_2_17 = {
        {"OCT", "77777"},
        {"EBANK=", "VGPREV"},
        {"-2CADR", "REDO2.17"}};

    /** 2.21SPOT. */
    public static final String[][] SPOT_2_21 = {
        {"DEC", "25"},
        {"EBANK=", "DVCNTR"},
        {"-2CADR", "R10,R11"}};

    /** 3.2SPOT EQUALS 1.2SPOT. */
    public static final String[][] SPOT_3_2 = SPOT_1_2;

    /** 3.3SPOT. */
    public static final String[][] SPOT_3_3 = {
        {"-GENADR", "ZOOMTIME"},
        {"EBANK=", "DVCNTR"},
        {"-2CADR", "ZOOM"}};

    /** 3.5SPOT. */
    public static final String[][] SPOT_3_5 = {
        {"OCT", "20000"},
        {"EBANK=", "TTOGO"},
        {"2CADR", "S40.13"}};

    /** 4.2SPOT. */
    public static final String[][] SPOT_4_2 = {
        {"DEC", "2500"},
        {"EBANK=", "TTOGO"},
        {"-2CADR", "TIG-5"},
        {"OCT", "77777"},
        {"EBANK=", "TTOGO"},
        {"-2CADR", "REDO4.2"}};

    /** 4.3SPOT. */
    public static final String[][] SPOT_4_3 = {
        {"OCT", "25000"},
        {"EBANK=", "DVCNTR"},
        {"2CADR", "GOABORT"}};

    /** 4.5SPOT. */
    public static final String[][] SPOT_4_5 = {
        {"DEC", "50"},
        {"EBANK=", "TTOGO"},
        {"-2CADR", "ULLAGOFF"}};

    /** 4.7SPOT. */
    public static final String[][] SPOT_4_7 = {
        {"DEC", "500"},
        {"EBANK=", "DVCNTR"},
        {"-2CADR", "TIG-0"}};

    /** 4.11SPOT. */
    public static final String[][] SPOT_4_11 = {
        {"-GENADR", "TGO +1"},
        {"EBANK=", "DVCNTR"},
        {"-2CADR", "ENGOFTSK"}};

    /** 4.13SPOT. */
    public static final String[][] SPOT_4_13 = {
        {"OCT", "12000"},
        {"EBANK=", "TRKMKCNT"},
        {"2CADR", "POSTBURN"}};

    /** 4.15SPOT. */
    public static final String[][] SPOT_4_15 = {
        {"DEC", "500"},
        {"EBANK=", "TTOGO"},
        {"-2CADR", "TIG-30"}};

    /** 4.17SPOT. */
    public static final String[][] SPOT_4_17 = {
        {"OCT", "77777"},
        {"EBANK=", "DVCNTR"},
        {"-2CADR", "TIG-5"}};

    /** 4.21SPOT. */
    public static final String[][] SPOT_4_21 = {
        {"OCT", "13000"},
        {"EBANK=", "STAR"},
        {"2CADR", "R51.1 +1"}};

    /** 4.23SPOT. */
    public static final String[][] SPOT_4_23 = {
        {"OCT", "77777"},
        {"EBANK=", "DVCNTR"},
        {"-2CADR", "IGNITION"}};

    /** 4.25SPOT. */
    public static final String[][] SPOT_4_25 = {
        {"GENADR", "SAVET-30"},
        {"-GENADR", "TIG-35"},
        {"EBANK=", "SAVET-30"},
        {"BBCON", "TIG-35"}};

    /** 4.27SPOT. */
    public static final String[][] SPOT_4_27 = {
        {"OCT", "52777"},
        {"EBANK=", "DVCNTR"},
        {"2CADR", "P70A"}};

    /** 4.31SPOT. */
    public static final String[][] SPOT_4_31 = {
        {"OCT", "52777"},
        {"EBANK=", "DVCNTR"},
        {"2CADR", "P71A"}};

    /** 4.33SPOT. */
    public static final String[][] SPOT_4_33 = {
        {"OCT", "46777"},
        {"EBANK=", "DVCNTR"},
        {"2CADR", "GOP00FIX"}};

    /** 4.35SPOT. */
    public static final String[][] SPOT_4_35 = {
        {"OCT", "46777"},
        {"EBANK=", "DVCNTR"},
        {"2CADR", "GOPOODOO"}};

    /** 4.37SPOT. */
    public static final String[][] SPOT_4_37 = {
        {"OCT", "52777"},
        {"EBANK=", "WHICH"},
        {"2CADR", "COMFAIL"}};

    /** 5.2SPOT. */
    public static final String[][] SPOT_5_2 = {
        {"OCT", "22000"},
        {"EBANK=", "DVCNTR"},
        {"2CADR", "NORMLIZE"},
        {"DEC", "200"},
        {"EBANK=", "DVCNTR"},
        {"-2CADR", "REREADAC"}};

    /** 5.4SPOT. */
    public static final String[][] SPOT_5_4 = {
        {"DEC", "200"},
        {"EBANK=", "DVCNTR"},
        {"-2CADR", "REREADAC"},
        {"OCT", "20000"},
        {"EBANK=", "DVCNTR"},
        {"2CADR", "SERVICER"}};

    /** 5.3SPOT. */
    public static final String[][] SPOT_5_3 = {
        {"DEC", "200"},
        {"EBANK=", "DVCNTR"},
        {"-2CADR", "REREADAC"}};

    /** 5.5SPOT. */
    public static final String[][] SPOT_5_5 = {
        {"OCT", "77777"},
        {"EBANK=", "DVCNTR"},
        {"-2CADR", "REDO5.5"}};

    /** 5.7SPOT. */
    public static final String[][] SPOT_5_7 = {
        {"OCT", "77777"},
        {"EBANK=", "DVCNTR"},
        {"-2CADR", "BIBIBIAS"}};

    /** 6.2SPOT EQUALS 1.2SPOT. */
    public static final String[][] SPOT_6_2 = SPOT_1_2;

    /** 6.3SPOT. */
    public static final String[][] SPOT_6_3 = {
        {"DEC", "100"},
        {"EBANK=", "TIG"},
        {"-2CADR", "CLOKTASK"}};

    /** 6.5SPOT. */
    public static final String[][] SPOT_6_5 = {
        {"OCT", "30000"},
        {"EBANK=", "TEPHEM"},
        {"2CADR", "TIMEDIDR"}};

    /** 6.7SPOT. */
    public static final String[][] SPOT_6_7 = {
        {"OCT", "17000"},
        {"EBANK=", "VGPREV"},
        {"2CADR", "REDO6.7"}};

}
