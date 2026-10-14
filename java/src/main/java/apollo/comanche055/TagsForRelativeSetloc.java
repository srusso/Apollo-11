package apollo.comanche055;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * TAGS_FOR_RELATIVE_SETLOC.agc (Comanche 055, pages 27-36).
 *
 * Tags for relative SETLOC and blank bank cards: assigns each symbolic fixed-memory
 * location tag (used by SETLOC in the other source files) to the fixed-memory bank
 * it lives in. Fixed memory is 120000 - 167777 octal in 36K words; bank numbers are octal
 * values shown here as Java octal literals. Banks 02 and 03 are the fixed-fixed blocks.
 * The module split is: module 1 = banks 0-5, module 2 = 6-13, module 3 = 14-21,
 * module 4 = 22-27, module 5 = 30-35, module 6 = 36-43.
 *
 * Also defines the EBANK-switching ECADR tags (EBMARKDO etc.) and THISAXIS = UNITX.
 */
public final class TagsForRelativeSetloc {
    private TagsForRelativeSetloc() {}

    /** Tag name -> fixed memory bank number (octal in the source). */
    public static final Map<String, Integer> BANK_OF_TAG;

    static {
        Map<String, Integer> m = new LinkedHashMap<>();
        m.put("FFTAG1", 02);
        m.put("FFTAG2", 02);
        m.put("FFTAG3", 02);
        m.put("FFTAG4", 02);
        m.put("FFTAG7", 02);
        m.put("FFTAG8", 02);
        m.put("FFTAG9", 02);
        m.put("FFTAG10", 02);
        m.put("FFTAG12", 02);
        m.put("P30SUBS", 02);
        m.put("STOPRAT", 02);
        m.put("P23S", 02);
        m.put("FFTAG5", 03);
        m.put("FFTAG6", 03);
        m.put("DAPS9", 03);
        m.put("FFTAG13", 03);
        m.put("DLAYJOB", 00);
        m.put("RESTART", 01);
        m.put("VERB37", 04);
        m.put("CONICS1", 04);
        m.put("PINBALL4", 04);
        m.put("CSI/CDH1", 04);
        m.put("INTPRET2", 04);
        m.put("IMUCAL1", 04);
        m.put("STBLEORB", 04);
        m.put("E/PROG", 04);
        m.put("MIDDGIM", 04);
        m.put("FRANDRES", 05);
        m.put("DOWNTELM", 05);
        m.put("DAPMASS", 05);
        m.put("CDHTAG", 05);
        m.put("IMUCOMP", 06);
        m.put("T4RUP", 06);
        m.put("IMUCAL2", 06);
        m.put("CSIPROG", 06);
        m.put("SXTMARKE", 07);
        m.put("R02", 07);
        m.put("MODESW", 07);
        m.put("XANG", 07);
        m.put("KEYRUPT", 07);
        m.put("CSIPROG6", 07);
        m.put("DISPLAYS", 010);
        m.put("PHASETAB", 010);
        m.put("COMGEOM2", 010);
        m.put("SXTMARK1", 010);
        m.put("P60S4", 010);
        m.put("OPTDRV", 010);
        m.put("CSIPROG8", 010);
        m.put("ORBITAL", 011);
        m.put("ORBITAL1", 011);
        m.put("INTVEL", 011);
        m.put("S52/2", 011);
        m.put("CSIPROG5", 011);
        m.put("INTINIT1", 011);
        m.put("CONICS", 012);
        m.put("CSIPROG2", 012);
        m.put("CSI/CDH2", 012);
        m.put("MODCHG2", 012);
        m.put("P76LOC", 013);
        m.put("LATLONG", 013);
        m.put("INTINIT", 013);
        m.put("SR52/1", 013);
        m.put("ORBITAL2", 013);
        m.put("CDHTAGS", 013);
        m.put("E/PROG1", 013);
        m.put("MODCHG3", 013);
        m.put("STARTAB", 014);
        m.put("RT53", 014);
        m.put("P50S1", 014);
        m.put("MEASINC2", 014);
        m.put("CSI/CDH3", 014);
        m.put("P50S", 015);
        m.put("ETRYDAP", 015);
        m.put("S52/3", 015);
        m.put("P40S1", 016);
        m.put("DAPROLL", 016);
        m.put("P50S2", 016);
        m.put("P23S1", 016);
        m.put("RTE2", 016);
        m.put("DAPS4", 017);
        m.put("DAPS5", 017);
        m.put("DAPS7", 017);
        m.put("P50S3", 017);
        m.put("DAPS6", 020);
        m.put("DAPS1", 020);
        m.put("DAPS2", 020);
        m.put("MANUSTUF", 020);
        m.put("R36CM", 020);
        m.put("VAC5LOC", 020);
        m.put("DAPS3", 021);
        m.put("MYSUBS", 021);
        m.put("KALCMON3", 021);
        m.put("RTBCODES", 022);
        m.put("RTBCODE1", 022);
        m.put("DAPS8", 022);
        m.put("APOPERI", 022);
        m.put("P40S5", 022);
        m.put("KALCMON2", 022);
        m.put("KALCMON1", 022);
        m.put("CSIPROG3", 022);
        m.put("P20S2", 023);
        m.put("INFLIGHT", 023);
        m.put("COMGEOM1", 023);
        m.put("POWFLITE", 023);
        m.put("POWFLIT1", 023);
        m.put("RENDGUID", 023);
        m.put("POWFLIT2", 023);
        m.put("R30LOC", 023);
        m.put("P11FOUR", 023);
        m.put("CSIPROG4", 023);
        m.put("LOADDAP", 024);
        m.put("P40S", 024);
        m.put("CSIPROG7", 024);
        m.put("REENTRY", 025);
        m.put("CDHTAG1", 025);
        m.put("INTPRET1", 026);
        m.put("REENTRY1", 026);
        m.put("P60S", 026);
        m.put("P60S1", 026);
        m.put("P60S2", 026);
        m.put("P60S3", 026);
        m.put("PLANTIN", 026);
        m.put("EPHEM", 026);
        m.put("P05P06", 026);
        m.put("26P50S", 026);
        m.put("TOF-FF", 027);
        m.put("TOF-FF1", 027);
        m.put("MANUVER", 027);
        m.put("MANUVER1", 027);
        m.put("VECPT", 027);
        m.put("UPDATE1", 027);
        m.put("UPDATE2", 027);
        m.put("R22S1", 027);
        m.put("P60S5", 027);
        m.put("P40S2", 027);
        m.put("IMUSUPER", 030);
        m.put("LOWSUPER", 030);
        m.put("FCSTART", 030);
        m.put("LOPC", 030);
        m.put("P20S1", 030);
        m.put("P20S6", 030);
        m.put("P40S3", 030);
        m.put("R35A", 030);
        m.put("R35", 031);
        m.put("RT23", 031);
        m.put("P30S1A", 031);
        m.put("R34", 031);
        m.put("CDHTAG2", 031);
        m.put("CSIPROG9", 031);
        m.put("R31", 031);
        m.put("P22S", 031);
        m.put("RTE3", 031);
        m.put("MSGSCAN1", 032);
        m.put("RTE", 032);
        m.put("DELRSPL1", 032);
        m.put("IMUCAL3", 032);
        m.put("TESTLEAD", 033);
        m.put("IMUCAL", 033);
        m.put("P11ONE", 034);
        m.put("P20S3", 034);
        m.put("P20S4", 034);
        m.put("RTECON", 034);
        m.put("RTECON1", 035);
        m.put("CSI/CDH", 035);
        m.put("P30S1", 035);
        m.put("P30S", 035);
        m.put("P17S1", 035);
        m.put("MEASINC3", 035);
        m.put("INTINIT2", 035);
        m.put("MEASINC", 036);
        m.put("MEASINC1", 036);
        m.put("P17S", 036);
        m.put("RTE1", 036);
        m.put("S3435LOC", 036);
        m.put("P20S", 037);
        m.put("BODYATT", 037);
        m.put("RENDEZ", 037);
        m.put("SERVICES", 037);
        m.put("P11TWO", 037);
        m.put("CDHTAG3", 037);
        m.put("PINSUPER", 040);
        m.put("SELFSUPR", 040);
        m.put("PINBALL1", 040);
        m.put("R36CM1", 040);
        m.put("PINBALL2", 041);
        m.put("R36LM", 041);
        m.put("SBAND", 042);
        m.put("PINBALL3", 042);
        m.put("EXTVBS", 042);
        m.put("R36LM1", 042);
        m.put("SELFCHEC", 043);
        m.put("EXTVERBS", 043);
        BANK_OF_TAG = Collections.unmodifiableMap(m);
    }

    /** THISAXIS = UNITX (rope specific assign avoiding the computer-flag check in integration). */
    public static final String THISAXIS = "UNITX";

    // ECADRs defined to facilitate EBANK switching: name -> {fixed bank, erasable symbol}.
    public static final String[][] ECADR_TAGS = {
        {"EBMARKDO", "07", "MARKDOWN"},
        {"EBMRKBUF", "07", "MRKBUF1"},
        {"EBDVCNTR", "24", "DVCNTR"},
        {"EBP40TMP", "24", "P40TMP"},
        {"EBDVCNT", "34", "DVCNTR"},
        {"EBQPLACE", "34", "QPLACES"},
        {"EBRN1", "37", "RN1"},
    };

    /** Returns the bank holding a tag, or -1 if unknown. */
    public static int bankOf(String tag) {
        return BANK_OF_TAG.getOrDefault(tag, -1);
    }
}
