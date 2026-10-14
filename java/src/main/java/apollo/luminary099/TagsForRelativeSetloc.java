package apollo.luminary099;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * TAGS_FOR_RELATIVE_SETLOC: assigns relative-SETLOC tags to fixed-memory banks/blocks.
 * Each entry maps a BANK/BLOCK declaration to the tags (EQUALS labels) defined at its start.
 */
public final class TagsForRelativeSetloc {
    private TagsForRelativeSetloc() {}

    /** Bank or block specifier mapped to the tags it defines (in source order). */
    public static final Map<String, List<String>> BANK_TAGS = new LinkedHashMap<>();

    static {
        BANK_TAGS.merge("BLOCK 02", List.of("RADARFF", "FFTAG1", "FFTAG2", "FFTAG3", "FFTAG4", "FFTAG7", "FFTAG8", "FFTAG9", "FFTAG10", "FFTAG11", "FFTAG12", "FFTAG13"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BLOCK 03", List.of("FFTAG5", "FFTAG6"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 00", List.of("DLAYJOB"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 01", List.of("RESTART", "LOADDAP1"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 04", List.of("R02", "VERB37", "PINBALL4", "CONICS1", "KEYRUPT", "R36LM", "UPDATE2", "E/PROG", "AOTMARK2"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 05", List.of("FRANDRES", "DOWNTELM", "ABORTS1", "EPHEM1", "ASENT3"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 06", List.of("IMUCOMP", "T4RUP", "RCSMONT", "MIDDGIM", "EARTHLOC"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 07", List.of("AOTMARK1", "MODESW", "ASENT2"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 10", List.of("RTBCODES", "DISPLAYS", "PHASETAB", "FLESHLOC", "SLCTMU"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 11", List.of("ORBITAL", "F2DPS*11", "INTVEL"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 12", List.of("CONICS", "ORBITAL1", "INTPRET2"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 13", List.of("LATLONG", "INTINIT", "LEMGEOM", "P76LOC", "ORBITAL2", "ABTFLGS"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 14", List.of("P50S1", "STARTAB", "ASENT4"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 15", List.of("P50S", "EPHEM"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 16", List.of("DAPS1"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 17", List.of("DAPS2", "P40S3"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 20", List.of("DAPS3", "LOADDAP", "RODTRAP"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 21", List.of("DAPS4", "R10", "R11"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 22", List.of("KALCMON1", "KALCMON2", "R30LOC", "RENDEZ", "SERV2", "LANDCNST"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 23", List.of("POWFLITE", "POWFLIT1", "INFLIGHT", "APOPERI", "R61", "R62", "INTPRET1", "MEASINC", "MEASINC1", "EXTVB1", "P12A", "NORMLIZ", "ASENT7"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 24", List.of("PLANTIN", "P20S"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 25", List.of("P20S1", "P20S2", "RADARUPT", "RRLEADIN", "R29S1", "PLANTIN3"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 26", List.of("P20S3", "BAWLANGS", "MANUVER", "MANUVER1", "PLANTIN1", "PLANTIN2"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 27", List.of("TOF-FF", "TOF-FF1", "P40S1", "VECPT", "ASENT1", "SERV3"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 30", List.of("LOWSUPER", "P12", "ASENT", "FCDUW", "FLOGSUB", "VB67A", "ASENT5"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 31", List.of("FTHROT", "F2DPS*31", "VB67"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 32", List.of("P20S4", "F2DPS*32", "ABORTS", "LRS22", "P66LOC", "R47", "SERV"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 33", List.of("SERVICES", "R29/SERV", "ASENT6"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 34", List.of("STBLEORB", "P30S1", "CSI/CDH1", "ASCFILT", "R12STUFF", "SERV4"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 35", List.of("CSI/CDH", "P30S", "GLM", "P40S2"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 36", List.of("P40S"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 37", List.of("P05P06", "IMU2", "IMU4", "R31", "IMUSUPER", "SERV1"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 40", List.of("PINBALL1", "SELFSUPR", "PINSUPER", "R31LOC"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 41", List.of("PINBALL2"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 42", List.of("SBAND", "PINBALL3"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
        BANK_TAGS.merge("BANK 43", List.of("EXTVERBS", "SELFCHEC", "HI6ZEROS", "LO6ZEROS", "HIDPHALF", "LODPHALF", "HIDP1/4", "LODP1/4", "HIUNITX", "HIUNITY", "HIUNITZ", "LOUNITX", "LOUNITY", "LOUNITZ", "DELRSPL", "ATOPTHIS", "ATOPOTH", "OTHPREC", "MOONTHIS", "MOONOTH", "MOVATHIS", "THISPREC", "NB1NB2", "ERASID", "DELAYNUM"), (a, c) -> { var m = new java.util.ArrayList<>(a); m.addAll(c); return m; });
    }
}
