package apollo.comanche055;

import apollo.agc.Agc;
import java.util.List;

/**
 * DOWNLINK_LISTS.agc (Comanche055): telemetry downlink list definitions (bank 22, DOWNTELM).
 *
 * <p>Special downlink op codes: nDNADR (n = 1..6) sends 2n AGC words starting at an erasable
 * address; DNCHAN sends two channels; DNPTR points to the next (sub)list. A leading '-' on the
 * op marks the end of a list. A snapshot sublist is one that starts with a -1DNADR.
 * Erasable addresses are referred to symbolically (plus word offset).
 */
public final class DownlinkLists {
    private final Agc agc;

    public DownlinkLists(Agc agc) {
        this.agc = agc;
    }

    /** One downlink list entry: op code (e.g. "1DNADR", "-DNPTR"), symbol, and word offset. */
    public record Entry(String op, String symbol, int offset) {
        /** True if this entry terminates its list (op begins with '-'). */
        public boolean isEnd() {
            return op.startsWith("-");
        }
    }

    private static Entry e(String op, String sym, int off) {
        return new Entry(op, sym, off);
    }

    /** LOWIDCOD: OCT 77340 (decimal 32480), the low ID code. */
    public static final int LOWIDCOD = 32480;
    /** ERASZERO / SPARE: EQUALS 7; SPARE indicates available space. */
    public static final String SPARE = "ERASZERO";

    /** CMPOWEDL */
    public static final List<Entry> CMPOWEDL = List.of(
        e("DNPTR", "CMPOWE01", 0),
        e("6DNADR", "DNTMBUFF", 0),
        e("DNPTR", "CMPOWE02", 0),
        e("4DNADR", "DNTMBUFF", 0),
        e("DNPTR", "CMPOWE03", 0),
        e("1DNADR", "TIG", 0),
        e("1DNADR", "DELLT4", 0),
        e("3DNADR", "RTARG", 0),
        e("1DNADR", "TGO", 0),
        e("1DNADR", "PIPTIME1", 0),
        e("3DNADR", "DELV", 0),
        e("1DNADR", "PACTOFF", 0),
        e("1DNADR", "PCMD", 0),
        e("1DNADR", "CSTEER", 0),
        e("3DNADR", "DELVEET1", 0),
        e("6DNADR", "REFSMMAT", 0),
        e("DNPTR", "CMPOWE04", 0),
        e("1DNADR", "TIME2", 0),
        e("DNPTR", "CMPOWE05", 0),
        e("6DNADR", "DNTMBUFF", 0),
        e("DNPTR", "CMPOWE02", 0),
        e("4DNADR", "DNTMBUFF", 0),
        e("DNPTR", "CMPOWE03", 0),
        e("DNPTR", "CMPOWE06", 0),
        e("1DNADR", "ELEV", 0),
        e("1DNADR", "CENTANG", 0),
        e("1DNADR", "DELTAR", 0),
        e("1DNADR", "STATE", 10),
        e("1DNADR", "TEVENT", 0),
        e("1DNADR", "PCMD", 0),
        e("1DNADR", "OPTMODES", 0),
        e("DNPTR", "CMPOWE07", 0),
        e("3DNADR", "VGTIG", 0),
        e("-3DNADR", "DELVEET2", 0)
    );

    /** CMPOWE01 */
    public static final List<Entry> CMPOWE01 = List.of(
        e("-1DNADR", "RN", 2),
        e("1DNADR", "RN", 4),
        e("1DNADR", "VN", 0),
        e("1DNADR", "VN", 2),
        e("1DNADR", "VN", 4),
        e("1DNADR", "PIPTIME", 0),
        e("-1DNADR", "RN", 0)
    );

    /** CMPOWE02 */
    public static final List<Entry> CMPOWE02 = List.of(
        e("-1DNADR", "CDUZ", 0),
        e("1DNADR", "ADOT", 0),
        e("1DNADR", "ADOT", 2),
        e("1DNADR", "ADOT", 4),
        e("-1DNADR", "CDUX", 0)
    );

    /** CMPOWE03 */
    public static final List<Entry> CMPOWE03 = List.of(
        e("2DNADR", "AK", 0),
        e("-2DNADR", "THETADX", 0)
    );

    /** CMPOWE04 */
    public static final List<Entry> CMPOWE04 = List.of(
        e("5DNADR", "STATE", 0),
        e("-6DNADR", "DSPTAB", 0)
    );

    /** CMPOWE05 */
    public static final List<Entry> CMPOWE05 = List.of(
        e("-1DNADR", "R-OTHER", 2),
        e("1DNADR", "R-OTHER", 4),
        e("1DNADR", "V-OTHER", 0),
        e("1DNADR", "V-OTHER", 2),
        e("1DNADR", "V-OTHER", 4),
        e("1DNADR", "T-OTHER", 0),
        e("-1DNADR", "R-OTHER", 0)
    );

    /** CMPOWE06 */
    public static final List<Entry> CMPOWE06 = List.of(
        e("1DNADR", "RSBBQ", 0),
        e("3DNADR", "CADRFLSH", 0),
        e("-2DNADR", "CDUS", 0)
    );

    /** CMPOWE07 */
    public static final List<Entry> CMPOWE07 = List.of(
        e("1DNADR", "LEMMASS", 0),
        e("1DNADR", "DAPDATR1", 0),
        e("2DNADR", "ERRORX", 0),
        e("3DNADR", "WBODY", 0),
        e("2DNADR", "REDOCTR", 0),
        e("1DNADR", "IMODES30", 0),
        e("DNCHAN", "11", 0),
        e("DNCHAN", "13", 0),
        e("DNCHAN", "30", 0),
        e("-DNCHAN", "32", 0)
    );

    /** CMCSTADL */
    public static final List<Entry> CMCSTADL = List.of(
        e("DNPTR", "CMCSTA01", 0),
        e("6DNADR", "DNTMBUFF", 0),
        e("DNPTR", "CMCSTA02", 0),
        e("4DNADR", "DNTMBUFF", 0),
        e("DNPTR", "CMCSTA03", 0),
        e("1DNADR", "TIG", 0),
        e("1DNADR", "BESTI", 0),
        e("4DNADR", "MARKDOWN", 0),
        e("4DNADR", "MARK2DWN", 0),
        e("2DNADR", "HAPOX", 0),
        e("1DNADR", "PACTOFF", 0),
        e("3DNADR", "VGTIG", 0),
        e("6DNADR", "REFSMMAT", 0),
        e("DNPTR", "CMCSTA04", 0),
        e("1DNADR", "TIME2", 0),
        e("DNPTR", "CMCSTA05", 0),
        e("6DNADR", "DNTMBUFF", 0),
        e("DNPTR", "CMCSTA02", 0),
        e("4DNADR", "DNTMBUFF", 0),
        e("DNPTR", "CMCSTA03", 0),
        e("DNPTR", "CMCSTA06", 0),
        e("3DNADR", "OGC", 0),
        e("1DNADR", "STATE", 10),
        e("1DNADR", "TEVENT", 0),
        e("1DNADR", "LAUNCHAZ", 0),
        e("1DNADR", "OPTMODES", 0),
        e("DNPTR", "CMCSTA07", 0),
        e("-6DNADR", "DSPTAB", 0)
    );

    /** CMRENDDL */
    public static final List<Entry> CMRENDDL = List.of(
        e("DNPTR", "CMREND01", 0),
        e("6DNADR", "DNTMBUFF", 0),
        e("DNPTR", "CMREND02", 0),
        e("4DNADR", "DNTMBUFF", 0),
        e("DNPTR", "CMREND03", 0),
        e("1DNADR", "TIG", 0),
        e("1DNADR", "DELLT4", 0),
        e("3DNADR", "RTARG", 0),
        e("1DNADR", "VHFTIME", 0),
        e("4DNADR", "MARKDOWN", 0),
        e("1DNADR", "VHFCNT", 0),
        e("1DNADR", "TTPI", 0),
        e("1DNADR", "ECSTEER", 0),
        e("1DNADR", "DELVTPF", 0),
        e("2DNADR", "TCDH", 0),
        e("1DNADR", "TPASS4", 0),
        e("3DNADR", "DELVSLV", 0),
        e("2DNADR", "RANGE", 0),
        e("DNPTR", "CMREND04", 0),
        e("1DNADR", "TIME2", 0),
        e("DNPTR", "CMREND05", 0),
        e("6DNADR", "DNTMBUFF", 0),
        e("DNPTR", "CMREND02", 0),
        e("4DNADR", "DNTMBUFF", 0),
        e("DNPTR", "CMREND03", 0),
        e("DNPTR", "CMREND06", 0),
        e("1DNADR", "DIFFALT", 0),
        e("1DNADR", "CENTANG", 0),
        e("1DNADR", "DELTAR", 0),
        e("3DNADR", "DELVEET3", 0),
        e("1DNADR", "OPTMODES", 0),
        e("DNPTR", "CMREND07", 0),
        e("1DNADR", "RTHETA", 0),
        e("2DNADR", "LAT(SPL)", 0),
        e("2DNADR", "VPRED", 0),
        e("-1DNADR", "STATE", 10)
    );

    /** CMENTRDL */
    public static final List<Entry> CMENTRDL = List.of(
        e("DNPTR", "CMENTR01", 0),
        e("6DNADR", "DNTMBUFF", 0),
        e("DNPTR", "CMENTR02", 0),
        e("4DNADR", "DNTMBUFF", 0),
        e("DNPTR", "CMENTR03", 0),
        e("2DNADR", "CMDAPMOD", 0),
        e("1DNADR", "L/D1", 0),
        e("6DNADR", "UPBUFF", 0),
        e("4DNADR", "UPBUFF", 12),
        e("2DNADR", "COMPNUMB", 0),
        e("1DNADR", "PAXERR1", 0),
        e("3DNADR", "LATANG", 0),
        e("2DNADR", "LAT(SPL)", 0),
        e("1DNADR", "ALFA/180", 0),
        e("DNPTR", "CMENTR04", 0),
        e("1DNADR", "TIME2", 0),
        e("DNPTR", "CMENTR05", 0),
        e("6DNADR", "DNTMBUFF", 0),
        e("DNPTR", "CMENTR02", 0),
        e("4DNADR", "DNTMBUFF", 0),
        e("2DNADR", "AK", 0),
        e("3DNADR", "ERRORX", 0),
        e("2DNADR", "CMDAPMOD", 0),
        e("6DNADR", "UPBUFF", 0),
        e("4DNADR", "UPBUFF", 12),
        e("1DNADR", "LEMMASS", 0),
        e("1DNADR", "DAPDATR1", 0),
        e("1DNADR", "ROLLTM", 0),
        e("1DNADR", "OPTMODES", 0),
        e("3DNADR", "WBODY", 0),
        e("2DNADR", "REDOCTR", 0),
        e("1DNADR", "IMODES30", 0),
        e("DNCHAN", "11", 0),
        e("DNCHAN", "13", 0),
        e("DNCHAN", "30", 0),
        e("DNCHAN", "32", 0),
        e("1DNADR", "RSBBQ", 0),
        e("3DNADR", "CADRFLSH", 0),
        e("1DNADR", "STATE", 10),
        e("-1DNADR", "GAMMAEI", 0)
    );

    /** CMENTR05 */
    public static final List<Entry> CMENTR05 = List.of(
        e("-1DNADR", "DELV", 0),
        e("1DNADR", "DELV", 2),
        e("1DNADR", "DELV", 4),
        e("1DNADR", "TTE", 0),
        e("1DNADR", "VIO", 0),
        e("1DNADR", "VPRED", 0),
        e("-1DNADR", "PIPTIME1", 0)
    );

    /** CMPG22DL */
    public static final List<Entry> CMPG22DL = List.of(
        e("DNPTR", "CMPG2201", 0),
        e("6DNADR", "DNTMBUFF", 0),
        e("DNPTR", "CMPG2202", 0),
        e("4DNADR", "DNTMBUFF", 0),
        e("DNPTR", "CMPG2203", 0),
        e("6DNADR", "SVMRKDAT", 0),
        e("6DNADR", "SVMRKDAT", 12),
        e("6DNADR", "SVMRKDAT", 24),
        e("1DNADR", "LANDMARK", 0),
        e("1DNADR", "SPARE", 0),
        e("1DNADR", "SPARE", 0),
        e("1DNADR", "SPARE", 0),
        e("DNPTR", "CMPG2204", 0),
        e("1DNADR", "TIME2", 0),
        e("DNPTR", "CMPG2205", 0),
        e("2DNADR", "DNTMBUFF", 0),
        e("1DNADR", "SPARE", 0),
        e("1DNADR", "SPARE", 0),
        e("1DNADR", "SPARE", 0),
        e("1DNADR", "SPARE", 0),
        e("DNPTR", "CMPG2202", 0),
        e("4DNADR", "DNTMBUFF", 0),
        e("DNPTR", "CMPG2203", 0),
        e("DNPTR", "CMPG2206", 0),
        e("1DNADR", "8NN", 0),
        e("1DNADR", "STATE", 10),
        e("3DNADR", "RLS", 0),
        e("1DNADR", "SPARE", 0),
        e("1DNADR", "OPTMODES", 0),
        e("DNPTR", "CMPG2207", 0),
        e("1DNADR", "SPARE", 0),
        e("1DNADR", "SPARE", 0),
        e("1DNADR", "SPARE", 0),
        e("1DNADR", "SPARE", 0),
        e("1DNADR", "SPARE", 0),
        e("-1DNADR", "SPARE", 0)
    );

    /** CMPG2205 */
    public static final List<Entry> CMPG2205 = List.of(
        e("-1DNADR", "LONG", 0),
        e("1DNADR", "ALT", 0),
        e("-1DNADR", "LAT", 0)
    );

    /** CMCSTA01 EQUALS CMPOWE01 */
    public static final List<Entry> CMCSTA01 = CMPOWE01;

    /** CMCSTA02 EQUALS CMPOWE02 */
    public static final List<Entry> CMCSTA02 = CMPOWE02;

    /** CMCSTA03 EQUALS CMPOWE03 */
    public static final List<Entry> CMCSTA03 = CMPOWE03;

    /** CMCSTA04 EQUALS CMPOWE04 */
    public static final List<Entry> CMCSTA04 = CMPOWE04;

    /** CMCSTA05 EQUALS CMPOWE05 */
    public static final List<Entry> CMCSTA05 = CMPOWE05;

    /** CMCSTA06 EQUALS CMPOWE06 */
    public static final List<Entry> CMCSTA06 = CMPOWE06;

    /** CMCSTA07 EQUALS CMPOWE07 */
    public static final List<Entry> CMCSTA07 = CMPOWE07;

    /** CMREND01 EQUALS CMPOWE01 */
    public static final List<Entry> CMREND01 = CMPOWE01;

    /** CMREND02 EQUALS CMPOWE02 */
    public static final List<Entry> CMREND02 = CMPOWE02;

    /** CMREND03 EQUALS CMPOWE03 */
    public static final List<Entry> CMREND03 = CMPOWE03;

    /** CMREND04 EQUALS CMPOWE04 */
    public static final List<Entry> CMREND04 = CMPOWE04;

    /** CMREND05 EQUALS CMPOWE05 */
    public static final List<Entry> CMREND05 = CMPOWE05;

    /** CMREND06 EQUALS CMPOWE06 */
    public static final List<Entry> CMREND06 = CMPOWE06;

    /** CMREND07 EQUALS CMPOWE07 */
    public static final List<Entry> CMREND07 = CMPOWE07;

    /** CMENTR01 EQUALS CMPOWE01 */
    public static final List<Entry> CMENTR01 = CMPOWE01;

    /** CMENTR02 EQUALS CMPOWE02 */
    public static final List<Entry> CMENTR02 = CMPOWE02;

    /** CMENTR03 EQUALS CMPOWE03 */
    public static final List<Entry> CMENTR03 = CMPOWE03;

    /** CMENTR04 EQUALS CMPOWE04 */
    public static final List<Entry> CMENTR04 = CMPOWE04;

    /** CMENTR07 EQUALS CMPOWE07 */
    public static final List<Entry> CMENTR07 = CMPOWE07;

    /** CMPG2201 EQUALS CMPOWE01 */
    public static final List<Entry> CMPG2201 = CMPOWE01;

    /** CMPG2202 EQUALS CMPOWE02 */
    public static final List<Entry> CMPG2202 = CMPOWE02;

    /** CMPG2203 EQUALS CMPOWE03 */
    public static final List<Entry> CMPG2203 = CMPOWE03;

    /** CMPG2204 EQUALS CMPOWE04 */
    public static final List<Entry> CMPG2204 = CMPOWE04;

    /** CMPG2206 EQUALS CMPOWE06 */
    public static final List<Entry> CMPG2206 = CMPOWE06;

    /** CMPG2207 EQUALS CMPOWE07 */
    public static final List<Entry> CMPG2207 = CMPOWE07;

    /** DNTABLE: GENADR table of the downlists, in order. */
    public static final List<List<Entry>> DNTABLE = List.of(CMCSTADL, CMENTRDL, CMRENDDL, CMPOWEDL, CMPG22DL);
}
