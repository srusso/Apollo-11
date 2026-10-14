package apollo.luminary099;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * DOWNLINK_LISTS (Luminary 099): the LM downlink telemetry lists.
 *
 * <p>Each list is a sequence of special downlink op codes:
 * <pre>
 *   OP CODE  SENDS                 BIT 15  BITS 14-12  BITS 11-0
 *   1DNADR   2 AGC words           0       0           ECADR
 *   2DNADR   4 AGC words           0       1           ECADR
 *   3DNADR   6 AGC words           0       2           ECADR
 *   4DNADR   8 AGC words           0       3           ECADR
 *   5DNADR   10 AGC words          0       4           ECADR
 *   6DNADR   12 AGC words          0       5           ECADR
 *   DNCHAN   two channels          0       7           CHANNEL
 *   DNPTR    points to next list   0       6           ADRES
 * </pre>
 * Rules: the end of a list is -XDNADR, -DNPTR or -DNCHAN (the {@code last} flag); a snapshot
 * sublist starts with -1DNADR and can contain only 1DNADRs; the TIME2 1DNADR must be in the control
 * list of a downlist.
 */
public final class DownlinkLists {
    private DownlinkLists() {
    }

    /** Kinds of downlink op code. */
    public enum Op { DNADR, DNPTR, DNCHAN }

    /**
     * One downlink list word.
     *
     * @param op      op code kind
     * @param words   for DNADR the multiplier X (X*2 AGC words are sent); 0 otherwise
     * @param last    true for the negated form that ends the list
     * @param operand erasable symbol (DNADR), list name (DNPTR) or channel number in octal (DNCHAN)
     */
    public record Entry(Op op, int words, boolean last, String operand) { }

    private static Entry adr(int n, boolean last, String sym) {
        return new Entry(Op.DNADR, n, last, sym);
    }

    private static Entry ptr(boolean last, String list) {
        return new Entry(Op.DNPTR, 0, last, list);
    }

    private static Entry chan(boolean last, String octalChannel) {
        return new Entry(Op.DNCHAN, 0, last, octalChannel);
    }

    /** LOWIDCOD: low ID code (OCT 77340). */
    public static final int LOWIDCOD = 077340;

    /** LMORBMDL. */
    public static final List<Entry> LMORBMDL = List.of(
            ptr(false, "LMORBM01"),
            adr(6, false, "DNTMBUFF"),
            adr(1, false, "DELLT4"),
            adr(3, false, "RTARG"),
            adr(1, false, "ELEV"),
            adr(1, false, "TEVENT"),
            adr(6, false, "REFSMMAT"),
            adr(1, false, "TCSI"),
            adr(3, false, "DELVEET1"),
            adr(3, false, "VGTIG"),
            adr(1, false, "DNLRVELZ"),
            adr(1, false, "TPASS4"),
            ptr(false, "LMORBM02"),
            adr(1, false, "TIME2"),
            ptr(false, "LMORBM03"),
            adr(6, false, "DNTMBUFF"),
            ptr(false, "LMORBM04"),
            adr(2, false, "POSTORKU"),
            adr(1, false, "SPARE"),
            adr(1, false, "TCDH"),
            adr(3, false, "DELVEET2"),
            adr(1, false, "TTPI"),
            adr(3, false, "DELVEET3"),
            adr(1, false, "DNRRANGE"),
            adr(2, false, "DNLRVELX"),
            adr(1, false, "DIFFALT"),
            adr(1, false, "LEMMASS"),
            adr(1, false, "IMODES30"),
            adr(1, false, "TIG"),
            ptr(false, "LMORBM05"),
            ptr(false, "LMORBM06"),
            adr(1, false, "SPARE"),
            adr(1, true, "TGO"));

    /** LMORBM01. */
    public static final List<Entry> LMORBM01 = List.of(
            adr(1, true, "R-OTHER +2"),
            adr(1, false, "R-OTHER +4"),
            adr(1, false, "V-OTHER"),
            adr(1, false, "V-OTHER +2"),
            adr(1, false, "V-OTHER +4"),
            adr(1, false, "T-OTHER"),
            adr(1, true, "R-OTHER"));

    /** LMORBM02. */
    public static final List<Entry> LMORBM02 = List.of(
            adr(2, false, "REDOCTR"),
            adr(1, false, "RSBBQ"),
            adr(2, false, "OMEGAP"),
            adr(2, false, "CDUXD"),
            adr(2, false, "CDUX"),
            adr(6, false, "STATE"),
            adr(6, true, "DSPTAB"));

    /** LMORBM03. */
    public static final List<Entry> LMORBM03 = List.of(
            adr(1, true, "RN +2"),
            adr(1, false, "RN +4"),
            adr(1, false, "VN"),
            adr(1, false, "VN +2"),
            adr(1, false, "VN +4"),
            adr(1, false, "PIPTIME"),
            adr(1, true, "RN"));

    /** LMORBM04. */
    public static final List<Entry> LMORBM04 = List.of(
            adr(2, false, "OMEGAPD"),
            adr(3, false, "CADRFLSH"),
            adr(1, true, "RADMODES"));

    /** LMORBM05. */
    public static final List<Entry> LMORBM05 = List.of(
            adr(2, false, "OMEGAP"),
            adr(2, false, "CDUXD"),
            adr(2, false, "CDUX"),
            adr(1, false, "ALPHAQ"),
            adr(1, false, "POSTORKP"),
            chan(false, "11"),
            chan(false, "13"),
            chan(false, "30"),
            chan(true, "32"));

    /** LMORBM06. */
    public static final List<Entry> LMORBM06 = List.of(
            adr(1, false, "PIPTIME1"),
            adr(3, true, "DELV"));

    /** LMCSTADL. */
    public static final List<Entry> LMCSTADL = List.of(
            ptr(false, "LMCSTA01"),
            adr(6, false, "DNTMBUFF"),
            adr(1, false, "AGSK"),
            adr(1, false, "TALIGN"),
            adr(2, false, "POSTORKU"),
            adr(1, false, "DNRRANGE"),
            adr(1, false, "TEVENT"),
            adr(6, false, "REFSMMAT"),
            adr(1, false, "AOTCODE"),
            adr(3, false, "RLS"),
            adr(2, false, "DNLRVELX"),
            ptr(false, "LMCSTA06"),
            ptr(false, "LMCSTA02"),
            adr(1, false, "TIME2"),
            ptr(false, "LMCSTA03"),
            adr(6, false, "DNTMBUFF"),
            ptr(false, "LMCSTA04"),
            ptr(false, "LMCSTA07"),
            adr(2, false, "DNLRVELX"),
            adr(2, false, "CDUS"),
            adr(1, false, "LASTYCMD"),
            adr(1, false, "LEMMASS"),
            adr(1, false, "IMODES30"),
            adr(1, false, "TIG"),
            ptr(false, "LMCSTA05"),
            adr(6, true, "DSPTAB"));

    /** LMCSTA01 = LMORBM01 (common downlist data). */
    public static final List<Entry> LMCSTA01 = LMORBM01;

    /** LMCSTA02 = LMORBM02 (common downlist data). */
    public static final List<Entry> LMCSTA02 = LMORBM02;

    /** LMCSTA03 = LMORBM03 (common downlist data). */
    public static final List<Entry> LMCSTA03 = LMORBM03;

    /** LMCSTA04 = LMORBM04 (common downlist data). */
    public static final List<Entry> LMCSTA04 = LMORBM04;

    /** LMCSTA05 = LMORBM05 (common downlist data). */
    public static final List<Entry> LMCSTA05 = LMORBM05;

    /** LMCSTA06. */
    public static final List<Entry> LMCSTA06 = List.of(
            adr(2, false, "X789"),
            adr(1, true, "LASTYCMD"));

    /** LMCSTA07. */
    public static final List<Entry> LMCSTA07 = List.of(
            adr(3, false, "OGC"),
            adr(1, false, "BESTI"),
            adr(3, false, "STARSAV1"),
            adr(3, true, "STARSAV2"));

    /** LMRENDDL. */
    public static final List<Entry> LMRENDDL = List.of(
            ptr(false, "LMREND01"),
            adr(6, false, "DNTMBUFF"),
            ptr(false, "LMREND07"),
            adr(4, false, "DNTMBUFF"),
            adr(1, false, "DELLT4"),
            adr(3, false, "RTARG"),
            adr(3, false, "DELVSLV"),
            adr(1, false, "TCSI"),
            adr(3, false, "DELVEET1"),
            adr(1, false, "SPARE"),
            adr(1, false, "TPASS4"),
            ptr(false, "LMREND06"),
            ptr(false, "LMREND02"),
            adr(1, false, "TIME2"),
            ptr(false, "LMREND03"),
            adr(6, false, "DNTMBUFF"),
            ptr(false, "LMREND04"),
            adr(2, false, "POSTORKU"),
            adr(1, false, "SPARE"),
            adr(1, false, "TCDH"),
            adr(3, false, "DELVEET2"),
            adr(1, false, "TTPI"),
            adr(3, false, "DELVEET3"),
            adr(1, false, "ELEV"),
            adr(2, false, "CDUS"),
            adr(1, false, "LASTYCMD"),
            adr(1, false, "LEMMASS"),
            adr(1, false, "IMODES30"),
            adr(1, false, "TIG"),
            ptr(false, "LMREND05"),
            adr(1, false, "DELTAR"),
            adr(1, false, "CENTANG"),
            adr(1, false, "NN"),
            adr(1, false, "DIFFALT"),
            adr(1, false, "DELVTPF"),
            adr(1, true, "SPARE"));

    /** LMREND01 = LMORBM01 (common downlist data). */
    public static final List<Entry> LMREND01 = LMORBM01;

    /** LMREND02 = LMORBM02 (common downlist data). */
    public static final List<Entry> LMREND02 = LMORBM02;

    /** LMREND03 = LMORBM03 (common downlist data). */
    public static final List<Entry> LMREND03 = LMORBM03;

    /** LMREND04 = LMORBM04 (common downlist data). */
    public static final List<Entry> LMREND04 = LMORBM04;

    /** LMREND05 = LMORBM05 (common downlist data). */
    public static final List<Entry> LMREND05 = LMORBM05;

    /** LMREND06 = LMCSTA06 (common downlist data). */
    public static final List<Entry> LMREND06 = LMCSTA06;

    /** LMREND07. */
    public static final List<Entry> LMREND07 = List.of(
            adr(1, true, "AIG"),
            adr(1, false, "AOG"),
            adr(1, false, "TANGNB"),
            adr(1, false, "MKTIME"),
            adr(1, true, "RANGRDOT"));

    /** LMDSASDL. */
    public static final List<Entry> LMDSASDL = List.of(
            ptr(false, "LMDSAS07"),
            ptr(false, "LMDSAS08"),
            adr(1, false, "TEVENT"),
            adr(3, false, "UNFC/2"),
            adr(3, false, "VGVECT"),
            adr(1, false, "TTF/8"),
            adr(1, false, "DELTAH"),
            adr(3, false, "RLS"),
            adr(1, false, "SPARE"),
            ptr(false, "LMDSAS09"),
            ptr(false, "LMDSAS02"),
            adr(1, false, "TIME2"),
            ptr(false, "LMDSAS03"),
            adr(6, false, "DNTMBUFF"),
            ptr(false, "LMDSAS04"),
            adr(2, false, "POSTORKU"),
            adr(3, false, "RGU"),
            adr(3, false, "VGU"),
            adr(3, false, "LAND"),
            adr(1, false, "AT"),
            adr(1, false, "TLAND"),
            adr(1, false, "FC"),
            adr(1, false, "LASTYCMD"),
            adr(1, false, "LEMMASS"),
            adr(1, false, "IMODES30"),
            adr(1, false, "TIG"),
            ptr(false, "LMDSAS05"),
            ptr(false, "LMDSAS06"),
            adr(1, false, "PSEUDO55"),
            adr(1, true, "TTOGO"));

    /** LMDSAS02 = LMORBM02 (common downlist data). */
    public static final List<Entry> LMDSAS02 = LMORBM02;

    /** LMDSAS03 = LMORBM03 (common downlist data). */
    public static final List<Entry> LMDSAS03 = LMORBM03;

    /** LMDSAS04 = LMORBM04 (common downlist data). */
    public static final List<Entry> LMDSAS04 = LMORBM04;

    /** LMDSAS05 = LMORBM05 (common downlist data). */
    public static final List<Entry> LMDSAS05 = LMORBM05;

    /** LMDSAS06 = LMORBM06 (common downlist data). */
    public static final List<Entry> LMDSAS06 = LMORBM06;

    /** LMDSAS07. */
    public static final List<Entry> LMDSAS07 = List.of(
            adr(1, true, "LRZCDUDL"),
            adr(1, false, "VSELECT"),
            adr(1, false, "LRVTIMDL"),
            adr(1, false, "VMEAS"),
            adr(1, false, "MKTIME"),
            adr(1, false, "HMEAS"),
            adr(1, false, "RM"),
            adr(1, false, "AIG"),
            adr(1, false, "AOG"),
            adr(1, false, "TANGNB"),
            adr(1, false, "MKTIME"),
            adr(1, true, "LRXCDUDL"));

    /** LMDSAS08. */
    public static final List<Entry> LMDSAS08 = List.of(
            adr(6, false, "DNTMBUFF"),
            adr(5, true, "DNTMBUFF +12D"));

    /** LMDSAS09 = LMCSTA06 (common downlist data). */
    public static final List<Entry> LMDSAS09 = LMCSTA06;

    /** LMLSALDL. */
    public static final List<Entry> LMLSALDL = List.of(
            ptr(false, "LMLSAL01"),
            adr(6, false, "DNTMBUFF"),
            ptr(false, "LMLSAL07"),
            adr(4, false, "DNTMBUFF"),
            adr(1, false, "TALIGN"),
            adr(6, false, "REFSMMAT"),
            adr(6, false, "YNBSAV"),
            ptr(false, "LMLSAL08"),
            ptr(false, "LMLSAL02"),
            adr(1, false, "TIME2"),
            ptr(false, "LMLSAL03"),
            adr(6, false, "DNTMBUFF"),
            ptr(false, "LMLSAL04"),
            ptr(false, "LMLSAL09"),
            adr(3, false, "GSAV"),
            adr(1, false, "AGSK"),
            adr(1, false, "LASTYCMD"),
            adr(1, false, "LEMMASS"),
            adr(1, false, "IMODES30"),
            adr(1, false, "TIG"),
            ptr(false, "LMLSAL05"),
            ptr(false, "LMLSAL06"),
            adr(1, false, "SPARE"),
            adr(1, true, "SPARE"));

    /** LMLSAL01 = LMORBM01 (common downlist data). */
    public static final List<Entry> LMLSAL01 = LMORBM01;

    /** LMLSAL02 = LMORBM02 (common downlist data). */
    public static final List<Entry> LMLSAL02 = LMORBM02;

    /** LMLSAL03 = LMORBM03 (common downlist data). */
    public static final List<Entry> LMLSAL03 = LMORBM03;

    /** LMLSAL04 = LMORBM04 (common downlist data). */
    public static final List<Entry> LMLSAL04 = LMORBM04;

    /** LMLSAL05 = LMORBM05 (common downlist data). */
    public static final List<Entry> LMLSAL05 = LMORBM05;

    /** LMLSAL06 = LMORBM06 (common downlist data). */
    public static final List<Entry> LMLSAL06 = LMORBM06;

    /** LMLSAL07 = LMREND07 (common downlist data). */
    public static final List<Entry> LMLSAL07 = LMREND07;

    /** LMLSAL08 = LMCSTA06 (common downlist data). */
    public static final List<Entry> LMLSAL08 = LMCSTA06;

    /** LMLSAL09 = LMCSTA07 (common downlist data). */
    public static final List<Entry> LMLSAL09 = LMCSTA07;

    /** LMAGSIDL. */
    public static final List<Entry> LMAGSIDL = List.of(
            adr(3, false, "AGSBUFF +0"),
            adr(1, false, "AGSBUFF +12D"),
            adr(3, false, "AGSBUFF +1"),
            adr(1, false, "AGSBUFF +13D"),
            adr(3, false, "AGSBUFF +6"),
            adr(1, false, "AGSBUFF +12D"),
            adr(3, false, "AGSBUFF +7"),
            adr(1, false, "AGSBUFF +13D"),
            adr(6, false, "COMPNUMB"),
            adr(6, false, "UPBUFF +8D"),
            ptr(false, "LMAGSI02"),
            adr(1, false, "TIME2"),
            ptr(false, "LMAGSI03"),
            adr(6, false, "DNTMBUFF"),
            ptr(false, "LMAGSI04"),
            adr(2, false, "POSTORKU"),
            adr(1, false, "SPARE"),
            adr(1, false, "SPARE"),
            adr(1, false, "AGSK"),
            adr(6, false, "UPBUFF"),
            adr(4, false, "UPBUFF +12D"),
            adr(1, false, "LEMMASS"),
            adr(1, false, "IMODES30"),
            adr(1, false, "SPARE"),
            ptr(false, "LMAGSI05"),
            adr(6, true, "DSPTAB"));

    /** LMAGSI02 = LMORBM02 (common downlist data). */
    public static final List<Entry> LMAGSI02 = LMORBM02;

    /** LMAGSI03 = LMORBM03 (common downlist data). */
    public static final List<Entry> LMAGSI03 = LMORBM03;

    /** LMAGSI04 = LMORBM04 (common downlist data). */
    public static final List<Entry> LMAGSI04 = LMORBM04;

    /** LMAGSI05 = LMORBM05 (common downlist data). */
    public static final List<Entry> LMAGSI05 = LMORBM05;

    /** All lists by their assembly label (aliases included). */
    public static final Map<String, List<Entry>> LISTS;

    static {
        Map<String, List<Entry>> m = new LinkedHashMap<>();
        m.put("LMORBMDL", LMORBMDL);
        m.put("LMORBM01", LMORBM01);
        m.put("LMORBM02", LMORBM02);
        m.put("LMORBM03", LMORBM03);
        m.put("LMORBM04", LMORBM04);
        m.put("LMORBM05", LMORBM05);
        m.put("LMORBM06", LMORBM06);
        m.put("LMCSTADL", LMCSTADL);
        m.put("LMCSTA01", LMCSTA01);
        m.put("LMCSTA02", LMCSTA02);
        m.put("LMCSTA03", LMCSTA03);
        m.put("LMCSTA04", LMCSTA04);
        m.put("LMCSTA05", LMCSTA05);
        m.put("LMCSTA06", LMCSTA06);
        m.put("LMCSTA07", LMCSTA07);
        m.put("LMRENDDL", LMRENDDL);
        m.put("LMREND01", LMREND01);
        m.put("LMREND02", LMREND02);
        m.put("LMREND03", LMREND03);
        m.put("LMREND04", LMREND04);
        m.put("LMREND05", LMREND05);
        m.put("LMREND06", LMREND06);
        m.put("LMREND07", LMREND07);
        m.put("LMDSASDL", LMDSASDL);
        m.put("LMDSAS02", LMDSAS02);
        m.put("LMDSAS03", LMDSAS03);
        m.put("LMDSAS04", LMDSAS04);
        m.put("LMDSAS05", LMDSAS05);
        m.put("LMDSAS06", LMDSAS06);
        m.put("LMDSAS07", LMDSAS07);
        m.put("LMDSAS08", LMDSAS08);
        m.put("LMDSAS09", LMDSAS09);
        m.put("LMLSALDL", LMLSALDL);
        m.put("LMLSAL01", LMLSAL01);
        m.put("LMLSAL02", LMLSAL02);
        m.put("LMLSAL03", LMLSAL03);
        m.put("LMLSAL04", LMLSAL04);
        m.put("LMLSAL05", LMLSAL05);
        m.put("LMLSAL06", LMLSAL06);
        m.put("LMLSAL07", LMLSAL07);
        m.put("LMLSAL08", LMLSAL08);
        m.put("LMLSAL09", LMLSAL09);
        m.put("LMAGSIDL", LMAGSIDL);
        m.put("LMAGSI02", LMAGSI02);
        m.put("LMAGSI03", LMAGSI03);
        m.put("LMAGSI04", LMAGSI04);
        m.put("LMAGSI05", LMAGSI05);
        LISTS = Collections.unmodifiableMap(m);
    }

    /** NOMDNLST: fresh start and post-P27 downlist. */
    public static final List<Entry> NOMDNLST = LMCSTADL;
    /** AGSLIST. */
    public static final List<Entry> AGSLIST = LMAGSIDL;
    /** UPDNLIST: update program (P27) downlist. */
    public static final List<Entry> UPDNLIST = LMAGSIDL;

    /** DNTABLE: the table of downlists selectable by downlist number 0..5. */
    public static final List<List<Entry>> DNTABLE = List.of(
            LMCSTADL,   // LM coast and align downlist
            LMAGSIDL,   // LM AGS initialization/update downlist
            LMRENDDL,   // LM rendezvous and pre-thrust downlist
            LMORBMDL,   // LM orbital maneuvers downlist
            LMDSASDL,   // LM descent and ascent downlist
            LMLSALDL);  // LM lunar surface align downlist
}
