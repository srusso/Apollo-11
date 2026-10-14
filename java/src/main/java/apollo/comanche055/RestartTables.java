package apollo.comanche055;

import apollo.agc.Agc;
import java.util.List;
import java.util.Map;

/**
 * RESTART_TABLES.agc (Comanche055): the restart tables (bank 01, RESTART) and the
 * NEWPHASE / PHASCHNG entry (bank 02, FFTAG1).
 *
 * <p>For each restart group g there is a pair of phase lists: even phases (g.2SPOT, g.4SPOT, ...)
 * are job/priority-type entries (PRDTTAB gives priority for jobs, delta time for tasks, as the
 * first word, followed by the 2CADR/-2CADR of the routine to restart) and odd phases
 * (g.3SPOT, ...) are the task (waitlist) entries. Sign on the CADR word denotes the
 * -2CADR/-GENADR form (task delta-time entries). Entries here are textual descriptions
 * of each table word, in table order, with the EBANK= setting preceding each CADR.
 */
public final class RestartTables {
    private final Agc agc;

    public RestartTables(Agc agc) {
        this.agc = agc;
    }

    /** PRDTTAB: EQUALS 12000 (octal). */
    public static final int PRDTTAB = 012000;
    /** CADRTAB: EQUALS 12001 (octal). */
    public static final int CADRTAB = 012001;

    /** SIZETAB: TC entries giving for each group/phase the table offsets, in the order 1.2, 1.3, ..., 6.2, 6.3. */
    public static final String[] SIZETAB = {
        "1.2SPOT-12006", "1.3SPOT-12004", "2.2SPOT-12006", "2.3SPOT-12004",
        "3.2SPOT-12006", "3.3SPOT-12004", "4.2SPOT-12006", "4.3SPOT-12004",
        "5.2SPOT-12006", "5.3SPOT-12004", "6.2SPOT-12006", "6.3SPOT-12004"
    };

    /** The restart table spots: label to the list of words (textual). */
    public static final Map<String, List<String>> SPOTS = new java.util.LinkedHashMap<>();
    static {

        SPOTS.put("SIZETAB", List.of());
        SPOTS.put("1.3SPOT", List.of("DEC 120", "-2CADR SETJTAG [EBANK=AOG]"));
        SPOTS.put("1.5SPOT", List.of("OCT 10000", "2CADR REDO40.9 [EBANK=DAPDATR1]"));
        SPOTS.put("1.7SPOT", List.of("OCT 10000", "2CADR RELINUS [EBANK=ESTROKER]"));
        SPOTS.put("1.11SPOT", List.of("OCT 10000", "2CADR PIKUP20 [EBANK=ESTROKER]"));
        SPOTS.put("2.3SPOT", List.of("GENADR 600SECS", "-GENADR STATEINT", "BBCON STATEINT"));
        SPOTS.put("2.5SPOT", List.of("OCT 05000", "2CADR STATINT1 [EBANK=RRECTCSM]"));
        SPOTS.put("2.7SPOT", List.of("OCT 10000", "2CADR R22 [EBANK=MRKBUF2]"));
        SPOTS.put("2.11SPOT", List.of("OCT 14000", "2CADR V94ENTER [EBANK=LANDMARK]"));
        SPOTS.put("2.13SPOT", List.of("OCT 10000", "2CADR REDOR22 [EBANK=MRKBUF2]"));
        SPOTS.put("3.3SPOT", List.of("OCT 20000", "2CADR S40.13 [EBANK=TGO]"));
        SPOTS.put("3.5SPOT", List.of("DEC 0", "DEC 0", "DEC 0"));
        SPOTS.put("3.7SPOT", List.of("OCT 22000", "2CADR MATRXJOB [EBANK=TEPHEM]"));
        SPOTS.put("3.11SPOT", List.of("OCT 22000", "2CADR REP11 [EBANK=TEPHEM]"));
        SPOTS.put("3.13SPOT", List.of("OCT 22000", "2CADR REP11A [EBANK=TEPHEM]"));
        SPOTS.put("3.15SPOT", List.of("-GENADR TGO +1", "-2CADR ENGINOFF [EBANK=TGO]"));
        SPOTS.put("4.2SPOT", List.of("OCT 77777", "-2CADR PRECHECK [EBANK=TIG]", "OCT 30000", "2CADR P47BODY [EBANK=DELVIMU]"));
        SPOTS.put("4.4SPOT", List.of("OCT 77777", "-2CADR PRECHECK [EBANK=TIG]", "DEC 2996", "-2CADR TTG/0 [EBANK=DAPDATR1]"));
        SPOTS.put("4.6SPOT", List.of("OCT 77777", "-2CADR PRECHECK [EBANK=TIG]", "DEC 2496", "-2CADR TIG-5 [EBANK=TIG]"));
        SPOTS.put("4.3SPOT", List.of("DEC 40", "-2CADR DOTVCON [EBANK=PACTOFF]"));
        SPOTS.put("4.5SPOT", List.of("DEC 160", "-2CADR DOSTRULL [EBANK=PACTOFF]"));
        SPOTS.put("4.7SPOT", List.of("DEC 500", "-2CADR TIG-0 [EBANK=PACTOFF]"));
        SPOTS.put("4.11SPOT", List.of("DEC 250", "-2CADR V97E40.6 [EBANK=DAPDATR1]"));
        SPOTS.put("4.13SPOT", List.of("DEC 200", "-2CADR R40ENABL [EBANK=WHOCARES]"));
        SPOTS.put("4.15SPOT", List.of("OCT 16000", "2CADR COMPVER [EBANK=OGC]"));
        SPOTS.put("4.17SPOT", List.of("OCT 16000", "2CADR AZMTHCG1 [EBANK=XSM]"));
        SPOTS.put("4.21SPOT", List.of("GENADR P40TMP", "-GENADR TIGBLNK", "BBCON TIGBLNK"));
        SPOTS.put("4.23SPOT", List.of("OCT 12000", "2CADR P40S/SV [EBANK=TIG]"));
        SPOTS.put("4.25SPOT", List.of("OCT 24000", "2CADR PROG52 [EBANK=BESTI]"));
        SPOTS.put("4.27SPOT", List.of("DEC 250", "-2CADR DOTVCRCS [EBANK=PACTOFF]"));
        SPOTS.put("4.31SPOT", List.of("OCT 13000", "2CADR R51 +1 [EBANK=STAR]"));
        SPOTS.put("4.33SPOT", List.of("DEC 2100", "-2CADR WAKEP62 [EBANK=AOG]"));
        SPOTS.put("4.35SPOT", List.of("OCT 12000", "2CADR POSTBURN [EBANK=DAPDATR1]"));
        SPOTS.put("4.37SPOT", List.of("DEC 500", "-2CADR TIGAVEG [EBANK=TIG]"));
        SPOTS.put("4.41SPOT", List.of("OCT 17000", "2CADR P67.1 [EBANK=AOG]"));
        SPOTS.put("4.43SPOT", List.of("-GENADR S61DT", "-2CADR S61.1C [EBANK=S61DT]"));
        SPOTS.put("4.45SPOT", List.of("OCT 13000", "2CADR S61.1A -1 [EBANK=AOG]"));
        SPOTS.put("4.47SPOT", List.of("OCT 17000", "2CADR PRE-HUNT [EBANK=AOG]"));
        SPOTS.put("4.51SPOT", List.of("OCT 77777", "-2CADR ATERTASK [EBANK=BODY3]"));
        SPOTS.put("4.53SPOT", List.of("DEC -0", "-2CADR V97ETASK [EBANK=END-E7]"));
        SPOTS.put("4.55SPOT", List.of("OCT 13000", "2CADR P65.1 [EBANK=RTINIT]"));
        SPOTS.put("4.57SPOT", List.of("-GENADR P40TMP", "-2CADR TIGON [EBANK=P40TMP]"));
        SPOTS.put("4.61SPOT", List.of("OCT 77777", "-2CADR IGNITION [EBANK=PACTOFF]"));
        SPOTS.put("4.63SPOT", List.of("OCT 77777", "-2CADR DOSPSOFF [EBANK=PACTOFF]"));
        SPOTS.put("4.65SPOT", List.of("DEC 10", "-2CADR TIG-5 [EBANK=TIG]"));
        SPOTS.put("4.67SPOT", List.of("DEC -0", "-2CADR V97TTASK [EBANK=CSMMASS]"));
        SPOTS.put("4.71SPOT", List.of("DEC 250", "-2CADR V97TRCS [EBANK=DAPDATR1]"));
        SPOTS.put("4.73SPOT", List.of("DEC -0", "-2CADR V97PTASK [EBANK=V97VCNTR]"));
        SPOTS.put("4.75SPOT", List.of("DEC -0", "-2CADR SPSOFF97 [EBANK=DAPDATR1]"));
        SPOTS.put("4.77SPOT", List.of("DEC -0", "-2CADR TIG-0 [EBANK=PACTOFF]"));
        SPOTS.put("5.2SPOT", List.of("OCT 32000", "2CADR NORMLIZE [EBANK=DVCNTR]", "DEC 200", "-2CADR REREADAC [EBANK=AOG]"));
        SPOTS.put("5.4SPOT", List.of("OCT 20000", "2CADR SERVICER [EBANK=DVCNTR]", "DEC 200", "-2CADR REREADAC [EBANK=AOG]"));
        SPOTS.put("5.3SPOT", List.of("DEC 200", "-2CADR REREADAC [EBANK=AOG]"));
        SPOTS.put("5.5SPOT", List.of("OCT 77777", "-2CADR REDO5.5 [EBANK=AOG]"));
        SPOTS.put("5.7SPOT", List.of("OCT 20000", "2CADR RSTGTS1 [EBANK=XSM]"));
        SPOTS.put("5.11SPOT", List.of("OCT 77777", "-2CADR ALLOOP1 [EBANK=XSM]"));
        SPOTS.put("5.13SPOT", List.of("OCT 20000", "2CADR WTLISTNT [EBANK=XSM]"));
        SPOTS.put("5.15SPOT", List.of("OCT 20000", "2CADR RESTEST1 [EBANK=XSM]"));
        SPOTS.put("5.17SPOT", List.of("OCT 20000", "2CADR GEOSTRT4 [EBANK=XSM]"));
        SPOTS.put("5.21SPOT", List.of("OCT 22000", "2CADR ALFLT1 [EBANK=XSM]"));
        SPOTS.put("5.23SPOT", List.of("OCT 77777", "-2CADR SPECSTS [EBANK=XSM]"));
        SPOTS.put("5.25SPOT", List.of("OCT 20000", "2CADR RESTEST3 [EBANK=XSM]"));
        SPOTS.put("5.27SPOT", List.of("OCT 20000", "2CADR RESTAIER [EBANK=XSM]"));
        SPOTS.put("5.31SPOT", List.of("DEC 0", "DEC 0", "DEC 0"));
        SPOTS.put("5.33SPOT", List.of("OCT 20000", "2CADR RESCHNG [EBANK=XSM]"));
        SPOTS.put("5.35SPOT", List.of("DEC 0", "2DEC 0"));
        SPOTS.put("5.37SPOT", List.of("OCT 77777", "-2CADR CHEKAVEG [EBANK=AOG]"));
        SPOTS.put("5.41SPOT", List.of("OCT 77777", "-2CADR PREREAD [EBANK=DVCNTR]"));
        SPOTS.put("6.2SPOT", List.of("OCT 77777", "-2CADR PRE40.6 [EBANK=AK]", "DEC 100", "-2CADR CLOKTASK [EBANK=TTOGO]"));
        SPOTS.put("6.3SPOT", List.of("DEC 100", "-2CADR CLOKTASK [EBANK=TIG]"));
        SPOTS.put("6.5SPOT", List.of("OCT 30000", "2CADR TIMEDIDR [EBANK=TEPHEM]"));
        SPOTS.put("6.7SPOT", List.of("OCT 0", "OCT 0", "OCT 0"));
        SPOTS.put("6.11SPOT", List.of("-GENADR CM/GYMDT", "-2CADR READGYMB [EBANK=CM/GYMDT]"));
        SPOTS.put("6.13SPOT", List.of("DEC 0", "DEC 0", "DEC 0"));
        SPOTS.put("1.2SPOT", SPOTS.get("3.2SPOT")); // 1.2SPOT EQUALS 3.2SPOT
        SPOTS.put("2.2SPOT", SPOTS.get("1.2SPOT")); // 2.2SPOT EQUALS 1.2SPOT
        SPOTS.put("3.2SPOT", SPOTS.get("4.2SPOT")); // 3.2SPOT EQUALS 4.2SPOT
    }

    // ---- NEWPHASE (bank 02, FFTAG1, COUNT* $$/PHASE) ----

    /**
     * NEWPHASE: store a new phase for a restart group. {@code group} is the group number
     * (the word following the call), {@code phase} the new phase code (A on entry), and
     * {@code deltaTime} the sign of the delta-T control (CCS L in the original): if the
     * phase argument is positive nothing about TBASE is changed; NUFAZ+10 sets TBASE.
     * Phases are held as -PHASEn and TBASEn pairs, indexed by the doubled group number.
     */
    public void newPhase(int group, int phase, boolean setTbase) {
        // INHINT
        int tempg = group * 2;            // DOUBLE; TS TEMPG
        if (setTbase) {                   // NUFAZ+10: set TBASE and store phase
            int a = Agc.add(phase, 1);    // INCR A
            // CS TIME1; TS TBASE1-2 (indexed by TEMPG)
            agc.set("TBASE" + group, ~agc.get("TIME1") & Agc.WORD_MASK);
            phase = a;
        }
        // CS L; DXCH -PHASE1-2 : phase stored complemented in -PHASEn
        agc.set("-PHASE" + group, ~phase & Agc.WORD_MASK);
        agc.set("TEMPG", tempg);
        // RELINT; TC Q
    }
}
