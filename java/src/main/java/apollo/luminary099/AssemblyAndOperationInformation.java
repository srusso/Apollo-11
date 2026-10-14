package apollo.luminary099;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * ASSEMBLY_AND_OPERATION_INFORMATION (Luminary 1A, build 099, Apollo 11 LGC).
 *
 * <p>This AGC source file contains no executable code: it is the front matter of the listing.
 * It describes the assembly (revision 001 of AGC program LMY99, 16:27 July 14, 1969), the table
 * of log sections, and the operating tables used by the crew: the verb list, noun list (normal and
 * mixed nouns with register assignments and scale types), alarm codes, checklist codes and option
 * codes. Those tables are reproduced here as read-only maps.
 */
public final class AssemblyAndOperationInformation {
    private AssemblyAndOperationInformation() {
    }

    /** Program identification. */
    public static final String PROGRAM = "LUMINARY 1A, build 099 (LMY99 revision 001, NASA 2021112-061)";

    /** Prepared by the MIT Instrumentation Laboratory, contract NAS 9-4065, report R-567 as amended. */
    public static final String REFERENCE = "R-567, GUIDANCE SYSTEM OPERATIONS PLAN FOR MANNED LM EARTH ORBITAL AND LUNAR MISSIONS";

    /** Description of a noun: title, number of components (0 if spare) and scale/format text. */
    public record Noun(int number, String title, int components, String format,
            String[] registers, String[] scaleTypes) { }

    /** Alarm code entry: severity marker ("*", "**", "P" or ""), meaning, and the setter(s). */
    public record Alarm(String code, String severity, String meaning, String setBy) { }

    /** Checklist (R1CODE) entry. */
    public record Checklist(String code, String kind, String action, String program) { }

    /** Option code entry. */
    public record Option(String code, String purpose, String input, String program) { }

    private static final Map<Integer, String> VERB_MAP = new LinkedHashMap<>();
    private static final Map<Integer, Noun> NOUN_MAP = new LinkedHashMap<>();
    private static final Map<String, String> SCALE_MAP = new LinkedHashMap<>();
    private static final Map<String, Alarm> ALARM_MAP = new LinkedHashMap<>();
    private static final Map<String, Checklist> CHECKLIST_MAP = new LinkedHashMap<>();
    private static final Map<String, Option> OPTION_MAP = new LinkedHashMap<>();

    static {
        // Verbs 00-39 regular, 40-99 extended
        VERB_MAP.put(0, "NOT IN USE");
        VERB_MAP.put(1, "DISPLAY OCTAL COMP 1 IN R1");
        VERB_MAP.put(2, "DISPLAY OCTAL COMP 2 IN R1");
        VERB_MAP.put(3, "DISPLAY OCTAL COMP 3 IN R1");
        VERB_MAP.put(4, "DISPLAY OCTAL COMP 1,2 IN R1,R2");
        VERB_MAP.put(5, "DISPLAY OCTAL COMP 1,2,3 IN R1,R2,R3");
        VERB_MAP.put(6, "DISPLAY DECIMAL IN R1 OR R1,R2 OR R1,R2,R3");
        VERB_MAP.put(7, "DISPLAY DP DECIMAL IN R1,R2 (TEST ONLY)");
        VERB_MAP.put(11, "MONITOR OCTAL COMP 1 IN R1");
        VERB_MAP.put(12, "MONITOR OCTAL COMP 2 IN R1");
        VERB_MAP.put(13, "MONITOR OCTAL COMP 3 IN R1");
        VERB_MAP.put(14, "MONITOR OCTAL COMP 1,2 IN R1,R2");
        VERB_MAP.put(15, "MONITOR OCTAL COMP 1,2,3 IN R1,R2,R3");
        VERB_MAP.put(16, "MONITOR DECIMAL IN R1 OR R1,R2 OR R1,R2,R3");
        VERB_MAP.put(17, "MONITOR DP DECIMAL IN R1,R2 (TEST ONLY)");
        VERB_MAP.put(21, "LOAD COMPONENT 1 INTO R1");
        VERB_MAP.put(22, "LOAD COMPONENT 2 INTO R2");
        VERB_MAP.put(23, "LOAD COMPONENT 3 INTO R3");
        VERB_MAP.put(24, "LOAD COMPONENT 1,2, INTO R1,R2");
        VERB_MAP.put(25, "LOAD COMPONENT 1,2,3 INTO R1,R2,R3");
        VERB_MAP.put(27, "DISPLAY FIXED MEMORY");
        VERB_MAP.put(30, "REQUEST EXECUTIVE");
        VERB_MAP.put(31, "REQUEST WAITLIST");
        VERB_MAP.put(32, "RECYCLE PROGRAM");
        VERB_MAP.put(33, "PROCEED WITHOUT DSKY INPUTS");
        VERB_MAP.put(34, "TERMINATE FUNCTION");
        VERB_MAP.put(35, "TEST LIGHTS");
        VERB_MAP.put(36, "REQUEST FRESH START");
        VERB_MAP.put(37, "CHANGE PROGRAM (MAJOR MODE)");
        VERB_MAP.put(40, "ZERO CDU-S");
        VERB_MAP.put(41, "COARSE ALIGN CDU-S");
        VERB_MAP.put(42, "FINE ALIGN IMU");
        VERB_MAP.put(43, "LOAD IMU ATT ERROR METERS");
        VERB_MAP.put(44, "TERMINATE RR CONTINUOUS DESIGNATE (V41N72 OPTION 2)");
        VERB_MAP.put(47, "INITIALIZE AGS (R47)");
        VERB_MAP.put(48, "REQUEST DAP DATA LOAD ROUTINE (R03)");
        VERB_MAP.put(49, "REQUEST CREW DEFINED MANEUVER ROUTINE (R62)");
        VERB_MAP.put(50, "PLEASE PERFORM");
        VERB_MAP.put(52, "MARK X-RETICLE");
        VERB_MAP.put(53, "MARK Y-RETICLE");
        VERB_MAP.put(54, "MARK X OR Y-RETICLE");
        VERB_MAP.put(55, "INCREMENT AGC TIME (DECIMAL)");
        VERB_MAP.put(56, "TERMINATE TRACKING (P20 + P25)");
        VERB_MAP.put(57, "PERMIT LANDING RADAR UPDATES");
        VERB_MAP.put(58, "INHIBIT LANDING RADAR UPDATES");
        VERB_MAP.put(60, "COMMAND LR TO POSITON 2.");
        VERB_MAP.put(61, "DISPLAY DAP FOLLOWING ATTITUDE ERRORS.");
        VERB_MAP.put(62, "DISPLAY TOTAL ATTITUDE ERRORS WITH RESPECT TO NOUN 22.");
        VERB_MAP.put(63, "SAMPLE RADAR ONCE PER SECOND (R04).");
        VERB_MAP.put(64, "REQUEST S-BAND ANTENNA ROUTINE (R05).");
        VERB_MAP.put(65, "DISABLE U AND V JET FIRINGS DURING DPS BURNS.");
        VERB_MAP.put(66, "VEHICLES ARE ATTACHED.  MOVE THIS VEHICLE STATE TO OTHER VEHICLE.");
        VERB_MAP.put(67, "DISPLAY W MATRIX");
        VERB_MAP.put(69, "CAUSE RESTART");
        VERB_MAP.put(70, "UPDATE LIFTOFF TIME");
        VERB_MAP.put(71, "UNIVERSAL UPDATE-BLOCK ADR");
        VERB_MAP.put(72, "UNIVERSAL UPDATE-SINGLE ADR");
        VERB_MAP.put(73, "UPDATE AGC TIME (OCTAL)");
        VERB_MAP.put(74, "INITIALIZE ERASABLE DUMP VIA DOWNLINK");
        VERB_MAP.put(75, "ENABLE U AND V JET FIRINGS DURING DPS BURNS.");
        VERB_MAP.put(76, "MINIMUM IMPUSE COMMAND MODE.");
        VERB_MAP.put(77, "RATE COMMAND AND ATTITUDE HOLD MODE");
        VERB_MAP.put(78, "LR SPURIOUS RETURN TEST START (R77)");
        VERB_MAP.put(79, "LR SPURIOUS RETURN TEST STOP");
        VERB_MAP.put(80, "UPDATE LEM STATE VECTOR");
        VERB_MAP.put(81, "UPDATE CSM STATE VECTOR");
        VERB_MAP.put(82, "REQUEST ORBIT PARAM DISPLAY (R30)");
        VERB_MAP.put(83, "REQUEST REND  PARAM DISPLAY (R31)");
        VERB_MAP.put(85, "DISPLAY RR LOS AZ AND ELEV");
        VERB_MAP.put(89, "REQUEST RENDEZVOUS FINAL ATTITUDE ROUTINE (R63)");
        VERB_MAP.put(90, "REQUEST RENDEZVOUS OUT OF PLANE DISPLAY ROUTINE (R36)");
        VERB_MAP.put(91, "DISPLAY BANK SUM");
        VERB_MAP.put(92, "OPERATE IMU PERFORMANCE TEST (P07)");
        VERB_MAP.put(93, "ENABLE W MATRIX INITIALIZATION");
        VERB_MAP.put(95, "NO UPDATE OF EITHER STATE VECTOR (P20 OR P22)");
        VERB_MAP.put(96, "INTERRUPT INTEGRATION AND GO TO P00");
        VERB_MAP.put(97, "PERFORM ENGINE FAIL PROCEDURE");
        VERB_MAP.put(99, "PLEASE ENABLE ENGINE");
        // Nouns: normal nouns 00-39 and mixed nouns 40-99
        NOUN_MAP.put(0, new Noun(0, "NOT IN USE", 0, "", new String[] {}, new String[] {}));
        NOUN_MAP.put(1, new Noun(1, "SPECIFY MACHINE ADDRESS (FRACTIONAL)", 3, ".xxxxx FOR EACH", new String[] {"SPECIFY ADDRESS"}, new String[] {"B"}));
        NOUN_MAP.put(2, new Noun(2, "SPECIFY MACHINE ADDRESS (WHOLE)", 3, "xxxxx. FOR EACH", new String[] {"SPECIFY ADDRESS"}, new String[] {"C"}));
        NOUN_MAP.put(3, new Noun(3, "SPECIFY MACHINE ADDRESS (DEGREES)", 3, "xxx.xx DEG FOR EACH", new String[] {"SPECIFY ADDRESS"}, new String[] {"D"}));
        NOUN_MAP.put(4, new Noun(4, "ANGULAR ERROR/DIFFERENCE", 1, "xxx.xx DEG", new String[] {"DSPTEM1"}, new String[] {"H"}));
        NOUN_MAP.put(5, new Noun(5, "ANGULAR ERROR/DIFFERENCE", 1, "xxx.xx DEG", new String[] {"DSPTEM1"}, new String[] {"H"}));
        NOUN_MAP.put(6, new Noun(6, "OPTION CODE", 3, "OCTAL ONLY FOR EACH", new String[] {"OPTION1"}, new String[] {"A"}));
        NOUN_MAP.put(7, new Noun(7, "ECADR OF WORD TO BE MODIFIED", 3, "OCTAL ONLY FOR EACH", new String[] {"XREG"}, new String[] {"A"}));
        NOUN_MAP.put(8, new Noun(8, "ALARM DATA", 3, "OCTAL ONLY FOR EACH", new String[] {"ALMCADR"}, new String[] {"A"}));
        NOUN_MAP.put(9, new Noun(9, "ALARM CODES", 3, "OCTAL ONLY FOR EACH", new String[] {"FAILREG"}, new String[] {"A"}));
        NOUN_MAP.put(10, new Noun(10, "CHANNEL TO BE SPECIFIED", 1, "OCTAL ONLY", new String[] {"SPECIFY CHANNEL"}, new String[] {"A"}));
        NOUN_MAP.put(11, new Noun(11, "TIG OF CSI", 3, "00xxx. HRS				DEC ONLY", new String[] {"TCSI"}, new String[] {"K"}));
        NOUN_MAP.put(12, new Noun(12, "OPTION CODE", 2, "OCTAL ONLY FOR EACH", new String[] {"OPTIONX"}, new String[] {"A"}));
        NOUN_MAP.put(13, new Noun(13, "TIG OF CDH", 3, "00xxx. HRS				DEC ONLY", new String[] {"TCDH"}, new String[] {"K"}));
        NOUN_MAP.put(14, new Noun(14, "CHECKLIST", 3, "xxxxx. FOR EACH", new String[] {"DSPTEMX"}, new String[] {"C"}));
        NOUN_MAP.put(15, new Noun(15, "INCREMENT MACHINE ADDRESS", 1, "OCTAL ONLY", new String[] {"INCREMENT ADDRESS"}, new String[] {"A"}));
        NOUN_MAP.put(16, new Noun(16, "TIME OF EVENT", 3, "00xxx. HRS				DEC ONLY", new String[] {"DSPTEMX"}, new String[] {"K"}));
        NOUN_MAP.put(17, new Noun(17, "SPARE", 0, "", new String[] {}, new String[] {}));
        NOUN_MAP.put(18, new Noun(18, "AUTO MANEUVER BALL ANGLES", 3, "xxx.xx DEG FOR EACH", new String[] {"FDAIX"}, new String[] {"D"}));
        NOUN_MAP.put(19, new Noun(19, "SPARE", 0, "", new String[] {}, new String[] {}));
        NOUN_MAP.put(20, new Noun(20, "ICDU ANGLES", 3, "xxx.xx DEG FOR EACH", new String[] {"CDUX"}, new String[] {"D"}));
        NOUN_MAP.put(21, new Noun(21, "PIPAS", 3, "xxxxx. PULSES FOR EACH", new String[] {"PIPAX"}, new String[] {"C"}));
        NOUN_MAP.put(22, new Noun(22, "NEW ICDU ANGLES", 3, "xxx.xx DEG FOR EACH", new String[] {"THETAD"}, new String[] {"D"}));
        NOUN_MAP.put(23, new Noun(23, "SPARE", 0, "", new String[] {}, new String[] {}));
        NOUN_MAP.put(24, new Noun(24, "DELTA TIME FOR AGC CLOCK", 3, "00xxx. HRS				DEC ONLY", new String[] {"DSPTEM2 +1"}, new String[] {"K"}));
        NOUN_MAP.put(25, new Noun(25, "CHECKLIST", 3, "xxxxx. FOR EACH", new String[] {"DSPTEM1"}, new String[] {"C"}));
        NOUN_MAP.put(26, new Noun(26, "PRIORITY/DELAY, ADRES, BBCON", 3, "OCTAL ONLY FOR EACH", new String[] {"DSPTEM1"}, new String[] {"A"}));
        NOUN_MAP.put(27, new Noun(27, "SELF TEST ON/OFF SWITCH", 1, "xxxxx.", new String[] {"SMODE"}, new String[] {"C"}));
        NOUN_MAP.put(28, new Noun(28, "SPARE", 0, "", new String[] {}, new String[] {}));
        NOUN_MAP.put(29, new Noun(29, "SPARE", 0, "", new String[] {}, new String[] {}));
        NOUN_MAP.put(30, new Noun(30, "SPARE", 0, "", new String[] {}, new String[] {}));
        NOUN_MAP.put(31, new Noun(31, "SPARE", 0, "", new String[] {}, new String[] {}));
        NOUN_MAP.put(32, new Noun(32, "TIME FROM PERIGEE", 3, "00xxx. HRS				DEC ONLY", new String[] {"-TPER"}, new String[] {"K"}));
        NOUN_MAP.put(33, new Noun(33, "TIME OF IGNITION", 3, "00xxx. HRS				DEC ONLY", new String[] {"TIG"}, new String[] {"K"}));
        NOUN_MAP.put(34, new Noun(34, "TIME OF EVENT", 3, "00xxx. HRS				DEC ONLY", new String[] {"DSPTEM1"}, new String[] {"K"}));
        NOUN_MAP.put(35, new Noun(35, "TIME FROM EVENT", 3, "00xxx. HRS				DEC ONLY", new String[] {"TTOGO"}, new String[] {"K"}));
        NOUN_MAP.put(36, new Noun(36, "TIME OF AGC CLOCK", 3, "00xxx. HRS				DEC ONLY", new String[] {"TIME2"}, new String[] {"K"}));
        NOUN_MAP.put(37, new Noun(37, "TIG OF TPI", 3, "00xxx. HRS				DEC ONLY", new String[] {"TTPI"}, new String[] {"K"}));
        NOUN_MAP.put(38, new Noun(38, "TIME OF STATE BEING INTEGRATED", 3, "00xxx. HRS				DEC ONLY", new String[] {"TET"}, new String[] {"K"}));
        NOUN_MAP.put(39, new Noun(39, "SPARE", 0, "", new String[] {}, new String[] {}));
        NOUN_MAP.put(40, new Noun(40, "TIME FROM IGNITION/CUTOFF", 3, "xxBxx  MIN/SEC				NO LOAD, DEC ONLY | VG,						xxxx.x FT/SEC | DELTA V (ACCUMULATED)				xxxx.x FT.SEC", new String[] {"TTOGO", "VGDISP", "DVTOTAL"}, new String[] {"L", "S", "S"}));
        NOUN_MAP.put(41, new Noun(41, "TARGET	AZIMUTH", 2, "xxx.xx DEG				(FOR SYSTEM TEST) | ELEVATION				xx.xxx DEG", new String[] {"DSPTEM1", "DSPTEM1 +1"}, new String[] {"D", "E"}));
        NOUN_MAP.put(42, new Noun(42, "APOGEE,", 3, "xxxx.x NAUT MI				DEC ONLY | PERIGEE,					xxxx.x NAUT MI | DELTA V (REQUIRED)				xxxx.x FT/SEC", new String[] {"HAPO", "HPER", "VGDISP"}, new String[] {"Q", "Q", "S"}));
        NOUN_MAP.put(43, new Noun(43, "LATITUDE,", 3, "xxx.xx DEG				DEC ONLY | LONGITUDE,					xxx.xx DEG | ALTITUDE					xxxx.x NAUT MI", new String[] {"LAT", "LONG", "ALT"}, new String[] {"H", "H", "Q"}));
        NOUN_MAP.put(44, new Noun(44, "APOGEE,", 3, "xxxx.x NAUT MI				NO LOAD, DEC ONLY | PERIGEE,					xxxx.x NAUT MI | TFF						xxBxx  MIN/SEC", new String[] {"HAPOX", "HPERX", "TFF"}, new String[] {"Q", "Q", "L"}));
        NOUN_MAP.put(45, new Noun(45, "MARKS,", 3, "xxxxx.					NO LOAD, DEC ONLY | TFI OF NEXT BURN,				xxBxx  MIN/SEC | MGA						xxx.xx DEG", new String[] {"TRKMKCNT", "TTOGO", "+MGA"}, new String[] {"C", "L", "H"}));
        NOUN_MAP.put(46, new Noun(46, "AUTOPILOT CONFIGURATION", 1, "OCTAL ONLY", new String[] {"DAPDATR1"}, new String[] {"A"}));
        NOUN_MAP.put(47, new Noun(47, "LEM WEIGHT,", 2, "xxxxx. LBS				DEC ONLY | CSM WEIGHT					xxxxx. LBS", new String[] {"LEMMASS", "CSMMASS"}, new String[] {"KK", "KK"}));
        NOUN_MAP.put(48, new Noun(48, "GIMBAL PITCH TRIM,", 2, "xxx.xx DEG				DEC ONLY | GIMBAL ROLL TRIM				xxx.xx DEG", new String[] {"PITTIME", "ROLLTIME"}, new String[] {"NN", "NN"}));
        NOUN_MAP.put(49, new Noun(49, "DELTA R,", 3, "xxxx.x NAUT MI				DEC ONLY | DELTA V,					xxxx.x FT/SEC | RADAR DATA SOURCE CODE				xxxxx.", new String[] {"R22DISP", "R22DISP +2", "WHCHREAD"}, new String[] {"Q", "S", "C"}));
        NOUN_MAP.put(50, new Noun(50, "SPARE", 0, "", new String[] {}, new String[] {}));
        NOUN_MAP.put(51, new Noun(51, "S-BAND ANTENNA ANGLES 	PITCH", 2, "xxx.xx DEG				DEC ONLY | YAW			xxx.xx DEG", new String[] {"ALPHASB", "BETASB"}, new String[] {"H", "H"}));
        NOUN_MAP.put(52, new Noun(52, "CENTRAL ANGLE OF ACTIVE VEHICLE", 1, "xxx.xx DEG", new String[] {"ACTCENT"}, new String[] {"H"}));
        NOUN_MAP.put(53, new Noun(53, "SPARE", 0, "", new String[] {}, new String[] {}));
        NOUN_MAP.put(54, new Noun(54, "RANGE,", 3, "xxx.xx NAUT MI				DEC ONLY | RANGE RATE,					xxxx.x FT/SEC | THETA						xxx.xx DEG", new String[] {"RANGE", "RRATE", "RTHETA"}, new String[] {"JJ", "S", "H"}));
        NOUN_MAP.put(55, new Noun(55, "NO. OF APSIDAL CROSSINGS", 3, "xxxxx.					DEC ONLY | ELEVATION ANGLE					xxx.xx DEG | CENTRAL ANGLE OF PASSIVE VEHICLE		xxx.xx DEG", new String[] {"NN", "ELEV", "CENTANG"}, new String[] {"C", "H", "H"}));
        NOUN_MAP.put(56, new Noun(56, "RR LOS	AZIMUTH", 2, "xxx.xx DEG | ELEVATION				xxx.xx DEG", new String[] {"RR-AZ", "RR-ELEV"}, new String[] {"H", "H"}));
        NOUN_MAP.put(57, new Noun(57, "DELTA R", 1, "xxxx.x NAUT MI				DEC ONLY", new String[] {"DELTAR"}, new String[] {"Q"}));
        NOUN_MAP.put(58, new Noun(58, "PERIGEE ALT (POST TPI)", 3, "xxxx.x NAUT MI				DEC ONLY | DELTA V TPI					xxxx.x FT/SEC | DELTA V TPF					xxxx.x FT/SEC", new String[] {"POSTTPI", "DELVTPI", "DELVTPF"}, new String[] {"Q", "S", "S"}));
        NOUN_MAP.put(59, new Noun(59, "DELTA VELOCITY LOS", 3, "xxxx.x FT/SEC FOR EA.			DEC ONLY", new String[] {"DVLOS", "DVLOS +2", "DVLOS +4"}, new String[] {"S", "S", "S"}));
        NOUN_MAP.put(60, new Noun(60, "HORIZONTAL VELOCITY", 3, "xxxx.x FT/SEC				DEC ONLY | ALTITUDE RATE					xxxx.x FT/SEC | COMPUTED ALTITUDE				xxxxx. FEET", new String[] {"VHORIZ", "HDOTDISP", "HCALC"}, new String[] {"S", "S", "RR"}));
        NOUN_MAP.put(61, new Noun(61, "TIME TO GO IN BRAKING PHASE", 3, "xxBxx  MIN/SEC				NO LOAD, DEC ONLY | TIME FROM IGNITION				xxBxx  MIN/SEC | Page 11 | CROSS RANGE DISTANCE				xxxx.x NAUT MI", new String[] {"TTFDISP", "TTOGO", "OUTOFPLN"}, new String[] {"L", "L", "QQ"}));
        NOUN_MAP.put(62, new Noun(62, "ABSOLUTE VALUE OF VELOCITY", 3, "xxxx.x FT/SEC				NO LOAD, DEC ONLY | TIME FROM IGNITION				xxBxx  MIN/SEC | DELTA V (ACCUMULATED)				xxxx.x FT/SEC", new String[] {"ABVEL", "TTOGO", "DVTOTAL"}, new String[] {"S", "L", "S"}));
        NOUN_MAP.put(63, new Noun(63, "ABSOLUTE VALUE OF VELOCITY", 3, "xxxx.x FT/SEC				DEC ONLY | ALTITUDE RATE					xxxx.x FT/SEC | COMPUTED ALTITUDE				xxxxx. FEET", new String[] {"ABVEL", "HDOTDISP", "HCALC1"}, new String[] {"S", "S", "RR"}));
        NOUN_MAP.put(64, new Noun(64, "TIME LEFT FOR REDESIGNATION- LPD ANGLE", 3, "xxBxx					NO LOAD, DEC ONLY | ALTITUDE RATE					xxxx.x FT/SEC | COMPUTED ALTITUDE				xxxxx. FEET", new String[] {"FUNNYDSP", "HDOTDISP", "HCALC"}, new String[] {"PP", "S", "RR"}));
        NOUN_MAP.put(65, new Noun(65, "SAMPLED AGC TIME", 3, "00xxx. HRS.				DEC ONLY | (FETCHED IN INTERRUPT)				000xx. MIN				MUST LOAD 3 COMPS | 0xx.xx SEC", new String[] {"SAMPTIME", "SAMPTIME", "SAMPTIME"}, new String[] {"K", "K", "K"}));
        NOUN_MAP.put(66, new Noun(66, "LR 	RANGE", 2, "xxxxx. FEET				NO LOAD, DEC ONLY | POSITION				+0000x", new String[] {"RSTACK +6", "CHANNEL 33"}, new String[] {"W", "TT"}));
        NOUN_MAP.put(67, new Noun(67, "LRVX", 3, "xxxxx. FT/SEC | LRVY						xxxxx. FT/SEC | LRVZ						xxxxx. FT/SEC", new String[] {"RSTACK", "RSTACK +2", "RSTACK +4"}, new String[] {"X", "Y", "Z"}));
        NOUN_MAP.put(68, new Noun(68, "SLANT RANGE TO LANDING SITE", 3, "xxxx.x NAUT MI				NO LOAD, DEC ONLY | TIME TO GO IN BRAKING PHASE			xxBxx  MIN/SEC | LR ALTITUDE - COMPUTED ALTITUDE		xxxxx. FEET", new String[] {"RANGEDSP", "TTFDISP", "DELTAH"}, new String[] {"QQ", "L", "RR"}));
        NOUN_MAP.put(69, new Noun(69, "SPARE", 0, "", new String[] {}, new String[] {}));
        NOUN_MAP.put(70, new Noun(70, "AOT DETENT CODE/STAR CODE", 3, "OCTAL ONLY FOR EACH", new String[] {"AOTCODE", "AOTCODE +1", "AOTCODE +2"}, new String[] {"A", "A", "A"}));
        NOUN_MAP.put(71, new Noun(71, "AOT DETENT CODE/STAR CODE", 3, "OCTAL ONLY FOR EACH", new String[] {"AOTCODE", "AOTCODE +1", "AOTCODE +2"}, new String[] {"A", "A", "A"}));
        NOUN_MAP.put(72, new Noun(72, "RR	360 - TRUNNION ANGLE", 2, "xxx.xx DEG | SHAFT ANGLE				xxx.xx DEG", new String[] {"CDUT", "CDUS"}, new String[] {"WW", "D"}));
        NOUN_MAP.put(73, new Noun(73, "NEW RR	360 - TRUNNION ANGLE", 2, "xxx.xx DEG | SHAFT ANGLE				xxx.xx DEG", new String[] {"TANG", "TANG +1"}, new String[] {"WW", "D"}));
        NOUN_MAP.put(74, new Noun(74, "TIME FROM IGNITION", 3, "xxBxx  MIN/SEC				NO LOAD, DEC ONLY | YAW AFTER VEHICLE RISE				xxx.xx DEG | PITCH AFTER VEHICLE RISE			xxx.xx DEG", new String[] {"TTOGO", "YAW", "PITCH"}, new String[] {"L", "H", "H"}));
        NOUN_MAP.put(75, new Noun(75, "DELTA ALTITUDE CDH", 3, "xxxx.x NAUT MI				NO LOAD, DEC ONLY | DELTA TIME (CDH-CSI OR TPI-CDH)			xxBxx  MIN/SEC | DELTA TIME (TPI-CDH OR TPI-NOMTPI)		xxBxx  MIN/SEC", new String[] {"DIFFALT", "T1TOT2", "T2TOT3"}, new String[] {"Q", "L", "L"}));
        NOUN_MAP.put(76, new Noun(76, "DESIRED HORIZONTAL VELOCITY", 3, "xxxx.x FT/SEC				DEC ONLY | DESIRED RADIAL VELOCITY				xxxx.x FT/SEC | CROSS-RANGE DISTANCE				xxxx.x NAUT MI", new String[] {"ZDOTD", "RDOTD", "XRANGE"}, new String[] {"S", "S", "Q"}));
        NOUN_MAP.put(77, new Noun(77, "TIME TO ENGINE CUTOFF", 2, "xxBxx  MIN/SEC				NO LOAD, DEC ONLY | VELOCITY NORMAL TO CSM PLANE			xxxx.x FT/SEC", new String[] {"TTOGO", "YDOT"}, new String[] {"L", "S"}));
        NOUN_MAP.put(78, new Noun(78, "RR	RANGE", 2, "xxx.xx NAUT MI				NO LOAD, DEC ONLY | RANGE RATE				xxxxx. FT/SEC", new String[] {"RSTACK", "RSTACK +2"}, new String[] {"U", "V"}));
        NOUN_MAP.put(79, new Noun(79, "CURSOR ANGLE", 3, "xxx.xx DEG				DEC ONLY | SPIRAL ANGLE					xxx.xx DEG | POSITION CODE					xxxxx.", new String[] {"CURSOR", "SPIRAL", "POSCODE"}, new String[] {"D", "D", "C"}));
        NOUN_MAP.put(80, new Noun(80, "DATA INDICATOR,", 2, "xxxxx. | OMEGA						xxx.xx DEG", new String[] {"DATAGOOD", "OMEGAD"}, new String[] {"C", "H"}));
        NOUN_MAP.put(81, new Noun(81, "DELTA V (LV)", 3, "xxxx.x FT/SEC FOR EACH 			DEC ONLY | Page 12", new String[] {"DELVLVC", "DELVLVC +2", "DELVLVC +4"}, new String[] {"S", "S", "S"}));
        NOUN_MAP.put(82, new Noun(82, "DELTA V (LV)", 3, "xxxx.x FT/SEC FOR EACH			DEC ONLY", new String[] {"DELVLVC", "DELVLVC +2", "DELVLVC +4"}, new String[] {"S", "S", "S"}));
        NOUN_MAP.put(83, new Noun(83, "DELTA V (BODY)", 3, "xxxx.x FT/SEC FOR EACH			DEC ONLY", new String[] {"DELVIMU", "DELVIMU +2", "DELVIMU +4"}, new String[] {"S", "S", "S"}));
        NOUN_MAP.put(84, new Noun(84, "DELTA V (OTHER VEHICLE)", 3, "xxxx.x FT/SEC FOR EACH			DEC ONLY", new String[] {"DELVOV", "DELVOV +2", "DELVOV +4"}, new String[] {"S", "S", "S"}));
        NOUN_MAP.put(85, new Noun(85, "VG (BODY)", 3, "xxxx.x FT/SEC FOR EACH			DEC ONLY", new String[] {"VGBODY", "VGBODY +2", "VGBODY +4"}, new String[] {"S", "S", "S"}));
        NOUN_MAP.put(86, new Noun(86, "VG (LV)", 3, "xxxx.x FT/SEC FOR EACH			DEC ONLY", new String[] {"DELVLVC", "DELVLVC +2", "DELVLVC +4"}, new String[] {"S", "S", "S"}));
        NOUN_MAP.put(87, new Noun(87, "BACKUP OPTICS LOS	AZIMUTH", 2, "xxx.xx DEG | ELEVATION		xxx.xx DEG", new String[] {"AZ", "EL"}, new String[] {"D", "D"}));
        NOUN_MAP.put(88, new Noun(88, "HALF UNIT SUN OR PLANET VECTOR", 3, ".xxxxx FOR EACH				DEC ONLY", new String[] {"STARAD", "STARAD +2", "STARAD +4"}, new String[] {"B", "B", "B"}));
        NOUN_MAP.put(89, new Noun(89, "LANDMARK	LATITUDE", 3, "xx.xxx DEG				DEC ONLY | LONGITUDE/2			xx.xxx DEC | ALTITUDE			xxx.xx NAUT MI", new String[] {"LANDLAT", "LANDLONG", "LANDALT"}, new String[] {"G", "G", "JJ"}));
        NOUN_MAP.put(90, new Noun(90, "Y", 3, "xxx.xx NM				DEC ONLY | Y DOT						xxxx.x FPS | PSI						xxx.xx DEG", new String[] {"RANGE", "RRATE", "RTHETA"}, new String[] {"JJ", "S", "H"}));
        NOUN_MAP.put(91, new Noun(91, "ALTITUDE", 3, "xxxxxB. NAUT MI | VELOCITY					xxxxx.  FT/SEC | FLIGHT PATH ANGLE				xxx.xx  DEG", new String[] {"P21ALT", "P21VEL", "P21GAM"}, new String[] {"Q", "P", "H"}));
        NOUN_MAP.put(92, new Noun(92, "SPARE", 0, "", new String[] {}, new String[] {}));
        NOUN_MAP.put(93, new Noun(93, "DELTA GYRO ANGLES", 3, "xx.xxx DEG FOR EACH", new String[] {"OGC", "OGC +2", "OGC +4"}, new String[] {"G", "G", "G"}));
        NOUN_MAP.put(94, new Noun(94, "SPARE", 0, "", new String[] {}, new String[] {}));
        NOUN_MAP.put(95, new Noun(95, "SPARE", 0, " | 96 	SPARE", new String[] {}, new String[] {}));
        NOUN_MAP.put(97, new Noun(97, "SYSTEM TEST INPUTS", 3, "xxxxx. FOR EACH", new String[] {"DSPTEM1", "DSPTEM1 +1", "DSPTEM1 +2"}, new String[] {"C", "C", "C"}));
        NOUN_MAP.put(98, new Noun(98, "SYSTEM TEST RESULTS AND INPUTS", 3, "xxxxx. | .xxxxx | xxxxx.", new String[] {"DSPTEM2", "DSPTEM2 +1", "DSPTEM2 +2"}, new String[] {"C", "B", "C"}));
        NOUN_MAP.put(99, new Noun(99, "RMS IN POSITION", 3, "xxxxx. FT				DEC ONLY | RMS IN VELOCITY					xxxx.x FT/SEC | RMS IN BIAS					xx.xxx RADIANS | Page 13", new String[] {"WWPOS", "WWVEL", "WWBIAS"}, new String[] {"XX", "YY", "AAA"}));
        // Scale types
        SCALE_MAP.put("A", "OCTAL");
        SCALE_MAP.put("C", "WHOLE");
        SCALE_MAP.put("G", "DP DEGREES(90)");
        SCALE_MAP.put("H", "DP DEGREES (360)");
        SCALE_MAP.put("K", "TIME (HR, MIN, SEC)");
        SCALE_MAP.put("L", "TIME (MIN/SEC)");
        SCALE_MAP.put("N", "TIME(SEC) DP");
        SCALE_MAP.put("P", "VELOCITY 2");
        SCALE_MAP.put("Q", "POSITION 4");
        SCALE_MAP.put("S", "VELOCITY 3");
        SCALE_MAP.put("U", "RENDEZVOUS");
        SCALE_MAP.put("V", "RENDEZVOUS");
        SCALE_MAP.put("W", "LANDING RADAR");
        SCALE_MAP.put("X", "LANDING RADAR");
        SCALE_MAP.put("Y", "LANDING RADAR");
        SCALE_MAP.put("Z", "LANDING RADAR");
        SCALE_MAP.put("AA", "INITIAL/FINAL");
        SCALE_MAP.put("BB", "ALTITUDE RATE");
        SCALE_MAP.put("CC", "FORWARD/LATERAL");
        SCALE_MAP.put("DD", "ROTATIONAL HAND");
        SCALE_MAP.put("EE", "OPTICAL TRACKER");
        SCALE_MAP.put("JJ", "POSITION5");
        SCALE_MAP.put("NN", "TRIM DEGREES 2");
        SCALE_MAP.put("PP", "2 INTEGERS");
        SCALE_MAP.put("QQ", "POSITION7");
        SCALE_MAP.put("RR", "COMPUTED ALTITUDE");
        SCALE_MAP.put("SS", "DP DEGREES");
        SCALE_MAP.put("TT", "LANDING RADAR");
        SCALE_MAP.put("XX", "POSITION 9");
        SCALE_MAP.put("YY", "VELOCITY 4");
        SCALE_MAP.put("AAA", "RADIANS");
        // Alarm codes (octal)
        ALARM_MAP.put("00105", new Alarm("00105", "**", "AOTMARK SYSTEM IN USE", ""));
        ALARM_MAP.put("00107", new Alarm("00107", "", "MORE THAN 5 MARK PAIRS", "AOTMARK"));
        ALARM_MAP.put("00111", new Alarm("00111", "", "MARK MISSING", "AOTMARK"));
        ALARM_MAP.put("00112", new Alarm("00112", "", "MARK OR MARK REJECT NOT BEING ACCEPTED", "AOTMARK"));
        ALARM_MAP.put("00113", new Alarm("00113", "", "NO INBITS", "AOTMARK"));
        ALARM_MAP.put("00114", new Alarm("00114", "", "MARK MADE BUT NOT DESIRED", "AOTMARK"));
        ALARM_MAP.put("00115", new Alarm("00115", "", "NO MARKS IN LAST PAIR TO REJECT", "AOTMARK"));
        ALARM_MAP.put("00206", new Alarm("00206", "", "ZERO ENCODE NOT ALLOWED WITH COARSE ALIGN", "IMU MODE SWITCHING"));
        ALARM_MAP.put("00206", new Alarm("00206", "", "+ GIMBAL LOC", ""));
        ALARM_MAP.put("00207", new Alarm("00207", "", "ISS TURNON REQUEST NOT PRESENT FOR 90 SEC", "T4RUPT"));
        ALARM_MAP.put("00210", new Alarm("00210", "", "IMU NOT OPERATING", "IMU MODE SWITCH, IMU-2, RD2, P51, P57"));
        ALARM_MAP.put("00211", new Alarm("00211", "", "COARSE ALIGN ERROR", "IMU MODE SWITCH"));
        ALARM_MAP.put("00212", new Alarm("00212", "", "PIPA FAIL BUT PIPA IS NOT BEING USED", "IMU MODE SWITCH, T4RPT"));
        ALARM_MAP.put("00213", new Alarm("00213", "", "IMU NOT OPERATING WITH TURN-ON REQUEST", "T4RUPT"));
        ALARM_MAP.put("00214", new Alarm("00214", "", "PROGRAM USING IMU WHEN TURNED OFF", "T4RUPT"));
        ALARM_MAP.put("00217", new Alarm("00217", "", "BAD RETURN FROM IMUSTALL", "P51, P52, P57"));
        ALARM_MAP.put("00220", new Alarm("00220", "", "IMU NOT ALIGNED - NO REFSMMAT", "R02, R47"));
        ALARM_MAP.put("00401", new Alarm("00401", "", "DESIRED GIMBAL ANGLE YIELDS GIMBAL LOCK", "INF ALIGN, IMU-2,"));
        ALARM_MAP.put("00402", new Alarm("00402", "", "FINDCDUW NOT CONTROLLING ATTITUDE", "FINDCDUW"));
        ALARM_MAP.put("00404", new Alarm("00404", "", "TWO STARS NOT AVAILABLE IN ANY DETENT", "R59, LUNAR SURFACE"));
        ALARM_MAP.put("00405", new Alarm("00405", "", "TWO STARS NOT AVAILABLE", "P52"));
        ALARM_MAP.put("00421", new Alarm("00421", "", "W-MATRIX OVERFLOW", "INTEGRV"));
        ALARM_MAP.put("00430", new Alarm("00430", "**", "ACCELERATION OVERFLOW IN INTEGRATION", "ORBITAL INTEGRATION"));
        ALARM_MAP.put("00501", new Alarm("00501", "P", "RADAR ANTENNA OUT OF LIMITS", "R23"));
        ALARM_MAP.put("00502", new Alarm("00502", "", "BAD RADAR GIMBAL ANGLE INPUT", "V41N72"));
        ALARM_MAP.put("00503", new Alarm("00503", "P", "RADAR ANTENNA DESIGNATE FAIL", "R21, NON-P IN V41N72"));
        ALARM_MAP.put("00510", new Alarm("00510", "", "RADAR AUTO DESCRETE NOT PRESENT", "R25"));
        ALARM_MAP.put("00511", new Alarm("00511", "", "LR NOT IN POSITION 2 OR REPOSITIONING", "SERVICER"));
        ALARM_MAP.put("00514", new Alarm("00514", "P", "RR GOES OUT OF AUTO MODE WHILE IN USE", "P20"));
        ALARM_MAP.put("00515", new Alarm("00515", "", "RR CDU FAIL DISCRETE PRESENT", "R25"));
        ALARM_MAP.put("00520", new Alarm("00520", "", "RADAR RUPT NOT EXPECTED AT THIS TIME", "RADAR READ"));
        ALARM_MAP.put("00521", new Alarm("00521", "", "COULD NOT READ RADAR", "P20"));
        ALARM_MAP.put("00522", new Alarm("00522", "", "LANDING RADAR POSITION CHANGE", "RADAR READ"));
        ALARM_MAP.put("00523", new Alarm("00523", "P", "LR ANTENNA DIDN'T ACHIEVE POSITION 2", "SERVICER, V60 (NON-P IN V60)"));
        ALARM_MAP.put("00525", new Alarm("00525", "P", "DELTA THETA GREATER THAN 3 DEGREES", "R22"));
        ALARM_MAP.put("00526", new Alarm("00526", "P", "RANGE GREATER THAN 400 NAUT. MILES", "P20, P22"));
        ALARM_MAP.put("00527", new Alarm("00527", "P", "LOS NOT IN MODE II COVERAGE WHILE ON", "R21, R24"));
        ALARM_MAP.put("00530", new Alarm("00530", "P", "LOS NOT IN MODE2 COVERAGE", "R21"));
        ALARM_MAP.put("00600", new Alarm("00600", "", "IMAGINARY ROOTS ON FIRST ITERATION", "P32, P72"));
        ALARM_MAP.put("00601", new Alarm("00601", "", "PERIGEE ALTITUDE CSI LT PMIN1", "P32, P72."));
        ALARM_MAP.put("00602", new Alarm("00602", "", "PERIGEE ALTITUDE CDH LT PMIN2", "P32, P72."));
        ALARM_MAP.put("00603", new Alarm("00603", "", "CSI TO CDH TIME LT TMIN12", "P32, P72, P33, P73"));
        ALARM_MAP.put("00604", new Alarm("00604", "", "CDH TO TPI TIME LT TMIN23", "P32, P72,"));
        ALARM_MAP.put("00605", new Alarm("00605", "", "NUMBER OF ITERATIONS EXCEEDS LOOP MAXIMUM", "P32, P72"));
        ALARM_MAP.put("00606", new Alarm("00606", "", "DV EXCEEDS MAXIMUM", "P32, P72"));
        ALARM_MAP.put("00607", new Alarm("00607", "**", "NO SOLN FROM TIME-THETA OR TIME-RADIUS", "TIMETHET, TIMERAD"));
        ALARM_MAP.put("00611", new Alarm("00611", "", "NO TIG FOR GIVEN ELEV ANGLE", "P34, P74"));
        ALARM_MAP.put("00701", new Alarm("00701", "", "ILLEGAL OPTION CODE SELECTED", "P57"));
        ALARM_MAP.put("00777", new Alarm("00777", "", "PIPA FAIL CAUSED THE ISS WARNING", "T4RUPT"));
        ALARM_MAP.put("01102", new Alarm("01102", "", "AGC SELF TEST ERROR", "SELF CHECK"));
        ALARM_MAP.put("01103", new Alarm("01103", "**", "UNUSED CCS BRANCH EXECUTED", "ABORT"));
        ALARM_MAP.put("01104", new Alarm("01104", "*", "DELAY ROUTINE BUSY", "EXEC"));
        ALARM_MAP.put("01105", new Alarm("01105", "", "DOWNLINK TOO FAST", "T4RUPT"));
        ALARM_MAP.put("01106", new Alarm("01106", "", "UPLINK TOO FAST", "T4RUPT"));
        ALARM_MAP.put("01107", new Alarm("01107", "", "PHASE TABLE FAILURE.  ASSUME", "RESTART"));
        ALARM_MAP.put("01201", new Alarm("01201", "*", "EXECUTIVE OVERFLOW - NO VAC AREAS", "EXEC"));
        ALARM_MAP.put("01202", new Alarm("01202", "*", "EXECUTIVE OVERFLOW - NO CORE SETS", "EXEC"));
        ALARM_MAP.put("01203", new Alarm("01203", "*", "WAITLIST OVERFLOW - TOO MANY TASKS", "WAITLIST"));
        ALARM_MAP.put("01204", new Alarm("01204", "**", "WAITLIST, VARDELAY, FIXDELAY, OR LONGCALL", "WAITLIST ROUTINES"));
        ALARM_MAP.put("01206", new Alarm("01206", "**", "SECOND JOB ATTEMPTS TO GO TO SLEEP", "PINBALL"));
        ALARM_MAP.put("01206", new Alarm("01206", "", "VIA KYBD AND DISPLAY PROGRAM", ""));
        ALARM_MAP.put("01207", new Alarm("01207", "*", "NO VAC AREAS FOR MARKS", "AOTMARK"));
        ALARM_MAP.put("01210", new Alarm("01210", "*", "TWO PROGRAMS USING DEVICE AT SAME TIME", "MODE SWITCHING"));
        ALARM_MAP.put("01211", new Alarm("01211", "*", "ILLEGAL INTERRUPT OF EXTENDED VERB", "AOTMARK"));
        ALARM_MAP.put("01301", new Alarm("01301", "", "ARCSIN-ARCCOS ARGUMENT TOO LARGE", "INTERPRETER"));
        ALARM_MAP.put("01302", new Alarm("01302", "**", "SQRT CALLED WITH NEGATIVE ARGUMENT", "INTERPRETER"));
        ALARM_MAP.put("01406", new Alarm("01406", "", "BAD RETURN FROM ROOTPSRS", "DESCENT GUIDANCE EQS."));
        ALARM_MAP.put("01406", new Alarm("01406", "**", "BAD RETURN FROM ROOTPSRS", "IGNITION ALGORITHM"));
        ALARM_MAP.put("01407", new Alarm("01407", "", "VG INCREASING (DELTA-V ACCUMULATED", "S40.8"));
        ALARM_MAP.put("01410", new Alarm("01410", "", "UNINTENTIONAL OVERFLOW IN GUIDANCE", "DESCENT GUIDANCE EQS."));
        ALARM_MAP.put("01412", new Alarm("01412", "", "DESCENT IGNALG NOT CONVERGING", "P63"));
        ALARM_MAP.put("01501", new Alarm("01501", "**", "KEYBOARD AND DISPLAY ALARM DURING", "PINBALL"));
        ALARM_MAP.put("01501", new Alarm("01501", "", "INTERNAL USE (NVSUB). ABORT", ""));
        ALARM_MAP.put("01502", new Alarm("01502", "**", "ILLEGAL FLASHING DISPLAY", "GOPLAY"));
        ALARM_MAP.put("01520", new Alarm("01520", "", "V37 REQUEST NOT PERMITTED AT THIS TIME", "V37"));
        ALARM_MAP.put("01600", new Alarm("01600", "", "OVERFLOW IN DRIFT TEST", "IMU 4"));
        ALARM_MAP.put("01601", new Alarm("01601", "", "BAD IMU TORQUE", "OPT PRE ALIGN CALIB"));
        ALARM_MAP.put("01601", new Alarm("01601", "", "IMU 4 (LEM)", ""));
        ALARM_MAP.put("01703", new Alarm("01703", "", "IGNITION TIME SLIPPED", "MIDTOAVE"));
        ALARM_MAP.put("01706", new Alarm("01706", "", "INCORRECT PROGRAM REQUESTED FOR VEHICLE", ""));
        ALARM_MAP.put("02000", new Alarm("02000", "*", "DAP STILL IN PROGRESS AT NEXT TIMES RUPT", "DAP"));
        ALARM_MAP.put("02001", new Alarm("02001", "", "JET FAILURES HAVE DISABLED Y-Z TRANS.", "DAP"));
        ALARM_MAP.put("02002", new Alarm("02002", "", "JET FAILURES HAVE DISABLED X TRANSLATION", "DAP"));
        ALARM_MAP.put("02003", new Alarm("02003", "", "JET FAILURES HAVE DISABLED P-ROTATION", "DAP"));
        ALARM_MAP.put("02004", new Alarm("02004", "", "JET FAILURES HAVE DISABLED U-V ROTATION", "DAP"));
        ALARM_MAP.put("03777", new Alarm("03777", "", "ICDU FAIL CAUSED THE ISS WARNING", "T4RUPT"));
        ALARM_MAP.put("04777", new Alarm("04777", "", "ICDU, PIPA FAILS CAUSED THE ISS WARNING", "T4RUPT"));
        ALARM_MAP.put("07777", new Alarm("07777", "", "IMU FAIL CAUSED THE ISS WARNING", "T4RUPT"));
        ALARM_MAP.put("10777", new Alarm("10777", "", "IMU, PIPA FAILS CAUSED THE ISS WARNING", "T4RUPT"));
        ALARM_MAP.put("13777", new Alarm("13777", "", "IMU, ICDU FAILS CAUSED THE ISS WARNING", "T4RUPT"));
        ALARM_MAP.put("14777", new Alarm("14777", "", "IMU, ICDU, PIPA FAILS CAUSED THE ISS WARNING", "T4RUPT"));
        // Checklist codes
        CHECKLIST_MAP.put("00014", new Checklist("00014", "PROCEED", "DO IMU FINE ALIGN ROUTINE", "P51, P63, P57"));
        CHECKLIST_MAP.put("00014", new Checklist("00014", "ENTER", "DO LANDING SITE DETERMINATION (N89DISP)", "P57OPTION2"));
        CHECKLIST_MAP.put("00015", new Checklist("00015", "PERFORM", "CELESTIAL BODY ACQUISITION", "R51, P51"));
        CHECKLIST_MAP.put("00062", new Checklist("00062", "SWITCH", "AGC POWER DOWN", "P06"));
        CHECKLIST_MAP.put("00201", new Checklist("00201", "SWITCH", "RR MODE TO AUTOMATIC", "P20, P22, R04"));
        CHECKLIST_MAP.put("00203", new Checklist("00203", "SWITCH", "GUID CONTROL TO GNC, MODE TO AUTO...", "P12, P42, P71"));
        CHECKLIST_MAP.put("00205", new Checklist("00205", "PERFORM", "MANUAL ACQUISITION OF RR", "R23"));
        CHECKLIST_MAP.put("00500", new Checklist("00500", "SWITCH", "LR ANTENNA TO POSITION 1", "P63"));
        // Option codes
        OPTION_MAP.put("00001", new Option("00001", "SPECIFY IMU ORIENTATION", "1=PREF 2=NOM 3=REFSMMAT", "P52		ALL"));
        OPTION_MAP.put("00002", new Option("00002", "SPECIFY VEHICLE", "1=THIS 2=OTHER", "P21,R30		ALL"));
        OPTION_MAP.put("00003", new Option("00003", "SPECIFY TRACKING ATTITUDE", "1=PREFERRED 2=OTHER", "R63		ALL"));
        OPTION_MAP.put("00004", new Option("00004", "SPECIFY RADAR", "1=RR 2=LR", "R04		SUNDANCE + LUMINARY"));
        OPTION_MAP.put("00005", new Option("00005", "SPECIFY SOR PHASE", "1=FIRST 2=SECOND", "P38		COLOSSUS + LUMINARY"));
        OPTION_MAP.put("00006", new Option("00006", "SPECIFY RR COARSE ALIGN OPTION", "1=LOCKON 2=CONTINUOUS DESIG.", "V41N72		SUNDANCE + LUMINARY"));
        OPTION_MAP.put("00010", new Option("00010", "SPECIFY ALIGNMENT MODE", "0=ANY TIME 1=REFSMMAT +G", "P57		LUMINARY"));
        OPTION_MAP.put("00012", new Option("00012", "SPECIFY CSM ORBIT OPTION", "1=NO ORBIT CHANGE 2=CHANGE", "P22		LUMINARY"));
    }

    /** Verb descriptions by verb number. */
    public static Map<Integer, String> verbs() {
        return Collections.unmodifiableMap(VERB_MAP);
    }

    /** Noun table by noun number. */
    public static Map<Integer, Noun> nouns() {
        return Collections.unmodifiableMap(NOUN_MAP);
    }

    /** Noun scale type (A, B, C ... AAA) descriptions. */
    public static Map<String, String> scaleTypes() {
        return Collections.unmodifiableMap(SCALE_MAP);
    }

    /** Alarm codes keyed by 5-digit octal string. */
    public static Map<String, Alarm> alarms() {
        return Collections.unmodifiableMap(ALARM_MAP);
    }

    /** Checklist codes keyed by 5-digit octal string. */
    public static Map<String, Checklist> checklists() {
        return Collections.unmodifiableMap(CHECKLIST_MAP);
    }

    /** Option codes keyed by 5-digit octal string. */
    public static Map<String, Option> options() {
        return Collections.unmodifiableMap(OPTION_MAP);
    }
}