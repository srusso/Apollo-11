package apollo.comanche055;

import apollo.agc.Agc;

/**
 * PINBALL_NOUN_TABLES.agc (Comanche 055, pages 268-284).
 *
 * Noun tables for PINBALL (the DSKY keyboard/display program) and the routines that read
 * them. Tables are static final arrays; entries that are erasable addresses (ECADR) are
 * kept symbolically as {@link Ad} values, entries that are octal constants as numbers.
 *
 * <pre>
 *  THE FOLLOWING REFERS TO THE NOUN TABLES
 *  COMPONENT CODE NUMBER        INTERPRETATION
 *  00000                1 COMPONENT
 *  00001                2 COMPONENT
 *  00010                3 COMPONENT
 *  X1XXX                BIT 4 = 1. DECIMAL ONLY
 *  1XXXX                BIT 5 = 1. NO LOAD
 *  END OF COMPONENT CODE NUMBERS
 *  SF ROUTINE CODE NUMBER    INTERPRETATION
 *  00000        OCTAL ONLY
 *  00001        STRAIGHT FRACTIONAL
 *  00010        CDU DEGREES (XXX.XX)
 *  00011        ARITHMETIC SF
 *  00100        ARITH DP1    OUT(MULT BY 2EXP14 AT END)    IN(STRAIGHT)
 *  00101        ARITH DP2    OUT(STRAIGHT)            IN(SL 7 AT END)
 *  00110        Y OPTICS DEGREES (XX.XXX MAX 89.999)
 *  00111        ARITH DP3    OUT ( SL 7 AT END)        IN ( STRAIGHT)
 *  01000        WHOLE HOURS IN R1, WHOLE MINUTES (MOD 60) IN R2,
 *             SECONDS (MOD 60) 0XX.XX IN R3. *** ALARMS IF USED WITH OCTAL
 *  01001        MINUTES (MOD 60) IN D1D2, D3 BLANK, SECONDS (MOD 60) IN D4D5
 *                 LIMITS TO 59B59 IF MAG EXCEEDS THIS VALUE.
 *                 ALARMS IF USED WITH OCTAL ******** IN (ALARM)
 *  01010        ARITH DP4    OUT (STRAIGHT)            IN (SL 3 AT END)
 *  01011        ARITH1 SF    OUT(MULT BY 2EXP14 AT END)    IN(STRAIGHT)
 *  01100        2 INTEGERS IN D1D2, D4D5, D3 BLANK.
 *                 ALARMS IF USED WITH OCTAL ******** IN (ALARM)
 *  01101    DP STRAIGHT FRACTIONAL
 *  END OF SF ROUTINE CODE NUMBERS
 *      SF CONSTANT CODE NUMBER    INTERPRETATION
 *     00000            WHOLE                USE ARITH
 *     00000            DP TIME SEC (XXX.XX SEC)    USE ARITHDP1
 *     00001            SPARE
 *     00010            CDU DEGREES             USE CDU DEGREES
 *     00010            Y OPTICS DEGREES    USE Y OPTICS DEGREES
 *     00011            DP DEGREES (90) XX.XXX DEG    USE ARITHDP3
 *     00100            DP DEGREES (360) XXX.XX DEG    USE ARITHDP4
 *     00101            DEGREES (180) XXX.XX DEG    USE ARITH
 *     00110            WEIGHT2 (XXXXX. LBS)        USE ARITH1
 *     00111            POSITION5 (XXX.XX NAUTICAL MILES)
 *                                 USE ARITHDP3
 *     01000            POSITION4 (XXXX.X NAUTICAL MILES)
 *                                 USE ARITHDP3
 *     01001            VELOCITY2 (XXXXX. FT/SEC)   USE ARITHDP4
 *     01010            VELOCITY3 (XXXX.X FT/SEC)   USE ARITHDP3
 *     01011            ELEVATION DEGREES (89.999MAX) USE ARITH
 *     01100            TRIM DEGREES (XXX.XX DEG)    USE ARITH
 *     01101            INERTIA (XXXXXBB. SLUG FT FT)  USE ARITH
 *     01101            THRUST MOMENT (XXXXXBB.FT LBS) USE ARITH
 *     01110            VELOCITY/2VS (XXXXX. FT/SEC)USE ARITHDP4
 *     01111            POSITION6 (XXXX.X NAUT MI)  USE ARITHDP3
 *     10000            DRAG ACCELERATION (XXX.XX G)USE ARITHDP2
 *     10001            POSITION8 (XXXX.X NAUT MI)  USE ARITHDP3
 *     10010            POSITION9 (XXXXX. FT)        USE ARITHDP3
 *     10011            VELOCITY4 (XXXX.X FT/SEC)   USE ARITHDP2
 *      END OF SF CONSTANT CODE NUMBERS
 *  FOR GREATER THAN SINGLE PRECISION SCALES, PUT ADDRESS IN MAJOR PART INTO
 *  NOUN TABLES.
 *  OCTAL LOADS PLACE +0 INTO MAJOR PART, DATA INTO MINOR PART.
 *  OCTAL DISPLAYS SHOW MINOR PART ONLY.
 *  TO GET AT BOTH MAJOR AND MINOR PARTS(IN OCTAL), USE NOUN 01.
 *  A NOUN MAY BE DECLARED :DECIMAL ONLY: BY MAKING BIT4=1 OF ITS COMPONENT
 *  CODE NUMBER.  IF THIS NOUN IS USED WITH ANY OCTAL DISPLAY VERB, OR IF
 *  DATA IS LOADED IN OCTAL, IT ALARMS.
 *  IN LOADING AN :HOURS, MINUTES, SECONDS: NOUN, ALL 3 WORDS MUST BE
 *  LOADED, OR ALARM.
 *  ALARM IF AN ATTEMPT IS MADE TO LOAD :SPLIT MINUTES/SECONDS: (MMBSS).
 *  THIS IS USED FOR DISPLAY ONLY.
 * </pre>
 */
public class PinballNounTables {
    private final Agc agc;

    /** A table word: either an octal constant or the ECADR of an erasable symbol. */
    public static final class Ad {
        public final String symbol;   // null for plain octal constants
        public final int value;
        private Ad(String symbol, int value) { this.symbol = symbol; this.value = value; }
        @Override public String toString() { return symbol != null ? "ECADR " + symbol : Integer.toOctalString(value); }
    }

    private static Ad oct(int v) { return new Ad(null, v); }
    private static Ad ecadr(String s) { return new Ad(s, 0); }

    /** MIXCON = OCT50: first mixed noun = 40 decimal. */
    public static final int MIXCON = 050;

    public PinballNounTables(Agc agc) {
        this.agc = agc;
    }

    /**
     * LODNNTAB: loads NNADTEM with the NNADTAB entry, NNTYPTEM with the NNTYPTAB entry. If the
     * noun is mixed, IDAD1TEM/IDAD2TEM/IDAD3TEM get the three IDADDTAB entries and RUTMXTEM
     * the RUTMXTAB entry. MIXBR is set to 1 (normal) or 2 (mixed).
     */
    public void lodnntab() {
        int noun = Agc.toSigned(agc.get("NOUNREG"));
        Ad nnad = NNADTAB[noun];
        agc.set("NNADTEM", nnad.value);          // (ECADR entries are resolved by the caller)
        agc.set("NNTYPTEM", NNTYPTAB[noun]);
        if (noun - MIXCON < 0 || noun < 040) {   // noun number L/ first mixed noun
            agc.set("MIXBR", 1);                 // normal
            return;
        }
        agc.set("MIXBR", 2);                     // mixed
        agc.set("RUTMXTEM", RUTMXTAB[noun - 040]);
        int idx = nnad.value & 01777;            // index into IDADDTAB
        agc.set("IDAD1TEM", IDADDTAB[idx].value);
        agc.set("IDAD2TEM", IDADDTAB[idx + 1].value);
        agc.set("IDAD3TEM", IDADDTAB[idx + 2].value);
    }

    /** GTSFOUT: loads SFTEMP1, SFTEMP2 with the DP SFOUTAB entries (2*SFCONUM in A). */
    public void gtsfout(int twiceSfconum) {
        agc.set("SFTEMP1", SFOUTAB[twiceSfconum]);
        agc.set("SFTEMP2", SFOUTAB[twiceSfconum + 1]);
    }

    /** GTSFIN: loads SFTEMP1, SFTEMP2 with the DP SFINTAB entries. */
    public void gtsfin(int twiceSfconum) {
        agc.set("SFTEMP1", SFINTAB[twiceSfconum]);
        agc.set("SFTEMP2", SFINTAB[twiceSfconum + 1]);
    }

    /** NNADTAB: normal nouns 00-39 then mixed nouns 40-99 (address or IDADDTAB index+component code). */
    public static final Ad[] NNADTAB = {
        oct(0), // 00 	NOT IN USE
        oct(040000), // 01 	SPECIFY MACHINE ADDRESS (FRACTIONAL)
        oct(040000), // 02 	SPECIFY MACHINE ADDRESS (WHOLE)
        oct(040000), // 03	SPECIFY MACHINE ADDRESS (DEGREES)
        oct(0), // 04	SPARE
        ecadr("DSPTEM1"), // 05 	ANGULAR ERROR/DIFFERENCE
        ecadr("OPTION1"), // 06	OPTION CODE
        ecadr("XREG"), // 07	ECADR OF WORD TO BE MODIFIED
        ecadr("ALMCADR"), // 08 	ALARM DATA
        ecadr("FAILREG"), // 09	ALARM CODES
        oct(077776), // 10	CHANNEL TO BE SPECIFIED
        ecadr("TCSI"), // 11	TIG OF CSI (HRS,MIN,SEC)
        ecadr("OPTIONX"), // 12	OPTION CODE
        ecadr("TCDH"), // 13	TIG OF CDH (HRS,MIN,SEC)
        oct(0), // 14	SPARE
        oct(077777), // 15	INCREMENT MACHINE ADDRESS
        ecadr("DSPTEMX"), // 16	TIME OF EVENT (HRS,MIN,SEC)
        ecadr("CPHIX"), // 17	ASTRONAUT TOTAL ATTITUDE
        ecadr("THETAD"), // 18	AUTO MANEUVER BALL ANGLES
        ecadr("THETAD"), // 19	BYPASS ATTITUDE TRIM MANEUVER
        ecadr("CDUX"), // 20	ICDU ANGLES
        ecadr("PIPAX"), // 21	PIPAS
        ecadr("THETAD"), // 22	NEW ICDU ANGLES
        oct(0), // 23	SPARE
        ecadr("DSPTEM2 +1"), // 24	DELTA TIME FOR AGC CLOCK(HRS,MIN,SEC)
        ecadr("DSPTEM1"), // 25	CHECKLIST
        ecadr("DSPTEM1"), // 26	PRIO/DELAY, ADRES, BBCON
        ecadr("SMODE"), // 27	SELF TEST ON/OFF SWITCH
        oct(0), // 28	SPARE
        ecadr("DSPTEM1"), // 29	XSM LAUNCH AZIMUTH
        ecadr("DSPTEM1"), // 30	TARGET CODES
        ecadr("DSPTEM1"), // 31	TIME OF LANDING SITE (HRS,MIN,SEC)
        ecadr("-TPER"), // 32	TIME TO PERIGEE (HRS,MIN,SEC)
        ecadr("TIG"), // 33	TIME OF IGNITION (HRS,MIN,SEC)
        ecadr("DSPTEM1"), // 34	TIME OF EVENT (HRS,MIN,SEC)
        ecadr("TTOGO"), // 35	TIME TO GO TO EVENT (HRS,MIN,SEC)
        ecadr("TIME2"), // 36	TIME OF AGC CLOCK (HRS,MIN,SEC)
        ecadr("TTPI"), // 37	TIG OF TPI (HRS,MIN,SEC)
        ecadr("TET"), // 38	TIME OF STATE VECTOR
        ecadr("T3TOT4"), // 39	DELTA TIME TO TRANSFER (HRS,MIN,SEC)
        oct(064000), // 40	TIME TO IGNITION/CUTOFF
        oct(02003), // 41	TARGET	AZIMUTH
        oct(024006), // 42	APOGEE
        oct(024011), // 43	LATITUDE
        oct(064014), // 44	APOGEE
        oct(064017), // 45	MARKS (VHF - OPTICS)
        oct(02022), // 46	AUTOPILOT CONFIGURATION
        oct(022025), // 47	THIS VEHICLE WEIGHT
        oct(022030), // 48	PITCH TRIM
        oct(024033), // 49	DELTA R
        oct(064036), // 50	SPLASH ERROR
        oct(022041), // 51	S-BAND ANTENNA	PITCH
        oct(044), // 52	CENTRAL ANGLE OF ACTIVE VEHICLE
        oct(024047), // 53	RANGE
        oct(024052), // 54	RANGE
        oct(024055), // 55	PERIGEE CODE
        oct(022060), // 56	REENTRY ANGLE,
        oct(020063), // 57	DELTA R
        oct(024066), // 58	PERIGEE ALT
        oct(024071), // 59	DELTA VELOCITY LOS
        oct(024074), // 60	GMAX
        oct(024077), // 61	IMPACT LATITUDE
        oct(024102), // 62	INERTIAL VEL MAG (V1)
        oct(064105), // 63	RANGE 297,431 TO SPLASH (RTGO)
        oct(024110), // 64	DRAG ACCELERATION
        oct(024113), // 65	SAMPLED AGC TIME (HRS,MIN,SEC)
        oct(024116), // 66	COMMAND BANK ANGLE (BETA)
        oct(024121), // 67	RANGE TO TARGET
        oct(024124), // 68	COMMAND BANK ANGLE (BETA)
        oct(024127), // 69	BETA
        oct(04132), // 70	STAR CODE
        oct(04135), // 71	STAR CODE
        oct(024140), // 72	DELT ANG
        oct(04143), // 73	ALTITUDE
        oct(04146), // 74	COMMAND BANK ANGLE (BETA)
        oct(064151), // 75	DELTA ALTITUDE CDH
        oct(0), // 76	SPARE
        oct(0), // 77	SPARE
        oct(0), // 78	SPARE
        oct(0), // 79	SPARE
        oct(064170), // 80	TIME TO IGNITION/CUTOFF
        oct(024173), // 81	DELTA V (LV)
        oct(024176), // 82	DELTA V (LV)
        oct(024201), // 83	DELTA V (BODY)
        oct(024204), // 84	DELTA V (OTHER VEHICLE)
        oct(024207), // 85	VG (BODY)
        oct(024212), // 86	DELTA V (LV)
        oct(02215), // 87	MARK DATA	SHAFT
        oct(024220), // 88	HALF UNIT SUN OR PLANET VECTOR
        oct(024223), // 89	LANDMARK	LATITUDE
        oct(024226), // 90	Y
        oct(02231), // 91	OCDU ANGLES	SHAFT
        oct(02234), // 92	NEW OPTICS ANGLES SHAFT
        oct(04237), // 93	DELTA GYRO ANGLES
        oct(02242), // 94	NEW OPTICS ANGLES SHAFT
        oct(04245), // 95	PREFERRED ATTITUDE ICDU ANGLES
        oct(04250), // 96	+X-AXIS ATTITUDE ICDU ANGLES
        oct(04253), // 97	SYSTEM TEST INPUTS
        oct(04256), // 98	SYSTEM TEST RESULTS
        oct(024261), // 99	RMS IN POSITION
    };

    /** NNTYPTAB: component code / SF routine / SF constant per noun. */
    public static final int[] NNTYPTAB = {
        00, // 00	NOT IN USE
        04040, // 01	3COMP FRACTIONAL
        04140, // 02	3COMP WHOLE
        04102, // 03	3COMP CDU DEGREES
        00, // 04	SPARE
        0504, // 05	1COMP DPDEG(360)
        02000, // 06	2COMP OCTAL ONLY
        04000, // 07 	3COMP OCTAL ONLY
        04000, // 08	3COMP OCTAL ONLY
        04000, // 09	3COMP OCTAL ONLY
        00, // 10	1COMP OCTAL ONLY
        024400, // 11	3COMP HMS (DEC ONLY)
        02000, // 12	2COMP OCTAL ONLY
        024400, // 13	3COMP HMS (DEC ONLY)
        00, // 14	SPARE
        00, // 15	1COMP OCTAL ONLY
        024400, // 16	3COMP HMS (DEC ONLY)
        04102, // 17 	3COMP CDU DEG
        04102, // 18	3COMP CDU DEG
        04102, // 19	3COMP CDU DEG
        04102, // 20	3COMP CDU DEGREES
        04140, // 21	3COMP WHOLE
        04102, // 22	3COMP CDU DEGREES
        00, // 23 	SPARE
        024400, // 24	3COMP HMS (DEC ONLY)
        04140, // 25	3COMP WHOLE
        04000, // 26	3COMP OCTAL ONLY
        0140, // 27	1COMP WHOLE
        00, // 28	SPARE
        020102, // 29	1COMP CDU DEG (DEC ONLY)
        04140, // 30	3COMP WHOLE
        024400, // 31	3COMP HMS (DEC ONLY)
        024400, // 32 	3COMP HMS (DEC ONLY)
        024400, // 33 	3COMP HMS (DEC ONLY)
        024400, // 34	3COMP HMS (DEC ONLY)
        024400, // 35 	3COMP HMS (DEC ONLY)
        024400, // 36 	3COMP HMS (DEC ONLY)
        024400, // 37 	3COMP HMS (DEC ONLY)
        024400, // 38 	3COMP HMS (DEC ONLY)
        024400, // 39	3COMP HMS (DEC ONLY)
        024500, // 40	3COMP	MIN/SEC, VEL3, VEL3
        0542, // 41	2COMP	CDU DEG, ELEV DEG
        024410, // 42	3COMP	POS4, POS4, VEL3
        020204, // 43	3COMP	DPDEG(360), DPDEG(360), POS4
        0410, // 44	3COMP	POS4, POS4, MIN/SEC
        010000, // 45	3COMP	2INT, MIN/SEC, DPDEG(360)
        00, // 46	2COMP 	OCTAL ONLY FOR EACH
        0306, // 47	2COMP	WEIGHT2 FOR EACH
        0614, // 48	2COMP	TRIM DEG, TRIM DEG
        0510, // 49	3COMP	POS4, VEL3, WHOLE
        0417, // 50	3COMP	POS6, POS4, MIN/SEC
        0204, // 51	2COMP	DPDEG(360), DPDEG(360)
        04, // 52	1COMP	DPDEG(360)
        010507, // 53	3COMP	POS5, VEL3, DPDEG(360)
        010507, // 54	3COMP	POS5, VEL3, DPDEG(360)
        010200, // 55	3COMP	WHOLE, DPDEG(360), DPDEG(360)
        0444, // 56	2COMP	DPDEG(360), VEL2
        010, // 57	1COMP	POS4
        024510, // 58	3COMP	POS4, VEL3, VEL3
        024512, // 59	3COMP	VEL3 FOR EACH
        010440, // 60	3COMP	WHOLE, VEL2, DPDEG(360)
        0204, // 61	3COMP 	DPDEG(360), DPDEG(360), WHOLE
        020451, // 62	3COMP	VEL2, VEL2, POS4
        0457, // 63	3COMP	POS6, VEL2, MIN/SEC
        036460, // 64	3COMP	DRAG ACCEL, VEL2, POS6
        00, // 65	3COMP	HMS (DEC ONLY)
        037044, // 66	3COMP 	DPDEG(360), POS8, POS6
        010217, // 67	3COMP	POS6, DPDEG(360), DPDEG(360)
        034444, // 68	3COMP	DPDEG(360), VEL2, VEL/2VS
        035004, // 69	3COMP	DPDEG(360), DRAG ACCEL,VEL/2VS
        00, // 70	3COMP	OCTAL ONLY FOR EACH
        00, // 71	3COMP	OCTAL ONLY FOR EACH
        0404, // 72	3COMP	DPDEG(360), POS4, WHOLE
        010450, // 73	3COMP	POS4, VEL2, DPDEG(360)
        040444, // 74	3COMP	DPDEG(360), VEL2, DRAG ACCEL
        010, // 75	3COMP	POS4, MIN/SEC, MIN/SEC
        00, // 76		SPARE
        00, // 77		SPARE
        00, // 78		SPARE
        00, // 79		SPARE
        022440, // 80	3COMP	MIN/SEC, VEL2, VEL2
        024512, // 81	3COMP	VEL3 FOR EACH
        024512, // 82	3COMP	VEL3 FOR EACH
        024512, // 83	3COMP	VEL3 FOR EACH
        024512, // 84	3COMP	VEL3 FOR EACH
        024512, // 85	3COMP	VEL3 FOR EACH
        022451, // 86	3COMP	VEL2 FOR EACH
        0102, // 87	2COMP	CDU DEG, Y OPTICS DEG
        00, // 88	3COMP	FRAC FOR EACH
        016143, // 89	3COMP 	DPDEG(90), DPDEG(90), POS5
        010507, // 90	3COMP	POS5, VEL3, DEPDEG(360)
        0102, // 91	2COMP	CDUDEG, YOPTICS DEG
        0102, // 92	2COMP	CDUDEG, YOPTICS DEG
        06143, // 93	3COMP	DPDEG(90) FOR EACH
        0102, // 94	2COMP	CDUDEG, YOPTICS DEG
        04102, // 95	3COMP	CDU DEG FOR EACH
        04102, // 96	3COMP	CDU DEG FOR EACH
        00, // 97	3COMP	WHOLE FOR EACH
        00, // 98	3COMP	WHOLE, FRAC, WHOLE
        01162, // 99	3COMP	POS9, VEL4, WHOLE
    };

    /** SFINTAB: DP scale factors for input (2 words per entry). */
    public static final int[] SFINTAB = {
        06, // WHOLE, DP TIME (SEC)
        03240,
        00, // SPARE
        00,
        00, // CDU DEGREES, Y OPTICS DEGREES
        00, // (SFCONS IN DEGINSF, OPTDEGIN)
        010707, // DP DEGREES (90)
        03435, // UPPED BY 1
        013070, // DP DEGREES (360)(POINT BETWN BITS 11-12)
        034345, // UPPED BY 1
        05, // DEGREES (180)
        021616,
        026113, // WEIGHT2
        031713,
        070, // POSITION5
        020460,
        01065, // POSITION4
        05740,
        011414, // VELOCITY2 	(POINT BETWN BITS 11-12)
        031463,
        07475, // VELOCITY3
        016051,
        01, // ELEVATION DEGREES
        03434,
        02, // TRIM DEGREES
        022245,
        014, // INERTIA, THRUST MOMENT
        035607,
        07606, // VELOCITY/2VS
        06300,
        016631, // POSITION 6
        011307,
        012000, // DRAG ACCELERATION (POINT BETWN BITS 7-8)
        00,
        027176, // POSITION 8
        014235,
        01670, 020000, // 2DEC 30480 B-19  POSITION 9 (word value approximated as x*2^(28-19))
        07475, 016051, // 2DEC 30.48 B-7  VELOCITY4 (approximated as x*2^(28-7))
    };

    /** SFOUTAB: DP scale factors for output (2 words per entry). */
    public static final int[] SFOUTAB = {
        05174, // WHOLE, DP TIME (SEC)
        013261,
        00, // SPARE
        00,
        00, // CDU DEGREES, Y OPTICS DEGREES
        00, // (SFCONS IN DEGOUTSF, OPTDEGOUT)
        0714, // DP DEGREES (90) (POINT BETWN BITS 7-8)
        031463,
        013412, // DP DEGREES (360)
        07534,
        05605, // DEGREES (180)
        03656,
        01, // WEIGHT2
        016170,
        0441, // POSITION5
        034306,
        07176, // POSITION4
        021603,
        015340, // VELOCITY2
        015340,
        01031, // VELOCITY3	(POINT BETWN BITS 7-8)
        021032,
        034631, // ELEVATION DEGREES
        023146,
        014340, // TRIM DEGREES
        024145,
        02363, // INERTIA, THRUST MOMENT
        03721,
        020373, // VELOCITY/ZVS
        02122,
        0424, // POSITION 6	(POINT BETWN BITS 7-8)
        030446,
        0631, // DRAG ACCELERATION
        023146,
        0260, // POSITION 8
        06213,
        01031, 021032, // 2DEC .032808399 VELOCITY4
        04231, 027400, // 2DEC 17.2010499 B-7  POSITION 9 (approximated)
        01031, 021032, // 2DEC .032808399  VELOCITY4
    };

    /** IDADDTAB: three display addresses per mixed noun (40-99). */
    public static final Ad[] IDADDTAB = {
        ecadr("TTOGO"), // 40	MIN/SEC			M/S
        ecadr("VGDISP"), // 40	VEL3			DP3
        ecadr("DVTOTAL"), // 40	VEL3			DP3
        ecadr("DSPTEM1"), // 41	CDU DEG			CDU
        ecadr("DSPTEM1 +1"), // 41	ELEV DEG		ARTH
        oct(0), // 41	SPARE COMPONENT
        ecadr("HAPO"), // 42	POS4			DP3
        ecadr("HPER"), // 42	POS4			DP3
        ecadr("VGDISP"), // 42 	VEL3			DP3
        ecadr("LAT"), // 43	DPDEG(360)		DP4
        ecadr("LONG"), // 43	DPDEG(360)		DP4
        ecadr("ALT"), // 43	POS4			DP3
        ecadr("HAPOX"), // 44	POS4			DP3
        ecadr("HPERX"), // 44	POS4			DP3
        ecadr("TFF"), // 44	MIN/SEC			M/S
        ecadr("VHFCNT"), // 45	2INT			2INT
        ecadr("TTOGO"), // 45	MIN/SEC			M/S
        ecadr("+MGA"), // 45	DPDEG(360)		DP4
        ecadr("DAPDATR1"), // 46	OCTAL ONLY		OCT
        ecadr("DAPDATR2"), // 46	OCATAL ONLY		OCT
        oct(0), // 46	SPARE COMPONENT
        ecadr("CSMMASS"), // 47	WEIGHT2			ARTH1
        ecadr("LEMMASS"), // 47	WEIGHT2			ARTH1
        oct(0), // 47	SPARE COMPONENT
        ecadr("PACTOFF"), // 48	TRIM DEG		ARTH
        ecadr("YACTOFF"), // 48	TRIM DEG		ARTH
        oct(0), // 48	SPARE COMPONENT
        ecadr("N49DISP"), // 49	POS4			DP3
        ecadr("N49DISP +2"), // 49	VEL3			DP3
        ecadr("N49DISP +4"), // 49	WHOLE			ARTH
        ecadr("RSP-RREC"), // 50	POS6			DP3
        ecadr("HPERX"), // 50 	POS4			DP3
        ecadr("TFF"), // 50	MIN/SEC			M/S
        ecadr("RHOSB"), // 51	DPDEG(360)
        ecadr("GAMMASB"), // 51	DPDEG(360)		DP4
        oct(0), // 51	SPARE COMPONENT
        ecadr("ACTCENT"), // 52	DPDEG(360)		DP4
        oct(0), // 52	SPARE COMPONENT
        oct(0), // 52 	SPARE COMPONENT
        ecadr("RANGE"), // 53	POS5			DP1
        ecadr("RRATE"), // 53 	VEL3			DP3
        ecadr("RTHETA"), // 53 	DPDEG(360)		DP4
        ecadr("RANGE"), // 54	POS5			DP1
        ecadr("RRATE"), // 54	VEL3			DP3
        ecadr("RTHETA"), // 54	DPDEG(360)		DP4
        ecadr("NN1"), // 55	WHOLE			ARTH
        ecadr("ELEV"), // 55	DPDEG(360)		DP4
        ecadr("CENTANG"), // 55	DPDEG(360)		DP4
        ecadr("RTEGAM2D"), // 56	DPDEG(360)		DP4
        ecadr("RTEDVD"), // 56	VEL2			DP4
        oct(0), // 56	SPARE COMPONENT
        ecadr("DELTAR"), // 57	POS4			DP3
        oct(0), // 57	SPARE COMPONENT
        oct(0), // 57	SPARE COMPONENT
        ecadr("POSTTPI"), // 58	POS4			DP3
        ecadr("DELVTPI"), // 58	VEL3			DP3
        ecadr("DELVTPF"), // 58	VEL3			DP3
        ecadr("DVLOS"), // 59	VEL3			DP3
        ecadr("DVLOS +2"), // 59	VEL3			DP3
        ecadr("DVLOS +4"), // 59	VEL3			DP3
        ecadr("GMAX"), // 60	WHOLE			ARTH
        ecadr("VPRED"), // 60	VEL2			DP4
        ecadr("GAMMAEI"), // 60	DPDEG(360)		DP4
        ecadr("LAT(SPL)"), // 61	DPDEG(360)		DP4
        ecadr("LNG(SPL)"), // 61	DPDEG(360)		DP4
        ecadr("HEADSUP"), // 61	WHOLE			ARTH
        ecadr("VMAGI"), // 62	VEL2			DP4
        ecadr("HDOT"), // 62	VEL2			DP4
        ecadr("ALTI"), // 62	POS4			DP3
        ecadr("RTGO"), // 63	POS6			DP3
        ecadr("VIO"), // 63	VEL2			DP4
        ecadr("TTE"), // 63	MIN/SEC			M/S
        ecadr("D"), // 64 	DRAG ACCEL		DP2
        ecadr("VMAGI"), // 64	VEL2			DP4
        ecadr("RTGON64"), // 64	POS6			DP3
        ecadr("SAMPTIME"), // 65	HMS (MIXED ONLY TO KEEP CODE 65) HMS
        ecadr("SAMPTIME"), // 65	HMS			HMS
        ecadr("SAMPTIME"), // 65	HMS			HMS
        ecadr("ROLLC"), // 66	DPDEG(360)		DP4
        ecadr("XRNGERR"), // 66	POS8			DP3
        ecadr("DNRNGERR"), // 66	POS6			DP3
        ecadr("RTGON67"), // 67	POS6			DP3
        ecadr("LAT"), // 67	DPDEG(360)		DP4
        ecadr("LONG"), // 67	DPDEG(360)		DP4
        ecadr("ROLLC"), // 68	DPDEG(360)		DP4
        ecadr("VMAGI"), // 68	VEL2			DP4
        ecadr("RDOT"), // 68	VEL/2VS			DP4
        ecadr("ROLLC"), // 69	DPDEG(360)		DP4
        ecadr("Q7"), // 69	DRAG ACCEL		DP2
        ecadr("VL"), // 69	VEL/2VS			DP4
        ecadr("STARCODE"), // 70 	OCTAL ONLY		OCT
        ecadr("LANDMARK"), // 70	OCTAL ONLY		OCT
        ecadr("HORIZON"), // 70	OCTAL ONLY		OCT
        ecadr("STARCODE"), // 71	OCTAL ONLY		OCT
        ecadr("LANDMARK"), // 71	OCTAL ONLY		OCT
        ecadr("HORIZON"), // 71 	OCTAL ONLY		OCT
        ecadr("THETZERO"), // 72 	DPDEG(360)		DP4
        ecadr("DELHITE"), // 72	POS4			DP3
        ecadr("OPTION2"), // 72	WHOLE			ARTH
        ecadr("P21ALT"), // 73	POS4			DP3
        ecadr("P21VEL"), // 73	VEL2			DP4
        ecadr("P21GAM"), // 73	DPDEG(360)		DP4
        ecadr("ROLLC"), // 74	DPDEG(360)		DP4
        ecadr("VMAGI"), // 74	VEL2			DP4
        ecadr("D"), // 74	DRAG ACCEL		DP2
        ecadr("DIFFALT"), // 75	POS4			DP3
        ecadr("T1TOT2"), // 75	MIN/SEC			M/S
        ecadr("T2TOT3"), // 75	MIN/SEC			M/S
        oct(0), // 76	SPARE
        oct(0), // 76	SPARE
        oct(0), // 76	SPARE
        oct(0), // 77	SPARE
        oct(0), // 77	SPARE
        oct(0), // 77	SPARE
        oct(0), // 78	SPARE
        oct(0), // 78	SPARE
        oct(0), // 78	SPARE
        oct(0), // 79	SPARE
        oct(0), // 79	SPARE
        oct(0), // 79	SPARE
        ecadr("TTOGO"), // 80	MIN/SEC			M/S
        ecadr("VGDISP"), // 80	VEL2			DP4
        ecadr("DVTOTAL"), // 80	VEL2			DP4
        ecadr("DELVLVC"), // 81	VEL3			DP3
        ecadr("DELVLVC +2"), // 81	VEL3			DP3
        ecadr("DELVLVC +4"), // 81	VEL3			DP3
        ecadr("DELVLVC"), // 82	VEL3			DP3
        ecadr("DELVLVC +2"), // 82	VEL3			DP3
        ecadr("DELVLVC +4"), // 82 	VEL3			DP3
        ecadr("DELVIMU"), // 83	VEL3			DP3
        ecadr("DELVIMU +2"), // 83	VEL3			DP3
        ecadr("DELVIMU +4"), // 83	VEL3			DP3
        ecadr("DELVOV"), // 84	VEL3			DP3
        ecadr("DELVOV +2"), // 84	VEL3			DP3
        ecadr("DELVOV +4"), // 84	VEL3			DP3
        ecadr("VGBODY"), // 85	VEL3			DP3
        ecadr("VGBODY +2"), // 85	VEL3			DP3
        ecadr("VGBODY +4"), // 85	VEL3			DP3
        ecadr("DELVLVC"), // 86	VEL2			DP4
        ecadr("DELVLVC +2"), // 86	VEL2			DP4
        ecadr("DELVLVC +4"), // 86	VEL2			DP4
        ecadr("MRKBUF1 +3"), // 87	CDU DEG			CDU
        ecadr("MRKBUF1 +5"), // 87	Y OPTICS DEG		YOPT
        oct(0), // 87	SPARE COMPONENT
        ecadr("STARSAV3"), // 88	DPFRAC		      DPFRAC
        ecadr("STARSAV3 +2"), // 88	DPFRAC		      DPFRAC
        ecadr("STARSAV3 +4"), // 88	DPFRAC		      DPFRAC
        ecadr("LANDLAT"), // 89	DPDEG(90)		DP3
        ecadr("LANDLONG"), // 89	DPDEG(90)		DP3
        ecadr("LANDALT"), // 89	POS5			DP1
        ecadr("RANGE"), // 90	POS5			DP1
        ecadr("RRATE"), // 90	VEL3			DP3
        ecadr("RTHETA"), // 90	DPDEG(360)		DP4
        ecadr("CDUS"), // 91	CDU DEG			CDU
        ecadr("CDUT"), // 91	Y OPTICS DEG		YOPT
        oct(0), // 91	SPARE COMPONENT
        ecadr("SAC"), // 92	CDU DEG			CDU
        ecadr("PAC"), // 92	Y OPTICS DEG		YOPT
        oct(0), // 92	SPARE COMPONENT
        ecadr("OGC"), // 93	DPDEG(90)		DP3
        ecadr("OGC +2"), // 93	DPDEG(90)		DP3
        ecadr("OGC +4"), // 93	DPDEG(90)		DP3
        ecadr("MRKBUF1 +3"), // 94	CDU DEG			CDU
        ecadr("MRKBUF1 +5"), // 94 	Y OPTICS DEG		YOPT
        oct(0), // 94 	SPARE
        ecadr("PRAXIS"), // 95	CDU DEG			CDU
        ecadr("PRAXIS +1"), // 95	CDU DEG			CDU
        ecadr("PRAXIS +2"), // 95	CDU DEG			CDU
        ecadr("CPHIX"), // 96	CDU DEG			CDU
        ecadr("CPHIX +1"), // 96	CDU DEG			CDU
        ecadr("CPHIX +2"), // 96	CDU DEG			CDU
        ecadr("DSPTEM1"), // 97	WHOLE			ARTH
        ecadr("DSPTEM1 +1"), // 97	WHOLE			ARTH
        ecadr("DSPTEM1 +2"), // 97	WHOLE			ARTH
        ecadr("DSPTEM2"), // 98	WHOLE			ARTH
        ecadr("DSPTEM2 +1"), // 98	FRAC			FRAC
        ecadr("DSPTEM2 +2"), // 98	WHOLE			ARTH
        ecadr("WWPOS"), // 99 	POS9			DP3
        ecadr("WWVEL"), // 99	VEL4			DP2
        ecadr("WWOPT"), // 99 	WHOLE			ARTH
    };

    /** RUTMXTAB: SF routine codes for each component of the mixed nouns 40-99. */
    public static final int[] RUTMXTAB = {
        016351, // 40	M/S, DP3, DP3
        0142, // 41	CDU, ARTH
        016347, // 42	DP3, DP3, DP3
        016512, // 43	DP4, DP4, DP3
        022347, // 44	DP3, DP3, M/S
        024454, // 45	2INT, M/S, DP4
        00, // 46	OCT, OCT
        0553, // 47	ARITH1, ARITH1
        0143, // 48	ARTH, ARTH
        06347, // 49	DP3, DP3, ARTH
        022347, // 50	DP3, DP3, M/S
        0512, // 51	DP4, DP4
        012, // 52	DP4
        024344, // 53 	DP1, DP3, DP4
        024344, // 54	DP1, DP3, DP4
        024503, // 55	ARTH, DP4 , DP4
        0512, // 56	DP4, DP4
        07, // 57	DP3
        016347, // 58	DP3, DP3, DP3
        016347, // 59	DP3, DP3, DP3
        024503, // 60	ARTH, DP4, DP4
        06512, // 61	DP4, DP4, ARTH
        016512, // 62	DP4, DP4, DP3
        022507, // 63	DP3, DP4, M/S
        016505, // 64	DP2, DP4, DP3
        020410, // 65	HMS, HMS, HMS
        016352, // 66	DP4, DP3, DP3
        024507, // 67	DP3, DP4, DP4
        024512, // 68	DP4, DP4, DP4
        024252, // 69	DP4, DP2, DP4
        00, // 70	OCT, OCT, OCT
        00, // 71	OCT, OCT, OCT
        06352, // 72	DP4, DP3, ARTH
        024507, // 73	DPR, DP4, DP4
        012512, // 74	DP4, DP4, DP2
        022447, // 75	DP3, M/S, M/S
        00, // 76	SPARE
        00, // 77	SPARE
        00, // 78	SPARE
        00, // 79	SPARE
        024511, // 80	M/S, DP4, DP4
        016347, // 81	DP3, DP3, DP3
        016347, // 82	DP3, DP3, DP3
        016347, // 83	DP3, DP3, DP3
        016347, // 84	DP3, DP3, DP3
        016347, // 85	DP3, DP3, DP3
        024512, // 86	DP4, DP4, DP4
        0302, // 87	CDU, YOPT
        032655, // 88	DPFRAC FOR EACH
        010347, // 89	DP3, DP3, DP1
        024344, // 90	DP1, DP3, DP4
        0302, // 91	CDU, YOPT
        0302, // 92	CDU, YOPT
        016347, // 93	DP3, DP3, DP3
        0302, // 94	CDU, YOPT
        04102, // 95	CDU, CDU, CDU
        04102, // 96	CDU, CDU, CDU
        06143, // 97	ARTH, ARTH, ARTH
        06043, // 98	ARTH, FRAC, ARTH
        06247, // 99	DP3, DP2, ARTH
    };
}
