package apollo.luminary099;

import apollo.agc.Agc;

/**
 * Pinball noun tables (PINBALL_NOUN_TABLES.agc): noun address, type, scale-factor, mixed-noun
 * index and routine tables, plus the table-reading routines LODNNTAB, GTSFOUT and GTSFIN.
 *
 * <p>Table entries are 15-bit words. ECADR entries are resolved through
 * {@link ErasableAssignments#address(String)} (the erasable-memory CADR; bank bits are not
 * modelled). 2DEC entries are expanded into their two words, high word first.
 *
 * <pre>
 * Component code: 00000 = 1 comp, 00001 = 2 comp, 00010 = 3 comp, X1XXX = decimal only,
 *                 1XXXX = no load.
 * SF routine codes: 0 octal only, 1 straight fractional, 2 CDU degrees, 3 arithmetic SF,
 *   4 ARITH DP1, 5 ARITH DP2, 6 landing radar position, 7 ARITH DP3, 8 hours/min/sec,
 *   9 min/sec (MMBSS), 10 ARITH DP4, 11 ARITH1 SF, 12 two integers, 13 360-CDU degrees.
 * </pre>
 *
 * <p>Noun numbers 00-39 are normal nouns; 40-99 are mixed nouns (MIXCON = 40).
 */
public final class PinballNounTables {
    private final Agc agc;

    public PinballNounTables(Agc agc) {
        this.agc = agc;
    }

    private static int ecadr(String expr) {
        return ErasableAssignments.address(expr) & Agc.WORD_MASK;
    }

    /** First mixed noun number (OCT50 = decimal 40). */
    public static final int MIXCON = 040;

    /** Noun address table: normal nouns 00-39 then mixed nouns 40-99 (component code and IDADDTAB index). (NNADTAB) */
    public static final int[] NNADTAB = {
        000000,  // 00 	NOT IN USE
        040000,  // 01 	SPECIFY MACHINE ADDRESS (FRACTIONAL)
        040000,  // 02 	SPECIFY MACHINE ADDRESS (WHOLE)
        040000,  // 03	SPECIFY MACHINE ADDRESS (DEGREES)
        ecadr("DSPTEM1"),  // 04	ANGULAR ERROR/DIFFERENCE
        ecadr("DSPTEM1"),  // 05 	ANGULAR ERROR/DIFFERENCE
        ecadr("OPTION1"),  // 06	OPTION CODE
        ecadr("XREG"),  // 07	ECADR OF WORD TO BE MODIFIED
        ecadr("ALMCADR"),  // 08 	ALARM DATA
        ecadr("FAILREG"),  // 09	ALARM CODES
        077776,  // 10	CHANNEL TO BE SPECIFIED
        ecadr("TCSI"),  // 11	TIG OF CSI (HRS,MIN,SEC)
        ecadr("OPTIONX"),  // 12	OPTION CODE
        ecadr("TCDH"),  // 13	TIG OF CDH (HRS,MIN,SEC)
        ecadr("DSPTEMX"),  // 14	CHECKLIST
        077777,  // 15	INCREMENT MACHINE ADDRESS
        ecadr("DSPTEMX"),  // 16	TIME OF EVENT (HRS,MIN,SEC)
        000000,  // 17	SPARE
        ecadr("FDAIX"),  // 18	AUTO MANEUVER BALL ANGLES
        000000,  // 19	SPARE
        ecadr("CDUX"),  // 20	ICDU ANGLES
        ecadr("PIPAX"),  // 21	PIPAS
        ecadr("THETAD"),  // 22	NEW ICDU ANGLES
        000000,  // 23	SPARE
        ecadr("DSPTEM2 +1"),  // 24	DELTA TIME FOR AGC CLOCK (HRS,MIN,SEC)
        ecadr("DSPTEM1"),  // 25	CHECKLIST
        ecadr("DSPTEM1"),  // 26	PRIO/DELAY, ADRES, BBCON
        ecadr("SMODE"),  // 27	SELF TEST ON/OFF SWITCH
        000000,  // 28	SPARE
        000000,  // 29	SPARE
        0,  // 30	SPARE
        0,  // 31	SPARE
        ecadr("-TPER"),  // 32	TIME TO PERIGEE (HRS,MIN,SEC)
        ecadr("TIG"),  // 33	TIME OF IGNITION (HRS,MIN,SEC)
        ecadr("DSPTEM1"),  // 34	TIME OF EVENT (HRS,MIN,SEC)
        ecadr("TTOGO"),  // 35	TIME TO GO TO EVENT (HRS,MIN,SEC)
        ecadr("TIME2"),  // 36	TIME OF AGC CLOCK (HRS,MIN,SEC)
        ecadr("TTPI"),  // 37	TIG OF TPI (HRS,MIN,SEC)
        ecadr("TET"),  // 38	TIME OF STATE BEING INTEGRATED
        000000,  // 39	SPARE
        064000,  // 40	TIME TO IGNITION/CUTOFF
        002003,  // 41	TARGET	AZIMUTH
        024006,  // 42	APOGEE
        024011,  // 43	LATITUDE
        064014,  // 44	APOGEE
        064017,  // 45	MARKS
        000022,  // 46	AUTOPILOT CONFIGURATION
        022025,  // 47	LEM WEIGHT
        022030,  // 48	GIMBAL PITCH TRIM
        024033,  // 49	DELTA R
        0,  // 50	SPARE
        022041,  // 51	S-BAND ANTENNA	PITCH
        000044,  // 52	CENTRAL ANGLE OF ACTIVE VEHICLE
        000000,  // 53	SPARE
        024052,  // 54	RANGE
        024055,  // 55	NO. OF APSIDAL CROSSINGS
        002060,  // 56	RR LOS	AZIMUTH
        020063,  // 57	DELTA R
        024066,  // 58	PERIGEE ALT
        024071,  // 59	DELTA VELOCITY LOS
        024074,  // 60	HORIZONTAL VELOCITY
        064077,  // 61	TIME TO GO IN BRAKING PHASE
        064102,  // 62	ABSOLUTE VALUE OF VELOCITY
        024105,  // 63	ABSOLUTE VALUE OF VELOCITY
        064110,  // 64	TIME LEFT FOR REDESIGNATION -- LPD ANGLE
        024113,  // 65	SAMPLED AGC TIME (HRS,MIN,SEC)
        062116,  // 66	LR	RANGE
        004121,  // 67	LRVX
        064124,  // 68	SLANT RANGE TO LANDING SIGHT
        000000,  // 69	SPARE
        004132,  // 70	AOT DETENT CODE/STAR CODE
        004135,  // 71	AOT DETENT CODE/STAR CODE
        002140,  // 72	RR	360 -- TRUNNION ANGLE
        002143,  // 73	NEW RR	360 -- TRUNNION ANGLE
        064146,  // 74	TIME TO IGNITION
        064151,  // 75	DELTA ALTITUDE CDH
        024154,  // 76	DESIRED HORIZONTAL VELOCITY
        062157,  // 77	TIME TO ENGINE CUTOFF
        002162,  // 78	RR	RANGE
        024165,  // 79	CURSOR ANGLE
        002170,  // 80	DATA INDICATOR
        024173,  // 81	DELTA V (LV)
        024176,  // 82	DELTA V (LV)
        024201,  // 83	DELTA V (BODY)
        024204,  // 84	DELTA V (OTHER VEHICLE)
        024207,  // 85	VG (BODY)
        024212,  // 86	VG (LV)
        002215,  // 87	BACKUP OPTICS LOS	AZIMUTH
        024220,  // 88	HALF UNIT SUN OR PLANET VECTOR
        024223,  // 89	LANDMARK	LATITUDE
        024226,  // 90	Y
        004231,  // 91	ALTITUDE
        000000,  // 92	SPARE
        004237,  // 93	DELTA GYRO ANGLES
        000000,  // 94	SPARE
        0,  // 95	SPARE
        0,  // 96	SPARE
        004253,  // 97	SYSTEM TEST INPUTS
        004256,  // 98	SYSTEM TEST RESULTS
        024261,  // 99	RMS IN POSITION
    };

    /** Noun type table: component code and scale-factor routine/constant numbers. (NNTYPTAB) */
    public static final int[] NNTYPTAB = {
        000000,  // 00	NOT IN USE
        004040,  // 01	3COMP FRACTIONAL
        004140,  // 02	3COMP WHOLE
        004102,  // 03	3COMP CDU DEGREES
        000504,  // 04	1COMP DPDEG(360)
        000504,  // 05	1COMP DPDEG(360)
        004000,  // 06	3COMP OCTAL ONLY
        004000,  // 07 	3COMP OCTAL ONLY
        004000,  // 08	3COMP OCTAL ONLY
        004000,  // 09	3COMP OCTAL ONLY
        000000,  // 10	1COMP OCTAL ONLY
        024400,  // 11	3COMP HMS (DEC ONLY)
        002000,  // 12	2COMP OCTAL ONLY
        024400,  // 13	3COMP HMS (DEC ONLY)
        004140,  // 14	3COMP WHOLE
        000000,  // 15	1COMP OCTAL ONLY
        024400,  // 16	3COMP HMS (DEC ONLY)
        0,  // 17 	SPARE
        004102,  // 18	3COMP CDU DEG
        000000,  // 19	SPARE
        004102,  // 20	3COMP CDU DEGREES
        004140,  // 21	3COMP WHOLE
        004102,  // 22	3COMP CDU DEGREES
        000000,  // 23 	SPARE
        024400,  // 24	3COMP HMS (DEC ONLY)
        004140,  // 25	3COMP WHOLE
        004000,  // 26	3COMP OCTAL ONLY
        000140,  // 27	1COMP WHILE
        000000,  // 28	SPARE
        000000,  // 29	SPARE
        0,  // 30	SPARE
        0,  // 31	SPARE
        024400,  // 32 	3COMP HMS (DEC ONLY)
        024400,  // 33 	3COMP HMS (DEC ONLY)
        024400,  // 34	3COMP HMS (DEC ONLY)
        024400,  // 35 	3COMP HMS (DEC ONLY)
        024400,  // 36 	3COMP HMS (DEC ONLY)
        024400,  // 37 	3COMP HMS (DEC ONLY)
        024400,  // 38 	3COMP HMS (DEC ONLY)
        000000,  // 39	SPARE
        024500,  // 40	3COMP	MIN/SEC, VEL3, VEL3
        000542,  // 41	2COMP	CDU DEG, ELEV DEG
        024410,  // 42	3COMP	POS4, POS4, VEL3
        020204,  // 43	3COMP	DPDEG(360), DPDEG(360) POS4
        000410,  // 44	3COMP	POS4, POS4, MIN/SEC
        010000,  // 45	3COMP	WHOLE, MIN/SEC, DPDEG(360)
        000000,  // 46	1COMP 	OCTAL ONLY
        000306,  // 47	2COMP	WEIGHT2 FOR EACH
        001367,  // 48	2COMP	TRIM DEG2 FOR EACH
        000510,  // 49	3COMP	POS4, VEL3, WHOLE
        0,  // 50	SPARE
        000204,  // 51	2COMP	DPDEG(360), DPDEG(360)
        000004,  // 52	1COMP	DPDEG(360)
        000000,  // 53	SPARE
        010507,  // 54	3COMP	POS5, VEL3, DPDEG(360)
        010200,  // 55	3COMP	WHOLE, DPDEG(360), DPDEG(360)
        000204,  // 56	2COMP	DPDEG(360), DPDEG(360)
        000010,  // 57	1COMP	POS4
        024510,  // 58	3COMP	POS4, VEL3, VEL3
        024512,  // 59	3COMP	VEL3 FOR EACH
        060512,  // 60	3COMP	VEL3, VEL3, COMP ALT
        054000,  // 61	3COMP 	MIN/SEC, MIN/SEC, POS7
        024012,  // 62	3COMP	VEL3, MIN/SEC, VEL3
        060512,  // 63	3COMP	VEL3, VEL3, COMP ALT
        060500,  // 64	3COMP	2INT, VEL3, COMP ALT
        000000,  // 65	3COMP	HMS (DEC ONLY)
        000016,  // 66	2COMP 	LANDING RADAR ALT, POSITION
        053223,  // 67	3COMP	LANDING RADAR VELX, Y, Z
        060026,  // 68	3COMP	POS7, MIN/SEC, COMP ALT
        000000,  // 69	SPARE
        0,  // 70	3COMP	OCTAL ONLY FOR EACH
        0,  // 71	3COMP	OCTAL ONLY FOR EACH
        000102,  // 72	2COMP	360-CDU DEG, CDU DEG
        000102,  // 73	2COMP	360-CDU DEG, CDU DEG
        010200,  // 74	3COMP	MIN/SEC, DPDEG(360), DPDEG(360)
        000010,  // 75	3COMP	POS4, MIN/SEC, MIN/SEC
        020512,  // 76	3COMP	VEL3, VEL3, POS4
        000500,  // 77	2COMP	MIN/SEC, VEL3
        000654,  // 78	2 COMP	RR RANGE, RR RANGE RATE
        000102,  // 79	3COMP	CDU DEG, CDU DEG, WHOLE
        000200,  // 80	2COMP	WHOLE, DPDEG(360)
        024512,  // 81	3COMP	VEL3 FOR EACH
        024512,  // 82	3COMP	VEL3 FOR EACH
        024512,  // 83	3COMP	VEL3 FOR EACH
        024512,  // 84	3COMP	VEL3 FOR EACH
        024512,  // 85	3COMP	VEL3 FOR EACH
        024512,  // 86	3COMP	VEL3 FOR EACH
        000102,  // 87	2COMP	CDU DEG FOR EACH
        0,  // 88	3COMP	FRAC FOR EACH
        016143,  // 89	3COMP 	DPDEG(90), DPDEG(90), POS5
        010507,  // 90	3COMP	POS5, VEL3, DEPDEG(360)
        010450,  // 91	3COMP	POS4, VEL2, DPDEG(360)
        000000,  // 92	SPARE
        006143,  // 93	3COMP	DPDEG(90) FOR EACH
        000000,  // 94	SPARE
        0,  // 95	SPARE
        0,  // 96	SPARE
        000000,  // 97	3COMP	WHOLE FOR EACH
        000000,  // 98	3COMP	WHOLE, FRAC, WHOLE
        071572,  // 99	3COMP	POS9, VEL4, RADIANS
    };

    /** Scale factor constants for input (double precision pairs). (SFINTAB) */
    public static final int[] SFINTAB = {
        000006,  // WHOLE, DP TIME (SEC)
        003240,
        000000,  // SPARE
        000000,
        000000,  // CDU DEGREES, 360-CDU DEGREES
        000000,  // (SFCONS IN DEGINSF)
        010707,  // DP DEGREES (90)
        003435,  // UPPED BY 1
        013070,  // DP DEGREES (360) (POINT BETWN BITS 11-12)
        034345,  // UPPED BY 1
        000005,  // DEGREES (180)
        021616,
        026113,  // WEIGHT2
        031713,
        000070,  // POSITION5
        020460,
        001065,  // POSITION4
        005740,
        011414,  // VELOCITY2 	(POINT BETWN BITS 11-12)
        031463,
        007475,  // VELOCITY3
        016051,
        000001,  // ELEVATION DEGREES
        003434,
        000047,  // RENDEZVOUS RADAR RANGE
        021135,
        077766,  // RENDESVOUS RADAR RANGE RATE
        050711,
        05,  // LANDING RADAR ALTITUDE (2DEC .9267840599 E5 B-28*, high word)
        025006,  // low word
        000002,  // INITIAL/FINAL ALTITUDE
        023224,
        000014,  // ALTITUDE RATE
        006500,
        000012,  // FORWARD/LATERAL VELOCITY
        036455,
        004256,  // ROT HAND CONT ANGLE RATE
        007071,
        077766,  // LANDING RADAR VELX (2DEC -1.552795030 E5 B-28*, high word)
        060557,  // low word
        05,  // LANDING RADAR VELY (2DEC .8250825087 E5 B-28*, high word)
        01114,  // low word
        07,  // LANDING RADAR VELZ (2DEC 1.153668673 E5 B-28*, high word)
        01247,  // low word
        004324,  // POSITION7
        027600,
        000036,  // TRIM DEGREES2
        020440,
        000035,  // COMPUTED ALTITUDE
        030400,
        023420,  // DP DEGREES
        000000,
        01670,  // POSITION 9 (2DEC 30480 B-19, high word)
        020000,  // low word
        07475,  // VELOCITY4 (2DEC 30.48 B-7, high word)
        016051,  // low word
        014400,  // RADIANS (2DEC 100 B-8, high word)
        00,  // low word
    };

    /** Scale factor constants for output (double precision pairs). (SFOUTAB) */
    public static final int[] SFOUTAB = {
        005174,  // WHOLE, DP TIME (SEC)
        013261,
        000000,  // SPARE
        000000,
        000000,  // CDU DEGREES, 360-CDU DEGREES
        000000,  // (SFCONS IN DEGOUTSF, 360 CDUO)
        000714,  // DP DEGREES (90) (POINT BETWN BITS 7-8)
        031463,
        013412,  // DP DEGREES (360)
        007534,
        005605,  // DEGREES (180)
        003656,
        000001,  // WEIGHT2
        016170,
        000441,  // POSITION5
        034306,
        007176,  // POSITION4	(POINT BETWN BITS 7-8)
        021603,
        015340,  // VELOCITY2
        015340,
        001031,  // VELOCITY3	(POINT BETWN BITS 7-8)
        021032,
        034631,  // ELEVATION DETREES
        023146,
        000636,  // RENDEZVOUS RADAR RANGE
        014552,
        074552,  // RENDEZVOUS RADAR RANGE RATE
        070307,
        05520,  // LANDING RADAR ALTITUDE (2DEC 1.079 E-5 B14, high word)
        015312,  // low word
        014226,  // INITIAL/FINAL ALTITUDE
        031757,
        002476,  // ALTITUDE RATE
        005531,
        002727,  // FORWARD/LATERAL VELOCTY
        016415,
        000007,  // ROT HAND CONT ANGLE RATE
        013734,
        074477,  // LANDING RADAR VELX (2DEC -.6440 E-5 B14, high word)
        050643,  // low word
        06265,  // LANDING RADAR VELY (2DEC 1.212 E-5 B14, high word)
        016004,  // low word
        04426,  // LANDING RADAR VELZ (2DEC .8668 E-5 B14, high word)
        031433,  // low word
        034772,  // POSITION7
        007016,
        001030,  // TRIM DEGREES2
        033675,
        001046,  // COMPUTED ALTITUDE
        015700,
        000321,  // DP DEGREES
        026706,
        04231,  // POSITION 9 (2DEC 17.2010499 B-7, high word)
        027400,  // low word
        01031,  // VELOCITY4 (2DEC .032808399, high word)
        021032,  // low word
        012172,  // RADIANS (2DEC .32, high word)
        034122,  // low word
    };

    /** Mixed-noun component address table (three entries per mixed noun). (IDADDTAB) */
    public static final int[] IDADDTAB = {
        ecadr("TTOGO"),  // 40	MIN/SEC			M/S
        ecadr("VGDISP"),  // 40	VEL3			DP3
        ecadr("DVTOTAL"),  // 40	VEL3			DP3
        ecadr("DSPTEM1"),  // 41	CDU DEG			CDU
        ecadr("DSPTEM1 +1"),  // 41	ELEV DEG		ARTH
        0,  // 41	SPARE COMPONENT
        ecadr("HAPO"),  // 42	POS4			DP3
        ecadr("HPER"),  // 42	POS4			DP3
        ecadr("VGDISP"),  // 42 	VEL3			DP3
        ecadr("LAT"),  // 43	DPDEG(360)		DP4
        ecadr("LONG"),  // 43	DPDEG(360		DP4
        ecadr("ALT"),  // 43	POS4			DP3
        ecadr("HAPOX"),  // 44	POS4			DP3
        ecadr("HPERX"),  // 44	POS4			DP3
        ecadr("TFF"),  // 44	MIN/SEC			M/S
        ecadr("TRKMKCNT"),  // 45	WHOLE			ARTH
        ecadr("TTOGO"),  // 45	MIN/SEC			M/S
        ecadr("+MGA"),  // 45	DPDEG(360)		DP4
        ecadr("DAPDATR1"),  // 46	OCTAL ONLY		OCT
        0,  // 46	SPARE COMPONENT
        0,  // 46	SPARE COMPONENT
        ecadr("LEMMASS"),  // 47	WEIGHT2			ARTH1
        ecadr("CSMMASS"),  // 47	WEIGHT2			ARTH1
        0,  // 47	SPARE COMPONENT
        ecadr("PITTIME"),  // 48	TRIM DEG2		ARTH
        ecadr("ROLLTIME"),  // 48	TRIM DEG2		ARTH
        0,  // 48	SPARE COMPONENT
        ecadr("R22DISP"),  // 49	POS4			DP3
        ecadr("R22DISP +2"),  // 49	VEL3			DP3
        ecadr("WHCHREAD"),  // 49	WHOLE			ARTH
        0,  // 50	SPARE
        0,  // 50 	SPARE
        0,  // 50	SPARE
        ecadr("ALPHASB"),  // 51	DPDEG(360)		DP4
        ecadr("BETASB"),  // 51	DPDEG(360)		DP4
        0,  // 51	SPARE COMPONENT
        ecadr("ACTCENT"),  // 52	DPDEG(360)		DP4
        000000,  // 52	SPARE COMPONENT
        000000,  // 52 	SPARE COMPONENT
        000000,  // 53	SPARE
        000000,  // 53
        000000,  // 53
        ecadr("RANGE"),  // 54	POS5			DP1
        ecadr("RRATE"),  // 54	VEL3			DP3
        ecadr("RTHETA"),  // 54	DPDEG(360)		DP4
        ecadr("NN"),  // 55	WHOLE			ARTH
        ecadr("ELEV"),  // 55	DPDEG(360)		DP4
        ecadr("CENTANG"),  // 55	DPDEG(360)		DP4
        ecadr("RR-AZ"),  // 56	DPDEG(360)		DP4
        ecadr("RR-ELEV"),  // 56	DPDEG(360)		DP4
        0,  // 56	SPARE COMPONENT
        ecadr("DELTAR"),  // 57	POS4			DP3
        0,  // 57	SPARE COMPONENT
        0,  // 57	SPARE COMPONENT
        ecadr("POSTTPI"),  // 58	POS4			DP3
        ecadr("DELVTPI"),  // 58	VEL3			DP3
        ecadr("DELVTPF"),  // 58	VEL3			DP3
        ecadr("DVLOS"),  // 59	VEL3			DP3
        ecadr("DVLOS +2"),  // 59	VEL3			DP3
        ecadr("DVLOS +4"),  // 59	VEL3			DP3
        ecadr("VHORIZ"),  // 60	VEL3			DP3
        ecadr("HDOTDISP"),  // 60	VEL3			DP3
        ecadr("HCALC"),  // 60	COMP ALT		DP1
        ecadr("TTFDISP"),  // 61	MIN/SEC			M/S
        ecadr("TTOGO"),  // 61	MIN/SEC			M/S
        ecadr("OUTOFPLN"),  // 61	POS7			DP4
        ecadr("ABVEL"),  // 62	VEL3			DP3
        ecadr("TTOGO"),  // 62	MIN/SEC			M/S
        ecadr("DVTOTAL"),  // 62	VEL3			DP3
        ecadr("ABVEL"),  // 63	VEL3			DP3
        ecadr("HDOTDISP"),  // 63	VEL3			DP3
        ecadr("HCALC1"),  // 63	COMP ALT		DP1
        ecadr("FUNNYDSP"),  // 64 	2INT			2INT
        ecadr("HDOTDISP"),  // 64	VEL3			DP3
        ecadr("HCALC"),  // 64	COMP ALT		DP1
        ecadr("SAMPTIME"),  // 65	HMS (MIXED ONLY TO KEEP CODE 65) HMS
        ecadr("SAMPTIME"),  // 65	HMS			HMS
        ecadr("SAMPTIME"),  // 65	HMS			HMS
        ecadr("RSTACK +6"),  // 66	LANDING RADAR ALT	DP1
        0,  // 66	LR POSITION		LRPOS
        0,  // 66	SPARE COMPONENT
        ecadr("RSTACK"),  // 67	LANDING RADAR VELX	DP1
        ecadr("RSTACK +2"),  // 67	LANDING RADAR VELY	DP1
        ecadr("RSTACK +4"),  // 67	LANDING RADAR VELZ	DP1
        ecadr("RANGEDSP"),  // 68	POS7			DP4
        ecadr("TTFDISP"),  // 68	MIN/SEC			M/S
        ecadr("DELTAH"),  // 68	COMP ALT		DP1
        000000,  // 69	SPARE
        000000,  // 69
        000000,  // 69
        ecadr("AOTCODE"),  // 70	OCTAL ONLY		OCT
        ecadr("AOTCODE +1"),  // 70 	OCTAL ONLY		OCT
        ecadr("AOTCODE +2"),  // 70	OCTAL ONLY		OCT
        ecadr("AOTCODE"),  // 71	OCTAL ONLY		OCT
        ecadr("AOTCODE +1"),  // 71	OCTAL ONLY		OCT
        ecadr("AOTCODE +2"),  // 71 	OCTAL ONLY		OCT
        ecadr("CDUT"),  // 72	360-CDU DEG		360-CDU
        ecadr("CDUS"),  // 72	CDU DEG			CDU
        0,  // 72	SPARE COMPONENT
        ecadr("TANG"),  // 73	360-CDU DEG		360-CDU
        ecadr("TANG +1"),  // 73	CDU DEG			CDU
        0,  // 73	SPARE COMPONENT
        ecadr("TTOGO"),  // 74	MIN/SEC			M/S
        ecadr("YAW"),  // 74	DPDEG(360)		DP4
        ecadr("PITCH"),  // 74	DPDEG(360)		DP4
        ecadr("DIFFALT"),  // 75	POS4			DP3
        ecadr("T1TOT2"),  // 75	MIN/SEC
        ecadr("T2TOT3"),  // 75	MIN/SEC			M/S
        ecadr("ZDOTD"),  // 76	VEL3			DP3
        ecadr("RDOTD"),  // 76	VEL3			DP3
        ecadr("XRANGE"),  // 76	POS4			DP3
        ecadr("TTOGO"),  // 77	MIN/SEC			M/S
        ecadr("YDOT"),  // 77 	VEL3			DP3
        0,  // 77	SPARE COMPONENT
        ecadr("RSTACK"),  // 78 	RR RANGE		DP1
        ecadr("RSTACK +2"),  // 78	RR RANGE RATE		DP1
        000000,  // 78	SPARE COMPONENT
        ecadr("CURSOR"),  // 79	CDU DEG			CDU
        ecadr("SPIRAL"),  // 79	CDU DEG			CDU
        ecadr("POSCODE"),  // 79	WHOLE			ARTH
        ecadr("DATAGOOD"),  // 80	WHOLE			ARTH
        ecadr("OMEGAD"),  // 80	DPDEG(360)		DP4
        0,  // 80	SPARE COMPONENT
        ecadr("DELVLVC"),  // 81	VEL3			DP3
        ecadr("DELVLVC +2"),  // 81	VEL3			DP3
        ecadr("DELVLVC +4"),  // 81	VEL3			DP3
        ecadr("DELVLVC"),  // 82	VEL3			DP3
        ecadr("DELVLVC +2"),  // 82	VEL3			DP3
        ecadr("DELVLVC +4"),  // 82	VEL3			DP3
        ecadr("DELVIMU"),  // 83	VEL3			DP3
        ecadr("DELVIMU +2"),  // 83	VEL3			DP3
        ecadr("DELVIMU +4"),  // 83	VEL3			DP3
        ecadr("DELVOV"),  // 84	VEL3			DP3
        ecadr("DELVOV +2"),  // 84	VEL3			DP3
        ecadr("DELVOV +4"),  // 84	VEL3			DP3
        ecadr("VGBODY"),  // 85	VEL3			DP3
        ecadr("VGBODY +2"),  // 85	VEL3			DP3
        ecadr("VGBODY +4"),  // 85	VEL3			DP3
        ecadr("DELVLVC"),  // 86	VEL3			DP3
        ecadr("DELVLVC +2"),  // 86	VEL3			DP3
        ecadr("DELVLVC +4"),  // 86	VEL3			DP3
        ecadr("AZ"),  // 87	CDU DEG			CDU
        ecadr("EL"),  // 87	CDU DEG			CDU
        0,  // 87	SPARE COMPONENT
        ecadr("STARAD"),  // 88	FRAC			FRAC
        ecadr("STARAD +2"),  // 88	FRAC			FRAC
        ecadr("STARAD +4"),  // 88	FRAC			FRAC
        ecadr("LANDLAT"),  // 89	DPDEG(90)		DP3
        ecadr("LANDLONG"),  // 89	DPDEG(90)		DP3
        ecadr("LANDALT"),  // 89	POS5			DP1
        ecadr("RANGE"),  // 90	POS5			DP1
        ecadr("RRATE"),  // 90	VEL3			DP3
        ecadr("RTHETA"),  // 90	DPDEG(360)		DP4
        ecadr("P21ALT"),  // 91	POS4			DP3
        ecadr("P21VEL"),  // 91 	VEL2			DP4
        ecadr("P21GAM"),  // 91	DPDEG(360)		DP4
        000000,  // 92	SPARE
        000000,  // 92
        000000,  // 92
        ecadr("OGC"),  // 93	DPDEG(90)		DP3
        ecadr("OGC +2"),  // 93	DPDEG(90)		DP3
        ecadr("OGC +4"),  // 93	DPDEG(90)		DP3
        000000,  // 94	SPARE
        000000,  // 94
        000000,  // 94
        0,  // 95	SPARE
        0,  // 95	SPARE
        0,  // 95	SPARE
        0,  // 96	SPARE
        0,  // 96	SPARE
        0,  // 96	SPARE
        ecadr("DSPTEM1"),  // 97	WHOLE			ARTH
        ecadr("DSPTEM1 +1"),  // 97	WHOLE			ARTH
        ecadr("DSPTEM1 +2"),  // 97	WHOLE			ARTH
        ecadr("DSPTEM2"),  // 98	WHOLE			ARTH
        ecadr("DSPTEM2 +1"),  // 98	FRAC			FRAC
        ecadr("DSPTEM2 +2"),  // 98	WHOLE			ARTH
        ecadr("WWPOS"),  // 99 	POS9			DP3
        ecadr("WWVEL"),  // 99	VEL4			DP2
        ecadr("WWBIAS"),  // 99 	RADIANS			DP4
    };

    /** Mixed-noun scale-factor routine table (indexed from noun 40). (RUTMXTAB) */
    public static final int[] RUTMXTAB = {
        016351,  // 40	M/S, DP3, DP3
        000142,  // 41	CDU, ARTH
        016347,  // 42	DP3, DP3, DP3
        016512,  // 43	DP4, DP4, DP3
        022347,  // 44	DP3, DP3, M/S
        024443,  // 45	ARTH, M/S, DP4
        000000,  // 46	OCT
        000553,  // 47	ARITH1, ARITH1
        000143,  // 48	ARTH, ARTH
        006347,  // 49	DP3, DP3, ARTH
        0,  // 50	SPARE
        000512,  // 51	DP4, DP4
        000012,  // 52	DP4
        000000,  // 53 	SPARE
        024344,  // 54	DP1, DP3, DP4
        024503,  // 55	ARTH, DP4, DP4
        000512,  // 56	DP4, DP4
        000007,  // 57	DP3
        016347,  // 58	DP3, DP3, DP3
        016347,  // 59	DP3, DP3, DP3
        010347,  // 60	DP3, DP3, DP1
        024451,  // 61	M/S, M/S, DP4
        016447,  // 62	DP3, M/S, DP3
        010347,  // 63	DP3, DP3, DP1
        010354,  // 64	2INT, DP3, DP1
        020410,  // 65	HMS, HMS, HMS
        000304,  // 66	DP1, LRPOS
        010204,  // 67	DP1, DP1, DP1
        010452,  // 68	DP4, M/S, DP1
        000000,  // 69	SPARE
        0,  // 70	OCT, OCT, OCT
        0,  // 71	OCT, OCT, OCT
        000115,  // 72	360-CDU, CDU
        000115,  // 73	360-CDU, CDU
        024511,  // 74	M/S, DP4, DP4
        022447,  // 75	DP3, M/S, M/S
        016347,  // 76	DP3, DP3, DP3
        000351,  // 77	M/S, DP3
        000204,  // 78	DP1, DP1
        006102,  // 79	CDU, CDU, ARTH
        000503,  // 80	ARTH, DP4
        016347,  // 81	DP3, DP3, DP3
        016347,  // 82	DP3, DP3, DP3
        016347,  // 83	DP3, DP3, DP3
        016347,  // 84	DP3, DP3, DP3
        016347,  // 85	DP3, DP3, DP3
        016347,  // 86	DP3, DP3, DP3
        000102,  // 87	CDU, CDU
        002041,  // 88	FRAC FOR EACH
        010347,  // 89	DP3, DP3, DP1
        024344,  // 90	DP1, DP3, DP4
        024507,  // 91	DP3, DP4, DP4
        000000,  // 92	SPARE
        016347,  // 93	DP3, DP3, DP3
        000000,  // 94	SPARE
        0,  // 95	SPARE
        0,  // 96	SPARE
        006143,  // 97	ARTH, ARTH, ARTH
        006043,  // 98	ARTH, FRAC, ARTH
        024247,  // 99	DP3, DP2, DP4
    };

    // ---- table-reading routines (executed in the table bank) ----

    /**
     * LODNNTAB: loads NNADTEM and NNTYPTEM for the noun in NOUNREG; for mixed nouns also
     * IDAD1TEM, IDAD2TEM, IDAD3TEM and RUTMXTEM. Sets MIXBR to 1 (normal) or 2 (mixed).
     */
    public void lodnntab() {
        int noun = agc.get("NOUNREG");
        agc.set("NNADTEM", NNADTAB[noun]);
        agc.set("NNTYPTEM", NNTYPTAB[noun]);
        if (noun - MIXCON < 0) {            // NOUN NUMBER L/ FIRST MIXED NOUN
            agc.set("MIXBR", 1);            // NORMAL
            return;                          // LODNLV
        }
        agc.set("MIXBR", 2);                 // LODMIXNN: MIXED
        agc.set("RUTMXTEM", RUTMXTAB[noun - MIXCON]);
        int index = agc.get("NNADTEM") & 01777;  // LOW10 mask
        agc.set("IDAD1TEM", IDADDTAB[index]);
        agc.set("IDAD2TEM", IDADDTAB[index + 1]);
        agc.set("IDAD3TEM", IDADDTAB[index + 2]);
    }

    /** GTSFOUT: loads SFTEMP1, SFTEMP2 with the DP SFOUTAB entry; 2*SFCONUM arrives in SFTEMP1. */
    public void gtsfout() {
        sfcom(SFOUTAB);
    }

    /** GTSFIN: loads SFTEMP1, SFTEMP2 with the DP SFINTAB entry; 2*SFCONUM arrives in SFTEMP1. */
    public void gtsfin() {
        sfcom(SFINTAB);
    }

    /** SFCOM: common exit of GTSFOUT/GTSFIN. */
    private void sfcom(int[] table) {
        int index = agc.get("SFTEMP1");
        agc.set("SFTEMP1", table[index]);
        agc.set("SFTEMP2", table[index + 1]);
    }
}
