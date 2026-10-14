package apollo.luminary099;

/**
 * Controlled constants of Luminary 099 (CONTROLLED_CONSTANTS.agc).
 *
 * <p>Single-precision (DEC) constants are 15-bit one's-complement words held as Java ints in
 * the AGC word format ({@link apollo.agc.Agc#fromSigned}); double-precision (2DEC) constants
 * are signed 28-bit values (fraction * 2^28, or an integer) held as longs. Each constant carries
 * the SETLOC/BANK it lived in as a comment. The assembler scale notation "B n" has been applied
 * at translation time (value * 10^E * 2^n).
 */
public final class ControlledConstants {
    private ControlledConstants() {}

    private static int w(int v) {
        return apollo.agc.Agc.fromSigned(v);
    }


    // SETLOC P40S
    // *** THE ORDER OF THE FOLLOWING SIX CONSTANTS MUST NOT BE CHANGED ***
    public static final long FDPS = 9158263L;  // 9817.5 LBS FORCE IN NEWTONS
    public static final long MDOTDPS = 4966056L;  // 32.62 LBS/SEC IN KGS/CS
    public static final long DTDECAY = -38L;
    public static final long FAPS = 3265056L;  // 3500 LBS FORCE IN NEWTONS
    public static final long MDOTAPS = 1723020L;  // 11.32 LBS/SEC IN KGS/CS
    public static final long ATDECAY = -10L;
    // ********************************************************************
    public static final long FRCS4 = 373125L;  // 400 LBS FORCE IN NEWTONS
    public static final long FRCS2 = 186563L;  // 200 LBS FORCE IN NEWTONS

    // SETLOC P40S1
    // *** APS IMPULSE DATA FOR P42 ***************************************
    public static final long K1VAL = 3986L;  // 2800 LB-SEC
    public static final long K2VAL = 498L;  // 700 LB-SEC
    public static final long K3VAL = 408132L;  // FAPS (3500 LBS THRUST)
    // ********************************************************************
    public static final long S40_136 = 244895L;  // .4671 M NEWTONS (DPS)
    public static final long S40_136_ = 250772403L;  // S40.136 SHIFTED LEFT 10.

    // SETLOC ASENT1
    public static final long _1_DV_A = 31876710L;  // 2 SECONDS WORTH OF INITIAL ASCENT
    // Page 39
    //  # STAGE ACCELERATION -- INVERTED (M/CS) 
    //  # 1) PREDICATED ON A LIFTOFF MASS OF 
    //  # 4869.9 KG (SNA-8-D-027 7/11/68) 
    //  # 2) PREDICATED ON A CONTRIBUTION TO VEH- 
    //  # ICLE ACCELERATION FROM RCS THRUSTERS 
    //  # EQUIV. TO 1 JET ON CONTINUOUSLY. 
    public static final long K_1_DV = 228956570L;  // DPS ENGINE THRUST IN NEWTONS / 100 CS.
    public static final long AT_A = 45194051L;  // INITIAL ASC. STG. ACCELERATION ** M/CS.
    //  # ASSUMPTIONS SAME AS FOR (1/DV)A. 
    public static final long TBUP_A = 188215296L;  // ESTIMATED BURN-UP TIME OF THE ASCENT STG.
    //  # ASSUMPTIONS SAME AS FOR (1/DV)A WITH THE 
    //  # ADDITIONAL ASSUMPTION THAT NET MASS-FLOW 
    //  # RATE = 5.299 KG/SEC = 5.135 (APS) + 
    //  # .164 (1 RCS JET). 

    // SETLOC ASENT
    public static final long AT_RCS = 21577916L;  // 4 JETS IN A DRY LEM

    // SETLOC SERVICES
    // *** THE ORDER OF THE FOLLOWING TWO CONSTANTS MUST NOT BE CHANGED *******
    public static final int APSVEX = w(-15514);  // 9942 FT/SEC IN M/CS.
    public static final int DPSVEX = w(-15134);  // VE (DPS) +2.95588868E+ 3
    // ************************************************************************

    // SETLOC F2DPS*31
    public static final long TRIMACCL = 2406094L;  // A (T) +3.50132708E- 1
    // Page 40
    // THROTTLING AND THRUST DETECTION PARAMETERS

    // SETLOC P40S
    public static final int THRESH1 = w(24);
    public static final int THRESH3 = w(12);
    /** HIRTHROT = BIT13 */
    public static final String HIRTHROT_ALIAS = "BIT13";

    // SETLOC FFTAG5
    public static final int THRESH2 = w(308);

    // SETLOC FTHROT
    public static final int FMAXODD = w(3841);  // FSAT 		+4.81454413 E+4
    public static final int FMAXPOS = w(3467);  // FMAX 		+4.34546769 E+4
    public static final int THROTLAG = w(20);  // TAU (TH)	+1.99999999 E-1
    public static final long SCALEFAC = 3268444L;  // BITPERF 	+7.97959872 E-2

    // SETLOC F2DPS*32
    public static final int DPSTHRSH = w(36);  // (THRESH1 + THRESH3 FOR P63)
    // Page 41
    // LM HARDWARE-RELATED PARAMETERS

    // SETLOC RADARUPT
    public static final int LVELBIAS = w(-12288);  // LANDING RADAR BIAS FOR 153.6 KC.
    public static final long RDOTBIAS = 17000L;  // BIAS COUNT FOR RR RANGE RATE.

    // SETLOC LRS22
    public static final long RDOTCONV = -65748541L;  // CONVERTS RR RDOT READING TO M/CS AT 2(7)
    public static final long RANGCONV = 95932926L;  // CONVERTS RR RANGE READING TO M. AT 2(-29

    // SETLOC SERVICES
    public static final long HBEAMANT = -125816183L;  // RANGE BEAM IN LR ANTENNA COORDINATES.
    // continuation of previous: 2DEC 0  ->  0
    // continuation of previous: 2DEC -.1741224271  ->  -46740633
    public static final long HSCAL = -88282838L;  // SCALES 1.079 FT/BIT TO 2(22)M.
    // ***** THE SEQUENCE OF THE FOLLOWING CONSTANTS MUST BE PRESERVED *********
    public static final long VZSCAL = 145245838L;  // SCALES .8668 FT/SEC/BIT TO 2(18) M/CS.
    public static final long VYSCAL = 203089473L;  // SCALES 1.212 FT/SEC/BIT TO 2(18) M/CS.
    public static final long VXSCAL = -107912228L;  // SCALES -.644 FT/SEC/BIT TO 2(18) M/CS.
    // *************************************************************************
    public static final int KPIP = w(839);  // SCALES DELV TO UNITS OF 2(5) M/CS.
    public static final long KPIP1 = 3435974L;  // SCALES DELV TO UNITS OF 2(7) M/CS.
    public static final long KPIP2 = 1717987L;  // SCALES DELV TO UNITS OF 2(8) M/CS.
    // Page 42
    public static final long ALTCONV = 23472648L;  // CONVERTS M*2(-24) TO BIT UNITS *2(-28).
    public static final long ARCONV1 = 172010499L;  // CONV. ALTRATE COMP. TO BIT UNITS<

    // SETLOC R10
    public static final int ARCONV = 024402;  // 656.1679798B-10 CONV ALTRATE TO BIT UNIT
    public static final int ARTOA = w(873);  // .25/2.345 B-1 4X/SEC CYCLE RATE.
    public static final int ARTOA2 = w(8943);  // (.5)/(2.345)(100)
    public static final int VELCONV = 022316;  // 588.914 B-10 CONV VEL. TO BIT UNITS.
    public static final int KPIP1_5 = w(839);  // SCALES DELV TO M/CS*2(-5).
    public static final int MAXVBITS = 000547;  // MAX. DISPLAYED VELOCITY 199.9989 FT/SEC.

    // SETLOC DAPS3
    public static final int TORKJET1 = w(616);  // 550 / .2 SCALED AT (+16) 64 / 180
    // Page 43
    // PARAMETERS RELATING TO MASS, INERTIA, AND VEHICLE DIMENSIONS

    // SETLOC FRANDRES
    public static final int FULLAPS = w(1262);  // NOMINAL FULL ASCENT MASS -- 2(16) KG.

    // SETLOC LOADDAP1
    public static final int MINLMD = w(-712);  // MIN. DESCENT STAGE MASS -- 2(16) KG.
    public static final int MINMINLM = w(-550);  // MIN ASCENT STAGE MASS -- 2(16) KG.
    /** MINCSM = BIT11 */
    public static final String MINCSM_ALIAS = "BIT11";  // MIN CSM MASS (OK FOR 1/ACCS) = 9050 LB

    // SETLOC DAPS3
    public static final int LOASCENT = w(550);  // MIN ASCENT LEM MASS -- 2(16) KG.
    public static final int HIDESCNT = w(3825);  // MAX DESCENT LEM MASS -- 2(16) KG.
    public static final int LODESCNT = w(438);  // MIN DESCENT STAGE (ALONE) -- 2(16) KG.
    // Page 44
    // PHYSICAL CONSTANTS ( TIME - INVARIANT )

    // SETLOC IMU2
    public static final long OMEG_MS = 65334634L;

    // SETLOC R30LOC
    // *** THE ORDER OF THE FOLLOWING TWO CONSTANTS MUST BE PRESERVED ***********
    public static final long _1_RTMUM = 198627193L;
    public static final long _1_RTMUE = 176229826L;
    // **************************************************************************

    // SETLOC P40S1
    public static final long EARTHMU = -155704375L;  // M(3)/CS(2)

    // SETLOC ASENT1
    public static final long MUM__37 = 957574L;
    public static final long MOONRATE = 3746009L;  // RAD/CS.

    // SETLOC SERVICES
    // *** THE ORDER OF THE FOLLOWING TWO CONSTANTS MUST BE PRESERVED ***********
    public static final long MUDT = -121644051L;
    public static final long MUDT1 = -1496209L;
    // **************************************************************************
    public static final long MUDTMUN = -95757383L;
    public static final long RESQ = 37887L;
    // Page 45
    public static final long _20J = 8715885L;
    public static final long _2J = 871588L;

    // SETLOC P50S1
    public static final long RSUBEM = 192201000L;
    public static final long RSUBM = 869045L;
    public static final long RSUBE = 3189083L;
    public static final long ROE = 690215L;

    // SETLOC CONICS1
    public static final long ERAD = 3186669L;  // PAD RADIUS
    public static final long _504RM = 869045L;  // METERS B-29 (EQUATORIAL MOON RADIUS)

    // SETLOC CONICS1
    // *** THE ORDER OF THE FOLLOWING CONSTANTS MUST BE PRESERVED **************
    public static final long MUTABLE = 155704375L;  // MUE
    // continuation of previous: 2DEC* .25087606 E-10 B+34*  ->  115696162  // 1/MUE
    // continuation of previous: 2DEC* 1.99650495 E5 B-18*  ->  204442107  // SQRT(MUE)
    // continuation of previous: 2DEC* .50087529 E-5 B+17*  ->  176229826  // 1/SQRT(MUE)
    // continuation of previous: 2DEC* 4.902778 E8 B-30*  ->  122569450  // MUM
    // continuation of previous: 2DEC* .203966 E-8 B+28*  ->  146972992  // 1/MUM
    // continuation of previous: 2DEC* 2.21422176 E4 B-15*  ->  181389047  // SQRT(MUM)
    // continuation of previous: 2DEC* .45162595 E-4 B+14*  ->  198627193  // 1/SQRT(MUM)
    // *************************************************************************
    // Page 46

    // SETLOC INTINIT
    public static final long OMEGMOON = 59936144L;

    // SETLOC ORBITAL2
    // *** THE ORDER OF THE FOLLOWING CONSTANTS MUST NOT BE CHANGED ************
    // continuation of previous: 2DEC* 1.32715445 E16 B-54*  ->  197761424
    public static final long MUM = 122569450L;
    public static final long MUEARTH = 155704375L;
    // continuation of previous: 2DEC 0  ->  0
    public static final long J4REQ_J3 = 19966430L;
    // continuation of previous: 2DEC -176236.02 B-25  ->  -1409888
    public static final long _2J3RE_J2 = -27109L;
    // continuation of previous: 2DEC* .3067493316 E18 B-60*  ->  71420644
    public static final long J2REQSQ = 99760848L;
    public static final long _3J22R2MU = 85726292L;
    // *************************************************************************

    // SETLOC TOF-FF1
    public static final long _1_RTMU = 176124180L;  // MODIFIED EARTH MU

    // SETLOC SBAND
    public static final long REMDIST = 192201000L;  // MEAN DISTANCE BETWEEN EARTH AND MOON.
    // Page 47
    // PHYSICAL CONSTANTS (TIME - VARIANT)

    // SETLOC STARTAB
    // continuation of previous: 2DEC +.8342971408 B-1  ->  111977467  // STAR 37	X
    // continuation of previous: 2DEC -.2392481515 B-1  ->  -32111343  // STAR 37	Y
    // continuation of previous: 2DEC -.4966976975 B-1  ->  -66665636  // STAR 37	Z
    // continuation of previous: 2DEC +.8139832631 B-1  ->  109250984  // STAR 36	X
    // continuation of previous: 2DEC -.5557243189 B-1  ->  -74588055  // STAR 36	Y
    // continuation of previous: 2DEC +.1691204557 B-1  ->  22698963  // STAR 36	Z
    // continuation of previous: 2DEC +.4541086270 B-1  ->  60949428  // STAR 35	X
    // continuation of previous: 2DEC -.5392368197 B-1  ->  -72375141  // STAR 35	Y
    // continuation of previous: 2DEC +.7092312789 B-1  ->  95191411  // STAR 35	Z
    // continuation of previous: 2DEC +.3201817378 B-1  ->  42974065  // STAR 34	X
    // continuation of previous: 2DEC -.4436021946 B-1  ->  -59539279  // STAR 34	Y
    // continuation of previous: 2DEC -.8370786986 B-1  ->  -112350801  // STAR 34	Z
    // continuation of previous: 2DEC +.5520184464 B-1  ->  74090662  // STAR 33	X
    // continuation of previous: 2DEC -.7933187400 B-1  ->  -106477439  // STAR 33 	Y
    // continuation of previous: 2DEC -.2567508745 B-1  ->  -34460519  // STAR 33	Z
    // continuation of previous: 2DEC +.4537196908 B-1  ->  60897226  // STAR 32	X
    // continuation of previous: 2DEC -.8779508801 B-1  ->  -117836572  // STAR 32	Y
    // continuation of previous: 2DEC +.1527766153 B-1  ->  20505330  // STAR 32	Z
    // continuation of previous: 2DEC +.2069525789 B-1  ->  27776705  // STAR 31	X
    // continuation of previous: 2DEC -.8719885748 B-1  ->  -117036325  // STAR 31	Y
    // continuation of previous: 2DEC -.4436288486 B-1  ->  -59542856  // STAR 31	Z
    // continuation of previous: 2DEC +.1217293692 B-1  ->  16338239  // STAR 30	X
    // continuation of previous: 2DEC -.7702732847 B-1  ->  -103384330  // STAR 30 	Y
    // Page 48
    // continuation of previous: 2DEC +.6259880410 B-1  ->  84018693  // STAR 30	Z
    // continuation of previous: 2DEC -.1124304773 B-1  ->  -15090163  // STAR 29	X
    // continuation of previous: 2DEC -.9694934200 B-1  ->  -130123204  // STAR 29	Y
    // continuation of previous: 2DEC +.2178116072 B-1  ->  29234179  // STAR 29	Z
    // continuation of previous: 2DEC -.1146237858 B-1  ->  -15384544  // STAR 28	X
    // continuation of previous: 2DEC -.3399692557 B-1  ->  -45629901  // STAR 28 	Y
    // continuation of previous: 2DEC -.9334250333 B-1  ->  -125282187  // STAR 28	Z
    // continuation of previous: 2DEC -.3516499609 B-1  ->  -47197659  // STAR 27	X
    // continuation of previous: 2DEC -.8240752703 B-1  ->  -110605510  // STAR 27	Y
    // continuation of previous: 2DEC -.4441196390 B-1  ->  -59608729  // STAR 27	Z
    // continuation of previous: 2DEC -.5326876930 B-1  ->  -71496132  // STAR 26	X
    // continuation of previous: 2DEC -.7160644554 B-1  ->  -96108544  // STAR 26	Y
    // continuation of previous: 2DEC +.4511047742 B-1  ->  60546258  // STAR 26	Z
    // continuation of previous: 2DEC -.7861763936 B-1  ->  -105518809  // STAR 25	X
    // continuation of previous: 2DEC -.5217996305 B-1  ->  -70034761  // STAR 25	Y
    // continuation of previous: 2DEC +.3311371675 B-1  ->  44444478  // STAR 25	Z
    // continuation of previous: 2DEC -.6898393233 B-1  ->  -92588667  // STAR 24	X
    // continuation of previous: 2DEC -.4182330640 B-1  ->  -56134292  // STAR 24	Y
    // continuation of previous: 2DEC -.5909338474 B-1  ->  -79313798  // STAR 24	Z
    // continuation of previous: 2DEC -.5812035376 B-1  ->  -78007818  // STAR 23	X
    // continuation of previous: 2DEC -.2909171294 B-1  ->  -39046236  // STAR 23	Y
    // continuation of previous: 2DEC +.7599800468 B-1  ->  102002795  // STAR 23 	Z
    // continuation of previous: 2DEC -.9170097662 B-1  ->  -123078967  // STAR 22	X
    // continuation of previous: 2DEC -.3502146628 B-1  ->  -47005016  // STAR 22	Y
    // continuation of previous: 2DEC -.1908999176 B-1  ->  -25622153  // STAR 22	Z
    // Page 49
    // continuation of previous: 2DEC -.4523440203 B-1  ->  -60712587  // STAR 21	X
    // continuation of previous: 2DEC -.0493710140 B-1  ->  -6626465  // STAR 21	Y
    // continuation of previous: 2DEC -.8904759346 B-1  ->  -119517657  // STAR 21	Z
    // continuation of previous: 2DEC -.9525211695 B-1  ->  -127845227  // STAR 20	X
    // continuation of previous: 2DEC -.0593434796 B-1  ->  -7964947  // STAR 20	Y
    // continuation of previous: 2DEC -.2986331746 B-1  ->  -40081866  // STAR 20	Z
    // continuation of previous: 2DEC -.9656605484 B-1  ->  -129608765  // STAR 19	X
    // continuation of previous: 2DEC +.0525933156 B-1  ->  7058955  // STAR 19	Y
    // continuation of previous: 2DEC +.2544280809 B-1  ->  34148759  // STAR 19	Z
    // continuation of previous: 2DEC -.8608205219 B-1  ->  -115537375  // STAR 18	X
    // continuation of previous: 2DEC +.4636213989 B-1  ->  62226211  // STAR 18	Y
    // continuation of previous: 2DEC +.2098647835 B-1  ->  28167574  // STAR 18	Z
    // continuation of previous: 2DEC -.7742591356 B-1  ->  -103919302  // STAR 17	X
    // continuation of previous: 2DEC +.6152504197 B-1  ->  82577513  // STAR 17	Y
    // continuation of previous: 2DEC -.1482892839 B-1  ->  -19903051  // STAR 17	Z
    // continuation of previous: 2DEC -.4657947941 B-1  ->  -62517919  // STAR 16	X
    // continuation of previous: 2DEC +.4774785033 B-1  ->  64086080  // STAR 16	Y
    // continuation of previous: 2DEC +.7450164351 B-1  ->  99994413  // STAR 16	Z
    // continuation of previous: 2DEC -.3612508532 B-1  ->  -48486269  // STAR 15	X
    // continuation of previous: 2DEC +.5747270840 B-1  ->  77138563  // STAR 15	Y
    // continuation of previous: 2DEC -.7342932655 B-1  ->  -98555174  // STAR 15	Z
    // continuation of previous: 2DEC -.4118589524 B-1  ->  -55278773  // STAR 14 	X
    // continuation of previous: 2DEC +.9065485360 B-1  ->  121674885  // STAR 14	Y
    // continuation of previous: 2DEC +.0924226975 B-1  ->  12404764  // STAR 14	Z
    // continuation of previous: 2DEC -.1820751783 B-1  ->  -24437717  // STAR 13	X
    // Page 50
    // continuation of previous: 2DEC +.9404899869 B-1  ->  126230429  // STAR 13	Y
    // continuation of previous: 2DEC -.2869271926 B-1  ->  -38510716  // STAR 13	Z
    // continuation of previous: 2DEC -.0614937230 B-1  ->  -8253548  // STAR 12 	X
    // continuation of previous: 2DEC +.6031563286 B-1  ->  80954272  // STAR 12	Y
    // continuation of previous: 2DEC -.7952489957 B-1  ->  -106736513  // STAR 12	Z
    // continuation of previous: 2DEC +.1371725575 B-1  ->  18410989  // STAR 11	X
    // continuation of previous: 2DEC +.6813721061 B-1  ->  91452216  // STAR 11	Y
    // continuation of previous: 2DEC +.7189685267 B-1  ->  96498322  // STAR 11	Z
    // continuation of previous: 2DEC +.2011399589 B-1  ->  26996548  // STAR 10	X
    // continuation of previous: 2DEC +.9690337941 B-1  ->  130061514  // STAR 10	Y
    // continuation of previous: 2DEC -.1432348512 B-1  ->  -19224656  // STAR 10	Z
    // continuation of previous: 2DEC +.3507315038 B-1  ->  47074386  // STAR 9	X
    // continuation of previous: 2DEC +.8926333307 B-1  ->  119807218  // STAR 9	Y
    // continuation of previous: 2DEC +.2831839492 B-1  ->  38008306  // STAR 9	Z
    // continuation of previous: 2DEC +.4105636020 B-1  ->  55104914  // STAR 8	X
    // continuation of previous: 2DEC +.4988110001 B-1  ->  66949279  // STAR 8	Y
    // continuation of previous: 2DEC +.7632988371 B-1  ->  102448236  // STAR 8	Z
    // continuation of previous: 2DEC +.7032235469 B-1  ->  94385067  // STAR 7	X
    // continuation of previous: 2DEC +.7075846047 B-1  ->  94970398  // STAR 7	Y
    // continuation of previous: 2DEC +.0692868685 B-1  ->  9299526  // STAR 7	Z
    // continuation of previous: 2DEC +.5450107404 B-1  ->  73150103  // STAR 6	X
    // continuation of previous: 2DEC +.5314955466 B-1  ->  71336125  // STAR 6	Y
    // continuation of previous: 2DEC -.6484410356 B-1  ->  -87032283  // STAR 6	Z
    // continuation of previous: 2DEC +.0130968840 B-1  ->  1757834  // STAR 5	X
    // continuation of previous: 2DEC +.0078062795 B-1  ->  1047741  // STAR 5	Y
    // Page 51
    // continuation of previous: 2DEC +.9998837600 B-1  ->  134202127  // STAR 5	Z
    // continuation of previous: 2DEC +.4917678276 B-1  ->  66003961  // STAR 4	X
    // continuation of previous: 2DEC +.2204887125 B-1  ->  29593494  // STAR 4	Y
    // continuation of previous: 2DEC -.8423473935 B-1  ->  -113057953  // STAR 4 	Z
    // continuation of previous: 2DEC +.4775639450 B-1  ->  64097548  // STAR 3	X
    // continuation of previous: 2DEC +.1166004340 B-1  ->  15649845  // STAR 3	Y
    // continuation of previous: 2DEC +.8708254803 B-1  ->  116880217  // STAR 3 	Z
    // continuation of previous: 2DEC +.9342640400 B-1  ->  125394797  // STAR 2	X
    // continuation of previous: 2DEC +.1735073142 B-1  ->  23287758  // STAR 2	Y
    // continuation of previous: 2DEC -.3115219339 B-1  ->  -41811766  // STAR 2	Z
    // continuation of previous: 2DEC +.8748658918 B-1  ->  117422512  // STAR 1	X
    // continuation of previous: 2DEC +.0260879174 B-1  ->  3501461  // STAR 1	Y
    // continuation of previous: 2DEC +.4836621670 B-1  ->  64916037  // STAR 1	Z
    public static final int CATLOG = w(6970);
    // *******************************************************************************

    // SETLOC EPHEM1
    public static final long KONMAT = 134217728L;  // ********************
    // continuation of previous: 2DEC 0  ->  0  // *
    // continuation of previous: 2DEC 0  ->  0  // *
    // continuation of previous: 2DEC 0  ->  0  // *
    // continuation of previous: 2DEC .91745 B-1  ->  123138055  // K1 COS(OBL)	     *
    // continuation of previous: 2DEC -.03571 B-1  ->  -4792915  // K2 SIN(OBL)SIN(IM) *
    // continuation of previous: 2DEC 0  ->  0  // *
    // continuation of previous: 2DEC .39784 B-1  ->  53397181  // K3 SIN(OBL)	     *
    // Page 52
    // continuation of previous: 2DEC .082354 B-1  ->  11053367  // K4 COS(OBL)SIN(IM) *
    public static final long CSTODAY = 270000L;  // * NOTE:          *
    public static final int RCB_13 = 000002;  // * TABLES CONTAIN *
    public static final int X___________ = 000000;  // * CONSTANTS FOR  *
    public static final long RATESP = 157200012L;  // LOMR		     * 1969 - 1970    *
    // continuation of previous: 2DEC .00273779 B+4  ->  11758719  // LOSR
    // continuation of previous: 2DEC -.00014719 B+4  ->  -632176  // LONR
    // continuation of previous: 2DEC .815282336  ->  218850686  // LOMO
    // continuation of previous: 2DEC .274674910  ->  73732485  // LOSO
    // continuation of previous: 2DEC .986209499  ->  264733597  // LONO
    public static final long VAL67 = 9269971L;  // AMOD
    // continuation of previous: 2DEC .530784445  ->  142481365  // AARG
    // continuation of previous: 2DEC .036291712 B+1  ->  19483965  // 1/27
    // continuation of previous: 2DEC .003505277 B+1  ->  1881881  // BMOD
    // continuation of previous: 2DEC .585365625  ->  157132888  // BARG
    // continuation of previous: 2DEC .03125 B+1  ->  16777216  // 1/32
    // continuation of previous: 2DEC .005325277 B+1  ->  2858986  // CMOD
    // continuation of previous: 2DEC -.01106341036  ->  -2969812  // CARG
    // continuation of previous: 2DEC .002737925 B+1  ->  1469912  // 1/365
    // ********************************************************************************

    // SETLOC PLANTIN2
    public static final long COSI = 134169642L;  // COS (5521.5 SEC.) B-1
    public static final long SINI = 3592444L;  // SIN (5521.5 SEC.) B-1
    public static final long NODDOT = -1227650L;  // REV/CSEC B+28 = -1.07047011 E-8 RAD/SEC
    public static final long FDOT = 153239957L;  // REV/CSEC B+27 =  2.67240410 E-6 RAD/SEC
    // Page 53
    public static final long BDOT = -8L;  // REV/CSEC B+28 = -7.19757301 E-14 RAD/SEC
    public static final long NODIO = 264733579L;  // REVS B-D	= 6.19653663041 RAD
    public static final long FSUBO = 222557296L;  // REVS B-D	= 5.20932947829 RAD
    public static final long BSUBO = 17480554L;  // REVS B-D	= 0.40916190299 RAD
    public static final long WEARTH = 261338451L;  // REV/CSEC B+23	= 7.29211494 E-5 RAD/SEC
}
