package apollo.luminary099;

/**
 * Fixed-fixed constant pool (Luminary 099, BLOCK 02/03 $$/FCONS). OCT/DEC words are int constants
 * (15-bit one's-complement, negative values already converted); fractional DEC values are doubles.
 * Some constants must be adjacent in memory (DPOSMAX/POSMAX, NEG0/ZERO); that ordering is only documentary here.
 * Aliases defined with "=" are expressed as references to the primary constant where resolvable.
 */
public final class FixedFixedConstantPool {
    private FixedFixedConstantPool() { }

    public static final int DPOSMAX = 037777; // OCT 37777 MUST PRECEDE POSMAX
    public static final int POSMAX = 037777; // OCT 37777
    public static final int NEG1_2 = 057777; // OCT -20000 USED BY SIN ROUTINE (MUST BE TWO
    public static final int BIT15 = 040000; // OCT 40000
    public static final int BIT14 = 020000; // OCT 20000
    public static final int BIT13 = 010000; // OCT 10000
    public static final int BIT12 = 04000; // OCT 04000
    public static final int BIT11 = 02000; // OCT 02000
    public static final int BIT10 = 01000; // OCT 01000
    public static final int BIT9 = 0400; // OCT 00400
    public static final int BIT8 = 0200; // OCT 00200
    public static final int BIT7 = 0100; // OCT 00100
    public static final int BIT6 = 040; // OCT 00040
    public static final int BIT5 = 020; // OCT 00020
    public static final int BIT4 = 010; // OCT 00010
    public static final int BIT3 = 04; // OCT 00004
    public static final int BIT2 = 02; // OCT 00002
    public static final int BIT1 = 01; // OCT 00001
    public static final int NEG0 = 077777; // OCT -0 MUST PRECEDE ZERO
    public static final int ZERO = 00; // OCT 0 MUST FOLLOW NEG0
    public static final int FIVE = 05; // OCT 5
    public static final int SEVEN = 07; // OCT 7
    public static final int ELEVEN = 11; // DEC 11
    public static final int OCT15 = 015; // OCT 15
    public static final int LOW4 = 017; // OCT 17
    public static final int BITS4_AND_5 = 030; // OCT 30
    public static final int OCT33 = 033; // OCT 33
    public static final int OCT35 = 035; // OCT 35
    public static final int CALLCODE = 032; // OCT 00032
    public static final int TBUILDFX = 37; // DEC 37 BUILDUP FOR CONVIENCE IN DAPTESTING
    public static final int TDECAYFX = 38; // DEC 38 CONVENIENCE FOR DAPTESTING
    public static final int OCT50 = 050; // OCT 50
    public static final int DEC45 = 45; // DEC 45
    public static final int SUPER011 = 060; // OCT 60 BITS FOR SUPERBNK SETTING 011.
    public static final int P_5SEC = 50; // DEC 50
    public static final int SUPER101 = 0120; // OCT 120 BITS FOR SUPERBNK SETTING 101
    public static final int SUPER110 = 0140; // OCT 140 BITS FOR SUPERBNK SETTING 110.
    public static final int C1SEC = 100; // DEC 100
    public static final int C2SECS = 200; // DEC 200
    public static final int GN_CCODE = 0401; // OCT 00401 SET S/C CONTROL SWITCH TO G/N
    public static final int C3SECS = 300; // DEC 300
    public static final int C4SECS = 400; // DEC 400
    public static final int LOW9 = 0777; // OCT 777
    public static final double C5_2 = .0363551; // DEC .0363551 (OCTAL 01124)
    public static final String V05N09 = "0509"; // VN (SAME AS OCTAL 1211)
    public static final int OCT1400 = 01400; // OCT 01400
    public static final String V06N22 = "0622"; // VN
    public static final int BITS2M_10 = 01776; // OCT 1776
    public static final int LOW10 = 01777; // OCT 1777
    public static final int LOW7_PLUS_2K = 02177; // OCT 2177 OP CODE MASK + BANK 1 FBANK SETTING.
    public static final int EBANK5 = 02400; // OCT 02400
    public static final int PRIO3 = 03000; // OCT 03000
    public static final int EBANK7 = 03400; // OCT 03400
    public static final int PRIO5 = 05000; // OCT 05000
    public static final int PRIO6 = 06000; // OCT 06000
    public static final int PRIO7 = 07000; // OCT 07000
    public static final int PRIO11 = 011000; // OCT 11000
    public static final int PRIO13 = 013000; // OCT 13000
    public static final int PRIO14 = 014000; // OCT 14000
    public static final int PRIO15 = 015000; // OCT 15000
    public static final int PRIO16 = 016000; // OCT 16000
    public static final int PRIO17 = 017000; // OCT 17000
    public static final int OCT17770 = 017770; // OCT 17770
    public static final int PRIO21 = 021000; // OCT 21000
    public static final int PRIO22 = 022000; // OCT 22000 SERVICE ROUTINES
    public static final int PRIO23 = 023000; // OCT 23000
    public static final int PRIO24 = 024000; // OCT 24000
    public static final int PRIO25 = 025000; // OCT 25000
    public static final int PRIO26 = 026000; // OCT 26000
    public static final int PRIO27 = 027000; // OCT 27000
    public static final int PRIO31 = 031000; // OCT 31000
    public static final double C1_2 = .7853134; // DEC .7853134 (OCTAL 31103)
    public static final int PRIO32 = 032000; // OCT 32000
    public static final int PRIO33 = 033000; // OCT 33000
    public static final int PRIO34 = 034000; // OCT 34000
    public static final int PRIO35 = 035000; // OCT 35000
    public static final int PRIO36 = 036000; // OCT 36000
    public static final int PRIO37 = 037000; // OCT 37000
    public static final int C63_64_PLUS_1 = 037401; // OCT 37401
    public static final int OCT37766 = 037766; // OCT 37766
    public static final int OCT37774 = 037774; // OCT 37774
    public static final int OCT37776 = 037776; // OCT 37776
    public static final int DLOADCOD = 040014; // OCT 40014
    public static final int DLOAD_STAR = 040015; // OCT 40015
    public static final int BIT15_PLUS_6 = 040040; // OCT 40040
    public static final int OCT40200 = 040200; // OCT 40200
    public static final int M_BIT14 = 057777; // OCT 57777
    public static final double C3_2 = -.3216147; // DEC -.3216147 (OCTAL 65552)
    public static final int C13_14_15 = 070000; // OCT 70000
    public static final int M_1_8 = 073777; // OCT 73777
    public static final int HIGH4 = 074000; // OCT 74000
    public static final int M_ENDERAS = -2001; // DEC -2001 (OCTAL 74056)
    public static final int HIGH9 = 077700; // OCT 77700
    public static final int NEG3 = -3; // DEC -3
    public static final int NEG2 = 077775; // OCT 77775
    public static final int NEGONE = -1; // DEC -1

    // Aliases (EQUALS)
    public static final int LIMITS = NEG1_2; // = NEG1/2
    public static final int DEC27 = OCT33; // = OCT33
    public static final int DEC29 = OCT35; // = OCT35
    public static final int SUPER100 = BIT7; // = BIT7 BITS FOR SUPERBNK SETTING 100
    public static final int NEG1 = NEGONE; // = NEGONE
    public static final int ONE = BIT1; // = BIT1
    public static final int TWO = BIT2; // = BIT2
    public static final int FOUR = BIT3; // = BIT3
    public static final int LOW3 = SEVEN; // = SEVEN
    public static final int EIGHT = BIT4; // = BIT4
    public static final int NOUTCON = ELEVEN; // = ELEVEN
    public static final int PRIO1 = BIT10; // = BIT10
    public static final int EBANK3 = OCT1400; // = OCT1400
    public static final int PRIO2 = BIT11; // = BIT11
    public static final int OCT120 = SUPER101; // = SUPER101
    public static final int OCT140 = SUPER110; // = SUPER110
    public static final int C2K = BIT11; // = BIT11
    public static final int EBANK4 = BIT11; // = BIT11
    public static final int PRIO4 = BIT12; // = BIT12
    public static final int EBANK6 = PRIO3; // = PRIO3
    public static final int QUARTER = BIT13; // = BIT13
    public static final int PRIO10 = BIT13; // = BIT13
    public static final int PRIO20 = BIT14; // = BIT14
    public static final int HALF = BIT14; // = BIT14
    public static final int B12T14 = PRIO34; // = PRIO34
    public static final int NEGMAX = BIT15; // = BIT15
    public static final int VLOADCOD = BIT15; // = BIT15
    public static final int MINUS1 = NEG1; // = NEG1
    public static final int POS1_2 = HALF; // = HALF
    // OCT11 = R2D1 (defined elsewhere / not resolvable here) P20S
    // THREE = OCTAL3 (defined elsewhere / not resolvable here)
    // LOW2 = THREE (defined elsewhere / not resolvable here)
    // SIX = REVCNT (defined elsewhere / not resolvable here)
    // NINE = R2D1 (defined elsewhere / not resolvable here)
    // TEN = BINCON (defined elsewhere / not resolvable here)
    // OCT23 = VD1 (defined elsewhere / not resolvable here)
    // OCT25 = MD1 (defined elsewhere / not resolvable here)
    // OCT10001 = CCSL (defined elsewhere / not resolvable here)
    // PRIO30 = CHRPRIO (defined elsewhere / not resolvable here)
    // BIT13-14 = PRIO30 (defined elsewhere / not resolvable here) INTERPRETER USES IN PROCESSING STORECODE
    // VLOAD* = OCT40001 (defined elsewhere / not resolvable here)
    // OCT60000 = RELTAB11 (defined elsewhere / not resolvable here)
    // BANKMASK = HI5 (defined elsewhere / not resolvable here)
}
