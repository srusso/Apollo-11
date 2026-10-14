package apollo.comanche055;

import apollo.agc.Agc;

/**
 * FIXED_FIXED_CONSTANT_POOL (Comanche 055, pages 1200-1204): constants kept in the
 * fixed-fixed memory (BLOCK 02/03). Octal values are 15-bit one's-complement words.
 * Floating point DEC entries are given as their numeric value (AGC scaling unspecified).
 * The original table is indexed positionally (DPOSMAX, POSMAX, LIMITS/NEG1/2, BIT15..BIT1,
 * NEG0, ZERO ...); the order is preserved below.
 */
public final class FixedFixedConstantPool {
    private final Agc agc;

    public FixedFixedConstantPool(Agc agc) {
        this.agc = agc;
    }

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
    public static final int ELEVEN = 11; // DEC 
    public static final int OCT15 = 015; // OCT 15 
    public static final int LOW4 = 017; // OCT 17 
    public static final int BITS4_5 = 030; // OCT 30 
    public static final int CALLCODE = 032; // OCT 00032 
    public static final int TBUILDFX = 37; // DEC BUILDUP FOR CONVIENCE IN DAPTESTING
    public static final int TDECAYFX = 38; // DEC CONVENIENCE FOR DAPTESTING
    public static final int OCT50 = 050; // OCT 50 
    public static final int DEC45 = 45; // DEC 
    public static final int SUPER011 = 060; // OCT 60 BITS FOR SUPERBNK SETTING 011.
    public static final int _5SEC = 50; // DEC 
    public static final int SUPER101 = 0120; // OCT 120 BITS FOR SUPERBNK SETTING 101
    public static final int SUPER110 = 0140; // OCT 140 BITS FOR SUPERBNK SETTING 110.
    public static final int _1SEC = 100; // DEC 
    public static final int _2SECS = 200; // DEC 
    public static final int GN_CCODE = 0401; // OCT 00401 SET S/C CONTROL SWITCH TO G/N
    public static final int _3SECS = 300; // DEC 
    public static final int _4SECS = 400; // DEC 
    public static final int LOW9 = 0777; // OCT 777 
    public static final double C5_2 = .0363551; // DEC (OCTAL 01124)
    public static final int V05N09 = 649; // VN verb/noun code (SAME AS OCTAL 1211)
    public static final int OCT1400 = 01400; // OCT 01400 
    public static final int V06N22 = 790; // VN verb/noun code 
    public static final int BITS2_10 = 01776; // OCT 1776 
    public static final int LOW10 = 01777; // OCT 1777 
    public static final int LOW7_2K = 02177; // OCT 2177 OP CODE MASK + BANK 1 FBANK SETTING.
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
    public static final double C1_2 = .7853134; // DEC (OCTAL 31103)
    public static final int PRIO32 = 032000; // OCT 32000 
    public static final int PRIO33 = 033000; // OCT 33000 
    public static final int PRIO34 = 034000; // OCT 34000 
    public static final int PRIO35 = 035000; // OCT 35000 
    public static final int PRIO36 = 036000; // OCT 36000 
    public static final int PRIO37 = 037000; // OCT 37000 
    public static final int _63_64_1 = 037401; // OCT 37401 
    public static final int OCT37766 = 037766; // OCT 37766 
    public static final int OCT37774 = 037774; // OCT 37774 
    public static final int OCT37776 = 037776; // OCT 37776 
    public static final int DLOADCOD = 040014; // OCT 40014 
    public static final int DLOAD_ = 040015; // OCT 40015 
    public static final int BIT15_6 = 040040; // OCT 40040 
    public static final int OCT40200 = 040200; // OCT 40200 
    public static final int _BIT14 = 057777; // OCT 57777 
    public static final double C3_2 = -.3216147; // DEC (OCTAL 65552)
    public static final int _13_14_15 = 070000; // OCT 70000 
    public static final int _1_8 = 073777; // OCT 73777 
    public static final int HIGH4 = 074000; // OCT 74000 
    public static final int _ENDERAS = -2001; // DEC (OCTAL 74056)
    public static final int HIGH9 = 077700; // OCT 77700 
    public static final int NEG3 = -3; // DEC 
    public static final int NEG2 = 077775; // OCT 77775 
    public static final int NEGONE = -1; // DEC 
}
