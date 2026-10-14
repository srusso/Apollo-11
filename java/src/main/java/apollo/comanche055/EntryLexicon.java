package apollo.comanche055;

import apollo.agc.Agc;

/**
 * ENTRY_LEXICON.agc (Comanche 055, pp. 837-843).
 * Pure documentation of the entry programs: variable lexicon (with maximum scale values and computer names),
 * extra erasable locations, switches, constants and gains, conversion factors, display quantities and body
 * attitude quantities. Only the numeric constants and gains are reproduced here.
 *
 * <p>Selected variable-to-computer-name mapping: URT0=RTINIT, UZ=UNITW, V=VEL, R=RN, VI=VN, RTE=RTEAST,
 * UTR=RTNORM, URT=RT, G=GDT/2, COSG=COSG/2, F1=FX+5, F2=FX+4, F3=FX, KA=KAT, PREDANGL=PREDANG,
 * RTOGO=FX+2, T=TIME2,TIME1, THETA=THETAH, VSQ=VSQUARE, WT=WIE(DTEAROT).
 *
 * <p>Extra erasables: GOTOADDR, XPIPBUF, YPIPBUF, ZPIPBUF, PIPCTR, JJ, MM, GRAD, FX (F3), FX+1 AREF,
 * FX+2 RTOGO, FX+3 RDOTREF, FX+4 F2, FX+5 F1, TEM1B, TIME/RTO, DTEAROT, UNITV, UNITR, -VREL.
 *
 * <p>Switches (CM/FLAGS = STATE+6): ENTRYDSP (92D, bit13, init 1), GONEPAST (95D, bit10), RELVELSW (96D, bit9),
 * EGSW (97D, bit8), FIRSTPAS (98D, bit7), HIND (99D, bit6), INRLSW (100D, bit5), LATSW (101D, bit4, init 1),
 * .05GSW (102D, bit3), GONEBY (112D, bit8, self-initializing).
 */
public final class EntryLexicon {
    private final Agc agc;

    public EntryLexicon(Agc agc) {
        this.agc = agc;
    }

    // ---- Constants and gains (page 841) ----
    public static final double C1 = 1.25;          // factor in ALP computation
    public static final double C16 = 0.01;         // CONSTD gain on drag
    public static final double C17 = 0.001;        // CONSTD gain on RDOT
    public static final double C18 = 500;          // bias vel. for final phase start, FPS
    public static final double C20 = 175;          // max drag for down-lift, FPSS
    public static final double CHOOK = 0.25;       // factor in AHOOK computation
    public static final double CH1 = 1.0;          // factor in GAMMAL computation
    public static final double COS15 = 0.965;      // cos(15 deg)
    public static final double DLEWD0 = -0.05;     // initial variation in LEWD
    public static final double D2 = 175;           // drag to change LEWD, FPSS
    public static final double DT = 2;             // computation cycle interval, sec
    public static final double GMAX = 257.6;       // max acceleration, FPSS (8 g)
    public static final double KA1 = 1.3;          // GS
    public static final double KA2 = 0.2;          // GS
    public static final double KA3 = 90;           // FPSS
    public static final double KA4 = 40;           // FPSS
    public static final double KB1 = 3.4;          // optimized upcontrol gain
    public static final double KB2 = 0.0034;       // optimized upcontrol gain
    public static final double KDMIN = 0.5;        // increment on Q7 to detect end of Kepler phase, FPSS
    public static final double KTETA = 1000;       // time of flight constant
    public static final double KLAT1 = 1.0 / 24;   // factor in KLAT calc
    public static final double K44 = 19749550;     // gain in initial roll section, FPS
    public static final double LATBIAS = 0.41252961; // lateral switch bias term, NM
    public static final double LEWD1 = 0.15;       // nominal upcontrol L/D
    public static final double POINT1 = 0.1;       // factor to reduce upcontrol gain
    public static final double Q2 = -1002;         // final phase range - 23500 Q3, NM
    public static final double Q3 = 0.07;          // final phase drange/dV, NM/FPS
    public static final double Q5 = 7050;          // final phase drange/dGamma, NM/RAD
    public static final double Q6 = 0.0349;        // final phase initial flight path angle, rad
    public static final double Q7F = 6;            // min drag for upcontrol, FPSS
    public static final double Q7MIN = 40;         // min Q7 in FACTOR calculation, FPSS
    public static final double Q19 = 0.5;          // factor in GAMMAL1 calculation
    public static final double Q21 = 1000;         // factor in Q2 calculation, NM
    public static final double Q22 = -1302;        // factor in Q2 calculation, NM
    public static final double VFINAL1 = 27000;    // vel. to start final phase on initial entry, FPS
    public static final double VFINAL = 26600;     // factor in initial up-down calc, FPS
    public static final double VLMIN = 18000;      // minimum VL, FPS
    public static final double VMIN_FRACTION_OF_VSAT = 0.5; // VMIN = VSAT/2
    public static final double VRCONTRL = 700;     // RDOT to start into HUNTEST, FPS
    public static final double TOLERANCE_25NM = 25; // tolerance to stop range iteration, NM
    public static final double VQUIT = 1000;       // velocity to stop steering, FPS

    // ---- Conversion factors and scaling constants (page 842) ----
    public static final double ATK = 3437.7468;    // NM/RAD
    public static final double G5 = 32.2;          // FPSS
    public static final double H5 = 28500;         // atmosphere scale height, FT
    public static final double J = 0.00162346;     // gravity harmonic coefficient
    public static final double KWE = 1546.70168;   // equatorial earth rate, FPS
    public static final double MUE = 3.986032233e14; // m^3/s^2
    public static final double RE = 21202900;      // earth radius, FT
    public static final double REQ = 20925738.2;   // earth equatorial radius, FT
    public static final double VSAT = 25766.1973;  // satellite velocity at RE, FPS
    public static final double WIE = 0.0000729211505; // earth rate, rad/sec
}
