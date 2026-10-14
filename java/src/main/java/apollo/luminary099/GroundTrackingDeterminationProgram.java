package apollo.luminary099;

import apollo.agc.Agc;

/**
 * GROUND_TRACKING_DETERMINATION_PROGRAM (Luminary 099): P21.
 *
 * Gives the astronaut details of the LM or CSM ground track (lat, long, altitude) at a
 * requested time without ground communication. Requested by V37E21E; terminated by V34E.
 * Subroutines: GOPERF4, GOFLASH, THISPREC, OTHPREC, LAT-LONG.
 * Displays: option code (R2: 00001 = THIS/LM, 00002 = OTHER/CSM), time to integrate (V06N34),
 * latitude/longitude/altitude (V06N43). Located in bank 33 at P20S, EBANK = P21TIME.
 */
public final class GroundTrackingDeterminationProgram {
    private final Agc agc;

    public static final String V06N43 = "00643"; // VN: display lat, long, alt
    public static final String V6N34 = "00634";  // VN: load desired time of lat-long
    /** 600SEC: 2DEC 60000 (10 minutes, centiseconds). */
    public static final int SEC600 = 60000;
    /** K.01 */
    public static final double K_01 = 0.01;

    public GroundTrackingDeterminationProgram(Agc agc) {
        this.agc = agc;
    }

    private static final String P21_BASER = "P21BASER";

    // Interpretive work values kept in Java (vectors / times)
    public double[] p21BaseR = new double[3];
    public double[] p21BaseV = new double[3];
    public double p21Time;
    public double tdec1;

    /** PROG21: assume vehicle = LM (R2 = 00001), option 2 request, then fall to P21PROG1. */
    public void prog21() {
        agc.set("OPTION2", 1);
        // CAF BIT2 ; BANKCALL GOPERF4 : option display. Terminate -> GOTOPOOH; proceed -> OK; V32 -> reload.
        new Goperf4Hook().gopread(agc);
        p21prog1();
    }

    /** P21PROG1: request the time of lat-long (V06N34) and run the integration. */
    public void p21prog1() {
        // CAF V6N34 ; BANKCALL GOFLASH : loaded time arrives in DSPTEM1; terminate -> GOTOPOOH
        double dsptem1 = dp("DSPTEM1");
        tdec1 = dsptem1;                 // STCALL TDEC1 ; INTSTALL
        boolean firstPassOrRecycle = agc.get("P21FLAG") != 0;
        agc.set("VINTFLAG", 0);
        if (firstPassOrRecycle) {
            p21cont();
            return;
        }
        // 1st pass: call base vector
        if (Agc.toSigned(agc.get("OPTION2")) != 0) {
            agc.set("VINTFLAG", 1);      // ONE -- other vehicle (CSM)
        }
        agc.set("DIM0FLAG", 0);
        agc.set("INTYPFLG", 0);          // precision
        // CALL INTEGRV -- calculate (defined elsewhere), then save base vector
        p21vsave();
    }

    /** P21CONT: recycle by integrating from the saved base vector. */
    public void p21cont() {
        // RCV = P21BASER ; VCV = P21BASEV ; TET = P21TIME
        agc.set("DIM0FLAG", 0);
        agc.set("MOONFLAG", 0);
        if (Agc.toSigned(agc.get("P21ORIG")) != 0) {
            agc.set("MOONFLAG", 1);      // 2 = Moon
        }
        // CALL INTEGRVS
        p21vsave();
    }

    /** P21VSAVE: save current base vector (TAT, RATT1, VATT1) and compute velocity and flight path angle. */
    public void p21vsave() {
        // P21TIME = TAT ; P21BASER = RATT1 (B-29 or B-27) ; P21BASEV = VATT1 (B-07 or B-05)
        // P21VEL = |VATT1| scaled for N91 ; P21GAM = asin(U(R).U(V)) (-90 to +90 degrees)
        // SLOAD OPTION2 ; SR1 ; BHIZ +3 : this vehicle (LM) on the surface skips setting P21FLAG
        boolean otherVehicle = (Agc.toSigned(agc.get("OPTION2")) >> 1) != 0;
        if (otherVehicle || agc.get("SURFFLAG") == 0) {
            agc.set("P21FLAG", 1);
        }
        p21dsp();
    }

    /** P21DSP: generate display data (LAT-LONG) and display V06N43, then recycle in 10 minutes. */
    public void p21dsp() {
        agc.set("LUNAFLAG", 0);
        if (Agc.toSigned(agc.get("X2")) != 0) {
            agc.set("LUNAFLAG", 1);      // 0 = Earth
        }
        // ALPHAV = RATT ; TAT -> LAT-LONG with ERADFLAG cleared; ALT = MPAC * K.01 -> P21ALT (N91)
        agc.set("ERADFLAG", 0);
        // CAF V06N43 ; BANKCALL GOFLASH : lat,long = 1/2 revs B0; alt = km B14
        // Terminate (GOTOPOOH) on V34E; V32E recycles:
        double next = dp("P21TIME") + SEC600; // DAD 600SEC
        setDp("DSPTEM1", next);
        p21prog1(); // RTB P21PROG1
    }

    private static final class Goperf4Hook {
        void gopread(Agc agc) {
            // BANKCALL GOPERF4 (defined elsewhere): TERMINATE / PROCEED / R2 loaded
        }
    }

    // ---- double-precision erasable helpers (two consecutive words: sym, sym+1) ----
    private double dp(String sym) {
        return Agc.toSigned(agc.get(sym)) / 16384.0 + Agc.toSigned(agc.get(sym + "+1")) / 268435456.0;
    }

    private void setDp(String sym, double v) {
        long q = Math.round(v * 268435456.0);
        int hi = (int) Math.floorDiv(q, 16384L);
        int lo = (int) (q - (long) hi * 16384L);
        agc.set(sym, Agc.fromSigned(hi));
        agc.set(sym + "+1", Agc.fromSigned(lo));
    }
}
