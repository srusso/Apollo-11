package apollo.comanche055;

import apollo.agc.Agc;

/**
 * GROUND_TRACKING_DETERMINATION_PROGRAM: P21, computes latitude, longitude and altitude of
 * the CSM (or LM) at a time specified by the crew, by integrating from a stored base vector.
 */
public class GroundTrackingDeterminationProgram {
    /** 600SEC: 10 minutes in centiseconds (2DEC 60000). */
    public static final int SEC600 = 60000;
    /** V06N43 display: lat, long, alt. */
    public static final int V06N43 = 0x0643;
    /** V6N34 display: desired time of lat-long. */
    public static final int V6N34 = 0x0634;
    /** K.01 constant. */
    public static final double K_01 = 0.01;

    private final Agc agc;

    public GroundTrackingDeterminationProgram(Agc agc) {
        this.agc = agc;
    }

    /** PROG21: select assumed vehicle (R2 = 00001 means LM), then request the time. */
    public void prog21() {
        agc.set("OPTION2", 1);
        // CAF BIT2; BANKCALL GOPERF4 : requests R2 (option) with a flash
        // TERMINATE -> GOTOPOOH ; PROCEED -> continue ; ENTER with data -> repeat request
        p21prog1();
    }

    /** P21PROG1: flash V6N34 for the desired time, then integrate to that time. */
    public void p21prog1() {
        // CAF V6N34; BANKCALL GOFLASH; TERMINATE -> GOTOPOOH; data entered -> reload
        // INTPRET:
        agc.set("TDEC1", agc.get("DSPTEM1")); // DLOAD DSPTEM1 ; STCALL TDEC1 ; INTSTALL
        if (agc.get("P21FLAG") != 0) {
            p21cont(); // recycle using stored base vector
            return;
        }
        // First pass: P21FLAG off -> set VINTFLAG if OPTION2 != 0 (LM = other vehicle)
        // BHIZ: zero => CM (this vehicle), nonzero => VINTFLAG set (other vehicle)
        // CLEAR DIM0FLAG, INTYPFLG (precision integration)
        // CALL INTEGRV ; GOTO P21VSAVE
        p21vsave();
    }

    /** P21CONT: integrate from stored base vector (P21BASER, P21BASEV, P21TIME). */
    public void p21cont() {
        // RCV = P21BASER ; VCV = P21BASEV ; TET = P21TIME
        // CLEAR DIM0FLAG, MOONFLAG ; if P21ORIG != 0 (2 = moon) SET MOONFLAG
        // CALL INTEGRVS
        p21vsave();
    }

    /** P21VSAVE: save the new base vector and compute /V/ and flight path angle for display. */
    public void p21vsave() {
        agc.set("P21TIME", agc.get("TAT"));
        // P21BASER <- RATT1 ; P21BASEV <- VATT1
        // P21VEL <- |VATT1| (scaled by shift 0,2)
        // P21GAM <- asin( (unit(RATT) . VATT) / P21VEL )  range -90..+90 degrees
        // P21ORIG <- X2 (0 earth, 2 moon) ; SET P21FLAG
        agc.set("P21FLAG", 1);
        p21dsp();
    }

    /** P21DSP: generate display data (lat, long, alt) and display V06N43. */
    public void p21dsp() {
        // CLEAR LUNAFLAG ; if X2 != 0 SET LUNAFLAG
        // ALPHAV <- RATT ; call LAT-LONG with TAT, ERADFLAG cleared
        // P21ALT <- ALT(meters B-29) * K.01 (alt/100 for N73)
        // CAF V06N43; GOFLASH ; TERMINATE -> GOTOPOOH ; PROCEED -> GOTOPOOH
        // V32E recycle -> falls through below
        p21recycle();
    }

    /** Recycle: DSPTEM1 = P21TIME + 600 s, then RTB P21PROG1. */
    public void p21recycle() {
        agc.set("DSPTEM1", Agc.add(agc.get("P21TIME"), SEC600 & Agc.WORD_MASK));
        // RTB P21PROG1 (re-entered after the crew's V32E; not called here to avoid unbounded recursion)
    }
}
