package apollo.comanche055;

import apollo.agc.Agc;

/**
 * TVCSTROKETEST: TVC (thrust vector control) stroke test. Drives a sequence of triangular
 * pulse-burst "strokes" into the TVC pitch command via the DAP.
 */
public class TvcStrokeTest {
    /** FCADDY: number of pulse bursts in 1/2 amplitude, set 1 (+10). */
    public static final int FCADDY = 10;
    /** FREVS: number of reversals minus 1, set 1. */
    public static final int FREVS = 3;
    /** FCARD: number of stroke sets. */
    public static final int FCARD = 4;
    /** FCARD1..3: reversals minus 1 for sets 2, 3, 4. */
    public static final int[] FCARD_REVS = {5, 9, 13};
    /** FCARD4..6: pulse bursts in 1/2 amplitude for sets 2, 3, 4. */
    public static final int[] FCARD_BURSTS = {6, 5, 4};
    /** 20MS = BIT2 (waitlist time). */
    public static final int MS20 = 0x0002;

    private static final int BIT11 = 0x0400;
    private static final int BIT14 = 0x2000;

    private final Agc agc;

    public TvcStrokeTest(Agc agc) {
        this.agc = agc;
    }

    /** STRKTSTI: stroke test initialization (called from TVC DAP init). Then falls into STRKCHK. */
    public void strktsti() {
        tstinit();
        strkchk();
    }

    /** STRKCHK: arm the stroke on the next DAP pass (CSM only; LM config bit 14 inhibits). */
    public void strkchk() {
        // INHINT
        if ((agc.get("DAPDATR1") & BIT14) != 0) {
            // LM configuration: BZF skip, no stroke
            return;
        }
        agc.set("STROKER", agc.get("ESTROKER")); // stroking done in pitch only
        // TCF ENDOFJOB
    }

    /** TSTINIT: initialise CADDY/N (sign changed to negative), REVS, CARD. */
    public void tstinit() {
        agc.set("CADDY", Agc.fromSigned(-FCADDY));
        agc.set("N", Agc.fromSigned(-FCADDY));
        agc.set("REVS", FREVS);
        agc.set("CARD", Agc.fromSigned(-FCARD));
    }

    /** HACK: entry from T5RUPT in TVCDAPS; saves Q in BUNKER and schedules HACKWLST in 20 ms. */
    public void hack(int q) {
        agc.set("BUNKER", q);
        // CAF 20MS; TC WAITLIST; 2CADR HACKWLST
        // Waitlist call modelled by the scheduler: hackwlst() runs 20 ms later.
    }

    /** HACKWLST: waitlist entry; BUNKER becomes TC TASKOVER. Performs one burst of the stroke. */
    public void hackwlst() {
        // BUNKER = TC TASKOVER (exit to task return)
        agc.set("TVCPITCH", Agc.add(agc.get("TVCPITCH"), agc.get("STROKER")));
        // release the error counters: WOR CHAN14 with BIT11
        agc.set("CHAN14", agc.get("CHAN14") | BIT11);
        agc.set("CADDY", Agc.add(agc.get("CADDY"), 1)); // INCR CADDY: count down bursts on this slope
        if (Agc.toSigned(agc.get("CADDY")) < 0) {
            return; // BZMF not taken: CS CADDY <= 0 means CADDY >= 0; still on slope -> exit
        }
        int revs = Agc.toSigned(agc.get("REVS"));
        if (revs > 0) {
            revup(revs);
            return;
        }
        if (revs == 0) {
            revupFinal();
            return;
        }
        // Negative REVS: set finished last pass; ready for next set
        agc.set("CARD", Agc.add(agc.get("CARD"), 1));
        if (Agc.toSigned(agc.get("CARD")) == 0) {
            strokill();
            return;
        }
        int card = Agc.toSigned(agc.get("CARD"));
        int idx = -card - 1; // sets 2..4 -> 0..2
        agc.set("REVS", Agc.fromSigned(-FCARD_REVS[idx]));
        int n = -FCARD_BURSTS[idx];
        agc.set("N", Agc.fromSigned(n));
        agc.set("CADDY", Agc.fromSigned(n));
    }

    /** STROKILL: reset STROKER to +0 to end the test. */
    public void strokill() {
        agc.set("STROKER", 0);
    }

    /** REVUP: all reversals except the last of a set; CADDY = 2 x N (2 x 1/2 amplitude). */
    private void revup(int revs) {
        agc.set("REVS", Agc.fromSigned(revs - 1));
        int n = Agc.toSigned(agc.get("N"));
        agc.set("CADDY", Agc.fromSigned(n * 2));
        reverseSlope();
    }

    /** REVUP+4: final reversal of the set; REVS = -1, CADDY = N (return to zero). */
    private void revupFinal() {
        agc.set("REVS", Agc.fromSigned(-1));
        agc.set("CADDY", agc.get("N"));
        reverseSlope();
    }

    private void reverseSlope() {
        agc.set("STROKER", Agc.fromSigned(-Agc.toSigned(agc.get("STROKER"))));
    }
}
