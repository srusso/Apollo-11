package apollo.luminary099;

import apollo.agc.Agc;

/**
 * THROTTLE_CONTROL_ROUTINES (Luminary 099, bank 31, FTHROT).
 * Computes the throttle command (THRUST / PIF) for the descent engine and the FWEIGHT mass-loss estimate.
 * Double-precision erasables are modelled as the pair SYMBOL (high) and SYMBOL+1 (low) keys, i.e. "FP" and "FP+1".
 */
public class ThrottleControlRoutines {
    /** DEC .13107 : acceleration-to-force constant. */
    public static final int AF_CNST = 0x0CCC; // /AF/CNST, .13107 * 2^14 (approx)
    /** FEXTRA = BIT13 (4096 pulses). */
    public static final int FEXTRA = 0x1000;
    public static final int BIT13 = 0x1000;
    public static final int BIT4 = 0x0008;

    private final Agc agc;
    private int rtnHold;

    public ThrottleControlRoutines(Agc agc) {
        this.agc = agc;
    }

    /** THROTTLE: compute present acceleration, thrust and throttle command. */
    public void throttle() {
        // CA ABDELV; MP /AF/CNST -> acceleration in units 2(-4) M/CS/CS
        int product = Agc.toSigned(agc.get("ABDELV")) * AF_CNST;
        agc.set("A", Agc.fromSigned(product >> 14));
        afdump();
    }

    /** AFDUMP: FP = present thrust, FC = thrust desired by guidance. */
    public void afdump() {
        massMult();                 // TC MASSMULT
        agc.set("FP", agc.get("MPAC"));
        agc.set("FP+1", agc.get("MPAC+1"));
        agc.set("MPAC", agc.get("/AFC/"));
        agc.set("MPAC+1", agc.get("/AFC/+1"));
        massMult();
        agc.set("FC", agc.get("MPAC"));          // FC = thrust desired by guidance
        agc.set("FCODD", agc.get("MPAC"));       // FCODD = what it is going to get
        agc.set("FCODD+1", agc.get("MPAC+1"));
        // Within 3 seconds of last throttle call, add FWEIGHT into FP.
        int dt = Agc.toSigned(agc.get("TIME1")) - Agc.toSigned(agc.get("TTHROT"));
        dt &= 0x3FFF;               // MASK POSMAX
        if (dt + 1 > 300) {         // 3SECS (centiseconds), branch if (TIME1-TTHROT+1) > 3 s
            // WHERETO directly
        } else {
            dasAdd("FP", "FWEIGHT");
        }
        wheretoLabel();
    }

    /** WHERETO: initialise L*WCR*T and H*GHCR*T from pad-loaded erasables in the W-matrix, choose throttle regime. */
    public void wheretoLabel() {
        agc.set("L*WCR*T", agc.get("LOWCRIT"));
        agc.set("L*WCR*T+1", agc.get("LOWCRIT+1"));
        agc.set("PIFPSET", 0);
        int fcold = Agc.toSigned(agc.get("FCOLD"));
        int highcrit = Agc.toSigned(agc.get("H*GHCR*T"));
        if (fcold - highcrit <= 0) {
            lowFcold();
            return;
        }
        int fcodd = Agc.toSigned(agc.get("FCODD"));
        if (fcodd - Agc.toSigned(agc.get("L*WCR*T")) <= 0) {
            fcompset();
        } else {
            // NOTE 1: throttle-down path, hold the thrust at FP
            flatout1(agc.get("FP"));
        }
    }

    /** FCOMPSET: PIFPSET = FP - FMAXODD. */
    public void fcompset() {
        flatout2(Agc.toSigned(agc.get("FP")) - Agc.toSigned(agc.get("FMAXODD")));
    }

    /** LOWFCOLD: FCOLD was in the low region. */
    public void lowFcold() {
        if (Agc.toSigned(agc.get("FCODD")) - Agc.toSigned(agc.get("H*GHCR*T")) <= 0) {
            dopif();
        } else {
            flatout1(agc.get("FMAXPOS")); // throttle-up
        }
    }

    /** FLATOUT1: FCODD = A; then PIFPSET = FEXTRA. */
    public void flatout1(int a) {
        agc.set("FCODD", a);
        flatout2(FEXTRA);
    }

    /** FLATOUT2: set PIFPSET. */
    public void flatout2(int value) {
        agc.set("PIFPSET", Agc.fromSigned(value));
        dopif();
    }

    /** DOPIF: PIF = FC - FP, never +0. */
    public void dopif() {
        // TC FASTCHNG (switch to fast-change mode) omitted - no Agc model
        int fcodd = Agc.toSigned(agc.get("FCODD"));
        agc.set("FCOLD", Agc.fromSigned(fcodd));
        int pif = fcodd - Agc.toSigned(agc.get("FP"));
        agc.set("PIF", Agc.fromSigned(pif));
        doit();
    }

    /** DOIT: output PSEUDO55/THRUST, set CHAN14 bit 4, and update the FWEIGHT mass-loss estimate. */
    public void doit() {
        int pif = Agc.toSigned(agc.get("PIF")) + Agc.toSigned(agc.get("PIFPSET"));
        agc.set("PSEUDO55", Agc.fromSigned(pif));
        agc.set("THRUST", Agc.fromSigned(pif));
        agc.set("CHAN14", agc.get("CHAN14") | BIT4);     // WOR CHAN14
        int time1 = agc.get("TIME1");
        agc.set("TTHROT", time1);
        boolean p66 = Agc.toSigned(agc.get("MODREG")) == 66;
        int q;
        if (p66) {
            agc.set("BUF", agc.get("THISTPIP+1"));   // initialise FWEIGHT comp as for P66
            q = 200;                                  // 2SECS
        } else {
            agc.set("BUF", agc.get("PIPTIME+1"));    // two second period
            q = 400;                                  // 4SECS
        }
        fwcompTail(q);
    }

    /** FWCOMP / FWCOMP+1: FWEIGHT = PIF * (time since last PIPA + THROTLAG) / q, scaled. */
    public void fwcompTail(int q) {
        agc.set("BUF+1", Agc.fromSigned((q * 32) & 0x3FFF)); // MP BIT6 -> LXCH BUF+1
        int dt = (Agc.toSigned(agc.get("TIME1")) - Agc.toSigned(agc.get("BUF"))
                + Agc.toSigned(agc.get("THROTLAG"))) & 0xFF;     // MASK LOW8: small and positive
        int pif = Agc.toSigned(agc.get("PIF"));
        long fw = ((long) dt * 16384 / q) * pif * 2;
        agc.set("FWEIGHT", Agc.fromSigned((int) (fw >> 14)));
        int adj = pif > 0 ? 1 : 1;      // CCS PIF; AD ONE either way
        int denom = Math.max(1, Agc.toSigned(agc.get("BUF+1")));
        long extra = (long) (pif + adj) * pif / denom;
        agc.set("FWEIGHT+1", Agc.fromSigned((int) extra));
        thdump();
    }

    /** THDUMP: return to caller (TC RTNHOLD). */
    public void thdump() {
        // return
    }

    /** FLATOUT: force the engine to FLATOUT (4096 pulses). */
    public void flatout() {
        whatout(BIT13);
    }

    /** WHATOUT: PIFPSET = A so FWEIGHT will be zero; clear FCOLD and PIF. */
    public void whatout(int a) {
        agc.set("PIFPSET", a);
        agc.set("FCOLD", 0);
        agc.set("PIF", 0);
        doit();
    }

    /** MASSMULT: MPAC = MPAC * MASS * SCALEFAC, properly scaled force; overflow saturates to POSMAX. */
    public void massMult() {
        long v = Agc.toSigned(agc.get("MPAC"));
        v = (v * Agc.toSigned(agc.get("MASS"))) >> 14;
        v = (v * Agc.toSigned(agc.get("SCALEFAC"))) >> 14;
        if (v > Agc.WORD_MASK >> 1) {
            agc.set("MPAC", Agc.WORD_MASK >> 1);     // POSMAX
        } else {
            agc.set("MPAC", Agc.fromSigned((int) v));
        }
    }

    private void dasAdd(String dst, String src) {
        agc.set(dst, Agc.add(agc.get(dst), agc.get(src)));
        agc.set(dst + "+1", Agc.add(agc.get(dst + "+1"), agc.get(src + "+1")));
    }
}
