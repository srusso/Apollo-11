package apollo.comanche055;

import apollo.agc.Agc;

/**
 * MYSUBS (Comanche055): DAP math subroutines, bank 20, EBANK= KMPAC.
 * Aliases: SPCOS1/SPCOS2 = SPCOS, SPSIN1/SPSIN2 = SPSIN.
 */
public final class Mysubs {
    public static final String SPCOS1 = "SPCOS";
    public static final String SPSIN1 = "SPSIN";
    public static final String SPCOS2 = "SPCOS";
    public static final String SPSIN2 = "SPSIN";

    private static final int POSMAX = 0x3FFF;
    private static final int NEGMAX = 0x4000;

    private final Agc agc;

    public Mysubs(Agc agc) {
        this.agc = agc;
    }

    /** SMALLMP: A(X+Y) in A on entry; result AX + AY in KMPAC (double precision). */
    public void smallmp(int a) {
        agc.set("KMPTEMP", a);
        // AY: low-order product of A * KMPAC+1 goes back to KMPAC+1 (single-precision MP, upper word dropped)
        long ay = (long) Agc.toSigned(a) * Agc.toSigned(agc.get("KMPAC+1"));
        int lowAy = (int) (ay & 0x3FFF);
        agc.set("KMPAC+1", Agc.fromSigned((ay < 0) ? -lowAy : lowAy));
        long ax = (long) Agc.toSigned(agc.get("KMPAC")) * Agc.toSigned(a);
        agc.set("KMPAC", 0);
        // DAS KMPAC: add AX (double length) into KMPAC pair
        long hi = ax >> 14;
        long lo = ax & 0x3FFF;
        agc.set("KMPAC", Agc.fromSigned((int) hi));
        agc.set("KMPAC+1", Agc.add(agc.get("KMPAC+1"), Agc.fromSigned((int) lo)));
    }

    /**
     * DPADD: double precision add of the pair at (A, L) into KMPAC, KMPAC+1,
     * with overflow correction (result is forced to +/-MAX on overflow, and the
     * sign of the two halves is made consistent, "TSK" label sets KMPAC).
     */
    public void dpadd(int aHi, int lLo) {
        long k = ((long) Agc.toSigned(agc.get("KMPAC")) << 14) + Agc.toSigned(agc.get("KMPAC+1"));
        long add = ((long) Agc.toSigned(aHi) << 14) + Agc.toSigned(lLo);
        long sum = k + add;
        long max = ((long) POSMAX << 14) + POSMAX;
        if (sum > max) {
            sum = max;                 // DPADD+ : positive overflow
        } else if (sum < -max) {
            sum = -max;                // DPADD- : negative overflow
        }
        int hi = (int) (sum / (1 << 14));
        int lo = (int) (sum % (1 << 14));
        agc.set("KMPAC", Agc.fromSigned(hi));
        agc.set("KMPAC+1", Agc.fromSigned(lo));
    }
}
