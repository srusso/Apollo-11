package apollo.luminary099;

import apollo.agc.Agc;

/**
 * Translation of RTB_OP_CODES.agc (Luminary 1A build 099).
 *
 * "Run The Basic" op codes: basic-language routines callable from interpretive code
 * (RTB). Each returns to the interpreter via DANZIG / NEWMODE / SLOAD2, which are
 * modelled as calls into {@link Interpreter}.
 */
public class RtbOpCodes {
    private static final int POSMAX = 037777;
    private static final int HALF = 020000;

    private final Agc agc;

    public RtbOpCodes(Agc agc) {
        this.agc = agc;
    }

    /** LOADTIME: load TIME2,TIME1 into MPAC (DP) via SLOAD2. */
    public void loadtime() {
        agc.set("L", agc.get("TIME1"));
        agc.set("A", agc.get("TIME2"));
        new Interpreter(agc).sload2();
    }

    /**
     * CDULOGIC: convert a single-precision 2's complement angle in MPAC (half-revolutions)
     * to a DP 1's complement number scaled in revolutions.
     */
    public void cdulogic() {
        int mpac = Agc.toSigned(agc.get("MPAC"));
        // CCS MPAC: for positive/+0 low word is 0; for negative low word is -HALF
        int low = mpac < 0 ? Agc.fromSigned(-HALF) : 0;
        agc.set("MPAC+1", low);
        // High word: MPAC * HALF (product, halved) added into the DP pair via DAS
        long dp = ((long) mpac * HALF);          // 28-bit product, high word / low word
        int hi = (int) (dp >> 14);
        int lo = (int) (dp & 037777);
        agc.set("MPAC", Agc.fromSigned(hi));
        agc.set("MPAC+1", Agc.add(agc.get("MPAC+1"), Agc.fromSigned(lo)));
        new Interpreter(agc).danzig(); // mode is already double precision
    }

    /** SGNAGREE: force TP sign agreement in MPAC. */
    public void sgnagree() {
        new Interpreter(agc).tpagree();
        new Interpreter(agc).danzig();
    }

    /**
     * 1STO2S: convert DP 1's complement angle (revolutions) to SP 2's complement angle
     * (half-revolutions).
     */
    public void oneTo2s() {
        oneTo2sub();
        agc.set("MPAC+1", 0);
        new Interpreter(agc).newmode(0);
    }

    /** V1STO2S: 1STO2S applied to a vector of three angles. */
    public void v1sto2s() {
        oneTo2sub();
        swap("MPAC+5", "MPAC");  swap("MPAC+6", "MPAC+1");
        oneTo2sub();
        agc.set("MPAC+2", agc.get("A"));
        swap("MPAC+3", "MPAC");  swap("MPAC+4", "MPAC+1");
        oneTo2sub();
        agc.set("MPAC+1", agc.get("A"));
        agc.set("MPAC", agc.get("MPAC+5"));
        tpmode();
    }

    /** TPMODE: mode is TP. */
    public void tpmode() {
        new Interpreter(agc).newmode(1);
    }

    /** 2V1STO2S: V1STO2S for a 2-component vector (used by the rendezvous radar). */
    public void twoV1sto2s() {
        oneTo2sub();
        swap("MPAC+3", "MPAC");  swap("MPAC+4", "MPAC+1");
        oneTo2sub();
        agc.set("L", agc.get("A"));
        agc.set("A", agc.get("MPAC+3"));
        new Interpreter(agc).sload2();
    }

    /** 1TO2SUB: double the DP value and convert 1's to 2's complement; result in A and MPAC. */
    public void oneTo2sub() {
        int dp = (Agc.toSigned(agc.get("MPAC")) << 14) | (Agc.toSigned(agc.get("MPAC+1")) & 037777);
        int doubled = dp << 1;
        int hi = doubled >> 14;
        int overflow = 0;
        if (hi > POSMAX) { hi -= 040000; overflow = 1; }
        if (hi < -POSMAX) { hi += 040000; overflow = -1; }
        int twos = hi < 0 ? (hi - 1) : hi; // 1's -> 2's complement of the upper word
        int result = Agc.fromSigned(twos + overflow);
        agc.set("MPAC", result);
        agc.set("A", result);
    }

    /**
     * CDUINC: increment the 2's complement register whose address is in BUF (here named by
     * {@code register}) by the 1's complement quantity in A (stored in TEM2).
     */
    public void cduinc(String register) {
        agc.set("TEM2", agc.get("A"));
        int twos = agc.get(register);
        int ones = (twos & Agc.SIGN_BIT) != 0 ? Agc.add(twos, Agc.WORD_MASK) : twos; // 2's -> 1's
        int sum = Agc.add(ones, agc.get("TEM2"));
        int back = (sum & Agc.SIGN_BIT) != 0 ? Agc.add(sum, 1) : sum; // 1's -> 2's
        agc.set("TEM2", back);
        agc.set(register, back);
    }

    /** PULSEIMU: RTB to torque gyros; ECADR of commands arrives in X1. */
    public void pulseimu() {
        pulseimu("X1");
    }

    /** PULSEIMU with the base symbol of the six command words given explicitly. */
    public void pulseimu(String ecadr) {
        Agc.stubCalled("RtbOpCodes.pulseimu ecadr=" + ecadr + " (command-word base symbol may be wrong)");
        agc.set("A", agc.get("X1"));
        new InterBankCommunication(agc).bankcall(0, () -> new ImuModeSwitchingRoutines(agc).imuPulse(ecadr));
        new Interpreter(agc).danzig();
    }

    /**
     * SIGNMPAC: set MPAC,MPAC+1 to +-POSMAX according to the sign of MPAC (+0 gives POSMAX,
     * -0 gives NEGMAX); also used to limit MPAC on interpretive overflow.
     */
    public void signmpac() {
        int a = agc.get("MPAC");
        boolean negative = (a & Agc.SIGN_BIT) != 0 && a != 0 ? true : a == Agc.WORD_MASK;
        if (!negative) {
            agc.set("MPAC", POSMAX);
            agc.set("MPAC+1", POSMAX);
            agc.set("MPAC+2", 0); // DPMODE: also zeroes MPAC+2
            new Interpreter(agc).sload2Plus2();
        } else {
            agc.set("MPAC", Agc.fromSigned(-POSMAX));
            agc.set("MPAC+1", Agc.fromSigned(-POSMAX));
            new Interpreter(agc).sload2();
        }
    }

    /** NORMUNX1: NORMUNIT using index register X1 (ZERO replaced by ONE). */
    public void normunx1() {
        normunitFrom(1);
    }

    /** NORMUNIT: like UNIT, but safe for very small vectors (shifts components left 13). */
    public void normunit() {
        normunitFrom(0);
    }

    private void normunitFrom(int extra) {
        agc.set("MPAC+2", agc.get("FIXLOC") + extra);
        new Interpreter(agc).vecagree();
        boolean noShift = false;
        for (String c : new String[] {"MPAC", "MPAC+3", "MPAC+5"}) {
            if (Agc.toSigned(agc.get(c)) != 0) { noShift = true; break; }
        }
        int thirteen = noShift ? 0 : 13;
        if (!noShift) {
            for (String lo : new String[] {"MPAC", "MPAC+3", "MPAC+5"}) {
                // Shift each component left 13 (multiply low word by BIT14, add into pair).
                int hi = agc.get(lo);
                int low = agc.get(nextWord(lo));
                long dp = ((long) Agc.toSigned(low)) << 13;
                agc.set(lo, Agc.fromSigned(Agc.toSigned(hi) + (int) (dp >> 14)));
                agc.set(nextWord(lo), Agc.fromSigned((int) (dp & 037777)));
            }
        }
        agc.set("37D", thirteen); // exponent of scale factor stored at 37D+index
        offtunit();
    }

    private static String nextWord(String name) {
        return name.equals("MPAC") ? "MPAC+1" : name.equals("MPAC+3") ? "MPAC+4" : "MPAC+6";
    }

    /** OFFTUNIT: continue at UNIT+1, skipping the VECAGREE done at UNIT. */
    public void offtunit() {
        new InterBankCommunication(agc).postjump(0, () -> new Interpreter(agc).unitPlus1());
    }

    /** VECSGNAG: force sign agreement of the vector in MPAC. */
    public void vecsgnag() {
        new Interpreter(agc).vecagree();
        new Interpreter(agc).danzig();
    }

    private void swap(String a, String b) {
        int t = agc.get(a);
        agc.set(a, agc.get(b));
        agc.set(b, t);
    }
}
