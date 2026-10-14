package apollo.comanche055;

import apollo.agc.Agc;

/**
 * RT8_OP_CODES: the interpreter's "RTB" (return to basic) subroutines and related
 * double-precision helpers, plus the QUALITY1/QUALITY2 J22 gravity terms (interpretive).
 * MPAC is the multi-purpose accumulator (MPAC..MPAC+6); vector components are at MPAC,
 * MPAC+3, MPAC+5 (double precision) or MPAC, MPAC+1, MPAC+2 (triple-precision layout).
 */
public class Rt8OpCodes {
    private final Agc agc;

    public static final int DPOSMAX_HI = 0x3FFF;
    public static final int DPOSMAX_LO = 0x3FFF;

    public Rt8OpCodes(Agc agc) {
        this.agc = agc;
    }

    private int mpac(int off) {
        return agc.get(off == 0 ? "MPAC" : "MPAC+" + off);
    }

    private void setMpac(int off, int v) {
        agc.set(off == 0 ? "MPAC" : "MPAC+" + off, v);
    }

    /** LOADTIME: load TIME2/TIME1 as a double precision number into MPAC (SLOAD2). */
    public void loadtime() {
        int hi = agc.get("TIME2");
        int lo = agc.get("TIME1");
        setMpac(0, hi);
        setMpac(1, lo);
        sload2();
    }

    private void sload2() {
        // SLOAD2: store DP number in MPAC and set DP mode
        agc.set("MODE", 2);
    }

    /** CDULOGIC: convert a CDU-style one's complement angle in MPAC to double precision. */
    public void cdulogic() {
        int a = Agc.toSigned(mpac(0));
        int low;
        if (a > 0) {
            low = 0;
        } else if (a < 0) {
            low = Agc.fromSigned(-0x2000); // CS HALF
        } else {
            low = 0;
        }
        setMpac(1, low);
        // XCH MPAC with 0, MP HALF, DAS MPAC: halve the value into the high word
        int hi = Agc.fromSigned(a / 2);
        setMpac(0, hi);
        setMpac(1, Agc.add(mpac(1), Agc.fromSigned((a & 1) != 0 ? 0x2000 : 0)));
        danzig();
    }

    /** DANZIG: return to the interpreter (mode already double precision). */
    public void danzig() {
        // TCF DANZIG
    }

    /** READPIPS: read the PIPA counters atomically into MPAC (vector, DP). */
    public void readpips() {
        // INHINT
        setMpac(0, agc.get("PIPAX"));
        setMpac(3, agc.get("PIPAY"));
        // RELINT
        setMpac(5, agc.get("PIPAZ"));
        setMpac(1, 0);
        setMpac(4, 0);
        setMpac(6, 0);
        vecmode();
    }

    /** VECMODE: set interpreter to vector mode. */
    public void vecmode() {
        agc.set("MODE", 3);
    }

    /** SGNAGREE: force sign agreement of the DP number in MPAC, then return to interpreter. */
    public void sgnagree() {
        tpagree();
        danzig();
    }

    private void tpagree() {
        // TC TPAGREE : make MPAC, MPAC+1, MPAC+2 agree in sign (provided by interpreter core)
    }

    /** 1STO2S: convert 1's-complement scalar in MPAC to 2's complement; clears MPAC+1. */
    public void oneStoTwoS() {
        oneToTwoSub();
        setMpac(1, 0);
        newmode();
    }

    private void newmode() {
        // NEWMODE: A holds the new mode; here recorded in MODE
    }

    /** V1STO2S: convert a vector of 1's-complement words into 2's complement. */
    public void v1sto2s() {
        int a = oneToTwoSub();
        int t5 = mpac(5), t6 = mpac(6);
        setMpac(5, mpac(0));
        setMpac(6, mpac(1));
        setMpac(0, t5);
        setMpac(1, t6);
        a = oneToTwoSub();
        setMpac(2, a);
        int t3 = mpac(3), t4 = mpac(4);
        setMpac(3, mpac(0));
        setMpac(4, mpac(1));
        setMpac(0, t3);
        setMpac(1, t4);
        a = oneToTwoSub();
        setMpac(1, a);
        setMpac(0, mpac(5));
        tpmode();
    }

    /** TPMODE: set triple precision mode. */
    public void tpmode() {
        agc.set("MODE", 1);
    }

    /** 2V1STO2S: two-component version used for 2-vectors. */
    public void twoV1sto2s() {
        oneToTwoSub();
        int t3 = mpac(3), t4 = mpac(4);
        setMpac(3, mpac(0));
        setMpac(4, mpac(1));
        setMpac(0, t3);
        setMpac(1, t4);
        int a = oneToTwoSub();
        agc.set("L", a);
        sload2();
    }

    /**
     * 1TO2SUB: subroutine; convert the DP word in MPAC,MPAC+1 from 1's to 2's complement
     * (double, correct for overflow with LIMITS). Result in A and MPAC.
     */
    public int oneToTwoSub() {
        int hi = Agc.toSigned(mpac(0));
        int doubled = hi * 2;
        int a;
        if (doubled > 0x3FFF) {
            a = 0x3FFF;            // overflow positive -> +MAX (LIMITS)
        } else if (doubled < -0x3FFF) {
            a = Agc.fromSigned(-0x3FFF);
        } else {
            a = Agc.fromSigned(doubled);
            if (doubled > 0) {
                a = Agc.add(a, 1);
            }
        }
        setMpac(0, a);
        return a;
    }

    /** INCRCDUS: increment the three CDU angles THETAD, +1, +2 by 1's complement increments in MPAC. */
    public void incrcdus() {
        String[] names = {"THETAD", "THETAD+1", "THETAD+2"};
        int[] offs = {0, 3, 5};
        for (int i = 0; i < 3; i++) {
            cduinc(names[i], mpac(offs[i]));
        }
        vecmode();
    }

    /** CDUINC: add a 1's complement increment to a 2's complement CDU angle stored at {@code name}. */
    public void cduinc(String name, int increment) {
        int angle = agc.get(name);
        int sum = (angle + (increment & Agc.WORD_MASK)) & Agc.WORD_MASK; // modulo-2^15 (2's complement)
        agc.set(name, sum);
    }

    /** PULSEIMU: send gyro pulse commands (address in X1 via FIXLOC) to IMUPULSE. */
    public void pulseimu() {
        // INDEX FIXLOC ; CA X1 ; BANKCALL IMUPULSE
        danzig();
    }

    /** TRANSP1: transpose the 3x3 matrix at XNB (DP elements). Swaps off-diagonal elements. */
    public void transp1() {
        transpose("XNB");
    }

    /** TRANSP2: transpose the 3x3 matrix at XNB1. */
    public void transp2() {
        transpose("XNB1");
    }

    private void transpose(String base) {
        swapDp(base, 2, 6);
        swapDp(base, 4, 12);
        swapDp(base, 10, 14);
        danzig();
    }

    private void swapDp(String base, int a, int b) {
        for (int w = 0; w < 2; w++) {
            String ka = base + "+" + (a + w);
            String kb = base + "+" + (b + w);
            int t = agc.get(ka);
            agc.set(ka, agc.get(kb));
            agc.set(kb, t);
        }
    }

    /** SIGNMPAC: set MPAC to +/-DPOSMAX according to the sign of its high word. */
    public void signmpac() {
        int hi = Agc.toSigned(mpac(0));
        if (hi == 0) {
            dpmode(); // DPMODE: sets MPAC+2 to zero
            return;
        }
        if (hi > 0) {
            setMpac(0, DPOSMAX_HI);
            setMpac(1, DPOSMAX_LO);
        } else {
            setMpac(0, Agc.fromSigned(-DPOSMAX_HI));
            setMpac(1, Agc.fromSigned(-DPOSMAX_LO));
        }
        sload2();
    }

    /** DPMODE: set double precision mode with MPAC+2 = 0. */
    public void dpmode() {
        setMpac(2, 0);
        sload2();
    }

    /** NORMUNX1: normalize unit vector, index X1 variant. */
    public void normunx1() {
        normunitCommon(1);
    }

    /** NORMUNIT: normalize unit vector helper for UNIT (vector in MPAC). */
    public void normunit() {
        normunitCommon(0);
    }

    private void normunitCommon(int a) {
        setMpac(2, Agc.add(a, agc.get("FIXLOC")));
        // BANKCALL VECAGREE: sign agreement in all components
        boolean noShift = true;
        int[] hi = {mpac(0), mpac(3), mpac(5)};
        for (int h : hi) {
            if (Agc.toSigned(h) != 0) {
                noShift = false;
            }
        }
        if (noShift) {
            // all high words zero -> shift left 13
            for (int[] p : new int[][] {{1, 0}, {4, 3}, {6, 5}}) {
                int v = Agc.toSigned(mpac(p[0]));
                long shifted = (long) v << 1; // MP BIT14 : value*2^1 into the DP pair
                setMpac(p[1], Agc.add(mpac(p[1]), Agc.fromSigned((int) (shifted >> 14))));
                setMpac(p[1] + 1, Agc.fromSigned((int) (shifted & 0x3FFF)));
            }
            agc.set("37D+" + mpac(2), 13); // CAF THIRTEEN ; INDEX MPAC+2 ; TS 37D
        } else {
            noshift();
        }
        offtunit();
    }

    private void noshift() {
        // CAF ZERO ; falls to OFFTUNIT-2 (scale count zero)
        agc.set("37D+" + mpac(2), 0);
    }

    /** OFFTUNIT: POSTJUMP to UNIT+1 in the interpreter (skips the VECAGREE already done). */
    public void offtunit() {
        // TC POSTJUMP ; CADR UNIT +1
    }

    /** VECSGNAG: force sign agreement of vector components, then return to interpreter. */
    public void vecsgnag() {
        // BANKCALL VECAGREE
        danzig();
    }

    /**
     * QUALITY1: J22 gravity term. If MOONFLAG is clear, load NBRANCH(URPV) (earth, no J22 term here);
     * else square the DP and go to QUALITY2. Interpretive code.
     */
    public void quality1() {
        if (agc.get("MOONFLAG") == 0) {
            // BOF MOONFLAG -> NBRANCH
            return;
        }
        // DLOAD URPV ; DSQ ; GOTO QUALITY2
        quality2();
    }

    /**
     * QUALITY2: compute 5(Y^2 - X^2)*UR in 2D,4D,6D with X,Y terms added/subtracted, then
     * multiply by coefficient E3J22R2M to give the J22 term (J22 x R^4, scaled B61) and
     * continue with COSPHI/2 (Z component of URPV).
     */
    public void quality2() {
        // PDDL DSQ URPV+2 (Y^2, B2); DSU (Y^2 - X^2); DMP 5/8 ; VXSC URPV  -> vector in 2D
        // VSL3 ; PDDL URPV ; SR1 ; DAD 2D : X' = X comp + 2*X ; STODL 2D
        // URPV+2 ; SR1 ; BDSU 4D : Y' = Y comp - 2*Y ; STORE 4D
        // SLOAD VXSC E3J22R2M : scale by coefficient
        // PDDL RVQ COSPHI/2 : return to caller with Z component
    }
}
