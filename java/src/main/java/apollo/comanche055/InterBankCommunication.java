package apollo.comanche055;

import apollo.agc.Agc;

/**
 * INTER-BANK_COMMUNICATION.agc (Comanche 055, pages 1103-1106).
 *
 * Bank-switching linkage. In the Java model there are no physical banks; a CADR is
 * represented as an int (FBANK in bits 15-11, address within bank in low 10 bits) and
 * "calls" are resolved by a caller-supplied target. The bank/superbank state is kept in
 * the FBANK / SUPERBNK pseudo-registers held in the Agc erasable table.
 */
public class InterBankCommunication {
    private final Agc agc;

    public static final int LOW10 = 01777;
    public static final int SUPER011 = 060;
    public static final int SUPER100 = 0100;
    public static final int SUPER101 = 0120;
    public static final int SUPER110 = 0140;
    /** PRIO12 = TCF 10000 = 12000. */
    public static final int PRIO12 = 012000;

    public InterBankCommunication(Agc agc) {
        this.agc = agc;
    }

    /**
     * BANKCALL: call a subroutine in another bank. The CADR following the TC is passed
     * here as {@code cadr}; C(A,L) are preserved for the callee. Saves the caller's
     * FBANK in BUF2+1 and the return in BUF2, then runs {@code target}.
     */
    public void bankcall(int cadr, Runnable target) {
        swcall(cadr, target);
    }

    /** SWCALL: identical to BANKCALL but the CADR arrives in A. */
    public void swcall(int cadr, Runnable target) {
        int savedFbank = agc.get("FBANK");
        agc.set("BUF2 +1", savedFbank);
        agc.set("FBANK", cadr);               // switch banks, saving return
        target.run();
        swreturn();
    }

    /** SWRETURN: restores the caller's FBANK, preserving C(A,L). */
    public void swreturn() {
        agc.set("FBANK", agc.get("BUF2 +1"));
    }

    /** POSTJUMP / BANKJUMP: unilateral bank jump with C(A,L) preserved. */
    public void bankjump(int cadr, Runnable target) {
        agc.set("FBANK", cadr);
        target.run();
    }

    /** MAKECADR: returns the return CADR saved by SWCALL or BANKCALL. */
    public int makecadr() {
        return (LOW10 & agc.get("BUF2")) + agc.get("BUF2 +1");
    }

    /**
     * SUPDACAL: fetch one word of data from a CADR including superbank.
     * Saves FBANK (bits 15-11) and SUPERBNK (bits 7-5), sets both for the data,
     * reads the word, then restores.
     */
    public int supdacal(int cadrWithSuperbits, int addressInBank, java.util.function.IntUnaryOperator read) {
        int oldFbank = agc.get("FBANK");
        int oldSuper = agc.get("SUPERBNK");
        agc.set("FBANK", cadrWithSuperbits);
        agc.set("SUPERBNK", cadrWithSuperbits & 0160);
        int word = read.applyAsInt(addressInBank & LOW10);
        agc.set("SUPERBNK", oldSuper);
        agc.set("FBANK", oldFbank);
        return word;
    }

    /** IBNKCALL: like BANKCALL, for use in interrupt context (RUPTREG3,4 hold return). */
    public void ibnkcall(int cadr, Runnable target) {
        agc.set("RUPTREG4", agc.get("FBANK"));
        agc.set("FBANK", cadr);
        target.run();
        iswretrn();
    }

    /** ISWCALL: like SWCALL, for interrupt context. */
    public void iswcall(int cadr, Runnable target) {
        ibnkcall(cadr, target);
    }

    /** ISWRETRN: restore FBANK from RUPTREG4. */
    public void iswretrn() {
        agc.set("FBANK", agc.get("RUPTREG4"));
    }

    /**
     * USPRCADR: access interpretive coding outside the user's FBANK. Sets EDOP to BIT8
     * (exit instruction), BANKSET to the user's BBANK and FBANK to the interpretive bank.
     */
    public void usprcadr(int cadr, Runnable interpretive) {
        agc.set("EDOP", 0200);                // BIT8
        agc.set("BANKSET", agc.get("BBANK"));
        agc.set("FBANK", cadr);
        interpretive.run();
    }

    /**
     * SUPERSW: write bits 7-6-5 of A into channel 07 (SUPERBNK). Callable by programs
     * in banks 00-27 only.
     */
    public void supersw(int a) {
        agc.set("SUPERBNK", a & 0160);
    }
}
