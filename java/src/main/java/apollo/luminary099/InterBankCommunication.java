package apollo.luminary099;

import apollo.agc.Agc;

/**
 * Translation of INTER-BANK_COMMUNICATION.agc (Luminary 1A build 099).
 *
 * Bank-switching linkage: BANKCALL/SWCALL, POSTJUMP/BANKJUMP, MAKECADR, SUPDACAL,
 * interrupt-time IBNKCALL, USPRCADR and SUPERSW. A CADR is modelled as an int
 * (bank number in the high bits, 10-bit relative address in the low bits) and the
 * callee as a {@link Runnable}.
 */
public class InterBankCommunication {
    private static final int LOW10 = 01777;

    private final Agc agc;

    public InterBankCommunication(Agc agc) {
        this.agc = agc;
    }

    /** BANKCALL: save A,L in BUF2, fetch the CADR following the call, switch banks, call, restore. */
    public void bankcall(int cadr, Runnable target) {
        agc.set("BUF2", agc.get("A"));
        agc.set("BUF2+1", agc.get("L"));
        swcall(cadr, target, true);
    }

    /** SWCALL: A holds the CADR; switch FBANK, run the target as a subroutine, return via SWRETURN. */
    public void swcall(int cadr, Runnable target, boolean restoreInputs) {
        int oldFbank = agc.get("FBANK");
        agc.set("FBANK", cadr & ~LOW10);
        if (restoreInputs) {
            agc.set("A", agc.get("BUF2"));
            agc.set("L", agc.get("BUF2+1"));
        }
        target.run();
        swreturn(oldFbank);
    }

    /** SWRETURN: restore the caller's FBANK, preserving A and L. */
    public void swreturn(int callerFbank) {
        agc.set("FBANK", callerFbank);
    }

    /** POSTJUMP: the word after the TC is a CADR; jump there with A preserved. */
    public void postjump(int cadr, Runnable target) {
        bankjump(cadr, target);
    }

    /** BANKJUMP: A holds the CADR; set FBANK and transfer control (never returns to the jumper). */
    public void bankjump(int cadr, Runnable target) {
        agc.set("FBANK", cadr & ~LOW10);
        target.run();
    }

    /** PRIO12: TCF 12000 (entry in fixed-fixed). */
    public void prio12(Runnable target) {
        target.run();
    }

    /** MAKECADR: combine the bank bits in BUF2 with the 10-bit address in BUF2+1. */
    public int makecadr() {
        return (LOW10 & agc.get("BUF2")) + agc.get("BUF2+1");
    }

    /**
     * SUPDACAL: fetch the first word of a double-precision datum that may lie in a
     * super-bank. FBANK and SUPERBNK are temporarily switched and restored.
     * @param memory the fixed-memory image addressed by (superbank, fbank, offset)
     */
    public int supdacal(int fbankAndSuper, int offset, java.util.function.IntBinaryOperator memory) {
        int oldFbank = agc.get("FBANK");
        int oldSuper = agc.get("SUPERBNK");
        agc.set("MPTEMP", fbankAndSuper);
        agc.set("FBANK", fbankAndSuper & 076000);
        agc.set("SUPERBNK", (fbankAndSuper & 0160) >> 0);
        int word = memory.applyAsInt(fbankAndSuper, offset & LOW10);
        agc.set("SUPERBNK", oldSuper);
        agc.set("FBANK", oldFbank);
        agc.set("MPTEMP", word);
        return word;
    }

    /** IBNKCALL: interrupt-time bank call using RUPTREG3,4 for the return address. */
    public void ibnkcall(int cadr, Runnable target) {
        int oldFbank = agc.get("FBANK");
        agc.set("RUPTREG3", agc.get("Q"));
        agc.set("FBANK", cadr & ~LOW10);
        target.run();
        agc.set("FBANK", oldFbank); // ISWRETRN
    }

    /** USPRCADR: set up EDOP/BANKSET and switch to the interpretive bank for a user CADR. */
    public void usprcadr(int cadr) {
        agc.set("LOC", agc.get("A"));
        agc.set("EDOP", 0200); // BIT8: exit instruction to EDOP
        agc.set("BANKSET", agc.get("BBANK"));
        agc.set("FBANK", cadr & ~LOW10);
        // continues at Q+10000 with the interpretive relative address (cadr & LOW10)
    }

    /** SUPERSW: write bits 7-6-5 of A into the super-bank register (channel 07). */
    public void supersw() {
        agc.set("SUPERBNK", agc.get("A") & 0160);
    }
}
