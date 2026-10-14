package apollo.luminary099;

import apollo.agc.Agc;

/**
 * INTERPRETER.agc (Luminary 099): the AGC interpretive language virtual machine.
 *
 * <p>Interpretive programs are lists of operations acting on the multi-purpose accumulator
 * MPAC (MPAC..MPAC+6), a push-down list, two index registers X1/X2 and the interpretive
 * switch words (STATE). In this translation each interpretive op code is a Java method.
 * Operand "addresses" are erasable symbol names (a String such as {@code "MIS+6"}); passing
 * {@code null} for an operand means "no address given: push up", i.e. take the operand
 * from the push-down list, exactly as the original PUSHUP routine does.
 *
 * <p>Number formats: a double-precision (DP) word pair (hi, lo) is a 28-bit one's-complement
 * fraction (hi holds the 14 most significant bits); a vector is three DP components at
 * offsets 0, 2 and 4; triple precision (TP) adds a third low-order word at offset 2.
 * MODE: 0 = DP scalar, 1 = TP scalar, -1 = vector.
 * Overflow sets OVFIND and leaves the saturated (POSMAX/NEGMAX) result.
 *
 * <p>Sections of the original: 1 dispatcher (INTPRET, DANZIG, OPJUMP, address processing, jump
 * tables), 2 load/store package, sequence changing, 3 add/subtract, multiply/divide
 * subroutines, vector operations, shifts, division, V/SC, UNIT, SQRT, trigonometry,
 * index register instructions, switch instructions.
 */
public class Interpreter {
    // ---- constants from the source ----
    public static final long ONE_DP = 1L << 28;           // 1.0 in DP scale
    public static final long MAXDP = ONE_DP - 1;          // POSMAX,POSMAX
    public static final int NO_WDS_DP = 2;                // pushdown words: DP
    public static final int NO_WDS_TP = 3;                // TP
    public static final int NO_WDS_VEC = 6;               // vector
    public static final int DEC45 = 45;                   // -ENDVAC: length of the VAC work area
    public static final int OCT1400 = 01400;
    public static final int LOW7 = 0177;
    public static final int OCT176 = 0176;
    public static final int B3TOB4 = 014;
    public static final int OCT1302 = 01302;              // SQRT of negative argument (POODOO)
    public static final int OCT1301 = 01301;              // ARCCOS/ARCSIN argument out of range (ALARM)
    public static final double SLOPEHI = 0.5884, BIASHI = 0.4192 / 2, SLOPELO = 0.8324, BIASLO = 0.2974 / 2;
    public static final int MODE_DP = 0, MODE_TP = 1, MODE_VEC = -1;

    /** The jump tables of the dispatcher, indexed by the op code field. */
    public static final String[] INDJUMP = {
        "VLOAD", "TAD", "SIGN", "VXSC", "CGOTO", "TLOAD", "DLOAD", "V/SC",
        "SLOAD", "SSP", "PDDL", "MXV", "PDVL", "CCALL", "VXM", "TSLC",
        "DMPR", "DDV", "BDDV", "GSHIFT", "VAD", "VSU", "BVSU", "DOT",
        "VXV", "VPROJ", "DSU", "BDSU", "DAD", "AVAILABLE", "DMP1", "SETPD"};
    public static final String[] MISCJUMP = {
        "AXT", "AXC", "LXA", "LXC", "SXA", "XCHX", "INCR", "TIX",
        "XAD", "XSU", "BZE/GOTO", "BPL/BMN", "RTB/BHIZ", "CALL/ITA", "SW/", "BOV(B)"};
    public static final String[] UNAJUMP = {
        "EXIT", "SQRT", "SINE", "COSINE", "ARCSIN", "ARCCOS", "DSQ", "ROUND",
        "COMP", "VDEF", "UNIT", "ABVALABS", "VSQ", "STADR", "RVQ", "PUSH"};
    public static final String[] STORJUMP = {
        "STORE", "STORE,1", "STORE,2", "STODL", "STODL*", "STOVL", "STOVL*", "STOTC"};

    protected final Agc agc;
    private int mode = MODE_DP;

    public Interpreter(Agc agc) {
        this.agc = agc;
    }

    // =====================================================================
    // Low level storage helpers
    // =====================================================================

    /** Symbol at word offset {@code off} from {@code sym}; understands "NAME+n" and "NAME+nD". */
    protected static String at(String sym, int off) {
        if (off == 0) {
            return sym;
        }
        int plus = sym.lastIndexOf('+');
        if (plus > 0) {
            String tail = sym.substring(plus + 1);
            String digits = tail.endsWith("D") ? tail.substring(0, tail.length() - 1) : tail;
            if (!digits.isEmpty() && digits.chars().allMatch(Character::isDigit)) {
                return sym.substring(0, plus) + "+" + (Integer.parseInt(digits) + off);
            }
        }
        return sym + "+" + off;
    }

    private static String mp(int i) {
        return at("MPAC", i);
    }

    /** Reads a DP value (28-bit signed) at sym, sym+1. */
    protected long dp(String sym) {
        return ((long) Agc.toSigned(agc.get(sym)) << 14) + Agc.toSigned(agc.get(at(sym, 1)));
    }

    /** Stores a DP value at sym, sym+1 with sign agreement between the two words. */
    protected void putDp(String sym, long v) {
        long m = Math.abs(v);
        int hi = (int) ((m >> 14) & 0x3FFF);
        int lo = (int) (m & 0x3FFF);
        if (v < 0) {
            hi = -hi;
            lo = -lo;
        }
        agc.set(sym, Agc.fromSigned(hi));
        agc.set(at(sym, 1), Agc.fromSigned(lo));
    }

    /** Reads a TP value (42-bit) at sym, sym+1, sym+2. */
    protected long tp(String sym) {
        return (dp(sym) << 14) + Agc.toSigned(agc.get(at(sym, 2)));
    }

    protected void putTp(String sym, long v) {
        long m = Math.abs(v);
        long hi = m >> 14;
        int low = (int) (m & 0x3FFF);
        putDp(sym, v < 0 ? -hi : hi);
        agc.set(at(sym, 2), Agc.fromSigned(v < 0 ? -low : low));
    }

    /** Reads vector components at sym, sym+2, sym+4. */
    protected long[] vec(String sym) {
        return new long[] {dp(sym), dp(at(sym, 2)), dp(at(sym, 4))};
    }

    protected void putVec(String sym, long[] v) {
        putDp(sym, v[0]);
        putDp(at(sym, 2), v[1]);
        putDp(at(sym, 4), v[2]);
    }

    private long mpacDp() {
        return dp("MPAC");
    }

    private long[] mpacVec() {
        return new long[] {dp("MPAC"), dp("MPAC+3"), dp("MPAC+5")};
    }

    private void setMpacDp(long v) {
        putDp("MPAC", saturate(v));
        agc.set("MPAC+2", 0);
        setMode(MODE_DP);
    }

    private void setMpacVec(long[] v) {
        putDp("MPAC", saturate(v[0]));
        putDp("MPAC+3", saturate(v[1]));
        putDp("MPAC+5", saturate(v[2]));
        setMode(MODE_VEC);
    }

    /** OVERFLOW: clamp to POSMAX/NEGMAX and set OVFIND. */
    private long saturate(long v) {
        if (v > MAXDP) {
            agc.set("OVFIND", 1);
            return MAXDP;
        }
        if (v < -MAXDP) {
            agc.set("OVFIND", 1);
            return -MAXDP;
        }
        return v;
    }

    public int mode() {
        return mode;
    }

    /** NEWMODE: prologue for mode-changing instructions. */
    protected void setMode(int m) {
        mode = m;
        agc.set("MODE", Agc.fromSigned(m));
    }

    /** True if OVFIND is set. */
    public boolean overflowed() {
        return agc.get("OVFIND") != 0;
    }

    // ---- push-down list: words live in erasable symbols "PD+ptr" ----

    private int pushloc() {
        return Agc.toSigned(agc.get("PUSHLOC"));
    }

    private int wordsFor(int m) {
        return m == MODE_VEC ? NO_WDS_VEC : (m == MODE_TP ? NO_WDS_TP : NO_WDS_DP);
    }

    /** SETPD: set the push-down pointer. */
    public void setpd(int ptr) {
        agc.set("PUSHLOC", Agc.fromSigned(ptr));
    }

    private void pushWords(String src, int words) {
        int p = pushloc();
        for (int i = 0; i < words; i++) {
            agc.set("PD+" + (p + i), agc.get(at(src, i)));
        }
        agc.set("PUSHLOC", Agc.fromSigned(p + words));
    }

    /** PUSHUP: operand is the given address or the top of the push-down list. */
    private String operand(String addr, int words) {
        if (addr != null) {
            return addr;
        }
        int p = pushloc() - words;
        agc.set("PUSHLOC", Agc.fromSigned(p));
        return "PD+" + p;
    }

    private String operandForMode(String addr) {
        return operand(addr, wordsFor(mode));
    }

    // =====================================================================
    // Section 1: dispatcher
    // =====================================================================

    /** INTPRET: enter interpretive mode (the following Java statements are the interpretive code). */
    public void intpret() {
        agc.set("BANKSET", agc.get("BBANK"));
        agc.set("EDOP", 0);
    }

    /** INTRSM: resume a suspended interpretive job. */
    public void intrsm() {
        intpret();
    }

    /** EXIT: leave interpretive mode and return to basic code. */
    public void exit() {
        agc.set("BBANK", agc.get("BANKSET"));
    }

    /**
     * Dispatches a mnemonic from the INDJUMP table; used when interpretive code is
     * represented as data.
     */
    public void dispatchIndexed(int opcode, String addr) {
        String name = INDJUMP[opcode & 037];
        switch (name) {
            case "VLOAD": vload(addr); break;
            case "TAD": tad(addr); break;
            case "SIGN": sign(addr); break;
            case "VXSC": vxsc(addr); break;
            case "TLOAD": tload(addr); break;
            case "DLOAD": dload(addr); break;
            case "V/SC": vdivsc(addr); break;
            case "SLOAD": sload(addr); break;
            case "PDDL": pddl(addr); break;
            case "MXV": mxv(addr); break;
            case "PDVL": pdvl(addr); break;
            case "VXM": vxm(addr); break;
            case "TSLC": tslc(addr); break;
            case "DMPR": dmpr(addr); break;
            case "DDV": ddv(addr); break;
            case "BDDV": bddv(addr); break;
            case "VAD": vad(addr); break;
            case "VSU": vsu(addr); break;
            case "BVSU": bvsu(addr); break;
            case "DOT": dot(addr); break;
            case "VXV": vxv(addr); break;
            case "VPROJ": vproj(addr); break;
            case "DSU": dsu(addr); break;
            case "BDSU": bdsu(addr); break;
            case "DAD": dad(addr); break;
            case "DMP1": dmp1(addr); break;
            default: throw new UnsupportedOperationException(name);
        }
    }

    /** Dispatches a unary op code from the UNAJUMP table. */
    public void dispatchUnary(int opcode) {
        switch (UNAJUMP[opcode & 017]) {
            case "EXIT": exit(); break;
            case "SQRT": sqrt(); break;
            case "SINE": sine(); break;
            case "COSINE": cosine(); break;
            case "ARCSIN": arcsin(); break;
            case "ARCCOS": arccos(); break;
            case "DSQ": dsq(); break;
            case "ROUND": round(); break;
            case "COMP": comp(); break;
            case "VDEF": vdef(); break;
            case "UNIT": unit(); break;
            case "ABVALABS": abvalabs(); break;
            case "VSQ": vsq(); break;
            case "STADR": stadr(); break;
            case "RVQ": rvq(); break;
            case "PUSH": push(); break;
            default: break;
        }
    }

    // =====================================================================
    // Section 2: load and store package
    // =====================================================================

    /** DLOAD: load a DP scalar. MPAC+2 is zeroed. */
    public void dload(String addr) {
        setMpacDp(dp(operandForLoad(addr, NO_WDS_DP)));
    }

    /** DLOAD* style helper: the load operand when pushing up is a DP (2 words). */
    private String operandForLoad(String addr, int words) {
        return operand(addr, words);
    }

    /** SLOAD: load single precision (low word zero). */
    public void sload(String addr) {
        String a = operandForLoad(addr, 1);
        long hi = Agc.toSigned(agc.get(a));
        setMpacDp(hi << 14);
        agc.set("MPAC+1", 0);
    }

    /** TLOAD: load triple precision. */
    public void tload(String addr) {
        String a = operandForLoad(addr, NO_WDS_TP);
        agc.set("MPAC+2", agc.get(at(a, 2)));
        putDp("MPAC", dp(a));
        setMode(MODE_TP);
    }

    /** VLOAD: load a vector (three DP components). */
    public void vload(String addr) {
        String a = operandForLoad(addr, NO_WDS_VEC);
        setMpacVec(vec(a));
    }

    /** PDDL: push down MPAC and reload DP. */
    public void pddl(String addr) {
        String a = addr; // the load must not consume the push before MPAC is saved
        long loaded = dp(a);
        push();
        setMpacDp(loaded);
    }

    /** PDVL: push down MPAC and load a vector. */
    public void pdvl(String addr) {
        long[] loaded = vec(addr);
        push();
        setMpacVec(loaded);
    }

    /** PUSH: push down MPAC leaving it loaded (also the "PUSH" unary op). */
    public void push() {
        pushWords("MPAC", mode == MODE_VEC ? 2 : 2);
        if (mode == MODE_VEC) {
            int p = pushloc();
            // x already pushed (2 words); push Y and Z (words at MPAC+3, MPAC+5)
            agc.set("PD+" + p, agc.get("MPAC+3"));
            agc.set("PD+" + (p + 1), agc.get("MPAC+4"));
            agc.set("PD+" + (p + 2), agc.get("MPAC+5"));
            agc.set("PD+" + (p + 3), agc.get("MPAC+6"));
            agc.set("PUSHLOC", Agc.fromSigned(p + 4));
        } else if (mode == MODE_TP) {
            int p = pushloc();
            agc.set("PD+" + p, agc.get("MPAC+2"));
            agc.set("PUSHLOC", Agc.fromSigned(p + 1));
        }
        // ENDDPUSH/ENDTPUSH: MPAC+2 zeroed, TP becomes DP
        if (mode != MODE_VEC) {
            agc.set("MPAC+2", 0);
            setMode(MODE_DP);
        }
    }

    /** SSP: store single precision constant at address (the word following the address). */
    public void ssp(String addr, int value) {
        agc.set(addr, value);
    }

    /** STORE: store MPAC (DP, TP or vector according to MODE) at addr. */
    public void store(String addr) {
        putDp(addr, mpacDp());           // STARTSTO: MPAC,+1 always
        if (mode == MODE_VEC) {          // VSTORE
            agc.set(at(addr, 2), agc.get("MPAC+3"));
            agc.set(at(addr, 3), agc.get("MPAC+4"));
            agc.set(at(addr, 4), agc.get("MPAC+5"));
            agc.set(at(addr, 5), agc.get("MPAC+6"));
        } else if (mode == MODE_TP) {    // TSTORE
            agc.set(at(addr, 2), agc.get("MPAC+2"));
        }
    }

    /** STORE,1 / STORE,2: store indexed by X1 or X2. */
    public void store(String addr, int indexReg) {
        store(indexed(addr, indexReg));
    }

    /** STODL: store, then DP load (null = push up). */
    public void stodl(String storeAddr, String loadAddr) {
        store(storeAddr);
        dload(loadAddr);
    }

    /** STOVL: store, then vector load (null = push up). */
    public void stovl(String storeAddr, String loadAddr) {
        store(storeAddr);
        vload(loadAddr);
    }

    /** STCALL: store, then call the interpretive subroutine. */
    public void stcall(String storeAddr, Runnable sub) {
        store(storeAddr);
        call(sub);
    }

    /** STOTC (stotc) : store and then transfer control (GOTO). */
    public void stotc(String storeAddr, Runnable target) {
        store(storeAddr);
        target.run();
    }

    /** STADR: push up on store address; the store code that follows takes its operand from the push-down list. */
    public void stadr() {
        // marker: no state change (see STADR at the head of the store package)
    }

    /** Resolves an operand address modified by index register 1 or 2 (the "*" forms). */
    private String indexed(String sym, int reg) {
        return at(sym, Agc.toSigned(agc.get(reg == 1 ? "X1" : "X2")));
    }

    /** DLOAD* etc: loads whose address is indexed by X1/X2. */
    public void dloadIndexed(String sym, int reg) {
        dload(indexed(sym, reg));
    }

    public void vloadIndexed(String sym, int reg) {
        vload(indexed(sym, reg));
    }

    // =====================================================================
    // Sequence changing (GOTO, CALL, RTB, branches)
    // =====================================================================

    /** CALL: call an interpretive subroutine, leaving the return in QPRET (modelled by the Java call stack). */
    public void call(Runnable sub) {
        sub.run();
    }

    /** CCALL: computed call - the target is selected by an index from a list. */
    public void ccall(Runnable[] list, String indexAddr) {
        list[Agc.toSigned(agc.get(indexAddr))].run();
    }

    /** CGOTO: computed goto. */
    public void cgoto(Runnable[] list, String indexAddr) {
        list[Agc.toSigned(agc.get(indexAddr))].run();
    }

    /** RVQ: return via QPRET (the caller continues after the Java call). */
    public void rvq() {
        // return handled by Java method return
    }

    /** RTB: return to basic language at the given routine, executing it. */
    public void rtb(Runnable basicRoutine) {
        exit();
        basicRoutine.run();
        intpret();
    }

    /** ITA: store the return address in erasable (QPRET -&gt; addr). */
    public void ita(String addr) {
        agc.set(addr, agc.get("QPRET"));
    }

    /** BRANCH: triple precision test, returns 1 (&gt;0), 0 (zero) or -1 (&lt;0). */
    public int branch() {
        long v = tp("MPAC");
        return Long.signum(v);
    }

    /** BZE: branch if MPAC is zero. */
    public boolean bze() {
        return branch() == 0;
    }

    /** BPL: branch if MPAC positive or zero. */
    public boolean bpl() {
        return branch() >= 0;
    }

    /** BMN: branch if MPAC negative nonzero. */
    public boolean bmn() {
        return branch() < 0;
    }

    /** BHIZ: branch if the high-order word of MPAC is zero. */
    public boolean bhiz() {
        return Agc.toSigned(agc.get("MPAC")) == 0;
    }

    /** BOV: branch on overflow; clears the indicator. */
    public boolean bov() {
        boolean was = agc.get("OVFIND") != 0;
        agc.set("OVFIND", 0);
        return was;
    }

    /** SETPD alias used by routines. */
    public void setpd(String sym) {
        agc.set("PUSHLOC", agc.get(sym));
    }

    // =====================================================================
    // Index register instructions
    // =====================================================================

    private String xreg(int reg) {
        return reg == 1 ? "X1" : "X2";
    }

    private int xval(int reg) {
        return Agc.toSigned(agc.get(xreg(reg)));
    }

    private void xset(int reg, int v) {
        agc.set(xreg(reg), Agc.fromSigned(v));
    }

    /** AXT,n: address to index true. */
    public void axt(int reg, int address) {
        xset(reg, address);
    }

    /** AXC,n: address to index complemented. */
    public void axc(int reg, int address) {
        xset(reg, -address);
    }

    /** AXC,n with an erasable symbol whose word offset (distance in erasable) is used as the address. */
    public void axc(int reg, String sym) {
        indexSymbols[reg] = sym;
        xset(reg, 0);
    }

    private final String[] indexSymbols = new String[3];

    /** The symbol whose address was loaded into an index register by axc(reg, sym). */
    public String indexSymbol(int reg) {
        return indexSymbols[reg];
    }

    /** LXA,n: load index from erasable. */
    public void lxa(int reg, String addr) {
        xset(reg, Agc.toSigned(agc.get(addr)));
    }

    /** LXC,n: load index from complement of erasable. */
    public void lxc(int reg, String addr) {
        xset(reg, -Agc.toSigned(agc.get(addr)));
    }

    /** SXA,n: store index in erasable. */
    public void sxa(int reg, String addr) {
        agc.set(addr, agc.get(xreg(reg)));
    }

    /** XCHX,n: exchange index with erasable. */
    public void xchx(int reg, String addr) {
        int t = agc.get(addr);
        agc.set(addr, agc.get(xreg(reg)));
        agc.set(xreg(reg), t);
    }

    /** INCR,n: increment index register by the given amount. */
    public void incr(int reg, int amount) {
        xset(reg, xval(reg) + amount);
    }

    /** XAD,n: add erasable to index. */
    public void xad(int reg, String addr) {
        xset(reg, xval(reg) + Agc.toSigned(agc.get(addr)));
    }

    /** XSU,n: subtract erasable from index. */
    public void xsu(int reg, String addr) {
        xset(reg, xval(reg) - Agc.toSigned(agc.get(addr)));
    }

    /** TIX,n: decrement index; returns true (branch) if the decremented value is positive. */
    public boolean tix(int reg) {
        int d = xval(reg) - 1;
        if (d <= 0) {
            return false; // no operation if decremented index is negative or zero
        }
        xset(reg, d);
        return true;
    }

    // =====================================================================
    // Interpretive switches (STATE words)
    // =====================================================================

    /** Switch names are erasable flag symbols: non-zero means set. */
    public boolean bon(String sw) {
        return agc.get(sw) != 0;
    }

    public boolean boff(String sw) {
        return agc.get(sw) == 0;
    }

    public void set(String sw) {
        agc.set(sw, 1);
    }

    public void clear(String sw) {
        agc.set(sw, 0);
    }

    public void invert(String sw) {
        agc.set(sw, agc.get(sw) == 0 ? 1 : 0);
    }

    /** BONSET: set the switch, branch if it was on. */
    public boolean bonset(String sw) {
        boolean was = bon(sw);
        set(sw);
        return was;
    }

    /** BOFSET: set the switch, branch if it was off. */
    public boolean bofset(String sw) {
        boolean was = bon(sw);
        set(sw);
        return !was;
    }

    /** BONINV: invert the switch, branch if it was on. */
    public boolean boninv(String sw) {
        boolean was = bon(sw);
        invert(sw);
        return was;
    }

    /** BOFINV: invert the switch, branch if it was off. */
    public boolean bofinv(String sw) {
        boolean was = bon(sw);
        invert(sw);
        return !was;
    }

    /** BONCLR: clear the switch, branch if it was on. */
    public boolean bonclr(String sw) {
        boolean was = bon(sw);
        clear(sw);
        return was;
    }

    /** BOFCLR: clear the switch, branch if it was off. */
    public boolean bofclr(String sw) {
        boolean was = bon(sw);
        clear(sw);
        return !was;
    }

    // =====================================================================
    // Section 3: add / subtract package
    // =====================================================================

    /** DAD: DP add. */
    public void dad(String addr) {
        String a = operand(addr, NO_WDS_DP);
        setMpacDp(mpacDp() + dp(a));
    }

    /** DSU: DP subtract (MPAC - X). */
    public void dsu(String addr) {
        String a = operand(addr, NO_WDS_DP);
        setMpacDp(mpacDp() - dp(a));
    }

    /** BDSU: DP subtract from (X - MPAC). */
    public void bdsu(String addr) {
        String a = operand(addr, NO_WDS_DP);
        setMpacDp(dp(a) - mpacDp());
    }

    /** TAD: triple precision add; overflow sets OVFIND. */
    public void tad(String addr) {
        String a = operand(addr, NO_WDS_TP);
        long r = tp("MPAC") + tp(a);
        long max = (1L << 42) - 1;
        if (Math.abs(r) > max) {
            agc.set("OVFIND", 1);
            r = r > 0 ? max : -max;
        }
        putTp("MPAC", r);
        setMode(MODE_TP);
    }

    /** VAD: vector add. */
    public void vad(String addr) {
        long[] a = vec(operand(addr, NO_WDS_VEC));
        long[] m = mpacVec();
        setMpacVec(new long[] {m[0] + a[0], m[1] + a[1], m[2] + a[2]});
    }

    /** VSU: vector subtract (MPAC - X). */
    public void vsu(String addr) {
        long[] a = vec(operand(addr, NO_WDS_VEC));
        long[] m = mpacVec();
        setMpacVec(new long[] {m[0] - a[0], m[1] - a[1], m[2] - a[2]});
    }

    /** BVSU: vector subtract from (X - MPAC). */
    public void bvsu(String addr) {
        long[] a = vec(operand(addr, NO_WDS_VEC));
        long[] m = mpacVec();
        setMpacVec(new long[] {a[0] - m[0], a[1] - m[1], a[2] - m[2]});
    }

    // =====================================================================
    // Multiply / round / polynomial subroutines
    // =====================================================================

    /** Product of two DP fractions, rounded toward zero to DP; MPAC+2 gets the 14 low-order bits. */
    private long mulDp(long a, long b, boolean keepLow) {
        long p = a * b; // up to 56 bits; the DP product is p / 2^28
        long m = Math.abs(p);
        long hi = m >> 28;
        if (keepLow) {
            agc.set("MPAC+2", Agc.fromSigned((int) (p < 0 ? -((m >> 14) & 0x3FFF) : ((m >> 14) & 0x3FFF))));
        }
        return p < 0 ? -hi : hi;
    }

    /** DMPSUB: multiply MPAC,+1 by the DP at addr, leaving the (TP) result in MPAC. */
    public void dmpsub(String addr) {
        long r = mulDp(mpacDp(), dp(addr), true);
        putDp("MPAC", saturate(r));
    }

    /** DMP: basic subroutine form used by PINBALL etc. (address of the multiplier). */
    public void dmp(String addr) {
        dmpsub(addr);
    }

    /** DMP1 (the DMP instruction): DP multiply. */
    public void dmp1(String addr) {
        dmpsub(operand(addr, NO_WDS_DP));
        setMode(MODE_TP);
    }

    /** DMPR: DP multiply and round. */
    public void dmpr(String addr) {
        dmpsub(operand(addr, NO_WDS_DP));
        roundsub();
    }

    /** ROUNDSUB / ROUND: round the TP contents of MPAC to DP (sets DP mode). */
    public void roundsub() {
        int low = Agc.toSigned(agc.get("MPAC+2"));
        long v = mpacDp();
        if (Math.abs(low) > 0x1FFF) { // magnitude greater than .5
            v += low > 0 ? 1 : -1;
        }
        setMpacDp(v);
    }

    /** ROUND (unary op). */
    public void round() {
        roundsub();
    }

    /** DMPNSUB: multiply the DP fraction in MPAC by the single precision integer n. */
    public long dmpnsub(int n) {
        long v = mpacDp() * n;
        setMpacDp(v);
        return v;
    }

    /** SHORTMP: multiply the TP MPAC by a single-precision number. */
    public void shortmp(int sp) {
        long r = (tp("MPAC") * Agc.toSigned(sp)) / (1L << 14);
        putTp("MPAC", r);
    }

    /** DSQ: DP square. */
    public void dsq() {
        long a = mpacDp();
        setMpacDp(mulDp(a, a, false));
    }

    /** POLY: evaluate A0 + A1 x + ... + An x^n, x = MPAC (DP); coefficients as DP values. */
    public void poly(long[] coef) {
        long x = mpacDp();
        long r = coef[coef.length - 1];
        for (int i = coef.length - 2; i >= 0; i--) {
            r = mulDp(r, x, false) + coef[i];
        }
        setMpacDp(r);
    }

    /** Converts a decimal fraction to a DP value, as 2DEC does. */
    public static long dec(double f) {
        return Math.round(f * ONE_DP);
    }

    /** ALSIGNAG / VECAGREE / TPAGREE: sign agreement is implicit in putDp(). */
    public void tpagree() {
        putTp("MPAC", tp("MPAC"));
    }

    // =====================================================================
    // Vector operations
    // =====================================================================

    /** DOTSUB: dot product of vector in MPAC with the vector at addr (stride 2) into MPAC. */
    private long dotWith(long[] a) {
        long[] m = mpacVec();
        return mulDp(m[0], a[0], false) + mulDp(m[1], a[1], false) + mulDp(m[2], a[2], false);
    }

    /** DOT: vector dot product, result DP scalar. */
    public void dot(String addr) {
        long[] a = vec(operand(addr, NO_WDS_VEC));
        setMpacDp(dotWith(a));
    }

    /** VSQ: square of the length of the vector. */
    public void vsq() {
        setMpacDp(dotWith(mpacVec()));
    }

    /** ABVAL / ABS (ABVALABS): length of vector or absolute value of scalar. */
    public void abvalabs() {
        if (mode == MODE_VEC) {
            abval();
        } else {
            abs();
        }
    }

    /** ABVAL: leaves length-squared in LVSQUARE and length in MPAC. */
    public void abval() {
        long sq = dotWith(mpacVec());
        putDp("LVSQUARE", sq);
        setMpacDp(sq);
        sqrt();
    }

    /** ABS: absolute value of the scalar. */
    public void abs() {
        if (branch() < 0) {
            comp();
        }
    }

    /** COMP: complement vector or scalar. */
    public void comp() {
        if (mode == MODE_VEC) {
            long[] m = mpacVec();
            setMpacVec(new long[] {-m[0], -m[1], -m[2]});
        } else {
            long tpv = tp("MPAC");
            putTp("MPAC", -tpv);
        }
    }

    /** SIGN: complement MPAC (vector or scalar) if the word at addr is negative. */
    public void sign(String addr) {
        long x = dp(addr);
        if (x < 0) {
            comp();
        }
    }

    /** VXSC: vector times scalar. If MPAC is a vector the scalar is at addr (or pushup). */
    public void vxsc(String addr) {
        if (mode == MODE_VEC) {
            long s = dp(operand(addr, NO_WDS_DP));
            long[] m = mpacVec();
            setMpacVec(new long[] {rnd(m[0], s), rnd(m[1], s), rnd(m[2], s)});
        } else {
            long s = mpacDp();
            long[] v = vec(operand(addr, NO_WDS_VEC));
            setMpacVec(new long[] {rnd(v[0], s), rnd(v[1], s), rnd(v[2], s)});
        }
    }

    private long rnd(long a, long b) {
        long p = a * b;
        long m = Math.abs(p);
        long hi = (m + (1L << 27)) >> 28; // VROUND
        return p < 0 ? -hi : hi;
    }

    /** VPROJ: vector projection (MPAC . X) MPAC ... (X . MPAC) X. */
    public void vproj(String addr) {
        long[] x = vec(operand(addr, NO_WDS_VEC));
        long d = dotWith(x);
        setMpacVec(new long[] {rnd(d, x[0]), rnd(d, x[1]), rnd(d, x[2])});
    }

    /** VXV: vector cross product MPAC x X. */
    public void vxv(String addr) {
        long[] x = vec(operand(addr, NO_WDS_VEC));
        long[] m = mpacVec();
        long cx = mulDp(m[1], x[2], false) - mulDp(m[2], x[1], false);
        long cy = mulDp(m[2], x[0], false) - mulDp(m[0], x[2], false);
        long cz = mulDp(m[0], x[1], false) - mulDp(m[1], x[0], false);
        setMpacVec(new long[] {cx, cy, cz});
    }

    /** MXV: matrix (rows stored consecutively, 3 DP per row) times vector in MPAC. */
    public void mxv(String addr) {
        String a = operand(addr, NO_WDS_VEC); // matrix base address
        long[] m = mpacVec();
        long[] r = new long[3];
        for (int i = 0; i < 3; i++) {
            long[] row = vec(at(a, 6 * i));
            r[i] = mulDp(row[0], m[0], false) + mulDp(row[1], m[1], false) + mulDp(row[2], m[2], false);
        }
        setMpacVec(r);
    }

    /** VXM: vector in MPAC times matrix (i.e. M-transpose x v). */
    public void vxm(String addr) {
        String a = operand(addr, NO_WDS_VEC);
        long[] m = mpacVec();
        long[] r = new long[3];
        for (int j = 0; j < 3; j++) {
            for (int i = 0; i < 3; i++) {
                r[j] += mulDp(m[i], dp(at(a, 6 * i + 2 * j)), false);
            }
        }
        setMpacVec(r);
    }

    /** VDEF: vector define from the scalar in MPAC (x) and two pushed-down scalars (y, z). */
    public void vdef() {
        long x = mpacDp();
        int p = pushloc() - 4;
        agc.set("PUSHLOC", Agc.fromSigned(p));
        long y = dp("PD+" + p);
        long z = dp("PD+" + (p + 2));
        setMpacVec(new long[] {x, y, z});
    }

    /** UNIT: half-unit vector in the direction of MPAC; leaves length in LV and its square in LVSQUARE. */
    public void unit() {
        long[] v = mpacVec();
        double x = (double) v[0] / ONE_DP, y = (double) v[1] / ONE_DP, z = (double) v[2] / ONE_DP;
        double sq = x * x + y * y + z * z;
        if (sq >= 1.0) {
            agc.set("OVFIND", 1);
            setMpacDp(MAXDP);
            return;
        }
        double len = Math.sqrt(sq);
        putDp("LVSQUARE", dec(sq));
        putDp("LV", dec(len));
        if (len == 0.0) {
            agc.set("OVFIND", 1);
            setMpacDp(MAXDP);
            return;
        }
        setMpacVec(new long[] {dec(x / len / 2), dec(y / len / 2), dec(z / len / 2)});
    }

    // =====================================================================
    // Division
    // =====================================================================

    /** DDV: MPAC divided by the DP at addr. */
    public void ddv(String addr) {
        long divisor = dp(operand(addr, NO_WDS_DP));
        divide(mpacDp(), divisor);
    }

    /** BDDV: the DP at addr divided by MPAC. */
    public void bddv(String addr) {
        long dividend = dp(operand(addr, NO_WDS_DP));
        divide(dividend, mpacDp());
    }

    /** DDV/BDDV core: DP fraction divide; overflow gives +-POSMAX with OVFIND set. */
    private void divide(long dividend, long divisor) {
        if (dividend == 0) {
            setMpacDp(0);
            return;
        }
        if (divisor == 0 || Math.abs(dividend) >= Math.abs(divisor)) {
            agc.set("OVFIND", 1);
            setMpacDp(((dividend < 0) != (divisor < 0)) ? -MAXDP : MAXDP);
            return;
        }
        long q = (dividend << 28) / divisor; // truncates toward zero
        setMpacDp(q);
    }

    /** V/SC: vector divided by scalar. */
    public void vdivsc(String addr) {
        long[] v;
        long s;
        if (mode == MODE_VEC) {
            s = dp(operand(addr, NO_WDS_DP));
            v = mpacVec();
        } else {
            s = mpacDp();
            v = vec(operand(addr, NO_WDS_VEC));
        }
        long[] r = new long[3];
        for (int i = 0; i < 3; i++) {
            if (s == 0 || Math.abs(v[i]) >= Math.abs(s)) {
                if (v[i] != 0) {
                    agc.set("OVFIND", 1);
                    r[i] = ((v[i] < 0) != (s < 0)) ? -MAXDP : MAXDP;
                }
            } else {
                r[i] = (v[i] << 28) / s;
            }
        }
        setMpacVec(r);
    }

    // =====================================================================
    // Shifts
    // =====================================================================

    private static long shr(long v, int n) {
        return v >= 0 ? v >> n : -((-v) >> n);
    }

    private static long shrRound(long v, int n) {
        if (n <= 0) {
            return v;
        }
        long m = (Math.abs(v) + (1L << (n - 1))) >> n;
        return v < 0 ? -m : m;
    }

    private void shiftScalar(int n, boolean round) {
        long v = mpacDp();
        if (n >= 0) {
            long tpv = tp("MPAC");
            long r = round ? shrRound(v, n) : shr(v, n);
            putTp("MPAC", shr(tpv, n));
            if (round) {
                putDp("MPAC", r);
            }
        } else {
            long r = v;
            for (int i = 0; i < -n; i++) {
                r = r * 2;
                if (Math.abs(r) > MAXDP) {
                    agc.set("OVFIND", 1);
                    r = r > 0 ? MAXDP : -MAXDP;
                }
            }
            putDp("MPAC", r);
            if (round) {
                roundsub();
            }
        }
        setMode(MODE_DP);
    }

    private void shiftVector(int n) {
        long[] m = mpacVec();
        long[] r = new long[3];
        for (int i = 0; i < 3; i++) {
            r[i] = n >= 0 ? shrRound(m[i], n) : m[i] << (-n);
        }
        setMpacVec(r);
    }

    /** SR n / SRR n / SL n / SLR n general shifts. Positive count shifts right, negative left. */
    public void gshift(int rightCount, boolean round) {
        if (mode == MODE_VEC) {
            shiftVector(rightCount);
        } else {
            shiftScalar(rightCount, round);
        }
    }

    public void sr(int n) { gshift(n, false); }
    public void srr(int n) { gshift(n, true); }
    public void sl(int n) { gshift(-n, false); }
    public void slr(int n) { gshift(-n, true); }
    public void sr1() { sr(1); } public void sr2() { sr(2); } public void sr3() { sr(3); } public void sr4() { sr(4); }
    public void sl1() { sl(1); } public void sl2() { sl(2); } public void sl3() { sl(3); } public void sl4() { sl(4); }
    public void sr1r() { srr(1); } public void sr2r() { srr(2); } public void sr3r() { srr(3); } public void sr4r() { srr(4); }
    public void sl1r() { slr(1); } public void sl2r() { slr(2); } public void sl3r() { slr(3); } public void sl4r() { slr(4); }

    /** VSR n / VSL n (n = 1..8). */
    public void vsr(int n) { shiftVector(n); }
    public void vsl(int n) { shiftVector(-n); }

    /** TSLC: shift MPAC left until its magnitude is at least .5; stores the negative count at addr. */
    public void tslc(String addr) {
        long v = tp("MPAC");
        int count = 0;
        if (v != 0) {
            long half = 1L << 40;
            while (Math.abs(v) < half) {
                v <<= 1;
                count++;
            }
        }
        putTp("MPAC", v);
        agc.set(addr, Agc.fromSigned(-count));
    }

    // =====================================================================
    // Square root and trigonometry
    // =====================================================================

    /** SQRT: square root of the MPAC fraction. Aborts with alarm 1302 for arguments below about -1.2e-4. */
    public void sqrt() {
        long v = mpacDp();
        if (v < 0) {
            if (v < -(ONE_DP * 12 / 100000)) {
                new AlarmAndAbort(agc).poodoo(OCT1302);
            }
            setMpacDp(0);
            return;
        }
        double r = Math.sqrt((double) v / ONE_DP);
        setMpacDp(Math.min(dec(r), MAXDP));
    }

    /** SINE: (1/2) sin(2 pi x), x in revolutions. */
    public void sine() {
        double x = (double) mpacDp() / ONE_DP;
        setMpacDp(dec(0.5 * Math.sin(2 * Math.PI * x)));
    }

    /** COSINE: (1/2) cos(2 pi x), x in revolutions. */
    public void cosine() {
        double x = (double) mpacDp() / ONE_DP;
        setMpacDp(dec(0.5 * Math.cos(2 * Math.PI * x)));
    }

    /** ARCSIN: (1/2 pi) asin(2 x), result in revolutions. */
    public void arcsin() {
        double x = 2.0 * mpacDp() / ONE_DP;
        if (Math.abs(x) > 1.0) {
            new AlarmAndAbort(agc).alarm(OCT1301, 0);
            setMpacDp(0);
            return;
        }
        setMpacDp(dec(Math.asin(x) / (2 * Math.PI)));
    }

    /** ARCCOS: (1/2 pi) acos(2 x), result in revolutions (ARCCOS(0) = .25). */
    public void arccos() {
        double x = 2.0 * mpacDp() / ONE_DP;
        if (Math.abs(x) > 1.0) {
            new AlarmAndAbort(agc).alarm(OCT1301, 0);
            setMpacDp(0);
            return;
        }
        setMpacDp(dec(Math.acos(x) / (2 * Math.PI)));
    }
}
