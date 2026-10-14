package apollo.agc;

import java.util.HashMap;
import java.util.Map;

/**
 * Minimal shared model of the AGC machine used by the Java translations.
 *
 * Erasable memory is modelled as named 15-bit one's-complement words (stored in an int),
 * plus a symbolic table so translated routines can refer to erasable symbols by name.
 * Routines are modelled as methods; "TC", "TCF" and "CALL" become Java method calls,
 * jumps to labels become method calls or state-machine transitions documented in comments.
 */
public final class Agc {
    public static final int WORD_MASK = 0x7FFF;
    public static final int SIGN_BIT = 0x4000;

    private final Map<String, Integer> erasable = new HashMap<>();

    /** Logs a call to a routine that is stubbed in this translation (no Java counterpart yet). */
    public static void stubCalled(String what) {
        System.err.println("[AGC STUB] " + what + " called");
    }

    /** Reads an erasable symbol (default 0). */
    public int get(String symbol) {
        return erasable.getOrDefault(symbol, 0);
    }

    /** Writes an erasable symbol, truncated to 15 bits. */
    public void set(String symbol, int value) {
        erasable.put(symbol, value & WORD_MASK);
    }

    /** Converts a 15-bit one's-complement word to a Java int. */
    public static int toSigned(int word) {
        word &= WORD_MASK;
        return (word & SIGN_BIT) != 0 ? -((~word) & WORD_MASK) : word;
    }

    /** Converts a Java int to a 15-bit one's-complement word. */
    public static int fromSigned(int value) {
        return value >= 0 ? value & WORD_MASK : (~(-value)) & WORD_MASK;
    }

    /** One's-complement addition with end-around carry, as performed by the AGC adder. */
    public static int add(int a, int b) {
        int sum = (a & WORD_MASK) + (b & WORD_MASK);
        if (sum > WORD_MASK) {
            sum = (sum & WORD_MASK) + 1;
        }
        return sum & WORD_MASK;
    }
}
