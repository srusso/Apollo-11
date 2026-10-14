package apollo.comanche055;

import apollo.agc.Agc;

/**
 * RCS-CSM_DAP_EXECUTIVE_PROGRAMS.agc (Comanche 055, pages 1037-1038).
 *
 * Calculation of AMGB, AMBG once every second.
 * AMGB = [1 sin(psi) 0; 0 cos(psi)cos(phi) sin(phi); 0 -cos(psi)sin(phi) cos(phi)]
 * where phi and psi are CDU angles.
 */
public class RcsCsmDapExecutivePrograms {
    private final Agc agc;

    /** QUADANGL: 7.25 degrees jet quad angular offset (DEC 660). */
    public static final int QUADANGL = 660;

    public RcsCsmDapExecutivePrograms(Agc agc) {
        this.agc = agc;
    }

    /** AMBGUPDT: update the AMGB matrix elements (RCS autopilot only). */
    public void ambgupdt() {
        int flagwrd6 = Agc.toSigned(agc.get("FLAGWRD6"));
        // BZMF ENDOFJOB (bit15 = 0 ... non-positive) ; MASK BIT14 / BZF ENDOFJOB: exit if not RCS
        if (flagwrd6 <= 0 || (agc.get("FLAGWRD6") & 0x2000) == 0) {
            endOfJob();
            return;
        }
        int cduz = agc.get("CDUZ");
        agc.set("AMGB1", spsin2(cduz));          // AMGB1 = sin(CDUZ)
        int capsi = spcos2(cduz);                // cos(PSI); must check for gimbal lock
        agc.set("CAPSI", capsi);

        // MSU computes QUADANGL - CDUX, COM negates: CDUX - 7.25 deg
        int cduxRel = Agc.fromSigned(Agc.toSigned(agc.get("CDUX")) - QUADANGL);
        agc.set("AMGB8", spcos1(cduxRel));
        agc.set("AMGB4", mp(agc.get("AMGB8"), capsi));
        agc.set("AMGB5", spsin1(cduxRel));
        int p = mp(agc.get("AMGB5"), capsi);
        agc.set("AMGB7", Agc.fromSigned(-Agc.toSigned(p)));  // COM
        endOfJob();
    }

    /** Fractional (single precision) multiply, high-order result. */
    private static int mp(int a, int b) {
        long prod = (long) Agc.toSigned(a) * Agc.toSigned(b);
        return Agc.fromSigned((int) (prod >> 14));
    }

    // Trig subroutines (SPSIN/SPCOS family) in another file.
    private int spsin2(int a) { return new SingleOrDouble().sin(a, 2); }
    private int spcos2(int a) { return new SingleOrDouble().cos(a, 2); }
    private int spsin1(int a) { return new SingleOrDouble().sin(a, 1); }
    private int spcos1(int a) { return new SingleOrDouble().cos(a, 1); }

    /** Adapter onto the SPSIN/SPCOS routines (units: revolutions scaled B0 or B-1). */
    private static final class SingleOrDouble {
        int sin(int a, int scale) {
            double rev = Agc.toSigned(a) / (double) (scale == 1 ? 16384 : 8192);
            return Agc.fromSigned((int) Math.round(Math.sin(2 * Math.PI * rev) * 16384));
        }
        int cos(int a, int scale) {
            double rev = Agc.toSigned(a) / (double) (scale == 1 ? 16384 : 8192);
            return Agc.fromSigned((int) Math.round(Math.cos(2 * Math.PI * rev) * 16384));
        }
    }

    /** ENDOFJOB: ends the current Executive job (see Executive). */
    private void endOfJob() {
        new Executive(agc).endOfJob();
    }
}
