package apollo.comanche055;

import apollo.agc.Agc;

/**
 * R60_62.agc (Comanche 055): R60 attitude maneuver routine, VECPOINT, and R62 (V49) maneuver by keyboard.
 *
 * R60CSM points a spacecraft axis; R62DISP (Verb 49) lets the astronaut load new ICDUs and start a
 * 3-axis maneuver; VECPOINT computes the CDU angles for a cross-product rotation (with a gimbal-lock
 * avoidance rotation about SCAXIS where needed).
 */
public final class R6062 {
    public static final int BIT5 = 020, BIT6 = 040, BIT7 = 0100, BIT12 = 04000;
    public static final int BIT14_7 = 020100;
    public static final int V06N18 = 0618;           // VN 0618 : display "please perform auto maneuver"
    public static final int V06N22 = 0622;           // display commanded ICDUs
    public static final int PRIO14 = 014, PRIO30 = 030;
    public static final int OCT71 = 071;             // 1.7SPOT for RELINUS
    public static final int OCT111 = 0111;           // 1.11 for PIKUP20

    // ---- fixed constants (bank 35, MANUVER1) ----
    public static final long SINGIMLC = Interpreter.dec(0.4285836003);  // sin(59 deg)            $2
    public static final long SINVEC1 = Interpreter.dec(0.3796356537);   // sin(49.4 deg)          $2
    public static final long SINVEC2 = Interpreter.dec(0.2462117800);   // sin(29.5 deg)          $2
    public static final long VECANG1 = Interpreter.dec(0.1388888889);   // 50 degrees             $360
    public static final long VECANG2 = Interpreter.dec(0.09722222222);  // 35 degrees             $360
    public static final long DPB_14 = 1L << 14;                          // DP constant 2^-14 (OCT 00001, 00000)

    private final Agc agc;
    private final Interpreter interp;
    private final Executive executive;

    public R6062(Agc agc) {
        this.agc = agc;
        this.interp = new Interpreter(agc);
        this.executive = new Executive(agc);
    }

    /** Hook for user continuations (BANKJUMP back to the caller saved in TEMPR60). */
    private Runnable returnTo;

    /** R60CSM: save the caller's return and start the maneuver logic. */
    public void r60csm(Runnable caller) {
        returnTo = caller;                      // TC MAKECADR ; TS TEMPR60
        agc.set("TEMPR60", agc.get("Q"));
        redomann();
    }

    /** REDOMANN: compute final angles (unless the 3-axis flag is set) then TOBALL. */
    public void redomann() {
        // INSERT PRIODSP CHECK WITH R22 (V06N49) WITH JENNINGS BRODEUR
        if ((agc.get("FLAGWRD5") & BIT6) == 0) {  // 3-axis flag not set
            vecpoint();                           // compute final angles
            interp.store("CPHI");                 // CPHI, CTHETA, CPSI
        }
        toball();
    }

    /** TOBALL: display "please perform auto maneuver" (V06N18). */
    public void toball() {
        int r = displayGoperf2r(V06N18);          // BANKCALL GOPERF2R
        switch (r) {
            case 0:                               // TC R61TEST: terminate
                r61test();
                return;
            case 1:                               // TC REDOMANC: proceed
                redomanc();
                return;
            case 2:                               // TCF ENDMANU1: enter, i.e. finished with R60
                endmanu1();
                return;
            default:
                chklinus();                       // check for priority displays
                executive.endofjob();
        }
    }

    /** REDOMANC: recompute final angles (unless 3-axis flag) after PROCEED. */
    public void redomanc() {
        if ((agc.get("FLAGWRD5") & BIT6) == 0) {
            vecpoint();
            interp.store("CPHI");
            toballc();
            return;
        }
        toballc();
    }

    /** TOBALLC: in AUTO with CTL GNC the maneuver is started; otherwise return to TOBALL. */
    public void toballc() {
        // CAF PRIO30; RXOR CHAN31; MASK 13,14,15 : zero if switches are AUTO and CTL GNC
        int chan31 = agc.get("CHAN31");
        if (((PRIO30 ^ chan31) & 070000) != 0) {
            toball();                             // not AUTO
            return;
        }
        // set up non-flashing V06 N18
        displayGodspr(V06N18);
        chklinus();
        startmnv();
    }

    /** STARTMNV: BANKCALL GOMANUR; ENDMANUV: finished maneuver, go back to TOBALL. */
    public void startmnv() {
        gomanur();
        toball();                                 // ENDMANUV
    }

    /** ENDMANU1: reset the 3-axis flag and return to the caller of R60. */
    public void endmanu1() {
        agc.set("3AXISFLG", 0);                   // TC DOWNFLAG ADRES 3AXISFLG (bit 6 flag 5)
        agc.set("FLAGWRD5", agc.get("FLAGWRD5") & ~BIT6);
        if (returnTo != null) {
            returnTo.run();                       // CAE TEMPR60 ; TC BANKJUMP
        }
    }

    /** CHKLINUS: check for priority display; if the flag is set, restart-protect and call LINUS. */
    public void chklinus() {
        if ((~agc.get("FLAGWRD4") & BIT12 & Agc.WORD_MASK) == 0) {
            return;                               // priority display flag not set: exit
        }
        agc.set("MPAC+2", agc.get("Q"));          // save return
        agc.set("TBASE1", Agc.add(agc.get("BUF2"), Agc.fromSigned(-3)));   // location for restart
        phaschng(OCT71);                          // 1.7SPOT for RELINUS
        linus(BIT7);                              // go set bits for priority display
    }

    /** RELINUS: restart entry after LINUS. */
    public void relinus() {
        if ((agc.get("FLAGWRD1") & BIT5) == 0) {  // track flag off
            goredo20();
            return;
        }
        upflag("PDSPFLAG");                       // R60 PRIODSP flag
        upflag("TARG1FLG");                       // for R52
        agc.set("OPTIND", 0);                     // OPTIND was set to -1 by V379
        executive.priochng(PRIO14);               // restore original priority
        // TC TBASE1 : return into the interrupted display
    }

    /** GOREDO20. */
    public void goredo20() {
        phaschng(OCT111);                         // 1.11 for PIKUP20
        executive.endofjob();
    }

    /** R61TEST: decide what PROCEED/terminate means (P00 / R61 in P20 / otherwise). */
    public void r61test() {
        if (agc.get("MODREG") == 0) {
            endmanu1();                           // P00: V49 or V89; user does ENDEXT
            return;
        }
        if ((agc.get("FLAGWRD4") & BIT12) != 0) {
            gotov56();                            // in R61 (P20)
        } else {
            gotopooh();
        }
    }

    // =====================================================================
    // VECPOINT
    // =====================================================================

    /**
     * VECPOINT: point spacecraft axis SCAXIS (half-unit, spacecraft coords) along POINTVSM
     * (half-unit, stable member coords). Returns the desired outer, inner and middle gimbal angles in
     * MPAC, MPAC+1, MPAC+2 (2s complement). Uses SCAXIS, POINTVSM, MIS, DEL, COF, VECQTEMP.
     */
    public void vecpoint() {
        interp.bov();                              // BOV VECLEAR : clear OVFIND
        interp.axc(2, "MIS");
        interp.rtb(() -> readcduk());              // present CDU angles -> PD 25, 26, 27
        interp.stcall("PD+25", () -> cdutodcm());  // S/C axes to stable member axes (MIS)
        interp.vload("POINTVSM");                  // resolve pointing direction VF into initial S/C axes
        interp.vxm("MIS");
        interp.unit();
        interp.store("PD+28");                     // PD 28..33 = VF
        interp.vxv("SCAXIS");                      // VF x VI, VI = SCAXIS
        interp.unit();
        if (interp.bov()) {                        // BOV PICKAXIS
            pickaxis();
            return;
        }
        interp.comp();                             // VCOMP
        interp.stodl("COF", "LV");                 // magnitude of cross product in 36D (LV)
        interp.dsu("DPB-14");
        if (interp.bmn()) {                        // less than B-14: UNIT invalid
            pickaxis();
            return;
        }
        interp.vload("SCAXIS");
        interp.dot("PD+28");
        interp.sl1();
        interp.arccos();
        compmatx();
    }

    /** COMPMATX: transformation from final S/C axes to initial S/C axes (MFI), then MFS = MIS MFI. */
    private void compmatx() {
        delcomp();                                 // CALL DELCOMP
        interp.axc(1, "MIS");
        interp.axc(2, "DEL");
        mxm3();                                    // MFS = MIS MFI (in PD list)
        interp.dload("PD+6");                      // MFS6 = SIN(CPSI)
        interp.abs();
        interp.dsu("SINGIMLC");                    // sin(59 deg)
        if (interp.bmn()) {                        // |CPSI| less than 59 deg: not in gimbal lock
            findgimb();
            return;
        }
        interp.dload("SCAXIS");                    // are we pointing the thrust axis?
        interp.abs();
        interp.dsu("SINVEC1");                     // sin(49.4 deg)
        if (interp.bpl()) {                        // yes: trying to point into gimbal lock
            findgimb();
            return;
        }
        interp.vload(null);
        interp.stadr();
        interp.stovl("MIS+12D", null);
        interp.stadr();                            // store MFS (PD list) in MIS
        interp.stovl("MIS+6", null);
        interp.stadr();
        interp.stovl("MIS", "MIS+6");              // inner gimbal axis in final S/C axes
        boolean igPlus = interp.bpl();             // choose IG axis direction closest to final X axis
        if (!igPlus) {
            interp.comp();                         // VCOMP
        }
        // IGSAMEX: shortest way to rotate S/C out of gimbal lock about +-SCAXIS
        interp.vxv("SCAXIS");
        if (interp.bmn()) {
            useScaxisPositive();                   // U=SCAXIS
        } else {
            interp.vload("SCAXIS");
            interp.comp();                         // rotate about -SCAXIS
            interp.stcall("COF", () -> chekaxis());
        }
    }

    private void useScaxisPositive() {
        interp.vload("SCAXIS");
        interp.store("COF");                       // rotate about +SCAXIS
        chekaxis();
    }

    /** CHEKAXIS: pick 50 degrees (AOT) or 35 degrees (transponder / Y-Z plane vector). */
    private void chekaxis() {
        interp.dload("SCAXIS");
        interp.abs();
        interp.dsu("SINVEC2");                     // sin(29.5 deg)
        if (interp.bpl()) {
            interp.dload("VECANG1");               // 50 degrees about +-SCAXIS ($360)
        } else {
            interp.dload("VECANG2");               // 35 degrees ($360)
        }
        compmfsn();
    }

    /** COMPMFSN: rotation about SCAXIS to bring MFS out of gimbal lock. */
    private void compmfsn() {
        delcomp();
        interp.axc(1, "MIS");
        interp.axc(2, "DEL");
        mxm3();                                    // new transformation DESIRED S/C to stable member
        findgimb();
    }

    /** FINDGIMB: extract the commanded CDU angles from the matrix and convert to 2s complement. */
    private void findgimb() {
        interp.axc(1, 0);
        dcmtocdu();
        interp.rtb(() -> v1sto2s());
        interp.setpd(0);
        // GOTO VECQTEMP : return to caller
    }

    /** PICKAXIS: VF x VI = 0, so VF is parallel or anti-parallel to VI. */
    private void pickaxis() {
        interp.vload("PD+28");
        interp.dot("SCAXIS");
        if (interp.bmn()) {
            rot180();
            return;
        }
        interp.tload("PD+25");                     // VF = VI: desired CDU = present CDU
        // GOTO VECQTEMP
    }

    /** ROT180: anti-parallel; a 180 degree rotation is required about a perpendicular axis. */
    private void rot180() {
        interp.vload("MIS+6");                     // Y stable member axis in initial S/C axes
        interp.vxv("HIUNITX");
        interp.unit();
        interp.vxv("SCAXIS");                      // UNIT(VI x UNIT(Y(SM) x X(I)))
        interp.unit();
        if (interp.bov()) {
            pickx();
            return;
        }
        interp.stodl("COF", "LV");                 // check magnitude of this vector
        interp.dsu("DPB-14");
        if (interp.bmn()) {                        // less than B-14: pick X axis
            pickx();
            return;
        }
        interp.vload("COF");
        xrot();
    }

    private void pickx() {
        interp.vload("HIUNITX");
        xrot();
    }

    private void xrot() {
        interp.stodl("COF", "HIDPHALF");
        compmatx();
    }

    // =====================================================================
    // R62: initiating automatic maneuver via keyboard (V49)
    // =====================================================================

    /** R62DISP: display commanded ICDUs CPHI, CTHETA, CPSI (V06N22) with flash. */
    public void r62disp() {
        int r = goflash(V06N22);
        switch (r) {
            case 0: endext(); break;               // TERMINATE
            case 1: gomove(); break;               // PROCEED
            default: r62disp(); break;             // ENTER: astronaut may load new ICDUs
        }
    }

    /** GOMOVE: set the 3-axis flag and start R60CSM. */
    public void gomove() {
        agc.set("FLAGWRD5", agc.get("FLAGWRD5") | BIT6);   // UPFLAG 3AXISFLG
        r60csm(this::endext);
    }

    // ---- links to other source files ----

    private int displayGoperf2r(int vn) { return new Pinball(agc).goperf2r(vn); }
    private void displayGodspr(int vn) { new Pinball(agc).godspr(vn); }
    private int goflash(int vn) { return new Pinball(agc).goflash(vn); }
    private void gomanur() { new AutomaticManeuvers(agc).gomanur(); }
    private void phaschng(int code) { new RestartsRoutine(agc).phaschng(code); }
    private void linus(int bits) { new Pinball(agc).linus(bits); }
    private void upflag(String flag) { agc.set(flag, 1); }
    private void endext() { new ExtendedVerbs(agc).endext(); }
    private void gotov56() { new ExtendedVerbs(agc).gotov56(); }
    private void gotopooh() { new ExtendedVerbs(agc).gotopooh(); }
    private void readcduk() { new Rt8OpCodes(agc).readcduk(); }
    private void cdutodcm() { new Anglfind(agc).cdutodcm(); }
    private void delcomp() { new Anglfind(agc).delcomp(); }
    private void mxm3() { new Anglfind(agc).mxm3(); }
    private void dcmtocdu() { new Anglfind(agc).dcmtocdu(); }
    private void v1sto2s() { new Rt8OpCodes(agc).v1sto2s(); }

    /** Placeholder naming for the PINBALL display routines (GOPERF2R, GODSPR, GOFLASH, LINUS). */
    private static final class Pinball {
        Pinball(Agc agc) { }
        int goperf2r(int vn) { return 3; }
        void godspr(int vn) { }
        int goflash(int vn) { return 1; }
        void linus(int bits) { }
    }
}
