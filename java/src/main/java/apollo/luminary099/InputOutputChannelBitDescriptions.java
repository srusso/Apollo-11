package apollo.luminary099;

/**
 * Input/output channel bit descriptions for Luminary 099 (LM AGC). Channel numbers are octal in the
 * original; bit masks are given as (1 << (bit-1)). Input channels 30-33 are inverted as sensed by software
 * (zero means the signal is present).
 */
public final class InputOutputChannelBitDescriptions {
    private InputOutputChannelBitDescriptions() { }

    private static int bit(int n) { return 1 << (n - 1); }

    // Channel numbers (octal in source)
    public static final int L = 01, Q = 02;
    /** HISCALAR: input, high 14 bits of 33-stage counter (B23 csec, LSB 5.12 s). */
    public static final int HISCALAR = 03;
    /** LOSCALAR: input, next 14 bits (B9 csec, LSB 1/3200 s). */
    public static final int LOSCALAR = 04;
    /** PYJETS: pitch RCS jet control (bits 1-8). */
    public static final int PYJETS = 05;
    /** ROLLJETS: roll RCS jet control (bits 1-8). */
    public static final int ROLLJETS = 06;
    /** SUPERBNK: fixed extension bits (bits 5-7), not reset by restart. */
    public static final int SUPERBNK = 07;
    /** OUT0: display system latching relays; bits 15-12 row number, bits 11-1 settings. */
    public static final int OUT0 = 010;
    /** DSALMOUT: engine on/off and indicator lamps. */
    public static final int DSALMOUT = 011;
    public static final int CHAN12 = 012, CHAN13 = 013, CHAN14 = 014;
    /** MNKEYIN: DSKY key code input (bits 5-1). */
    public static final int MNKEYIN = 015;
    /** NAVKEYIN: optics mark / thrust control input (bits 3-7). */
    public static final int NAVKEYIN = 016;
    public static final int CHAN30 = 030, CHAN31 = 031, CHAN32 = 032, CHAN33 = 033;
    /** DNT M1 / M2: downlink serialization words. */
    public static final int DNTM1 = 034, DNTM2 = 035;

    // Channel 11 (DSALMOUT)
    public static final int DSALM_ISS_WARNING = bit(1), DSALM_COMP_ACTY = bit(2), DSALM_UPLINK_ACTY = bit(3),
            DSALM_TEMP_CAUTION = bit(4), DSALM_KEY_REL = bit(5), DSALM_FLASH_VN = bit(6),
            DSALM_OPER_ERR = bit(7), DSALM_TEST_CONNECTOR = bit(9), DSALM_CAUTION_RESET = bit(10),
            DSALM_ENGINE_ON = bit(13), DSALM_ENGINE_OFF = bit(14);

    // Channel 12
    public static final int C12_ZERO_RR_CDU = bit(1), C12_ENABLE_RR_CDU_ERR = bit(2), C12_COARSE_ALIGN = bit(4),
            C12_ZERO_IMU_CDU = bit(5), C12_ENABLE_IMU_ERR = bit(6), C12_DISPLAY_INERTIAL = bit(8),
            C12_MINUS_PITCH_TRIM = bit(9), C12_PLUS_PITCH_TRIM = bit(10), C12_MINUS_ROLL_TRIM = bit(11),
            C12_PLUS_ROLL_TRIM = bit(12), C12_LR_POS2 = bit(13), C12_RR_LOCKON_ENABLE = bit(14),
            C12_ISS_TURNON_DELAY_COMPLETE = bit(15);

    // Channel 13
    public static final int C13_RADAR_C = bit(1), C13_RADAR_B = bit(2), C13_RADAR_A = bit(3),
            C13_RADAR_ACTIVITY = bit(4), C13_BLOCK_UPLINK = bit(6), C13_DOWNLINK_WORD_ORDER = bit(7),
            C13_RHC_COUNTER_ENABLE = bit(8), C13_RHC_READ_START = bit(9), C13_TEST_ALARMS = bit(10),
            C13_ENABLE_STANDBY = bit(11), C13_RESET_TRAP_31A = bit(12), C13_RESET_TRAP_31B = bit(13),
            C13_RESET_TRAP_32 = bit(14), C13_ENABLE_T6RUPT = bit(15);

    // Channel 14
    public static final int C14_OUTLINK_ACTY = bit(1), C14_ALT_RATE_SELECT = bit(2), C14_ALT_METER_ACTY = bit(3),
            C14_THRUST_DRIVE_ACTY = bit(4), C14_GYRO_ENABLE = bit(6), C14_GYRO_SEL_B = bit(7),
            C14_GYRO_SEL_A = bit(8), C14_GYRO_NEGATIVE = bit(9), C14_GYRO_ACTIVITY = bit(10),
            C14_DRIVE_CDU_S = bit(11), C14_DRIVE_CDU_T = bit(12), C14_DRIVE_CDU_Z = bit(13),
            C14_DRIVE_CDU_Y = bit(14), C14_DRIVE_CDU_X = bit(15);

    // Channel 16 (NAVKEYIN)
    public static final int C16_OPTICS_X_MARK = bit(3), C16_OPTICS_Y_MARK = bit(4),
            C16_MARK_REJECT = bit(5), C16_DESCENT_PLUS = bit(6), C16_DESCENT_MINUS = bit(7);

    // Channel 30 (inverted)
    public static final int C30_ABORT_DESCENT = bit(1), C30_ENGINE_ARMED = bit(3), C30_ABORT_ASCENT = bit(4),
            C30_AUTO_THROTTLE = bit(5), C30_DISPLAY_INERTIAL = bit(6), C30_RR_CDU_FAIL = bit(7),
            C30_IMU_OPERATE = bit(9), C30_LGC_CONTROL = bit(10), C30_IMU_CAGE = bit(11),
            C30_IMU_CDU_FAIL = bit(12), C30_IMU_FAIL = bit(13), C30_ISS_TURNON_REQ = bit(14),
            C30_IMU_TEMP_OK = bit(15);

    // Channel 31 (inverted): attitude/translation controllers
    public static final int C31_PLUS_PITCH = bit(1), C31_MINUS_PITCH = bit(2), C31_PLUS_YAW = bit(3),
            C31_MINUS_YAW = bit(4), C31_PLUS_ROLL = bit(5), C31_MINUS_ROLL = bit(6),
            C31_PLUS_X = bit(7), C31_MINUS_X = bit(8), C31_PLUS_Y = bit(9), C31_MINUS_Y = bit(10),
            C31_PLUS_Z = bit(11), C31_MINUS_Z = bit(12), C31_ATT_HOLD = bit(13), C31_AUTO_STAB = bit(14),
            C31_OUT_OF_DETENT = bit(15);

    // Channel 32 (inverted): thruster-pair disables; bit 14 = PROCEED key
    public static final int C32_THR_2_4 = bit(1), C32_THR_5_8 = bit(2), C32_THR_1_3 = bit(3),
            C32_THR_6_7 = bit(4), C32_THR_14_16 = bit(5), C32_THR_13_15 = bit(6), C32_THR_9_12 = bit(7),
            C32_THR_10_11 = bit(8), C32_DESCENT_GIMBALS_DISABLED = bit(9),
            C32_DESCENT_GIMBAL_FAILURE = bit(10), C32_PROCEED_KEY = bit(14);

    // Channel 33 (inverted)
    public static final int C33_RR_AUTO_POWER = bit(2), C33_RR_RANGE_LOW = bit(3), C33_RR_DATA_GOOD = bit(4),
            C33_LR_RANGE_GOOD = bit(5), C33_LR_POS1 = bit(6), C33_LR_POS2 = bit(7), C33_LR_VEL_GOOD = bit(8),
            C33_LR_RANGE_LOW = bit(9), C33_BLOCK_UPLINK = bit(10), C33_UPLINK_TOO_FAST = bit(11),
            C33_DOWNLINK_TOO_FAST = bit(12), C33_PIPA_FAIL = bit(13), C33_REPEATED_ALARMS = bit(14),
            C33_OSCILLATOR_STOPPED = bit(15);
}
