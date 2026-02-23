package Constants;

public final class TurretConstants {
    private TurretConstants() {}

    // ==================== Odometry-Based Turret Geometry ====================

    // Turret offset from robot center (inches, robot-relative)
    public static double TURRET_OFFSET_X = 0.0;  // Centered on X
    public static double TURRET_OFFSET_Y = -1.0;  // 1" backward of center

    // Servo mapping
    public static final double SERVO_CENTER_POSITION = 0.5;
    public static final double GEAR_RATIO = 2.5;              // 2.5:1 gear reduction
    public static final double SERVO_RANGE_DEGREES = 355.0;   // Servo total range in degrees
// Total turret range = 355 / 2.5 = 142° (-71 to +71)
    public static final double DEGREES_PER_SERVO_UNIT = SERVO_RANGE_DEGREES / GEAR_RATIO; // 120.0

    // Hard stop limits (turret degrees, robot-relative)
    public static double HARD_STOP_CW = 68.5;    // Max right
    public static double HARD_STOP_CCW = -68.5;   // Max left

    // EMA smoothing factor (0 = no smoothing, 1 = no filtering)
    public static double SMOOTHING_ALPHA = 0.35;

    // Limelight Tx auto-correction constants
    public static double TX_CORRECTION_GAIN = -0.3;      // Degrees correction per degree Tx (kept low to avoid jitter)
    public static double TX_CORRECTION_DEADBAND = 0.5;    // Ignore Tx below this (degrees)
    public static double TX_CORRECTION_MAX = 5.0;         // Max correction magnitude (degrees)

    // Alliance-specific tracking offset (degrees) — tune on robot
    public static double BLUE_TURRET_TRACKING_OFFSET = 0.0;
    public static double RED_TURRET_TRACKING_OFFSET = -2.0;

    // ==================== Legacy Constants (used by AutonTemplate) ====================

    // Default servo position
    public static final double DEFAULT = 0.5;

    // Auto turret servo positions
    public static final double TURRET_POSE_AUTO = 0.8;
    public static final double TURRET_POSE_AUTO_LONG = 0.33;
    public static final double TURRET_BLUE_AUTO_POSE = 1;
    public static final double TURRET_RED_AUTO_POSE = 0.0;

    // Proportional tracking gains (used by auto Tx-based tracking)
    public static final double TELEOP_GAIN = 1.0 / 600;
    public static final double AUTO_ALIGN_GAIN = 1.0 / 1000;
    public static final double AUTO_LOOP_GAIN_RED = 1.0 / 800;

    // Tracking deadbands (degrees, used by auto)
    public static final double TRACKING_DEADBAND = 1.5;
    public static final double AUTO_ALIGN_DEADBAND = 2.0;

    // Auto-center timeout (seconds, used by auto)
    public static final double RESET_TIMEOUT_SEC = 1.0;
}
