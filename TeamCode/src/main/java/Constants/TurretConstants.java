package Constants;

public final class TurretConstants {
    private TurretConstants() {}

    // Default position
    public static final double DEFAULT = 0.5;

    // Auto turret positions
    public static final double TURRET_POSE_AUTO = 0.8;
    public static final double TURRET_POSE_AUTO_LONG = 0.33;
    public static final double TURRET_BLUE_AUTO_POSE = 1;
    public static final double TURRET_RED_AUTO_POSE = 0.0;
    public static final double TURRET_MEOW_POSE = 0.3;

    // Proportional tracking gains
    public static final double TELEOP_GAIN = 1.0 / 600;
    public static final double AUTO_ALIGN_GAIN = 1.0 / 1000;
    public static final double AUTO_LOOP_GAIN_RED = 1.0 / 800;

    // Tracking deadbands (degrees)
    public static final double TRACKING_DEADBAND = 1.5;
    public static final double AUTO_ALIGN_DEADBAND = 2.0;

    // Auto-center timeout (seconds)
    public static final double RESET_TIMEOUT_SEC = 1.0;
}
