package Constants;

public final class SpindexerConstants {
    private SpindexerConstants() {}

    // Spindexer servo positions (3 fixed positions)
    public static final double POSE_ONE = 0.035;
    public static final double POSE_TWO = 0.42;
    public static final double POSE_THREE = 0.8;

    // Linkage servo positions
    public static final double LINKAGE_DOWN = 0.67;
    public static final double LINKAGE_UP = 0;

    // Fire state machine timing (ms)
    public static final long FIRE_TIME_MS = 200;
    public static final long RETRACT_TIME_MS = 150;
    public static final long ROTATION_SETTLE_MS = 275;
    public static final long ROTATION_LONG_SETTLE_MS = 400;

    // Color sensor thresholds (RGB sum)
    public static final double BALL_PRESENT = 2000;
    public static final double BALL_ABSENT = 1500;

    // Auto-intake distribution interval (ms)
    public static final long AUTO_INTAKE_INTERVAL_MS = 300;

}
