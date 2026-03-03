package Constants;

public final class LimelightConstants {
    private LimelightConstants() {}

    public static final int PIPELINE_APRILTAG = 0;
    public static final int PIPELINE_LONG_APRILTAG = 1;
    public static final int PIPELINE_LOCALIZATION = 2;  // MegaTag2 relocalization pipeline
    public static final int PIPELINE_GOAL_RED = 2;
    public static final int PIPELINE_GOAL_BLUE = 3;

    // Limelight returns meters, pinpoint uses inches
    public static final double METERS_TO_INCHES = 39.3701;
}
