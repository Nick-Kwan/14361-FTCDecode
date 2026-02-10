package Constants;

public final class ShooterConstants {
    private ShooterConstants() {}

    // Velocity presets
    public static final double SHOOTER_AUTO = -0.57;
    public static final double SHOOTER_SHORT_AUTO = -0.4;

    // PID coefficients (TeleOp)
    public static final double SHOOT_P = 3;
    public static final double SHOOT_I = 0;
    public static final double SHOOT_D = 0;
    public static final double SHOOT_V = 0.7;

    // PID coefficients (Auto)
    public static final double AUTO_VELO_P = 0.4;

    // Hood positions
    public static final double HOOD_POSE_MID = 0.5;
    public static final double HOOD_POSE_AUTO = 0.5;
    public static final double HOOD_POSE_LONG = 0.21;

    // ShooterLUT data points: {targetY, rpm, hoodPos, turretOffset}
    public static final double[][] LUT_DATA = {
        {15.0, 0.55, 0.6, 0.0},
        { 5.0, 0.6,  0.45, 0.0},
        { 0.0, 0.74, 0.27, 0.0},
        {-2.7, 0.82, 0.2,  0.0}
    };

}
