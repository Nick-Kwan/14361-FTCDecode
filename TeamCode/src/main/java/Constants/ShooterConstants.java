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
    // Updated from 0.4 to 0.63 in hkhk commit
    public static final double AUTO_VELO_P = 0.63;

    // Hood positions
    public static final double HOOD_POSE_MID = 0.5;
    public static final double HOOD_POSE_AUTO = 0.5;
    public static final double HOOD_POSE_LONG = 0.21;

    // ShooterLUT data points: {targetY, rpm, hoodPos, turretOffset}
    // Updated from Kwan "hkhk" commit (d32cdda) - 6 data points for improved accuracy
    public static final double[][] LUT_DATA = {
        {15.0, 0.55, 0.6,   0.0},
        {10.0, 0.57, 0.52,  0.0},  // NEW
        { 5.0, 0.62, 0.435, 0.0},  // UPDATED
        { 0.0, 0.72, 0.23,  0.0},  // UPDATED
        {-1.2, 0.76, 0.22,  0.0},  // NEW
        {-2.7, 0.82, 0.2,   0.0}
    };

}
