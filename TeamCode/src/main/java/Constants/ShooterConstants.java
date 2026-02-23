package Constants;

public final class ShooterConstants {
    private ShooterConstants() {}

    // Velocity presets (for autonomous fixed-power shots)
    public static final double SHOOTER_AUTO = -0.57;
    public static final double SHOOTER_SHORT_AUTO = -0.4;

    // PID coefficients (TeleOp)
    public static final double SHOOT_P = 3;
    public static final double SHOOT_I = 0;
    public static final double SHOOT_D = 0;
    public static final double SHOOT_V = 0.7;

    // PID coefficients (Auto)
    public static final double AUTO_VELO_P = 0.63;

    // Hood positions (for fixed-position presets)
    public static final double HOOD_POSE_MID = 0.5;
    public static final double HOOD_POSE_AUTO = 0.5;
    public static final double HOOD_POSE_LONG = 0.21;

    // Default distance when turret reference is unavailable
    public static final double DEFAULT_DISTANCE = 80.0;

    // Max distance to feed into LUT (must be within LUT bounds)
    public static final double MAX_LUT_DISTANCE = 199.0;

    // ==================== Distance-Based LUT Data ====================
    // Estimated from 14361's existing Ty-based data + field geometry
    // TUNE ON ROBOT — these are starting estimates

    // Distance-based Velocity LUT: {distance_inches, velocity_power}
    // Increased by 20 for now
    public static double[][] VELOCITY_DATA = {
        {20,    0.55},   // Closest possible
        {60,   0.55},   // ~Ty=15 equivalent
        {80,   0.57},   // ~Ty=10 equivalent
        {100,   0.62},   // ~Ty=5 equivalent
        {120,  0.72},   // ~Ty=0 equivalent
        {135,  0.76},   // ~Ty=-1.2 equivalent
        {150,  0.82},   // ~Ty=-2.7 equivalent
        {200,  0.82},   // Fallback for extreme distances
    };

    // Distance-based Hood Angle LUT: {distance_inches, hood_servo_position}
    public static double[][] HOOD_DATA = {
        {20,    0.6},
        {60,   0.6},
        {80,   0.52},
        {100,   0.435},
        {120,  0.23},
        {135,  0.22},
        {150,  0.2},
        {200,  0.2},
    };
}
