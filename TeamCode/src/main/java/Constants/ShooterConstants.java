package Constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterConstants {
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

    public static double hoodPosition = 0.5;
    public static double vel = 0.5;

    // ==================== Distance-Based LUT Data ====================
    // Estimated from 14361's existing Ty-based data + field geometry
    // TUNE ON ROBOT — these are starting estimates

    // Distance-based Velocity LUT: {distance_inches, velocity_power}
    public static double[][] VELOCITY_DATA = {
        {0,    0.48},
        {32,    0.48},   // Closest possible
        {40,   0.52},   // ~Ty=15 equivalent
        {60,   0.55},   // ~Ty=10 equivalent
        {80,   0.58},   // ~Ty=5 equivalent
        {100,  0.615},   // ~Ty=0 equivalent
        {110,  0.638},   // ~Ty=-1.2 equivalent
        {120,  0.66},   // ~Ty=-2.7 equivalent
        {130,  0.69},   // Fallback for extreme distances
        {140,  0.72},   // ~Ty=0 equivalent
        {150,  0.75},   // ~Ty=-1.2 equivalent
        {160,  0.77},   // ~Ty=-2.7 equivalent
        {200,  0.84},   // Fallback for extreme distances
        {300,  0.84}
    };

    // Distance-based Hood Angle LUT: {distance_inches, hood_servo_position}
    public static double[][] HOOD_DATA = {
        {0,    1},
        {32,    1},
        {40,   0.85},
        {60,   0.55},
        {80,   0.56},
        {100,  0.5},
        {110,  0.47},
        {120,  0.45},
        {130,  0.4},
        {140,  0.33},   // ~Ty=0 equivalent
        {150,  0.315},   // ~Ty=-1.2 equivalent
        {160,  0.27},   // ~Ty=-2.7 equivalent
        {200,  0.26},   // Fallback for extreme distances
        {300,  0.26}
    };
}
