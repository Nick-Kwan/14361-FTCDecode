package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import Constants.EnumConstants;
import Constants.FieldMap;
import Constants.TurretConstants;
import com.pedropathing.geometry.Pose;

import utility.RobotHardware;

public class Turret extends SubsystemBase {
    private final RobotHardware robot;
    private double currentPosition = TurretConstants.DEFAULT;

    // Odometry-based tracking state
    private boolean trackingEnabled = false;
    private boolean targetOutOfRange = false;
    private double degreesOutOfRange = 0;
    private double currentTargetDegrees = 0;

    // EMA smoothing state
    private double smoothedTargetDegrees = 0;
    private boolean smoothingInitialized = false;

    // Limelight Tx correction
    private Limelight limelight;
    private boolean txCorrectionEnabled = false;
    private double lastTxCorrection = 0;

    // Pre-computed turret offset in polar form for efficiency
    private static final double TURRET_OFFSET_MAG = Math.sqrt(
            TurretConstants.TURRET_OFFSET_X * TurretConstants.TURRET_OFFSET_X +
            TurretConstants.TURRET_OFFSET_Y * TurretConstants.TURRET_OFFSET_Y);
    private static final double TURRET_OFFSET_ANGLE = Math.atan2(
            TurretConstants.TURRET_OFFSET_Y, TurretConstants.TURRET_OFFSET_X);

    // Reusable array to avoid allocation
    private final double[] turretFieldPos = new double[2];

    public Turret() {
        this.robot = RobotHardware.getInstance();
    }

    // ==================== Odometry-Based Tracking ====================

    /** Enable/disable odometry-based auto-tracking in periodic() */
    public void setTrackingEnabled(boolean enabled) {
        this.trackingEnabled = enabled;
        if (enabled) {
            smoothingInitialized = false; // Reset smoothing on enable
        }
    }

    public boolean isTrackingEnabled() {
        return trackingEnabled;
    }

    /** Set the Limelight reference for Tx correction */
    public void setLimelight(Limelight limelight) {
        this.limelight = limelight;
    }

    /** Enable/disable Limelight Tx correction on top of odometry tracking */
    public void setTxCorrectionEnabled(boolean enabled) {
        this.txCorrectionEnabled = enabled;
    }

    /** Get the last applied Tx correction value (for telemetry) */
    public double getLastTxCorrection() {
        return lastTxCorrection;
    }

    /** Compute clamped Tx correction with deadband, gain, and max clamp */
    private double getTxCorrection(double tx) {
        if (Math.abs(tx) < TurretConstants.TX_CORRECTION_DEADBAND) {
            lastTxCorrection = 0;
            return 0;
        }
        double correction = tx * TurretConstants.TX_CORRECTION_GAIN;
        correction = Math.max(-TurretConstants.TX_CORRECTION_MAX,
                     Math.min(TurretConstants.TX_CORRECTION_MAX, correction));
        lastTxCorrection = correction;
        return correction;
    }

    /**
     * Get the turret's position in field coordinates, accounting for
     * the turret's offset from the robot center.
     */
    public double[] getTurretFieldPosition() {
        double robotHeading = robot.cachedHeading;
        double combinedAngle = robotHeading + TURRET_OFFSET_ANGLE;
        turretFieldPos[0] = robot.cachedPoseX + TURRET_OFFSET_MAG * Math.sin(combinedAngle);
        turretFieldPos[1] = robot.cachedPoseY - TURRET_OFFSET_MAG * Math.cos(combinedAngle);
        return turretFieldPos;
    }

    /**
     * Core tracking calculation: compute the turret angle (in degrees, robot-relative)
     * needed to point at the goal from the current robot position.
     */
    public double getDegreesToGoal() {
        Pose goalPosition = FieldMap.getGoalPosition();
        double[] turretPos = getTurretFieldPosition();

        // Vector from turret to goal
        double deltaX = goalPosition.getX() - turretPos[0];
        double deltaY = goalPosition.getY() - turretPos[1];

        // Field angle to goal using atan2
        double fieldAngleRad = Math.atan2(deltaY, deltaX);

        // Convert to robot-relative by subtracting robot heading
        double turretAngleRad = fieldAngleRad - robot.cachedHeading;

        // Convert to degrees and normalize to [-180, 180]
        double turretAngleDeg = Math.toDegrees(turretAngleRad);
        turretAngleDeg = normalizeAngle(turretAngleDeg);

        // Apply alliance-specific calibration offset
        turretAngleDeg += (FieldMap.allianceColor == EnumConstants.AllianceColor.Blue)
                ? TurretConstants.BLUE_TURRET_TRACKING_OFFSET
                : TurretConstants.RED_TURRET_TRACKING_OFFSET;

        // Check hard limits
        if (turretAngleDeg > TurretConstants.HARD_STOP_CW) {
            targetOutOfRange = true;
            degreesOutOfRange = turretAngleDeg - TurretConstants.HARD_STOP_CW;
        } else if (turretAngleDeg < TurretConstants.HARD_STOP_CCW) {
            targetOutOfRange = true;
            degreesOutOfRange = TurretConstants.HARD_STOP_CCW - turretAngleDeg;
        } else {
            targetOutOfRange = false;
            degreesOutOfRange = 0;
        }

        return turretAngleDeg;
    }

    /**
     * Convert turret angle in degrees to servo position.
     * 14361: servo 0.5 = 0°, servo range is 142° total.
     * servoPos = 0.5 + degrees/142, clamped to [0, 1].
     */
    private double turretDegreesToServoPosition(double turretDegrees) {
        double position = TurretConstants.SERVO_CENTER_POSITION
                - (turretDegrees / TurretConstants.DEGREES_PER_SERVO_UNIT);
        return Math.max(0.0, Math.min(1.0, position));
    }

    /**
     * Set the turret to a target angle (in degrees, robot-relative) with EMA smoothing.
     * Clamps to hard stop limits before smoothing.
     */
    public void setTurretAngle(double targetAngleDeg) {
        targetAngleDeg = Math.max(TurretConstants.HARD_STOP_CCW,
                Math.min(TurretConstants.HARD_STOP_CW, targetAngleDeg));

        // Apply EMA smoothing
        if (!smoothingInitialized) {
            smoothedTargetDegrees = targetAngleDeg;
            smoothingInitialized = true;
        } else {
            smoothedTargetDegrees = TurretConstants.SMOOTHING_ALPHA * targetAngleDeg
                    + (1.0 - TurretConstants.SMOOTHING_ALPHA) * smoothedTargetDegrees;
        }

        currentTargetDegrees = smoothedTargetDegrees;
        double servoPos = turretDegreesToServoPosition(currentTargetDegrees);
        setPosition(servoPos);
    }

    /** Normalize angle to [-180, 180] */
    private static double normalizeAngle(double degrees) {
        degrees = degrees % 360;
        if (degrees > 180) degrees -= 360;
        if (degrees < -180) degrees += 360;
        return degrees;
    }

    // ==================== Direct Servo Control ====================

    /** Set turret servo position directly, clamped to 0-1 range */
    public void setPosition(double position) {
        this.currentPosition = Math.max(0.0, Math.min(1.0, position));
        robot.turretServo.setPosition(currentPosition);
    }

    /** Get the last commanded turret position */
    public double getPosition() {
        return currentPosition;
    }

    /** Center the turret to default position (0.5) */
    public void center() {
        setPosition(TurretConstants.DEFAULT);
    }

    /** Get current target angle in degrees (for telemetry) */
    public double getCurrentTargetDegrees() {
        return currentTargetDegrees;
    }

    /** Whether the target is outside the turret's range of motion */
    public boolean isTargetOutOfRange() {
        return targetOutOfRange;
    }

    public double getDegreesOutOfRange() {
        return degreesOutOfRange;
    }

    // ==================== Legacy Tx-Based Tracking (for Auto) ====================

    /**
     * Apply proportional tracking adjustment based on Limelight Tx.
     * Used by AutonTemplate for Tx-based tracking during autonomous.
     */
    public void trackTarget(double tx, double gain, double deadband) {
        if (Math.abs(tx) > deadband) {
            setPosition(currentPosition + tx * gain);
        }
    }

    // ==================== Periodic ====================

    @Override
    public void periodic() {
        if (trackingEnabled) {
            double degreesToGoal = getDegreesToGoal();

            // Apply Limelight Tx correction if available
            if (txCorrectionEnabled && limelight != null && limelight.isValid()) {
                degreesToGoal += getTxCorrection(limelight.getTx());
            }

            // Clamp to hard stops before passing to setTurretAngle
            degreesToGoal = Math.max(TurretConstants.HARD_STOP_CCW,
                            Math.min(TurretConstants.HARD_STOP_CW, degreesToGoal));

            setTurretAngle(degreesToGoal);
        }
    }
}
