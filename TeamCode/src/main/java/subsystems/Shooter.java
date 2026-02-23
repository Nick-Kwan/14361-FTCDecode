package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.pedropathing.geometry.Pose;

import java.util.List;

import Constants.FieldMap;
import Constants.ShooterConstants;
import utility.RobotHardware;

public class Shooter extends SubsystemBase {
    private final RobotHardware robot;
    private InterpLUT velocityLUT;
    private InterpLUT hoodLUT;

    // Reference to turret for distance calculation
    private Turret turret;

    // Whether distance-based auto-aim is active
    private boolean autoAimEnabled = false;

    // Cached distance for telemetry
    private double lastDistance = 0;

    public Shooter() {
        this.robot = RobotHardware.getInstance();
        initializeLUTs();
    }

    /** Set turret reference for distance calculations */
    public void setTurret(Turret turret) {
        this.turret = turret;
    }

    // ==================== LUT Initialization ====================

    private void initializeLUTs() {
        velocityLUT = new InterpLUT();
        for (double[] entry : ShooterConstants.VELOCITY_DATA) {
            velocityLUT.add(entry[0], entry[1]);
        }
        velocityLUT.createLUT();

        hoodLUT = new InterpLUT();
        for (double[] entry : ShooterConstants.HOOD_DATA) {
            hoodLUT.add(entry[0], entry[1]);
        }
        hoodLUT.createLUT();
    }

    // ==================== Distance Calculation ====================

    /** Calculate Euclidean distance from turret field position to goal */
    public double getDistanceToTarget() {
        if (turret == null) {
            return ShooterConstants.DEFAULT_DISTANCE;
        }

        double[] turretPos = turret.getTurretFieldPosition();
        Pose goalPosition = FieldMap.getGoalPosition();

        double deltaX = goalPosition.getX() - turretPos[0];
        double deltaY = goalPosition.getY() - turretPos[1];

        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    // ==================== Distance-Based Auto-Aim ====================

    /** Enable/disable distance-based auto-aim in periodic() */
    public void setAutoAimEnabled(boolean enabled) {
        this.autoAimEnabled = enabled;
    }

    /** Update velocity and hood from distance-based LUT */
    public void updateFromDistance() {
        double distance = Math.min(getDistanceToTarget(), ShooterConstants.MAX_LUT_DISTANCE);
        lastDistance = distance;
        setVelocity(velocityLUT.get(distance));
        setHoodAngle(hoodLUT.get(distance));
    }

    /** Get last calculated distance (for telemetry) */
    public double getLastDistance() {
        return lastDistance;
    }

    // ==================== Direct Control ====================

    /** Set flywheel velocity via MotorGroup (VelocityControl mode) */
    public void setVelocity(double velocity) {
        robot.shooterMotors.set(velocity);
    }

    /** Stop the flywheels */
    public void stopShooter() {
        robot.shooterMotors.set(0);
    }

    /** Set hood servo position directly (0-1 range) */
    public void setHoodAngle(double position) {
        robot.adjustableHoodServo.setPosition(position);
    }

    /** Get current flywheel velocities */
    public List<Double> getVelocities() {
        return robot.shooterMotors.getVelocities();
    }

    // ==================== PID Configuration ====================

    /** Configure PID for teleop mode (P=3, V=0.7 feedforward) */
    public void configureForTeleOp() {
        robot.shooterMotors.setVeloCoefficients(
            ShooterConstants.SHOOT_P,
            ShooterConstants.SHOOT_I,
            ShooterConstants.SHOOT_D
        );
        robot.shooterMotors.setFeedforwardCoefficients(0, ShooterConstants.SHOOT_V);
    }

    /** Configure PID for autonomous mode */
    public void configureForAuto() {
        robot.shooterMotors.setVeloCoefficients(
            ShooterConstants.AUTO_VELO_P,
            ShooterConstants.SHOOT_I,
            ShooterConstants.SHOOT_D
        );
    }

    // ==================== Periodic ====================

    @Override
    public void periodic() {
        if (autoAimEnabled) {
            updateFromDistance();
        }
    }
}
