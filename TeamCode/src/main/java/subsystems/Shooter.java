package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import utility.RobotHardware;
import Constants.ShooterConstants;

import java.util.List;
import java.util.Map;
import java.util.TreeMap;

public class Shooter extends SubsystemBase {
    private final RobotHardware robot;
    private final TreeMap<Double, ShotConfig> lut = new TreeMap<>();

    public Shooter() {
        this.robot = RobotHardware.getInstance();
        initLUT();
    }

    /** Shot configuration with lerp interpolation (replaces old shotConfig.java) */
    public static class ShotConfig {
        public final double rpm;
        public final double hoodPos;
        public final double turretOffset;

        public ShotConfig(double rpm, double hoodPos, double turretOffset) {
            this.rpm = rpm;
            this.hoodPos = hoodPos;
            this.turretOffset = turretOffset;
        }

        public static ShotConfig lerp(ShotConfig a, ShotConfig b, double t) {
            return new ShotConfig(
                a.rpm + (b.rpm - a.rpm) * t,
                a.hoodPos + (b.hoodPos - a.hoodPos) * t,
                a.turretOffset + (b.turretOffset - a.turretOffset) * t
            );
        }
    }

    private void initLUT() {
        for (double[] entry : ShooterConstants.LUT_DATA) {
            lut.put(entry[0], new ShotConfig(entry[1], entry[2], entry[3]));
        }
    }

    /** Look up shot configuration for a given Ty value with lerp interpolation */
    public ShotConfig getConfigForTargetY(double y) {
        Map.Entry<Double, ShotConfig> floor = lut.floorEntry(y);
        Map.Entry<Double, ShotConfig> ceil = lut.ceilingEntry(y);

        if (floor == null) return ceil.getValue();
        if (ceil == null) return floor.getValue();
        if (floor.getKey().equals(ceil.getKey())) return floor.getValue();

        double d0 = floor.getKey();
        double d1 = ceil.getKey();
        double t = (y - d0) / (d1 - d0);

        return ShotConfig.lerp(floor.getValue(), ceil.getValue(), t);
    }

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

    /** Configure shooter from LUT based on Limelight Ty, then apply velocity + hood */
    public void prepareForShot(double targetY) {
        ShotConfig config = getConfigForTargetY(targetY);
        setVelocity(config.rpm);
        setHoodAngle(config.hoodPos);
    }

    /** Demo LUT shoot - only adjusts hood angle (for judging demo, no velocity change) */
    public void prepareForShotHoodOnly(double targetY) {
        ShotConfig config = getConfigForTargetY(targetY);
        setHoodAngle(config.hoodPos);
        // No velocity change - for demo purposes
    }

    /** Get current flywheel velocities (list of [motor2, motor1] speeds) */
    public List<Double> getVelocities() {
        return robot.shooterMotors.getVelocities();
    }

    /** Configure PID for teleop mode (P=3, V=0.7 feedforward) */
    public void configureForTeleOp() {
        robot.shooterMotors.setVeloCoefficients(
            ShooterConstants.SHOOT_P,
            ShooterConstants.SHOOT_I,
            ShooterConstants.SHOOT_D
        );
        robot.shooterMotors.setFeedforwardCoefficients(0, ShooterConstants.SHOOT_V);
    }

    /** Configure PID for autonomous mode (P=0.4) */
    public void configureForAuto() {
        robot.shooterMotors.setVeloCoefficients(
            ShooterConstants.AUTO_VELO_P,
            ShooterConstants.SHOOT_I,
            ShooterConstants.SHOOT_D
        );
    }

    @Override
    public void periodic() {
    }
}
