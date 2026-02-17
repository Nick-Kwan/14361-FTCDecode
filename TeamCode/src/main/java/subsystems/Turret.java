package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import utility.RobotHardware;
import Constants.TurretConstants;

public class Turret extends SubsystemBase {
    private final RobotHardware robot;
    private double currentPosition = TurretConstants.DEFAULT;

    public Turret() {
        this.robot = RobotHardware.getInstance();
    }

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

    /**
     * Apply proportional tracking adjustment based on Limelight Tx.
     * Preserves exact behavior from TeleOpBlue: position += Tx * gain when |Tx| > deadband.
     *
     * @param tx Limelight horizontal offset (degrees)
     * @param gain proportional gain (e.g. TELEOP_GAIN = 1/600)
     * @param deadband minimum |Tx| to trigger adjustment (e.g. 1.5 degrees)
     */
    public void trackTarget(double tx, double gain, double deadband) {
        if (Math.abs(tx) > deadband) {
            setPosition(currentPosition + tx * gain);
        }
    }

    @Override
    public void periodic() {
    }
}
