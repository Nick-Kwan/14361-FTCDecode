package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import utility.RobotHardware;
import Constants.EnumConstants;

public class Intake extends SubsystemBase {
    private final RobotHardware robot;
    private EnumConstants.IntakeState currentState = EnumConstants.IntakeState.Idle;

    // Motor power constants (from old RobotConstants.Intake)
    private static final double INTAKE_POWER = 0.85;
    private static final double REVERSE_POWER = -1.0;

    // Servo positions (from old RobotConstants.Intake)
    private static final double SERVO_DOWN = 1.0;
    private static final double SERVO_UP = 0.25;

    public Intake() {
        this.robot = RobotHardware.getInstance();
    }

    public void startIntaking() {
        robot.intakeMotor.setPower(INTAKE_POWER);
        currentState = EnumConstants.IntakeState.Intaking;
    }

    public void startIntakingMax() {
        robot.intakeMotor.setPower(1.0);
        currentState = EnumConstants.IntakeState.Intaking;
    }

    public void stopIntaking() {
        robot.intakeMotor.setPower(0);
        currentState = EnumConstants.IntakeState.Idle;
    }

    public void reverseIntaking() {
        robot.intakeMotor.setPower(REVERSE_POWER);
        currentState = EnumConstants.IntakeState.Reversing;
    }

    public void deploy() {
        robot.intakeServo.setPosition(SERVO_DOWN);
    }

    public void retract() {
        robot.intakeServo.setPosition(SERVO_UP);
    }

    public EnumConstants.IntakeState getCurrentState() {
        return currentState;
    }

    @Override
    public void periodic() {
    }
}
