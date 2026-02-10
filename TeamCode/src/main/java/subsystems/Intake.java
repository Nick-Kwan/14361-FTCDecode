package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import utility.RobotHardware;
import Constants.EnumConstants;
import Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final RobotHardware robot;
    private EnumConstants.IntakeState currentState = EnumConstants.IntakeState.Idle;

    public Intake() {
        this.robot = RobotHardware.getInstance();
    }

    public void startIntaking() {
        robot.intakeMotor.setPower(IntakeConstants.INTAKE_POWER);
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
        robot.intakeMotor.setPower(IntakeConstants.REVERSE_POWER);
        currentState = EnumConstants.IntakeState.Reversing;
    }

    public void deploy() {
        robot.intakeServo.setPosition(IntakeConstants.SERVO_DOWN);
    }

    public void retract() {
        robot.intakeServo.setPosition(IntakeConstants.SERVO_UP);
    }

    public EnumConstants.IntakeState getCurrentState() {
        return currentState;
    }

    @Override
    public void periodic() {
    }
}
