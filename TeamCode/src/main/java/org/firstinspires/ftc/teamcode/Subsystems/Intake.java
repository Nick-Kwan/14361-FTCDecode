package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.command.Subsystem;

public class Intake implements Subsystem{
    private RobotHardware robot;
    public Intake() {
        this.robot = RobotHardware.getInstance();
    }

    public void startIntaking(){
        robot.intakeMotor.setPower(RobotConstants.Intake.intakeMotorON);
    }

    public void stopIntaking(){
        robot.intakeMotor.setPower(RobotConstants.Intake.intakeMotorOff);
    }

    public void reverseIntaking() {
        robot.intakeMotor.setPower(RobotConstants.Intake.intakeMotorReverse);
    }

    public void intakeDown(){
        robot.intakeServo.setPosition(RobotConstants.Intake.intakeServoDown);
    }

    public void intakeUp(){
        robot.intakeServo.setPosition(RobotConstants.Intake.intakeServoUp);
    }
}
