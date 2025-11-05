package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.util.RobotConstants;

import com.arcrobotics.ftclib.command.Subsystem;

public class Spindexer implements Subsystem{
    private RobotHardware robot;

    public Spindexer(){
        this.robot = RobotHardware.getInstance();
    }

    public enum DetectedColor{
        RED,
        BLUE,
        YELLOW,
        UNKOWN
    }
    public void spindexerUp() {
        robot.spindexerLinkageServo.setPosition(RobotConstants.Spindexer.spindexerLinkageServoUp);
    }
    public void spindexerDown() {
        robot.spindexerLinkageServo.setPosition(RobotConstants.Spindexer.spindexerLinkageServoDown);
    }

    public void shooterOn() {
        robot.shooter.setPower(RobotConstants.Drivetrain.shooterLongOn);
    }
    public void shooterOff() {
        robot.shooter.setPower(RobotConstants.Drivetrain.shooterOff);
    }
    public void shooterReverse() {
        robot.shooter.setPower(RobotConstants.Drivetrain.shooterReverse);
    }

    public boolean getTouchSensorState() {
        return robot.touchSensor.getState();
    }
    public boolean isLimitSwitchClosed(){
        return !robot.magneticLimitSensor.getState();
    }


    public double detectColor (){
        double allColor = robot.colorSensorOne.red() + robot.colorSensorOne.green() + robot.colorSensorOne.blue();
        return allColor;
    }

}
