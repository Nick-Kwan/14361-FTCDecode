package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class Mecanum implements Subsystem {
    private RobotHardware robot;

    public Mecanum(){
        this.robot = RobotHardware.getInstance();
    }

    public void periodic(double slowmode) {
        double y = robot.driver.getLeftY();
        double x = robot.driver.getLeftX() * 1.1;
        double rx = robot.driver.getRightX() * 1.1;

        double heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;


        robot.leftFront.setPower(frontLeftPower * slowmode);
        robot.leftRear.setPower(backLeftPower * slowmode);
        robot.rightFront.setPower(frontRightPower * slowmode);
        robot.rightRear.setPower(backRightPower * slowmode);
    }
}
