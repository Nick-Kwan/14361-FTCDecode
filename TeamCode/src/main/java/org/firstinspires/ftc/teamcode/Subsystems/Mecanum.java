package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class Mecanum implements Subsystem {
    private RobotHardware robot;
    private double heading, frontLeftPower, backLeftPower, frontRightPower, backRightPower, rotY, rotX, rx, x, y, denominator;

    public Mecanum(){
        this.robot = RobotHardware.getInstance();
    }

    public void periodic(double slowmode) {
        double y = robot.driver.getLeftY();
        double x = robot.driver.getLeftX();
        rx = gamepad1.right_stick_x;

        heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        frontLeftPower = (rotY + rotX + rx) / denominator;
        backLeftPower = (rotY - rotX + rx) / denominator;
        frontRightPower = (rotY - rotX - rx) / denominator;
        backRightPower = (rotY + rotX - rx) / denominator;

        robot.leftFront.setPower(frontLeftPower * slowmode);
        robot.leftRear.setPower(backLeftPower * slowmode);
        robot.rightFront.setPower(frontRightPower * slowmode);
        robot.rightRear.setPower(backRightPower * slowmode);
    }
}
