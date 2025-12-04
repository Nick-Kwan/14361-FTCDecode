package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.util.RobotConstants;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class turret extends OpMode {

    private RobotHardware robot;

    public turret(){
        this.robot = RobotHardware.getInstance();
    }


    public static PIDController controller;
    public static double p =0.03, i = 0.00005, d = 0.00001;
    public static double f = 0;
    public static int target = 0;
    public final double ticks_in_degree = (double) 28 /360;
    private DcMotorEx shooterMotor;
    @Override
    public void init() {
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooterMotor =hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.shooterOne);

    }

    @Override
    public void loop() {
        controller.setPID(p,i,d);
        double shooterPower = shooterMotor.getPower();
        double pid = controller.calculate(shooterPower, target);

        shooterMotor.setPower(pid);

        telemetry.addData("power" , shooterPower);
        telemetry.addData("target" , target);
        telemetry.update();
    }
}
