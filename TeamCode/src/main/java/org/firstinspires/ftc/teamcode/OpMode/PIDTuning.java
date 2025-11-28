package org.firstinspires.ftc.teamcode.OpMode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

@Config
@TeleOp (name = "PID Tuning")
public class PIDTuning extends OpMode{
    public PIDFCoefficients pid;
    public static double p =0.03, i = 0.00005, d = 0.00001;
    public static double f = 0;

    public static int target = 0;
    public DcMotorEx shooter;
    @Override
    public void init() {
        pid = new PIDFCoefficients(p,i,d,f);
        shooter = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.shooter);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,pid);
    }

    @Override
    public void loop() {
        shooter.setVelocity(((double) target /60) * 28);
        telemetry.addData("Shooter Velocity " , (shooter.getVelocity()/28) * 60);
        telemetry.addData("PID Values", shooter.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER));
        telemetry.update();
    }
}
