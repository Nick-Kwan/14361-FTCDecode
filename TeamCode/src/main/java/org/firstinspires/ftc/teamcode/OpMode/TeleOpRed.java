package org.firstinspires.ftc.teamcode.OpMode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.States.SpindexerStates;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import com.pedropathing.util.Timer;

@TeleOp(name = "TeleOpRed")

public class TeleOpRed extends CommandOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    private GamepadEx driver;
    boolean temp = true;

    private Timer llResetTimer;
    private SpindexerStates spindexerState = SpindexerStates.poseOne;
    public void setSpindexerState(SpindexerStates state) {
        spindexerState = state;
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        driver = new GamepadEx(gamepad1);

        robot.init(hardwareMap,driver);
        llResetTimer = new Timer();
    }

    @Override
    public void run() {

        CommandScheduler.getInstance().run();
        telemetry.addData("Magnet State : ", robot.spindexer.isLimitSwitchClosed());
        telemetry.addData("Touch Sensor : ", !robot.spindexer.getTouchSensorState());
        telemetry.addData("Color One : ", robot.spindexer.detectColorOne_1());
        telemetry.addData("Color Two : ", robot.spindexer.detectColorTwo_1());
        telemetry.addData("Color Three : ", robot.spindexer.detectColorThree_1());
        telemetry.addData("Spin State : " , spindexerState);
        telemetry.addData("Shooter Power: " , robot.shooterOne.getPower());

        robot.limelight.start();
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        robot.limelight.updateRobotOrientation(orientation.getYaw());
        robot.limelight.pipelineSwitch(2);
        LLResult llResult = robot.limelight.getLatestResult();
        if (llResult != null && llResult.isValid()){
            Pose3D botPose = llResult.getBotpose();
            telemetry.addData("Target x", llResult.getTx());
            telemetry.addData("Target y", llResult.getTy());
            telemetry.addData("Target Area", llResult.getTa());
            telemetry.addData("BotPose", botPose.toString());
            telemetry.addData("Yaw", botPose.getOrientation().getYaw());
            if (llResult.getTx() > 2){
                robot.turretServo.setPosition(robot.turretServo.getPosition() + 0.015);
            }
            else if (llResult.getTx() < -2){
                robot.turretServo.setPosition(robot.turretServo.getPosition() - 0.015);
            }
            temp = true;
        }
        else if (!llResult.isValid()){
            if (temp){
                llResetTimer.resetTimer();
                temp = false;
            }
            if (llResetTimer.getElapsedTimeSeconds() > 1 && llResult.getTx() == 0){
                robot.turretServo.setPosition(RobotConstants.Drivetrain.turretPose);
                temp = true;
            }
        }


        if(driver.gamepad.left_trigger > 0.1) {
            robot.mecanum.periodic(0.3);
        } else {
            robot.mecanum.periodic(1);
        }

        if (driver.gamepad.ps) {
            robot.imu.resetYaw();
        }

        if(driver.gamepad.right_trigger > 0.1) {
            robot.intake.startIntaking();
            robot.intake.intakeDown();
        }
        else if (driver.gamepad.right_bumper){
            robot.intake.reverseIntaking();
            robot.intake.intakeDown();
        }
        else {
            robot.intake.stopIntaking();
            robot.intake.intakeUp();
        }

        if (driver.gamepad.a){
            robot.spindexer.spindexerUp();
        }
        else {
            robot.spindexer.spindexerDown();
        }

        if (!robot.spindexer.getTouchSensorState()) {

            if (driver.gamepad.dpad_left) {
                robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
            }

            if (driver.gamepad.dpad_up) {
                robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseTwo);
            }

            if (driver.gamepad.dpad_right) {
                robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseThree);
            }
        }

        if (driver.gamepad.triangle){
            robot.shooterOne.setPower(RobotConstants.Drivetrain.shooterShortOn);
        }
        if (driver.gamepad.circle){
            robot.shooterOne.setPower(RobotConstants.Drivetrain.shooterLongOn);
        }
        if (driver.gamepad.square){
            robot.shooterOne.setPower(RobotConstants.Drivetrain.shooterOff);
        }

        if (driver.gamepad.leftStickButtonWasPressed()){
            robot.shooterOne.setPower(robot.shooterOne.getPower() + 0.05);
        }
        if (driver.gamepad.rightStickButtonWasPressed()){
            robot.shooterOne.setPower(robot.shooterOne.getPower() - 0.05);
        }


        telemetry.update();

        driver.readButtons();

    }
}

