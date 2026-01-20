package org.firstinspires.ftc.teamcode.OpMode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@TeleOp(name = "TeleOpBlue")

public class TeleOpBlue extends CommandOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    private GamepadEx driver;
    boolean temp = true;

    private Timer llResetTimer;
    private boolean timerTaskTemp;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        driver = new GamepadEx(gamepad1);

        robot.init(hardwareMap,driver);
        llResetTimer = new Timer();
        robot.pathTimer = new ElapsedTime();
    }

    @Override
    public void run() {
        // Telemetry and Limelight

        CommandScheduler.getInstance().run();
        telemetry.addData("Magnet State : ", robot.spindexer.isLimitSwitchClosed());
        telemetry.addData("Touch Sensor : ", !robot.spindexer.getTouchSensorState());
        telemetry.addData("April Tag ID # : ", robot.aprilID);
        List<Double> velocities = robot.shooterMotors.getVelocities();
        telemetry.addData("Left Flywheel Velocity", velocities.get(0));
        telemetry.addData("Right Flywheel Velocity", velocities.get(1));

        robot.limelight.start();
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        robot.limelight.updateRobotOrientation(orientation.getYaw());
        robot.limelight.pipelineSwitch(3);
        LLResult llResult = robot.limelight.getLatestResult();
        if (llResult != null && llResult.isValid()){
            robot.targetY = llResult.getTy();
            Pose3D botPose = llResult.getBotpose();
            telemetry.addData("Target x", llResult.getTx());
            telemetry.addData("Target y", robot.targetY);
            telemetry.addData("Target Area", llResult.getTa());
            telemetry.addData("Position", botPose.getPosition());
            telemetry.addData("BotPose", botPose.toString());
            telemetry.addData("Yaw", botPose.getOrientation().getYaw());
            telemetry.addData("April Tag ID(Fiducial results) ", llResult.getFiducialResults());
            if (llResult.getTx() > 1.5){
                robot.turretServo.setPosition(robot.turretServo.getPosition() + llResult.getTx() / 600);
            }
            else if (llResult.getTx() < -1.5){
                robot.turretServo.setPosition(robot.turretServo.getPosition() + llResult.getTx() / 600);
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
        robot.hubs.forEach(LynxModule::clearBulkCache);


        // TimerTask commands
        robot.intake.timerTaskSetup();


        // Controls
        robot.ShooterLUT.getReadyToShoot();

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
            if (robot.autoIntakeIntermittenceBool){
                robot.autoIntakeIntermittenceBool = false;
                robot.timer2.schedule(robot.autoIntakeIntermittence,300);
            }


        }
        else if (driver.gamepad.right_bumper){
            robot.intake.reverseIntaking();
            robot.intake.intakeDown();
        }
        else {
            robot.intake.stopIntaking();
            robot.intake.intakeUp();
        }

        if (driver.gamepad.aWasPressed()) {
            robot.spindexer.spindexerUp();
        }else if (driver.gamepad.aWasReleased()){
            robot.spindexer.spindexerDown();
        }

        if (driver.gamepad.yWasPressed()){
            if (robot.atPoseOne){
                robot.spindexer.teleOpShootPoseOne();
            } else if (robot.atPoseTwo){
                robot.spindexer.teleOpShootPoseTwo();
            } else if (robot.atPoseThree){
                robot.spindexer.teleOpShootPoseThree();
            }
        }

        if (!robot.spindexer.getTouchSensorState()){
            if (driver.gamepad.dpadLeftWasPressed()) {
                robot.spindexer.setPose(Spindexer.SpindexerStates.moveLeft);
            }
            if (driver.gamepad.dpad_up) {
                robot.spindexer.setPoseTwo();
            }
            if (driver.gamepad.dpadRightWasPressed()) {
                  robot.spindexer.setPose(Spindexer.SpindexerStates.moveRight);
            }

        }

        if (driver.gamepad.leftBumperWasPressed()) {
            if (robot.atPoseTwo){
                robot.spindexer.sorting();
                robot.spindexer.waitM(2000);
            }else{
                robot.spindexer.setPoseTwo();
                robot.timer1.schedule(robot.autoSortDelay,250);
            }
        }
        telemetry.update();

        driver.readButtons();

    }
}
