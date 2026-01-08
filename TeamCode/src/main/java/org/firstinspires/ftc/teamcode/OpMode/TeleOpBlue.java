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
//        telemetry.addData("Spindexer Voltage" , robot.spindexerServoInput.getVoltage() / 3.3);
//        telemetry.addData("Power" , robot.power);
//        telemetry.addData("Target" , robot.target);
//        //telemetry.addData("Current" , robot.spindexer.getSpindexerTurns());
//        telemetry.addData("Previous" , robot.prevPose);
//        telemetry.addData("Turns" , robot.turns);

//        telemetry.addData("Color 1.1 : ", robot.spindexer.detectColorOne_1());
//        telemetry.addData("Color 2.1 : ", robot.spindexer.detectColorTwo_1());
//        telemetry.addData("Color 3.1 : ", robot.spindexer.detectColorThree_1());
//        telemetry.addData("Color 1.2 : ", robot.spindexer.detectColorOne_2());
//        telemetry.addData("Color 2.2 : ", robot.spindexer.detectColorTwo_2());
//        telemetry.addData("Color 3.2 : ", robot.spindexer.detectColorThree_2());
        //telemetry.addData("Shooter Velocity " , (robot.shooterOne.getVelocity()/28) * 60);

        robot.limelight.start();
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        robot.limelight.updateRobotOrientation(orientation.getYaw());
        robot.limelight.pipelineSwitch(3);
        LLResult llResult = robot.limelight.getLatestResult();
        if (llResult != null && llResult.isValid()){
            Pose3D botPose = llResult.getBotpose();
            telemetry.addData("Target x", llResult.getTx());
            telemetry.addData("Target y", llResult.getTy());
            telemetry.addData("Target Area", llResult.getTa());
            telemetry.addData("BotPose", botPose.toString());
            telemetry.addData("Yaw", botPose.getOrientation().getYaw());
//            telemetry.addData("Barcode results ", llResult.getBarcodeResults());
//            telemetry.addData("Classifier results ", llResult.getClassifierResults());
//            telemetry.addData("Detector results ", llResult.getDetectorResults());
//            List<LLResultTypes.FiducialResult> ID = llResult.getFiducialResults();
//            for (LLResultTypes.FiducialResult id : ID) {
//                robot.aprilID = id.getFiducialId();
//                telemetry.addData("ID" ,robot.aprilID);
//            }
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


//        // TimerTask commands
//        TimerTask startIntake = new TimerTask() {
//            public void run() {
//                robot.intake.startIntaking();
//            }
//        };


        // Controls
//        robot.spindexer.setSpindexerServo();
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
//            robot.spindexer.autoIntake();
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
            //robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerUp(),1000 );
            robot.spindexer.spindexerUp();
        }
        else {
            //robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerDown(), 1000);
            robot.spindexer.spindexerDown();
        }

        if (!robot.spindexer.getTouchSensorState()){
            if (driver.gamepad.dpadLeftWasPressed()) {
                robot.spindexer.setPose(Spindexer.SpindexerStates.moveLeft);
//                robot.turns = robot.spindexer.getSpindexerTurns();
//                robot.spindexer.setSpindexerPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
//                robot.spindexer.setPoseOne();
                //robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
            }

            if (driver.gamepad.dpadUpWasPressed()) {
//                robot.turns = robot.spindexer.getSpindexerTurns();
//                robot.spindexer.setSpindexerPosition(RobotConstants.Spindexer.spindexerServoPoseTwo);
                robot.spindexer.setPoseTwo();
                //robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseTwo);
            }

            if (driver.gamepad.dpadRightWasPressed()) {
                  robot.spindexer.setPose(Spindexer.SpindexerStates.moveRight);
//                robot.turns = robot.spindexer.getSpindexerTurns();
//                robot.spindexer.setSpindexerPosition(RobotConstants.Spindexer.spindexerServoPoseThree);
//                robot.spindexer.setPoseThree();
                //robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseThree);
            }
        }
//        if (driver.gamepad.dpad_left) {
//            robot.spindexer.setPoseOne();
//            //robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
//        }
//
//        if (driver.gamepad.dpad_up) {
//            robot.spindexer.setPoseTwo();
//            //robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseTwo);
//        }
//
//        if (driver.gamepad.dpad_right) {
//            robot.spindexer.setPoseThree();
//            //robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseThree);
//        }

        if (driver.gamepad.triangle){
            robot.shooterMotors.set(0.6);
            robot.adjustableHoodServo.setPosition(RobotConstants.Drivetrain.hoodPoseMid);
//            robot.shooterOne.setPower(RobotConstants.Drivetrain.shooterShortOn);
//            robot.shooterTwo.setPower(RobotConstants.Drivetrain.shooterShortOn);
        }
        if (driver.gamepad.circle){
            robot.shooterMotors.set(0.75);
            robot.adjustableHoodServo.setPosition(RobotConstants.Drivetrain.hoodPoseLong);
//            robot.shooterOne.setPower(RobotConstants.Drivetrain.shooterLongOn);
//            robot.shooterTwo.setPower(RobotConstants.Drivetrain.shooterLongOn);
        }
        if (driver.gamepad.square){
            robot.shooterMotors.set(0.0);
//            robot.shooterOne.setPower(RobotConstants.Drivetrain.shooterOff);
//            robot.shooterTwo.setPower(RobotConstants.Drivetrain.shooterOff);
        }

        if (driver.gamepad.leftStickButtonWasPressed()){
            robot.adjustableHoodServo.setPosition(robot.adjustableHoodServo.getPosition() + 0.05);
        }
        if (driver.gamepad.rightStickButtonWasPressed()){
            robot.adjustableHoodServo.setPosition(robot.adjustableHoodServo.getPosition() - 0.05);
        }

//        if (driver.gamepad.triangle){
//            robot.shooterOne.setVelocity(((double) 2500 /60) * 28);
//        }
//        if (driver.gamepad.circle){
//            robot.shooterOne.setVelocity(((double) 3500 /60) * 28);
//        }
//        if (driver.gamepad.square){
//            robot.shooterOne.setPower(RobotConstants.Drivetrain.shooterOff);
//        }
//
//        if (driver.gamepad.leftStickButtonWasPressed()){
//            robot.shooterOne.setVelocity(robot.shooterOne.getVelocity() - 50);
//        }
//        if (driver.gamepad.rightStickButtonWasPressed()){
//            robot.shooterOne.setVelocity(robot.shooterOne.getVelocity() + 50);
//        }

        if (driver.gamepad.leftBumperWasPressed()) {
            robot.spindexer.setPoseTwo();
            robot.spindexer.sortingTeleOp();
        }



        telemetry.update();

        driver.readButtons();

    }
}
