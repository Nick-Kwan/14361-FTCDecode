package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.States.AutoIntakeStates;
import org.firstinspires.ftc.teamcode.States.SecondShooterStates;
import org.firstinspires.ftc.teamcode.States.ShooterStates;
import org.firstinspires.ftc.teamcode.States.SortingStates;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.util.RobotConstants;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class Spindexer implements Subsystem{
    private RobotHardware robot;
    public static final Spindexer INSTANCE = new Spindexer();

    public Spindexer(){
        this.robot = RobotHardware.getInstance();
    }
    public SpindexerStates spindexerState;
    public ShooterStates shootingState = ShooterStates.One;
    public SortingStates sortState = SortingStates.Checking;
    public AutoIntakeStates intakeState = AutoIntakeStates.Checking;
    public SecondShooterStates secondShootingState = SecondShooterStates.One;
    public ElapsedTime shooterTimer = new ElapsedTime();
    public ElapsedTime secondShooterTimer = new ElapsedTime();
    public ElapsedTime thirdShooterTimer = new ElapsedTime();
    public ElapsedTime time = new ElapsedTime();
    //private ScheduledExecutorService s = Executors.newScheduledThreadPool(1);
    private double prevPose;
    public double currentPose;
    private double tempTarget;
    private double turns;
    //private ElapsedTime pathTimer;
    public void setSortingState(SortingStates state) {
        sortState = state;
    }
    public void setAutoIntakeState(AutoIntakeStates state) {
        intakeState = state;
    }
    public void setSpindexerState(SpindexerStates state){
        spindexerState = state;
    }
    public enum SpindexerStates{
        moveLeft, moveRight
    }
    public void waitM (double time){
        robot.pathTimer.reset();
        while (robot.pathTimer.milliseconds() < time){

        }
    }
    public void waitMTeleOp (double time){
        robot.pathTimer.reset();
        robot.driver.readButtons();
        if(robot.driver.gamepad.left_trigger > 0.1) {
            robot.mecanum.periodic(0.3);
        } else {
            robot.mecanum.periodic(1);
        }
        while (robot.pathTimer.milliseconds() < time){
            robot.orientation = robot.imu.getRobotYawPitchRollAngles();
            robot.limelight.updateRobotOrientation(robot.orientation.getYaw());
            robot.llResult = robot.limelight.getLatestResult();
            if (robot.aprilID < 20){
                robot.limelight.pipelineSwitch(0);
                robot.limelightTemp = false;
            }            else if (robot.aprilID > 20){
                robot.limelight.pipelineSwitch(3);
                robot.limelightTemp = true;
            }
            if (robot.llResult != null && robot.llResult.isValid()) {
                Pose3D botPose = robot.llResult.getBotpose();
//                telemetry.addData("Target x", robot.llResult.getTx());
//                telemetry.addData("Target y", robot.llResult.getTy());
//                telemetry.addData("Target Area", robot.llResult.getTa());
//                telemetry.addData("BotPose", botPose.toString());
//                telemetry.addData("Yaw", botPose.getOrientation().getYaw());
                List<LLResultTypes.FiducialResult> ID = robot.llResult.getFiducialResults();
                for (LLResultTypes.FiducialResult id : ID) {
                    if (id.getFiducialId() == 21 || id.getFiducialId() == 22 || id.getFiducialId() == 23){
                        robot.aprilID = id.getFiducialId();
                    }
                    else {
                        break;
                    }
                    telemetry.addData("ID", robot.aprilID);
                }
                if (robot.limelightTemp){
                    if (robot.llResult.getTx() > 2){
                        robot.turretServo.setPosition(robot.turretServo.getPosition() + robot.llResult.getTx() / 670);
                    }
                    else if (robot.llResult.getTx() < -2){
                        robot.turretServo.setPosition(robot.turretServo.getPosition() + robot.llResult.getTx() / 670);
                    }
                    robot.limelightLoopTemp = true;
                }
            }
            else if (!robot.llResult.isValid()){
                if (robot.limelightLoopTemp){
                    robot.llResetTimer.resetTimer();
                    robot.limelightLoopTemp = false;
                }
                if (robot.llResetTimer.getElapsedTimeSeconds() > 1 && robot.llResult.getTx() == 0){
                    robot.turretServo.setPosition(RobotConstants.Drivetrain.turretPose);
                    robot.limelightLoopTemp = true;
                }


            }
//            telemetry.addData("ID", robot.aprilID);
//            telemetry.update();
        }
    }
    public void spindexerUp() {
        robot.spindexerLinkageServo.setPosition(RobotConstants.Spindexer.spindexerLinkageServoUp);
    }
    public void spindexerDown() {
        robot.spindexerLinkageServo.setPosition(RobotConstants.Spindexer.spindexerLinkageServoDown);
    }


    public void setPose(SpindexerStates state){
    spindexerState = state;
        switch (state){
            case moveLeft:
                if (!robot.spindexer.getTouchSensorState()){
                    if (robot.atPoseOne){
                        setPoseThree();
                        break;
                    }
                    if (robot.atPoseTwo){
                        setPoseOne();
                        break;
                    }
                    if (robot.atPoseThree){
                        setPoseTwo();
                        break;
                    }
                }
            case moveRight:
                if (!robot.spindexer.getTouchSensorState()){
                    if (robot.atPoseOne){
                        setPoseTwo();
                        break;
                    }
                    if (robot.atPoseTwo){
                        setPoseThree();
                        break;
                    }
                    if (robot.atPoseThree){
                        setPoseOne();
                        break;
                    }
                }
        }
    }
    public void setPoseOne(){
        robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
        robot.atPoseOne = true;
        robot.atPoseTwo = false;
        robot.atPoseThree = false;
    }
    public void setPoseTwo(){
        robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseTwo);
        robot.atPoseOne = false;
        robot.atPoseTwo = true;
        robot.atPoseThree = false;
    }
    public void setPoseThree(){
        robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseThree);
        robot.atPoseOne = false;
        robot.atPoseTwo = false;
        robot.atPoseThree = true;
    }

    public void shoot(){
        if (robot.shootTemp){
            robot.shootTimer.resetTimer();
            robot.shootTemp = false;
        }
        robot.spindexer.spindexerUp();
        if (robot.shootTimer.getElapsedTimeSeconds() > 2){
            robot.spindexer.spindexerDown();
        }
    }

    public boolean getTouchSensorState() {
        return robot.touchSensor.getState();
    }
    public boolean isLimitSwitchClosed(){
        return !robot.magneticLimitSensor.getState();
    }


    public double detectColorOne_1(){
        double allColor = robot.colorSensorOne_1.red() + robot.colorSensorOne_1.green() + robot.colorSensorOne_1.blue();
        return allColor;
    }
    public double detectColorTwo_1(){
        double allColor = robot.colorSensorTwo_1.red() + robot.colorSensorTwo_1.green() + robot.colorSensorTwo_1.blue();
        return allColor;
    }
    public double detectColorThree_1(){
        double allColor = robot.colorSensorThree_1.red() + robot.colorSensorThree_1.green() + robot.colorSensorThree_1.blue();
        return allColor;
    }
    public double detectColorOne_2(){
        double allColor = robot.colorSensorOne_2.red() + robot.colorSensorOne_2.green() + robot.colorSensorOne_2.blue();
        return allColor;
    }
    public double detectColorTwo_2(){
        double allColor = robot.colorSensorTwo_2.red() + robot.colorSensorTwo_2.green() + robot.colorSensorTwo_2.blue();
        return allColor;
    }
    public double detectColorThree_2(){
        double allColor = robot.colorSensorThree_2.red() + robot.colorSensorThree_2.green() + robot.colorSensorThree_2.blue();
        return allColor;
    }

    public void teleOpShootPoseOne(){
        robot.spindexer.spindexerUp();
        robot.timer1.schedule(robot.spindexderDown1,200);
        robot.timer1.schedule(robot.spindexderSetPoseTwo,350);
        robot.timer1.schedule(robot.spindexderUp2,600);
        robot.timer1.schedule(robot.spindexderDown2,800);
        robot.timer1.schedule(robot.spindexderSetPoseThree,950);
        robot.timer1.schedule(robot.spindexderUp3,1200);
        robot.timer1.schedule(robot.spindexderDown3,1400);
    }

    public void teleOpShootPoseTwo(){
        robot.spindexer.spindexerUp();
        robot.timer1.schedule(robot.spindexderDown1,200);
        robot.timer1.schedule(robot.spindexderSetPoseOne,350);
        robot.timer1.schedule(robot.spindexderUp2,600);
        robot.timer1.schedule(robot.spindexderDown2,800);
        robot.timer1.schedule(robot.spindexderSetPoseThree,950);
        robot.timer1.schedule(robot.spindexderUp3,1400);
        robot.timer1.schedule(robot.spindexderDown3,1600);
    }

    public void teleOpShootPoseTwo_2(){
        robot.spindexer.spindexerUp();
        robot.timer1.schedule(robot.spindexderDown1,200);
        robot.timer1.schedule(robot.spindexderSetPoseOne,350);
        robot.timer1.schedule(robot.spindexderUp2,600);
        robot.timer1.schedule(robot.spindexderDown2,800);
        robot.timer1.schedule(robot.spindexderSetPoseThree,950);
        robot.timer1.schedule(robot.spindexderUp3,1400);
        robot.timer1.schedule(robot.spindexderDown3,1600);
    }

    public void teleOpShootPoseThree (){
        robot.spindexer.spindexerUp();
        robot.timer1.schedule(robot.spindexderDown1,200);
        robot.timer1.schedule(robot.spindexderSetPoseTwo,350);
        robot.timer1.schedule(robot.spindexderUp2,600);
        robot.timer1.schedule(robot.spindexderDown2,800);
        robot.timer1.schedule(robot.spindexderSetPoseOne,950);
        robot.timer1.schedule(robot.spindexderUp3,1200);
        robot.timer1.schedule(robot.spindexderDown3,1400);
    }


    public void autoShootPoseOne(){
        robot.s.schedule(() -> {
            robot.spindexer.spindexerUp();
        }, robot.d+= 250 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.spindexerDown();
        }, robot.d+= 200 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.setPoseTwo();
        }, robot.d+= 150 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.spindexerUp();
        }, robot.d+= 250 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.spindexerDown();
        }, robot.d+= 200 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.setPoseThree();
        }, robot.d+= 150 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.spindexerUp();
        }, robot.d+= 250 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.spindexerDown();
        }, robot.d+= 200 , TimeUnit.MILLISECONDS);
//        robot.timer1.schedule(robot.spindexderUp1,250);
//        robot.timer1.schedule(robot.spindexderDown1,450);
//        robot.timer1.schedule(robot.spindexderSetPoseTwo,600);
//        robot.timer1.schedule(robot.spindexderUp2,850);
//        robot.timer1.schedule(robot.spindexderDown2,1050);
//        robot.timer1.schedule(robot.spindexderSetPoseThree,1200);
//        robot.timer1.schedule(robot.spindexderUp3,1450);
//        robot.timer1.schedule(robot.spindexderDown3,1650);
    }

    public void autoShootPoseTwo(){
        robot.spindexer.spindexerUp();
        robot.s.schedule(() -> {
            robot.spindexer.spindexerDown();
        }, robot.d+= 250 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.spindexerDown();
        }, robot.d+= 200 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.setPoseOne();
        }, robot.d+= 150 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.spindexerUp();
        }, robot.d+= 250 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.spindexerDown();
        }, robot.d+= 200 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.setPoseThree();
        }, robot.d+= 150 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.spindexerUp();
        }, robot.d+= 450 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.spindexerDown();
        }, robot.d+= 200 , TimeUnit.MILLISECONDS);
//        robot.timer1.schedule(robot.spindexderUp1,250);
//        robot.timer1.schedule(robot.spindexderDown1,450);
//        robot.timer1.schedule(robot.spindexderSetPoseOne,600);
//        robot.timer1.schedule(robot.spindexderUp2,850);
//        robot.timer1.schedule(robot.spindexderDown2,1050);
//        robot.timer1.schedule(robot.spindexderSetPoseThree,1450);
//        robot.timer1.schedule(robot.spindexderUp3,1700);
//        robot.timer1.schedule(robot.spindexderDown3,1900);
    }

    public void autoShootPoseTwo_2 (){
        robot.spindexer.spindexerUp();
        robot.s.schedule(() -> {
            robot.spindexer.spindexerDown();
        }, robot.d+= 250 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.spindexerDown();
        }, robot.d+= 200 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.setPoseThree();
        }, robot.d+= 150 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.spindexerUp();
        }, robot.d+= 250 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.spindexerDown();
        }, robot.d+= 200 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.setPoseOne();
        }, robot.d+= 150 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.spindexerUp();
        }, robot.d+= 450 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.spindexerDown();
        }, robot.d+= 200 , TimeUnit.MILLISECONDS);
        //robot.spindexer.spindexerUp();
//        robot.timer1.schedule(robot.spindexderUp1,250);
//        robot.timer1.schedule(robot.spindexderDown1,450);
//        robot.timer1.schedule(robot.spindexderSetPoseThree,600);
//        robot.timer1.schedule(robot.spindexderUp2,850);
//        robot.timer1.schedule(robot.spindexderDown2,1050);
//        robot.timer1.schedule(robot.spindexderSetPoseOne,1450);
//        robot.timer1.schedule(robot.spindexderUp3,1700);
//        robot.timer1.schedule(robot.spindexderDown3,1900);
    }

    public void autoShootPoseThree(){
        // Total = 1650
        robot.s.schedule(() -> {
            robot.spindexer.spindexerUp();
        }, robot.d+= 250 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.spindexerDown();
        }, robot.d+= 200 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.setPoseTwo();
        }, robot.d+= 150 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.spindexerUp();
        }, robot.d+= 250 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.spindexerDown();
        }, robot.d+= 200 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.setPoseOne();
        }, robot.d+= 150 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.spindexerUp();
        }, robot.d+= 250 , TimeUnit.MILLISECONDS);
        robot.s.schedule(() -> {
            robot.spindexer.spindexerDown();
        }, robot.d+= 200 , TimeUnit.MILLISECONDS);
//        robot.timer1.schedule(robot.spindexderUp1,250);
//        robot.timer1.schedule(robot.spindexderDown1,450);
//        robot.timer1.schedule(robot.spindexderSetPoseTwo,600);
//        robot.timer1.schedule(robot.spindexderUp2,850);
//        robot.timer1.schedule(robot.spindexderDown2,1050);
//        robot.timer1.schedule(robot.spindexderSetPoseOne,1200);
//        robot.timer1.schedule(robot.spindexderUp3,1450);
//        robot.timer1.schedule(robot.spindexderDown3,1650);
    }


    public void sortingAuto(){
        setPoseTwo();
        //robot.intake.timerTaskSetup();
        robot.d = 0;
        //robot.aprilID = 21;
        // Green Purple Purple
        if (robot.aprilID == 21){
            // Slot 2 = Purple | Slot 1 = Purple
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                //robot.blueSorted.waitM(250);
                setPoseThree();
                autoShootPoseThree();
            }
            // Slot 2 = Purple | Slot 1 = Green
            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
                //robot.blueSorted.waitM(250);
                setPoseOne();
                autoShootPoseOne();
            }
            // Slot 2 = Green | Slot 1 = Purple
            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                //robot.blueSorted.waitM(500);
                autoShootPoseTwo_2();
            }
            else {
                //robot.blueSorted.waitM(500);
                autoShootPoseTwo();
            }
        }
        // Purple Green Purple
        else if (robot.aprilID == 22) {
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                //robot.blueSorted.waitM(500);
                autoShootPoseTwo_2();
            }
            // Slot 2 = Purple | Slot 1 = Green
            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
                //robot.blueSorted.waitM(500);
                autoShootPoseTwo();
            }
            // Slot 2 = Green | Slot 1 = Purple
            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                //robot.blueSorted.waitM(250);
                setPoseOne();
                autoShootPoseOne();
            }
            else {
                //robot.blueSorted.waitM(500);
                autoShootPoseTwo();
            }
        }
        // Purple Purple Green
        else if (robot.aprilID == 23){
            // Slot 2 = Purple | Slot 1 = Purple
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                //robot.blueSorted.waitM(500);
                autoShootPoseTwo();
            }
            // Slot 2 = Purple | Slot 1 = Green
            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
                //robot.blueSorted.waitM(500);
                autoShootPoseTwo_2();
            }
            // Slot 2 = Green | Slot 1 = Purple
            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                //robot.blueSorted.waitM(250);
                setPoseOne();
                autoShootPoseOne();
            }
            else {
                //robot.blueSorted.waitM(500);
                autoShootPoseTwo();
            }
        }
    }


    public void sorting(){
        setPoseTwo();
        robot.intake.timerTaskAutoSetup();
        //robot.aprilID = 21;
        // Green Purple Purple
        if (robot.aprilID == 21){
            // Slot 2 = Purple | Slot 1 = Purple
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                //robot.blueSorted.waitM(250);
                setPoseThree();
                teleOpShootPoseThree();
            }
            // Slot 2 = Purple | Slot 1 = Green
            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
                //robot.blueSorted.waitM(250);
                setPoseOne();
                teleOpShootPoseOne();
            }
            // Slot 2 = Green | Slot 1 = Purple
            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                //robot.blueSorted.waitM(500);
                teleOpShootPoseTwo_2();
            }
            else {
                //robot.blueSorted.waitM(500);
                teleOpShootPoseTwo();
            }
        }
        // Purple Green Purple
        else if (robot.aprilID == 22) {
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                //robot.blueSorted.waitM(500);
                teleOpShootPoseTwo_2();
            }
            // Slot 2 = Purple | Slot 1 = Green
            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
                //robot.blueSorted.waitM(500);
                teleOpShootPoseTwo();
            }
            // Slot 2 = Green | Slot 1 = Purple
            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                //robot.blueSorted.waitM(250);
                setPoseOne();
                teleOpShootPoseOne();
            }
            else {
                //robot.blueSorted.waitM(500);
                teleOpShootPoseTwo();
            }
        }
        // Purple Purple Green
        else if (robot.aprilID == 23){
            // Slot 2 = Purple | Slot 1 = Purple
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                //robot.blueSorted.waitM(500);
                teleOpShootPoseTwo();
            }
            // Slot 2 = Purple | Slot 1 = Green
            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
                //robot.blueSorted.waitM(500);
                teleOpShootPoseTwo_2();
            }
            // Slot 2 = Green | Slot 1 = Purple
            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                //robot.blueSorted.waitM(250);
                setPoseOne();
                teleOpShootPoseOne();
            }
            else {
                //robot.blueSorted.waitM(500);
                teleOpShootPoseTwo();
            }
        }
    }

    public void autoIntake(){
        // At Pose One or Pose Three
        if (robot.atPoseOne && !robot.atPoseTwo && !robot.atPoseThree) {
            if ((robot.spindexer.detectColorThree_1() > 2000 || robot.spindexer.detectColorThree_2() > 2000) && (robot.spindexer.detectColorTwo_1() < 1500 || robot.spindexer.detectColorTwo_2() < 1500)){
                robot.spindexer.setPoseTwo();
            }
            if ((robot.spindexer.detectColorOne_1() > 2000 || robot.spindexer.detectColorOne_2() > 2000) && (robot.spindexer.detectColorTwo_1() < 1500 || robot.spindexer.detectColorTwo_2() < 1500)){
                robot.spindexer.setPoseThree();
            }
        }
        // When the Spindexer is at Pose Two
        // Pose: Two Two Three One Three
        else if (!robot.atPoseOne && robot.atPoseTwo && !robot.atPoseThree) {
            if ((robot.spindexer.detectColorOne_1() > 2000 || robot.spindexer.detectColorOne_2() > 2000) && (robot.spindexer.detectColorTwo_1() < 1500 || robot.spindexer.detectColorTwo_2() < 1500)){
                robot.spindexer.setPoseOne();
            }
            else if ((robot.spindexer.detectColorThree_1() > 2000 || robot.spindexer.detectColorThree_2() > 2000) && (robot.spindexer.detectColorTwo_1() < 1500 || robot.spindexer.detectColorTwo_2() < 1500)){
                robot.spindexer.setPoseThree();
            }
        }
//        // When the Spindexer is at Pose Three
//        // Pose: Three Three One Two One
        else if (!robot.atPoseOne && !robot.atPoseTwo && robot.atPoseThree) {
            if ((robot.spindexer.detectColorThree_1() > 2000 || robot.spindexer.detectColorThree_2() > 2000) && (robot.spindexer.detectColorTwo_1() < 1500 || robot.spindexer.detectColorTwo_2() < 1500)){
                robot.spindexer.setPoseOne();
            }
            else if ((robot.spindexer.detectColorOne_1() > 2000 || robot.spindexer.detectColorOne_2() > 2000) && (robot.spindexer.detectColorTwo_1() < 1500 || robot.spindexer.detectColorTwo_2() < 1500)){
                robot.spindexer.setPoseTwo();
            }
        }
    }


//    public void noSorting (){
//        setPoseThree();
//        robot.blueSorted.waitM(200);
//        spindexerUp();
//        robot.blueSorted.waitM(350);
//        spindexerDown();
//        robot.blueSorted.waitM(200);
//        setPoseTwo();
//        robot.blueSorted.waitM(200);
//        spindexerUp();
//        robot.blueSorted.waitM(350);
//        spindexerDown();
//        robot.blueSorted.waitM(200);
//        setPoseOne();
//        robot.blueSorted.waitM(200);
//        spindexerUp();
//        robot.blueSorted.waitM(350);
//        spindexerDown();
//        robot.blueSorted.waitM(200);
//    }

//    public void noSortingRed (){
//        setPoseOne();
//        robot.redSorted.waitM(200);
//        spindexerUp();
//        robot.redSorted.waitM(350);
//        spindexerDown();
//        robot.redSorted.waitM(200);
//        setPoseTwo();
//        robot.redSorted.waitM(200);
//        spindexerUp();
//        robot.redSorted.waitM(350);
//        spindexerDown();
//        robot.redSorted.waitM(200);
//        setPoseThree();
//        robot.redSorted.waitM(200);
//        spindexerUp();
//        robot.redSorted.waitM(350);
//        spindexerDown();
//        robot.redSorted.waitM(200);
//    }


//    public void sortingBlueAuto (){
////        robot.pathTimer = new ElapsedTime();
//        //robot.aprilID = 22;
//        setPoseTwo();
//        // Green Purple Purple
//        if (robot.aprilID == 21){
//            // Slot 2 = Purple | Slot 1 = Purple
//            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
//                //robot.blueSorted.waitM(250);
//                setPoseThree();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//                robot.blueSorted.waitM(150);
//                setPoseTwo();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//                robot.blueSorted.waitM(150);
//                setPoseOne();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//            }
//            // Slot 2 = Purple | Slot 1 = Green
//            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
//                //robot.blueSorted.waitM(250);
//                setPoseOne();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//                robot.blueSorted.waitM(150);
//                setPoseTwo();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//                robot.blueSorted.waitM(150);
//                setPoseThree();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//            }
//            // Slot 2 = Green | Slot 1 = Purple
//            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
//                //robot.blueSorted.waitM(500);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//                robot.blueSorted.waitM(150);
//                setPoseThree();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//                robot.blueSorted.waitM(150);
//                setPoseOne();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//            }
//            else {
//                //robot.blueSorted.waitM(500);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//                robot.blueSorted.waitM(150);
//                setPoseOne();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//                robot.blueSorted.waitM(150);
//                setPoseThree();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//            }
//        }
//        // Purple Green Purple
//        else if (robot.aprilID == 22){
//            // Slot 2 = Purple | Slot 1 = Purple
//            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
//                //robot.blueSorted.waitM(500);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//                robot.blueSorted.waitM(150);
//                setPoseThree();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//                robot.blueSorted.waitM(150);
//                setPoseOne();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//            }
//            // Slot 2 = Purple | Slot 1 = Green
//            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
//                //robot.blueSorted.waitM(500);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//                robot.blueSorted.waitM(150);
//                setPoseOne();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//                robot.blueSorted.waitM(150);
//                setPoseThree();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//            }
//            // Slot 2 = Green | Slot 1 = Purple
//            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
//                //robot.blueSorted.waitM(250);
//                setPoseOne();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//                robot.blueSorted.waitM(150);
//                setPoseTwo();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//                robot.blueSorted.waitM(150);
//                setPoseThree();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//            }
//            else {
//                //robot.blueSorted.waitM(500);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//                robot.blueSorted.waitM(150);
//                setPoseOne();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//                robot.blueSorted.waitM(150);
//                setPoseThree();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//            }
//        }
//        // Purple Purple Green
//        else if (robot.aprilID == 23){
//            // Slot 2 = Purple | Slot 1 = Purple
//            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
//                //robot.blueSorted.waitM(500);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//                robot.blueSorted.waitM(150);
//                setPoseOne();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//                robot.blueSorted.waitM(150);
//                setPoseThree();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//            }
//            // Slot 2 = Purple | Slot 1 = Green
//            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
//                //robot.blueSorted.waitM(500);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//                robot.blueSorted.waitM(150);
//                setPoseThree();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//                robot.blueSorted.waitM(150);
//                setPoseOne();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//            }
//            // Slot 2 = Green | Slot 1 = Purple
//            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
//                //robot.blueSorted.waitM(250);
//                setPoseOne();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//                robot.blueSorted.waitM(150);
//                setPoseThree();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//                robot.blueSorted.waitM(150);
//                setPoseTwo();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//            }
//            else {
//                //robot.blueSorted.waitM(500);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//                robot.blueSorted.waitM(150);
//                setPoseOne();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//                robot.blueSorted.waitM(150);
//                setPoseThree();
//                robot.blueSorted.waitM(450);
//                spindexerUp();
//                robot.blueSorted.waitM(200);
//                spindexerDown();
//            }
//        }
//    }
//
//
////    public void sortingRedAuto (){
////        //        robot.pathTimer = new ElapsedTime();
////        //robot.aprilID = 22;
////        setPoseTwo();
////        // Green Purple Purple
////        if (robot.aprilID == 21){
////            // Slot 2 = Purple | Slot 1 = Purple
//            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
//                //robot.blueSorted.waitM(250);
//                setPoseThree();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//                robot.redSorted.waitM(150);
//                setPoseTwo();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//                robot.redSorted.waitM(150);
//                setPoseOne();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//            }
//            // Slot 2 = Purple | Slot 1 = Green
//            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
//                //robot.blueSorted.waitM(250);
//                setPoseOne();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//                robot.redSorted.waitM(150);
//                setPoseTwo();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//                robot.redSorted.waitM(150);
//                setPoseThree();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//            }
//            // Slot 2 = Green | Slot 1 = Purple
//            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
//                //robot.blueSorted.waitM(500);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//                robot.redSorted.waitM(150);
//                setPoseThree();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//                robot.redSorted.waitM(150);
//                setPoseOne();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//            }
//            else {
//                //robot.blueSorted.waitM(500);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//                robot.redSorted.waitM(150);
//                setPoseOne();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//                robot.redSorted.waitM(150);
//                setPoseThree();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//            }
//        }
//        // Purple Green Purple
//        else if (robot.aprilID == 22){
//            // Slot 2 = Purple | Slot 1 = Purple
//            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
//                //robot.blueSorted.waitM(500);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//                robot.redSorted.waitM(150);
//                setPoseThree();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//                robot.redSorted.waitM(150);
//                setPoseOne();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//            }
//            // Slot 2 = Purple | Slot 1 = Green
//            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
//                //robot.blueSorted.waitM(500);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//                robot.redSorted.waitM(150);
//                setPoseOne();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//                robot.redSorted.waitM(150);
//                setPoseThree();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//            }
//            // Slot 2 = Green | Slot 1 = Purple
//            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
//                //robot.blueSorted.waitM(250);
//                setPoseOne();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//                robot.redSorted.waitM(150);
//                setPoseTwo();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//                robot.redSorted.waitM(150);
//                setPoseThree();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//            }
//            else {
//                //robot.blueSorted.waitM(500);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//                robot.redSorted.waitM(150);
//                setPoseOne();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//                robot.redSorted.waitM(150);
//                setPoseThree();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//            }
//        }
//        // Purple Purple Green
//        else if (robot.aprilID == 23){
//            // Slot 2 = Purple | Slot 1 = Purple
//            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
//                //robot.blueSorted.waitM(500);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//                robot.redSorted.waitM(150);
//                setPoseOne();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//                robot.redSorted.waitM(150);
//                setPoseThree();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//            }
//            // Slot 2 = Purple | Slot 1 = Green
//            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
//                //robot.blueSorted.waitM(500);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//                robot.redSorted.waitM(150);
//                setPoseThree();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//                robot.redSorted.waitM(150);
//                setPoseOne();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//            }
//            // Slot 2 = Green | Slot 1 = Purple
//            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
//                //robot.blueSorted.waitM(250);
//                setPoseOne();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//                robot.redSorted.waitM(150);
//                setPoseThree();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//                robot.redSorted.waitM(150);
//                setPoseTwo();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//            }
//            else {
//                //robot.blueSorted.waitM(500);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//                robot.redSorted.waitM(150);
//                setPoseOne();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//                robot.redSorted.waitM(150);
//                setPoseThree();
//                robot.redSorted.waitM(450);
//                spindexerUp();
//                robot.redSorted.waitM(200);
//                spindexerDown();
//            }
//        }
//    }
//

    public void sortingTeleOp (){
        //robot.aprilID = 23;
        setPoseTwo();
        // Green Purple Purple
        if (robot.aprilID == 21){
            // Slot 2 = Purple | Slot 1 = Purple
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                waitMTeleOp(250);
                setPoseThree();
                waitMTeleOp(500);
                spindexerUp();
                waitMTeleOp(500);
                spindexerDown();
                waitMTeleOp(250);
                setPoseTwo();
                waitMTeleOp(500);
                spindexerUp();
                waitMTeleOp(500);
                spindexerDown();
                waitMTeleOp(250);
                setPoseOne();
                waitMTeleOp(500);
                spindexerUp();
                waitMTeleOp(500);
                spindexerDown();
            }
            // Slot 2 = Purple | Slot 1 = Green
            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
                waitMTeleOp(250);
                setPoseOne();
                waitMTeleOp(500);
                spindexerUp();
                waitMTeleOp(500);
                spindexerDown();
                waitMTeleOp(250);
                setPoseTwo();
                waitMTeleOp(500);
                spindexerUp();
                waitMTeleOp(500);
                spindexerDown();
                waitMTeleOp(250);
                setPoseThree();
                waitMTeleOp(500);
                spindexerUp();
                waitMTeleOp(500);
                spindexerDown();
            }
            // Slot 2 = Green | Slot 1 = Purple
            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                waitMTeleOp(500);
                spindexerUp();
                waitMTeleOp(500);
                spindexerDown();
                waitMTeleOp(250);
                setPoseThree();
                waitMTeleOp(500);
                spindexerUp();
                waitMTeleOp(500);
                spindexerDown();
                waitMTeleOp(250);
                setPoseOne();
                waitMTeleOp(500);
                spindexerUp();
                waitMTeleOp(500);
                spindexerDown();
            }
        }
        // Purple Green Purple
        else if (robot.aprilID == 22){
            // Slot 2 = Purple | Slot 1 = Purple
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                waitMTeleOp(500);
                spindexerUp();
                waitMTeleOp(500);
                spindexerDown();
                waitMTeleOp(250);
                setPoseThree();
                waitMTeleOp(500);
                spindexerUp();
                waitMTeleOp(500);
                spindexerDown();
                waitMTeleOp(250);
                setPoseOne();
                waitMTeleOp(500);
                spindexerUp();
                waitMTeleOp(500);
                spindexerDown();
            }
            // Slot 2 = Purple | Slot 1 = Green
            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
                waitMTeleOp(500);
                spindexerUp();
                waitMTeleOp(500);
                spindexerDown();
                waitMTeleOp(250);
                setPoseOne();
                waitMTeleOp(500);
                spindexerUp();
                waitMTeleOp(500);
                spindexerDown();
                waitMTeleOp(250);
                setPoseThree();
                waitMTeleOp(500);
                spindexerUp();
                waitMTeleOp(500);
                spindexerDown();
            }
            // Slot 2 = Green | Slot 1 = Purple
            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                waitMTeleOp(250);
                setPoseOne();
                waitMTeleOp(500);
                spindexerUp();
                waitMTeleOp(500);
                spindexerDown();
                waitMTeleOp(250);
                setPoseTwo();
                waitMTeleOp(500);
                spindexerUp();
                waitMTeleOp(500);
                spindexerDown();
                waitMTeleOp(250);
                setPoseThree();
                waitMTeleOp(500);
                spindexerUp();
                waitMTeleOp(500);
                spindexerDown();
            }
        }
        // Purple Purple Green
        else if (robot.aprilID == 23){
            // Slot 2 = Purple | Slot 1 = Purple
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                waitMTeleOp(500);
                spindexerUp();
                waitMTeleOp(500);
                spindexerDown();
                waitMTeleOp(250);
                setPoseOne();
                waitMTeleOp(500);
                spindexerUp();
                waitMTeleOp(500);
                spindexerDown();
                waitMTeleOp(250);
                setPoseThree();
                waitMTeleOp(500);
                spindexerUp();
                waitMTeleOp(500);
                spindexerDown();
            }
            // Slot 2 = Purple | Slot 1 = Green
            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
                waitMTeleOp(500);
                spindexerUp();
                waitMTeleOp(500);
                spindexerDown();
                waitMTeleOp(250);
                setPoseThree();
                waitMTeleOp(500);
                spindexerUp();
                waitMTeleOp(500);
                spindexerDown();
                waitMTeleOp(250);
                setPoseOne();
                waitMTeleOp(500);
                spindexerUp();
                waitMTeleOp(500);
                spindexerDown();
            }
            // Slot 2 = Green | Slot 1 = Purple
            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                waitMTeleOp(250);
                setPoseOne();
                waitMTeleOp(500);
                spindexerUp();
                waitMTeleOp(500);
                spindexerDown();
                waitMTeleOp(250);
                setPoseThree();
                waitMTeleOp(500);
                spindexerUp();
                waitMTeleOp(500);
                spindexerDown();
                waitMTeleOp(250);
                setPoseTwo();
                waitMTeleOp(500);
                spindexerUp();
                waitMTeleOp(500);
                spindexerDown();
            }
        }
    }
}

