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
    private double prevPose;
    public double currentPose;
    private double tempTarget;
    private double turns;

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

    @FunctionalInterface
    public interface WaitMs {
        void waitM(double ms);
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

    public boolean getTouchSensorState() {
        return robot.touchSensor.getState();
    }
    public boolean isLimitSwitchClosed(){
        return !robot.magneticLimitSensor.getState();
    }


    public double detectColorOne_1(){
        return robot.colorSensorOne_1.red() + robot.colorSensorOne_1.green() + robot.colorSensorOne_1.blue();
    }
    public double detectColorTwo_1(){
        return robot.colorSensorTwo_1.red() + robot.colorSensorTwo_1.green() + robot.colorSensorTwo_1.blue();
    }
    public double detectColorThree_1(){
        return robot.colorSensorThree_1.red() + robot.colorSensorThree_1.green() + robot.colorSensorThree_1.blue();
    }
    public double detectColorOne_2(){
        return robot.colorSensorOne_2.red() + robot.colorSensorOne_2.green() + robot.colorSensorOne_2.blue();
    }
    public double detectColorTwo_2(){
        return robot.colorSensorTwo_2.red() + robot.colorSensorTwo_2.green() + robot.colorSensorTwo_2.blue();
    }
    public double detectColorThree_2(){
        return robot.colorSensorThree_2.red() + robot.colorSensorThree_2.green() + robot.colorSensorThree_2.blue();
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
    }

    public void autoShootPoseThree(){
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
    }


    public void sortingAuto(){
        setPoseTwo();
        robot.d = 0;
        // Green Purple Purple
        if (robot.aprilID == 21){
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                setPoseThree();
                autoShootPoseThree();
            }
            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
                setPoseOne();
                autoShootPoseOne();
            }
            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                autoShootPoseTwo_2();
            }
            else {
                autoShootPoseTwo();
            }
        }
        // Purple Green Purple
        else if (robot.aprilID == 22) {
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                autoShootPoseTwo_2();
            }
            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
                autoShootPoseTwo();
            }
            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                setPoseOne();
                autoShootPoseOne();
            }
            else {
                autoShootPoseTwo();
            }
        }
        // Purple Purple Green
        else if (robot.aprilID == 23){
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                autoShootPoseTwo();
            }
            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
                autoShootPoseTwo_2();
            }
            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                setPoseOne();
                autoShootPoseOne();
            }
            else {
                autoShootPoseTwo();
            }
        }
    }


    public void sorting(){
        setPoseTwo();
        robot.intake.timerTaskSetup();
        // Green Purple Purple
        if (robot.aprilID == 21){
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                setPoseThree();
                teleOpShootPoseThree();
            }
            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
                setPoseOne();
                teleOpShootPoseOne();
            }
            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                teleOpShootPoseTwo();
            }
            else {
                teleOpShootPoseTwo();
            }
        }
        // Purple Green Purple
        else if (robot.aprilID == 22) {
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                teleOpShootPoseTwo();
            }
            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
                teleOpShootPoseTwo();
            }
            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                setPoseOne();
                teleOpShootPoseOne();
            }
            else {
                teleOpShootPoseTwo();
            }
        }
        // Purple Purple Green
        else if (robot.aprilID == 23){
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                teleOpShootPoseTwo();
            }
            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
                teleOpShootPoseTwo();
            }
            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                setPoseOne();
                teleOpShootPoseOne();
            }
            else {
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
        else if (!robot.atPoseOne && robot.atPoseTwo && !robot.atPoseThree) {
            if ((robot.spindexer.detectColorOne_1() > 2000 || robot.spindexer.detectColorOne_2() > 2000) && (robot.spindexer.detectColorTwo_1() < 1500 || robot.spindexer.detectColorTwo_2() < 1500)){
                robot.spindexer.setPoseOne();
            }
            else if ((robot.spindexer.detectColorThree_1() > 2000 || robot.spindexer.detectColorThree_2() > 2000) && (robot.spindexer.detectColorTwo_1() < 1500 || robot.spindexer.detectColorTwo_2() < 1500)){
                robot.spindexer.setPoseThree();
            }
        }
        // When the Spindexer is at Pose Three
        else if (!robot.atPoseOne && !robot.atPoseTwo && robot.atPoseThree) {
            if ((robot.spindexer.detectColorThree_1() > 2000 || robot.spindexer.detectColorThree_2() > 2000) && (robot.spindexer.detectColorTwo_1() < 1500 || robot.spindexer.detectColorTwo_2() < 1500)){
                robot.spindexer.setPoseOne();
            }
            else if ((robot.spindexer.detectColorOne_1() > 2000 || robot.spindexer.detectColorOne_2() > 2000) && (robot.spindexer.detectColorTwo_1() < 1500 || robot.spindexer.detectColorTwo_2() < 1500)){
                robot.spindexer.setPoseTwo();
            }
        }
    }

    private void noSortingHelper(Runnable[] poses, WaitMs waiter) {
        for (Runnable pose : poses) {
            pose.run();
            waiter.waitM(200);
            spindexerUp();
            waiter.waitM(350);
            spindexerDown();
            waiter.waitM(200);
        }
    }

    public void noSorting (){
        noSortingHelper(new Runnable[]{this::setPoseThree, this::setPoseTwo, this::setPoseOne}, robot.blueSorted::waitM);
    }

    public void noSortingRed (){
        noSortingHelper(new Runnable[]{this::setPoseOne, this::setPoseTwo, this::setPoseThree}, robot.redSorted::waitM);
    }

    private void shootAtThreePoses(Runnable poseA, Runnable poseB, Runnable poseC, WaitMs waiter) {
        poseA.run();
        waiter.waitM(450);
        spindexerUp();
        waiter.waitM(200);
        spindexerDown();
        waiter.waitM(150);
        poseB.run();
        waiter.waitM(450);
        spindexerUp();
        waiter.waitM(200);
        spindexerDown();
        waiter.waitM(150);
        poseC.run();
        waiter.waitM(450);
        spindexerUp();
        waiter.waitM(200);
        spindexerDown();
    }

    private void shootCurrentThenTwoPoses(Runnable poseB, Runnable poseC, WaitMs waiter) {
        spindexerUp();
        waiter.waitM(200);
        spindexerDown();
        waiter.waitM(150);
        poseB.run();
        waiter.waitM(450);
        spindexerUp();
        waiter.waitM(200);
        spindexerDown();
        waiter.waitM(150);
        poseC.run();
        waiter.waitM(450);
        spindexerUp();
        waiter.waitM(200);
        spindexerDown();
    }

    private void sortingAllianceAuto(WaitMs waiter) {
        setPoseTwo();
        // Green Purple Purple
        if (robot.aprilID == 21){
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                shootAtThreePoses(this::setPoseThree, this::setPoseTwo, this::setPoseOne, waiter);
            }
            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
                shootAtThreePoses(this::setPoseOne, this::setPoseTwo, this::setPoseThree, waiter);
            }
            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                shootCurrentThenTwoPoses(this::setPoseThree, this::setPoseOne, waiter);
            }
            else {
                shootCurrentThenTwoPoses(this::setPoseOne, this::setPoseThree, waiter);
            }
        }
        // Purple Green Purple
        else if (robot.aprilID == 22){
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                shootCurrentThenTwoPoses(this::setPoseThree, this::setPoseOne, waiter);
            }
            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
                shootCurrentThenTwoPoses(this::setPoseOne, this::setPoseThree, waiter);
            }
            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                shootAtThreePoses(this::setPoseOne, this::setPoseTwo, this::setPoseThree, waiter);
            }
            else {
                shootCurrentThenTwoPoses(this::setPoseOne, this::setPoseThree, waiter);
            }
        }
        // Purple Purple Green
        else if (robot.aprilID == 23){
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                shootCurrentThenTwoPoses(this::setPoseOne, this::setPoseThree, waiter);
            }
            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
                shootCurrentThenTwoPoses(this::setPoseThree, this::setPoseOne, waiter);
            }
            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                shootAtThreePoses(this::setPoseOne, this::setPoseThree, this::setPoseTwo, waiter);
            }
            else {
                shootCurrentThenTwoPoses(this::setPoseOne, this::setPoseThree, waiter);
            }
        }
    }

    public void sortingBlueAuto (){
        sortingAllianceAuto(robot.blueSorted::waitM);
    }

    public void sortingRedAuto (){
        sortingAllianceAuto(robot.redSorted::waitM);
    }

    public void sortingTeleOp (){
        setPoseTwo();
        // Green Purple Purple
        if (robot.aprilID == 21){
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                shootAtThreePoses(this::setPoseThree, this::setPoseTwo, this::setPoseOne, this::waitMTeleOp);
            }
            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
                shootAtThreePoses(this::setPoseOne, this::setPoseTwo, this::setPoseThree, this::waitMTeleOp);
            }
            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                shootCurrentThenTwoPoses(this::setPoseThree, this::setPoseOne, this::waitMTeleOp);
            }
        }
        // Purple Green Purple
        else if (robot.aprilID == 22){
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                shootCurrentThenTwoPoses(this::setPoseThree, this::setPoseOne, this::waitMTeleOp);
            }
            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
                shootCurrentThenTwoPoses(this::setPoseOne, this::setPoseThree, this::waitMTeleOp);
            }
            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                shootAtThreePoses(this::setPoseOne, this::setPoseTwo, this::setPoseThree, this::waitMTeleOp);
            }
        }
        // Purple Purple Green
        else if (robot.aprilID == 23){
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                shootCurrentThenTwoPoses(this::setPoseOne, this::setPoseThree, this::waitMTeleOp);
            }
            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
                shootCurrentThenTwoPoses(this::setPoseThree, this::setPoseOne, this::waitMTeleOp);
            }
            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                shootAtThreePoses(this::setPoseOne, this::setPoseThree, this::setPoseTwo, this::waitMTeleOp);
            }
        }
    }
}
