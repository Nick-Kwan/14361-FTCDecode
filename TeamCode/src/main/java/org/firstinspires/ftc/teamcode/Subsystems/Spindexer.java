package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.States.AutoIntakeStates.Checking;
import static org.firstinspires.ftc.teamcode.States.AutoIntakeStates.Exit;

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
    private ScheduledExecutorService s = Executors.newScheduledThreadPool(1);
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

//    public void setSpindexerServo(){
////        double error = getSpindexerTurns() - robot.target;
////        robot.power = -robot.spindexerServoPID.calculate(error);
////        robot.spindexerServo.setPower(robot.power);
////        if (Math.abs(error) < 0.02){
////            robot.spindexerServo.setPower(0);
////        }
//
////        if (Math.abs(robot.target + 1 - getSpindexerTurns()) < Math.abs(getSpindexerTurns() - robot.target)){
////            robot.turns += 1;
////        }
//        robot.power = robot.spindexerServoPID.calculate(getRealSpindexerPosition() , getTargetPose());
//        //robot.spindexerServo.setPower(robot.power);
//    }
//
//    public double getRealSpindexerPosition(){
//        return (robot.spindexerServoInput.getVoltage() / 3.3) ;
//    }
//    public double getSpindexerTurns(){
//        currentPose = getRealSpindexerPosition();
//        if (currentPose - robot.target > 0.55){
//            robot.turns++;
//        }
//        else if (currentPose - robot.target < -0.55){
//            robot.turns--;
//        }
//        return robot.turns;
//
//    }
//    public double getTargetPose(){
//        return robot.target;
//    }
//
//
//    public void setSpindexerPosition(double t){
////        if (Math.abs((t + 1) - getSpindexerTurns()) < Math.abs(getSpindexerTurns() - t)){
////            robot.turns += 1;
////        }
//        robot.target = t;
//    }
//    public void setCurrentPose(){
//        robot.currentPose = robot.currentPose + getSpindexerTurns();
//    }
//


    public void setPose(SpindexerStates state){
    spindexerState = state;
        switch (state){
            case moveLeft:
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
            case moveRight:
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



    public void autoIntake(){
        // At Pose One
        if (robot.atPoseOne && !robot.atPoseTwo && !robot.atPoseThree) {
            // All the slots have balls
            if ((robot.spindexer.detectColorOne_1() > 2000 || robot.spindexer.detectColorOne_2() > 2000) && (robot.spindexer.detectColorTwo_1() > 2000 || robot.spindexer.detectColorTwo_2() > 2000) && (robot.spindexer.detectColorThree_1() > 2000 || robot.spindexer.detectColorThree_2() > 2000)){
                robot.spindexer.setPoseOne();
            }
            // The first slot or third slot is empty
            if (robot.spindexer.detectColorOne_1() < 1000 || robot.spindexer.detectColorThree_1() < 1000){
                robot.spindexer.setPoseOne();
            }
            // The first and third slots are taken
            if ((robot.spindexer.detectColorOne_1() > 2000 || robot.spindexer.detectColorOne_2() > 2000) && robot.spindexer.detectColorTwo_1() < 1000 && (robot.spindexer.detectColorThree_1() > 2000 || robot.spindexer.detectColorThree_2() > 2000)){
                robot.spindexer.setPoseTwo();
            }
        }
        // When the Spindexer is at Pose Two
        // Pose: Two Two Three One Three
        else if (!robot.atPoseOne && robot.atPoseTwo && !robot.atPoseThree) {
            // All the slots have balls
            if ((robot.spindexer.detectColorOne_1() > 2000 || robot.spindexer.detectColorOne_2() > 2000) && (robot.spindexer.detectColorTwo_1() > 2000 || robot.spindexer.detectColorTwo_2() > 2000) && (robot.spindexer.detectColorThree_1() > 2000 || robot.spindexer.detectColorThree_2() > 2000)){
                robot.spindexer.setPoseTwo();
            }
            // The first slot or third slot is empty
            if (robot.spindexer.detectColorOne_1() < 1000 || robot.spindexer.detectColorThree_1() < 1000){
                robot.spindexer.setPoseTwo();
            }
            // The first and third slots are taken
            if ((robot.spindexer.detectColorOne_1() > 2000 || robot.spindexer.detectColorOne_2() > 2000) && robot.spindexer.detectColorTwo_1() < 1000 && (robot.spindexer.detectColorThree_1() > 2000 || robot.spindexer.detectColorThree_2() > 2000)){
                robot.spindexer.setPoseThree();
            }
        }
        // When the Spindexer is at Pose Three
        // Pose: Three Three One Two One
        else if (!robot.atPoseOne && !robot.atPoseTwo && robot.atPoseThree) {
            // All the slots have balls
            if ((robot.spindexer.detectColorOne_1() > 2000 || robot.spindexer.detectColorOne_2() > 2000) && (robot.spindexer.detectColorTwo_1() > 2000 || robot.spindexer.detectColorTwo_2() > 2000) && (robot.spindexer.detectColorThree_1() > 2000 || robot.spindexer.detectColorThree_2() > 2000)){
                robot.spindexer.setPoseThree();
            }
            // The first slot or third slot is empty
            if (robot.spindexer.detectColorOne_1() < 1000 || robot.spindexer.detectColorThree_1() < 1000){
                robot.spindexer.setPoseThree();
            }
            // The first and third slots are taken
            if ((robot.spindexer.detectColorOne_1() > 2000 || robot.spindexer.detectColorOne_2() > 2000) && robot.spindexer.detectColorTwo_1() < 1000 && (robot.spindexer.detectColorThree_1() > 2000 || robot.spindexer.detectColorThree_2() > 2000)){
                robot.spindexer.setPoseOne();
            }
        }
    }

    public void autoIntakeOLD (){
        setAutoIntakeState(Checking);
        switch (intakeState){
            case Checking:
                // When the Spindexer is at Pose One
                if (robot.atPoseOne && !robot.atPoseTwo && !robot.atPoseThree) {
                    // All the slots have balls
                    if ((robot.spindexer.detectColorOne_1() > 2000 || robot.spindexer.detectColorOne_2() > 2000) && (robot.spindexer.detectColorTwo_1() > 2000 || robot.spindexer.detectColorTwo_2() > 2000) && (robot.spindexer.detectColorThree_1() > 2000 || robot.spindexer.detectColorThree_2() > 2000)){
                        robot.spindexer.setPoseOne();
                        setAutoIntakeState(Exit);
                    }
                    // The first slot is empty
                    if (robot.spindexer.detectColorOne_1() > 1000){
                        robot.spindexer.setPoseOne();
                        setAutoIntakeState(Exit);
                    }
                    // The second slot is empty and the other two slots are taken
                    if ((robot.spindexer.detectColorOne_1() > 2000 || robot.spindexer.detectColorOne_2() > 2000) && robot.spindexer.detectColorTwo_1() < 1000 && (robot.spindexer.detectColorThree_1() > 2000 || robot.spindexer.detectColorThree_2() > 2000)){
                        robot.spindexer.setPoseTwo();
                        setAutoIntakeState(Exit);
                    }
                    // The third slot is empty and the other two slots are taken
                    if ((robot.spindexer.detectColorOne_1() > 2000 || robot.spindexer.detectColorOne_2() > 2000) && (robot.spindexer.detectColorTwo_1() > 2000 || robot.spindexer.detectColorTwo_2() > 2000) && robot.spindexer.detectColorThree_1() < 1000){
                        robot.spindexer.setPoseThree();
                        setAutoIntakeState(Exit);
                    }
                    // The first slot is taken and the other two slots are empty
                    if ((robot.spindexer.detectColorOne_1() > 2000 || robot.spindexer.detectColorOne_2() > 2000) && robot.spindexer.detectColorTwo_1() < 1000 && robot.spindexer.detectColorThree_1() < 1000){
                        robot.spindexer.setPoseTwo();
                        setAutoIntakeState(Exit);
                    }
                }
                // When the Spindexer is at Pose Two
                // Pose: Two Two Three One Three
                else if (!robot.atPoseOne && robot.atPoseTwo && !robot.atPoseThree) {
                    // All the slots have balls
                    if ((robot.spindexer.detectColorOne_1() > 2000 || robot.spindexer.detectColorOne_2() > 2000) && (robot.spindexer.detectColorTwo_1() > 2000 || robot.spindexer.detectColorTwo_2() > 2000) && (robot.spindexer.detectColorThree_1() > 2000 || robot.spindexer.detectColorThree_2() > 2000)){
                        robot.spindexer.setPoseTwo();
                        setAutoIntakeState(Exit);
                    }
                    // The first slot is empty
                    if (robot.spindexer.detectColorOne_1() > 1000){
                        robot.spindexer.setPoseTwo();
                        setAutoIntakeState(Exit);
                    }
                    // The second slot is empty and the other two slots are taken
                    if ((robot.spindexer.detectColorOne_1() > 2000 || robot.spindexer.detectColorOne_2() > 2000) && robot.spindexer.detectColorTwo_1() < 1000 && (robot.spindexer.detectColorThree_1() > 2000 || robot.spindexer.detectColorThree_2() > 2000)){
                        robot.spindexer.setPoseThree();
                        setAutoIntakeState(Exit);
                    }
                    // The third slot is empty and the other two slots are taken
                    if ((robot.spindexer.detectColorOne_1() > 2000 || robot.spindexer.detectColorOne_2() > 2000) && (robot.spindexer.detectColorTwo_1() > 2000 || robot.spindexer.detectColorTwo_2() > 2000) && robot.spindexer.detectColorThree_1() < 1000){
                        robot.spindexer.setPoseOne();
                        setAutoIntakeState(Exit);
                    }
                    // The first slot is taken and the other two slots are empty
                    if ((robot.spindexer.detectColorOne_1() > 2000 || robot.spindexer.detectColorOne_2() > 2000) && robot.spindexer.detectColorTwo_1() < 1000 && robot.spindexer.detectColorThree_1() < 1000){
                        robot.spindexer.setPoseThree();
                        setAutoIntakeState(Exit);
                    }
                }
                // When the Spindexer is at Pose Three
                // Pose: Three Three One Two One
                else if (!robot.atPoseOne && !robot.atPoseTwo && robot.atPoseThree) {
                    // All the slots have balls
                    if ((robot.spindexer.detectColorOne_1() > 2000 || robot.spindexer.detectColorOne_2() > 2000) && (robot.spindexer.detectColorTwo_1() > 2000 || robot.spindexer.detectColorTwo_2() > 2000) && (robot.spindexer.detectColorThree_1() > 2000 || robot.spindexer.detectColorThree_2() > 2000)){
                        robot.spindexer.setPoseThree();
                        setAutoIntakeState(Exit);
                    }
                    // The first slot is empty
                    if (robot.spindexer.detectColorOne_1() > 1000){
                        robot.spindexer.setPoseThree();
                        setAutoIntakeState(Exit);
                    }
                    // The second slot is empty and the other two slots are taken
                    if ((robot.spindexer.detectColorOne_1() > 2000 || robot.spindexer.detectColorOne_2() > 2000) && robot.spindexer.detectColorTwo_1() < 1000 && (robot.spindexer.detectColorThree_1() > 2000 || robot.spindexer.detectColorThree_2() > 2000)){
                        robot.spindexer.setPoseOne();
                        setAutoIntakeState(Exit);
                    }
                    // The third slot is empty and the other two slots are taken
                    if ((robot.spindexer.detectColorOne_1() > 2000 || robot.spindexer.detectColorOne_2() > 2000) && (robot.spindexer.detectColorTwo_1() > 2000 || robot.spindexer.detectColorTwo_2() > 2000) && robot.spindexer.detectColorThree_1() < 1000){
                        robot.spindexer.setPoseTwo();
                        setAutoIntakeState(Exit);
                    }
                    // The first slot is taken and the other two slots are empty
                    if ((robot.spindexer.detectColorOne_1() > 2000 || robot.spindexer.detectColorOne_2() > 2000) && robot.spindexer.detectColorTwo_1() < 1000 && robot.spindexer.detectColorThree_1() < 1000){
                        robot.spindexer.setPoseOne();
                        setAutoIntakeState(Exit);
                    }
                }
            case Exit:
                break;

        }
    }


    public void sortingBlueAuto (){
//        robot.pathTimer = new ElapsedTime();
        //robot.aprilID = 22;
        setPoseTwo();
        // Green Purple Purple
        if (robot.aprilID == 21){
            // Slot 2 = Purple | Slot 1 = Purple
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                robot.blueSorted.waitM(250);
                setPoseThree();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
                robot.blueSorted.waitM(250);
                setPoseTwo();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
                robot.blueSorted.waitM(250);
                setPoseOne();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
            }
            // Slot 2 = Purple | Slot 1 = Green
            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
                robot.blueSorted.waitM(250);
                setPoseOne();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
                robot.blueSorted.waitM(250);
                setPoseTwo();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
                robot.blueSorted.waitM(250);
                setPoseThree();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
            }
            // Slot 2 = Green | Slot 1 = Purple
            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
                robot.blueSorted.waitM(250);
                setPoseThree();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
                robot.blueSorted.waitM(250);
                setPoseOne();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
            }
            else {
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
                robot.blueSorted.waitM(250);
                setPoseOne();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
                robot.blueSorted.waitM(250);
                setPoseThree();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
            }
        }
        // Purple Green Purple
        else if (robot.aprilID == 22){
            // Slot 2 = Purple | Slot 1 = Purple
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
                robot.blueSorted.waitM(250);
                setPoseThree();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
                robot.blueSorted.waitM(250);
                setPoseOne();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
            }
            // Slot 2 = Purple | Slot 1 = Green
            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
                robot.blueSorted.waitM(250);
                setPoseOne();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
                robot.blueSorted.waitM(250);
                setPoseThree();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
            }
            // Slot 2 = Green | Slot 1 = Purple
            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                robot.blueSorted.waitM(250);
                setPoseOne();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
                robot.blueSorted.waitM(250);
                setPoseTwo();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
                robot.blueSorted.waitM(250);
                setPoseThree();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
            }
            else {
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
                robot.blueSorted.waitM(250);
                setPoseOne();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
                robot.blueSorted.waitM(250);
                setPoseThree();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
            }
        }
        // Purple Purple Green
        else if (robot.aprilID == 23){
            // Slot 2 = Purple | Slot 1 = Purple
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
                robot.blueSorted.waitM(250);
                setPoseOne();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
                robot.blueSorted.waitM(250);
                setPoseThree();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
            }
            // Slot 2 = Purple | Slot 1 = Green
            else if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() < robot.colorSensorOne_1.green()){
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
                robot.blueSorted.waitM(250);
                setPoseThree();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
                robot.blueSorted.waitM(250);
                setPoseOne();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
            }
            // Slot 2 = Green | Slot 1 = Purple
            else if (robot.colorSensorTwo_1.blue() < robot.colorSensorTwo_1.green() && robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                robot.blueSorted.waitM(250);
                setPoseOne();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
                robot.blueSorted.waitM(250);
                setPoseThree();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
                robot.blueSorted.waitM(250);
                setPoseTwo();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
            }
            else {
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
                robot.blueSorted.waitM(250);
                setPoseOne();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
                robot.blueSorted.waitM(250);
                setPoseThree();
                robot.blueSorted.waitM(500);
                spindexerUp();
                robot.blueSorted.waitM(500);
                spindexerDown();
            }
        }
    }


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
    public void NOsorting (){
        robot.aprilID = 21;
        robot.spindexer.setPoseThree();
        if (robot.aprilID == 21){
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green()){
                robot.timerTeleOp.schedule(robot.timerTaskCommands.setPoseTwo(), 250);
                waitM(750);
                if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green()){
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.setPoseOne(), 250);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerUp(), 500);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerDown(), 500);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.setPoseTwo(), 250);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerUp(), 500);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerDown(), 500);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerUp(), 500);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerDown(), 500);
                }
                else if (robot.colorSensorTwo_1.green() > robot.colorSensorTwo_1.blue()){
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerUp(), 500);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerDown(), 500);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.setPoseThree(), 250);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerUp(), 500);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerDown(), 500);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.setPoseOne(), 250);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerUp(), 500);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerDown(), 500);
                }
            }
            else {
                robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerUp(), 500);
                robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerDown(), 500);
                robot.timerTeleOp.schedule(robot.timerTaskCommands.setPoseTwo(), 250);
                robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerUp(), 500);
                robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerDown(), 500);
                robot.timerTeleOp.schedule(robot.timerTaskCommands.setPoseOne(), 250);
                robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerUp(), 500);
                robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerDown(), 500);
            }
        }
        // Purple Green Purple
        if (robot.aprilID == 22){
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green()){
                robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerUp(), 500);
                robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerDown(), 500);
                robot.timerTeleOp.schedule(robot.timerTaskCommands.setPoseTwo(), 250);
                waitM(750);
                if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green()){
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.setPoseOne(), 250);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerUp(), 500);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerDown(), 500);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.setPoseTwo(), 250);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerUp(), 500);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerDown(), 500);
                }
                else if (robot.colorSensorTwo_1.green() > robot.colorSensorTwo_1.blue()){
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerUp(), 500);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerDown(), 500);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.setPoseOne(), 250);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerUp(), 500);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerDown(), 500);
                }
            }
            else {
                robot.timerTeleOp.schedule(robot.timerTaskCommands.setPoseTwo(), 250);
                robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerUp(), 500);
                robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerDown(), 500);
                robot.timerTeleOp.schedule(robot.timerTaskCommands.setPoseThree(), 250);
                robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerUp(), 500);
                robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerDown(), 500);
                robot.timerTeleOp.schedule(robot.timerTaskCommands.setPoseOne(), 250);
                robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerUp(), 500);
                robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerDown(), 500);
            }
        }
        // Purple Purple Green
        if (robot.aprilID == 23){
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green()){
                robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerUp(), 500);
                robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerDown(), 500);
                robot.timerTeleOp.schedule(robot.timerTaskCommands.setPoseTwo(), 250);
                waitM(750);
                if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green()){
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerUp(), 500);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerDown(), 500);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.setPoseOne(), 250);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerUp(), 500);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerDown(), 500);
                }
                else if (robot.colorSensorTwo_1.green() > robot.colorSensorTwo_1.blue()){
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.setPoseOne(), 250);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerUp(), 500);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerDown(), 500);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.setPoseTwo(), 250);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerUp(), 500);
                    robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerDown(), 500);
                }
            }
            else {
                robot.timerTeleOp.schedule(robot.timerTaskCommands.setPoseTwo(), 250);
                robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerUp(), 500);
                robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerDown(), 500);
                robot.timerTeleOp.schedule(robot.timerTaskCommands.setPoseOne(), 250);
                robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerUp(), 500);
                robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerDown(), 500);
                robot.timerTeleOp.schedule(robot.timerTaskCommands.setPoseThree(), 250);
                robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerUp(), 500);
                robot.timerTeleOp.schedule(robot.timerTaskCommands.spindexerDown(), 500);
            }
        }
    }


    public void sortingOldAuto (){
        // Remove this for actual gameplay
        robot.aprilID = 21;
        robot.spindexer.setPoseThree();
        if (robot.aprilID == 21){
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green()){
                robot.blueSorted.waitM(250);
                robot.spindexer.setPoseTwo();
                robot.blueSorted.waitM(750);
                if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green()){
                    robot.blueSorted.waitM(250);
                    robot.spindexer.setPoseOne();
                    robot.blueSorted.waitM(250);
                    robot.spindexer.setPoseOne();
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerUp();
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerDown();
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerDown();
                    robot.blueSorted.waitM(250);
                    robot.spindexer.setPoseTwo();
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerUp();
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerDown();
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerUp();
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerDown();
                }
                else if (robot.colorSensorTwo_1.green() > robot.colorSensorTwo_1.blue()){
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerUp();
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerDown();
                    robot.blueSorted.waitM(250);
                    robot.spindexer.setPoseThree();
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerUp();
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerDown();
                    robot.blueSorted.waitM(250);
                    robot.spindexer.setPoseOne();
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerUp();
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerDown();
                }
            }
            else {
                robot.blueSorted.waitM(500);
                robot.spindexer.spindexerUp();
                robot.blueSorted.waitM(500);
                robot.spindexer.spindexerDown();
                robot.blueSorted.waitM(250);
                robot.spindexer.setPoseTwo();
                robot.blueSorted.waitM(500);
                robot.spindexer.spindexerUp();
                robot.blueSorted.waitM(500);
                robot.spindexer.spindexerDown();
                robot.blueSorted.waitM(250);
                robot.spindexer.setPoseOne();
                robot.blueSorted.waitM(500);
                robot.spindexer.spindexerUp();
                robot.blueSorted.waitM(500);
                robot.spindexer.spindexerDown();
            }
        }
        // Purple Green Purple
        if (robot.aprilID == 22){
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green()){
                robot.blueSorted.waitM(500);
                robot.spindexer.spindexerUp();
                robot.blueSorted.waitM(500);
                robot.spindexer.spindexerDown();
                robot.blueSorted.waitM(250);
                robot.spindexer.setPoseTwo();
                waitM(750);
                if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green()){
                    robot.blueSorted.waitM(250);
                    robot.spindexer.setPoseOne();
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerUp();
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerDown();
                    robot.blueSorted.waitM(250);
                    robot.spindexer.setPoseTwo();
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerUp();
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerDown();
                }
                else if (robot.colorSensorTwo_1.green() > robot.colorSensorTwo_1.blue()){
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerUp();
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerDown();
                    robot.blueSorted.waitM(250);
                    robot.spindexer.setPoseOne();
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerUp();
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerDown();
                }
            }
            else {
                robot.blueSorted.waitM(250);
                robot.spindexer.setPoseTwo();
                robot.blueSorted.waitM(500);
                robot.spindexer.spindexerUp();
                robot.blueSorted.waitM(500);
                robot.spindexer.spindexerDown();
                robot.blueSorted.waitM(250);
                robot.spindexer.setPoseThree();
                robot.blueSorted.waitM(500);
                robot.spindexer.spindexerUp();
                robot.blueSorted.waitM(500);
                robot.spindexer.spindexerDown();
                robot.blueSorted.waitM(250);
                robot.spindexer.setPoseOne();
                robot.blueSorted.waitM(500);
                robot.spindexer.spindexerUp();
                robot.blueSorted.waitM(500);
                robot.spindexer.spindexerDown();
            }
        }
        // Purple Purple Green
        if (robot.aprilID == 23){
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green()){
                robot.blueSorted.waitM(500);
                robot.spindexer.spindexerUp();
                robot.blueSorted.waitM(500);
                robot.spindexer.spindexerDown();
                robot.blueSorted.waitM(250);
                robot.spindexer.setPoseTwo();
                robot.blueSorted.waitM(750);
                if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green()){
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerUp();
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerDown();
                    robot.blueSorted.waitM(250);
                    robot.spindexer.setPoseOne();
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerUp();
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerDown();
                }
                else if (robot.colorSensorTwo_1.green() > robot.colorSensorTwo_1.blue()){
                    robot.blueSorted.waitM(250);
                    robot.spindexer.setPoseOne();
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerUp();
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerDown();
                    robot.blueSorted.waitM(250);
                    robot.spindexer.setPoseTwo();
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerUp();
                    robot.blueSorted.waitM(500);
                    robot.spindexer.spindexerDown();
                }
            }
            else {
                robot.blueSorted.waitM(250);
                robot.spindexer.setPoseTwo();
                robot.blueSorted.waitM(500);
                robot.spindexer.spindexerUp();
                robot.blueSorted.waitM(500);
                robot.spindexer.spindexerDown();
                robot.blueSorted.waitM(250);
                robot.spindexer.setPoseOne();
                robot.blueSorted.waitM(500);
                robot.spindexer.spindexerUp();
                robot.blueSorted.waitM(500);
                robot.spindexer.spindexerDown();
                robot.blueSorted.waitM(250);
                robot.spindexer.setPoseThree();
                robot.blueSorted.waitM(500);
                robot.spindexer.spindexerUp();
                robot.blueSorted.waitM(500);
                robot.spindexer.spindexerDown();
            }
        }
    }



    public void sortingOldTeleOp (){
        // Remove this for actual gameplay
        robot.aprilID = 21;
        robot.spindexer.setPoseThree();
        if (robot.aprilID == 21){
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green()){
                waitMTeleOp(250);
                robot.spindexer.setPoseTwo();
                waitMTeleOp(750);
                if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green()){
                    waitMTeleOp(250);
                    robot.spindexer.setPoseOne();
                    waitMTeleOp(250);
                    robot.spindexer.setPoseOne();
                    waitMTeleOp(500);
                    robot.spindexer.spindexerUp();
                    waitMTeleOp(500);
                    robot.spindexer.spindexerDown();
                    waitMTeleOp(500);
                    robot.spindexer.spindexerDown();
                    waitMTeleOp(250);
                    robot.spindexer.setPoseTwo();
                    waitMTeleOp(500);
                    robot.spindexer.spindexerUp();
                    waitMTeleOp(500);
                    robot.spindexer.spindexerDown();
                    waitMTeleOp(500);
                    robot.spindexer.spindexerUp();
                    waitMTeleOp(500);
                    robot.spindexer.spindexerDown();
                }
                else if (robot.colorSensorTwo_1.green() > robot.colorSensorTwo_1.blue()){
                    waitMTeleOp(500);
                    robot.spindexer.spindexerUp();
                    waitMTeleOp(500);
                    robot.spindexer.spindexerDown();
                    waitMTeleOp(250);
                    robot.spindexer.setPoseThree();
                    waitMTeleOp(500);
                    robot.spindexer.spindexerUp();
                    waitMTeleOp(500);
                    robot.spindexer.spindexerDown();
                    waitMTeleOp(250);
                    robot.spindexer.setPoseOne();
                    waitMTeleOp(500);
                    robot.spindexer.spindexerUp();
                    waitMTeleOp(500);
                    robot.spindexer.spindexerDown();
                }
            }
            else {
                waitMTeleOp(500);
                robot.spindexer.spindexerUp();
                waitMTeleOp(500);
                robot.spindexer.spindexerDown();
                waitMTeleOp(250);
                robot.spindexer.setPoseTwo();
                waitMTeleOp(500);
                robot.spindexer.spindexerUp();
                waitMTeleOp(500);
                robot.spindexer.spindexerDown();
                waitMTeleOp(250);
                robot.spindexer.setPoseOne();
                waitMTeleOp(500);
                robot.spindexer.spindexerUp();
                waitMTeleOp(500);
                robot.spindexer.spindexerDown();
            }
        }
        // Purple Green Purple
        if (robot.aprilID == 22){
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green()){
                waitMTeleOp(500);
                robot.spindexer.spindexerUp();
                waitMTeleOp(500);
                robot.spindexer.spindexerDown();
                waitMTeleOp(250);
                robot.spindexer.setPoseTwo();
                waitMTeleOp(750);
                if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green()){
                    waitMTeleOp(250);
                    robot.spindexer.setPoseOne();
                    waitMTeleOp(500);
                    robot.spindexer.spindexerUp();
                    waitMTeleOp(500);
                    robot.spindexer.spindexerDown();
                    waitMTeleOp(250);
                    robot.spindexer.setPoseTwo();
                    waitMTeleOp(500);
                    robot.spindexer.spindexerUp();
                    waitMTeleOp(500);
                    robot.spindexer.spindexerDown();
                }
                else if (robot.colorSensorTwo_1.green() > robot.colorSensorTwo_1.blue()){
                    waitMTeleOp(500);
                    robot.spindexer.spindexerUp();
                    waitMTeleOp(500);
                    robot.spindexer.spindexerDown();
                    waitMTeleOp(250);
                    robot.spindexer.setPoseOne();
                    waitMTeleOp(500);
                    robot.spindexer.spindexerUp();
                    waitMTeleOp(500);
                    robot.spindexer.spindexerDown();
                }
            }
            else {
                waitMTeleOp(250);
                robot.spindexer.setPoseTwo();
                waitMTeleOp(500);
                robot.spindexer.spindexerUp();
                waitMTeleOp(500);
                robot.spindexer.spindexerDown();
                waitMTeleOp(250);
                robot.spindexer.setPoseThree();
                waitMTeleOp(500);
                robot.spindexer.spindexerUp();
                waitMTeleOp(500);
                robot.spindexer.spindexerDown();
                waitMTeleOp(250);
                robot.spindexer.setPoseOne();
                waitMTeleOp(500);
                robot.spindexer.spindexerUp();
                waitMTeleOp(500);
                robot.spindexer.spindexerDown();
            }
        }
        // Purple Purple Green
        if (robot.aprilID == 23){
            if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green()){
                waitMTeleOp(500);
                robot.spindexer.spindexerUp();
                waitMTeleOp(500);
                robot.spindexer.spindexerDown();
                waitMTeleOp(250);
                robot.spindexer.setPoseTwo();
                waitMTeleOp(750);
                if (robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green()){
                    waitMTeleOp(500);
                    robot.spindexer.spindexerUp();
                    waitMTeleOp(500);
                    robot.spindexer.spindexerDown();
                    waitMTeleOp(250);
                    robot.spindexer.setPoseOne();
                    waitMTeleOp(500);
                    robot.spindexer.spindexerUp();
                    waitMTeleOp(500);
                    robot.spindexer.spindexerDown();
                }
                else if (robot.colorSensorTwo_1.green() > robot.colorSensorTwo_1.blue()){
                    waitMTeleOp(250);
                    robot.spindexer.setPoseOne();
                    waitMTeleOp(500);
                    robot.spindexer.spindexerUp();
                    waitMTeleOp(500);
                    robot.spindexer.spindexerDown();
                    waitMTeleOp(250);
                    robot.spindexer.setPoseTwo();
                    waitMTeleOp(500);
                    robot.spindexer.spindexerUp();
                    waitMTeleOp(500);
                    robot.spindexer.spindexerDown();
                }
            }
            else {
                waitMTeleOp(250);
                robot.spindexer.setPoseTwo();
                waitMTeleOp(500);
                robot.spindexer.spindexerUp();
                waitMTeleOp(500);
                robot.spindexer.spindexerDown();
                waitMTeleOp(250);
                robot.spindexer.setPoseOne();
                waitMTeleOp(500);
                robot.spindexer.spindexerUp();
                waitMTeleOp(500);
                robot.spindexer.spindexerDown();
                waitMTeleOp(250);
                robot.spindexer.setPoseThree();
                waitMTeleOp(500);
                robot.spindexer.spindexerUp();
                waitMTeleOp(500);
                robot.spindexer.spindexerDown();
            }
        }
    }
}

