package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.States.AutoIntakeStates.Checking;
import static org.firstinspires.ftc.teamcode.States.AutoIntakeStates.Exit;

import org.firstinspires.ftc.teamcode.States.AutoIntakeStates;
import org.firstinspires.ftc.teamcode.States.SecondShooterStates;
import org.firstinspires.ftc.teamcode.States.ShooterStates;
import org.firstinspires.ftc.teamcode.States.SortingStates;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.util.RobotConstants;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

public class Spindexer implements Subsystem{
    private RobotHardware robot;
    public static final Spindexer INSTANCE = new Spindexer();

    public Spindexer(){
        this.robot = RobotHardware.getInstance();
    }
    public ShooterStates shootingState = ShooterStates.One;
    public SortingStates sortState = SortingStates.Checking;
    public AutoIntakeStates intakeState = AutoIntakeStates.Checking;
    public SecondShooterStates secondShootingState = SecondShooterStates.One;
    public ElapsedTime shooterTimer = new ElapsedTime();
    public ElapsedTime secondShooterTimer = new ElapsedTime();
    public ElapsedTime thirdShooterTimer = new ElapsedTime();
    public ElapsedTime time = new ElapsedTime();
    private ScheduledExecutorService s = Executors.newScheduledThreadPool(1);
    //private ElapsedTime pathTimer;
    public void setSortingState(SortingStates state) {
        sortState = state;
    }
    public void setAutoIntakeState(AutoIntakeStates state) {
        intakeState = state;
    }
    public void waitM (double time){
        robot.pathTimer.reset();
        while (robot.pathTimer.milliseconds() < time){

        }
    }
    public void spindexerUp() {
        robot.spindexerLinkageServo.setPosition(RobotConstants.Spindexer.spindexerLinkageServoUp);
    }
    public void spindexerDown() {
        robot.spindexerLinkageServo.setPosition(RobotConstants.Spindexer.spindexerLinkageServoDown);
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


    public void shooterOn() {
        robot.shooterOne.setPower(RobotConstants.Drivetrain.shooterLongOn);
    }
    public void shooterOff() {
        robot.shooterOne.setPower(RobotConstants.Drivetrain.shooterOff);
    }
    public void shooterReverse() {
        robot.shooterOne.setPower(RobotConstants.Drivetrain.shooterReverse);
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




    public void autoIntake (){
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

    public void sorting (){
        setSortingState(SortingStates.Checking);
        switch (sortState){
            case Checking:
                // Green Purple Purple
                if (robot.aprilID == 21){
                    waitM(750);
                    if (robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                        waitM(250);
                        robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseTwo);
                        waitM(750);
                        if (robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                            waitM(250);
                            robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
                            waitM(500);
                            robot.spindexer.spindexerUp();
                            waitM(500);
                            robot.spindexer.spindexerDown();
                            waitM(250);
                            robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseTwo);
                            waitM(500);
                            robot.spindexer.spindexerUp();
                            waitM(500);
                            robot.spindexer.spindexerDown();
                            waitM(250);
                            robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseThree);
                            waitM(500);
                            robot.spindexer.spindexerUp();
                            waitM(500);
                            robot.spindexer.spindexerDown();
                            setSortingState(SortingStates.Exit);
                        }
                        else if (robot.colorSensorOne_1.green() > robot.colorSensorOne_1.blue()){
                            waitM(500);
                            robot.spindexer.spindexerUp();
                            waitM(500);
                            robot.spindexer.spindexerDown();
                            waitM(250);
                            robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseThree);
                            waitM(500);
                            robot.spindexer.spindexerUp();
                            waitM(500);
                            robot.spindexer.spindexerDown();
                            waitM(250);
                            robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
                            waitM(500);
                            robot.spindexer.spindexerUp();
                            waitM(500);
                            robot.spindexer.spindexerDown();
                            setSortingState(SortingStates.Exit);
                        }
                    }
                    else {
                        waitM(500);
                        robot.spindexer.spindexerUp();
                        waitM(500);
                        robot.spindexer.spindexerDown();
                        waitM(250);
                        robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseTwo);
                        waitM(500);
                        robot.spindexer.spindexerUp();
                        waitM(500);
                        robot.spindexer.spindexerDown();
                        waitM(250);
                        robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
                        waitM(500);
                        robot.spindexer.spindexerUp();
                        waitM(500);
                        robot.spindexer.spindexerDown();
                        waitM(250);
                        setSortingState(SortingStates.Exit);
                    }
                }
                // Purple Green Purple
                if (robot.aprilID == 22){
                    waitM(750);
                    if (robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                        waitM(500);
                        robot.spindexer.spindexerUp();
                        waitM(500);
                        robot.spindexer.spindexerDown();
                        waitM(250);
                        robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseTwo);
                        waitM(750);
                        if (robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                            waitM(250);
                            robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
                            waitM(500);
                            robot.spindexer.spindexerUp();
                            waitM(500);
                            robot.spindexer.spindexerDown();
                            waitM(250);
                            robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseTwo);
                            waitM(500);
                            robot.spindexer.spindexerUp();
                            waitM(500);
                            robot.spindexer.spindexerDown();
                            setSortingState(SortingStates.Exit);
                        }
                        else if (robot.colorSensorOne_1.green() > robot.colorSensorOne_1.blue()){
                            waitM(500);
                            robot.spindexer.spindexerUp();
                            waitM(500);
                            robot.spindexer.spindexerDown();
                            waitM(250);
                            robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
                            waitM(500);
                            robot.spindexer.spindexerUp();
                            waitM(500);
                            robot.spindexer.spindexerDown();
                            setSortingState(SortingStates.Exit);
                        }
                    }
                    else {
                        waitM(250);
                        robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseTwo);
                        waitM(500);
                        robot.spindexer.spindexerUp();
                        waitM(500);
                        robot.spindexer.spindexerDown();
                        waitM(250);
                        robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseThree);
                        waitM(500);
                        robot.spindexer.spindexerUp();
                        waitM(500);
                        robot.spindexer.spindexerDown();
                        waitM(250);
                        robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
                        waitM(500);
                        robot.spindexer.spindexerUp();
                        waitM(500);
                        robot.spindexer.spindexerDown();
                        setSortingState(SortingStates.Exit);
                    }
                }
                // Purple Purple Green
                if (robot.aprilID == 23){
                    waitM(750);
                    if (robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                        waitM(500);
                        robot.spindexer.spindexerUp();
                        waitM(500);
                        robot.spindexer.spindexerDown();
                        waitM(250);
                        robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseTwo);
                        waitM(750);
                        if (robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green()){
                            waitM(500);
                            robot.spindexer.spindexerUp();
                            waitM(500);
                            robot.spindexer.spindexerDown();
                            waitM(250);
                            robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
                            waitM(500);
                            robot.spindexer.spindexerUp();
                            waitM(500);
                            robot.spindexer.spindexerDown();
                            setSortingState(SortingStates.Exit);
                        }
                        else if (robot.colorSensorOne_1.green() > robot.colorSensorOne_1.blue()){
                            waitM(250);
                            robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
                            waitM(500);
                            robot.spindexer.spindexerUp();
                            waitM(500);
                            robot.spindexer.spindexerDown();
                            waitM(250);
                            robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseTwo);
                            waitM(500);
                            robot.spindexer.spindexerUp();
                            waitM(500);
                            robot.spindexer.spindexerDown();
                            setSortingState(SortingStates.Exit);
                        }
                    }
                    else {
                        waitM(250);
                        robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseTwo);
                        waitM(500);
                        robot.spindexer.spindexerUp();
                        waitM(500);
                        robot.spindexer.spindexerDown();
                        waitM(250);
                        robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
                        waitM(500);
                        robot.spindexer.spindexerUp();
                        waitM(500);
                        robot.spindexer.spindexerDown();
                        waitM(250);
                        robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseThree);
                        waitM(500);
                        robot.spindexer.spindexerUp();
                        waitM(500);
                        robot.spindexer.spindexerDown();
                        setSortingState(SortingStates.Exit);
                    }
                }
            case Exit:
                break;

        }
    }



                }

