package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.States.ShooterStates.Four;
import static org.firstinspires.ftc.teamcode.States.ShooterStates.Three;
import static org.firstinspires.ftc.teamcode.States.ShooterStates.Two;

import org.firstinspires.ftc.teamcode.States.SecondShooterStates;
import org.firstinspires.ftc.teamcode.States.ShooterStates;
import org.firstinspires.ftc.teamcode.States.SpindexerStates;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.util.RobotConstants;

import com.arcrobotics.ftclib.command.Subsystem;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class Spindexer implements Subsystem{
    private RobotHardware robot;
    public static final Spindexer INSTANCE = new Spindexer();

    public Spindexer(){
        this.robot = RobotHardware.getInstance();
    }
    public ShooterStates shootingState = ShooterStates.One;
    public SecondShooterStates secondShootingState = SecondShooterStates.One;
    public ElapsedTime shooterTimer = new ElapsedTime();
    public ElapsedTime secondShooterTimer = new ElapsedTime();
    public ElapsedTime thirdShooterTimer = new ElapsedTime();
    public ElapsedTime time = new ElapsedTime();
    private ScheduledExecutorService s = Executors.newScheduledThreadPool(1);
    private boolean shooterTimerTemp;
    public void setSecondShootingState(SecondShooterStates state) {
        secondShootingState = state;
    }
    public void setSpindexerState(ShooterStates state) {
        shootingState = state;
    }

    public enum DetectedColor{
        RED,
        BLUE,
        YELLOW,
        UNKOWN
    }
    public void spindexerUp() {
        robot.spindexerLinkageServo.setPosition(RobotConstants.Spindexer.spindexerLinkageServoUp);
    }
    public void spindexerDown() {
        robot.spindexerLinkageServo.setPosition(RobotConstants.Spindexer.spindexerLinkageServoDown);
    }

    public void shooterOn() {
        robot.shooter.setPower(RobotConstants.Drivetrain.shooterLongOn);
    }
    public void shooterOff() {
        robot.shooter.setPower(RobotConstants.Drivetrain.shooterOff);
    }
    public void shooterReverse() {
        robot.shooter.setPower(RobotConstants.Drivetrain.shooterReverse);
    }

    public boolean getTouchSensorState() {
        return robot.touchSensor.getState();
    }
    public boolean isLimitSwitchClosed(){
        return !robot.magneticLimitSensor.getState();
    }


    public double detectColor (){
        double allColor = robot.colorSensorOne.red() + robot.colorSensorOne.green() + robot.colorSensorOne.blue();
        return allColor;
    }

    public void shootingOne () {
        switch (shootingState) {


            case One:

                switch (secondShootingState) {
                    case One:
                        if (shooterTimerTemp) {
                            shooterTimer.reset();
                            shooterTimerTemp = false;
                        }
                        if (shooterTimer.seconds() > 1) {
                            robot.spindexer.spindexerUp();
                            shooterTimerTemp = true;
                            shooterTimer.reset();
                            setSecondShootingState(SecondShooterStates.Two);
                        }
                    case Two:
                        if (shooterTimerTemp) {
                            shooterTimer.reset();
                            shooterTimerTemp = false;
                        }
                        if (shooterTimer.seconds() > 1) {
                            robot.spindexer.spindexerDown();
                            shooterTimerTemp = true;
                            shooterTimer.reset();
                            setSecondShootingState(SecondShooterStates.Three);
                        }
                    case Three:
                        if (shooterTimerTemp) {
                            shooterTimer.reset();
                            shooterTimerTemp = false;
                        }
                        if (shooterTimer.seconds() > 1) {
                            robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseTwo);
                            shooterTimerTemp = true;
                            secondShooterTimer.reset();
                            robot.shotCounter ++;
                            //robot.isShootingOne = false;
                            setSpindexerState(Two);
                            break;
                        }
                }
            case Two:
                if (robot.shotCounter == 1) {
                    break;
                }
                break;
        }
    }

    public void shootingTwo () {
        switch (shootingState) {


            case One:

                switch (secondShootingState) {
                    case One:
                        if (shooterTimerTemp) {
                            shooterTimer.reset();
                            shooterTimerTemp = false;
                        }
                        if (shooterTimer.seconds() > 1) {
                            robot.spindexer.spindexerUp();
                            shooterTimerTemp = true;
                            shooterTimer.reset();
                            setSecondShootingState(SecondShooterStates.Two);
                        }
                    case Two:
                        if (shooterTimerTemp) {
                            shooterTimer.reset();
                            shooterTimerTemp = false;
                        }
                        if (shooterTimer.seconds() > 1) {
                            robot.spindexer.spindexerDown();
                            shooterTimerTemp = true;
                            shooterTimer.reset();
                            setSecondShootingState(SecondShooterStates.Three);
                        }
                    case Three:
                        if (shooterTimerTemp) {
                            shooterTimer.reset();
                            shooterTimerTemp = false;
                        }
                        if (shooterTimer.seconds() > 1) {
                            robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseThree);
                            shooterTimerTemp = true;
                            secondShooterTimer.reset();
                            //robot.isShootingTwo = false;
                            robot.shotCounter ++;
                            setSpindexerState(Two);
                            break;
                        }
                }
            case Two:
                if (robot.shotCounter == 2) {
                    break;
                }
                break;
        }
    }
    public void shootingThree () {
        switch (shootingState) {


            case One:

                switch (secondShootingState) {
                    case One:
                        if (shooterTimerTemp) {
                            shooterTimer.reset();
                            shooterTimerTemp = false;
                        }
                        if (shooterTimer.seconds() > 1) {
                            robot.spindexer.spindexerUp();
                            shooterTimerTemp = true;
                            shooterTimer.reset();
                            setSecondShootingState(SecondShooterStates.Two);
                        }
                    case Two:
                        if (shooterTimerTemp) {
                            shooterTimer.reset();
                            shooterTimerTemp = false;
                        }
                        if (shooterTimer.seconds() > 1) {
                            robot.spindexer.spindexerDown();
                            shooterTimerTemp = true;
                            shooterTimer.reset();
                            setSecondShootingState(SecondShooterStates.Three);
                        }
                    case Three:
                        if (shooterTimerTemp) {
                            shooterTimer.reset();
                            shooterTimerTemp = false;
                        }
                        if (shooterTimer.seconds() > 1) {
                            robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
                            shooterTimerTemp = true;
                            secondShooterTimer.reset();
                            robot.shotCounter++;
                            //robot.isShootingThree = false;
                            setSpindexerState(Two);
                            break;
                        }
                }
            case Two:
                if (robot.shotCounter == 3) {
                    break;
                }
                break;
        }
    }
//                switch (secondShootingState) {
//                    case One:
//                        if (shooterTimerTemp) {
//                            secondShooterTimer.reset();
//                            shooterTimerTemp = false;
//                        }
//                        if (secondShooterTimer.seconds() > 1) {
//                            robot.spindexer.spindexerUp();
//                            shooterTimerTemp = true;
//                            secondShooterTimer.reset();
//                            setSecondShootingState(SecondShooterStates.Two);
//                        }
//                    case Two:
//                        if (shooterTimerTemp) {
//                            secondShooterTimer.reset();
//                            shooterTimerTemp = false;
//                        }
//                        if (secondShooterTimer.seconds() > 1) {
//                            robot.spindexer.spindexerDown();
//                            shooterTimerTemp = true;
//                            secondShooterTimer.reset();
//                            setSecondShootingState(SecondShooterStates.Three);
//                        }
//                    case Three:
//                        if (shooterTimerTemp) {
//                            secondShooterTimer.reset();
//                            shooterTimerTemp = false;
//                        }
//                        if (secondShooterTimer.seconds() > 1) {
//                            robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseThree);
//                            shooterTimerTemp = true;
//                            thirdShooterTimer.reset();
//                            setSpindexerState(Three);
//                            break;
//                        }
//                }
//            case Three:
//                switch (secondShootingState) {
//                    case One:
//                        if (shooterTimerTemp) {
//                            thirdShooterTimer.reset();
//                            shooterTimerTemp = false;
//                        }
//                        if (thirdShooterTimer.seconds() > 1) {
//                            robot.spindexer.spindexerUp();
//                            shooterTimerTemp = true;
//                            thirdShooterTimer.reset();
//                            setSecondShootingState(SecondShooterStates.Two);
//                        }
//                    case Two:
//                        if (shooterTimerTemp) {
//                            thirdShooterTimer.reset();
//                            shooterTimerTemp = false;
//                        }
//                        if (thirdShooterTimer.seconds() > 1) {
//                            robot.spindexer.spindexerDown();
//                            shooterTimerTemp = true;
//                            thirdShooterTimer.reset();
//                            setSecondShootingState(SecondShooterStates.Three);
//                        }
//                    case Three:
//                        if (shooterTimerTemp) {
//                            thirdShooterTimer.reset();
//                            shooterTimerTemp = false;
//                        }
//                        if (thirdShooterTimer.seconds() > 1) {
//                            robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
//                            shooterTimerTemp = true;
//                            //thirdShooterTimer.reset();
//                            robot.isShooting = false;
//                            setSpindexerState(Four);
//                            break;
//                        }
//                }
//
//            case Four:
//                if (!robot.isShooting){
//                    break;
//
//                }
//                setSecondShootingState(SecondShooterStates.One);
//                switch (secondShootingState) {
//                    case One:
//                        if (shooterTimerTemp) {
//                            shooterTimer.reset();
//                            shooterTimerTemp = false;
//                        }
//                        if (shooterTimer.seconds() > 1) {
//                            robot.spindexer.spindexerUp();
//                            shooterTimerTemp = true;
//                            shooterTimer.reset();
//                            setSecondShootingState(SecondShooterStates.Two);
//                        }
//                    case Two:
//                        if (shooterTimerTemp) {
//                            shooterTimer.reset();
//                            shooterTimerTemp = false;
//                        }
//                        if (shooterTimer.seconds() > 1) {
//                            robot.spindexer.spindexerDown();
//                            shooterTimerTemp = true;
//                            shooterTimer.reset();
//                            setSecondShootingState(SecondShooterStates.Three);
//                        }
//                    case Three:
//                        if (shooterTimerTemp) {
//                            shooterTimer.reset();
//                            shooterTimerTemp = false;
//                        }
//                        if (shooterTimer.seconds() > 1) {
//                            robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseThree);
//                            shooterTimerTemp = true;
//                            shooterTimer.reset();
//                            setSpindexerState(Three);
//                            break;
//                        }
//                }
//            case Three:
//                setSecondShootingState(SecondShooterStates.One);
//                switch (secondShootingState) {
//                    case One:
//                        if (shooterTimerTemp) {
//                            shooterTimer.reset();
//                            shooterTimerTemp = false;
//                        }
//                        if (shooterTimer.seconds() > 1) {
//                            robot.spindexer.spindexerUp();
//                            shooterTimerTemp = true;
//                            shooterTimer.reset();
//                            setSecondShootingState(SecondShooterStates.Two);
//                        }
//                    case Two:
//                        if (shooterTimerTemp) {
//                            shooterTimer.reset();
//                            shooterTimerTemp = false;
//                        }
//                        if (shooterTimer.seconds() > 1) {
//                            robot.spindexer.spindexerDown();
//                            shooterTimerTemp = true;
//                            shooterTimer.reset();
//                            setSecondShootingState(SecondShooterStates.Three);
//                        }
//                    case Three:
//                        if (shooterTimerTemp) {
//                            shooterTimer.reset();
//                            shooterTimerTemp = false;
//                        }
//                        if (shooterTimer.seconds() > 1) {
//                            robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
//                            shooterTimerTemp = true;
//                            shooterTimer.reset();
//                            robot.isShooting = false;
//                            setSpindexerState(Four);
//                            break;
//                        }
//                }
//            case Four:
//                if (!robot.isShooting){
//                    break;
//                }

//                if (shooterTimerTemp) {
//                    shooterTimer.resetTimer();
//                    shooterTimerTemp = false;
//                }
////                if (shooterTimer.getElapsedTimeSeconds() < 1 && robot.spindexer.getTouchSensorState()){
////                    robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
////                    robot.spindexer.spindexerUp();
////                }
//                if (shooterTimer.getElapsedTimeSeconds() > 1){
//                    robot.spindexer.spindexerUp();
//                }
//                if (shooterTimer.getElapsedTimeSeconds() > 2){
//                    robot.spindexer.spindexerDown();
//                }
//                if (shooterTimer.getElapsedTimeSeconds() > 2.5){
//                    robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseTwo);
//                    shooterTimerTemp = true;
//                    setSpindexerState(Three);
//                }

//                if (shooterTimerTemp) {
//                    shooterTimer.resetTimer();
//                    shooterTimerTemp = false;
//                }
////                if (shooterTimer.getElapsedTimeSeconds() > 0.3){
////                    robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseThree);
////                }
//                if (shooterTimer.getElapsedTimeSeconds() > 1){
//                    robot.spindexer.spindexerUp();
//                }
//                if (shooterTimer.getElapsedTimeSeconds() > 2){
//                    robot.spindexer.spindexerDown();
//                }
//                if (shooterTimer.getElapsedTimeSeconds() > 2.5){
//                    robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseThree);
//                    shooterTimerTemp = true;
//                    setSpindexerState(Four);
//                }

//                if (shooterTimerTemp) {
//                    shooterTimer.resetTimer();
//                    shooterTimerTemp = false;
//                }
////                if (shooterTimer.getElapsedTimeSeconds() > 0.3){
////                    robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
////                }
//                if (shooterTimer.getElapsedTimeSeconds() > 1){
//                    robot.spindexer.spindexerUp();
//                }
//                if (shooterTimer.getElapsedTimeSeconds() > 2){
//                    robot.spindexer.spindexerDown();
//                }
//                if (shooterTimer.getElapsedTimeSeconds() > 2.5){
//                    robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
//                    robot.isShooting = false;
//                }
                }

