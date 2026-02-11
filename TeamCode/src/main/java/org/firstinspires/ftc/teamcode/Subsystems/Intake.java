package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import com.arcrobotics.ftclib.command.Subsystem;

import java.util.TimerTask;

public class Intake implements Subsystem{
    private RobotHardware robot;
    public Intake() {
        this.robot = RobotHardware.getInstance();
    }

    public void startIntaking(){
        robot.intakeMotor.setPower(RobotConstants.Intake.intakeMotorON);
    }

    public void startIntakingMax(){
        robot.intakeMotor.setPower(RobotConstants.Intake.intakeMotorFull);
    }
    public void stopIntaking(){
        robot.intakeMotor.setPower(RobotConstants.Intake.intakeMotorOff);
    }

    public void reverseIntaking() {
        robot.intakeMotor.setPower(RobotConstants.Intake.intakeMotorReverse);
    }

    public void intakeDown(){
        robot.intakeServo.setPosition(RobotConstants.Intake.intakeServoDown);
    }

    public void intakeUp(){
        robot.intakeServo.setPosition(RobotConstants.Intake.intakeServoUp);
    }


    public void setTargetVelocity(double t){
        robot.target = t;
    }

    public void setTargetAngle(double a){
        robot.hoodAngle = a;
    }

    public void setTargetTurretPos(double p){
        robot.turretPos = p;
    }

    public void setShooterVelocity(double t){
        robot.shooterMotors.set(t);
    }

    public void setAutoShooterVelocity(){
        robot.shooterMotors.set(robot.target);
    }

    public void setHoodAngle(double a){
        robot.adjustableHoodServo.setPosition(a);
    }

    public void setTurretPos(){
        robot.turretServo.setPosition(robot.turretPos);
    }

    public void timerTaskSetup(){
        robot.timer1 = new java.util.Timer();
        robot.timer2 = new java.util.Timer();
        robot.spindexderUp1 = new TimerTask() {
            @Override
            public void run() {
                robot.spindexer.spindexerUp();
            }
        };
        robot.spindexderUp2 = new TimerTask() {
            @Override
            public void run() {
                robot.spindexer.spindexerUp();
            }
        };
        robot.spindexderUp3 = new TimerTask() {
            @Override
            public void run() {
                robot.spindexer.spindexerUp();
            }
        };
        robot.spindexderDown1 = new TimerTask() {
            @Override
            public void run() {
                robot.spindexer.spindexerDown();
            }
        };
        robot.spindexderDown2 = new TimerTask() {
            @Override
            public void run() {
                robot.spindexer.spindexerDown();
            }
        };
        robot.spindexderDown3 = new TimerTask() {
            @Override
            public void run() {
                robot.spindexer.spindexerDown();
            }
        };
        robot.spindexderSetPoseOne = new TimerTask() {
            @Override
            public void run() {
                robot.spindexer.setPoseOne();
            }
        };
        robot.spindexderSetPoseTwo = new TimerTask() {
            @Override
            public void run() {
                robot.spindexer.setPoseTwo();
            }
        };
        robot.spindexderSetPoseThree = new TimerTask() {
            @Override
            public void run() {
                robot.spindexer.setPoseThree();
            }
        };
        robot.autoIntakeIntermittence = new TimerTask() {
            @Override
            public void run() {
                robot.spindexer.autoIntake();
                robot.autoIntakeIntermittenceBool = true;
            }
        };
        robot.autoSortDelay = new TimerTask() {
            @Override
            public void run() {
                robot.spindexer.sorting();
            }
        };
    }
}
