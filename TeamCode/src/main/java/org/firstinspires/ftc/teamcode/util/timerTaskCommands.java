package org.firstinspires.ftc.teamcode.util;

import java.util.TimerTask;

public class timerTaskCommands{
    private RobotHardware robot;
    public timerTaskCommands() {
        this.robot = RobotHardware.getInstance();
    }

    public TimerTask spindexerUp(){
        TimerTask spindexerUp = new TimerTask() {
            public void run() {
                robot.spindexer.spindexerUp();
            }
        };
        return  spindexerUp;
    }
    public TimerTask spindexerDown(){
        TimerTask spindexerDown = new TimerTask() {
            public void run() {
                robot.spindexer.spindexerDown();
            }
        };
        return  spindexerDown;
    }
    public TimerTask setPoseOne(){
        TimerTask setPoseOne = new TimerTask() {
            public void run() {
                robot.spindexer.setPoseOne();
            }
        };
        return  setPoseOne;
    }
    public TimerTask setPoseTwo(){
        TimerTask setPoseTwo = new TimerTask() {
            public void run() {
                robot.spindexer.setPoseTwo();
            }
        };
        return  setPoseTwo;
    }
    public TimerTask setPoseThree(){
        TimerTask setPoseThree = new TimerTask() {
            public void run() {
                robot.spindexer.setPoseThree();
            }
        };
        return  setPoseThree;
    }
}
