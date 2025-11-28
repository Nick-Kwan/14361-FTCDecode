package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;


public class RobotConstants {
    @Config
    public static class Drivetrain {
        // Naming
        public static String leftFront = "leftFront";
        public static String leftRear = "leftRear";
        public static String rightFront = "rightFront";
        public static String rightRear = "rightRear";
        public static String limelight = "limelight";
        public static String turret = "turret";
        public static String shooter = "shooter";
        public static double shooterLongOn = -0.55;
        public static double shooterShortOn = -0.47;
        public static double shooterEightOn = -0.8;
        public static double shooterAuto = -0.57;
        public static double shooterShortAuto = -0.4;
        public static double shooterFullOn = -1;
        public static double shooterOff = 0;
        public static double shooterReverse = 1;
        public static double turretPose = 0.5;
    }

    @Config
    public static class Intake {
        // Naming
        public static String intakeMotor = "intakeMotor";
        public static String intakeServo = "intakeServo";

        // Values
        public static double intakeMotorON = 0.7;
        public static int intakeMotorOff = 0;
        public static double intakeMotorReverse = -1.0;
        public static double intakeServoDown = 1.0; //need to tune value
        public static double intakeServoUp = 0.25; //need to tune value
    }

    @Config
    public static class Spindexer {
        // Naming
        public static String spindexerLinkageServo = "spindexerLinkageServo";
        public static String spindexerServo = "spindexerServo";
        public static String touchSensor = "touchSensor";
        public static String magneticLimitSensor = "magneticLimitSensor";
        public static String colorSensorOne = "colorSensorOne";
        public static String colorSensorTwo = "colorSensorTwo";

        // Values
        public static double spindexerLinkageServoDown = 0.62;
        public static double spindexerLinkageServoUp = 0;
        // Change values
        public static double spindexerServoPoseOne = 0.006;
        public static double spindexerServoPoseTwo = 0.38
                ;
        public static double spindexerServoPoseThree = 0.754;
        public static double spindexerServoPoseFour = 1;
    }

    @Config
    public static class Auto {
        public static double initXB = 32.5;
        public static double initYB = 135;
        public double getInitXB (){
            return initXB;
        }
        public double getInitYB (){
            return initYB;
        }
        public static double initXR = 110.5;
        public static double initYR = 135;
        public double getInitXR (){
            return initXR;
        }
        public double getInitYR (){
            return initYR;
        }
    }

}
