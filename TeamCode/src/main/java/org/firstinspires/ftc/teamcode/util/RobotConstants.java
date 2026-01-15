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
        public static String shooterOne = "shooterOne";
        public static String shooterTwo = "shooterTwo";
        public static String adjustableHoodServo = "adjustableHoodServo";

        public static double shooterLongOn = 0.55;
        public static double shooterShortOn = 0.47;
        public static double shooterEightOn = -0.8;
        public static double shooterAuto = -0.57;
        public static double shooterShortAuto = -0.4;
        public static double shooterFullOn = -1;
        public static double shooterOff = 0;
        public static double shooterReverse = 1;
        public static double turretPose = 0.5;
        public static double turretPoseAuto = 0.8;
        public static double turretMeowPose = 0.3;
        public static double turretBlueAutoPose = 1;
        public static double turretRedAutoPose = 0.2;
        public static double hoodPoseMid = 0.5;
        public static double hoodPoseAuto = 0.5;
        public static double hoodPoseLong = 0.21;
        public static double hoodPoseRetract = 0;
        public static double hoodPoseFull = 1;
        public static double initX = 18.5;
        public static double initY = 114;
        public static double targetFar = 0.537;
        public static double shootP = 0.4;
        public static double shootI = 0;
        public static double shootD = 0;
        public static double shootF = 0;
        public static double shootV = 0.7;
    }

    @Config
    public static class Intake {
        // Naming
        public static String intakeMotor = "intakeMotor";
        public static String intakeServo = "intakeServo";

        // Values
        public static double intakeMotorON = 0.85;
        public static double intakeMotorFull = 0.85;
        public static int intakeMotorOff = 0;
        public static double intakeMotorReverse = -1.0;
        public static double intakeServoDown = 1.0; //need to tune value
        public static double intakeServoUp = 0.25; //need to tune value
        public static double target = 0.45;
        public static double hoodAngle = 0.5;
        public static double turretPos = 0.5;
    }

    @Config
    public static class Spindexer {
        // Naming
        public static String spindexerLinkageServo = "spindexerLinkageServo";
        public static String spindexerServo = "spindexerServo";
        public static String touchSensor = "touchSensor";
        public static String magneticLimitSensor = "magneticLimitSensor";
        public static String colorSensorOne_1 = "colorSensorOne_1";
        public static String colorSensorTwo_1 = "colorSensorTwo_1";
        public static String colorSensorThree_1 = "colorSensorThree_1";
        public static String colorSensorOne_2 = "colorSensorOne_2";
        public static String colorSensorTwo_2 = "colorSensorTwo_2";
        public static String colorSensorThree_2 = "colorSensorThree_2";
        public static String spindexerServoInput = "spindexerServoInput";
        public static String spindexerServoPID = "spindexerServoPID";

        // Values
        public static double spindexerLinkageServoDown = 0.67;
        public static double spindexerLinkageServoUp = 0;
        // Change values
        public static double spindexerServoPoseOne = 0.035;
        public static double spindexerServoPoseTwo = 0.42;
        public static double spindexerServoPoseThree = 0.8;
        public static double spindexerServoPoseFour = 1;
        public static double spinP = 0.7;
        public static double spinI = 0;
        public static double spinD = 0.001;
        public static double spinF = 0;
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
        public static double initXR = 125.5;
        public static double initYR = 114;
        public double getInitXR (){
            return initXR;
        }
        public double getInitYR (){
            return initYR;
        }
    }

}
