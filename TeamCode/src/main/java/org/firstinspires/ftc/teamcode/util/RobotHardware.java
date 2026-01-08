package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
//import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.pedroPathing.autoPaths.blueSorted;



import java.util.ArrayList;
import java.util.List;
import java.util.Timer;

@Config
public class RobotHardware {

    // Intake variables
    public DcMotorEx intakeMotor;
    public Servo intakeServo;
    // Spindexer variables
    public Servo spindexerLinkageServo;
    public Servo spindexerServo;
    public AnalogInput spindexerServoInput;
    public PIDFController spindexerServoPID;
    public double power;
    public double target;


//    public AbsoluteAnalogEncoder spindexerEncoder;
//    public CRServoEx spindexerServo;
    public boolean isShootingOne;
    public boolean isShootingTwo;
    public boolean isShootingThree;
    public double prevPose;
    public double prevTarget;
    public double currentPose;
    public double turns;
    public int shotCounter;
    public DigitalChannel touchSensor;
    public DigitalChannel magneticLimitSensor;
    public ColorSensor colorSensorOne_1;
    public ColorSensor colorSensorOne_2;
    public ColorSensor colorSensorTwo_1;
    public ColorSensor colorSensorTwo_2;
    public ColorSensor colorSensorNA;
    public ColorSensor colorSensorThree_1;
    public ColorSensor colorSensorThree_2;
    public boolean atPoseOne, atPoseTwo, atPoseThree;
    public ElapsedTime pathTimer;

    public com.pedropathing.util.Timer shootTimer;
    public PIDFCoefficients pid = new PIDFCoefficients(1.1958759124,0.1195875912,0,11.9587591241);

    // Drivetrain variables
    public DcMotorEx leftFront, leftRear, rightFront, rightRear;
    public IMU imu;
    public Limelight3A limelight;
    public YawPitchRollAngles orientation;
    public LLResult llResult;
    public boolean limelightTemp;
    public boolean limelightLoopTemp;
    public com.pedropathing.util.Timer llResetTimer;
    public static int aprilID;
    public Servo turretServo;
    public DcMotor shooterOne;
    public DcMotor shooterTwo;
    public Servo adjustableHoodServo;
    public Motor m_shooterOne;
    public Motor m_shooterTwo;
    public MotorGroup shooterMotors;
    public static double kP = 20;
    public static double kV = 0.7;
    public List<LynxModule> hubs;
    public Timer timerTeleOp;
    public boolean shootTemp;

    // Hardware variables
    private HardwareMap hardwareMap;
    private static RobotHardware instance = null;
    public Mecanum mecanum;
    private boolean enabled;

    public GamepadEx driver;

    //Subsystems
    public Intake intake;

    public Spindexer spindexer;
    public timerTaskCommands timerTaskCommands;
    public blueSorted blueSorted;
    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(final HardwareMap hardwareMap, GamepadEx driver) {
        this.hardwareMap = hardwareMap;
        this.driver = driver;

        //Intake setup
        this.intakeMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Intake.intakeMotor);
        this.intakeServo = hardwareMap.servo.get(RobotConstants.Intake.intakeServo);
        this.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.intakeMotor.setPower(RobotConstants.Intake.intakeMotorOff);
        this.intakeServo.setPosition(RobotConstants.Intake.intakeServoUp);



        // Drivetrain setup
        leftFront = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.leftFront);
        leftRear = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.leftRear);
        rightRear = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.rightRear);
        rightFront = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.rightFront);

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT, //
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
        imu.resetYaw();
        this.timerTeleOp = new Timer();



        // Spindexer setup
        this.spindexerLinkageServo = hardwareMap.servo.get(RobotConstants.Spindexer.spindexerLinkageServo);
        this.spindexerLinkageServo.setPosition(RobotConstants.Spindexer.spindexerLinkageServoDown);
        this.spindexerServo = hardwareMap.servo.get(RobotConstants.Spindexer.spindexerServo);
        this.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
        this.atPoseOne = true;
        this.atPoseTwo = false;
        this.atPoseThree = false;

//        this.spindexerServoInput = hardwareMap.get(AnalogInput.class, RobotConstants.Spindexer.spindexerServoInput);
//        this.spindexerServo = hardwareMap.crservo.get(RobotConstants.Spindexer.spindexerServo);
//        this.spindexerServoPID = new PIDFController(RobotConstants.Spindexer.spinP,RobotConstants.Spindexer.spinI,RobotConstants.Spindexer.spinD,RobotConstants.Spindexer.spinF);
//        this.prevPose = RobotConstants.Spindexer.spindexerServoPoseTwo;
//        this.currentPose = RobotConstants.Spindexer.spindexerServoPoseTwo;
//        this.target = RobotConstants.Spindexer.spindexerServoPoseTwo;
//        this.turns = 0;


        this.touchSensor = hardwareMap.get(DigitalChannel.class,RobotConstants.Spindexer.touchSensor);
        this.touchSensor.setMode(DigitalChannel.Mode.INPUT);

        this.magneticLimitSensor = hardwareMap.get(DigitalChannel.class, RobotConstants.Spindexer.magneticLimitSensor);
        this.magneticLimitSensor.setMode(DigitalChannel.Mode.INPUT);

        this.colorSensorOne_1 = hardwareMap.get(ColorSensor.class, RobotConstants.Spindexer.colorSensorOne_1);
        this.colorSensorTwo_1 = hardwareMap.get(ColorSensor.class, RobotConstants.Spindexer.colorSensorTwo_1);
        this.colorSensorThree_1 = hardwareMap.get(ColorSensor.class, RobotConstants.Spindexer.colorSensorThree_1);
        this.colorSensorOne_2 = hardwareMap.get(ColorSensor.class, RobotConstants.Spindexer.colorSensorOne_2);
        this.colorSensorTwo_2 = hardwareMap.get(ColorSensor.class, RobotConstants.Spindexer.colorSensorTwo_2);
        this.colorSensorThree_2 = hardwareMap.get(ColorSensor.class, RobotConstants.Spindexer.colorSensorThree_2);


        // Limelight and turret setup
        limelight = hardwareMap.get(Limelight3A.class,RobotConstants.Drivetrain.limelight);
        limelight.pipelineSwitch(3);
        pathTimer = new ElapsedTime();
        shootTimer = new com.pedropathing.util.Timer();
        imu = hardwareMap.get(IMU.class, "imu");
    RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revOrientation));
        orientation = imu.getRobotYawPitchRollAngles();
        llResult = limelight.getLatestResult();
        llResetTimer = new com.pedropathing.util.Timer();

        this.turretServo = hardwareMap.servo.get(RobotConstants.Drivetrain.turret);
        this.turretServo.setPosition(RobotConstants.Drivetrain.turretPose);

        this.shooterOne = hardwareMap.get(DcMotor.class, RobotConstants.Drivetrain.shooterOne);
        this.shooterOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shooterOne.setDirection(DcMotor.Direction.REVERSE);
        this.shooterTwo = hardwareMap.get(DcMotor.class, RobotConstants.Drivetrain.shooterTwo);
        this.shooterTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.m_shooterOne = new Motor(hardwareMap, "shooterOne", Motor.GoBILDA.BARE);
        this.shooterOne = m_shooterOne.motor;
        this.m_shooterTwo = new Motor(hardwareMap, "shooterTwo", Motor.GoBILDA.BARE);
        this.shooterTwo = m_shooterTwo.motor;
        this.shooterMotors = new MotorGroup(this.m_shooterOne,this.m_shooterTwo);

        this.shooterMotors.setRunMode(Motor.RunMode.RawPower);
        this.hubs = hardwareMap.getAll(LynxModule.class);
        this.hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        this.adjustableHoodServo = hardwareMap.servo.get(RobotConstants.Drivetrain.adjustableHoodServo);
        this.adjustableHoodServo.setPosition(RobotConstants.Drivetrain.hoodPoseMid);

        mecanum = new Mecanum();
        intake = new Intake();
        spindexer = new Spindexer();
        timerTaskCommands = new timerTaskCommands();
        blueSorted = new blueSorted();

    }

    public void init(final HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;



        //Intake setup
        this.intakeMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Intake.intakeMotor);
        this.intakeMotor.setPower(RobotConstants.Intake.intakeMotorOff);
        this.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.intakeServo = hardwareMap.servo.get(RobotConstants.Intake.intakeServo);
        this.intakeServo.setPosition(RobotConstants.Intake.intakeServoUp);



        // Drivetrain setup
        leftFront = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.leftFront);
        leftRear = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.leftRear);
        rightRear = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.rightRear);
        rightFront = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.rightFront);

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT, //
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
        imu.resetYaw();
        isShootingOne = true;
        isShootingTwo = true;
        isShootingThree = true;
        shotCounter = 0;



        // Spindexer setup
        this.spindexerLinkageServo = hardwareMap.servo.get(RobotConstants.Spindexer.spindexerLinkageServo);
        this.spindexerLinkageServo.setPosition(RobotConstants.Spindexer.spindexerLinkageServoDown);
        this.spindexerServo = hardwareMap.servo.get(RobotConstants.Spindexer.spindexerServo);
        this.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseThree);
        //spindexer.setPoseThree();

        this.touchSensor = hardwareMap.get(DigitalChannel.class,RobotConstants.Spindexer.touchSensor);
        this.touchSensor.setMode(DigitalChannel.Mode.INPUT);

        this.magneticLimitSensor = hardwareMap.get(DigitalChannel.class, RobotConstants.Spindexer.magneticLimitSensor);
        this.magneticLimitSensor.setMode(DigitalChannel.Mode.INPUT);

        this.colorSensorOne_1 = hardwareMap.get(ColorSensor.class, RobotConstants.Spindexer.colorSensorOne_1);
        this.colorSensorTwo_1 = hardwareMap.get(ColorSensor.class, RobotConstants.Spindexer.colorSensorTwo_1);
        this.colorSensorThree_1 = hardwareMap.get(ColorSensor.class, RobotConstants.Spindexer.colorSensorThree_1);
        this.colorSensorOne_2 = hardwareMap.get(ColorSensor.class, RobotConstants.Spindexer.colorSensorOne_2);
        this.colorSensorTwo_2 = hardwareMap.get(ColorSensor.class, RobotConstants.Spindexer.colorSensorTwo_2);
        this.colorSensorThree_2 = hardwareMap.get(ColorSensor.class, RobotConstants.Spindexer.colorSensorThree_2);


        // Limelight and turret setup
        limelight = hardwareMap.get(Limelight3A.class,RobotConstants.Drivetrain.limelight);
        limelight.pipelineSwitch(0);
        aprilID = 0;
        this.pathTimer = new ElapsedTime();
        shootTimer = new com.pedropathing.util.Timer();
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revOrientation));
        orientation = imu.getRobotYawPitchRollAngles();
        llResult = limelight.getLatestResult();
        llResetTimer = new com.pedropathing.util.Timer();

        this.turretServo = hardwareMap.servo.get(RobotConstants.Drivetrain.turret);
        //this.turretServo.setPosition(RobotConstants.Drivetrain.turretAutoPose);

        this.shooterOne = hardwareMap.get(DcMotor.class, RobotConstants.Drivetrain.shooterOne);
        this.shooterOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shooterOne.setDirection(DcMotor.Direction.REVERSE);
        this.shooterTwo = hardwareMap.get(DcMotor.class, RobotConstants.Drivetrain.shooterTwo);
        this.shooterTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.m_shooterOne = new Motor(hardwareMap, "shooterOne", Motor.GoBILDA.BARE);
        this.shooterOne = m_shooterOne.motor;
        this.m_shooterTwo = new Motor(hardwareMap, "shooterTwo", Motor.GoBILDA.BARE);
        this.shooterTwo = m_shooterTwo.motor;
        this.shooterMotors = new MotorGroup(this.m_shooterOne,this.m_shooterTwo);

        this.shooterMotors.setRunMode(Motor.RunMode.RawPower);
        this.hubs = hardwareMap.getAll(LynxModule.class);
        this.hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        this.adjustableHoodServo = hardwareMap.servo.get(RobotConstants.Drivetrain.adjustableHoodServo);
        this.adjustableHoodServo.setPosition(RobotConstants.Drivetrain.hoodPoseAuto);

        mecanum = new Mecanum();
        intake = new Intake();
        spindexer = new Spindexer();
        timerTaskCommands = new timerTaskCommands();
        blueSorted = new blueSorted();
    }



}
