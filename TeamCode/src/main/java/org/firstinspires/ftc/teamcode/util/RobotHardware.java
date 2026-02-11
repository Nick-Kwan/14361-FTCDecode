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

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.pedroPathing.autoPaths.blueSorted;
import org.firstinspires.ftc.teamcode.pedroPathing.autoPaths.blueSortedLong;
import org.firstinspires.ftc.teamcode.pedroPathing.autoPaths.redSorted;


import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

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
    public boolean autoIntakeIntermittenceBool;
    public TimerTask spindexderUp1, spindexderUp2, spindexderUp3, spindexderDown1, spindexderDown2, spindexderDown3, spindexderSetPoseOne, spindexderSetPoseTwo, spindexderSetPoseThree, autoIntakeIntermittence, autoSortDelay;
    public java.util.Timer timer1, timer2;
    public ScheduledExecutorService s;
    public int d;

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
    public double targetY;
    public double hoodAngle;
    public double turretPos;
    public boolean limelightTemp;
    public boolean limelightLoopTemp;
    public com.pedropathing.util.Timer llResetTimer;
    public static int aprilID;
    public Servo turretServo;
    public DcMotor shooterOne;
    public DcMotor shooterTwo;
    public PIDFController shooterPID;
    public double avgVelocity;
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
    public ShooterLUT ShooterLUT;
    private boolean enabled;

    public GamepadEx driver;

    //Subsystems
    public Intake intake;

    public Spindexer spindexer;
    public timerTaskCommands timerTaskCommands;
    public blueSorted blueSorted;
    public blueSortedLong blueSortedLong;
    public redSorted redSorted;
    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    private void initCommon(final HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

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
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
        imu.resetYaw();

        // Spindexer sensors setup
        this.spindexerLinkageServo = hardwareMap.servo.get(RobotConstants.Spindexer.spindexerLinkageServo);
        this.spindexerLinkageServo.setPosition(RobotConstants.Spindexer.spindexerLinkageServoDown);
        this.spindexerServo = hardwareMap.servo.get(RobotConstants.Spindexer.spindexerServo);

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
        pathTimer = new ElapsedTime();
        shootTimer = new com.pedropathing.util.Timer();
        orientation = imu.getRobotYawPitchRollAngles();
        llResult = limelight.getLatestResult();
        llResetTimer = new com.pedropathing.util.Timer();

        this.turretServo = hardwareMap.servo.get(RobotConstants.Drivetrain.turret);

        // Shooter motors setup
        this.m_shooterOne = new Motor(hardwareMap, "shooterOne", Motor.GoBILDA.BARE);
        this.m_shooterOne.setInverted(true);
        this.m_shooterOne.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        this.m_shooterTwo = new Motor(hardwareMap, "shooterTwo", Motor.GoBILDA.BARE);
        this.m_shooterTwo.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        this.shooterMotors = new MotorGroup(m_shooterTwo,m_shooterOne);
        this.shooterMotors.setRunMode(Motor.RunMode.VelocityControl);

        target = RobotConstants.Intake.target;
        hoodAngle = RobotConstants.Intake.hoodAngle;
        targetY = 0;

        this.hubs = hardwareMap.getAll(LynxModule.class);
        this.hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        this.adjustableHoodServo = hardwareMap.servo.get(RobotConstants.Drivetrain.adjustableHoodServo);

        d = 0;

        // Subsystem construction
        mecanum = new Mecanum();
        intake = new Intake();
        spindexer = new Spindexer();
        ShooterLUT = new ShooterLUT();
        timerTaskCommands = new timerTaskCommands();
        blueSorted = new blueSorted();
        redSorted = new redSorted();
    }

    public void init(final HardwareMap hardwareMap, GamepadEx driver) {
        initCommon(hardwareMap);
        this.driver = driver;

        this.timerTeleOp = new Timer();
        this.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
        this.atPoseOne = true;
        this.atPoseTwo = false;
        this.atPoseThree = false;

        limelight.pipelineSwitch(3);
        this.turretServo.setPosition(RobotConstants.Drivetrain.turretPose);
        this.adjustableHoodServo.setPosition(RobotConstants.Drivetrain.hoodPoseMid);

        this.shooterMotors.setVeloCoefficients(RobotConstants.Drivetrain.shootP,RobotConstants.Drivetrain.shootI,RobotConstants.Drivetrain.shootD);
        this.shooterMotors.setFeedforwardCoefficients(0,RobotConstants.Drivetrain.shootV);

        autoIntakeIntermittenceBool = true;

        s = Executors.newScheduledThreadPool(2);
    }

    public void init(final HardwareMap hardwareMap) {
        initCommon(hardwareMap);

        this.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseTwo);
        isShootingOne = true;
        isShootingTwo = true;
        isShootingThree = true;
        shotCounter = 0;

        limelight.pipelineSwitch(0);
        aprilID = 0;
        this.adjustableHoodServo.setPosition(RobotConstants.Drivetrain.hoodPoseAuto);

        this.shooterMotors.setVeloCoefficients(0.4,RobotConstants.Drivetrain.shootI,RobotConstants.Drivetrain.shootD);

        s = Executors.newScheduledThreadPool(1);

        blueSortedLong = new blueSortedLong();
    }

}
