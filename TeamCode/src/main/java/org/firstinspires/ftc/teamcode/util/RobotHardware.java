package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

@Config
public class RobotHardware {

    // Intake variables
    public DcMotorEx intakeMotor;
    public Servo intakeServo;
    // Spindexer variables
    public Servo spindexerLinkageServo;
    public Servo spindexerServo;
    public boolean isShootingOne;
    public boolean isShootingTwo;
    public boolean isShootingThree;
    public int shotCounter;
    public DigitalChannel touchSensor;
    public DigitalChannel magneticLimitSensor;
    public ColorSensor colorSensorOne;
    public ColorSensor colorSensorTwo;

    // Drivetrain variables
    public DcMotorEx leftFront, leftRear, rightFront, rightRear;
    public IMU imu;
    public Limelight3A limelight;
    public Servo turretServo;
    public DcMotorEx shooter;

    // Hardware variables
    private HardwareMap hardwareMap;
    private static RobotHardware instance = null;
    public Mecanum mecanum;
    private boolean enabled;

    public GamepadEx driver;

    //Subsystems
    public Intake intake;

    public Spindexer spindexer;
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
        this.intakeMotor.setPower(RobotConstants.Intake.intakeMotorOff);
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



        // Spindexer setup
        this.spindexerLinkageServo = hardwareMap.servo.get(RobotConstants.Spindexer.spindexerLinkageServo);
        this.spindexerLinkageServo.setPosition(RobotConstants.Spindexer.spindexerLinkageServoDown);
        this.spindexerServo = hardwareMap.servo.get(RobotConstants.Spindexer.spindexerServo);
        this.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);

        this.touchSensor = hardwareMap.get(DigitalChannel.class,RobotConstants.Spindexer.touchSensor);
        this.touchSensor.setMode(DigitalChannel.Mode.INPUT);

        this.magneticLimitSensor = hardwareMap.get(DigitalChannel.class, RobotConstants.Spindexer.magneticLimitSensor);
        this.magneticLimitSensor.setMode(DigitalChannel.Mode.INPUT);

        this.colorSensorOne = hardwareMap.get(ColorSensor.class, RobotConstants.Spindexer.colorSensorOne);
        this.colorSensorTwo = hardwareMap.get(ColorSensor.class, RobotConstants.Spindexer.colorSensorTwo);


        // Limelight and turret setup
        limelight = hardwareMap.get(Limelight3A.class,RobotConstants.Drivetrain.limelight);
        limelight.pipelineSwitch(3);
        imu = hardwareMap.get(IMU.class, "imu");
    RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revOrientation));

        this.turretServo = hardwareMap.servo.get(RobotConstants.Drivetrain.turret);
        this.turretServo.setPosition(RobotConstants.Drivetrain.turretPose);

        this.shooter = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.shooter);

        mecanum = new Mecanum();
        intake = new Intake();
        spindexer = new Spindexer();
    }

    public void init(final HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;



        //Intake setup
        this.intakeMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Intake.intakeMotor);
        this.intakeMotor.setPower(RobotConstants.Intake.intakeMotorOff);
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

        this.touchSensor = hardwareMap.get(DigitalChannel.class,RobotConstants.Spindexer.touchSensor);
        this.touchSensor.setMode(DigitalChannel.Mode.INPUT);

        this.magneticLimitSensor = hardwareMap.get(DigitalChannel.class, RobotConstants.Spindexer.magneticLimitSensor);
        this.magneticLimitSensor.setMode(DigitalChannel.Mode.INPUT);

        this.colorSensorOne = hardwareMap.get(ColorSensor.class, RobotConstants.Spindexer.colorSensorOne);
        this.colorSensorTwo = hardwareMap.get(ColorSensor.class, RobotConstants.Spindexer.colorSensorTwo);


        // Limelight and turret setup
        limelight = hardwareMap.get(Limelight3A.class,RobotConstants.Drivetrain.limelight);
        limelight.pipelineSwitch(3);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revOrientation));

        this.turretServo = hardwareMap.servo.get(RobotConstants.Drivetrain.turret);
        this.turretServo.setPosition(RobotConstants.Drivetrain.turretPose);

        this.shooter = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.shooter);

        mecanum = new Mecanum();
        intake = new Intake();
        spindexer = new Spindexer();
    }



}
