package utility;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.List;

import Constants.IntakeConstants;
import Constants.NamingConstants;
import Constants.OdometryConstants;
import Constants.SpindexerConstants;

public class RobotHardware {

    // Drivetrain
    public DcMotorEx leftFront, leftRear, rightFront, rightRear;
    public IMU imu;

    // Pinpoint odometry
    public GoBildaPinpointDriver pinpoint;

    // Cached pose fields (updated each loop via updateCachedPose)
    public double cachedPoseX = 0;
    public double cachedPoseY = 0;
    public double cachedHeading = 0;
    public double cachedVelX = 0;
    public double cachedVelY = 0;
    public double cachedHeadingVel = 0;

    // Intake
    public DcMotorEx intakeMotor;
    public Servo intakeServo;
    public Servo LEDlight;

    // Shooter (FTCLib motors)
    public Motor m_shooterOne, m_shooterTwo;
    public MotorGroup shooterMotors;
    public Servo adjustableHoodServo;

    // Turret
    public ServoImplEx turretServo;

    // Spindexer
    public CRServo spindexerCRServo;
    public Servo spindexerLinkageServo;
    public DigitalChannel touchSensor;
    public DigitalChannel magneticLimitSensor;

    // Color Sensors
    public ColorSensor colorSensorOne_1, colorSensorOne_2;
    public ColorSensor colorSensorTwo_1, colorSensorTwo_2;
    public ColorSensor colorSensorThree_1, colorSensorThree_2;

    // Vision
    public Limelight3A limelight;

    // Bulk caching
    public List<LynxModule> hubs;

    // Gamepad
    public GamepadEx driver;

    // Thread-safe singleton
    private static volatile RobotHardware instance = null;
    private static final Object lock = new Object();

    public static RobotHardware getInstance() {
        if (instance == null) {
            synchronized (lock) {
                if (instance == null) {
                    instance = new RobotHardware();
                }
            }
        }
        return instance;
    }

    public void init(HardwareMap hardwareMap, GamepadEx driver) {
        this.driver = driver;
        init(hardwareMap);
    }

    public void init(HardwareMap hardwareMap) {
        // Bulk caching
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Drivetrain
        leftFront = hardwareMap.get(DcMotorEx.class, NamingConstants.LEFT_FRONT);
        leftRear = hardwareMap.get(DcMotorEx.class, NamingConstants.LEFT_REAR);
        rightFront = hardwareMap.get(DcMotorEx.class, NamingConstants.RIGHT_FRONT);
        rightRear = hardwareMap.get(DcMotorEx.class, NamingConstants.RIGHT_REAR);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                OdometryConstants.LOGO_DIRECTION,
                OdometryConstants.USB_DIRECTION
        ));
        imu.initialize(parameters);
        imu.resetYaw();

        // Intake
        intakeMotor = hardwareMap.get(DcMotorEx.class, NamingConstants.INTAKE_MOTOR);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setPower(0);
        intakeServo = hardwareMap.servo.get(NamingConstants.INTAKE_SERVO);
        intakeServo.setPosition(IntakeConstants.SERVO_UP);

        // Shooter motors (FTCLib)
        m_shooterOne = new Motor(hardwareMap, NamingConstants.SHOOTER_ONE, Motor.GoBILDA.BARE);
        m_shooterOne.setInverted(true);
        m_shooterOne.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        m_shooterTwo = new Motor(hardwareMap, NamingConstants.SHOOTER_TWO, Motor.GoBILDA.BARE);
        m_shooterTwo.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooterMotors = new MotorGroup(m_shooterTwo, m_shooterOne);
        shooterMotors.setRunMode(Motor.RunMode.VelocityControl);

        // Hood and turret servos
        adjustableHoodServo = hardwareMap.servo.get(NamingConstants.ADJUSTABLE_HOOD_SERVO);
        turretServo = hardwareMap.get(ServoImplEx.class, NamingConstants.TURRET);
        turretServo.setPwmRange(new PwmControl.PwmRange(500, 2500)); // Full range for Axon servo

        // Spindexer
        spindexerCRServo = hardwareMap.get(CRServo.class, NamingConstants.SPINDEXER_CR_SERVO);
        spindexerCRServo.setPower(0);
        spindexerLinkageServo = hardwareMap.servo.get(NamingConstants.SPINDEXER_LINKAGE_SERVO);
        spindexerLinkageServo.setPosition(SpindexerConstants.LINKAGE_DOWN);

        // Digital sensors
        touchSensor = hardwareMap.get(DigitalChannel.class, NamingConstants.TOUCH_SENSOR);
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        magneticLimitSensor = hardwareMap.get(DigitalChannel.class, NamingConstants.MAGNETIC_LIMIT_SENSOR);
        magneticLimitSensor.setMode(DigitalChannel.Mode.INPUT);

        // Color sensors
        colorSensorOne_1 = hardwareMap.get(ColorSensor.class, NamingConstants.COLOR_SENSOR_ONE_1);
        colorSensorOne_2 = hardwareMap.get(ColorSensor.class, NamingConstants.COLOR_SENSOR_ONE_2);
        colorSensorTwo_1 = hardwareMap.get(ColorSensor.class, NamingConstants.COLOR_SENSOR_TWO_1);
        colorSensorTwo_2 = hardwareMap.get(ColorSensor.class, NamingConstants.COLOR_SENSOR_TWO_2);
        colorSensorThree_1 = hardwareMap.get(ColorSensor.class, NamingConstants.COLOR_SENSOR_THREE_1);
        colorSensorThree_2 = hardwareMap.get(ColorSensor.class, NamingConstants.COLOR_SENSOR_THREE_2);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, NamingConstants.LIMELIGHT);

        // Pinpoint odometry — Pedro configures encoder directions, offsets, and IMU reset
        // in its constructor. Only set yawScalar here (Pedro doesn't configure this).
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, NamingConstants.PINPOINT);
        pinpoint.setYawScalar(OdometryConstants.yawScalar);

        // LED
        LEDlight = hardwareMap.servo.get(NamingConstants.LEDLIGHT);
        LEDlight.setPosition(0.722);
    }

    /** Read current pose from pinpoint and cache all values for this loop iteration */
    public void updateCachedPose() {
        cachedPoseX = pinpoint.getPosX(DistanceUnit.INCH);
        cachedPoseY = pinpoint.getPosY(DistanceUnit.INCH);
        cachedHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        cachedVelX = pinpoint.getVelX(DistanceUnit.INCH);
        cachedVelY = pinpoint.getVelY(DistanceUnit.INCH);
        cachedHeadingVel = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }

    public void clearBulkCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }
}
