package teleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

import Constants.DriveConstants;
import Constants.EnumConstants;
import Constants.LimelightConstants;
import Constants.ShooterConstants;
import Constants.SpindexerConstants;
import Constants.TurretConstants;
import commands.ShootingCommands;
import commands.SortedShootCommand;
import subsystems.Intake;
import subsystems.Limelight;
import subsystems.MecanumDrive;
import subsystems.Shooter;
import subsystems.Spindexer;
import subsystems.Turret;
import utility.RobotHardware;
import utility.TelemetryHelper;

/**
 * Abstract TeleOp template for command-based architecture.
 * Subclasses set alliance color and limelight pipeline.
 *
 * Preserves all control mappings from old TeleOpBlue/Red:
 * - Left trigger: slow mode (0.3x speed)
 * - PS button: reset IMU yaw
 * - Right trigger: intake deploy + start + auto-intake distribution
 * - Right bumper: intake reverse + deploy
 * - A pressed/released: linkage up/down (manual single-shot)
 * - Y pressed: shoot all 3 balls from current position
 * - DpadLeft/Right: rotate spindexer (touch sensor guarded)
 * - DpadUp: go to pose 2
 * - Left bumper: sorted shooting
 */
abstract public class TeleOpTemplate extends CommandOpMode {
    protected RobotHardware robot;
    protected MecanumDrive mecanumDrive;
    protected Intake intake;
    protected Shooter shooter;
    protected Spindexer spindexer;
    protected Turret turret;
    protected Limelight limelight;
    protected TelemetryHelper telemetryHelper;
    protected GamepadEx driverGamepad;

    // Limelight tracking state
    private boolean wasTrackingValid = false;
    private ElapsedTime llResetTimer;

    // Auto-intake distribution timing
    private boolean autoIntakeReady = true;
    private ElapsedTime autoIntakeTimer;

    // Cached IMU heading (read once per loop, shared by drive + limelight)
    private double cachedHeadingRad = 0;

    /** Subclass returns the goal-tracking pipeline (3=Blue, 2=Red) */
    protected abstract int getGoalPipeline();

    /** Subclass returns the alliance color */
    protected abstract EnumConstants.AllianceColor getAllianceColor();

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        driverGamepad = new GamepadEx(gamepad1);
        robot = RobotHardware.getInstance();
        robot.init(hardwareMap, driverGamepad);

        // Create subsystems
        mecanumDrive = new MecanumDrive();
        intake = new Intake();
        shooter = new Shooter();
        spindexer = new Spindexer();
        turret = new Turret();
        limelight = new Limelight();
        telemetryHelper = new TelemetryHelper();

        // Configure shooter PID for teleop
        shooter.configureForTeleOp();

        // Init limelight
        limelight.start();
        limelight.switchPipeline(getGoalPipeline());

        // Init turret to center
        turret.center();

        // Init spindexer to pose 1
        spindexer.setPoseOne();

        // Init hood to mid
        shooter.setHoodAngle(ShooterConstants.HOOD_POSE_MID);

        // Timers
        llResetTimer = new ElapsedTime();
        autoIntakeTimer = new ElapsedTime();

        // Register subsystems with the command scheduler
        register(mecanumDrive, intake, shooter, spindexer, turret, limelight);

        // Configure all button bindings
        configureButtonBindings();
    }

    /**
     * Configure all button bindings for TeleOp controls.
     * Uses FTCLib's GamepadButton and Trigger for command-based control.
     */
    protected void configureButtonBindings() {
        // Default command for driving (field-relative mecanum with slow mode on left trigger)
        mecanumDrive.setDefaultCommand(
            new RunCommand(() -> {
                double ly = driverGamepad.getLeftY();
                double lx = driverGamepad.getLeftX();
                double rx = driverGamepad.getRightX();
                double speed = (gamepad1.left_trigger > DriveConstants.TRIGGER_THRESHOLD)
                    ? DriveConstants.SLOW_MODE_FACTOR : 1.0;
                mecanumDrive.drive(ly, lx, rx, speed, cachedHeadingRad);
            }, mecanumDrive)
        );

        // PS button: Reset IMU yaw
        new Trigger(() -> gamepad1.ps)
            .whenActive(new InstantCommand(mecanumDrive::resetYaw));

        // Right trigger: Intake deploy + start + auto-intake distribution
        new Trigger(() -> gamepad1.right_trigger > DriveConstants.TRIGGER_THRESHOLD)
            .whenActive(() -> {
                intake.startIntaking();
                intake.deploy();
                // Auto-intake distribution on a 300ms interval
                spindexer.autoIntake();

            })
            .whenInactive(() -> {
                intake.stopIntaking();
                intake.retract();
            });

        // Right bumper: Intake reverse + deploy
        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(() -> {
                intake.reverseIntaking();
                intake.deploy();
            })
            .whenReleased(() -> {
                intake.stopIntaking();
                intake.retract();
            });

        // A button: Linkage up (pressed) / linkage down (released) - manual single shot
        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
            .whenPressed(new InstantCommand(spindexer::linkageUp))
            .whenReleased(new InstantCommand(spindexer::linkageDown));

        // Y button: Shoot all 3 balls from current position
        new GamepadButton(driverGamepad, GamepadKeys.Button.Y)
            .whenPressed(() -> schedule(ShootingCommands.shootAll(spindexer)));

        // Dpad Left: Rotate spindexer left (touch sensor guarded)
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(new InstantCommand(() -> {
                if (!spindexer.getTouchSensorState()) {
                    spindexer.rotateLeft();
                }
            }));

        // Dpad Right: Rotate spindexer right (touch sensor guarded)
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(new InstantCommand(() -> {
                if (!spindexer.getTouchSensorState()) {
                    spindexer.rotateRight();
                }
            }));

        // Dpad Up: Go to pose 2
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)
            .whenPressed(() -> {
                new InstantCommand(spindexer::setPoseTwo);
                new InstantCommand(turret::center);
            });

        // Left bumper: Sorted shooting
        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(() -> {
                if (spindexer.getCurrentPosition() == EnumConstants.SpindexerPosition.PoseTwo) {
                    schedule(SortedShootCommand.build(spindexer, limelight));
                } else {
                    spindexer.setPoseTwo();
                    // Wait for rotation to settle, then sorted shoot
                    schedule(new WaitCommand(SpindexerConstants.ROTATION_SETTLE_MS)
                        .andThen(SortedShootCommand.build(spindexer, limelight)));
                }
            });
    }

    @Override
    public void run() {
        // 1. Clear bulk cache (must happen before any hardware reads)
        robot.clearBulkCache();

        // 2. Cache IMU heading once per loop (used by drive + limelight)
        cachedHeadingRad = robot.imu.getRobotYawPitchRollAngles().getYaw(
                org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS);

        // 3. Run command scheduler + subsystem periodic() methods
        super.run();

        // 4. Update limelight orientation
        limelight.updateOrientation(Math.toDegrees(cachedHeadingRad));

        // 5. Turret tracking via Limelight Tx
        updateLimelightTracking();

        // 6. Auto-aim shooter from LUT based on Ty
        shooter.prepareForShot(limelight.getTy());

        // 7. Telemetry
        telemetryHelper.update(telemetry, shooter, turret, spindexer, limelight);
    }

    private void updateLimelightTracking() {
        if (limelight.isValid()) {
            turret.trackTarget(
                limelight.getTx(),
                TurretConstants.TELEOP_GAIN,
                TurretConstants.TRACKING_DEADBAND
            );
            wasTrackingValid = true;
        } else {
            if (wasTrackingValid) {
                llResetTimer.reset();
                wasTrackingValid = false;
            }
            // Auto-center turret after timeout with no valid target
            if (llResetTimer.seconds() > TurretConstants.RESET_TIMEOUT_SEC) {
                turret.center();
            }
        }
    }
}
