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
import com.pedropathing.geometry.Pose;

import Constants.DriveConstants;
import Constants.EnumConstants;
import Constants.FieldMap;
import Constants.LimelightConstants;
import Constants.OdometryConstants;
import Constants.ShooterConstants;
import Constants.SpindexerConstants;
import Constants.TurretConstants;
import commands.RelocalizePinpointCommand;
import commands.ResetPositionCommand;
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
 * Uses odometry-based turret tracking and distance-based shooter LUT.
 *
 * Subclasses set alliance color and limelight pipeline.
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

    /** Subclass returns the goal-tracking pipeline (3=Blue, 2=Red) */
    protected abstract int getGoalPipeline();

    /** Subclass returns the alliance color */
    protected abstract EnumConstants.AllianceColor getAllianceColor();

    /** Drive input multiplier: 1.0 for Red (default), -1.0 for Blue (driver faces opposite) */
    protected double getDriveDirectionMultiplier() { return 1.0; }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        // Set alliance color globally
        FieldMap.allianceColor = getAllianceColor();

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

        // Link turret <-> shooter references
        shooter.setTurret(turret);

        // Configure shooter PID for teleop
        shooter.configureForTeleOp();

        // Set pinpoint starting position from auto handoff or alliance default
        Pose startPose;
        if (OdometryConstants.endingAutonPose != null) {
            startPose = OdometryConstants.endingAutonPose;
        } else {
            startPose = (getAllianceColor() == EnumConstants.AllianceColor.Red)
                    ? OdometryConstants.redStartPoint
                    : OdometryConstants.blueStartPoint;
        }
        robot.pinpoint.setPosition(OdometryConstants.toPose2D(startPose));
        robot.pinpoint.update();

        // Init limelight
        limelight.start();
        limelight.switchPipeline(getGoalPipeline());

        // Init turret to center
        turret.center();

        // Init spindexer to pose 1
        spindexer.setPoseOne();

        // Init hood to mid
        shooter.setHoodAngle(ShooterConstants.HOOD_POSE_MID);

        // Enable odometry-based turret tracking and distance-based auto-aim
        turret.setLimelight(limelight);
        turret.setTrackingEnabled(true);
        turret.setTxCorrectionEnabled(true);
        shooter.setAutoAimEnabled(true);

        // Register subsystems with the command scheduler
        register(mecanumDrive, intake, shooter, spindexer, turret, limelight);

        // Configure all button bindings
        configureButtonBindings();
    }

    /**
     * Configure all button bindings for TeleOp controls.
     */
    protected void configureButtonBindings() {
        // Default command for driving (field-relative mecanum with slow mode on left trigger)
        mecanumDrive.setDefaultCommand(
            new RunCommand(() -> {
                double dir = getDriveDirectionMultiplier();
                double ly = driverGamepad.getLeftY() * dir;
                double lx = driverGamepad.getLeftX() * dir;
                double rx = driverGamepad.getRightX();
                double speed = (gamepad1.left_trigger > DriveConstants.TRIGGER_THRESHOLD)
                    ? DriveConstants.SLOW_MODE_FACTOR : 1.0;
                mecanumDrive.drive(ly, lx, rx, speed, robot.cachedHeading);
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

        // Dpad Down: Reset pinpoint position to alliance default
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(() -> schedule(new ResetPositionCommand()));

        // Left bumper: Sorted shooting
        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(() -> {
                if (spindexer.getCurrentPosition() == EnumConstants.SpindexerPosition.PoseTwo) {
                    schedule(SortedShootCommand.build(spindexer, limelight));
                } else {
                    spindexer.setPoseTwo();
                    schedule(new WaitCommand(SpindexerConstants.ROTATION_SETTLE_MS)
                        .andThen(SortedShootCommand.build(spindexer, limelight)));
                }
            });

        // B button: Relocalize pinpoint via Limelight MegaTag2
        new GamepadButton(driverGamepad, GamepadKeys.Button.B)
            .whenPressed(() -> schedule(
                new RelocalizePinpointCommand(limelight, turret, getGoalPipeline())));
    }

    @Override
    public void run() {
        // 1. Clear bulk cache (must happen before any hardware reads)
        robot.clearBulkCache();

        // 2. Update pinpoint odometry
        robot.pinpoint.update();

        // 3. Cache pose fields for this loop iteration
        robot.updateCachedPose();

        // 4. Run command scheduler + subsystem periodic() methods
        //    turret.periodic() handles odometry-based tracking
        //    shooter.periodic() handles distance-based auto-aim
        super.run();

        // 5. Update limelight orientation with cached heading
        limelight.updateOrientation(Math.toDegrees(robot.cachedHeading));

        // 6. Telemetry
        telemetryHelper.update(telemetry, shooter, turret, spindexer, limelight);
        telemetry.addData("Pose", "X:%.1f Y:%.1f H:%.1f",
                robot.cachedPoseX, robot.cachedPoseY,
                Math.toDegrees(robot.cachedHeading));
        telemetry.addData("Distance", "%.1f in", shooter.getLastDistance());
        telemetry.addData("Turret Deg", "%.1f", turret.getCurrentTargetDegrees());
        telemetry.addData("Out of Range", turret.isTargetOutOfRange());
        telemetry.addData("Tx Correction", "%.2f", turret.getLastTxCorrection());
        telemetry.addData("Reloc", limelight.getLastRelocDebug());
    }
}
