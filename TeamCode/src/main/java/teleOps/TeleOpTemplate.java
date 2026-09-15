package teleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import Constants.DriveConstants;
import Constants.EnumConstants;
import Constants.FieldMap;
import Constants.LimelightConstants;
import Constants.OdometryConstants;
import Constants.ShooterConstants;
import pedroPathing.Constants;
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
    protected Follower follower;
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

        // Snapshot the raw Pinpoint position BEFORE Pedro follower creation.
        // The Pinpoint hardware retains its accumulated position from auton,
        // but Pedro's PinpointLocalizer constructor overwrites it with (0,0,0).
        robot.pinpoint.update();
        Pose2D rawPinpointPose = robot.pinpoint.getPosition();

        // Create Pedro follower (single source of truth for coordinates)
        // NOTE: This resets the pinpoint position internally via setStartPose(default)
        follower = Constants.createFollower(hardwareMap);

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

        // Set starting pose through Pedro follower (auto handoff or alliance default)
        // Priority: 1) static variable from auton stop()
        //           2) raw pinpoint position retained from auton hardware
        //           3) alliance default corner
        Pose startPose;
        if (OdometryConstants.endingAutonPose != null) {
            startPose = OdometryConstants.endingAutonPose;
            OdometryConstants.endingAutonPose = null;
        } else if (Math.abs(rawPinpointPose.getX(DistanceUnit.INCH)) > 1
                || Math.abs(rawPinpointPose.getY(DistanceUnit.INCH)) > 1) {
            // Pinpoint retained a non-origin position from auton
            startPose = new Pose(
                    rawPinpointPose.getX(DistanceUnit.INCH),
                    rawPinpointPose.getY(DistanceUnit.INCH),
                    rawPinpointPose.getHeading(AngleUnit.RADIANS));
        } else {
            startPose = (getAllianceColor() == EnumConstants.AllianceColor.Red)
                    ? OdometryConstants.redStartPoint
                    : OdometryConstants.blueStartPoint;
        }
        follower.setStartingPose(startPose);

        // Init limelight and wire follower for coordinate-consistent relocalization
        limelight.setFollower(follower);
        limelight.start();
        limelight.switchPipeline(getGoalPipeline());

        // Init turret to center
        turret.center();

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

        // Right trigger: Intake deploy + start
        new Trigger(() -> gamepad1.right_trigger > DriveConstants.TRIGGER_THRESHOLD)
            .whenActive(() -> {
                intake.startIntaking();
                intake.deploy();
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

        // Y button: Shoot all 4 slots sequentially
        new GamepadButton(driverGamepad, GamepadKeys.Button.Y)
            .whenPressed(() -> schedule(ShootingCommands.shootAllSlots(spindexer)));

        // Dpad Left: Advance spindexer one slot (index cycle)
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(new InstantCommand(spindexer::advanceSlot));

        // Dpad Right: Advance spindexer one slot (same as left — CR servo is unidirectional)
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(new InstantCommand(spindexer::advanceSlot));

        // Dpad Up: Advance one slot
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)
            .whenPressed(new InstantCommand(spindexer::advanceSlot));

        // Dpad Down: Reset pinpoint position to alliance default
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(new ResetPositionCommand(follower));

        // Left bumper: Shoot all slots (sorted shoot deprecated, falls through to shootAll)
        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(() -> schedule(ShootingCommands.shootAllSlots(spindexer)));

        // B button: Relocalize pinpoint via Limelight MegaTag2
        new GamepadButton(driverGamepad, GamepadKeys.Button.B)
            .whenPressed(() -> schedule(
                new RelocalizePinpointCommand(limelight, turret, getGoalPipeline())));
    }

    @Override
    public void run() {
        // 1. Clear bulk cache (must happen before any hardware reads)
        robot.clearBulkCache();

        // 2. Update Pedro follower (reads pinpoint through Pedro's coordinate system)
        follower.update();

        // 3. Cache pose from follower (single source of truth for coordinates)
        Pose currentPose = follower.getPose();
        robot.cachedPoseX = currentPose.getX();
        robot.cachedPoseY = currentPose.getY();
        robot.cachedHeading = currentPose.getHeading();

        // 4. Run command scheduler + subsystem periodic() methods
        //    turret.periodic() handles odometry-based tracking
        //    shooter.periodic() handles distance-based auto-aim
        super.run();

        // 5. Update limelight orientation with cached heading
        double turretDegreeOffset = (turret.getPosition()-0.5)*141;
        limelight.updateOrientation(Math.toDegrees(robot.cachedHeading) + turretDegreeOffset);

        // 6. Telemetry
        telemetryHelper.update(telemetry, shooter, turret, spindexer, limelight);
        telemetry.addData("Pose", "X:%.1f Y:%.1f H:%.1f",
                robot.cachedPoseX, robot.cachedPoseY,
                Math.toDegrees(robot.cachedHeading));
        telemetry.addData("Distance", "%.1f in", shooter.getLastDistance());
        telemetry.addData("Turret Deg", "%.1f", turret.getCurrentTargetDegrees());
        Pose3D llposeez = limelight.getLatestResult().getBotpose();
        Pose3D llpose = limelight.getLatestResult().getBotpose_MT2();
        telemetry.addData("Limelight Pose MT1", "%.1f, %.1f, %.1f", (llposeez.getPosition().x+(72*(1.0/LimelightConstants.METERS_TO_INCHES))) * LimelightConstants.METERS_TO_INCHES, (llposeez.getPosition().y+(72*(1.0/LimelightConstants.METERS_TO_INCHES))) * LimelightConstants.METERS_TO_INCHES, llposeez.getOrientation().getYaw(AngleUnit.DEGREES) + 180);
        telemetry.addData("Limelight Pose MT2", "%.1f, %.1f, %.1f", (llpose.getPosition().x+(72*(1.0/LimelightConstants.METERS_TO_INCHES))) * LimelightConstants.METERS_TO_INCHES, (llpose.getPosition().y+(72*(1.0/LimelightConstants.METERS_TO_INCHES))) * LimelightConstants.METERS_TO_INCHES, llpose.getOrientation().getYaw(AngleUnit.DEGREES) + 180);
        telemetry.addData("Out of Range", turret.isTargetOutOfRange());
        telemetry.addData("Tx Correction", "%.2f", turret.getLastTxCorrection());
        telemetry.addData("Reloc", limelight.getLastRelocDebug());
    }
}
