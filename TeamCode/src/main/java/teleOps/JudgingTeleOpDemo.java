package teleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import Constants.DriveConstants;
import Constants.LimelightConstants;
import Constants.ShooterConstants;
import Constants.TurretConstants;
import commands.ShootingCommands;
import subsystems.Intake;
import subsystems.Limelight;
import subsystems.Shooter;
import subsystems.Spindexer;
import subsystems.Turret;
import utility.RobotHardware;
import utility.TelemetryHelper;

/**
 * Simplified TeleOp for competition judging demonstrations.
 *
 * Key differences from regular TeleOp:
 * - No drive controls (robot stays stationary for safety)
 * - Hood-only LUT adjustments (no flywheel velocity changes)
 * - Simplified control scheme for judges to operate
 *
 * Controls:
 * - Right trigger: Deploy intake and start intaking
 * - Right bumper: Reverse intake
 * - A button: Manual single-shot (linkage up/down)
 * - Y button: Shoot all 4 slots
 * - Left bumper: Shoot all 4 slots
 * - DpadLeft/Right/Up: Advance spindexer one slot
 * - PS button: Reset IMU yaw
 */
@TeleOp(name = "Judging Demo", group = "Tuning")
public class JudgingTeleOpDemo extends CommandOpMode {
    protected RobotHardware robot;
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

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        driverGamepad = new GamepadEx(gamepad1);
        robot = RobotHardware.getInstance();
        robot.init(hardwareMap, driverGamepad);

        // Create subsystems (no MecanumDrive for judging demo)
        intake = new Intake();
        shooter = new Shooter();
        spindexer = new Spindexer();
        turret = new Turret();
        limelight = new Limelight();
        telemetryHelper = new TelemetryHelper();

        // Configure shooter PID for teleop
        shooter.configureForTeleOp();

        // Init limelight (blue pipeline by default)
        limelight.start();
        limelight.switchPipeline(LimelightConstants.PIPELINE_GOAL_BLUE);

        // Init turret to center
        turret.center();

        // Init hood to mid
        shooter.setHoodAngle(ShooterConstants.HOOD_POSE_MID);

        // Timers
        llResetTimer = new ElapsedTime();

        // Register subsystems with the command scheduler
        register(intake, shooter, spindexer, turret, limelight);

        // Configure all button bindings
        configureButtonBindings();

        telemetry.addLine("=== JUDGING DEMO MODE ===");
        telemetry.addLine("Drive controls DISABLED for safety");
        telemetry.addLine("Hood adjusts automatically via Limelight");
        telemetry.update();
    }

    /**
     * Configure simplified button bindings for judging demo.
     */
    protected void configureButtonBindings() {
        // PS button: Reset IMU yaw (not really used without drive, but kept for consistency)
        new Trigger(() -> gamepad1.ps)
            .whenActive(new InstantCommand(() -> robot.imu.resetYaw()));

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

        // Dpad Left: Advance spindexer one slot
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(new InstantCommand(spindexer::advanceSlot));

        // Dpad Right: Advance spindexer one slot (CR servo is unidirectional)
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(new InstantCommand(spindexer::advanceSlot));

        // Dpad Up: Advance spindexer one slot
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)
            .whenPressed(new InstantCommand(spindexer::advanceSlot));

        // Left bumper: Shoot all 4 slots
        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(() -> schedule(ShootingCommands.shootAllSlots(spindexer)));
    }

    @Override
    public void run() {
        // 1. Clear bulk cache (must happen before any hardware reads)
        robot.clearBulkCache();

        // 2. Run command scheduler + subsystem periodic() methods
        super.run();

        // 3. Update limelight orientation
        double yaw = robot.imu.getRobotYawPitchRollAngles().getYaw();
        limelight.updateOrientation(yaw);

        // 4. Turret tracking via Limelight Tx
        updateLimelightTracking();

        // 5. Demo mode - fixed hood position (no velocity changes for safety)
        shooter.setHoodAngle(ShooterConstants.HOOD_POSE_MID);

        // 6. Telemetry
        telemetry.addLine("=== JUDGING DEMO MODE ===");
        telemetryHelper.update(telemetry, shooter, turret, spindexer, limelight);
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("  RT: Intake | RB: Reverse");
        telemetry.addLine("  A: Fire (hold) | Y: Shoot All");
        telemetry.addLine("  LB: Shoot All");
        telemetry.addLine("  D-Pad: Advance Spindexer Slot");
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
            if (llResetTimer.seconds() > TurretConstants.RESET_TIMEOUT_SEC
                    && limelight.getTx() == 0) {
                turret.center();
                wasTrackingValid = true;
            }
        }
    }
}
