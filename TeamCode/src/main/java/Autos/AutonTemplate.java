package Autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import Constants.LimelightConstants;
import Constants.ShooterConstants;
import Constants.TurretConstants;
import pedroPathing.Constants;
import subsystems.Intake;
import subsystems.Limelight;
import subsystems.Shooter;
import subsystems.Spindexer;
import subsystems.Turret;
import utility.RobotHardware;

/**
 * Base template for all autonomous OpModes.
 * Subclasses implement buildPaths() to create paths and set autonomousCommand
 * via the CommandSequenceBuilder.
 */
public abstract class AutonTemplate extends OpMode {
    protected Follower follower;
    protected Timer pathTimer, opmodeTimer;

    protected RobotHardware robotHardware;
    protected Shooter shooter;
    protected Turret turret;
    protected Intake intake;
    protected Spindexer spindexer;
    protected Limelight limelight;

    private final ElapsedTime loopTimer = new ElapsedTime();

    /** The autonomous command sequence built by subclasses */
    protected Command autonomousCommand;

    // ==================== Auto Tracking Configuration ====================
    // Subclasses set these in init() to configure loop-level tracking

    /** Enable continuous limelight tracking + pipeline switching in loop() */
    protected boolean autoTrackingEnabled = true;

    /** Turret proportional gain for auto tracking */
    protected double autoTrackingGain = TurretConstants.AUTO_ALIGN_GAIN;

    /** Turret position to reset to when no valid target */
    protected double autoTurretResetPosition = TurretConstants.TURRET_POSE_AUTO;

    /** AprilTag pipeline number */
    protected int aprilTagPipeline = LimelightConstants.PIPELINE_APRILTAG;

    /** Goal-tracking pipeline (2=Red, 3=Blue) — subclass sets in init() */
    protected int goalPipeline = LimelightConstants.PIPELINE_GOAL_BLUE;

    // Tracking state (managed by updateAutoTracking)
    private boolean limelightOnGoalPipeline = false;
    private boolean wasTrackingValid = false;
    private Timer llResetTimer;

    /**
     * Subclasses must implement this to build paths.
     * Should also call follower.setStartingPose().
     * Do NOT build the autonomousCommand here — build it in init() after super.init().
     */
    protected abstract void buildPaths();

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        llResetTimer = new Timer();
        opmodeTimer.resetTimer();

        // Create Pedro follower
        follower = Constants.createFollower(hardwareMap);

        // Init hardware
        robotHardware = RobotHardware.getInstance();
        robotHardware.init(hardwareMap);

        // Create subsystems
        intake = new Intake();
        shooter = new Shooter();
        turret = new Turret();
        spindexer = new Spindexer();
        limelight = new Limelight();

        // Configure shooter PID for auto
        shooter.configureForAuto();

        // Init hood for auto
        shooter.setHoodAngle(ShooterConstants.HOOD_POSE_AUTO);

        // Init limelight
        limelight.start();
        limelight.switchPipeline(LimelightConstants.PIPELINE_APRILTAG);
        limelight.resetAprilID();

        // Register subsystems with the command scheduler
        CommandScheduler.getInstance().registerSubsystem(intake, shooter, turret, spindexer, limelight);

        // Subclass builds paths (and sets starting pose)
        buildPaths();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        llResetTimer.resetTimer();

        // Schedule the autonomous command
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }

        loopTimer.reset();
    }

    @Override
    public void loop() {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        // Update follower FIRST (before commands run)
        follower.update();

        // Clear bulk cache
        robotHardware.clearBulkCache();

        // Update limelight orientation BEFORE scheduler runs periodic()
        double yaw = robotHardware.imu.getRobotYawPitchRollAngles().getYaw();
        limelight.updateOrientation(yaw);

        // Run the command scheduler (runs subsystem periodic + scheduled commands)
        CommandScheduler.getInstance().run();

        // Auto tracking (pipeline switching + turret tracking)
        if (autoTrackingEnabled) {
            updateAutoTracking();
        }

        // Telemetry
        if (autonomousCommand != null) {
            telemetry.addData("Auto Status", autonomousCommand.isFinished() ? "Finished" : "Running");
        }
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Position", "X:%.1f Y:%.1f H:%.1f",
            follower.getPose().getX(), follower.getPose().getY(),
            Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Spindexer", spindexer.getCurrentPosition());
        telemetry.addData("April ID", limelight.getAprilID());
        telemetry.addData("Flick State", spindexer.getFlickState());
        telemetry.addData("LL Valid", limelight.isValid());
        telemetry.addData("Tx", "%.2f", limelight.getTx());
        telemetry.addData("Loop", "%.1f ms", loopMs);
        telemetry.update();
    }

    /**
     * Continuous limelight tracking: pipeline switching + turret alignment.
     */
    private void updateAutoTracking() {
        int aprilID = limelight.getAprilID();

        // Pipeline switching based on detected AprilTag
        if (aprilID < 20) {
            limelight.switchPipeline(aprilTagPipeline);
            limelightOnGoalPipeline = false;
        } else if (aprilID > 20) {
            limelight.switchPipeline(goalPipeline);
            limelightOnGoalPipeline = true;
        }

        // Turret tracking when on goal pipeline
        if (limelightOnGoalPipeline && limelight.isValid()) {
            turret.trackTarget(limelight.getTx(), autoTrackingGain, TurretConstants.AUTO_ALIGN_DEADBAND);
            wasTrackingValid = true;
        } else if (!limelight.isValid()) {
            if (wasTrackingValid) {
                llResetTimer.resetTimer();
                wasTrackingValid = false;
            }
            if (llResetTimer.getElapsedTimeSeconds() > TurretConstants.RESET_TIMEOUT_SEC) {
                turret.setPosition(autoTurretResetPosition);
            }
        }
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
    }
}