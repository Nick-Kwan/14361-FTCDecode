package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import subsystems.Limelight;
import subsystems.Turret;

/**
 * Command that relocalizes the pinpoint odometry using Limelight MegaTag2.
 *
 * Flow:
 * 1. Center turret to 0.5 so Limelight has a clear view
 * 2. Switch to localization pipeline
 * 3. Wait for pipeline to settle (150ms)
 * 4. Poll Limelight for botpose
 * 5. Apply pose to pinpoint
 * 6. Switch back to goal pipeline
 */
public class RelocalizePinpointCommand extends CommandBase {
    private final Limelight limelight;
    private final Turret turret;
    private final int goalPipeline;

    private final ElapsedTime timer = new ElapsedTime();
    private boolean success = false;

    private static final double SETTLE_SECONDS = 0.15;
    private static final double TIMEOUT_SECONDS = 0.5;

    public RelocalizePinpointCommand(Limelight limelight, Turret turret, int goalPipeline) {
        this.limelight = limelight;
        this.turret = turret;
        this.goalPipeline = goalPipeline;
    }

    @Override
    public void initialize() {
        // Center turret so Limelight has a clear view for AprilTag scan
        turret.setPosition(0.5);
        limelight.switchToLocalizationPipeline();
        timer.reset();
        success = false;
    }

    @Override
    public void execute() {
        if (timer.seconds() < SETTLE_SECONDS) {
            return; // Wait for pipeline to stabilize
        }
        limelight.updateLimelightPose();
        success = limelight.relocalizePinpoint();
    }

    @Override
    public boolean isFinished() {
        return success || timer.seconds() >= TIMEOUT_SECONDS;
    }

    @Override
    public void end(boolean interrupted) {
        // Switch back to goal tracking pipeline
        limelight.switchToGoalPipeline(goalPipeline);
    }
}
