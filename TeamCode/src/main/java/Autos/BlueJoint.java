package Autos;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Constants.EnumConstants;
import Constants.LimelightConstants;
import Constants.ShooterConstants;
import Constants.TurretConstants;
import commands.CommandSequenceBuilder;

@Autonomous(name = "Blue Joint", group = "Auto")
public class BlueJoint extends AutonTemplate {

    // ===== POSES =====
    // Based on Kwan blueJoint.java (d32cdda)
    private final Pose startPose = new Pose(18.5, 114, Math.toRadians(90));
    private final Pose shootOnePose = new Pose(53.5, 90, Math.toRadians(180));
    private final Pose collectControlOnePose = new Pose(72, 62);
    private final Pose collectOnePose = new Pose(25.5, 62, Math.toRadians(180));
    private final Pose releaseControlPose = new Pose(28, 65);
    private final Pose releasePose = new Pose(19, 65, Math.toRadians(270));
    private final Pose shootControlTwoPose = new Pose(49.5, 65);
    private final Pose shootTwoPose = new Pose(56.5, 86.5, Math.toRadians(180));
    private final Pose collectControlTwoPose = new Pose(56, 86);
    private final Pose collectTwoPose = new Pose(27, 86, Math.toRadians(180));
    private final Pose shootControlThreePose = new Pose(49, 89);
    private final Pose shootThreePose = new Pose(54, 89, Math.toRadians(180));
    private final Pose parkPose = new Pose(24, 89, Math.toRadians(180));

    // ===== PATHS =====
    private Path scorePreload;
    private PathChain collectOne, release, scoreOne, collectTwo, scoreTwo, park;

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        scorePreload = new Path(new BezierLine(startPose, shootOnePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), shootOnePose.getHeading());

        collectOne = follower.pathBuilder()
            .addPath(new BezierCurve(shootOnePose, collectControlOnePose, collectOnePose))
            .setLinearHeadingInterpolation(shootOnePose.getHeading(), collectOnePose.getHeading())
            .addParametricCallback(0.2, () -> spindexer.setPoseOne())
            .addParametricCallback(0.84, () -> spindexer.setPoseTwo())
            .addParametricCallback(0.93, () -> spindexer.setPoseThree())
            .build();

        release = follower.pathBuilder()
            .addPath(new BezierCurve(collectOnePose, releaseControlPose, releasePose))
            .setLinearHeadingInterpolation(collectOnePose.getHeading(), releasePose.getHeading())
            .build();

        scoreOne = follower.pathBuilder()
            .addPath(new BezierCurve(releasePose, shootControlTwoPose, shootTwoPose))
            .setLinearHeadingInterpolation(releasePose.getHeading(), shootTwoPose.getHeading())
            .build();

        collectTwo = follower.pathBuilder()
            .addPath(new BezierCurve(shootTwoPose, collectControlTwoPose, collectTwoPose))
            .setLinearHeadingInterpolation(shootTwoPose.getHeading(), collectTwoPose.getHeading())
            .addParametricCallback(0.2, () -> spindexer.setPoseThree())
            .addParametricCallback(0.83, () -> spindexer.setPoseTwo())
            .addParametricCallback(0.92, () -> spindexer.setPoseOne())
            .build();

        scoreTwo = follower.pathBuilder()
            .addPath(new BezierCurve(collectTwoPose, shootControlThreePose, shootThreePose))
            .setLinearHeadingInterpolation(collectTwoPose.getHeading(), shootThreePose.getHeading())
            .build();

        park = follower.pathBuilder()
            .addPath(new BezierCurve(shootThreePose, parkPose))
            .setConstantHeadingInterpolation(parkPose.getHeading())
            .build();
    }

    @Override
    public void init() {
        super.init();

        // Configure auto tracking
        goalPipeline = LimelightConstants.PIPELINE_GOAL_BLUE;
        autoTrackingGain = TurretConstants.AUTO_ALIGN_GAIN;
        autoTurretResetPosition = TurretConstants.TURRET_POSE_AUTO;

        // Initial turret position
        turret.setPosition(TurretConstants.TURRET_BLUE_AUTO_POSE);

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
            // Score preload
            .setShooterVelocity(0.46)
            .rotateTo(EnumConstants.SpindexerPosition.PoseTwo)
            .moveTo(scorePreload, 1.0)

            // Sorted shoot cycle 1
            .delay(1.0)
            .sortedShoot()
            .delay(1.9)
            .rotateTo(EnumConstants.SpindexerPosition.PoseOne)
            .intakeStart()
            .moveTo(collectOne, 0.8)

            // Release to human player zone
            .delay(0.5)
            .intakeStop()
            .moveTo(release)

            // Wait for human player load (7.5s in original)
            .delay(7.5)
            .moveTo(scoreOne, 1.0)

            // Shoot cycle 2
            .delay(1.0)
            .rotateTo(EnumConstants.SpindexerPosition.PoseTwo)
            .delay(0.25)
            .sortedShoot()
            .delay(2.2)
            .rotateTo(EnumConstants.SpindexerPosition.PoseThree)
            .intakeStart()
            .moveTo(collectTwo, 0.7)

            // Return to score 2
            .delay(0.5)
            .addAction(() -> intake.startIntakingMax())
            .moveTo(scoreTwo, 1.0)

            // Final shoot cycle
            .rotateTo(EnumConstants.SpindexerPosition.PoseTwo)
            .intakeStop()
            .delay(1.0)
            .sortedShoot()
            .delay(1.9)
            .rotateTo(EnumConstants.SpindexerPosition.PoseOne)
            .intakeStop()
            .moveTo(park, 1.0)

            .build();
    }
}
