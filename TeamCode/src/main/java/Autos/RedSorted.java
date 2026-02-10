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

@Autonomous(name = "Red Sorted", group = "Auto")
public class RedSorted extends AutonTemplate {

    // ===== POSES =====
    // Updated from Kwan hkhk commit (d32cdda) - mirrored for red alliance
    private final Pose startPose = new Pose(125.5, 114, Math.toRadians(90));
    private final Pose shootOnePose = new Pose(90.5, 90, Math.toRadians(0));
    private final Pose collectControlOnePose = new Pose(88, 85);
    private final Pose collectOnePose = new Pose(117, 85, Math.toRadians(0));
    private final Pose shootControlTwoPose = new Pose(95, 89);
    private final Pose shootTwoPose = new Pose(90, 89, Math.toRadians(0));
    private final Pose collectControlTwoPose = new Pose(72, 56.5);
    private final Pose collectTwoPose = new Pose(118.5, 56.5, Math.toRadians(0));
    private final Pose shootControlThreePose = new Pose(95, 56.5);
    private final Pose shootControl2ThreePose = new Pose(103.5, 86.5);
    private final Pose shootThreePose = new Pose(87, 86.5, Math.toRadians(0));
    private final Pose collectControlThreePose = new Pose(60, 34);
    private final Pose collectThreePose = new Pose(122, 34, Math.toRadians(0));
    private final Pose shootControlFourPose = new Pose(105, 34);
    private final Pose shootControl2FourPose = new Pose(110, 88);
    private final Pose shootFourPose = new Pose(90, 88, Math.toRadians(0));
    private final Pose parkPose = new Pose(120, 88, Math.toRadians(10));

    // ===== PATHS =====
    private Path scorePreload;
    private PathChain collectOne, scoreOne, collectTwo, scoreTwo;
    private PathChain collectThree, scoreThree, park;

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        scorePreload = new Path(new BezierLine(startPose, shootOnePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), shootOnePose.getHeading());

        collectOne = follower.pathBuilder()
            .addPath(new BezierCurve(shootOnePose, collectControlOnePose, collectOnePose))
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .addParametricCallback(0.2, () -> spindexer.setPoseThree())
            .addParametricCallback(0.74, () -> spindexer.setPoseTwo())
            .addParametricCallback(0.89, () -> spindexer.setPoseOne())
            .build();

        scoreOne = follower.pathBuilder()
            .addPath(new BezierCurve(collectOnePose, shootControlTwoPose, shootTwoPose))
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

        collectTwo = follower.pathBuilder()
            .addPath(new BezierCurve(shootTwoPose, collectControlTwoPose, collectTwoPose))
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .addParametricCallback(0.2, () -> spindexer.setPoseThree())
            .addParametricCallback(0.78, () -> spindexer.setPoseTwo())
            .addParametricCallback(0.93, () -> spindexer.setPoseOne())
            .build();

        scoreTwo = follower.pathBuilder()
            .addPath(new BezierCurve(collectTwoPose, shootControlThreePose, shootControl2ThreePose, shootThreePose))
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

        collectThree = follower.pathBuilder()
            .addPath(new BezierCurve(shootThreePose, collectControlThreePose, collectThreePose))
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .addParametricCallback(0.2, () -> spindexer.setPoseThree())
            .addParametricCallback(0.83, () -> spindexer.setPoseTwo())
            .addParametricCallback(0.92, () -> spindexer.setPoseOne())
            .build();

        scoreThree = follower.pathBuilder()
            .addPath(new BezierCurve(collectThreePose, shootControlFourPose, shootControl2FourPose, shootFourPose))
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

        park = follower.pathBuilder()
            .addPath(new BezierLine(shootFourPose, parkPose))
            .setLinearHeadingInterpolation(shootFourPose.getHeading(), parkPose.getHeading())
            .build();
    }

    @Override
    public void init() {
        super.init();

        // Configure auto tracking for red alliance
        goalPipeline = LimelightConstants.PIPELINE_GOAL_RED;
        autoTrackingGain = TurretConstants.AUTO_ALIGN_GAIN;
        autoTurretResetPosition = TurretConstants.TURRET_POSE_AUTO;

        // Initial turret position
        turret.setPosition(TurretConstants.TURRET_RED_AUTO_POSE);

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
            // Score preload
            .setShooterVelocity(0.46)
            .rotateTo(EnumConstants.SpindexerPosition.PoseTwo)
            .moveTo(scorePreload, 1.0)

            // Sorted shoot cycle 1
            .delay(1.0)
            .sortedShoot()
            .delay(2.5)
            .rotateTo(EnumConstants.SpindexerPosition.PoseThree)
            .intakeStart()
            .moveTo(collectOne, 1.0)

            // Return to score 1
            .delay(0.5)
            .addAction(() -> intake.startIntakingMax())
            .moveTo(scoreOne, 1.0)

            // Sorted shoot cycle 2
            .rotateTo(EnumConstants.SpindexerPosition.PoseTwo)
            .intakeStop()
            .delay(1.0)
            .sortedShoot()
            .delay(2.5)
            .rotateTo(EnumConstants.SpindexerPosition.PoseThree)
            .intakeStart()
            .moveTo(collectTwo, 1.0)

            // Return to score 2
            .delay(0.5)
            .addAction(() -> intake.startIntakingMax())
            .moveTo(scoreTwo, 1.0)

            // Sorted shoot cycle 3
            .rotateTo(EnumConstants.SpindexerPosition.PoseTwo)
            .intakeStop()
            .delay(1.0)
            .sortedShoot()
            .delay(2.5)
            .rotateTo(EnumConstants.SpindexerPosition.PoseThree)
            .intakeStart()
            .moveTo(collectThree, 1.0)

            // Return to score 3
            .delay(0.5)
            .addAction(() -> intake.startIntakingMax())
            .moveTo(scoreThree, 1.0)

            // Final shoot cycle + park
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
