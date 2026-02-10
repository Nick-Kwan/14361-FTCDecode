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

@Autonomous(name = "Blue Sorted", group = "Auto")
public class BlueSorted extends AutonTemplate {

    // ===== POSES =====
    // Updated from Kwan hkhk commit (d32cdda) for improved path following
    private final Pose startPose = new Pose(18.5, 114, Math.toRadians(90));
    private final Pose shootOnePose = new Pose(53.5, 90, Math.toRadians(180));
    private final Pose collectControlOnePose = new Pose(54, 84);
    private final Pose collectOnePose = new Pose(24, 84, Math.toRadians(180));
    private final Pose shootControlTwoPose = new Pose(49, 89);
    private final Pose shootTwoPose = new Pose(54, 89, Math.toRadians(180));
    private final Pose collectTwoPose = new Pose(44, 61, Math.toRadians(180));
    private final Pose goCollectTwoPose = new Pose(25, 61, Math.toRadians(180));
    private final Pose collectControlTwoPose = new Pose(72, 61);
    private final Pose shootControlThreePose = new Pose(49, 61);
    private final Pose shootControl2ThreePose = new Pose(40.5, 86.5);
    private final Pose shootThreePose = new Pose(57, 86.5, Math.toRadians(180));
    private final Pose collectThreePose = new Pose(46, 37, Math.toRadians(180));
    private final Pose goCollectThreePose = new Pose(23, 37, Math.toRadians(180));
    private final Pose collectControlThreePose = new Pose(84, 33.5);
    private final Pose shootControlFourPose = new Pose(39, 37);
    private final Pose shootControl2FourPose = new Pose(34, 88);
    private final Pose shootFourPose = new Pose(54, 88, Math.toRadians(180));
    private final Pose parkPose = new Pose(24, 88, Math.toRadians(180));

    // ===== PATHS =====
    private Path scorePreload;
    private PathChain collectOne, scoreOne, collectTwo, goCollectTwo, scoreTwo;
    private PathChain collectThree, goCollectThree, scoreThree, park;

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        scorePreload = new Path(new BezierLine(startPose, shootOnePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), shootOnePose.getHeading());

        collectOne = follower.pathBuilder()
            .addPath(new BezierCurve(shootOnePose, collectControlOnePose, collectOnePose))
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .addParametricCallback(0.2, () -> spindexer.setPoseThree())
            .addParametricCallback(0.74, () -> spindexer.setPoseTwo())
            .addParametricCallback(0.89, () -> spindexer.setPoseOne())
            .build();

        scoreOne = follower.pathBuilder()
            .addPath(new BezierCurve(collectOnePose, shootControlTwoPose, shootTwoPose))
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .build();

        collectTwo = follower.pathBuilder()
            .addPath(new BezierCurve(shootTwoPose, collectControlTwoPose, collectTwoPose))
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .addParametricCallback(0.2, () -> spindexer.setPoseThree())
            .addParametricCallback(0.78, () -> spindexer.setPoseTwo())
            .addParametricCallback(0.93, () -> spindexer.setPoseOne())
            .build();

        goCollectTwo = follower.pathBuilder()
            .addPath(new BezierLine(collectTwoPose, goCollectTwoPose))
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .addParametricCallback(0.2, () -> spindexer.setPoseThree())
            .addParametricCallback(0.3, () -> spindexer.setPoseOne())
            .build();

        scoreTwo = follower.pathBuilder()
            .addPath(new BezierCurve(goCollectTwoPose, shootControlThreePose, shootControl2ThreePose, shootThreePose))
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .build();

        collectThree = follower.pathBuilder()
            .addPath(new BezierLine(shootThreePose, collectThreePose))
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .build();

        goCollectThree = follower.pathBuilder()
            .addPath(new BezierLine(collectThreePose, goCollectThreePose))
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .addParametricCallback(0.2, () -> spindexer.setPoseThree())
            .addParametricCallback(0.3, () -> spindexer.setPoseOne())
            .build();

        scoreThree = follower.pathBuilder()
            .addPath(new BezierCurve(goCollectThreePose, shootControlFourPose, shootControl2FourPose, shootFourPose))
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .build();

        park = follower.pathBuilder()
            .addPath(new BezierLine(shootFourPose, parkPose))
            .setConstantHeadingInterpolation(Math.toRadians(180))
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
            .sortedShoot(EnumConstants.AllianceColor.Blue)
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
            .sortedShoot(EnumConstants.AllianceColor.Blue)
            .delay(2.5)
            .rotateTo(EnumConstants.SpindexerPosition.PoseThree)
            .intakeStart()
            .moveTo(collectTwo, 1.0)

            // Extended collection 2
            .delay(0.5)
            .moveTo(goCollectTwo, 1.0)
            .delay(0.5)
            .addAction(() -> intake.startIntakingMax())
            .moveTo(scoreTwo, 1.0)

            // Sorted shoot cycle 3
            .rotateTo(EnumConstants.SpindexerPosition.PoseTwo)
            .intakeStop()
            .delay(1.0)
            .sortedShoot(EnumConstants.AllianceColor.Blue)
            .delay(2.5)
            .rotateTo(EnumConstants.SpindexerPosition.PoseThree)
            .intakeStart()
            .moveTo(collectThree, 1.0)

            // Extended collection 3
            .delay(0.5)
            .moveTo(goCollectThree, 1.0)
            .delay(0.5)
            .addAction(() -> intake.startIntakingMax())
            .moveTo(scoreThree, 1.0)

            // Final shoot cycle + park
            .rotateTo(EnumConstants.SpindexerPosition.PoseTwo)
            .intakeStop()
            .delay(1.0)
            .sortedShoot(EnumConstants.AllianceColor.Blue)
            .delay(1.9)
            .rotateTo(EnumConstants.SpindexerPosition.PoseOne)
            .intakeStop()
            .moveTo(park, 1.0)

            .build();
    }
}
