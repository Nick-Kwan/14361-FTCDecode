package Autos;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Constants.EnumConstants;
import Constants.LimelightConstants;
import Constants.TurretConstants;
import commands.CommandSequenceBuilder;

@Autonomous(name = "Blue 15 Release", group = "Auto")
public class BlueRelease15 extends AutonTemplate {

    // ===== POSES =====
    // Updated from Kwan hkhk commit (d32cdda) for improved path following
    private final Pose startPose = new Pose(18.5, 114, Math.toRadians(90));
    private final Pose shootOnePose = new Pose(53.5, 90, Math.toRadians(180));
    private final Pose collectOnePose = new Pose(46, 62, Math.toRadians(180));
    private final Pose goCollectOnePose = new Pose(25, 62,Math.toRadians(180));
    private final Pose shootControlTwoPose = new Pose(46.8, 64.8);
    private final Pose shootTwoPose = new Pose(54.7, 77.9, Math.toRadians(180));
    private final Pose releasePose = new Pose(13, 59, Math.toRadians(150));
    private final Pose releaseControlPose = new Pose(40, 61.3, Math.toRadians(180));
    private final Pose shootThreePose = new Pose(54.7, 77.9, Math.toRadians(180));
    private final Pose shootControlThreePose = new Pose(40, 61.3, Math.toRadians(180));
    private final Pose collectControlTwoPose = new Pose(54, 89);
    private final Pose collectTwoPose = new Pose(24, 89, Math.toRadians(180));
    private final Pose shootFourPose = new Pose(57, 89, Math.toRadians(180));
    private final Pose collectThreePose = new Pose(46, 37, Math.toRadians(180));
    private final Pose goCollectThreePose = new Pose(23, 37, Math.toRadians(180));
    private final Pose shootControlFourPose = new Pose(39, 37);
    private final Pose shootControl2FourPose = new Pose(34, 88);
    private final Pose shootFivePose = new Pose(61.7, 105.3, Math.toRadians(180));
    private final Pose parkPose = new Pose(24, 88, Math.toRadians(180));

    // ===== PATHS =====
    private Path scorePreload;
    private PathChain collectOne, release, scoreOne, collectTwo, goCollectOne, scoreTwo;
    private PathChain collectThree, goCollectThree, scoreThree, scoreFour, park;

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        scorePreload = new Path(new BezierLine(startPose, shootOnePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), shootOnePose.getHeading());

        collectOne = follower.pathBuilder()
                .addPath(new BezierLine(shootOnePose, collectOnePose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        goCollectOne = follower.pathBuilder()
                .addPath(new BezierLine(collectOnePose, goCollectOnePose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.35, () -> spindexer.setPoseThree())
                .addParametricCallback(0.45, () -> spindexer.setPoseOne())
                .build();

        scoreOne = follower.pathBuilder()
                .addPath(new BezierCurve(goCollectOnePose, shootControlTwoPose, shootTwoPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.8, () -> intake.stopIntaking())
                .addParametricCallback(0.9, () -> spindexer.setPoseTwo())
                .build();

        release = follower.pathBuilder()
                .addPath(new BezierCurve(shootTwoPose,releaseControlPose,releasePose))
                .setLinearHeadingInterpolation(shootTwoPose.getHeading(),releasePose.getHeading())
//                .addParametricCallback(0.97, () -> spindexer.setPoseOne())
                .build();

        scoreTwo = follower.pathBuilder()
                .addPath(new BezierCurve(releasePose,shootControlThreePose,shootThreePose))
                .setLinearHeadingInterpolation(releasePose.getHeading(), shootThreePose.getHeading())
                .addParametricCallback(0.05, () -> intake.reverseIntaking())
                .addParametricCallback(0.35, () -> intake.startIntaking())
                .addParametricCallback(0.9, () -> spindexer.setPoseTwo())
                .build();


        collectTwo = follower.pathBuilder()
                .addPath(new BezierCurve(shootThreePose, collectControlTwoPose, collectTwoPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.5, () -> spindexer.setPoseThree())
                .addParametricCallback(0.6, () -> spindexer.setPoseOne())
                .build();


        scoreThree = follower.pathBuilder()
                .addPath(new BezierLine(collectTwoPose, shootFourPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.6, () -> spindexer.setPoseTwo())
                .addParametricCallback(0.9, () -> intake.stopIntaking())
                .build();

        collectThree = follower.pathBuilder()
                .addPath(new BezierLine(shootFourPose, collectThreePose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        goCollectThree = follower.pathBuilder()
                .addPath(new BezierLine(collectThreePose, goCollectThreePose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.33, () -> spindexer.setPoseThree())
                .addParametricCallback(0.43, () -> spindexer.setPoseOne())
                .build();

        scoreFour = follower.pathBuilder()
                .addPath(new BezierCurve(goCollectThreePose, shootControlFourPose, shootControl2FourPose, shootFivePose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.6, () -> spindexer.setPoseTwo())
                .addParametricCallback(0.9, () -> intake.stopIntaking())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(shootFivePose, parkPose))
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
                .setShooterVelocity(0.37)
                .rotateTo(EnumConstants.SpindexerPosition.PoseTwo)
                .moveTo(scorePreload, 1.0,false)

                // Sorted shoot cycle 1
//                .delay(0.5)
                .sortedShoot()
                .rotateTo(EnumConstants.SpindexerPosition.PoseThree)
                .intakeStart()

                // Collect 1
                .moveTo(collectOne, 1.0,false)
                .moveTo(goCollectOne, 0.35,false)

                .moveTo(scoreOne, 1.0,false)
//                .intakeStop()
                .delay(0.1)
                .sortedShoot()
                .rotateTo(EnumConstants.SpindexerPosition.PoseThree)
                .intakeStart()

                // Release the gate
                .moveTo(release,1.0)
                .delay(0.1)
                .rotateTo(EnumConstants.SpindexerPosition.PoseOne)
                .delay(0.2)
//                .delay(0.3)

                // Sorted shoot cycle 2
                .moveTo(scoreTwo, 1.0,false)
//                .intakeStop()
                .delay(0.1)
                .sortedShoot()
                .rotateTo(EnumConstants.SpindexerPosition.PoseThree)
                .intakeStart()

                // Collect 2
                .moveTo(collectTwo, 0.35,false)

                // Sorted shoot cycle 3
//                .rotateTo(EnumConstants.SpindexerPosition.PoseTwo)
                .moveTo(scoreThree, 1.0,false)
//                .intakeStop()
                .delay(0.1)
                .sortedShoot()
                .rotateTo(EnumConstants.SpindexerPosition.PoseThree)
                .intakeStart()

                // Collect 3
                .moveTo(collectThree, 1.0,false)
                .moveTo(goCollectThree, 0.35,false)

                // Sorted shoot cycle 4
                .moveTo(scoreFour, 1.0,false)
//                .intakeStop()
//                .rotateTo(EnumConstants.SpindexerPosition.PoseTwo)
                .delay(0.1)
                .sortedShoot()

                // Park
                .rotateTo(EnumConstants.SpindexerPosition.PoseOne)
                .intakeStop()
//                .moveTo(park, 1.0)

                .build();
    }
}
