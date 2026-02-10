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

@Autonomous(name = "Red Sorted Long", group = "Auto")
public class RedSortedLong extends AutonTemplate {

    // ===== POSES =====
    // Based on Kwan redSortedLong.java (d32cdda) - mirrored for red alliance
    private final Pose startPose = new Pose(80.5, 8.5, Math.toRadians(90));
    private final Pose collectControlOnePose = new Pose(80.5, 25);
    private final Pose collectOnePose = new Pose(128, 11, Math.toRadians(0));
    private final Pose shootOnePose = new Pose(88, 14, Math.toRadians(70));
    private final Pose collectControlTwoPose = new Pose(79, 34);
    private final Pose collectTwoPose = new Pose(125, 34, Math.toRadians(90));
    private final Pose shootControlTwoPose = new Pose(88, 38);
    private final Pose shootTwoPose = new Pose(88, 14, Math.toRadians(70));
    private final Pose collectControlThreePose = new Pose(64, 58);
    private final Pose collectThreePose = new Pose(122, 58, Math.toRadians(90));
    private final Pose shootControlThreePose = new Pose(89, 62);
    private final Pose shootThreePose = new Pose(88, 14, Math.toRadians(70));
    private final Pose collectControlFourPose = new Pose(64, 81);
    private final Pose collectFourPose = new Pose(117, 81, Math.toRadians(90));
    private final Pose shootFourPose = new Pose(94, 81, Math.toRadians(70));
    private final Pose parkPose = new Pose(119, 81, Math.toRadians(0));

    // ===== PATHS =====
    private Path toCollect1;
    private PathChain toScore1, toCollect2, toScore2;
    private PathChain toCollect3, toScore3, toCollect4, toScore4, toPark;

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        toCollect1 = new Path(new BezierCurve(startPose, collectControlOnePose, collectOnePose));
        toCollect1.setLinearHeadingInterpolation(startPose.getHeading(), collectOnePose.getHeading());

        toScore1 = follower.pathBuilder()
            .addPath(new BezierCurve(collectOnePose, shootOnePose))
            .setLinearHeadingInterpolation(collectOnePose.getHeading(), shootOnePose.getHeading())
            .setGlobalDeceleration()
            .build();

        toCollect2 = follower.pathBuilder()
            .addPath(new BezierCurve(shootOnePose, collectControlTwoPose, collectTwoPose))
            .addParametricCallback(0.2, () -> spindexer.setPoseThree())
            .addParametricCallback(0.79, () -> spindexer.setPoseTwo())
            .setTangentHeadingInterpolation()
            .setGlobalDeceleration()
            .build();

        toScore2 = follower.pathBuilder()
            .addPath(new BezierCurve(collectTwoPose, shootControlTwoPose, shootTwoPose))
            .setTangentHeadingInterpolation()
            .setReversed()
            .setGlobalDeceleration()
            .build();

        toCollect3 = follower.pathBuilder()
            .addPath(new BezierCurve(shootTwoPose, collectControlThreePose, collectThreePose))
            .addParametricCallback(0.2, () -> spindexer.setPoseThree())
            .addParametricCallback(0.74, () -> spindexer.setPoseTwo())
            .addParametricCallback(0.85, () -> spindexer.setPoseOne())
            .setTangentHeadingInterpolation()
            .setGlobalDeceleration()
            .build();

        toScore3 = follower.pathBuilder()
            .addPath(new BezierCurve(collectThreePose, shootControlThreePose, shootThreePose))
            .setTangentHeadingInterpolation()
            .setReversed()
            .setGlobalDeceleration()
            .build();

        toCollect4 = follower.pathBuilder()
            .addPath(new BezierCurve(shootThreePose, collectControlFourPose, collectFourPose))
            .addParametricCallback(0.2, () -> spindexer.setPoseThree())
            .addParametricCallback(0.79, () -> spindexer.setPoseTwo())
            .addParametricCallback(0.89, () -> spindexer.setPoseOne())
            .setTangentHeadingInterpolation()
            .setGlobalDeceleration()
            .build();

        toScore4 = follower.pathBuilder()
            .addPath(new BezierCurve(collectFourPose, shootFourPose))
            .setTangentHeadingInterpolation()
            .setReversed()
            .setGlobalDeceleration()
            .build();

        toPark = follower.pathBuilder()
            .addPath(new BezierLine(shootFourPose, parkPose))
            .setTangentHeadingInterpolation()
            .setGlobalDeceleration()
            .build();
    }

    @Override
    public void init() {
        super.init();

        // Configure auto tracking — long auto uses different settings
        goalPipeline = LimelightConstants.PIPELINE_GOAL_RED;
        autoTrackingGain = 1.0 / 700;
        autoTurretResetPosition = TurretConstants.DEFAULT;
        aprilTagPipeline = 1;

        // Initial positions
        turret.setPosition(TurretConstants.DEFAULT);
        shooter.setHoodAngle(ShooterConstants.HOOD_POSE_LONG);
        spindexer.setPoseTwo();

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
            // Shoot preload
            .setShooterVelocity(ShooterConstants.SHOOTER_AUTO)
            .delay(1.0)
            .sortedShoot()

            // Collect 1
            .rotateTo(EnumConstants.SpindexerPosition.PoseOne)
            .delay(0.45)
            .intakeStart()
            .moveTo(toCollect1, 0.8)

            // Score 1
            .delay(0.3)
            .rotateTo(EnumConstants.SpindexerPosition.PoseTwo)
            .delay(1.0)
            .moveTo(toScore1)

            // Shoot + collect 2
            .delay(0.5)
            .sortedShoot()
            .delay(0.4)
            .moveTo(toCollect2)

            // Score 2
            .delay(1.0)
            .rotateTo(EnumConstants.SpindexerPosition.PoseTwo)
            .moveTo(toScore2)

            // Shoot + collect 3
            .sortedShoot()
            .delay(0.2)
            .moveTo(toCollect3)

            // Score 3
            .delay(1.0)
            .rotateTo(EnumConstants.SpindexerPosition.PoseTwo)
            .moveTo(toScore3)

            // Shoot + collect 4
            .sortedShoot()
            .delay(0.2)
            .moveTo(toCollect4)

            // Score 4 (different hood + max power)
            .delay(1.0)
            .setHoodAngle(0.4)
            .rotateTo(EnumConstants.SpindexerPosition.PoseTwo)
            .moveTo(toScore4, 1.0)

            // Final shoot + park
            .setTurretPosition(0.2)
            .delay(0.2)
            .sortedShoot()
            .delay(0.2)
            .moveTo(toPark)

            .build();
    }
}
