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

@Autonomous(name = "Blue Sorted Long", group = "Auto")
public class BlueSortedLong extends AutonTemplate {

    // ===== POSES =====
    private final Pose startPose = new Pose(63.5, 8.5, Math.toRadians(90));
    private final Pose collectControlPoint1 = new Pose(34.5, 20);
    private final Pose collectPose1 = new Pose(16, 10, Math.toRadians(180));
    private final Pose shootPose1 = new Pose(56, 14, Math.toRadians(110));
    private final Pose collectControlPoint2 = new Pose(65, 38);
    private final Pose collectPose2 = new Pose(19, 38, Math.toRadians(90));
    private final Pose shootControlPoint2 = new Pose(56, 38);
    private final Pose shootPose2 = new Pose(56, 14, Math.toRadians(110));
    private final Pose collectControlPoint3 = new Pose(65, 62);
    private final Pose collectPose3 = new Pose(25, 62, Math.toRadians(90));
    private final Pose shootControlPoint3 = new Pose(55, 62);
    private final Pose shootPose3 = new Pose(56, 14, Math.toRadians(110));
    private final Pose collectControlPoint4 = new Pose(65, 86);
    private final Pose collectPose4 = new Pose(25, 86, Math.toRadians(90));
    private final Pose shootPose4 = new Pose(50, 86, Math.toRadians(110));
    private final Pose parkPose = new Pose(25, 86, Math.toRadians(180));

    // ===== PATHS =====
    private Path toCollect1;
    private PathChain toScore1, toCollect2, toScore2;
    private PathChain toCollect3, toScore3, toCollect4, toScore4, toPark;

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        toCollect1 = new Path(new BezierCurve(startPose, collectControlPoint1, collectPose1));
        toCollect1.setLinearHeadingInterpolation(startPose.getHeading(), collectPose1.getHeading());

        toScore1 = follower.pathBuilder()
            .addPath(new BezierCurve(collectPose1, shootPose1))
            .setLinearHeadingInterpolation(collectPose1.getHeading(), shootPose1.getHeading())
            .setGlobalDeceleration()
            .build();

        toCollect2 = follower.pathBuilder()
            .addPath(new BezierCurve(shootPose1, collectControlPoint2, collectPose2))
            .addParametricCallback(0.2, () -> spindexer.setPoseThree())
            .addParametricCallback(0.79, () -> spindexer.setPoseTwo())
            .setTangentHeadingInterpolation()
            .setGlobalDeceleration()
            .build();

        toScore2 = follower.pathBuilder()
            .addPath(new BezierCurve(collectPose2, shootControlPoint2, shootPose2))
            .setTangentHeadingInterpolation()
            .setReversed()
            .setGlobalDeceleration()
            .build();

        toCollect3 = follower.pathBuilder()
            .addPath(new BezierCurve(shootPose2, collectControlPoint3, collectPose3))
            .addParametricCallback(0.2, () -> spindexer.setPoseThree())
            .addParametricCallback(0.74, () -> spindexer.setPoseTwo())
            .addParametricCallback(0.85, () -> spindexer.setPoseOne())
            .setTangentHeadingInterpolation()
            .setGlobalDeceleration()
            .build();

        toScore3 = follower.pathBuilder()
            .addPath(new BezierCurve(collectPose3, shootControlPoint3, shootPose3))
            .setTangentHeadingInterpolation()
            .setReversed()
            .setGlobalDeceleration()
            .build();

        toCollect4 = follower.pathBuilder()
            .addPath(new BezierCurve(shootPose3, collectControlPoint4, collectPose4))
            .addParametricCallback(0.2, () -> spindexer.setPoseThree())
            .addParametricCallback(0.79, () -> spindexer.setPoseTwo())
            .addParametricCallback(0.89, () -> spindexer.setPoseOne())
            .setTangentHeadingInterpolation()
            .setGlobalDeceleration()
            .build();

        toScore4 = follower.pathBuilder()
            .addPath(new BezierCurve(collectPose4, shootPose4))
            .setTangentHeadingInterpolation()
            .setReversed()
            .setGlobalDeceleration()
            .build();

        toPark = follower.pathBuilder()
            .addPath(new BezierLine(shootPose4, parkPose))
            .setTangentHeadingInterpolation()
            .setGlobalDeceleration()
            .build();
    }

    @Override
    public void init() {
        super.init();

        // Configure auto tracking — long auto uses different settings
        goalPipeline = LimelightConstants.PIPELINE_GOAL_BLUE;
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
            .setTurretPosition(0.8)
            .delay(0.2)
            .sortedShoot()
            .delay(0.2)
            .moveTo(toPark)

            .build();
    }
}
