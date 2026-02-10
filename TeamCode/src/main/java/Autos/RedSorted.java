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
    private final Pose startPose = new Pose(125.5, 114, Math.toRadians(90));
    private final Pose shootPose1 = new Pose(90.5, 90, Math.toRadians(0));
    private final Pose collectControlPoint1 = new Pose(75, 76);
    private final Pose collectPose1 = new Pose(117, 81, Math.toRadians(0));
    private final Pose shootPose2 = new Pose(90, 89, Math.toRadians(5));
    private final Pose collectControlPoint2 = new Pose(72, 51);
    private final Pose collectPose2 = new Pose(118.5, 56, Math.toRadians(5));
    private final Pose shootPose3 = new Pose(87, 86.5, Math.toRadians(10));
    private final Pose collectControlPoint3 = new Pose(80, 24);
    private final Pose collectPose3 = new Pose(122, 30, Math.toRadians(10));
    private final Pose shootPose4 = new Pose(90, 88, Math.toRadians(10));
    private final Pose parkPose = new Pose(120, 88, Math.toRadians(10));

    // ===== PATHS =====
    private Path scorePreload;
    private PathChain toCollect1, toScore1, toCollect2, toScore2, toCollect3, toScore3, toPark;

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        scorePreload = new Path(new BezierLine(startPose, shootPose1));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading());

        toCollect1 = follower.pathBuilder()
            .addPath(new BezierCurve(shootPose1, collectControlPoint1, collectPose1))
            .setLinearHeadingInterpolation(shootPose1.getHeading(), collectPose1.getHeading())
            .addParametricCallback(0.2, () -> spindexer.setPoseThree())
            .addParametricCallback(0.84, () -> spindexer.setPoseTwo())
            .addParametricCallback(0.93, () -> spindexer.setPoseOne())
            .setGlobalDeceleration()
            .build();

        toScore1 = follower.pathBuilder()
            .addPath(new BezierCurve(collectPose1, shootPose2))
            .setLinearHeadingInterpolation(collectPose1.getHeading(), shootPose2.getHeading())
            .setGlobalDeceleration()
            .build();

        toCollect2 = follower.pathBuilder()
            .addPath(new BezierCurve(shootPose2, collectControlPoint2, collectPose2))
            .setLinearHeadingInterpolation(shootPose2.getHeading(), collectPose2.getHeading())
            .addParametricCallback(0.2, () -> spindexer.setPoseThree())
            .addParametricCallback(0.83, () -> spindexer.setPoseTwo())
            .addParametricCallback(0.92, () -> spindexer.setPoseOne())
            .setGlobalDeceleration()
            .build();

        toScore2 = follower.pathBuilder()
            .addPath(new BezierCurve(collectPose2, shootPose3))
            .setLinearHeadingInterpolation(collectPose2.getHeading(), shootPose3.getHeading())
            .setGlobalDeceleration()
            .build();

        toCollect3 = follower.pathBuilder()
            .addPath(new BezierCurve(shootPose3, collectControlPoint3, collectPose3))
            .setLinearHeadingInterpolation(shootPose3.getHeading(), collectPose3.getHeading())
            .addParametricCallback(0.2, () -> spindexer.setPoseThree())
            .addParametricCallback(0.79, () -> spindexer.setPoseTwo())
            .addParametricCallback(0.89, () -> spindexer.setPoseOne())
            .setGlobalDeceleration()
            .build();

        toScore3 = follower.pathBuilder()
            .addPath(new BezierCurve(collectPose3, shootPose4))
            .setLinearHeadingInterpolation(collectPose3.getHeading(), shootPose4.getHeading())
            .setGlobalDeceleration()
            .build();

        toPark = follower.pathBuilder()
            .addPath(new BezierCurve(shootPose3, parkPose))
            .setLinearHeadingInterpolation(shootPose3.getHeading(), parkPose.getHeading())
            .setGlobalDeceleration()
            .build();
    }

    @Override
    public void init() {
        super.init();

        // Configure auto tracking — red uses different gain
        goalPipeline = LimelightConstants.PIPELINE_GOAL_RED;
        autoTrackingGain = TurretConstants.AUTO_LOOP_GAIN_RED;
        autoTurretResetPosition = TurretConstants.TURRET_RED_AUTO_POSE;

        // Initial positions
        turret.setPosition(0);
        spindexer.setPoseThree();

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
            // Score preload
            .setHoodAngle(ShooterConstants.HOOD_POSE_AUTO)
            .rotateTo(EnumConstants.SpindexerPosition.PoseTwo)
            .moveTo(scorePreload, 0.67)

            // Sorted shoot cycle 1
            .setTurretPosition(TurretConstants.TURRET_RED_AUTO_POSE)
            .delay(0.1)
            .sortedShoot(EnumConstants.AllianceColor.Red)
            .delay(0.25)
            .rotateTo(EnumConstants.SpindexerPosition.PoseThree)
            .intakeStart()
            .moveTo(toCollect1, 0.6)

            // Return to score 1
            .addAction(() -> intake.startIntakingMax())
            .delay(0.5)
            .moveTo(toScore1, 0.67)

            // Sorted shoot cycle 2
            .intakeStop()
            .setTurretPosition(TurretConstants.TURRET_RED_AUTO_POSE)
            .delay(0.25)
            .rotateTo(EnumConstants.SpindexerPosition.PoseTwo)
            .delay(0.25)
            .sortedShoot(EnumConstants.AllianceColor.Red)
            .delay(0.25)
            .intakeStart()
            .rotateTo(EnumConstants.SpindexerPosition.PoseThree)
            .moveTo(toCollect2, 0.6)

            // Return to score 2
            .delay(0.5)
            .addAction(() -> intake.startIntakingMax())
            .moveTo(toScore2, 0.67)

            // Sorted shoot cycle 3
            .rotateTo(EnumConstants.SpindexerPosition.PoseTwo)
            .setTurretPosition(TurretConstants.TURRET_RED_AUTO_POSE)
            .intakeStop()
            .delay(0.25)
            .sortedShoot(EnumConstants.AllianceColor.Red)
            .delay(0.25)
            .rotateTo(EnumConstants.SpindexerPosition.PoseThree)
            .intakeStart()
            .moveTo(toCollect3, 0.5)

            // Return to score 3
            .delay(0.5)
            .addAction(() -> intake.startIntakingMax())
            .moveTo(toScore3, 0.67)

            // Final no-sorting cycle + park
            .rotateTo(EnumConstants.SpindexerPosition.PoseOne)
            .setTurretPosition(TurretConstants.TURRET_RED_AUTO_POSE)
            .delay(0.25)
            .noSortingRed()
            .delay(0.25)
            .rotateTo(EnumConstants.SpindexerPosition.PoseOne)
            .intakeStop()
            .moveTo(toPark, 0.67)

            .build();
    }
}
