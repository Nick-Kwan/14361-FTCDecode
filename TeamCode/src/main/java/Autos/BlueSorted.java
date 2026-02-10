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
    private final Pose startPose = new Pose(18.5, 114, Math.toRadians(90));
    private final Pose shootPose1 = new Pose(52.5, 90, Math.toRadians(180));
    private final Pose collectControlPoint1 = new Pose(70, 86);
    private final Pose collectPose1 = new Pose(22, 86, Math.toRadians(180));
    private final Pose shootPose2 = new Pose(54.5, 89, Math.toRadians(185));
    private final Pose collectControlPoint2 = new Pose(72, 59);
    private final Pose collectPose2 = new Pose(26, 59, Math.toRadians(185));
    private final Pose shootPose3 = new Pose(56.5, 86.5, Math.toRadians(190));
    private final Pose collectControlPoint3 = new Pose(80, 32);
    private final Pose collectPose3 = new Pose(22, 36, Math.toRadians(190));
    private final Pose shootPose4 = new Pose(56.5, 88, Math.toRadians(190));
    private final Pose parkPose = new Pose(22, 88, Math.toRadians(190));

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
            .addParametricCallback(0.79, () -> spindexer.setPoseTwo())
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
            .addParametricCallback(0.74, () -> spindexer.setPoseTwo())
            .addParametricCallback(0.85, () -> spindexer.setPoseOne())
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

        // Configure auto tracking
        goalPipeline = LimelightConstants.PIPELINE_GOAL_BLUE;
        autoTrackingGain = TurretConstants.AUTO_ALIGN_GAIN;
        autoTurretResetPosition = TurretConstants.TURRET_POSE_AUTO;

        // Initial turret position
        turret.setPosition(TurretConstants.TURRET_BLUE_AUTO_POSE);

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
            // Score preload
            .setHoodAngle(ShooterConstants.HOOD_POSE_AUTO)
            .rotateTo(EnumConstants.SpindexerPosition.PoseTwo)
            .moveTo(scorePreload, 0.67)

            // Sorted shoot cycle 1
            .setTurretPosition(TurretConstants.TURRET_POSE_AUTO)
            .delay(0.25)
            .sortedShoot(EnumConstants.AllianceColor.Blue)
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
            .setTurretPosition(TurretConstants.TURRET_POSE_AUTO)
            .rotateTo(EnumConstants.SpindexerPosition.PoseTwo)
            .delay(0.25)
            .sortedShoot(EnumConstants.AllianceColor.Blue)
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
            .setTurretPosition(TurretConstants.TURRET_POSE_AUTO)
            .intakeStop()
            .delay(0.25)
            .sortedShoot(EnumConstants.AllianceColor.Blue)
            .delay(0.25)
            .rotateTo(EnumConstants.SpindexerPosition.PoseThree)
            .intakeStart()
            .moveTo(toCollect3, 0.5)

            // Return to score 3
            .delay(0.5)
            .addAction(() -> intake.startIntakingMax())
            .moveTo(toScore3, 0.67)

            // Final no-sorting cycle + park
            .rotateTo(EnumConstants.SpindexerPosition.PoseThree)
            .setTurretPosition(TurretConstants.TURRET_POSE_AUTO)
            .delay(0.25)
            .noSortingBlue()
            .delay(0.25)
            .rotateTo(EnumConstants.SpindexerPosition.PoseThree)
            .intakeStop()
            .moveTo(toPark, 0.67)

            .build();
    }
}
