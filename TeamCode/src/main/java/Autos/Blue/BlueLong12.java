package Autos.Blue;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Autos.AutonTemplate;
import Constants.EnumConstants;
import Constants.LimelightConstants;
import Constants.ShooterConstants;
import Constants.TurretConstants;
import commands.CommandSequenceBuilder;

@Autonomous(name = "Blue 12 Long", group = "Blue")
public class BlueLong12 extends AutonTemplate {

    // ===== POSES =====
    private final Pose startPose = new Pose(63.5, 8.5, Math.toRadians(90));
    private final Pose collectControlPoint1 = new Pose(63.5, 23);
    private final Pose collectPose1 = new Pose(15, 10, Math.toRadians(180));
    private final Pose shootPose1 = new Pose(51, 16.5, Math.toRadians(150));
//    private final Pose collectControlPoint2 = new Pose(65, 38);
    private final Pose collectPose2 = new Pose(46, 39, Math.toRadians(180));
    private final Pose goCollectPose2 = new Pose(23, 39, Math.toRadians(180));
//    private final Pose shootControlPoint2 = new Pose(56, 38);
    private final Pose shootPose2 = new Pose(50.8, 14, Math.toRadians(150));
    private final Pose collectPose3 = new Pose(46, 64,Math.toRadians(180));
    private final Pose goCollectPose3 = new Pose(23, 64, Math.toRadians(180));
    private final Pose releasePose = new Pose(22, 66, Math.toRadians(180));
//    private final Pose shootControlPoint3 = new Pose(55, 62);
    private final Pose shootPose3 = new Pose(50.8, 14, Math.toRadians(150));
    private final Pose parkPose = new Pose(50.8, 30, Math.toRadians(180));
    private final Pose collectPose4 = new Pose(46, 86,Math.toRadians(180));
    private final Pose goCollectPose4 = new Pose(23, 86, Math.toRadians(180));
    private final Pose shootPose4 = new Pose(57, 107, Math.toRadians(180));

    // ===== PATHS =====
    private Path toCollect1;
    private PathChain toScore1, toCollect2,goCollect2, toScore2;
    private PathChain toCollect3, goCollect3,release, toScore3, toCollect4, goCollect4, toScore4, toPark;

    @Override
    protected void buildPaths() {
        follower.setStartingPose(super.startPose);

        toCollect1 = new Path(new BezierCurve(startPose, collectControlPoint1, collectPose1));
        toCollect1.setLinearHeadingInterpolation(startPose.getHeading(), collectPose1.getHeading());

        toScore1 = follower.pathBuilder()
            .addPath(new BezierLine(collectPose1, shootPose1))
            .setLinearHeadingInterpolation(collectPose1.getHeading(), shootPose1.getHeading())
                .addParametricCallback(0.9, () -> spindexer.setPoseTwo())
            .build();

        toCollect2 = follower.pathBuilder()
            .addPath(new BezierLine(shootPose1, collectPose2))
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .build();

        goCollect2 = follower.pathBuilder()
                .addPath(new BezierLine(collectPose2, goCollectPose2))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.5, () -> spindexer.setPoseThree())
                .addParametricCallback(0.6, () -> spindexer.setPoseOne())
                .build();

        toScore2 = follower.pathBuilder()
            .addPath(new BezierLine(goCollectPose2, shootPose2))
            .setLinearHeadingInterpolation(goCollectPose2.getHeading(),shootPose2.getHeading())
//                .addParametricCallback(0.8, () -> intake.stopIntaking())
                .addParametricCallback(0.9, () -> spindexer.setPoseTwo())
            .build();

        toCollect3 = follower.pathBuilder()
            .addPath(new BezierLine(shootPose2, collectPose3))
            .setLinearHeadingInterpolation(shootPose2.getHeading(),collectPose3.getHeading())
            .build();

        goCollect3 = follower.pathBuilder()
                .addPath(new BezierLine(collectPose3, goCollectPose3))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.5, () -> spindexer.setPoseThree())
                .addParametricCallback(0.6, () -> spindexer.setPoseOne())
                .build();

        release = follower.pathBuilder()
                .addPath(new BezierLine(goCollectPose3,releasePose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        toScore3 = follower.pathBuilder()
            .addPath(new BezierLine(releasePose, shootPose3))
            .setLinearHeadingInterpolation(releasePose.getHeading(),shootPose3.getHeading())
//                .addParametricCallback(0.8, () -> intake.stopIntaking())
                .addParametricCallback(0.9, () -> spindexer.setPoseTwo())
            .build();

        toPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose3,parkPose))
                .setLinearHeadingInterpolation(shootPose3.getHeading(),parkPose.getHeading())
                .build();

//        toCollect4 = follower.pathBuilder()
//            .addPath(new BezierLine(shootPose3, collectPose4))
//            .setLinearHeadingInterpolation(shootPose3.getHeading(),collectPose4.getHeading())
//            .build();
//
//        goCollect4 = follower.pathBuilder()
//                .addPath(new BezierLine(collectPose4, goCollectPose4))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .addParametricCallback(0.5, () -> spindexer.setPoseThree())
//                .addParametricCallback(0.6, () -> spindexer.setPoseOne())
//                .build();

//        toScore4 = follower.pathBuilder()
//            .addPath(new BezierLine(goCollectPose4, shootPose4))
//            .setLinearHeadingInterpolation(goCollectPose4.getHeading(),shootPose4.getHeading())
////                .addParametricCallback(0.8, () -> intake.stopIntaking())
//                .addParametricCallback(0.9, () -> spindexer.setPoseTwo())
//            .build();

    }

    @Override
    public void init() {
        super.startPose = startPose;
        super.init();

        // Configure auto tracking — long auto uses different settings
        autoTrackingEnabled = true;
        goalPipeline = LimelightConstants.PIPELINE_GOAL_BLUE;
        aprilTagPipeline = LimelightConstants.PIPELINE_LONG_APRILTAG;
        autoTrackingGain = 1.0 / 700;
        autoTurretResetPosition = TurretConstants.DEFAULT;

        // Initial positions
        turret.setPosition(TurretConstants.DEFAULT);
        shooter.setHoodAngle(ShooterConstants.HOOD_POSE_LONG);
        spindexer.setPoseTwo();

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
            // Shoot preload
            .delay(1.0)
            .sortedShoot()

            // Collect 1
            .rotateTo(EnumConstants.SpindexerPosition.PoseThree)
            .intakeStart()
            .moveTo(toCollect1, 1.0)
                .delay(0.3)

            // Score 1
            .rotateTo(EnumConstants.SpindexerPosition.PoseOne)
            .moveTo(toScore1,false)

            // Shoot + collect 2
            .sortedShoot()
                .intakeStart()
                .rotateTo(EnumConstants.SpindexerPosition.PoseThree)
            .moveTo(toCollect2,1.0,false)
                .moveTo(goCollect2,0.35)

            // Score 2
            .moveTo(toScore2)

            // Shoot + collect 3
            .sortedShoot()
                .intakeStart()
                .rotateTo(EnumConstants.SpindexerPosition.PoseThree)
            .moveTo(toCollect3,false)
                .moveTo(goCollect3,0.35)
                .moveTo(release,1.0,true)

            // Score 3
            .moveTo(toScore3,false)

            // Shoot + collect 4
            .sortedShoot()
                .moveTo(toPark)

            .build();
    }
}
