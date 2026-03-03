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

@Autonomous(name = "Blue Joint Long", group = "Blue")
public class BlueLongJoint extends AutonTemplate {

    // ===== POSES =====
    private final Pose startPose = new Pose(63.5, 8.5, Math.toRadians(90));
    private final Pose collectControlPoint1 = new Pose(34.5, 20);
    private final Pose collectPose1 = new Pose(13, 10, Math.toRadians(195));
    private final Pose shootPose1 = new Pose(51, 16.5, Math.toRadians(150));
    private final Pose collectPose2 = new Pose(46, 39,Math.toRadians(180));
    private final Pose goCollectPose2 = new Pose(23, 39, Math.toRadians(180));
    //    private final Pose shootControlPoint2 = new Pose(56, 38);
    private final Pose shootPose2 = new Pose(52, 12, Math.toRadians(150));
    private final Pose collectPose3 = new Pose(16.5, 10,Math.toRadians(195));
    private final Pose shootPose3 = new Pose(52, 10, Math.toRadians(150));
    private final Pose collectPose4 = new Pose(16.5, 10,Math.toRadians(195));
    private final Pose shootPose4 = new Pose(52, 10, Math.toRadians(150));
    private final Pose collectControlPose5 = new Pose(5, 10,Math.toRadians(195));
    private final Pose collectControl2Pose5 = new Pose(5, 10,Math.toRadians(195));
    private final Pose collectPose5 = new Pose(17.5, 10,Math.toRadians(195));
    private final Pose shootPose5 = new Pose(52, 10, Math.toRadians(150));

    private final Pose parkPose = new Pose(50, 29, Math.toRadians(90));

    // ===== PATHS =====
    private Path toCollect1;
    private PathChain toScore1, toCollect2, goCollect2, toScore2;
    private PathChain toCollect3, goCollect3, toScore3, toCollect4, goCollect4, toScore4, toCollect5, toScore5, toPark;

    @Override
    protected void buildPaths() {
        follower.setStartingPose(super.startPose);

        toCollect1 = new Path(new BezierCurve(startPose, collectControlPoint1, collectPose1));
        toCollect1.setLinearHeadingInterpolation(startPose.getHeading(), collectPose1.getHeading());

        toScore1 = follower.pathBuilder()
                .addPath(new BezierLine(collectPose1, shootPose1))
                .setLinearHeadingInterpolation(collectPose1.getHeading(), shootPose1.getHeading())
//                .addParametricCallback(0.2, () -> intake.stopIntaking())
//                .addParametricCallback(0.5, () -> spindexer.setPoseTwo())
                .build();

        toCollect2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose1, collectPose2))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        goCollect2 = follower.pathBuilder()
                .addPath(new BezierLine(collectPose2, goCollectPose2))
                .setLinearHeadingInterpolation(collectPose2.getHeading(),goCollectPose2.getHeading())
                .addParametricCallback(0.35, () -> spindexer.setPoseThree())
                .addParametricCallback(0.45, () -> spindexer.setPoseOne())
                .build();

        toScore2 = follower.pathBuilder()
                .addPath(new BezierLine(goCollectPose2, shootPose2))
                .setLinearHeadingInterpolation(collectPose2.getHeading(),shootPose2.getHeading())
//                .addParametricCallback(0.2, () -> intake.stopIntaking())
                .addParametricCallback(0.5, () -> spindexer.setPoseTwo())
                .build();


        toCollect3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, collectPose3))
                .setLinearHeadingInterpolation(shootPose2.getHeading(),collectPose3.getHeading())
                .addParametricCallback(0.75, () -> spindexer.setPoseThree())
                .addParametricCallback(0.95, () -> spindexer.setPoseTwo())
                .build();


        toScore3 = follower.pathBuilder()
                .addPath(new BezierLine(collectPose3, shootPose3))
                .setLinearHeadingInterpolation(collectPose3.getHeading(),shootPose3.getHeading())
                .addParametricCallback(0.5, () -> intake.reverseIntaking())
                .addParametricCallback(0.9, () -> intake.startIntaking())
                .build();

        toCollect4 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose3, collectPose4))
                .setLinearHeadingInterpolation(shootPose3.getHeading(),collectPose4.getHeading())
                .addParametricCallback(0.75, () -> spindexer.setPoseThree())
                .addParametricCallback(0.95, () -> spindexer.setPoseTwo())
                .build();


        toScore4 = follower.pathBuilder()
                .addPath(new BezierLine(collectPose4, shootPose4))
                .setLinearHeadingInterpolation(collectPose4.getHeading(),shootPose4.getHeading())
                .addParametricCallback(0.5, () -> intake.reverseIntaking())
                .addParametricCallback(0.9, () -> intake.startIntaking())
                .build();

        toCollect5 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose4, collectPose5))
                .setLinearHeadingInterpolation(shootPose4.getHeading(),collectPose5.getHeading())
                .addParametricCallback(0.75, () -> spindexer.setPoseThree())
                .addParametricCallback(0.95, () -> spindexer.setPoseTwo())
                .build();

        toScore5 = follower.pathBuilder()
                .addPath(new BezierLine(collectPose5, shootPose5))
                .setLinearHeadingInterpolation(collectPose5.getHeading(),shootPose5.getHeading())
                .addParametricCallback(0.5, () -> intake.reverseIntaking())
                .addParametricCallback(0.9, () -> intake.startIntaking())
                .build();

        toPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose5, parkPose))
                .setLinearHeadingInterpolation(shootPose5.getHeading(),parkPose.getHeading())
                .build();

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
                .intakeStart()
                .rotateTo(EnumConstants.SpindexerPosition.PoseOne)
                .moveTo(toCollect1,false)
                .delay(0.3)
                .rotateTo(EnumConstants.SpindexerPosition.PoseTwo)

                // Score 1
                .moveTo(toScore1,false)

                // Shoot + collect 2
                .sortedShoot()
                .rotateTo(EnumConstants.SpindexerPosition.PoseThree)
                .moveTo(toCollect2,false)
                .moveTo(goCollect2,0.35,false)

                // Score 2
                .moveTo(toScore2,1.0)

                // Shoot + collect 3
                .sortedShoot()
                .rotateTo(EnumConstants.SpindexerPosition.PoseThree)
                .moveTo(toCollect3,1.0,false)

                // Score 3
                .moveTo(toScore3,1.0,false)

                // Shoot + collect 4
                .sortedShoot()
                .rotateTo(EnumConstants.SpindexerPosition.PoseThree)
                .moveTo(toCollect4,1.0,false)

                // Score 4

                .moveTo(toScore4, 1.0,false)
                .sortedShoot()
                .rotateTo(EnumConstants.SpindexerPosition.PoseThree)
                .moveTo(toCollect5,1.0,false)

                .moveTo(toScore5, 1.0,false)
                .sortedShoot()

                .moveTo(toPark,1.0)

                .build();
    }
}

