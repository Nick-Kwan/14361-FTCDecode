package Autos;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Constants.LimelightConstants;
import Constants.ShooterConstants;
import commands.CommandSequenceBuilder;

@Autonomous(name = "Blue Close", group = "Auto")
public class BlueClose extends AutonTemplate {

    // ===== POSES =====
    private final Pose startPose = new Pose(32.5, 135, Math.toRadians(180));
    private final Pose shootPose1 = new Pose(48, 97, Math.toRadians(130));
    private final Pose collectControlPoint1 = new Pose(84, 85.5);
    private final Pose collectApproach1 = new Pose(42, 86.5, Math.toRadians(180));
    private final Pose collectPose1 = new Pose(37, 86.5, Math.toRadians(180));
    private final Pose collectPose2 = new Pose(32, 86.5, Math.toRadians(180));
    private final Pose shootPose2 = new Pose(48, 97, Math.toRadians(155));
    private final Pose collectControlPoint2 = new Pose(61.5, 61.5);
    private final Pose collectPose3 = new Pose(42, 63.5, Math.toRadians(180));
    private final Pose collectPose4 = new Pose(37, 63.5, Math.toRadians(180));
    private final Pose collectPose5 = new Pose(32, 63.5, Math.toRadians(180));
    private final Pose shootPose3 = new Pose(48, 97, Math.toRadians(150));
    private final Pose collectControlPoint3 = new Pose(62, 36);
    private final Pose collectPose6 = new Pose(42, 38, Math.toRadians(180));

    // ===== PATHS =====
    private Path scorePreload;
    private PathChain toCollect1, toCollect2, toCollect3, toScore1;
    private PathChain toCollect4, toCollect5, toCollect6, toScore2;
    private PathChain toCollect7;

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        scorePreload = new Path(new BezierLine(startPose, shootPose1));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading());

        toCollect1 = follower.pathBuilder()
            .addPath(new BezierCurve(shootPose1, collectControlPoint1, collectApproach1))
            .setLinearHeadingInterpolation(shootPose1.getHeading(), collectApproach1.getHeading())
            .setGlobalDeceleration()
            .build();

        toCollect2 = follower.pathBuilder()
            .addPath(new BezierLine(collectApproach1, collectPose1))
            .setLinearHeadingInterpolation(collectApproach1.getHeading(), collectPose1.getHeading())
            .setGlobalDeceleration()
            .build();

        toCollect3 = follower.pathBuilder()
            .addPath(new BezierLine(collectPose1, collectPose2))
            .setLinearHeadingInterpolation(collectPose1.getHeading(), collectPose2.getHeading())
            .setGlobalDeceleration()
            .build();

        toScore1 = follower.pathBuilder()
            .addPath(new BezierLine(collectPose2, shootPose2))
            .setLinearHeadingInterpolation(collectPose2.getHeading(), shootPose2.getHeading())
            .setGlobalDeceleration()
            .build();

        toCollect4 = follower.pathBuilder()
            .addPath(new BezierCurve(shootPose2, collectControlPoint2, collectPose3))
            .setLinearHeadingInterpolation(shootPose2.getHeading(), collectPose3.getHeading())
            .setGlobalDeceleration()
            .build();

        toCollect5 = follower.pathBuilder()
            .addPath(new BezierLine(collectPose3, collectPose4))
            .setConstantHeadingInterpolation(collectPose4.getHeading())
            .setGlobalDeceleration()
            .build();

        toCollect6 = follower.pathBuilder()
            .addPath(new BezierLine(collectPose4, collectPose5))
            .setConstantHeadingInterpolation(collectPose5.getHeading())
            .setGlobalDeceleration()
            .build();

        toScore2 = follower.pathBuilder()
            .addPath(new BezierLine(collectPose5, shootPose3))
            .setLinearHeadingInterpolation(collectPose5.getHeading(), shootPose3.getHeading())
            .setGlobalDeceleration()
            .build();

        toCollect7 = follower.pathBuilder()
            .addPath(new BezierCurve(shootPose3, collectControlPoint3, collectPose6))
            .setLinearHeadingInterpolation(shootPose3.getHeading(), collectPose6.getHeading())
            .setGlobalDeceleration()
            .build();
    }

    @Override
    public void init() {
        super.init();

        // Close autos don't use limelight tracking
        autoTrackingEnabled = false;
        goalPipeline = LimelightConstants.PIPELINE_GOAL_BLUE;

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
            // Score preload
            .setShooterVelocity(ShooterConstants.SHOOTER_AUTO)
            .intakeStart()
            .moveTo(scorePreload)

            // Shoot cycle 1 (3 balls)
            .delay(1.25)
            .shootAll()

            // Collect from row 1
            .intakeStart()
            .moveTo(toCollect1)
            .intakeStop()
            .delay(0.6)
            .intakeStart()
            .delay(0.5)
            .moveTo(toCollect2)
            .intakeStop()
            .setShooterVelocity(ShooterConstants.SHOOTER_SHORT_AUTO)
            .delay(0.6)
            .intakeStart()
            .delay(0.2)
            .moveTo(toCollect3)

            // Return to shoot
            .intakeStop()
            .moveTo(toScore1)

            // Shoot cycle 2 (3 balls)
            .delay(0.5)
            .setShooterVelocity(ShooterConstants.SHOOTER_AUTO)
            .shootAll()

            // Collect from row 2
            .intakeStart()
            .moveTo(toCollect4)
            .intakeStop()
            .delay(0.6)
            .intakeStart()
            .delay(0.5)
            .moveTo(toCollect5)
            .intakeStop()
            .setShooterVelocity(ShooterConstants.SHOOTER_SHORT_AUTO)
            .delay(0.6)
            .intakeStart()
            .delay(0.2)
            .moveTo(toCollect6)

            // Return to shoot
            .intakeStop()
            .moveTo(toScore2)

            // Shoot cycle 3 (3 balls)
            .delay(0.5)
            .setShooterVelocity(ShooterConstants.SHOOTER_AUTO)
            .shootAll()

            // Park at collect 7
            .moveTo(toCollect7)

            .build();
    }
}
