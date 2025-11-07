package org.firstinspires.ftc.teamcode.pedroPathing.autoPaths;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

import org.firstinspires.ftc.teamcode.States.SecondShooterStates;
import org.firstinspires.ftc.teamcode.States.ShooterStates;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

@Autonomous (name = "NextFTC")
public class nextFTC extends PedroOpMode {
    public nextFTC() {
        super(Spindexer.INSTANCE, );
    }
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final RobotHardware robot = RobotHardware.getInstance();
    private static double initX = RobotConstants.Auto.initX;
    private static double initY = RobotConstants.Auto.initY;
    private boolean pathTemp = false;
    private boolean actionTempOne, actionTempTwo,actionTempThree,actionTempFour,actionTempFive, actionTempSix = false;
    public double getInitX(){
        return initX;
    }
    public double getInitY(){
        return initY;
    }



    // Setting up the poses for the paths
    private final Pose startPose = new Pose(initX, initY, Math.toRadians(180));

    private final Pose shootOnePose = new Pose(48, 95.5, Math.toRadians(140));

    private final Pose collectOnePose = new Pose(12.5, 85.5, Math.toRadians(180));
    private final Pose collectControlOnePose = new Pose(78, 83.5, Math.toRadians(180));


    // Creating the paths from the poses
    private Path scorePreload;
    private PathChain pickupOne, scoreOne;
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, shootOnePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), shootOnePose.getHeading());

        pickupOne = follower.pathBuilder()
                .addPath(new BezierCurve(shootOnePose,collectControlOnePose,collectOnePose))
                .setLinearHeadingInterpolation(shootOnePose.getHeading(),collectOnePose.getHeading())
                .build();
    }

    // Running the paths after creating them
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.isShooting = true;
                robot.shooter.setPower(RobotConstants.Drivetrain.shooterShortOn);
                //robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
                follower.followPath(scorePreload);
                setPathState(1);

                break;
            case 1:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    if (pathTemp) {
                        pathTimer.resetTimer();
                        pathTemp = false;
                    }
                    while (robot.isShooting){
                        robot.spindexer.shootingThree();
                    }
                    pathTemp = true;
                    if (!robot.isShooting){
                        follower.followPath(pickupOne,true);
                    }
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }



    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        follower.update();
        autonomousPathUpdate();

    }

    @Override
    public void init() {
        CommandScheduler.getInstance().run();
        robot.init(hardwareMap);

        robot.spindexer.shootingState = ShooterStates.One;
        robot.spindexer.secondShootingState = SecondShooterStates.One;
        robot.isShooting = true;

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        robot.spindexer.shooterTimer = new ElapsedTime();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("shooter state" , robot.spindexer.shootingState);
        telemetry.addData("second shooter state" , robot.spindexer.secondShootingState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Touch Sensor : ", !robot.spindexer.getTouchSensorState());
        telemetry.update();
    }

    @Override
    public void init_loop() {}


    @Override
    public void start() {
        opmodeTimer.resetTimer();
        actionTimer.resetTimer();
        robot.spindexer.shooterTimer.reset();
        setPathState(0);
    }
}
