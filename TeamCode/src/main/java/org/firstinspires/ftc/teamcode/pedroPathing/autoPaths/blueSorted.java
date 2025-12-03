package org.firstinspires.ftc.teamcode.pedroPathing.autoPaths;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.States.ActionStates;
import org.firstinspires.ftc.teamcode.States.SecondShooterStates;
import org.firstinspires.ftc.teamcode.States.ShooterStates;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

import java.util.List;

@Autonomous (name = "Blue Sorted", group = "Auto")
public class blueSorted extends OpMode{

    private Follower follower;
    private Timer  actionTimer, opmodeTimer;
    private ElapsedTime pathTimer;
    private int pathState;

    public ActionStates actionState = ActionStates.shootOne;
    public void setActionState(ActionStates state) {
        actionState = state;
    }

    private final RobotHardware robot = RobotHardware.getInstance();

    private static double initX = 18.5;
    private static double initY = 114;
    private boolean pathTemp = false;
    private boolean actionTempOne, actionTempTwo,actionTempThree,actionTempFour,actionTempFive, actionTempSix = false;
    public double getInitX(){
        return initX;
    }
    public double getInitY(){
        return initY;
    }



    // Setting up the poses for the paths
    private final Pose startPose = new Pose(initX, initY, Math.toRadians(90));

    private final Pose shootOnePose = new Pose(43.5, 99, Math.toRadians(130));
    private final Pose collectOnePose = new Pose(25,84,Math.toRadians(180));
    private final Pose collectControlOnePose = new Pose(76,83,Math.toRadians(180));
    private final Pose releasePose = new Pose(20,69.5,Math.toRadians(90));
    private final Pose shootTwoPose = new Pose(43.5, 99, Math.toRadians(130));
    private final Pose shootTwoControlPose = new Pose(44.5, 71.5, Math.toRadians(130));
    private final Pose collectTwoPose = new Pose(25,60,Math.toRadians(180));
    private final Pose collectControlTwoPose = new Pose(76,59,Math.toRadians(180));
    private final Pose shootThreePose = new Pose(43.5, 99, Math.toRadians(130));
    private final Pose collectThreePose = new Pose(25,34,Math.toRadians(180));
    private final Pose collectControlThreePose = new Pose(76,33,Math.toRadians(180));
    private final Pose shootFourPose = new Pose(43.5, 99, Math.toRadians(130));
    private final Pose parkPose = new Pose(25,90,Math.toRadians(180));





    // Creating the paths from the poses
    private Path scorePreload;
    private PathChain  collectOne, release, scoreOne, collectTwo, scoreTwo, collectThree, scoreThree, park;

    public void waitM (double time){
        pathTimer.reset();
        while (pathTimer.milliseconds() < time){

        }
    }
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, shootOnePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), shootOnePose.getHeading());

        collectOne = follower.pathBuilder()
                .addPath(new BezierCurve(shootOnePose,collectControlOnePose,collectOnePose))
                .setLinearHeadingInterpolation(shootOnePose.getHeading(),collectOnePose.getHeading())
                .build();

        release = follower.pathBuilder()
                .addPath(new BezierCurve(collectOnePose,releasePose))
                .setLinearHeadingInterpolation(collectOnePose.getHeading(),releasePose.getHeading())
                .build();

        scoreOne = follower.pathBuilder()
                .addPath(new BezierCurve(releasePose,shootTwoControlPose,shootTwoPose))
                .setLinearHeadingInterpolation(releasePose.getHeading(),shootTwoPose.getHeading())
                .build();

        collectTwo = follower.pathBuilder()
                .addPath(new BezierCurve(shootTwoPose,collectControlTwoPose,collectTwoPose))
                .setLinearHeadingInterpolation(shootTwoPose.getHeading(),collectTwoPose.getHeading())
                .build();

        scoreTwo = follower.pathBuilder()
                .addPath(new BezierCurve(collectTwoPose,shootThreePose))
                .setLinearHeadingInterpolation(collectTwoPose.getHeading(),shootThreePose.getHeading())
                .build();

        collectThree = follower.pathBuilder()
                .addPath(new BezierCurve(shootThreePose,collectControlThreePose,collectThreePose))
                .setLinearHeadingInterpolation(shootThreePose.getHeading(),collectThreePose.getHeading())
                .build();

        scoreThree = follower.pathBuilder()
                .addPath(new BezierCurve(collectThreePose,shootFourPose))
                .setLinearHeadingInterpolation(collectThreePose.getHeading(),shootFourPose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierCurve(shootFourPose,parkPose))
                .setLinearHeadingInterpolation(shootFourPose.getHeading(),parkPose.getHeading())
                .build();
    }


    // Running the paths after creating them
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.isShootingOne = true;
                robot.isShootingTwo = true;
                robot.isShootingThree = true;
                robot.aprilID = 21;
                robot.shooter.setPower(RobotConstants.Drivetrain.shooterAuto);
                robot.intake.startIntaking();
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
                    // Shoot the first ball
                    waitM(1250);
                    robot.spindexer.sorting();
                    robot.intake.intakeDown();
                    robot.intake.startIntaking();
                    // Go to pick up the first set
                    follower.setMaxPower(0.7);
                    follower.followPath(collectOne);
//                    waitM(300);
//                    robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseTwo);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    waitM(500);
                    robot.intake.intakeUp();
                    robot.intake.stopIntaking();
                    follower.followPath(release);
                    setPathState(3);
                    break;
                }
            case 3:
                if (!follower.isBusy()){
                    follower.followPath(scoreOne,true);
                    //robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
                    setPathState(4);
                }

            case 4:
                if (!follower.isBusy()){
                    robot.spindexer.sorting();
                    robot.intake.intakeDown();
                    robot.intake.startIntaking();
                    follower.setMaxPower(0.7);
                    follower.followPath(collectTwo);
                    setPathState(5);
                }
            case 5:
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    waitM(500);
                    robot.intake.intakeUp();
                    robot.intake.stopIntaking();
                    follower.followPath(scoreTwo);
                    setPathState(6);
                }
            case 6:
                if (!follower.isBusy()){
                    robot.spindexer.sorting();
                    robot.intake.intakeDown();
                    robot.intake.startIntaking();
                    follower.setMaxPower(0.7);
                    follower.followPath(collectThree);
                    setPathState(7);
                }
            case 7:
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    waitM(500);
                    robot.intake.intakeUp();
                    robot.intake.stopIntaking();
                    follower.followPath(scoreThree);
                    setPathState(8);
                }
            case 8:
                if (!follower.isBusy()){
                    robot.spindexer.sorting();
                    follower.followPath(park);
                    setPathState(9);
                }
            case 9:
                if (!follower.isBusy()){
                    break;
                }
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
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
        robot.isShootingOne = true;
        robot.isShootingTwo = true;
        robot.isShootingThree = true;

        pathTimer = new ElapsedTime();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        robot.spindexer.shooterTimer = new ElapsedTime();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("action state", actionState);
        telemetry.addData("shot counter", robot.shotCounter);
//            telemetry.addData("robot shooting condition one" , robot.isShootingOne);
//            telemetry.addData("robot shooting condition two" , robot.isShootingTwo);
//            telemetry.addData("robot shooting condition three" , robot.isShootingThree);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Touch Sensor : ", !robot.spindexer.getTouchSensorState());
        robot.limelight.start();
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        robot.limelight.updateRobotOrientation(orientation.getYaw());
        robot.limelight.pipelineSwitch(1);
        LLResult llResult = robot.limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose();
            telemetry.addData("Target x", llResult.getTx());
            telemetry.addData("Target y", llResult.getTy());
            telemetry.addData("Target Area", llResult.getTa());
            telemetry.addData("BotPose", botPose.toString());
            telemetry.addData("Yaw", botPose.getOrientation().getYaw());
            List<LLResultTypes.FiducialResult> ID = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult id : ID) {
                robot.aprilID = id.getFiducialId();
                telemetry.addData("ID" ,robot.aprilID);
            }
            telemetry.update();
        }
    }

    @Override
    public void init_loop() {}


    @Override
    public void start() {
        opmodeTimer.resetTimer();
        actionTimer.resetTimer();
        robot.spindexer.shooterTimer.reset();
        robot.spindexer.secondShooterTimer.reset();
        robot.spindexer.thirdShooterTimer.reset();
        setPathState(0);
    }
}

