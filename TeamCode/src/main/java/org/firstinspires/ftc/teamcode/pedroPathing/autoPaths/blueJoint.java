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
import org.firstinspires.ftc.teamcode.States.SortingStates;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@Autonomous (name = "Blue Joint", group = "Auto")
public class blueJoint extends OpMode{

    private Follower follower;
    private Timer  actionTimer, opmodeTimer;
    private ElapsedTime pathTimer;
    private Timer llResetTimer;
    private int pathState;
    private boolean limelightTemp;
    private boolean limelightLoopTemp;
    private int t;
    private ScheduledExecutorService autoSchedule;
    private boolean tempAutoSpec = true;
    private boolean ranCase0 = false;
    private boolean ranCase1 = false;
    private boolean ranCase2 = false;
    private boolean ranCase3 = false;
    private boolean ranCase4 = false;
    private boolean ranCase5 = false;
    private boolean ranCase6 = false;
    private boolean ranCase7 = false;
    private boolean ranCase8 = false;
    private LLResult llResult;
    private YawPitchRollAngles orientation;

    public ActionStates actionState = ActionStates.shootOne;
    public void setActionState(ActionStates state) {
        actionState = state;
    }

    private RobotHardware robot = RobotHardware.getInstance();

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

    public blueJoint(){
        this.robot = RobotHardware.getInstance();
    }

    // Setting up the poses for the paths
    private final Pose startPose = new Pose(initX, initY, Math.toRadians(90));

    private final Pose shootOnePose = new Pose(53.5, 90, Math.toRadians(180));
    private final Pose collectOnePose = new Pose(25.5,62,Math.toRadians(180));
    private final Pose collectControlOnePose = new Pose(72,62);
    private final Pose releasePose = new Pose(19,65,Math.toRadians(270));
    private final Pose releaseControlPose = new Pose(28,65);
    private final Pose shootTwoPose = new Pose(56.5, 86.5);
    private final Pose shootControlTwoPose = new Pose(49.5, 65);
    private final Pose shootControl2TwoPose = new Pose(106,86.5);
    private final Pose collectTwoPose = new Pose(27,86);
    private final Pose collectControlTwoPose = new Pose(56,86);
    private final Pose shootThreePose = new Pose(54, 89);
    private final Pose shootControlThreePose = new Pose(49, 89);
    private final Pose parkPose = new Pose(24,89);






    // Creating the paths from the poses
    private Path scorePreload;
    private PathChain  collectOne, release, scoreOne, prepareCollectTwo, collectTwo, scoreTwo, collectThree, scoreThree, park;
    private Runnable setPoseOne = () -> {
        robot.spindexer.setPoseOne();
    };
    private Runnable setPoseTwo = () -> {
        robot.spindexer.setPoseTwo();
    };
    private Runnable setPoseThree = () -> {
        robot.spindexer.setPoseThree();
    };






    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, shootOnePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), shootOnePose.getHeading());
        collectOne = follower.pathBuilder()
                .addPath(new BezierCurve(shootOnePose,collectControlOnePose,collectOnePose))
                .setLinearHeadingInterpolation(shootOnePose.getHeading(),collectOnePose.getHeading())
                .addParametricCallback(0.2,setPoseOne)
                .addParametricCallback(0.84,setPoseTwo)
                .addParametricCallback(0.93,setPoseThree)
                //.addParametricCallback(0.75,setPoseTwo)
                .build();

        release = follower.pathBuilder()
                .addPath(new BezierCurve(collectOnePose,releaseControlPose,releasePose))
                .setLinearHeadingInterpolation(collectOnePose.getHeading(), releasePose.getHeading())
                .build();

        scoreOne = follower.pathBuilder()
                .addPath(new BezierCurve(releasePose,shootControlTwoPose,shootTwoPose))
                .setLinearHeadingInterpolation(releasePose.getHeading(), shootTwoPose.getHeading())
                .build();

        collectTwo = follower.pathBuilder()
                .addPath(new BezierCurve(shootTwoPose,collectControlTwoPose,collectTwoPose))
                .setLinearHeadingInterpolation(shootTwoPose.getHeading(), collectTwoPose.getHeading())
                .addParametricCallback(0.2,setPoseThree)
                .addParametricCallback(0.83,setPoseTwo)
                .addParametricCallback(0.92,setPoseOne)
                .build();

        scoreTwo = follower.pathBuilder()
                .addPath(new BezierCurve(collectTwoPose,shootControlThreePose,shootThreePose))
                .setLinearHeadingInterpolation(collectTwoPose.getHeading(), shootThreePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierCurve(shootThreePose,parkPose))
                .setConstantHeadingInterpolation(parkPose.getHeading())
                .build();


    }


    // Running the paths after creating them
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy() && !ranCase0){
                    ranCase0 = true;
                    t = 0;
                    // old 0.515
                    robot.intake.setTargetVelocity(0.46);
                    robot.spindexer.setPoseTwo();
                    follower.setMaxPower(1.0);
                    follower.followPath(scorePreload);
                    setPathState(1);
                }
                break;
            case 1:
                if(!follower.isBusy() && !ranCase1) {
                    // Shoot the first ball
                    ranCase1 = true;
                    t = 0;
                    autoSchedule.schedule(() -> {
                        robot.spindexer.sortingAuto();
                    }, t += 1000, TimeUnit.MILLISECONDS);
//                    waitM(100);
//                    robot.spindexer.sortingRedAuto();
                    autoSchedule.schedule(() -> {
                        robot.spindexer.setPoseOne();
                        robot.intake.intakeDown();
                        robot.intake.startIntaking();
                        follower.setMaxPower(0.8);
                        follower.followPath(collectOne,true);
//                        actionTimer.resetTimer();
                        setPathState(2);
                    }, t += 1900, TimeUnit.MILLISECONDS);
                }
                break;
            case 2:
                if (!follower.isBusy() && !ranCase2){
                    ranCase2 = true;
                    t = 0;
                    autoSchedule.schedule(() -> {
                        robot.intake.intakeUp();
                        robot.intake.stopIntaking();
                        follower.followPath(release);
                        setPathState(3);
                    }, t += 500, TimeUnit.MILLISECONDS);
                }
                break;
            case 3:
                if (!follower.isBusy() && !ranCase3){
                    ranCase3 = true;
                    t = 0;
                    follower.setMaxPower(1.0);
//                    robot.intake.startIntakingMax();
                    autoSchedule.schedule(() -> {
                        follower.followPath(scoreOne);
                        setPathState(4);
                    }, t += 7500, TimeUnit.MILLISECONDS);
                }
                break;
            case 4:
                if (!follower.isBusy() && !ranCase4){
                    ranCase4 = true;
                    t = 0;
                    autoSchedule.schedule(() -> {
                        robot.spindexer.setPoseTwo();
                    }, t += 1000, TimeUnit.MILLISECONDS);
                    autoSchedule.schedule(() -> {
                        robot.spindexer.sortingAuto();
                    }, t += 250, TimeUnit.MILLISECONDS);
                    autoSchedule.schedule(() -> {
                        robot.intake.intakeDown();
                        robot.intake.startIntaking();
                        robot.spindexer.setPoseThree();
                        follower.setMaxPower(0.7);
                        follower.followPath(collectTwo,true);
                        setPathState(5);
                    }, t += 2200, TimeUnit.MILLISECONDS);
                }
                break;

            case 5:
                if (!follower.isBusy() && !ranCase5){
                    ranCase5 = true;
                    t = 0;
                    follower.setMaxPower(1.0);
                    autoSchedule.schedule(() -> {
                        robot.intake.startIntakingMax();
                        follower.followPath(scoreTwo);
                        setPathState(6);
                    }, t += 500, TimeUnit.MILLISECONDS);
                }
                break;
            case 6:
                if (!follower.isBusy() && !ranCase6){
                    ranCase6 = true;
                    t = 0;
                    robot.spindexer.setPoseTwo();
                    robot.intake.intakeUp();
                    autoSchedule.schedule(() -> {
                        robot.spindexer.sortingAuto();
                    }, t += 1000, TimeUnit.MILLISECONDS);
                    autoSchedule.schedule(() -> {
                        robot.spindexer.setPoseOne();
                        robot.intake.intakeUp();
                        robot.intake.stopIntaking();
                        follower.setMaxPower(1.0);
                        follower.followPath(park);
                        setPathState(7);
                    }, t += 1900, TimeUnit.MILLISECONDS);
                }
                break;
            case 7:
                if (!follower.isBusy()){
                    robot.s.shutdown();
                    break;
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        //robot.pathTimer.reset();
    }


//    public void waitM (double time){
//        robot.pathTimer.reset();
//        while (robot.pathTimer.milliseconds() < time){
    ////            follower.update();
    ////            autonomousPathUpdate();
//            orientation = robot.imu.getRobotYawPitchRollAngles();
//            robot.limelight.updateRobotOrientation(orientation.getYaw());
//            robot.intake.setAutoShooterVelocity();
//            llResult = robot.limelight.getLatestResult();
//            if (robot.aprilID < 20){
//                robot.limelight.pipelineSwitch(0);
//                limelightTemp = false;
//            }
//            else if (robot.aprilID > 20){
//                robot.limelight.pipelineSwitch(2);
//                limelightTemp = true;
//            }
//            if (llResult != null && llResult.isValid()) {
//                Pose3D botPose = llResult.getBotpose();
//                List<LLResultTypes.FiducialResult> ID = llResult.getFiducialResults();
//                for (LLResultTypes.FiducialResult id : ID) {
//                    if (id.getFiducialId() == 21 || id.getFiducialId() == 22 || id.getFiducialId() == 23){
//                        robot.aprilID = id.getFiducialId();
//                    }
//                    else {
//                        break;
//                    }
//                    telemetry.addData("ID", robot.aprilID);
//                }
//                if (limelightTemp){
//                    if (llResult.getTx() > 2){
//                        robot.turretServo.setPosition(robot.turretServo.getPosition() + llResult.getTx() / 1000);
//                    }
//                    else if (llResult.getTx() < -2){
//                        robot.turretServo.setPosition(robot.turretServo.getPosition() + llResult.getTx() / 1000);
//                    }
//                    limelightLoopTemp = true;
//                }
//            }
//            else if (!llResult.isValid()){
//                if (limelightLoopTemp){
//                    robot.llResetTimer.resetTimer();
//                    limelightLoopTemp = false;
//                }
//                if (robot.llResetTimer.getElapsedTimeSeconds() > 1 && llResult.getTx() == 0){
//                    robot.turretServo.setPosition(RobotConstants.Drivetrain.turretRedAutoPose);
//                    limelightLoopTemp = true;
//                }
//
//
//            }
//        }
//    }



    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        follower.update();
        autonomousPathUpdate();
        robot.intake.setAutoShooterVelocity();
        robot.limelight.start();
        orientation = robot.imu.getRobotYawPitchRollAngles();
        robot.limelight.updateRobotOrientation(orientation.getYaw());
        llResult = robot.limelight.getLatestResult();
        if (robot.aprilID < 20){
            robot.limelight.pipelineSwitch(0);
            limelightTemp = false;
        }
        else if (robot.aprilID > 20){
            robot.limelight.pipelineSwitch(2);
            limelightTemp = true;
        }
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose();
            telemetry.addData("Target x", llResult.getTx());
            telemetry.addData("Target y", llResult.getTy());
            telemetry.addData("Target Area", llResult.getTa());
            telemetry.addData("BotPose", botPose.toString());
            telemetry.addData("Yaw", botPose.getOrientation().getYaw());
            List<LLResultTypes.FiducialResult> ID = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult id : ID) {
                if (id.getFiducialId() == 21 || id.getFiducialId() == 22 || id.getFiducialId() == 23){
                    robot.aprilID = id.getFiducialId();
                }
                else {
                    break;
                }
                telemetry.addData("ID", robot.aprilID);
            }
            if (limelightTemp){
                if (llResult.getTx() > 2){
                    robot.turretServo.setPosition(robot.turretServo.getPosition() + llResult.getTx() / 750);
                }
                else if (llResult.getTx() < -2){
                    robot.turretServo.setPosition(robot.turretServo.getPosition() + llResult.getTx() / 750);
                }
                limelightLoopTemp = true;
            }
        }
        else if (!llResult.isValid()){
            if (limelightLoopTemp){
                llResetTimer.resetTimer();
                limelightLoopTemp = false;
            }
            if (llResetTimer.getElapsedTimeSeconds() > 1 && llResult.getTx() == 0){
                robot.turretServo.setPosition(RobotConstants.Drivetrain.turretRedAutoPose);
                limelightLoopTemp = true;
            }


        }
        telemetry.addData("Path Completion: ", follower.getPathCompletion());
        telemetry.addData("Heading", follower.getHeading());;
        telemetry.addData("ID", robot.aprilID);
        List<Double> velocities = robot.shooterMotors.getVelocities();
        telemetry.addData("Left Flywheel Velocity", velocities.get(0));
        telemetry.addData("Right Flywheel Velocity", velocities.get(1));
        telemetry.addData("Busy State: ", follower.isBusy());

        telemetry.update();

    }

    @Override
    public void init() {
        CommandScheduler.getInstance().run();
        robot.init(hardwareMap);
        robot.turretServo.setPosition(RobotConstants.Drivetrain.turretRedAutoPose);
        robot.adjustableHoodServo.setPosition(RobotConstants.Drivetrain.hoodPoseAuto);
        robot.spindexer.setPoseTwo();


        robot.pathTimer = new ElapsedTime();
        llResetTimer = new Timer();

        autoSchedule = Executors.newScheduledThreadPool(1);
        t = 0;

        ranCase0 = false;
        ranCase1 = false;
        ranCase2 = false;
        ranCase3 = false;
        ranCase4 = false;
        ranCase5 = false;
        ranCase6 = false;
        ranCase7 = false;
        ranCase8 = false;


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();


        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("action state", actionState);
        telemetry.addData("shot counter", robot.shotCounter);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Touch Sensor : ", !robot.spindexer.getTouchSensorState());
    }

    @Override
    public void init_loop() {}


    @Override
    public void start() {
        robot.pathTimer.reset();
        llResetTimer.resetTimer();
        setPathState(0);
    }
}
