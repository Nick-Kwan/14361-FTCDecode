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

@Autonomous (name = "Blue Sorted", group = "Auto")
public class blueSorted extends OpMode{


    private Follower follower;
    private Timer opmodeTimer;
    //private ElapsedTime pathTimer;
    private Timer llResetTimer;
    private int pathState;
    private LLResult llResult;
    private YawPitchRollAngles orientation;
    private boolean limelightTemp;
    private boolean limelightLoopTemp;
    private ScheduledExecutorService autoSchedule;
    private int t;
    private boolean ranCase0 = false;
    private boolean ranCase1 = false;
    private boolean ranCase2 = false;
    private boolean ranCase3 = false;
    private boolean ranCase4 = false;
    private boolean ranCase5 = false;
    private boolean ranCase6 = false;
    private boolean ranCase7 = false;
    private boolean ranCase8 = false;
    private boolean ranCase9 = false;
    private boolean tempAutoSpec = true;

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

    public blueSorted(){
        this.robot = RobotHardware.getInstance();
    }


    // Setting up the poses for the paths
    private final Pose startPose = new Pose(initX, initY, Math.toRadians(90));

    private final Pose shootOnePose = new Pose(53.5, 90, Math.toRadians(180));
    private final Pose collectOnePose = new Pose(24,84);
    private final Pose collectControlOnePose = new Pose(54,84);
    //private final Pose releasePose = new Pose(20,75,Math.toRadians(90));
    private final Pose shootTwoPose = new Pose(54,89);
    private final Pose shootControlTwoPose = new Pose(49, 89);
    private final Pose collectTwoPose = new Pose(44,61);
    private final Pose goCollectTwoPose = new Pose(25,61);
    private final Pose collectControlTwoPose = new Pose(72,61);
    private final Pose shootThreePose = new Pose(57, 86.5);
    private final Pose shootControlThreePose = new Pose(49, 61);
    private final Pose shootControl2ThreePose = new Pose(40.5, 86.5);
    private final Pose collectThreePose = new Pose(46,37);
    private final Pose goCollectThreePose = new Pose(23,37);

    private final Pose collectControlThreePose = new Pose(84,33.5);
    private final Pose shootFourPose = new Pose(54, 88);
    private final Pose shootControlFourPose = new Pose(39, 37);
    private final Pose shootControl2FourPose = new Pose(34, 88);
    private final Pose parkPose = new Pose(24,88);
    // Not in use rn
//    private final Pose collectThreePose = new Pose(25,34,Math.toRadians(180));
//    private final Pose collectControlThreePose = new Pose(76,33,Math.toRadians(180));
//    private final Pose shootFourPose = new Pose(43.5, 99, Math.toRadians(130));


    // Creating the paths from the poses
    private Path scorePreload;
    private PathChain  collectOne, release, scoreOne, prepareCollectTwo, collectTwo, scoreTwo, collectThree, scoreThree, park, goCollectTwo, goCollectThree;
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
                .addParametricCallback(0.2,setPoseThree)
                .addParametricCallback(0.84,setPoseTwo)
                .addParametricCallback(0.93,setPoseOne)
                //.addParametricCallback(0.75,setPoseTwo)
                .build();

        scoreOne = follower.pathBuilder()
                .addPath(new BezierCurve(collectOnePose,shootControlTwoPose,shootTwoPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .setLinearHeadingInterpolation(collectOnePose.getHeading(), shootTwoPose.getHeading())
                .build();

        collectTwo = follower.pathBuilder()
                .addPath(new BezierLine(shootTwoPose,collectTwoPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .setLinearHeadingInterpolation(shootTwoPose.getHeading(), collectTwoPose.getHeading())
//                .addParametricCallback(0.2,setPoseThree)
//                .addParametricCallback(0.83,setPoseTwo)
//                .addParametricCallback(0.92,setPoseOne)
                .build();

        goCollectTwo = follower.pathBuilder()
                .addPath(new BezierLine(collectTwoPose,goCollectTwoPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .setLinearHeadingInterpolation(collectTwoPose.getHeading(), goCollectTwoPose.getHeading())
                .addParametricCallback(0.2,setPoseThree)
                //.addParametricCallback(0.83,setPoseTwo)
                .addParametricCallback(0.3,setPoseOne)
                .build();


        scoreTwo = follower.pathBuilder()
                .addPath(new BezierLine(goCollectTwoPose,shootThreePose))
//                .setLinearHeadingInterpolation(goCollectTwoPose.getHeading(), shootThreePose.getHeading())
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        collectThree = follower.pathBuilder()
                .addPath(new BezierLine(shootThreePose,collectThreePose))
//                .setLinearHeadingInterpolation(shootThreePose.getHeading(), collectThreePose.getHeading())
                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .addParametricCallback(0.2,setPoseThree)
//                .addParametricCallback(0.84,setPoseTwo)
//                .addParametricCallback(0.93,setPoseOne)
                .build();

        goCollectThree = follower.pathBuilder()
                .addPath(new BezierLine(collectThreePose,goCollectThreePose))
//                .setLinearHeadingInterpolation(collectThreePose.getHeading(), goCollectThreePose.getHeading())
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.2,setPoseThree)
                //.addParametricCallback(0.83,setPoseTwo)
                .addParametricCallback(0.3,setPoseOne)
                .build();

        scoreThree = follower.pathBuilder()
                .addPath(new BezierCurve(goCollectThreePose,shootControlFourPose,shootControl2FourPose,shootFourPose))
//                .setLinearHeadingInterpolation(goCollectThreePose.getHeading(), shootFourPose.getHeading())
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(shootFourPose,parkPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
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
                    follower.followPath(scorePreload,false);
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
                        robot.spindexer.setPoseThree();
                        robot.intake.intakeDown();
                        robot.intake.startIntaking();
                        follower.setMaxPower(1.0);
                        follower.followPath(collectOne,false);
//                        actionTimer.resetTimer();
                        setPathState(2);
                    }, t += 2500, TimeUnit.MILLISECONDS);
//                    waitM(250);
//                    robot.spindexer.setPoseThree();
//                    robot.intake.intakeDown();
//                    robot.intake.startIntaking();
//                    follower.setMaxPower(0.6);
//                    follower.followPath(collectOne,true);
//                    actionTimer.resetTimer();
//                    setPathState(3);
                }
                break;
            case 2:
                if (!follower.isBusy() && !ranCase2){
                    ranCase2 = true;
                    t = 0;
                    follower.setMaxPower(1.0);
//                    robot.intake.startIntakingMax();
                    autoSchedule.schedule(() -> {
                        follower.followPath(scoreOne,false);
                        setPathState(3);
                    }, t += 500, TimeUnit.MILLISECONDS);
                }
                break;
            case 3:
                if (!follower.isBusy() && !ranCase3){
                    ranCase3 = true;
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
                        follower.setMaxPower(1.0);
                        follower.followPath(collectTwo,false);
                        setPathState(4);
                    }, t += 2500, TimeUnit.MILLISECONDS);
                }
                break;

            case 4:
                if (!follower.isBusy() && !ranCase4){
                    ranCase4 = true;
                    t = 0;
                    follower.setMaxPower(0.5);
                    follower.followPath(goCollectTwo,false);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() && !ranCase5){
                    ranCase5 = true;
                    t = 0;
                    follower.setMaxPower(1.0);
                    autoSchedule.schedule(() -> {
                        follower.followPath(scoreTwo,false);
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
                        robot.spindexer.setPoseThree();
                        robot.intake.intakeDown();
                        robot.intake.startIntaking();
                        follower.setMaxPower(1.0);
                        follower.followPath(collectThree,false);
                        setPathState(7);
                    }, t += 2500, TimeUnit.MILLISECONDS);
                }
                break;
            case 7:
                if (!follower.isBusy() && !ranCase7){
                    ranCase7 = true;
                    t = 0;
                    follower.setMaxPower(0.4);
                    follower.followPath(goCollectThree,false);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy() && !ranCase8){
                    ranCase8 = true;
                    t = 0;
                    follower.setMaxPower(1.0);
                    autoSchedule.schedule(() -> {
                        robot.intake.startIntakingMax();
                        follower.followPath(scoreThree,false);
                        setPathState(9);
                    }, t += 500, TimeUnit.MILLISECONDS);
                }
                break;
            case 9:
                if (!follower.isBusy() && !ranCase9){
                    ranCase9 = true;
                    t = 0;
                    robot.spindexer.setPoseTwo();
                    autoSchedule.schedule(() -> {
                        robot.spindexer.sortingAuto();
                    }, t += 1000, TimeUnit.MILLISECONDS);
                    autoSchedule.schedule(() -> {
                        robot.spindexer.setPoseOne();
                        robot.intake.intakeUp();
                        robot.intake.stopIntaking();
                        follower.followPath(park,false);
                        setPathState(10);
                    }, t += 2500, TimeUnit.MILLISECONDS);
                }
                break;
            case 10:
                if (!follower.isBusy()){
                    robot.s.shutdown();
                    break;
                }
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
            robot.limelight.pipelineSwitch(3);
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
                    robot.turretServo.setPosition(robot.turretServo.getPosition() + llResult.getTx() / 1000);
                }
                else if (llResult.getTx() < -2){
                    robot.turretServo.setPosition(robot.turretServo.getPosition() + llResult.getTx() / 1000);
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
                robot.turretServo.setPosition(RobotConstants.Drivetrain.turretBlueAutoPose);
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
        robot.turretServo.setPosition(RobotConstants.Drivetrain.turretBlueAutoPose);
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
        ranCase9 = false;


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