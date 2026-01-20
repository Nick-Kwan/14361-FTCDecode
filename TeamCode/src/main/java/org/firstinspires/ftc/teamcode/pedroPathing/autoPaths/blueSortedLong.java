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
import java.util.TimerTask;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@Autonomous (name = "Blue Sorted Long", group = "Auto")
public class blueSortedLong extends OpMode{


    private Follower follower;
    //private ElapsedTime pathTimer;
    private Timer llResetTimer;
    public java.util.Timer autoTimer;
    private  ScheduledExecutorService autoSchedule;
    private boolean ranCase0 = false;
    private boolean ranCase1 = false;
    private boolean ranCase2 = false;
    private boolean ranCase3 = false;
    private boolean ranCase4 = false;
    private boolean ranCase5 = false;
    private boolean ranCase6 = false;
    private boolean ranCase7 = false;
    private boolean ranCase8 = false;


    private int t;
    private int pathState;
    private LLResult llResult;
    private YawPitchRollAngles orientation;
    private boolean limelightTemp;
    private boolean limelightLoopTemp;
    private boolean tempAutoSpec = true;


    public ActionStates actionState = ActionStates.shootOne;
    public void setActionState(ActionStates state) {
        actionState = state;
    }

    private RobotHardware robot = RobotHardware.getInstance();
    private Runnable setPoseOne = () -> {
        robot.spindexer.setPoseOne();
    };

    private static double initX = 63.5;
    private static double initY = 8.5;
    private boolean pathTemp = false;
    private boolean actionTempOne, actionTempTwo,actionTempThree,actionTempFour,actionTempFive, actionTempSix = false;
    public double getInitX(){
        return initX;
    }
    public double getInitY(){
        return initY;
    }

    public blueSortedLong(){
        this.robot = RobotHardware.getInstance();
    }


    // Setting up the poses for the paths
    private final Pose startPose = new Pose(initX, initY, Math.toRadians(90));
    private final Pose collectOnePose = new Pose(17,8.5, Math.toRadians(180));
    private final Pose collectControlOnePose = new Pose(34.5,15,Math.toRadians(172));
    private final Pose shootOnePose = new Pose(56, 14, Math.toRadians(110));
    private final Pose collectTwoPose = new Pose(27,26,Math.toRadians(90));
    private final Pose collectControlTwoPose = new Pose(25,8,Math.toRadians(90));
    private final Pose shootTwoPose = new Pose(56, 14, Math.toRadians(110));
    private final Pose collectThreePose = new Pose(27,50,Math.toRadians(90));
    private final Pose collectControlThreePose = new Pose(25,28,Math.toRadians(135.8));
    private final Pose shootThreePose = new Pose(56, 14, Math.toRadians(110));
    private final Pose collectFourPose = new Pose(27,74,Math.toRadians(90));
    private final Pose collectControlFourPose = new Pose(25,50,Math.toRadians(151.5));
    private final Pose shootFourPose = new Pose(56, 14, Math.toRadians(110));
    private final Pose parkPose = new Pose(36,14,Math.toRadians(180));

    // Creating the paths from the poses
    private Path collectOne;
    private PathChain scoreOne, collectTwo, scoreTwo, collectThree, scoreThree, collectFour, scoreFour, park;
//    private Runnable setPoseOne = () -> {
//        robot.spindexer.setPoseOne();
//    };
//    private Runnable setPoseTwo = () -> {
//        robot.spindexer.setPoseTwo();
//    };
//    private Runnable setPoseThree = () -> {
//        robot.spindexer.setPoseThree();
//    };


    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        collectOne = new Path(new BezierCurve(startPose, collectControlOnePose, collectOnePose));
        collectOne.setLinearHeadingInterpolation(startPose.getHeading(), collectOnePose.getHeading());

        scoreOne = follower.pathBuilder()
                .addPath(new BezierCurve(collectOnePose,shootOnePose))
                .setLinearHeadingInterpolation(collectOnePose.getHeading(), shootOnePose.getHeading())
                .build();

        collectTwo = follower.pathBuilder()
                .addPath(new BezierCurve(shootOnePose,collectControlTwoPose,collectTwoPose))
                .setLinearHeadingInterpolation(shootOnePose.getHeading(), collectTwoPose.getHeading())
                .build();

        scoreTwo = follower.pathBuilder()
                .addPath(new BezierLine(collectTwoPose,shootTwoPose))
                .setLinearHeadingInterpolation(collectTwoPose.getHeading(),shootTwoPose.getHeading())
                .build();

        collectThree = follower.pathBuilder()
                .addPath(new BezierCurve(shootTwoPose,collectControlThreePose,collectThreePose))
                .setTangentHeadingInterpolation()
                .build();

        scoreThree = follower.pathBuilder()
                .addPath(new BezierLine(collectThreePose,shootThreePose))
                .setLinearHeadingInterpolation(collectThreePose.getHeading(),shootThreePose.getHeading())
                .build();

        collectFour = follower.pathBuilder()
                .addPath(new BezierCurve(shootThreePose,collectControlFourPose,collectFourPose))
                .setTangentHeadingInterpolation()
                .build();

        scoreFour = follower.pathBuilder()
                .addPath(new BezierLine(collectFourPose,shootFourPose))
                .setLinearHeadingInterpolation(collectFourPose.getHeading(),shootFourPose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(shootFourPose,parkPose))
                .setLinearHeadingInterpolation(shootFourPose.getHeading(), parkPose.getHeading())
                .build();

    }


    // Running the paths after creating them
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy() && !ranCase0){
                    ranCase0 = true;
                    t = 0;
                    robot.intake.setTargetVelocity(0.57);
                    follower.setMaxPower(0.8);
                    robot.adjustableHoodServo.setPosition(RobotConstants.Drivetrain.hoodPoseLong);
                    autoSchedule.schedule(() -> {
                        robot.spindexer.sortingAuto();
                    }, t += 1000 , TimeUnit.MILLISECONDS);
                    autoSchedule.schedule(() -> {
                        robot.spindexer.setPoseOne();
                    }, t += 1700 , TimeUnit.MILLISECONDS);
                    autoSchedule.schedule(() -> {
                        robot.intake.intakeDown();
                        robot.intake.startIntaking();
                        //robot.spindexer.setPoseOne();
                        follower.followPath(collectOne);
                        setPathState(1);
                    }, t += 1850 , TimeUnit.MILLISECONDS);
                }
                break;
            case 1:
                if(!follower.isBusy() && !ranCase1) {
                    ranCase1 = true;
                    t = 0;
                    autoSchedule.schedule(() -> {
                        robot.spindexer.setPoseTwo();
                    }, t += 300 , TimeUnit.MILLISECONDS);
                    autoSchedule.schedule(() -> {
                        //robot.turretServo.setPosition(RobotConstants.Drivetrain.turretPoseAutoLong);
                        //robot.spindexer.setPoseTwo();
                        follower.followPath(scoreOne,true);
                        setPathState(2);
                    }, t += 1000 , TimeUnit.MILLISECONDS);
                }
                break;
            case 2:
                if (!follower.isBusy() && !ranCase2){
                    ranCase2 = true;
                    t = 0;
//                    autoSchedule.schedule(() -> {
//                        robot.spindexer.sortingAuto();
//                    }, t += 500 , TimeUnit.MILLISECONDS);
                    robot.spindexer.sortingAuto();
                    autoSchedule.schedule(() -> {
                        robot.spindexer.setPoseOne();
                    }, t += 1700 , TimeUnit.MILLISECONDS);
                    autoSchedule.schedule(() -> {
                        //robot.spindexer.setPoseOne();
                        follower.followPath(collectTwo);
                        setPathState(3);
                    }, t += 1850 , TimeUnit.MILLISECONDS);
                    break;
                }
                break;
            case 3:
                if (!follower.isBusy() && !ranCase3){
                    ranCase3 = true;
                    t = 0;
                    autoSchedule.schedule(() -> {
                        robot.spindexer.setPoseTwo();
                    }, t += 200 , TimeUnit.MILLISECONDS);
                    autoSchedule.schedule(() -> {
                        //.turretServo.setPosition(RobotConstants.Drivetrain.turretPoseAutoLong);
                        //robot.spindexer.setPoseTwo();
                        follower.followPath(scoreTwo,true);
                        setPathState(4);
                    }, t += 1000 , TimeUnit.MILLISECONDS);
                }
                break;
            case 4:
                if (!follower.isBusy() && !ranCase4){
                    ranCase4 = true;
                    t = 0;
//                    autoSchedule.schedule(() -> {
//                        robot.spindexer.sortingAuto();
//                    }, t += 500 , TimeUnit.MILLISECONDS);
                    robot.spindexer.sortingAuto();
                    autoSchedule.schedule(() -> {
                        robot.spindexer.setPoseOne();
                    }, t += 1700 , TimeUnit.MILLISECONDS);
                    autoSchedule.schedule(() -> {
                        //robot.spindexer.setPoseOne();
                        follower.followPath(collectThree);
                        setPathState(5);
                    }, t += 1850 , TimeUnit.MILLISECONDS);
                }
                break;
            case 5:
                if (!follower.isBusy() && !ranCase5){
                    ranCase5 = true;
                    t = 0;
                    autoSchedule.schedule(() -> {
                        robot.spindexer.setPoseTwo();
                    }, t += 200 , TimeUnit.MILLISECONDS);
                    autoSchedule.schedule(() -> {
                        //robot.turretServo.setPosition(RobotConstants.Drivetrain.turretPoseAutoLong);
                        //robot.spindexer.setPoseTwo();
                        follower.followPath(scoreThree,true);
                        setPathState(6);
                    }, t += 1000 , TimeUnit.MILLISECONDS);
                }
                break;
            case 6:
                if (!follower.isBusy() && !ranCase6){
                    ranCase6 = true;
                    t = 0;
//                    autoSchedule.schedule(() -> {
//                        robot.spindexer.sortingAuto();
//                    }, t += 500 , TimeUnit.MILLISECONDS);
                    robot.spindexer.sortingAuto();
                    autoSchedule.schedule(() -> {
                        robot.spindexer.setPoseOne();
                    }, t += 1700 , TimeUnit.MILLISECONDS);
                    autoSchedule.schedule(() -> {
                        //robot.spindexer.setPoseOne();
                        follower.followPath(collectFour);
                        setPathState(7);
                    }, t += 1850 , TimeUnit.MILLISECONDS);
                }
                break;
            case 7:
                if (!follower.isBusy() && !ranCase7){
                    ranCase7 = true;
                    t = 0;
                    autoSchedule.schedule(() -> {
                        robot.spindexer.setPoseTwo();
                    }, t += 200 , TimeUnit.MILLISECONDS);
                    autoSchedule.schedule(() -> {
                       // robot.turretServo.setPosition(RobotConstants.Drivetrain.turretPoseAutoLong);
                        follower.followPath(scoreFour,true);
                        setPathState(8);
                    }, t += 1000 , TimeUnit.MILLISECONDS);
                }
                break;
            case 8:
                if (!follower.isBusy() && !ranCase8) {
                    ranCase8 = true;
                    t = 0;
//                    autoSchedule.schedule(() -> {
//                        robot.spindexer.sortingAuto();
//                    }, t += 500 , TimeUnit.MILLISECONDS);
                    robot.spindexer.sortingAuto();
                    autoSchedule.schedule(() -> {
                        follower.followPath(park);
                        setPathState(9);
                    }, t += 1600, TimeUnit.MILLISECONDS);
                }
                break;
            case 9:
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


    @Override
    public void loop() {
//        robot.intake.timerTaskSetup();
//        autoTimer = new java.util.Timer();

        CommandScheduler.getInstance().run();
        follower.update();
        autonomousPathUpdate();
        robot.intake.setAutoShooterVelocity();
        //robot.intake.timerTaskAutoSetup();
        robot.limelight.start();
        orientation = robot.imu.getRobotYawPitchRollAngles();
        robot.limelight.updateRobotOrientation(orientation.getYaw());
        llResult = robot.limelight.getLatestResult();
        if (robot.aprilID < 20){
            robot.limelight.pipelineSwitch(1);
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
                robot.turretServo.setPosition(RobotConstants.Drivetrain.turretPose);
                limelightLoopTemp = true;
            }


        }
        telemetry.addData("Path Completion: ", follower.getPathCompletion());
        telemetry.addData("Heading", follower.getHeading());;
        telemetry.addData("ID", robot.aprilID);

        telemetry.update();

    }

    @Override
    public void init() {
        CommandScheduler.getInstance().run();
        robot.init(hardwareMap);
        robot.turretServo.setPosition(RobotConstants.Drivetrain.turretPose);
        robot.adjustableHoodServo.setPosition(RobotConstants.Drivetrain.hoodPoseLong);
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

//        robot.s = Executors.newScheduledThreadPool(1);
//        robot.d = 0;
        //robot.timer1 = new java.util.Timer();

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
