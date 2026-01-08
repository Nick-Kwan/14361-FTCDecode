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

@Autonomous (name = "Red Sorted", group = "Auto")
public class redSorted extends OpMode{

    private Follower follower;
    private Timer  actionTimer, opmodeTimer;
    private ElapsedTime pathTimer;
    private Timer llResetTimer;
    private int pathState;
    private boolean limelightTemp;
    private boolean limelightLoopTemp;
    private boolean tempAutoSpec = true;
    private LLResult llResult;
    private YawPitchRollAngles orientation;

    public ActionStates actionState = ActionStates.shootOne;
    public void setActionState(ActionStates state) {
        actionState = state;
    }

    private final RobotHardware robot = RobotHardware.getInstance();

    private static double initX = 125.5;
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

    private final Pose shootOnePose = new Pose(90.5, 90, Math.toRadians(40));
    private final Pose collectOnePose = new Pose(119,80,Math.toRadians(-10));
    private final Pose collectControlOnePose = new Pose(74,84,Math.toRadians(-10));
    //private final Pose releasePose = new Pose(20,75,Math.toRadians(90));
    private final Pose shootTwoPose = new Pose(88.5, 89, Math.toRadians(0));
    private final Pose prepareCollectTwoPose = new Pose(88,57,Math.toRadians(-10));
    //private final Pose prepareCollectControlTwoPose = new Pose(63.5,62,Math.toRadians(180));
    //private final Pose shootControlTwoPose = new Pose(76, 57.5, Math.toRadians(130));
    //private final Pose shootTwoControlPose = new Pose(44.5, 71.5, Math.toRadians(110));
    private final Pose collectTwoPose = new Pose(113,57,Math.toRadians(-10));
    // private final Pose collectControlTwoPose = new Pose(77,54.5,Math.toRadians(180));
    private final Pose shootThreePose = new Pose(84, 90, Math.toRadians(0));
    private final Pose parkPose = new Pose(115,90,Math.toRadians(0));

    // Not in use rn
    private final Pose collectThreePose = new Pose(25,34,Math.toRadians(180));
    private final Pose collectControlThreePose = new Pose(76,33,Math.toRadians(180));
    private final Pose shootFourPose = new Pose(43.5, 99, Math.toRadians(130));





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
                .addParametricCallback(0.2,setPoseThree)
                .addParametricCallback(0.77,setPoseTwo)
                .addParametricCallback(0.93,setPoseOne)
                //.addParametricCallback(0.75,setPoseTwo)
                .build();

//        release = follower.pathBuilder()
//                .addPath(new BezierCurve(collectOnePose,releasePose))
//                .setLinearHeadingInterpolation(collectOnePose.getHeading(),releasePose.getHeading())
//                .build();

        scoreOne = follower.pathBuilder()
                .addPath(new BezierCurve(collectOnePose,shootTwoPose))
                .setLinearHeadingInterpolation(collectOnePose.getHeading(),shootTwoPose.getHeading())
                .build();

        prepareCollectTwo = follower.pathBuilder()
                .addPath(new BezierCurve(shootTwoPose,prepareCollectTwoPose))
                .setLinearHeadingInterpolation(shootTwoPose.getHeading(), prepareCollectTwoPose.getHeading())
                .build();

        collectTwo = follower.pathBuilder()
                .addPath(new BezierCurve(prepareCollectTwoPose,collectTwoPose))
                .setLinearHeadingInterpolation(prepareCollectTwoPose.getHeading(),collectTwoPose.getHeading())
                .addParametricCallback(0.2,setPoseThree)
                .addParametricCallback(0.725,setPoseTwo)
                .addParametricCallback(0.85,setPoseOne)
                .build();

        scoreTwo = follower.pathBuilder()
                .addPath(new BezierCurve(collectTwoPose,shootThreePose))
                .setLinearHeadingInterpolation(collectTwoPose.getHeading(),shootThreePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierCurve(shootThreePose,parkPose))
                .setLinearHeadingInterpolation(shootThreePose.getHeading(),parkPose.getHeading())
                .build();

    }


    // Running the paths after creating them
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.shooterMotors.set(0.62);
                robot.adjustableHoodServo.setPosition(RobotConstants.Drivetrain.hoodPoseAuto);
                robot.spindexer.setPoseThree();
                follower.setMaxPower(0.67);
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
                    robot.turretServo.setPosition(RobotConstants.Drivetrain.turretPose);
                    //waitM(1250);
                    //robot.spindexer.sorting();
                    robot.intake.intakeDown();
                    robot.intake.startIntaking();
                    // Go to pick up the first set
                    //follower.setMaxPower(0.65);
                    //robot.spindexer.setPoseOne();
                    follower.setMaxPower(0.5);
                    follower.followPath(collectOne);
                    actionTimer.resetTimer();
//                    waitM(300);
//                    robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseTwo);
                    setPathState(3);
                }
                break;
            // IT SKIPS THIS RN
            case 2:
//                if (tempAutoSpec) {
//                    actionTimer.resetTimer();
//                    tempAutoSpec = false;
//                }
//                if (actionTimer.getElapsedTimeSeconds() > 0.5){
//                    robot.spindexer.setPoseTwo();
//                }
//                if (follower.getPathCompletion() > 0.4 || follower.getPathCompletion() < 0.9){
//                    robot.spindexer.setPoseTwo();
//                }
                if (!follower.isBusy()){
                    //robot.spindexer.setPoseTwo();
                    //robot.spindexer.setPoseOne();
                    waitM(500);
                    robot.intake.intakeUp();
                    follower.setMaxPower(0.7);
//                    waitM(500);
//                    robot.intake.intakeUp();
//                    robot.intake.stopIntaking();
                    follower.followPath(release);
                    setPathState(3);
                    break;
                }
            case 3:
                if (!follower.isBusy()){
                    follower.setMaxPower(0.67);
                    robot.intake.intakeUp();
                    robot.intake.startIntakingMax();
                    //robot.intake.intakeUp();
                    waitM(500);
                    //robot.spindexer.setPoseThree();
                    follower.followPath(scoreOne,true);
                    //robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
                    setPathState(4);
                }

            case 4:
                if (!follower.isBusy()){
                    while (llResult.isValid() && (llResult.getTx() > 1 || llResult.getTx() < -1)){
//                            if (llResult.getTx() > 2){
//                                robot.turretServo.setPosition(robot.turretServo.getPosition() + llResult.getTx() / 550);
//                            }
//                            else if (llResult.getTx() < -2){
                        robot.turretServo.setPosition(robot.turretServo.getPosition() + llResult.getTx() / 650);
                        waitM(100);
                        llResult = robot.limelight.getLatestResult();
//                            }
//                            else {
//                                break;
//                            }
                    }
                    robot.spindexer.setPoseThree();
                    //robot.turretServo.setPosition(RobotConstants.Drivetrain.turretMeowPose);
                    //waitM(500);
                    //robot.spindexer.setSortingState(SortingStates.Checking);
                    //waitM(250);
                    robot.intake.intakeUp();

                    //robot.spindexer.sorting();
                    waitM(250);
                    robot.intake.intakeDown();
                    robot.intake.startIntaking();
                    robot.spindexer.setPoseThree();
                    follower.followPath(prepareCollectTwo);
                    setPathState(5);
                }
            case 5:
                if (!follower.isBusy()){
                    follower.setMaxPower(0.3);
                    follower.followPath(collectTwo);
                    setPathState(6);
                }
            case 6:
                if (!follower.isBusy()){
                    follower.setMaxPower(0.67);
                    waitM(500);
                    //robot.spindexer.setPoseThree();
                    robot.intake.intakeUp();
                    robot.intake.startIntakingMax();
                    follower.followPath(scoreTwo);
                    setPathState(7);
                }
            case 7:
                if (!follower.isBusy()){
                    while (llResult.isValid() && (llResult.getTx() > 1 || llResult.getTx() < -1)){
//                            if (llResult.getTx() > 2){
//                                robot.turretServo.setPosition(robot.turretServo.getPosition() + llResult.getTx() / 550);
//                            }
//                            else if (llResult.getTx() < -2){
                        robot.turretServo.setPosition(robot.turretServo.getPosition() + llResult.getTx() / 650);
                        waitM(100);
                        llResult = robot.limelight.getLatestResult();
//                            }
//                            else {
//                                break;
//                            }
                    }
                    robot.spindexer.setPoseThree();
                    //robot.turretServo.setPosition(RobotConstants.Drivetrain.turretMeowPose);
                    //waitM(500);
                    //robot.spindexer.sorting();
                    robot.intake.intakeUp();
                    robot.intake.stopIntaking();
                    follower.setMaxPower(0.5);
                    follower.followPath(park);
                    setPathState(8);
                }
            case 8:
                if (!follower.isBusy()){
                    break;
                }
            case 9:
                if (!follower.isBusy()){
                    //robot.spindexer.sorting();
                    follower.followPath(park);
                    setPathState(10);
                }
            case 10:
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
                if (llResult.getTx() > 1){
                    robot.turretServo.setPosition(robot.turretServo.getPosition() + llResult.getTx() / 725);
                }
                else if (llResult.getTx() < -1){
                    robot.turretServo.setPosition(robot.turretServo.getPosition() + llResult.getTx() / 725);
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
        telemetry.addData("Action Timer: ", actionTimer.getElapsedTime());
        telemetry.addData("Path Completion: ", follower.getPathCompletion());
        telemetry.addData("ID", robot.aprilID);
        telemetry.update();

    }

    @Override
    public void init() {
        CommandScheduler.getInstance().run();
        robot.init(hardwareMap);
        robot.turretServo.setPosition(RobotConstants.Drivetrain.turretRedAutoPose);


        pathTimer = new ElapsedTime();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        llResetTimer = new Timer();
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
//        robot.limelight.start();
//        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
//        robot.limelight.updateRobotOrientation(orientation.getYaw());
//        robot.limelight.pipelineSwitch(1);
//        LLResult llResult = robot.limelight.getLatestResult();
//        if (llResult != null && llResult.isValid()) {
//            Pose3D botPose = llResult.getBotpose();
//            telemetry.addData("Target x", llResult.getTx());
//            telemetry.addData("Target y", llResult.getTy());
//            telemetry.addData("Target Area", llResult.getTa());
//            telemetry.addData("BotPose", botPose.toString());
//            telemetry.addData("Yaw", botPose.getOrientation().getYaw());
//            List<LLResultTypes.FiducialResult> ID = llResult.getFiducialResults();
//            for (LLResultTypes.FiducialResult id : ID) {
//                robot.aprilID = id.getFiducialId();
//                telemetry.addData("ID" ,robot.aprilID);
//            }
//            telemetry.update();
//        }
    }

    @Override
    public void init_loop() {}


    @Override
    public void start() {
        opmodeTimer.resetTimer();
        actionTimer.resetTimer();
        llResetTimer.resetTimer();
        robot.spindexer.shooterTimer.reset();
        robot.spindexer.secondShooterTimer.reset();
        robot.spindexer.thirdShooterTimer.reset();
        setPathState(0);
    }
}
