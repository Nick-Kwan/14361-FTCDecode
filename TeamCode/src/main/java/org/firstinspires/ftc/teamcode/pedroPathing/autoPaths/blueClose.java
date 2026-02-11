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

@Autonomous (name = "Blue Close", group = "Auto")
public class blueClose  extends OpMode{

    private Follower follower;
    private Timer  actionTimer, opmodeTimer;
    //private ElapsedTime pathTimer;
    private int pathState;

    public ActionStates actionState = ActionStates.shootOne;
    public void setActionState(ActionStates state) {
        actionState = state;
    }

    private final RobotHardware robot = RobotHardware.getInstance();

    private static double initX = RobotConstants.Auto.initXB;
    private static double initY = RobotConstants.Auto.initYB;
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

    private final Pose shootOnePose = new Pose(48, 97, Math.toRadians(130));

    private final Pose goingToCollectOnePose = new Pose(42, 86.5, Math.toRadians(180));
    private final Pose goingToCollectControlOnePose = new Pose(84, 85.5, Math.toRadians(180));
    private final Pose collectOnePose = new Pose(37,86.5,Math.toRadians(180));
    private final Pose collectTwoPose = new Pose(32,86.5,Math.toRadians(180));
    private final Pose collectThreePose = new Pose(25,86.5,Math.toRadians(180));
    private final Pose shootTwoPose = new Pose(48, 97, Math.toRadians(155));
    private final Pose collectFourPose = new Pose(42,63.5,Math.toRadians(180));
    private final Pose collectControlFourPose = new Pose(61.5,61.5,Math.toRadians(180));
    private final Pose collectFivePose = new Pose(37,63.5,Math.toRadians(180));
    private final Pose collectSixPose = new Pose(32,63.5,Math.toRadians(180));
    private final Pose shootThreePose = new Pose(48, 97, Math.toRadians(150));
    private final Pose collectSevenPose = new Pose(42,38,Math.toRadians(180));
    private final Pose collectControlSevenPose = new Pose(62,36,Math.toRadians(180));



    // Creating the paths from the poses
    private Path scorePreload;
    private PathChain goingToCollectOne, collectOne, collectTwo, collectThree, scoreOne, collectFour, collectFive, collectSix, scoreTwo, collectSeven;

    public void waitM (double time){
        robot.pathTimer.reset();
        while (robot.pathTimer.milliseconds() < time){

        }
    }
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, shootOnePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), shootOnePose.getHeading());

        collectOne = follower.pathBuilder()
                .addPath(new BezierCurve(shootOnePose,goingToCollectControlOnePose,goingToCollectOnePose))
                .setLinearHeadingInterpolation(shootOnePose.getHeading(),goingToCollectOnePose.getHeading())
                .build();

        collectTwo = follower.pathBuilder()
                .addPath(new BezierLine(goingToCollectOnePose,collectOnePose))
                .setLinearHeadingInterpolation(goingToCollectOnePose.getHeading(), collectOnePose.getHeading())
                .build();

        collectThree = follower.pathBuilder()
                .addPath(new BezierLine(collectOnePose,collectTwoPose))
                .setLinearHeadingInterpolation(collectOnePose.getHeading(), collectTwoPose.getHeading())
                .build();

//        collectThree = follower.pathBuilder()
//                .addPath(new BezierLine(collectTwoPose,collectThreePose))
//                .setLinearHeadingInterpolation(collectTwoPose.getHeading(), collectThreePose.getHeading())
//                .build();

        scoreOne = follower.pathBuilder()
                .addPath(new BezierLine(collectTwoPose,shootTwoPose))
                .setLinearHeadingInterpolation(collectTwoPose.getHeading(),shootTwoPose.getHeading())
                .build();

        collectFour = follower.pathBuilder()
                .addPath(new BezierCurve(shootTwoPose,collectControlFourPose,collectFourPose))
                .setLinearHeadingInterpolation(shootTwoPose.getHeading(), collectFourPose.getHeading())
                .build();

        collectFive = follower.pathBuilder()
                .addPath(new BezierLine(collectFourPose,collectFivePose))
                .setConstantHeadingInterpolation(collectFivePose.getHeading())
                .build();

        collectSix = follower.pathBuilder()
                .addPath(new BezierLine(collectFivePose,collectSixPose))
                .setConstantHeadingInterpolation(collectSixPose.getHeading())
                .build();

        scoreTwo = follower.pathBuilder()
                .addPath(new BezierLine(collectSixPose,shootThreePose))
                .setLinearHeadingInterpolation(collectSixPose.getHeading(), shootThreePose.getHeading())
                .build();

        collectSeven = follower.pathBuilder()
                .addPath(new BezierCurve(shootThreePose,collectControlSevenPose,collectSevenPose))
                .setLinearHeadingInterpolation(shootThreePose.getHeading(), collectSevenPose.getHeading())
                .build();
}


// Running the paths after creating them
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.isShootingOne = true;
                robot.isShootingTwo = true;
                robot.isShootingThree = true;
                robot.shooterOne.setPower(RobotConstants.Drivetrain.shooterAuto);
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
                    robot.spindexer.spindexerUp();
                    waitM(500);
                    robot.spindexer.spindexerDown();
                    //robot.shooterOne.setPower(RobotConstants.Drivetrain.shooterShortOn);
                    waitM(250);
//                    robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseTwo);
                    // Shoot the second ball
                    waitM(500);
                    robot.spindexer.spindexerUp();
                    waitM(500);
                    //robot.shooterOne.setPower(RobotConstants.Drivetrain.shooterSixSevenOn);
                    robot.spindexer.spindexerDown();
                    waitM(250);
//                    robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
                    // Shoot the third ball
                    waitM(500);
                    robot.spindexer.spindexerUp();
                    waitM(500);
                    robot.spindexer.spindexerDown();
                    waitM(250);
                    //robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
                    robot.intake.intakeDown();
                    robot.intake.startIntaking();
                    // Go to pick up the first set
                    follower.followPath(collectOne);
//                    waitM(300);
//                    robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseTwo);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()){
                    robot.intake.intakeUp();
                    waitM(500);
//                    robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseTwo);
                    waitM(100);
                    robot.intake.intakeDown();
                    waitM(500);
                    follower.followPath(collectTwo,true);
//                    waitM(300);
//                    robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
                    setPathState(3);
                    break;
                }
            case 3:
                if (!follower.isBusy()){
                    robot.intake.intakeUp();
                    robot.shooterOne.setPower(RobotConstants.Drivetrain.shooterShortAuto);
                    waitM(500);
//                    robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseThree);
                    waitM(100);
                    robot.intake.intakeDown();
                    waitM(200);
                    follower.followPath(collectThree,true);
                    //robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
                    setPathState(4);
                }
            case 4:
                if (!follower.isBusy()){
                    robot.intake.intakeUp();
                    follower.followPath(scoreOne,true);
                    setPathState(5);
                }
            case 5:
                if (!follower.isBusy()){
                    waitM(500);
                    robot.spindexer.spindexerUp();
                    waitM(500);
                    robot.shooterOne.setPower(RobotConstants.Drivetrain.shooterAuto);
                    robot.spindexer.spindexerDown();
                    //robot.shooterOne.setPower(RobotConstants.Drivetrain.shooterShortOn);
                    waitM(250);
//                    robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseTwo);
                    // Shoot the second ball
                    waitM(500);
                    robot.spindexer.spindexerUp();
                    waitM(500);
                    //robot.shooterOne.setPower(RobotConstants.Drivetrain.shooterSixSevenOn);
                    robot.spindexer.spindexerDown();
                    waitM(250);
//                    robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
                    // Shoot the third ball
                    waitM(500);
                    robot.spindexer.spindexerUp();
                    waitM(500);
                    robot.spindexer.spindexerDown();
                    robot.intake.intakeDown();
                    robot.intake.startIntaking();
                    follower.followPath(collectFour);
                    setPathState(6);
                }
            case 6:
                if (!follower.isBusy()){
                    robot.intake.intakeUp();
                    waitM(500);
//                    robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseTwo);
                    waitM(100);
                    robot.intake.intakeDown();
                    waitM(500);
                    follower.followPath(collectFive,true);
                    setPathState(7);
                    break;
                }
            case 7:
                if (!follower.isBusy()){
                    robot.intake.intakeUp();
                    robot.shooterOne.setPower(RobotConstants.Drivetrain.shooterShortAuto);
                    waitM(500);
//                    robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseThree);
                    waitM(100);
                    robot.intake.intakeDown();
                    waitM(200);
                    follower.followPath(collectSix,true);
                    setPathState(8);
                }
            case 8:
                if (!follower.isBusy()){
                    robot.intake.intakeUp();
                    follower.followPath(scoreTwo,true);
                    setPathState(9);
                }
            case 9:
                if (!follower.isBusy()){
                    waitM(500);
                    robot.spindexer.spindexerUp();
                    waitM(500);
                    robot.shooterOne.setPower(RobotConstants.Drivetrain.shooterAuto);
                    robot.spindexer.spindexerDown();
                    //robot.shooterOne.setPower(RobotConstants.Drivetrain.shooterShortOn);
                    waitM(250);
//                    robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseTwo);
                    // Shoot the second ball
                    waitM(500);
                    robot.spindexer.spindexerUp();
                    waitM(500);
                    //robot.shooterOne.setPower(RobotConstants.Drivetrain.shooterSixSevenOn);
                    robot.spindexer.spindexerDown();
                    waitM(250);
//                    robot.spindexerServo.setPosition(RobotConstants.Spindexer.spindexerServoPoseOne);
                    // Shoot the third ball
                    waitM(500);
                    robot.spindexer.spindexerUp();
                    waitM(500);
                    robot.spindexer.spindexerDown();
                    follower.followPath(collectSeven);
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
        robot.pathTimer.reset();
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

            robot.pathTimer = new ElapsedTime();
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
