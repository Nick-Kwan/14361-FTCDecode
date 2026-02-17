//package subsystems;
//
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.qualcomm.hardware.limelightvision.LLResult;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//
//import utility.RobotHardware;
//
//public class RobotLocalization extends SubsystemBase {
//    private final RobotHardware robot;
//
//    Pose robotPose;
//
//    // Limelight
//    LLResult result;
//    Pose limeLightPose;
//
//    // Pinpoint
//    Pose pinpointPose;
//
//    public RobotLocalization() {
//        this.robot = RobotHardware.getInstance();
//
//        limeLightPose = new Pose(0, 0, 0);
//        robotPose = new Pose(0, 0, 0);
//        pinpointPose = new Pose(0, 0, Math.toRadians(0));
//        setPinpointPose(new Pose(0, 0, Math.toRadians(0)););
//    }
//
//    public void periodic() {
//        setLimelightPose();
//        updatePinpointPose();
//        updateRobotVelocity();
//
//        robotPose = pinpointPose;
//    }
//
//    public void setLimelightPose() {
//        result = robot.limelight.getLatestResult();
//        if (result != null && result.isValid()) {
//            Pose3D botpose = result.getBotpose();
//            if (botpose != null) {
//                limeLightPose = new Pose(-1 * botpose.getPosition().x * 39.370079, -1 * botpose.getPosition().y * 39.370079, Math.toRadians(((360 - (-1 * robot.limelight.getLatestResult().getBotpose().getOrientation().getYaw(AngleUnit.DEGREES))) % 360) - 180));
//            }
//        }
//    }
//
//    public Pose getLimelightPose() {
//        return limeLightPose;
//    }
//
//    public void updatePinpointPose() {
//        robot.pinpointDrive.updatePoseEstimate();
//        pinpointPose = new Pose2d(robot.pinpointDrive.pinpoint.getPosition().getX(DistanceUnit.INCH), robot.pinpointDrive.pinpoint.getPosition().getY(DistanceUnit.INCH), robot.pinpointDrive.pinpoint.getHeading());
//    }
//
//    public void updateRobotVelocity() {
//        robotVelocity = robot.pinpointDrive.pinpoint.getVelocityRR();
//    }
//
//    public PoseVelocity2d getRobotVelocity() {
//        return robotVelocity;
//    }
//
//    public void relocalizePinpointWithLimelight() {
//        robotPose = limeLightPose;
//        robot.pinpointDrive.pinpoint.setPosition(limeLightPose);
//        pinpointPose = limeLightPose;
//        robot.pinpointDrive.updatePoseEstimate();
//    }
//
//    public void setPinpointPose(Pose2d pose) {
//        robot.pinpointDrive.pinpoint.setPosition(pose);
//    }
//
//    public Pose2d getRobotPose() {
//        return robotPose;
//    }
//}