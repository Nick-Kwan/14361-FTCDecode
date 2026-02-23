package Constants;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public final class OdometryConstants {

    // IMU orientation
    public static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    public static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;

    // Starting positions (no auto — Pedro coordinates)
    public static Pose redStartPoint = new Pose(7.25, 9, Math.toRadians(0));
    public static Pose blueStartPoint = new Pose(136.75, 9, Math.toRadians(180));
    public static Pose standardStartPoint = blueStartPoint;

    // Pinpoint yaw scalar — tune on robot
    public static double yawScalar = 1.0;

    // Goal positions (Pedro coordinates)
    public static double BLUE_GOAL_X = 3.0;
    public static double BLUE_GOAL_Y = 140.0;
    public static double RED_GOAL_X = 140.0;
    public static double RED_GOAL_Y = 140.0;

    // Auto → TeleOp handoff: set by AutonTemplate.stop(), read by TeleOpTemplate
    public static Pose endingAutonPose = null;

    /** Convert a Pedro Pose to a Pose2D for pinpoint.setPosition() */
    public static Pose2D toPose2D(Pose pose) {
        return new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(),
                AngleUnit.RADIANS, pose.getHeading());
    }
}
