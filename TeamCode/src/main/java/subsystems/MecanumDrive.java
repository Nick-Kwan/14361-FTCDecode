package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import utility.RobotHardware;
import Constants.DriveConstants;

public class MecanumDrive extends SubsystemBase {
    private final RobotHardware robot;

    public MecanumDrive() {
        this.robot = RobotHardware.getInstance();
    }

    /**
     * Field-relative mecanum drive.
     * DRIVE_COMP (1.1) is applied to strafe and rotation to counteract imperfect strafing.
     *
     * @param ly forward/back input (-1 to 1)
     * @param lx strafe input (-1 to 1)
     * @param rx rotation input (-1 to 1)
     * @param speedMultiplier overall speed factor (1.0 = full, SLOW_MODE_FACTOR = slow)
     */
    public void drive(double ly, double lx, double rx, double speedMultiplier) {
        double x = lx * DriveConstants.DRIVE_COMP;
        double rotation = rx * DriveConstants.DRIVE_COMP;

        double heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-heading) - ly * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + ly * Math.cos(-heading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotation), 1);
        double frontLeftPower = (rotY + rotX + rotation) / denominator;
        double backLeftPower = (rotY - rotX + rotation) / denominator;
        double frontRightPower = (rotY - rotX - rotation) / denominator;
        double backRightPower = (rotY + rotX - rotation) / denominator;

        robot.leftFront.setPower(frontLeftPower * speedMultiplier);
        robot.leftRear.setPower(backLeftPower * speedMultiplier);
        robot.rightFront.setPower(frontRightPower * speedMultiplier);
        robot.rightRear.setPower(backRightPower * speedMultiplier);
    }

    public void resetYaw() {
        robot.imu.resetYaw();
    }

    public void stop() {
        robot.leftFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightRear.setPower(0);
    }

    @Override
    public void periodic() {
    }
}
