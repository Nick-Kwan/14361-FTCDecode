package utility;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Constants.EnumConstants;
import subsystems.Shooter;
import subsystems.Turret;
import subsystems.Spindexer;
import subsystems.Limelight;

import java.util.List;

public class TelemetryHelper {
    private EnumConstants.ShootMode shootMode = EnumConstants.ShootMode.Sorted;

    public void setShootMode(EnumConstants.ShootMode mode) {
        this.shootMode = mode;
    }

    public EnumConstants.ShootMode getShootMode() {
        return shootMode;
    }

    public void update(Telemetry telemetry, Shooter shooter, Turret turret,
                       Spindexer spindexer, Limelight limelight) {
        telemetry.addData("Shoot Mode", shootMode);

        if (limelight != null) {
            telemetry.addData("Limelight Valid", limelight.isValid());
            telemetry.addData("Tx", "%.2f", limelight.getTx());
            telemetry.addData("Ty", "%.2f", limelight.getTy());
            telemetry.addData("April ID", limelight.getAprilID());
        }

        if (turret != null) {
            telemetry.addData("Turret Pos", "%.3f", turret.getPosition());
        }

        if (spindexer != null) {
            telemetry.addData("Spindexer", spindexer.getCurrentPosition());
            telemetry.addData("Touch Sensor", !spindexer.getTouchSensorState());
            telemetry.addData("Mag Limit", spindexer.isLimitSwitchClosed());
        }

        if (shooter != null) {
            List<Double> velocities = shooter.getVelocities();
            if (velocities != null && velocities.size() >= 2) {
                telemetry.addData("Flywheel L", "%.2f", velocities.get(0));
                telemetry.addData("Flywheel R", "%.2f", velocities.get(1));
            }
        }

        telemetry.update();
    }

    /** Simple overload for when subsystems aren't available */
    public void update(Telemetry telemetry) {
        telemetry.addData("Shoot Mode", shootMode);
        telemetry.update();
    }
}
