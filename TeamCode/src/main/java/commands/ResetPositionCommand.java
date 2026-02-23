package commands;

import com.arcrobotics.ftclib.command.CommandBase;

import Constants.EnumConstants;
import Constants.FieldMap;
import Constants.OdometryConstants;
import com.pedropathing.geometry.Pose;

import utility.RobotHardware;

/**
 * Instant command that resets the pinpoint position to the alliance default start.
 * Used for DPad_Down button binding when position has drifted.
 */
public class ResetPositionCommand extends CommandBase {
    private final RobotHardware robot;

    public ResetPositionCommand() {
        this.robot = RobotHardware.getInstance();
    }

    @Override
    public void initialize() {
        Pose resetPose = (FieldMap.allianceColor == EnumConstants.AllianceColor.Red)
                ? OdometryConstants.redStartPoint
                : OdometryConstants.blueStartPoint;
        robot.pinpoint.setPosition(OdometryConstants.toPose2D(resetPose));
        robot.pinpoint.update();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
