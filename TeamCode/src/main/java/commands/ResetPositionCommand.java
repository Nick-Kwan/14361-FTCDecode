package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import Constants.EnumConstants;
import Constants.FieldMap;
import Constants.OdometryConstants;

/**
 * Instant command that resets the position to the alliance default start.
 * Writes through Pedro's Follower so coordinates stay consistent.
 * Used for DPad_Down button binding when position has drifted.
 */
public class ResetPositionCommand extends CommandBase {
    private final Follower follower;

    public ResetPositionCommand(Follower follower) {
        this.follower = follower;
    }

    @Override
    public void initialize() {
//        Pose resetPose;
//        if (FieldMap.allianceColor == EnumConstants.AllianceColor.Red){
//            resetPose = OdometryConstants.redStartPoint;
//        }
//        else {
//            resetPose = OdometryConstants.blueStartPoint;
//        }
        Pose resetPose = (FieldMap.allianceColor == EnumConstants.AllianceColor.Red)
                ? OdometryConstants.redStartPoint
                : OdometryConstants.blueStartPoint;
//        OdometryConstants.endingAutonPose = resetPose;
        follower.setPose(resetPose);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
