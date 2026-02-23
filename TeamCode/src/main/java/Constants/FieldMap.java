package Constants;

import com.pedropathing.geometry.Pose;

/**
 * Goal position lookup by alliance color.
 * Stores current alliance color and provides the corresponding goal position.
 */
public final class FieldMap {
    private FieldMap() {}

    // Current alliance color — set by TeleOp/Auto during init
    public static EnumConstants.AllianceColor allianceColor = EnumConstants.AllianceColor.Blue;

    /** Get the goal position for the current alliance */
    public static Pose getGoalPosition() {
        if (allianceColor == EnumConstants.AllianceColor.Red) {
            return new Pose(OdometryConstants.RED_GOAL_X, OdometryConstants.RED_GOAL_Y, 0);
        }
        return new Pose(OdometryConstants.BLUE_GOAL_X, OdometryConstants.BLUE_GOAL_Y, 0);
    }
}
