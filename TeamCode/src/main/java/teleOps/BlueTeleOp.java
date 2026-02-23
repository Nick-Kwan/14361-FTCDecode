package teleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Constants.EnumConstants;
import Constants.LimelightConstants;

@TeleOp(name = "TeleOpBlue")
public class BlueTeleOp extends TeleOpTemplate {

    @Override
    protected int getGoalPipeline() {
        return LimelightConstants.PIPELINE_GOAL_BLUE;
    }

    @Override
    protected EnumConstants.AllianceColor getAllianceColor() {
        return EnumConstants.AllianceColor.Blue;
    }

    @Override
    protected double getDriveDirectionMultiplier() {
        return -1.0;
    }
}
