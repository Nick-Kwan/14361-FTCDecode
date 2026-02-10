package teleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Constants.EnumConstants;
import Constants.LimelightConstants;

@TeleOp(name = "TeleOpRed")
public class RedTeleOp extends TeleOpTemplate {

    @Override
    protected int getGoalPipeline() {
        return LimelightConstants.PIPELINE_GOAL_RED;
    }

    @Override
    protected EnumConstants.AllianceColor getAllianceColor() {
        return EnumConstants.AllianceColor.Red;
    }
}
