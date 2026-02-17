package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import Constants.IntakeConstants;
import utility.RobotHardware;
import Constants.LimelightConstants;

import java.util.List;

public class Limelight extends SubsystemBase {
    private final RobotHardware robot;
    private LLResult latestResult;
    private int aprilID = 0;
    private boolean valid = false;
    private double tx = 0;
    private double ty = 0;

    public Limelight() {
        this.robot = RobotHardware.getInstance();
    }

    /** Start the Limelight sensor */
    public void start() {
        robot.limelight.start();
    }

    /** Update the robot orientation for MegaTag2 */
    public void updateOrientation(double yaw) {
        robot.limelight.updateRobotOrientation(yaw);
    }

    /** Switch to a specific pipeline (see LimelightConstants) */
    public void switchPipeline(int pipeline) {
        robot.limelight.pipelineSwitch(pipeline);
    }

    /** Poll the latest result from the Limelight and cache Tx/Ty/AprilTag ID */
    public void pollResult() {
        latestResult = robot.limelight.getLatestResult();
        if (latestResult != null && latestResult.isValid()) {
            valid = true;
            LEDgreen();
            tx = latestResult.getTx();
            ty = latestResult.getTy();
            // Scan for valid AprilTag IDs (21, 22, 23)
            List<LLResultTypes.FiducialResult> fiducials = latestResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducials) {
                int id = fr.getFiducialId();
                if (id == 21 || id == 22 || id == 23) {
                    aprilID = id;
                    break;
                }
            }
        } else {
            LEDred();
            valid = false;
        }
    }
    public void LEDgreen() {
        robot.LEDlight.setPosition(0.5);
    }
    public void LEDred() {
        robot.LEDlight.setPosition(0.277);
    }
    public void LEDpurple() {
        robot.LEDlight.setPosition(0.722);
    }
    public boolean isValid() { return valid; }
    public double getTx() { return tx; }
    public double getTy() { return ty; }
    public int getAprilID() { return aprilID; }
    public LLResult getLatestResult() { return latestResult; }

    /** Reset the cached AprilTag ID */
    public void resetAprilID() {
        aprilID = 0;
    }

    @Override
    public void periodic() {
        pollResult();
    }
}
