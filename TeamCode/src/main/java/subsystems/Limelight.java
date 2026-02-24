package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

import Constants.LimelightConstants;
import Constants.OdometryConstants;
import utility.RobotHardware;

public class Limelight extends SubsystemBase {
    private final RobotHardware robot;
    private LLResult latestResult;
    private int aprilID = 0;
    private boolean valid = false;
    private double tx = 0;
    private double ty = 0;

    // Pedro follower for coordinate-consistent pose writes
    private Follower follower;

    // Relocalization state
    private Pose limelightPose = null;
    private String lastRelocDebug = "none";

    public Limelight() {
        this.robot = RobotHardware.getInstance();
    }

    /** Set the Pedro follower for coordinate-consistent relocalization */
    public void setFollower(Follower follower) {
        this.follower = follower;
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

    // ==================== Relocalization ====================

    /** Switch to the localization pipeline (MegaTag2 AprilTag pose estimation) */
    public void switchToLocalizationPipeline() {
        switchPipeline(LimelightConstants.PIPELINE_LOCALIZATION);
    }

    /** Switch back to the goal tracking pipeline */
    public void switchToGoalPipeline(int goalPipeline) {
        switchPipeline(goalPipeline);
    }

    /**
     * Poll the Limelight for a MegaTag2 botpose and cache it as a Pedro Pose.
     * Converts from meters to inches. The coordinate conversion from FTC field
     * coordinates to Pedro coordinates must be verified on the actual robot.
     */
    public void updateLimelightPose() {
        LLResult result = robot.limelight.getLatestResult();
        if (result != null && result.isValid()
                && !result.getFiducialResults().isEmpty()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                // Convert from meters to inches
                double xInches = botpose.getPosition().x * LimelightConstants.METERS_TO_INCHES;
                double yInches = botpose.getPosition().y * LimelightConstants.METERS_TO_INCHES;
                double headingRad = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

                // Convert FTC field coordinates to Pedro coordinates
                // FTC: origin at field center, X=right, Y=forward
                // Pedro: origin at corner, axes depend on setup
                // NOTE: This conversion must be verified/tuned on the actual robot.
                // Using negated coordinates similar to the old RobotLocalization approach.
                double pedroX = -xInches;
                double pedroY = -yInches;
                double pedroHeading = ((2 * Math.PI - (-headingRad)) % (2 * Math.PI)) - Math.PI;

                limelightPose = new Pose(pedroX, pedroY, pedroHeading);

                lastRelocDebug = String.format("raw=(%.3fm, %.3fm, %.1f) -> pedro=(%.1f, %.1f, %.1f)",
                        botpose.getPosition().x, botpose.getPosition().y,
                        botpose.getOrientation().getYaw(AngleUnit.DEGREES),
                        limelightPose.getX(), limelightPose.getY(),
                        Math.toDegrees(limelightPose.getHeading()));
            }
        }
    }

    /**
     * Apply the cached Limelight pose to the pinpoint odometry.
     * Returns true if the relocalization was successful.
     */
    public boolean relocalizePinpoint() {
        if (limelightPose == null) {
            lastRelocDebug = "no limelight pose cached";
            return false;
        }

        if (follower != null) {
            // Write through Pedro's coordinate system (single source of truth)
            follower.setStartingPose(limelightPose);
        } else {
            // Fallback to raw pinpoint (no follower configured)
            robot.pinpoint.setPosition(OdometryConstants.toPose2D(limelightPose));
            robot.pinpoint.update();
        }
        lastRelocDebug = String.format("APPLIED (%.1f, %.1f)",
                limelightPose.getX(), limelightPose.getY());
        limelightPose = null;
        return true;
    }

    /** Get the last relocalization debug string (for telemetry) */
    public String getLastRelocDebug() {
        return lastRelocDebug;
    }

    // ==================== LED Control ====================

    public void LEDgreen() {
        robot.LEDlight.setPosition(0.5);
    }
    public void LEDred() {
        robot.LEDlight.setPosition(0.277);
    }
    public void LEDpurple() {
        robot.LEDlight.setPosition(0.722);
    }

    // ==================== Getters ====================

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
