package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import utility.RobotHardware;
import Constants.SpindexerConstants;
import Constants.EnumConstants;

public class Spindexer extends SubsystemBase {
    private final RobotHardware robot;

    // Current position tracking
    private EnumConstants.SpindexerPosition currentPosition = EnumConstants.SpindexerPosition.PoseOne;

    // FlickState machine for linkage servo (fire cycle: up -> wait -> down)
    private EnumConstants.FlickState flickState = EnumConstants.FlickState.Idle;
    private final ElapsedTime flickTimer = new ElapsedTime();

    public Spindexer() {
        this.robot = RobotHardware.getInstance();
    }

    // ==================== POSITION CONTROL ====================

    public void setPoseOne() {
        robot.spindexerServo.setPosition(SpindexerConstants.POSE_ONE);
        currentPosition = EnumConstants.SpindexerPosition.PoseOne;
    }

    public void setPoseTwo() {
        robot.spindexerServo.setPosition(SpindexerConstants.POSE_TWO);
        currentPosition = EnumConstants.SpindexerPosition.PoseTwo;
    }

    public void setPoseThree() {
        robot.spindexerServo.setPosition(SpindexerConstants.POSE_THREE);
        currentPosition = EnumConstants.SpindexerPosition.PoseThree;
    }

    /**
     * Rotate left (wrapping through the 3 positions).
     * Guarded by touch sensor — only rotates when getTouchSensorState() is false,
     * matching old Spindexer.setPose(moveLeft) behavior.
     */
    public void rotateLeft() {
        if (getTouchSensorState()) return;
        switch (currentPosition) {
            case PoseOne:   setPoseThree(); break;
            case PoseTwo:   setPoseOne();   break;
            case PoseThree: setPoseTwo();   break;
        }
    }

    /**
     * Rotate right (wrapping through the 3 positions).
     * Guarded by touch sensor — same guard as rotateLeft.
     */
    public void rotateRight() {
        if (getTouchSensorState()) return;
        switch (currentPosition) {
            case PoseOne:   setPoseTwo();   break;
            case PoseTwo:   setPoseThree(); break;
            case PoseThree: setPoseOne();   break;
        }
    }

    public EnumConstants.SpindexerPosition getCurrentPosition() {
        return currentPosition;
    }

    // ==================== LINKAGE CONTROL (FIRE) ====================

    /** Raise the linkage (fires a ball into the shooter) */
    public void linkageUp() {
        robot.spindexerLinkageServo.setPosition(SpindexerConstants.LINKAGE_UP);
    }

    /** Lower the linkage (retract after firing) */
    public void linkageDown() {
        robot.spindexerLinkageServo.setPosition(SpindexerConstants.LINKAGE_DOWN);
    }

    /**
     * Start a flick cycle: linkage up -> FIRE_TIME_MS -> linkage down -> RETRACT_TIME_MS -> idle.
     * Only triggers if currently idle.
     */
    public void triggerFlick() {
        if (flickState == EnumConstants.FlickState.Idle) {
            flickState = EnumConstants.FlickState.Start;
        }
    }

    public boolean isFlickIdle() {
        return flickState == EnumConstants.FlickState.Idle;
    }

    public EnumConstants.FlickState getFlickState() {
        return flickState;
    }

    private void flickStateMachinePeriodic() {
        if (flickState == EnumConstants.FlickState.Idle) return;

        switch (flickState) {
            case Start:
                linkageUp();
                flickTimer.reset();
                flickState = EnumConstants.FlickState.Extended;
                break;
            case Extended:
                if (flickTimer.milliseconds() < SpindexerConstants.FIRE_TIME_MS) break;
                linkageDown();
                flickTimer.reset();
                flickState = EnumConstants.FlickState.Retracted;
                break;
            case Retracted:
                if (flickTimer.milliseconds() < SpindexerConstants.RETRACT_TIME_MS) break;
                flickState = EnumConstants.FlickState.Idle;
                break;
        }
    }

    // ==================== SENSORS ====================

    /**
     * Raw touch sensor state.
     * Returns true when the digital channel reads HIGH (sensor not activated).
     * Old code checked: if (!getTouchSensorState()) { allow rotation }
     */
    public boolean getTouchSensorState() {
        return robot.touchSensor.getState();
    }

    /** Magnetic limit switch: true when closed (circuit active) */
    public boolean isLimitSwitchClosed() {
        return !robot.magneticLimitSensor.getState();
    }

    // Color sensor RGB sum accessors (one pair per spindexer position)
    public double detectColorOne_1() {
        return robot.colorSensorOne_1.red() + robot.colorSensorOne_1.green() + robot.colorSensorOne_1.blue();
    }
    public double detectColorOne_2() {
        return robot.colorSensorOne_2.red() + robot.colorSensorOne_2.green() + robot.colorSensorOne_2.blue();
    }
    public double detectColorTwo_1() {
        return robot.colorSensorTwo_1.red() + robot.colorSensorTwo_1.green() + robot.colorSensorTwo_1.blue();
    }
    public double detectColorTwo_2() {
        return robot.colorSensorTwo_2.red() + robot.colorSensorTwo_2.green() + robot.colorSensorTwo_2.blue();
    }
    public double detectColorThree_1() {
        return robot.colorSensorThree_1.red() + robot.colorSensorThree_1.green() + robot.colorSensorThree_1.blue();
    }
    public double detectColorThree_2() {
        return robot.colorSensorThree_2.red() + robot.colorSensorThree_2.green() + robot.colorSensorThree_2.blue();
    }

    /** Ball present at position 1: either sensor exceeds BALL_PRESENT threshold */
    public boolean isBallPresentAtOne() {
        return detectColorOne_1() > SpindexerConstants.BALL_PRESENT
            || detectColorOne_2() > SpindexerConstants.BALL_PRESENT;
    }
    public boolean isBallPresentAtTwo() {
        return detectColorTwo_1() > SpindexerConstants.BALL_PRESENT
            || detectColorTwo_2() > SpindexerConstants.BALL_PRESENT;
    }
    public boolean isBallPresentAtThree() {
        return detectColorThree_1() > SpindexerConstants.BALL_PRESENT
            || detectColorThree_2() > SpindexerConstants.BALL_PRESENT;
    }

    /**
     * Slot empty check at position 2: either sensor below BALL_ABSENT threshold.
     * Uses OR (not AND) — matches old autoIntake behavior exactly.
     */
    public boolean isSlotEmptyAtTwo() {
        return detectColorTwo_1() < SpindexerConstants.BALL_ABSENT
            || detectColorTwo_2() < SpindexerConstants.BALL_ABSENT;
    }

    /** Blue > green comparison for sorting decisions (uses primary sensor _1) */
    public boolean isBlueGreaterThanGreenAtOne() {
        return robot.colorSensorOne_1.blue() > robot.colorSensorOne_1.green();
    }
    public boolean isBlueGreaterThanGreenAtTwo() {
        return robot.colorSensorTwo_1.blue() > robot.colorSensorTwo_1.green();
    }

    // ==================== AUTO-INTAKE DISTRIBUTION ====================

    /**
     * Auto-intake ball distribution logic.
     * Moves spindexer to an empty slot when current slot detects a ball.
     * Preserves exact behavior from old Spindexer.autoIntake():
     * - PoseOne: two independent if checks (second can override first)
     * - PoseTwo/PoseThree: else-if checks (first match wins)
     */
    public void autoIntake() {
        switch (currentPosition) {
            case PoseOne:
                // Two independent checks — matches old code (not else-if)
                if (isBallPresentAtThree() && isSlotEmptyAtTwo()) {
                    setPoseTwo();
                }
                if (isBallPresentAtOne() && isSlotEmptyAtTwo()) {
                    setPoseThree();
                }
                break;
            case PoseTwo:
                if (isBallPresentAtOne() && isSlotEmptyAtTwo()) {
                    setPoseOne();
                } else if (isBallPresentAtThree() && isSlotEmptyAtTwo()) {
                    setPoseThree();
                }
                break;
            case PoseThree:
                if (isBallPresentAtThree() && isSlotEmptyAtTwo()) {
                    setPoseOne();
                } else if (isBallPresentAtOne() && isSlotEmptyAtTwo()) {
                    setPoseTwo();
                }
                break;
        }
    }

    // ==================== PERIODIC ====================

    @Override
    public void periodic() {
        flickStateMachinePeriodic();
    }
}
