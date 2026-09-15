package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import utility.RobotHardware;
import Constants.SpindexerConstants;
import Constants.EnumConstants;

/**
 * Spindexer subsystem for a 4-slot continuous-rotation design.
 *
 * Hardware:
 *   - 1 Axon MAX v2 CR Servo (continuous rotation) for spinning the carousel
 *   - 1 stationary Hall Effect / magnetic limit sensor for slot alignment
 *   - 1 flipper (linkage) servo to push balls upward
 *
 * State machine flow (runs in periodic()):
 *   IDLE → [advanceSlot()] → SPINNING → [magnet detected] → ALIGNED
 *     → FLIPPING (linkageUp) → RETRACTING (linkageDown) → COOLDOWN (debounce) → IDLE
 *
 * The full index cycle (spin → align → flip → retract → cooldown) runs automatically
 * once advanceSlot() is called. Each cycle services one slot.
 */
public class Spindexer extends SubsystemBase {
    private final RobotHardware robot;

    // State machine
    private EnumConstants.SpindexerState state = EnumConstants.SpindexerState.IDLE;
    private final ElapsedTime stateTimer = new ElapsedTime();

    // Slot tracking (0-based, wraps at SLOT_COUNT)
    private int currentSlot = 0;
    private int slotsIndexed = 0;

    public Spindexer() {
        this.robot = RobotHardware.getInstance();
    }

    // ==================== PUBLIC API ====================

    /**
     * Start an index cycle: spin the CR servo until the next magnet is detected,
     * then automatically flip the ball and retract.
     * Only triggers if the state machine is currently IDLE.
     */
    public void advanceSlot() {
        if (state == EnumConstants.SpindexerState.IDLE) {
            state = EnumConstants.SpindexerState.SPINNING;
            robot.spindexerCRServo.setPower(SpindexerConstants.SPIN_POWER);
        }
    }

    /**
     * Emergency stop — halt the CR servo and return to IDLE immediately.
     */
    public void stop() {
        robot.spindexerCRServo.setPower(0);
        state = EnumConstants.SpindexerState.IDLE;
    }

    /** @return current state of the indexing state machine */
    public EnumConstants.SpindexerState getState() {
        return state;
    }

    /** @return true if the state machine is idle and ready for the next advance */
    public boolean isIdle() {
        return state == EnumConstants.SpindexerState.IDLE;
    }

    /** @return current slot index (0 to SLOT_COUNT-1) */
    public int getCurrentSlot() {
        return currentSlot;
    }

    /** @return total number of slots indexed since last reset */
    public int getSlotsIndexed() {
        return slotsIndexed;
    }

    /** Reset the slot counter to 0 */
    public void resetSlotCount() {
        currentSlot = 0;
        slotsIndexed = 0;
    }

    // ==================== LINKAGE (FLIPPER) MANUAL CONTROL ====================

    /** Raise the flipper (pushes ball upward into shooter) */
    public void linkageUp() {
        robot.spindexerLinkageServo.setPosition(SpindexerConstants.LINKAGE_UP);
    }

    /** Lower the flipper (retract after pushing) */
    public void linkageDown() {
        robot.spindexerLinkageServo.setPosition(SpindexerConstants.LINKAGE_DOWN);
    }

    // ==================== SENSORS ====================

    /**
     * Magnetic sensor: returns true when a slot's magnet is aligned
     * with the stationary Hall Effect sensor (circuit active / pin LOW).
     */
    public boolean isMagnetDetected() {
        return !robot.magneticLimitSensor.getState();
    }

    // ==================== STATE MACHINE (runs in periodic) ====================

    private void indexStateMachine() {
        switch (state) {
            case IDLE:
                // Nothing to do — waiting for advanceSlot() call
                break;

            case SPINNING:
                // CR servo is running; poll the magnet sensor
                if (isMagnetDetected()) {
                    // Slot aligned — halt CR servo immediately
                    robot.spindexerCRServo.setPower(0);
                    state = EnumConstants.SpindexerState.ALIGNED;
                }
                break;

            case ALIGNED:
                // Magnet detected and CR servo stopped — begin flipper cycle
                linkageUp();
                stateTimer.reset();
                state = EnumConstants.SpindexerState.FLIPPING;
                break;

            case FLIPPING:
                // Wait for flipper to fully extend
                if (stateTimer.milliseconds() >= SpindexerConstants.FIRE_TIME_MS) {
                    linkageDown();
                    stateTimer.reset();
                    state = EnumConstants.SpindexerState.RETRACTING;
                }
                break;

            case RETRACTING:
                // Wait for flipper to fully retract
                if (stateTimer.milliseconds() >= SpindexerConstants.RETRACT_TIME_MS) {
                    // Update slot tracking
                    currentSlot = (currentSlot + 1) % SpindexerConstants.SLOT_COUNT;
                    slotsIndexed++;
                    stateTimer.reset();
                    state = EnumConstants.SpindexerState.COOLDOWN;
                }
                break;

            case COOLDOWN:
                // Debounce delay — prevent re-triggering on the same magnet
                if (stateTimer.milliseconds() >= SpindexerConstants.DEBOUNCE_MS) {
                    state = EnumConstants.SpindexerState.IDLE;
                }
                break;
        }
    }

    // ==================== PERIODIC ====================

    @Override
    public void periodic() {
        indexStateMachine();
    }
}
