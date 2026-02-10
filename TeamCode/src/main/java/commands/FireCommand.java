package commands;

import com.arcrobotics.ftclib.command.CommandBase;

import Constants.EnumConstants.FlickState;
import subsystems.Spindexer;

/**
 * Triggers the spindexer linkage flick cycle (one fire).
 * Sequence: linkageUp -> FIRE_TIME_MS -> linkageDown -> RETRACT_TIME_MS -> idle.
 * Waits for the state machine to return to Idle before finishing.
 */
public class FireCommand extends CommandBase {
    private final Spindexer spindexer;
    private boolean triggered = false;

    public FireCommand(Spindexer spindexer) {
        this.spindexer = spindexer;
        addRequirements(spindexer);
    }

    @Override
    public void initialize() {
        triggered = false;
        if (spindexer.isFlickIdle()) {
            spindexer.triggerFlick();
            triggered = true;
        }
    }

    @Override
    public void execute() {
        if (!triggered && spindexer.isFlickIdle()) {
            spindexer.triggerFlick();
            triggered = true;
        }
    }

    @Override
    public boolean isFinished() {
        return triggered && spindexer.isFlickIdle();
    }

    @Override
    public void end(boolean interrupted) {
    }
}
