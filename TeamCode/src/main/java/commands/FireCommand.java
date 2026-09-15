package commands;

import com.arcrobotics.ftclib.command.CommandBase;

import subsystems.Spindexer;

/**
 * Triggers one full spindexer index cycle:
 *   spin CR servo → detect magnet → stop → flip ball up → retract → cooldown → idle.
 *
 * Calls advanceSlot() and waits for the state machine to return to IDLE.
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
        if (spindexer.isIdle()) {
            spindexer.advanceSlot();
            triggered = true;
        }
    }

    @Override
    public void execute() {
        // If we couldn't trigger in initialize (state machine was busy), retry
        if (!triggered && spindexer.isIdle()) {
            spindexer.advanceSlot();
            triggered = true;
        }
    }

    @Override
    public boolean isFinished() {
        // Done when we've triggered the cycle and it's returned to IDLE
        return triggered && spindexer.isIdle();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            spindexer.stop();
        }
    }
}
