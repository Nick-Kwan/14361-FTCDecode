package commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;

import subsystems.Limelight;
import subsystems.Spindexer;

/**
 * @deprecated Sorted shooting was designed for the 3-slot positional servo model
 * with AprilTag-based ball/goal color matching. The 4-slot CR servo design does not
 * support targeted slot selection. This class now falls through to
 * {@link ShootingCommands#shootAllSlots(Spindexer)}.
 *
 * Retained temporarily for API compatibility with existing auto routines.
 * Will be removed once all callers are migrated.
 */
@Deprecated
public class SortedShootCommand extends CommandBase {
    private final Spindexer spindexer;
    private final Limelight limelight;
    private Command delegate;

    public SortedShootCommand(Spindexer spindexer, Limelight limelight) {
        this.spindexer = spindexer;
        this.limelight = limelight;
    }

    @Override
    public void initialize() {
        // Fall through to shooting all slots unsorted
        delegate = ShootingCommands.shootAllSlots(spindexer);
        delegate.initialize();
    }

    @Override
    public void execute() {
        if (delegate != null) delegate.execute();
    }

    @Override
    public boolean isFinished() {
        return delegate != null && delegate.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (delegate != null) delegate.end(interrupted);
    }

    // ==================== Static Factory (kept for API compatibility) ====================

    /**
     * @deprecated Falls through to {@link ShootingCommands#shootAllSlots(Spindexer)}.
     */
    @Deprecated
    public static Command build(Spindexer spindexer, Limelight limelight) {
        return ShootingCommands.shootAllSlots(spindexer);
    }
}
