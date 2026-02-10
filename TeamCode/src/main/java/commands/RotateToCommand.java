package commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import Constants.EnumConstants.SpindexerPosition;
import subsystems.Spindexer;

/**
 * Instantly rotates the spindexer to a specific position (PoseOne/Two/Three).
 * The servo command is sent immediately; caller should add a WaitCommand
 * for ROTATION_SETTLE_MS if needed before firing.
 */
public class RotateToCommand extends InstantCommand {

    public RotateToCommand(Spindexer spindexer, SpindexerPosition targetPosition) {
        super(() -> {
            switch (targetPosition) {
                case PoseOne:   spindexer.setPoseOne();   break;
                case PoseTwo:   spindexer.setPoseTwo();   break;
                case PoseThree: spindexer.setPoseThree(); break;
            }
        }, spindexer);
    }
}
