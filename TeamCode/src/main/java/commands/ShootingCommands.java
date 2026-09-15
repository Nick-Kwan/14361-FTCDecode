package commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import Constants.SpindexerConstants;
import subsystems.Spindexer;

/**
 * Factory class for composing shooting command sequences.
 *
 * With the 4-slot CR servo design, the spindexer can only advance forward
 * one slot at a time (spin until magnet → flip → retract → cooldown).
 * There is no concept of "go to a specific pose" — only sequential advancement.
 *
 * Primary method: shootAllSlots() fires all 4 slots in sequence.
 */
public class ShootingCommands {

    /**
     * Fire all 4 slots sequentially.
     * Each FireCommand triggers one full index cycle:
     *   spin → magnet detect → stop → flip → retract → cooldown.
     *
     * Total timing per slot ≈ spin time (variable) + FIRE_TIME + RETRACT_TIME + DEBOUNCE
     */
    public static Command shootAllSlots(Spindexer spindexer) {
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        for (int i = 0; i < SpindexerConstants.SLOT_COUNT; i++) {
            sequence.addCommands(new FireCommand(spindexer));
        }
        return sequence;
    }

    /**
     * Fire a specific number of slots sequentially.
     * Useful for partial firing (e.g., fire 2 of 4 slots).
     */
    public static Command shootNSlots(Spindexer spindexer, int count) {
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        int slotsToFire = Math.min(count, SpindexerConstants.SLOT_COUNT);
        for (int i = 0; i < slotsToFire; i++) {
            sequence.addCommands(new FireCommand(spindexer));
        }
        return sequence;
    }
}
