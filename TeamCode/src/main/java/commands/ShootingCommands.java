package commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import Constants.EnumConstants.SpindexerPosition;
import Constants.SpindexerConstants;
import subsystems.Spindexer;

/**
 * Factory class for composing shooting command sequences.
 *
 * Two firing patterns (matching old Spindexer behavior):
 * - shootAtThreePoses: rotate to poseA first, then fire each pose in order
 * - shootFromCurrent: fire the current pose first, then rotate+fire remaining
 *
 * Timing per shot-cycle matches old timer-based code:
 * - FireCommand: linkageUp(0ms) -> FIRE_TIME(200ms) -> linkageDown -> RETRACT_TIME(150ms) = 350ms
 * - ROTATION_SETTLE: 250ms after RotateTo before next fire
 */
public class ShootingCommands {

    /**
     * Rotate to poseA first, then fire poseA -> poseB -> poseC.
     * Used when the ball in the current slot should NOT be fired first.
     *
     * Timing: rotate(0) + settle(250) + fire(350) + rotate(0) + settle(250) + fire(350) +
     *         rotate(0) + settle(250) + fire(350) = ~1550ms total
     */
    public static Command shootAtThreePoses(Spindexer spindexer,
                                             SpindexerPosition poseA,
                                             SpindexerPosition poseB,
                                             SpindexerPosition poseC) {
        if (poseA == SpindexerPosition.PoseOne){
            return new SequentialCommandGroup(
                    new RotateToCommand(spindexer, poseA),
                    new WaitCommand(SpindexerConstants.ROTATION_SETTLE_MS),
                    new FireCommand(spindexer),
                    new RotateToCommand(spindexer, poseB),
                    new WaitCommand(SpindexerConstants.ROTATION_SETTLE_MS),
                    new FireCommand(spindexer),
                    new RotateToCommand(spindexer, poseC),
                    new WaitCommand(SpindexerConstants.ROTATION_SETTLE_MS),
                    new FireCommand(spindexer)
            );
        }
        return new SequentialCommandGroup(
            new RotateToCommand(spindexer, poseA),
            new WaitCommand(SpindexerConstants.ROTATION_SETTLE_MS),
            new FireCommand(spindexer),
            new RotateToCommand(spindexer, poseB),
            new WaitCommand(SpindexerConstants.ROTATION_SETTLE_MS),
            new FireCommand(spindexer),
            new RotateToCommand(spindexer, poseC),
            new WaitCommand(SpindexerConstants.ROTATION_SETTLE_MS),
            new FireCommand(spindexer)
        );
    }

    /**
     * Fire the current pose immediately, then rotate to poseB -> poseC.
     * Used when the ball in the current slot SHOULD be fired first.
     *
     * Timing: fire(350) + rotate(0) + settle(250) + fire(350) + rotate(0) + settle(250) + fire(350) = ~1550ms
     */
    public static Command shootFromCurrent(Spindexer spindexer,
                                            SpindexerPosition poseB,
                                            SpindexerPosition poseC) {
        return new SequentialCommandGroup(
            new FireCommand(spindexer),
            new RotateToCommand(spindexer, poseB),
            new WaitCommand(SpindexerConstants.ROTATION_SETTLE_MS),
            new FireCommand(spindexer),
            new RotateToCommand(spindexer, poseC),
            new WaitCommand(SpindexerConstants.ROTATION_LONG_SETTLE_MS),
            new FireCommand(spindexer)
        );
    }

    /**
     * Shoot all 3 balls based on the current spindexer position.
     * Fires current position first, then cycles through the other two.
     * Matches old teleOpShootPoseXxx() behavior.
     */
    public static Command shootAll(Spindexer spindexer) {
        SpindexerPosition current = spindexer.getCurrentPosition();
        switch (current) {
            case PoseOne:
                // Old teleOpShootPoseOne: fire 1, move→2, fire 2, move→3, fire 3
                return shootFromCurrent(spindexer,
                    SpindexerPosition.PoseTwo, SpindexerPosition.PoseThree);
            case PoseTwo:
                // Old teleOpShootPoseTwo: fire 2, move→1, fire 1, move→3, fire 3
                return shootFromCurrent(spindexer,
                    SpindexerPosition.PoseOne, SpindexerPosition.PoseThree);
            case PoseThree:
                // Old teleOpShootPoseThree: fire 3, move→2, fire 2, move→1, fire 1
                return shootFromCurrent(spindexer,
                    SpindexerPosition.PoseTwo, SpindexerPosition.PoseOne);
            default:
                return new SequentialCommandGroup();
        }
    }

    /**
     * No-sorting blue auto: fixed order 3→2→1 (matches old noSorting()).
     */
    public static Command noSortingBlue(Spindexer spindexer) {
        return shootAtThreePoses(spindexer,
            SpindexerPosition.PoseThree, SpindexerPosition.PoseTwo, SpindexerPosition.PoseOne);
    }

    /**
     * No-sorting red auto: fixed order 1→2→3 (matches old noSortingRed()).
     */
    public static Command noSortingRed(Spindexer spindexer) {
        return shootAtThreePoses(spindexer,
            SpindexerPosition.PoseOne, SpindexerPosition.PoseTwo, SpindexerPosition.PoseThree);
    }
}
