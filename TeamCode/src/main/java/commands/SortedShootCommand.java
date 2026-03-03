package commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;

import Constants.EnumConstants.SpindexerPosition;
import subsystems.Limelight;
import subsystems.Spindexer;

/**
 * Sorted shooting command based on AprilTag ID and color sensor readings.
 *
 * Reads the AprilTag pattern (21/22/23) and ball colors (green vs purple)
 * to determine the correct firing order that matches balls to goal slots.
 *
 * Color detection: blue > green = purple ball, green > blue = green ball.
 *
 * Two usage modes:
 * - Constructor (for auto): new SortedShootCommand(spindexer, limelight)
 *   Reads sensors at EXECUTION time (in initialize()), suitable for command sequences
 *   built during init() but executed later.
 *
 * - Static factory (for teleop): SortedShootCommand.build(spindexer, limelight)
 *   Reads sensors at CALL time, suitable for commands built and scheduled immediately.
 *
 * Must be called when spindexer is at PoseTwo.
 */
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
        delegate = buildFromSensors(spindexer, limelight);
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

    // ==================== Static Factory (reads sensors NOW — for TeleOp) ====================

    public static Command build(Spindexer spindexer, Limelight limelight) {
        return buildFromSensors(spindexer, limelight);
    }

    // ==================== Decision Tree ====================

    /**
     * Determines firing order based on AprilTag pattern and ball colors.
     *
     * AprilTag patterns (which slot is green):
     *   Tag 21: Green, Purple, Purple
     *   Tag 22: Purple, Green, Purple
     *   Tag 23: Purple, Purple, Green
     *
     * Color sensor: blue > green = purple ball, green > blue = green ball.
     * "twoBlue" = ball at PoseTwo is purple, "oneBlue" = ball at PoseOne is purple.
     */
    private static Command buildFromSensors(Spindexer spindexer, Limelight limelight) {
        int aprilID = limelight.aprilID;
        boolean twoIsPurple = spindexer.isBlueGreaterThanGreenAtTwo();
        boolean oneIsPurple = spindexer.isBlueGreaterThanGreenAtOne();

        // Tag 21: Green, Purple, Purple
        if (aprilID == 21) {
            if (twoIsPurple && oneIsPurple) {
                return ShootingCommands.shootAtThreePoses(spindexer,
                    SpindexerPosition.PoseThree, SpindexerPosition.PoseTwo, SpindexerPosition.PoseOne);
            } else if (twoIsPurple && !oneIsPurple) {
                return ShootingCommands.shootAtThreePoses(spindexer,
                    SpindexerPosition.PoseOne, SpindexerPosition.PoseTwo, SpindexerPosition.PoseThree);
            } else if (!twoIsPurple && oneIsPurple) {
                return ShootingCommands.shootFromCurrent(spindexer,
                    SpindexerPosition.PoseThree, SpindexerPosition.PoseOne);
            } else {
                return ShootingCommands.shootFromCurrent(spindexer,
                    SpindexerPosition.PoseOne, SpindexerPosition.PoseThree);
            }
        }
        // Tag 22: Purple, Green, Purple
        else if (aprilID == 22) {
            if (twoIsPurple && oneIsPurple) {
                return ShootingCommands.shootFromCurrent(spindexer,
                    SpindexerPosition.PoseThree, SpindexerPosition.PoseOne);
            } else if (twoIsPurple && !oneIsPurple) {
                return ShootingCommands.shootFromCurrent(spindexer,
                    SpindexerPosition.PoseOne, SpindexerPosition.PoseThree);
            } else if (!twoIsPurple && oneIsPurple) {
                return ShootingCommands.shootAtThreePoses(spindexer,
                    SpindexerPosition.PoseOne, SpindexerPosition.PoseTwo, SpindexerPosition.PoseThree);
            } else {
                return ShootingCommands.shootFromCurrent(spindexer,
                    SpindexerPosition.PoseOne, SpindexerPosition.PoseThree);
            }
        }
        // Tag 23: Purple, Purple, Green
        else if (aprilID == 23) {
            if (twoIsPurple && oneIsPurple) {
                return ShootingCommands.shootFromCurrent(spindexer,
                    SpindexerPosition.PoseOne, SpindexerPosition.PoseThree);
            } else if (twoIsPurple && !oneIsPurple) {
                return ShootingCommands.shootFromCurrent(spindexer,
                    SpindexerPosition.PoseThree, SpindexerPosition.PoseOne);
            } else if (!twoIsPurple && oneIsPurple) {
                return ShootingCommands.shootAtThreePoses(spindexer,
                    SpindexerPosition.PoseOne, SpindexerPosition.PoseThree, SpindexerPosition.PoseTwo);
            } else {
                return ShootingCommands.shootFromCurrent(spindexer,
                    SpindexerPosition.PoseOne, SpindexerPosition.PoseThree);
            }
        }

        // Fallback: no valid tag, shoot all unsorted
        return ShootingCommands.shootAll(spindexer);
    }
}
