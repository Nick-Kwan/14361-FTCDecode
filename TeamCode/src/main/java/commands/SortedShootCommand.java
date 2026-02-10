package commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;

import Constants.EnumConstants.AllianceColor;
import Constants.EnumConstants.SpindexerPosition;
import subsystems.Limelight;
import subsystems.Spindexer;

/**
 * Sorted shooting command based on AprilTag ID and color sensor readings.
 *
 * Two usage modes:
 * - Constructor (for auto): new SortedShootCommand(spindexer, limelight, alliance)
 *   Reads sensors at EXECUTION time (in initialize()), suitable for command sequences
 *   built during init() but executed later.
 *
 * - Static factory (for teleop): SortedShootCommand.build(spindexer, limelight)
 *   Reads sensors at CALL time, suitable for commands built and scheduled immediately.
 *
 * Decision tree matches old Spindexer.sortingAllianceAuto() for blue
 * and sortingRedAuto() for red (mirrored PoseOne/PoseThree, inverted color checks).
 *
 * Must be called when spindexer is at PoseTwo.
 */
public class SortedShootCommand extends CommandBase {
    private final Spindexer spindexer;
    private final Limelight limelight;
    private final AllianceColor alliance;
    private Command delegate;

    public SortedShootCommand(Spindexer spindexer, Limelight limelight, AllianceColor alliance) {
        this.spindexer = spindexer;
        this.limelight = limelight;
        this.alliance = alliance;
    }

    @Override
    public void initialize() {
        if (alliance == AllianceColor.Blue) {
            delegate = buildBlueFromSensors(spindexer, limelight);
        } else {
            delegate = buildRedFromSensors(spindexer, limelight);
        }
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

    /**
     * Build a blue sorted shooting command. Reads sensors at call time.
     * The spindexer should already be at PoseTwo before calling this.
     */
    public static Command build(Spindexer spindexer, Limelight limelight) {
        return buildBlueFromSensors(spindexer, limelight);
    }

    // ==================== Blue Decision Tree ====================

    private static Command buildBlueFromSensors(Spindexer spindexer, Limelight limelight) {
        int aprilID = limelight.getAprilID();
        boolean twoBlue = spindexer.isBlueGreaterThanGreenAtTwo();
        boolean oneBlue = spindexer.isBlueGreaterThanGreenAtOne();

        // Tag 21: Green, Purple, Purple
        if (aprilID == 21) {
            if (twoBlue && oneBlue) {
                return ShootingCommands.shootAtThreePoses(spindexer,
                    SpindexerPosition.PoseThree, SpindexerPosition.PoseTwo, SpindexerPosition.PoseOne);
            } else if (twoBlue && !oneBlue) {
                return ShootingCommands.shootAtThreePoses(spindexer,
                    SpindexerPosition.PoseOne, SpindexerPosition.PoseTwo, SpindexerPosition.PoseThree);
            } else if (!twoBlue && oneBlue) {
                return ShootingCommands.shootFromCurrent(spindexer,
                    SpindexerPosition.PoseThree, SpindexerPosition.PoseOne);
            } else {
                return ShootingCommands.shootFromCurrent(spindexer,
                    SpindexerPosition.PoseOne, SpindexerPosition.PoseThree);
            }
        }
        // Tag 22: Purple, Green, Purple
        else if (aprilID == 22) {
            if (twoBlue && oneBlue) {
                return ShootingCommands.shootFromCurrent(spindexer,
                    SpindexerPosition.PoseThree, SpindexerPosition.PoseOne);
            } else if (twoBlue && !oneBlue) {
                return ShootingCommands.shootFromCurrent(spindexer,
                    SpindexerPosition.PoseOne, SpindexerPosition.PoseThree);
            } else if (!twoBlue && oneBlue) {
                return ShootingCommands.shootAtThreePoses(spindexer,
                    SpindexerPosition.PoseOne, SpindexerPosition.PoseTwo, SpindexerPosition.PoseThree);
            } else {
                return ShootingCommands.shootFromCurrent(spindexer,
                    SpindexerPosition.PoseOne, SpindexerPosition.PoseThree);
            }
        }
        // Tag 23: Purple, Purple, Green
        else if (aprilID == 23) {
            if (twoBlue && oneBlue) {
                return ShootingCommands.shootFromCurrent(spindexer,
                    SpindexerPosition.PoseOne, SpindexerPosition.PoseThree);
            } else if (twoBlue && !oneBlue) {
                return ShootingCommands.shootFromCurrent(spindexer,
                    SpindexerPosition.PoseThree, SpindexerPosition.PoseOne);
            } else if (!twoBlue && oneBlue) {
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

    // ==================== Red Decision Tree ====================
    // Mirrors blue with inverted color checks and PoseOne<->PoseThree swapped

    private static Command buildRedFromSensors(Spindexer spindexer, Limelight limelight) {
        int aprilID = limelight.getAprilID();
        boolean twoBlue = spindexer.isBlueGreaterThanGreenAtTwo();
        boolean oneBlue = spindexer.isBlueGreaterThanGreenAtOne();
        // For red: "our ball" = NOT blue (red has green > blue)

        // Tag 21: Green, Purple, Purple
        if (aprilID == 21) {
            if (!twoBlue && !oneBlue) {
                return ShootingCommands.shootAtThreePoses(spindexer,
                    SpindexerPosition.PoseOne, SpindexerPosition.PoseTwo, SpindexerPosition.PoseThree);
            } else if (!twoBlue && oneBlue) {
                return ShootingCommands.shootAtThreePoses(spindexer,
                    SpindexerPosition.PoseThree, SpindexerPosition.PoseTwo, SpindexerPosition.PoseOne);
            } else if (twoBlue && !oneBlue) {
                return ShootingCommands.shootFromCurrent(spindexer,
                    SpindexerPosition.PoseOne, SpindexerPosition.PoseThree);
            } else {
                return ShootingCommands.shootFromCurrent(spindexer,
                    SpindexerPosition.PoseThree, SpindexerPosition.PoseOne);
            }
        }
        // Tag 22: Purple, Green, Purple
        else if (aprilID == 22) {
            if (!twoBlue && !oneBlue) {
                return ShootingCommands.shootFromCurrent(spindexer,
                    SpindexerPosition.PoseOne, SpindexerPosition.PoseThree);
            } else if (!twoBlue && oneBlue) {
                return ShootingCommands.shootFromCurrent(spindexer,
                    SpindexerPosition.PoseThree, SpindexerPosition.PoseOne);
            } else if (twoBlue && !oneBlue) {
                return ShootingCommands.shootAtThreePoses(spindexer,
                    SpindexerPosition.PoseThree, SpindexerPosition.PoseTwo, SpindexerPosition.PoseOne);
            } else {
                return ShootingCommands.shootFromCurrent(spindexer,
                    SpindexerPosition.PoseThree, SpindexerPosition.PoseOne);
            }
        }
        // Tag 23: Purple, Purple, Green
        else if (aprilID == 23) {
            if (!twoBlue && !oneBlue) {
                return ShootingCommands.shootFromCurrent(spindexer,
                    SpindexerPosition.PoseThree, SpindexerPosition.PoseOne);
            } else if (!twoBlue && oneBlue) {
                return ShootingCommands.shootFromCurrent(spindexer,
                    SpindexerPosition.PoseOne, SpindexerPosition.PoseThree);
            } else if (twoBlue && !oneBlue) {
                return ShootingCommands.shootAtThreePoses(spindexer,
                    SpindexerPosition.PoseThree, SpindexerPosition.PoseOne, SpindexerPosition.PoseTwo);
            } else {
                return ShootingCommands.shootFromCurrent(spindexer,
                    SpindexerPosition.PoseThree, SpindexerPosition.PoseOne);
            }
        }

        // Fallback: no valid tag, shoot all unsorted
        return ShootingCommands.shootAll(spindexer);
    }
}
