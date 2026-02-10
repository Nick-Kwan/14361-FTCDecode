package commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import Constants.EnumConstants.SpindexerPosition;
import subsystems.Intake;
import subsystems.Limelight;
import subsystems.Shooter;
import subsystems.Spindexer;
import subsystems.Turret;

/**
 * Fluent builder API for constructing autonomous command sequences.
 *
 * Example usage:
 * <pre>
 * autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
 *     .moveTo(pathToShootingPos)
 *     .shootAll()
 *     .parallel(p -> p.moveTo(pathToIntake).intakeStart())
 *     .intakeStop()
 *     .sortedShoot()
 *     .build();
 * </pre>
 */
public class CommandSequenceBuilder {
    protected final Follower follower;
    protected final Intake intake;
    protected final Spindexer spindexer;
    protected final Limelight limelight;
    protected final Shooter shooter;
    protected final Turret turret;

    private final List<Command> commands = new ArrayList<>();

    public CommandSequenceBuilder(Follower follower, Intake intake, Spindexer spindexer,
                                   Limelight limelight, Shooter shooter, Turret turret) {
        this.follower = follower;
        this.intake = intake;
        this.spindexer = spindexer;
        this.limelight = limelight;
        this.shooter = shooter;
        this.turret = turret;
    }

    // ==================== Move-To Methods ====================

    public CommandSequenceBuilder moveTo(Path path) {
        commands.add(new FollowPathCommand(follower, path, true));
        return this;
    }

    public CommandSequenceBuilder moveTo(Path path, boolean holdEnd) {
        commands.add(new FollowPathCommand(follower, path, holdEnd));
        return this;
    }

    public CommandSequenceBuilder moveTo(PathChain pathChain) {
        commands.add(new FollowPathCommand(follower, pathChain, true));
        return this;
    }

    public CommandSequenceBuilder moveTo(PathChain pathChain, boolean holdEnd) {
        commands.add(new FollowPathCommand(follower, pathChain, holdEnd));
        return this;
    }

    public CommandSequenceBuilder moveTo(Path path, double maxPower) {
        commands.add(new FollowPathCommand(follower, path, maxPower, true));
        return this;
    }

    public CommandSequenceBuilder moveTo(PathChain pathChain, double maxPower) {
        commands.add(new FollowPathCommand(follower, pathChain, maxPower, true));
        return this;
    }

    // ==================== Shooting Methods ====================

    /** Shoot all 3 balls based on current spindexer position */
    public CommandSequenceBuilder shootAll() {
        commands.add(ShootingCommands.shootAll(spindexer));
        return this;
    }

    /** Shoot at 3 specific poses in order (rotate first) */
    public CommandSequenceBuilder shootAtThreePoses(SpindexerPosition a, SpindexerPosition b, SpindexerPosition c) {
        commands.add(ShootingCommands.shootAtThreePoses(spindexer, a, b, c));
        return this;
    }

    /** Shoot current pose, then two more poses */
    public CommandSequenceBuilder shootFromCurrent(SpindexerPosition b, SpindexerPosition c) {
        commands.add(ShootingCommands.shootFromCurrent(spindexer, b, c));
        return this;
    }

    /** Sorted shooting based on AprilTag + color sensors (reads sensors at execution time) */
    public CommandSequenceBuilder sortedShoot() {
        commands.add(new SortedShootCommand(spindexer, limelight));
        return this;
    }

    /** No-sorting blue: fixed order 3→2→1 */
    public CommandSequenceBuilder noSortingBlue() {
        commands.add(ShootingCommands.noSortingBlue(spindexer));
        return this;
    }

    /** No-sorting red: fixed order 1→2→3 */
    public CommandSequenceBuilder noSortingRed() {
        commands.add(ShootingCommands.noSortingRed(spindexer));
        return this;
    }

    // ==================== Spindexer Methods ====================

    /** Rotate spindexer to a specific position */
    public CommandSequenceBuilder rotateTo(SpindexerPosition position) {
        commands.add(new RotateToCommand(spindexer, position));
        return this;
    }

    /** Fire a single ball (linkage flick cycle) */
    public CommandSequenceBuilder fire() {
        commands.add(new FireCommand(spindexer));
        return this;
    }

    // ==================== Intake Methods ====================

    public CommandSequenceBuilder intakeStart() {
        commands.add(new IntakeStartCommand(intake));
        return this;
    }

    public CommandSequenceBuilder intakeStop() {
        commands.add(new IntakeStopCommand(intake));
        return this;
    }

    // ==================== Turret Methods ====================

    /** Set turret to a specific servo position */
    public CommandSequenceBuilder setTurretPosition(double position) {
        commands.add(new InstantCommand(() -> turret.setPosition(position)));
        return this;
    }

    // ==================== Shooter Methods ====================

    /** Set shooter velocity directly */
    public CommandSequenceBuilder setShooterVelocity(double velocity) {
        commands.add(new InstantCommand(() -> shooter.setVelocity(velocity)));
        return this;
    }

    /** Set hood angle directly */
    public CommandSequenceBuilder setHoodAngle(double position) {
        commands.add(new InstantCommand(() -> shooter.setHoodAngle(position)));
        return this;
    }

    // ==================== Limelight Methods ====================

    /** Switch limelight pipeline */
    public CommandSequenceBuilder switchPipeline(int pipeline) {
        commands.add(new InstantCommand(() -> limelight.switchPipeline(pipeline)));
        return this;
    }

    // ==================== Timing Methods ====================

    /** Add a delay in seconds */
    public CommandSequenceBuilder delay(double seconds) {
        commands.add(new WaitCommand((long)(seconds * 1000)));
        return this;
    }

    // ==================== Custom ====================

    /** Add a custom command */
    public CommandSequenceBuilder addCommand(Command command) {
        commands.add(command);
        return this;
    }

    /** Add a custom instant action */
    public CommandSequenceBuilder addAction(Runnable action) {
        commands.add(new InstantCommand(action));
        return this;
    }

    // ==================== Parallel Groups ====================

    /**
     * Add commands that run in parallel.
     * All commands added within the consumer run simultaneously.
     *
     * Example: .parallel(p -> p.moveTo(path).intakeStart())
     */
    public CommandSequenceBuilder parallel(Consumer<ParallelBuilder> builder) {
        ParallelBuilder pb = new ParallelBuilder(follower, intake, spindexer, limelight, shooter, turret);
        builder.accept(pb);
        commands.add(pb.build());
        return this;
    }

    // ==================== Build ====================

    /** Build the final sequential command group */
    public Command build() {
        return new SequentialCommandGroup(commands.toArray(new Command[0]));
    }

    // ==================== Parallel Builder ====================

    public static class ParallelBuilder {
        private final Follower follower;
        private final Intake intake;
        private final Spindexer spindexer;
        private final Limelight limelight;
        private final Shooter shooter;
        private final Turret turret;
        private final List<Command> parallelCommands = new ArrayList<>();

        private ParallelBuilder(Follower follower, Intake intake, Spindexer spindexer,
                                Limelight limelight, Shooter shooter, Turret turret) {
            this.follower = follower;
            this.intake = intake;
            this.spindexer = spindexer;
            this.limelight = limelight;
            this.shooter = shooter;
            this.turret = turret;
        }

        public ParallelBuilder moveTo(Path path) {
            parallelCommands.add(new FollowPathCommand(follower, path, true));
            return this;
        }
        public ParallelBuilder moveTo(PathChain pathChain) {
            parallelCommands.add(new FollowPathCommand(follower, pathChain, true));
            return this;
        }
        public ParallelBuilder shootAll() {
            parallelCommands.add(ShootingCommands.shootAll(spindexer));
            return this;
        }
        public ParallelBuilder intakeStart() {
            parallelCommands.add(new IntakeStartCommand(intake));
            return this;
        }
        public ParallelBuilder intakeStop() {
            parallelCommands.add(new IntakeStopCommand(intake));
            return this;
        }
        public ParallelBuilder delay(double seconds) {
            parallelCommands.add(new WaitCommand((long)(seconds * 1000)));
            return this;
        }
        public ParallelBuilder addCommand(Command command) {
            parallelCommands.add(command);
            return this;
        }

        ParallelCommandGroup build() {
            return new ParallelCommandGroup(parallelCommands.toArray(new Command[0]));
        }
    }
}
