package commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import subsystems.Intake;

/**
 * Stops the intake motor and retracts the intake.
 * Completes immediately.
 */
public class IntakeStopCommand extends InstantCommand {

    public IntakeStopCommand(Intake intake) {
        super(() -> {
            intake.stopIntaking();
            intake.retract();
        }, intake);
    }
}
