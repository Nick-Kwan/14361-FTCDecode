package commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import subsystems.Intake;

/**
 * Starts the intake motor and deploys the intake.
 * Completes immediately — intake continues running until stopped.
 */
public class IntakeStartCommand extends InstantCommand {

    public IntakeStartCommand(Intake intake) {
        super(() -> {
            intake.deploy();
            intake.startIntaking();
        }, intake);
    }
}
