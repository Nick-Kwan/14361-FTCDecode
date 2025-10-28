package org.firstinspires.ftc.teamcode.OpMode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.util.RobotHardware;
@TeleOp(name = "TeleOp")

public class TeleOpStuff extends CommandOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    private GamepadEx driver;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        driver = new GamepadEx(gamepad1);

        robot.init(hardwareMap,driver);
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        if(driver.gamepad.left_trigger > 0.1) {
            robot.mecanum.periodic(0.3);
        } else {
            robot.mecanum.periodic(1);
        }

        driver.readButtons();

    }
}
