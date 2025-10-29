package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.command.Subsystem;

public class Intake implements Subsystem{
    private RobotHardware robot;
    public Intake() {
        this.robot = RobotHardware.getInstance();
    }

    public void StartIntaking(){
        robot.intakeMotor.setPower(.5);
    }

    public void StopIntaking(){
        robot.intakeMotor.setPower(0);
    }

    public void SpitOut(){
        robot.intakeMotor.setPower(-.5);
    }
}
