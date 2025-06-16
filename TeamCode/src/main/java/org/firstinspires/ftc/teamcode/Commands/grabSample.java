package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class grabSample extends CommandBase {

    private HardwareSubsystem robot;

    public grabSample(HardwareSubsystem robot) {
        this.robot = robot;
        addRequirements(robot);
    }

    @Override
    public void initialize() {
        robot.ClawSetState(Subsystem.ClawState.Closed);
    }
}
