package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class raiseSlides extends CommandBase {

    private HardwareSubsystem robot;

    public raiseSlides(HardwareSubsystem robot) {
        this.robot = robot;
        addRequirements(robot);
    }

    @Override
    public void initialize() {
        robot.ArmSetState(Subsystem.ArmState.Score);
        robot.SlideSetState(Subsystem.SlideState.Score);
    }
}
