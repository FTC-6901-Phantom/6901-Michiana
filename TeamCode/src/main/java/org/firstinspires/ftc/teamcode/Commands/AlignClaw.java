package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Subsystems.HardwareSubsystem.nuetralPose;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.Vision.Limelight;

public class AlignClaw extends CommandBase {
    private HardwareSubsystem intakeSubsystem;
    private Limelight.SampleState buffer;

    public AlignClaw(HardwareSubsystem intakeSubsystem, Limelight.SampleState buffer) {
        this.intakeSubsystem = intakeSubsystem;
        this.buffer = buffer;
    }

    @Override
    public void initialize() {
        if (this.buffer == null) return;
        this.intakeSubsystem.setTilt(nuetralPose - buffer.angle / 180);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean i) {
        this.buffer.angle = 0;
    }
}