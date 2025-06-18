package org.firstinspires.ftc.teamcode.Util;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.AlignClaw;
import org.firstinspires.ftc.teamcode.Commands.DriveToSample;
import org.firstinspires.ftc.teamcode.Commands.FollowPath;
import org.firstinspires.ftc.teamcode.Commands.ScanForSample;
import org.firstinspires.ftc.teamcode.Commands.dropSample;
import org.firstinspires.ftc.teamcode.Commands.grabSample;
import org.firstinspires.ftc.teamcode.Commands.raiseSlides;
import org.firstinspires.ftc.teamcode.Subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Vision.Limelight;

import java.util.function.BooleanSupplier;

public class cmd {

    // Extensions of builtin commands
    public static Command sleep(long ms) { return new WaitCommand(ms); }
    public static Command sleepUntil(BooleanSupplier condition) { return new WaitUntilCommand(condition); }

    //Vision
    public static ScanForSample scanForSample(Limelight limelight, Limelight.SampleState buffer, Telemetry telemetry, Follower follower, HardwareSubsystem robot, boolean isSub) {
        return new ScanForSample(limelight, buffer, telemetry, follower, robot, isSub);
    }
    public static DriveToSample driveToSample(Follower follower, Limelight.SampleState buffer, Telemetry telemetry) {
        return new DriveToSample(follower, buffer, telemetry);
    }
    public static AlignClaw alignClaw(HardwareSubsystem robot, Limelight.SampleState buffer) {
        return new AlignClaw(robot, buffer);
    }

    //Tele Stuff
    public static Command grabSample(HardwareSubsystem robot) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> robot.ClawSetState(Subsystem.ClawState.Open)),
                new InstantCommand(() -> robot.ArmSetState(Subsystem.ArmState.Hover)),
                new InstantCommand(() -> robot.WristSetState(Subsystem.WristState.Horizontal)),
                new InstantCommand(() -> robot.PitchSetState(Subsystem.PitchState.Intake)),
                new WaitCommand(300),
                new InstantCommand(() -> robot.ArmSetState(Subsystem.ArmState.Intake)),
                new InstantCommand(() -> robot.ClawSetState(Subsystem.ClawState.Closed)),
                new WaitCommand(200),
                new InstantCommand(() -> robot.ArmSetState(Subsystem.ArmState.Reset)) // Assuming Reset == Neutral
        );
    }


    public static Command dropSample(HardwareSubsystem robot) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> robot.SlideSetState(Subsystem.SlideState.Score)),
                new WaitCommand(1000),
                new InstantCommand(() -> robot.ArmSetState(Subsystem.ArmState.Score)),
                new InstantCommand(() -> robot.PitchSetState(Subsystem.PitchState.Score)), // or Spec if that's what you meant
                new WaitCommand(300),
                new InstantCommand(() -> robot.ClawSetState(Subsystem.ClawState.Open)),
                new WaitCommand(200),
                new InstantCommand(() -> robot.ArmSetState(Subsystem.ArmState.Reset)), // or Neutral
                new InstantCommand(() -> robot.SlideSetState(Subsystem.SlideState.Retracted))
        );
    }
    public static InstantCommand raiseSlides(HardwareSubsystem robot) {
        return new InstantCommand(() -> robot.SlideSetState(Subsystem.SlideState.Score));

    }
    public static InstantCommand teleopCycle(IntakeSubsystem intakeSubsystem) {
        return new InstantCommand(intakeSubsystem::nextCycle);
    }
    public static InstantCommand teleopWrist(IntakeSubsystem intakeSubsystem) {
        return new InstantCommand(intakeSubsystem::nextWrist);
    }
    public static InstantCommand teleopReset(IntakeSubsystem intakeSubsystem) {
        return new InstantCommand(intakeSubsystem::Reset);
    }
    public static InstantCommand teleSwap(IntakeSubsystem intakeSubsystem) {
        return new InstantCommand(intakeSubsystem::swapTele);
    }

    //Auto
    public static FollowPath followPath(Follower follower, PathChain pathChain) {
        return new FollowPath(follower, pathChain);
    }
//    public static grabSample grabSample(HardwareSubsystem robot) {
//        return new grabSample(robot);
//    }
//    public static raiseSlides raiseSlides(HardwareSubsystem robot) {
//        return new raiseSlides(robot);
//    }
//    public static dropSample dropSample(HardwareSubsystem robot) {
//        return new dropSample(robot);
//    }

}
