package org.firstinspires.ftc.teamcode.OpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Subsystems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Util.Constants.FConstants;
import org.firstinspires.ftc.teamcode.Util.Constants.LConstants;
import org.firstinspires.ftc.teamcode.Util.Controls.Buttons;
import org.firstinspires.ftc.teamcode.Util.cmd;
import org.firstinspires.ftc.teamcode.Vision.Math.Vector;

@TeleOp
public class TeleOpMain extends CommandOpMode {

    public FollowerSubsystem followerSubsystem;
    public IntakeSubsystem intakeSubsystem;
    public Buttons buttons;

    @Override
    public void initialize() {
        followerSubsystem = new FollowerSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        buttons = new Buttons(gamepad1);

        CommandScheduler.getInstance().registerSubsystem(followerSubsystem);
        CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);

        buttons.Cycle.whenPressed(cmd.teleopCycle(intakeSubsystem));
        buttons.Wrist.whenPressed(cmd.teleopWrist(intakeSubsystem));

        schedule(new RunCommand(() -> {
            followerSubsystem.setMovement(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        }));
        schedule(new RunCommand(telemetry::update));
    }
}