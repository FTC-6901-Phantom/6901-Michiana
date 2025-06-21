package org.firstinspires.ftc.teamcode.OpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.pedropathing.localization.PoseUpdater;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Subsystems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Hang;
import org.firstinspires.ftc.teamcode.Subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Util.Constants.FConstants;
import org.firstinspires.ftc.teamcode.Util.Constants.LConstants;
import org.firstinspires.ftc.teamcode.Util.Controls.Buttons;
import org.firstinspires.ftc.teamcode.Util.cmd;
import org.firstinspires.ftc.teamcode.Vision.Math.Vector;

@TeleOp
public class TeleOpMain extends CommandOpMode {

    public FollowerSubsystem followerSubsystem;
    public IntakeSubsystem intakeSubsystem;
    public HardwareSubsystem robot;
    public Buttons buttons;
    public Hang hang;

    @Override
    public void initialize() {
        followerSubsystem = new FollowerSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        robot = new HardwareSubsystem(hardwareMap, telemetry);
        buttons = new Buttons(gamepad1);

        hang = new Hang(this);

        CommandScheduler.getInstance().registerSubsystem(followerSubsystem);
        CommandScheduler.getInstance().registerSubsystem(robot);
        CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);

        // Button bindings...
        buttons.Cycle.whenPressed(cmd.teleopCycle(intakeSubsystem));
        buttons.Wrist.whenPressed(cmd.teleopWrist(intakeSubsystem));
        buttons.Lift.whenPressed(cmd.teleopReset(intakeSubsystem));
        buttons.Swap.whenPressed(cmd.teleSwap(intakeSubsystem));

        // Movement
        schedule(new RunCommand(() ->
                followerSubsystem.setMovement(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x)
        ));

        // Hang control
        schedule(new RunCommand(() -> hang.teleOp()));

        // Telemetry
        schedule(new RunCommand(() -> {
            telemetry.addData("LeftClimb Pos", hang.LeftClimb.getCurrentPosition());
            telemetry.addData("RightClimb Pos", hang.RightClimb.getCurrentPosition());
            telemetry.update();
        }));
    }
}
