package org.firstinspires.ftc.teamcode.OpMode.Auto;

import static org.firstinspires.ftc.teamcode.OpMode.Auto.AutoPath.preload;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Util.Constants.FConstants;
import org.firstinspires.ftc.teamcode.Util.Constants.LConstants;
import org.firstinspires.ftc.teamcode.Util.cmd;

@Autonomous
public class SampAuto extends CommandOpMode {

    public IntakeSubsystem intake;
    public HardwareSubsystem robot;
    public Follower follower;

    private final Pose startPose = new Pose(7.2, 113.1, Math.toRadians(0));  // Starting position

    @Override
    public void initialize() {
        this.intake = new IntakeSubsystem(hardwareMap, telemetry);
        this.robot = new HardwareSubsystem(hardwareMap, telemetry);
        robot.init();

        CommandScheduler.getInstance().registerSubsystem(intake);

        Constants.setConstants(FConstants.class, LConstants.class);
        this.follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        this.follower.setStartingPose(startPose);

        schedule(
                new RunCommand(follower::update),
                new RunCommand(telemetry::update),
                new SequentialCommandGroup(
                        cmd.sleepUntil(this::opModeIsActive),

                        cmd.followPath(follower, preload()).alongWith(cmd.raiseSlides(robot)).andThen(
                                cmd.dropSample(robot))
                )
        );
    }
}
