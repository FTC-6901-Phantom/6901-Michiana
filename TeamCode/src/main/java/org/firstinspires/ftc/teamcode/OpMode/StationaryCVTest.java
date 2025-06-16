package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.Util.Constants.FConstants;
import org.firstinspires.ftc.teamcode.Util.Constants.LConstants;
import org.firstinspires.ftc.teamcode.Util.cmd;
import org.firstinspires.ftc.teamcode.Vision.Limelight;
import org.firstinspires.ftc.teamcode.Vision.Math.Vector;

public class StationaryCVTest extends CommandOpMode {
    public Follower follower;
    public Limelight limelight;
    public HardwareSubsystem robot;
    public Limelight.SampleState buffer;

    @Override
    public void initialize() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(Vector.cartesian(0, 0).pose(0));

        this.buffer = new Limelight.SampleState();
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.limelight = new Limelight(hardwareMap, Limelight.Targets.YellowOnly);
        this.robot = new HardwareSubsystem(hardwareMap, telemetry);

        schedule(
                new RunCommand(follower::update),
                new RunCommand(telemetry::update),
                new SequentialCommandGroup(
                        cmd.scanForSample(limelight, buffer, telemetry, follower, robot, false),
                        cmd.sleep(1000),
                        cmd.driveToSample(follower, buffer, telemetry).alongWith(
                                cmd.alignClaw(robot, buffer)
                        ),
                        cmd.sleep(1000),
                        cmd.grabSample(robot)
                )
        );
    }
}
