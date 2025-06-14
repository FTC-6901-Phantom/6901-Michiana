package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Vision.Limelight;
import org.firstinspires.ftc.teamcode.Vision.Math.Kinematics;
import org.firstinspires.ftc.teamcode.Vision.Math.Vector;

public class DriveToSample extends CommandBase {
    private Follower follower;
    private Limelight.SampleState buffer;
    private final Telemetry telemetry;

    public DriveToSample(Follower follower, Limelight.SampleState buffer, Telemetry telemetry) {
        this.follower = follower;
        this.buffer = buffer;
        this.telemetry = telemetry;
    }

    @Override
    public void initialize() {
        this.follower.setMaxPower(0.8);

        /*
            Calculates the following:
            - Sample position on XZ plane regardless of limelight angle
            -   ^ Center of sample, homography postcalculation dynamically
            - Robot position at the time of seeing a sample, even if it has since moved
            - Robot rotation at the time of seeing a sample
            - The robot movement required to go from its position when it saw the sample to the position of the sample
            - Figure out how much of that can be done without moving the robot, just using slides
            - Calculate required slide, claw, and drivetrain movement
        */
        Kinematics kinematics = new Kinematics(buffer);

        /*
            Calculate the path from the robots current position to that of the sample.
        */
        follower.followPath(kinematics.instantPath(follower), true);
    }

    @Override
    public void execute() {
        follower.update();
    }

    @Override
    public boolean isFinished() {
        return follower.getCurrentTValue() > 0.85;
    }

    @Override
    public void end(boolean i) {
        this.follower.setMaxPower(1);
        buffer.center = Vector.cartesian(0, 0);
        buffer.robotPosition = Vector.cartesian(0, 0);
        buffer.robotRotation = 0;
        buffer.Tilt = 0;
    }
}
