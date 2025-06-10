package org.firstinspires.ftc.teamcode.OpMode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.Util.Constants.FConstants;
import org.firstinspires.ftc.teamcode.Util.Constants.LConstants;
import org.firstinspires.ftc.teamcode.Vision.Limelight;
import org.firstinspires.ftc.teamcode.Vision.Math.Vector;

public class StationaryCVTest extends OpMode {
    public Follower follower;
    public Limelight limelight;
    public HardwareSubsystem robot;

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(Vector.cartesian(0, 0).pose(0));
        this.limelight = new Limelight(hardwareMap, Limelight.Targets.YellowOnly);
        robot = new HardwareSubsystem(this);
    }

    @Override
    public void loop() {
        follower.update();
    }
}
