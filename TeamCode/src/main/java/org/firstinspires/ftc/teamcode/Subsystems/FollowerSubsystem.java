package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.Constants.FConstants;
import org.firstinspires.ftc.teamcode.Util.Constants.LConstants;
import org.firstinspires.ftc.teamcode.Vision.Math.Vector;

public class FollowerSubsystem extends SubsystemBase {

    private final Follower follower;
    private double forward, strafe, turn;

    public FollowerSubsystem(HardwareMap hardwareMap) {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(0)));
        follower.startTeleopDrive();
    }

    public void imuReset() {
        follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(0)));
    }

    public void setMovement(double f, double s, double t) {
        this.forward = f;
        this.strafe = s;
        this.turn = t;
    }

    @Override
    public void periodic() {
        follower.setTeleOpMovementVectors(forward, strafe, turn, false);
        follower.update();
    }
}