package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;

public class FollowPath extends CommandBase {
    private Follower follower;
    private PathChain pathChain;

    private double speed;
    private boolean holdEnd;

    public FollowPath(Follower follower, PathChain path) {
        this.follower = follower;
        this.pathChain = path;
        this.holdEnd = true;
        this.speed = 1;
    }

    public FollowPath(Follower follower, PathChain path, boolean holdEnd) {
        this.follower = follower;
        this.pathChain = path;
        this.holdEnd = holdEnd;
        this.speed = 1;
    }

    public FollowPath setSpeed(double speed) {
        this.speed = speed;
        this.follower.setMaxPower(speed);
        return this;
    }

    @Override
    public void initialize() {
        this.follower.breakFollowing();
        this.follower.setMaxPower(this.speed);
        this.follower.followPath(this.pathChain, this.holdEnd);
    }

    @Override
    public void execute() { this.follower.update(); }

    @Override
    public boolean isFinished() {
        return this.follower.getCurrentTValue() > 0.95;
    }

    @Override
    public void end(boolean wasInterrupted) {
        if (wasInterrupted) follower.holdPoint(follower.getPose());
        follower.setMaxPower(1);
    }
}