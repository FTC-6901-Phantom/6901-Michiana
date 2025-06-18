package org.firstinspires.ftc.teamcode.OpMode.Auto;


import com.arcrobotics.ftclib.command.*;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.AutoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

import org.firstinspires.ftc.teamcode.Subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Util.Constants.FConstants;
import org.firstinspires.ftc.teamcode.Util.Constants.LConstants;
import org.firstinspires.ftc.teamcode.Util.cmd;

@Autonomous(name = "0+6", group = "Daniella Cortez")
public class SixSamp extends CommandOpMode {

    public AutoSubsystem autoSubsystem;
    public HardwareSubsystem robot;
    public Follower follower;

    public double xTolerance = 3, yTolerance = 3;
    public int scoreTime = 300, grabTime = 700;

    @Override
    public void initialize() {
        autoSubsystem = new AutoSubsystem(hardwareMap, telemetry);
        robot = autoSubsystem.robot;

        CommandScheduler.getInstance().registerSubsystem(autoSubsystem);
        CommandScheduler.getInstance().registerSubsystem(robot);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(Paths.start);

        schedule(
                new RunCommand(follower::update),
                new RunCommand(telemetry::update),
                new SequentialCommandGroup(
                        // Score preload
                        cmd.followPath(follower, Paths.score1()),
                        new InstantCommand(() -> autoSubsystem.autoState = Subsystem.AutoState.Score),
                        new WaitUntilCommand(() -> follower.atPose(Paths.score, xTolerance, yTolerance)),
                        new InstantCommand(() -> autoSubsystem.nextAutoCycle()).andThen(new WaitCommand(scoreTime))
                                .andThen(new InstantCommand(() -> autoSubsystem.nextAutoCycle())),

                        // Grab 2nd sample
                        cmd.followPath(follower, Paths.grab2()),
                        new WaitUntilCommand(() -> follower.atPose(Paths.second, xTolerance, yTolerance)),
                        new InstantCommand(() -> autoSubsystem.nextAutoCycle()).andThen(new WaitCommand(grabTime)),

                        // Score 2nd sample
                        cmd.followPath(follower, Paths.score2()),
                        new WaitUntilCommand(() -> follower.atPose(Paths.score, xTolerance, yTolerance)),
                        new InstantCommand(() -> autoSubsystem.nextAutoCycle()).andThen(new WaitCommand(scoreTime))
                                .andThen(new InstantCommand(() -> autoSubsystem.nextAutoCycle())),

                        // Grab 3rd sample
                        cmd.followPath(follower, Paths.grab3()),
                        new WaitUntilCommand(() -> follower.atPose(Paths.third, xTolerance, yTolerance)),
                        new InstantCommand(() -> autoSubsystem.nextAutoCycle()).andThen(new WaitCommand(grabTime)),


                        // Score 3rd sample
                        cmd.followPath(follower, Paths.score3()),
                        new WaitUntilCommand(() -> follower.atPose(Paths.score, xTolerance, yTolerance)),
                        new InstantCommand(() -> autoSubsystem.nextAutoCycle()).andThen(new WaitCommand(scoreTime))
                                .andThen(new InstantCommand(() -> autoSubsystem.nextAutoCycle())),
                        // Grab 4th sample
                        cmd.followPath(follower, Paths.grab4()),
                        new WaitUntilCommand(() -> follower.atPose(Paths.fourth, xTolerance, yTolerance)),
                        new InstantCommand(() -> autoSubsystem.nextAutoCycle()).andThen(new WaitCommand(grabTime)),


                        // Score 4th sample
                        cmd.followPath(follower, Paths.score4()),
                        new WaitUntilCommand(() -> follower.atPose(Paths.score, xTolerance, yTolerance)),
                        new InstantCommand(() -> autoSubsystem.nextAutoCycle()),

                        //Sub Cycle 1
                        cmd.followPath(follower, Paths.sub1()),
                        new InstantCommand(() -> autoSubsystem.nextAutoCycle()).andThen(new WaitCommand(grabTime)),

                        //Score 5th
                        cmd.followPath(follower, Paths.score5()),
                        new WaitUntilCommand(() -> follower.atPose(Paths.score, xTolerance, yTolerance)),
                        new InstantCommand(() -> autoSubsystem.nextAutoCycle()),

                        //Sub Cycle 2
                        cmd.followPath(follower, Paths.sub2()),
                        new InstantCommand(() -> autoSubsystem.nextAutoCycle()).andThen(new WaitCommand(grabTime)),

                        //Score 6th
                        cmd.followPath(follower, Paths.score6()),
                        new WaitUntilCommand(() -> follower.atPose(Paths.score, xTolerance, yTolerance)),
                        new InstantCommand(() -> autoSubsystem.nextAutoCycle())

                        //Sub Cycle 3
//                        cmd.followPath(follower, Paths.sub3()),
//                        new InstantCommand(() -> autoSubsystem.nextAutoCycle()).andThen(new WaitCommand(grabTime)),
//
//                        //Score 6th
//                        cmd.followPath(follower, Paths.score7()),
//                        new WaitUntilCommand(() -> follower.atPose(Paths.score, xTolerance, yTolerance)),
//                        new InstantCommand(() -> autoSubsystem.nextAutoCycle())
                )
        );
    }
}


class Paths {
    public static Pose start = new Pose(6.25, 114, Math.toRadians(270));
    public static Pose score = new Pose(14.5, 128, Math.toRadians(-45));
    public static Pose second = new Pose (32, 122, Math.toRadians(0));
    public static Pose third = new Pose(31, 131, Math.toRadians(0));
    public static Pose fourth = new Pose(35, 133, Math.toRadians(28));
    public static Pose sub = new Pose(63, 94, Math.toRadians(-90));
    public static Pose park = new Pose(57.25, 94.5, Math.toRadians(270));
    public static PathChain score1() {
        return new PathBuilder()
                .addPath(
                        new BezierLine(
                                start, score
                        )
                )
                .setLinearHeadingInterpolation(start.getHeading(), score.getHeading())
                .build();
    }

    public static PathChain grab2() {
        return new PathBuilder()
                .addPath(
                        new BezierLine(
                                score, second
                        )
                )
                .setLinearHeadingInterpolation(score.getHeading(), second.getHeading())
                .build();
    }

    public static PathChain score2() {
        return new PathBuilder()
                .addPath(
                        new BezierLine(
                                second, score
                        )
                )
                .setLinearHeadingInterpolation(second.getHeading(), score.getHeading())
                .build();
    }

    public static PathChain grab3() {
        return new PathBuilder()
                .addPath(
                        new BezierLine(
                                score, third
                        )
                )
                .setLinearHeadingInterpolation(score.getHeading(), third.getHeading())
                .build();
    }

    public static PathChain score3() {
        return new PathBuilder()
                .addPath(
                        new BezierLine(
                                third, score
                        )
                )
                .setLinearHeadingInterpolation(third.getHeading(), score.getHeading())
                .build();
    }

    public static PathChain grab4() {
        return new PathBuilder()
                .addPath(
                        new BezierLine(
                                score, fourth
                        )
                )
                .setLinearHeadingInterpolation(score.getHeading(), fourth.getHeading())
                .build();
    }

    public static PathChain score4() {
        return new PathBuilder()
                .addPath(
                        new BezierLine(
                                fourth, score
                        )
                )
                .setLinearHeadingInterpolation(fourth.getHeading(), score.getHeading())
                .build();
    }

    public static PathChain sub1() {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                score, sub
                        )
                )
                .setLinearHeadingInterpolation(score.getHeading(), sub.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();
    }

    public static PathChain score5() {
        return new PathBuilder()
                .addPath(
                        new BezierLine(
                                sub, score
                        )
                )
                .setLinearHeadingInterpolation(sub.getHeading(), score.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();
    }

    public static PathChain sub2() {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                score, sub
                        )
                )
                .setLinearHeadingInterpolation(score.getHeading(), sub.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();
    }

    public static PathChain score6() {
        return new PathBuilder()
                .addPath(
                        new BezierLine(
                                sub, score
                        )
                )
                .setLinearHeadingInterpolation(sub.getHeading(), score.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();
    }

    public static PathChain sub3() {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                score, sub
                        )
                )
                .setLinearHeadingInterpolation(score.getHeading(), sub.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();
    }

    public static PathChain score7() {
        return new PathBuilder()
                .addPath(
                        new BezierLine(
                                sub, score
                        )
                )
                .setLinearHeadingInterpolation(sub.getHeading(), score.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();
    }


    public static PathChain park(Pose current) {
        return new PathBuilder()
                .addPath(
                        new BezierLine(
                                score, park
                        )
                )
                .setLinearHeadingInterpolation(current.getHeading(), park.getHeading())
                .setZeroPowerAccelerationMultiplier(1)
                .build();
    }
}