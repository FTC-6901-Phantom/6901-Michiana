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
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

import org.firstinspires.ftc.teamcode.Subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Util.Constants.FConstants;
import org.firstinspires.ftc.teamcode.Util.Constants.LConstants;
import org.firstinspires.ftc.teamcode.Util.cmd;

@Autonomous(name = "0+4", group = "Daniella Cortez")
public class SixSamp extends CommandOpMode {

    public IntakeSubsystem intakeSubsystem;
    public HardwareSubsystem robot;
    public Follower follower;

    @Override
    public void initialize() {
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        robot = intakeSubsystem.robot;

        CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
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
                        new InstantCommand(() -> intakeSubsystem.cycleState = Subsystem.CycleState.Score),
                        new WaitCommand(1000),
                        new InstantCommand(() -> intakeSubsystem.cycleState = Subsystem.CycleState.Finish),

                        // Grab 2nd sample
                        cmd.followPath(follower, Paths.grab2()),
                        new InstantCommand(() -> intakeSubsystem.cycleState = Subsystem.CycleState.Hover),
                        new WaitCommand(800),
                        new InstantCommand(() -> intakeSubsystem.cycleState = Subsystem.CycleState.Intake),
                        new WaitCommand(200),
                        new InstantCommand(() -> intakeSubsystem.cycleState = Subsystem.CycleState.Grab),
                        new WaitCommand(300),
                        new InstantCommand(() -> intakeSubsystem.cycleState = Subsystem.CycleState.Return),

                        // Score 2nd sample
                        cmd.followPath(follower, Paths.score2()),
                        new InstantCommand(() -> intakeSubsystem.cycleState = Subsystem.CycleState.Score),
                        new WaitCommand(1000),
                        new InstantCommand(() -> intakeSubsystem.cycleState = Subsystem.CycleState.Finish),

                        // Grab 3rd sample
                        cmd.followPath(follower, Paths.grab3()),
                        new InstantCommand(() -> intakeSubsystem.cycleState = Subsystem.CycleState.Hover),
                        new WaitCommand(800),
                        new InstantCommand(() -> intakeSubsystem.cycleState = Subsystem.CycleState.Intake),
                        new WaitCommand(200),
                        new InstantCommand(() -> intakeSubsystem.cycleState = Subsystem.CycleState.Grab),
                        new WaitCommand(300),
                        new InstantCommand(() -> intakeSubsystem.cycleState = Subsystem.CycleState.Return),


                        // Score 3rd sample
                        cmd.followPath(follower, Paths.score3()),
                        new InstantCommand(() -> intakeSubsystem.cycleState = Subsystem.CycleState.Score),
                        new WaitCommand(1000),
                        new InstantCommand(() -> intakeSubsystem.cycleState = Subsystem.CycleState.Finish),
                        
                        // Grab 4th sample
                        cmd.followPath(follower, Paths.grab4()),
                        new InstantCommand(() -> intakeSubsystem.cycleState = Subsystem.CycleState.Hover),
                        new WaitCommand(800),
                        new InstantCommand(() -> intakeSubsystem.cycleState = Subsystem.CycleState.Intake),
                        new WaitCommand(200),
                        new InstantCommand(() -> intakeSubsystem.cycleState = Subsystem.CycleState.Grab),
                        new WaitCommand(300),
                        new InstantCommand(() -> intakeSubsystem.cycleState = Subsystem.CycleState.Return),


                        // Score 4th sample
                        cmd.followPath(follower, Paths.score4()),
                        new InstantCommand(() -> intakeSubsystem.cycleState = Subsystem.CycleState.Score),
                        new WaitCommand(1000),
                        new InstantCommand(() -> intakeSubsystem.cycleState = Subsystem.CycleState.Finish)                )
        );
    }
}


class Paths {
    public static Pose start = new Pose(6.25, 114, Math.toRadians(270));
    public static Pose score = new Pose(14.5, 128, Math.toRadians(-45));
    public static Pose second = new Pose (32, 122, Math.toRadians(0));
    public static Pose third = new Pose(31, 131, Math.toRadians(0));
    public static Pose fourth = new Pose(35, 133, Math.toRadians(28));
    public static Pose drag = new Pose(72-8, 100, Math.toRadians(270));
    public static Pose sub2 = new Pose(63, 94, Math.toRadians(-90));
    public static Pose subControlPoint = new Pose(66.40214477211796, 111.95710455764075);
    public static Pose sub3 = new Pose(67, 92, Math.toRadians(-90));
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

    public static PathChain sub2() {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                score, subControlPoint, sub2
                        )
                )
                .setLinearHeadingInterpolation(score.getHeading(), sub2.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();
    }

    public static PathChain score5() {
        return new PathBuilder()
                .addPath(
                        new BezierLine(
                                sub2, score
                        )
                )
                .setLinearHeadingInterpolation(sub2.getHeading(), score.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();
    }

    public static PathChain sub3() {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                score, subControlPoint, sub3
                        )
                )
                .setLinearHeadingInterpolation(score.getHeading(), sub3.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();
    }

    public static PathChain score6() {
        return new PathBuilder()
                .addPath(
                        new BezierLine(
                                sub3, score
                        )
                )
                .setLinearHeadingInterpolation(sub3.getHeading(), score.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();
    }

    public static PathChain drag() {
        return new PathBuilder()
                .addPath(
                        new BezierLine(
                                score,drag
                        )
                )
                .setLinearHeadingInterpolation(score.getHeading(), drag.getHeading())
                .setZeroPowerAccelerationMultiplier(4)
                .build();
    }

    public static PathChain park(Pose current) {
        return new PathBuilder()
                .addPath(
                        new BezierLine(
                                current, park
                        )
                )
                .setLinearHeadingInterpolation(current.getHeading(), park.getHeading())
                .setZeroPowerAccelerationMultiplier(1)
                .build();
    }
}