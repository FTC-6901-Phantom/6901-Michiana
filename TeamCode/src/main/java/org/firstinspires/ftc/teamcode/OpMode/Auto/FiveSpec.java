/*
package org.firstinspires.ftc.teamcode.OpMode.Auto;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.AutoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Util.Constants.FConstants;
import org.firstinspires.ftc.teamcode.Util.Constants.LConstants;
import org.firstinspires.ftc.teamcode.Util.cmd;

@Autonomous(name = "5+0", group = "Daniella Cortez")
public class FiveSpec extends CommandOpMode {

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
                        new InstantCommand(() -> autoSubsystem.nextAutoCycle())
                )
        );
    }
}


class Pathings {
    public static PathChain scorePreload() {
        return new PathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(7.402, 65.000, Point.CARTESIAN),
                                new Point(42.400, 65.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

    }

    public static PathChain pushOne() {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(42.400, 65.000, Point.CARTESIAN),
                                new Point(8.972, 36.336, Point.CARTESIAN),
                                new Point(54.280, 30.953, Point.CARTESIAN),
                                new Point(70.879, 36.112, Point.CARTESIAN),
                                new Point(60.000, 23.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Point(60.000, 23.000, Point.CARTESIAN),
                                new Point(15.700, 23.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public static PathChain pushTwo() {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(15.700, 23.000, Point.CARTESIAN),
                                new Point(85.009, 24.897, Point.CARTESIAN),
                                new Point(60.000, 12.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Point(60.000, 12.000, Point.CARTESIAN),
                                new Point(15.700, 12.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public static PathChain pushThree() {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(15.700, 12.000, Point.CARTESIAN),
                                new Point(85.458, 15.477, Point.CARTESIAN),
                                new Point(60.000, 7.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Point(60.000, 7.500, Point.CARTESIAN),
                                new Point(7.200, 7.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public static PathChain scoreCycleOne() {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(7.200, 7.500, Point.CARTESIAN),
                                new Point(31.850, 22.654, Point.CARTESIAN),
                                new Point(9.196, 73.794, Point.CARTESIAN),
                                new Point(42.400, 67.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public static PathChain pickCycleTwo() {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(42.400, 67.500, Point.CARTESIAN),
                                new Point(4.935, 69.084, Point.CARTESIAN),
                                new Point(36.785, 28.935, Point.CARTESIAN),
                                new Point(7.200, 33.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public static PathChain scoreCycleTwo() {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(7.200, 33.000, Point.CARTESIAN),
                                new Point(21.533, 28.486, Point.CARTESIAN),
                                new Point(10.542, 65.944, Point.CARTESIAN),
                                new Point(42.400, 70.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public static PathChain pickCycleThree() {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(42.400, 67.500, Point.CARTESIAN),
                                new Point(4.935, 69.084, Point.CARTESIAN),
                                new Point(36.785, 28.935, Point.CARTESIAN),
                                new Point(7.200, 33.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public static PathChain scoreCycleThree() {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(7.200, 33.000, Point.CARTESIAN),
                                new Point(21.533, 28.486, Point.CARTESIAN),
                                new Point(10.542, 65.944, Point.CARTESIAN),
                                new Point(42.400, 72.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public static PathChain pickCycleFour() {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(42.400, 67.500, Point.CARTESIAN),
                                new Point(4.935, 69.084, Point.CARTESIAN),
                                new Point(36.785, 28.935, Point.CARTESIAN),
                                new Point(7.200, 33.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public static PathChain scoreCycleFour() {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(7.200, 33.000, Point.CARTESIAN),
                                new Point(21.533, 28.486, Point.CARTESIAN),
                                new Point(10.542, 65.944, Point.CARTESIAN),
                                new Point(42.400, 75.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public static PathChain pickCycleFive() {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(42.400, 67.500, Point.CARTESIAN),
                                new Point(4.935, 69.084, Point.CARTESIAN),
                                new Point(36.785, 28.935, Point.CARTESIAN),
                                new Point(7.200, 33.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public static PathChain scoreCycleFive() {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(7.200, 33.000, Point.CARTESIAN),
                                new Point(21.533, 28.486, Point.CARTESIAN),
                                new Point(10.542, 65.944, Point.CARTESIAN),
                                new Point(42.400, 77.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }
}*/
