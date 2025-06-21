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

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Subsystems.AutoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

import org.firstinspires.ftc.teamcode.Subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.Util.Constants.FConstants;
import org.firstinspires.ftc.teamcode.Util.Constants.LConstants;
import org.firstinspires.ftc.teamcode.Util.OpModeCommand;
import org.firstinspires.ftc.teamcode.Util.cmd;

@Autonomous(name = "0+6~")
public class SixSamp extends OpModeCommand {

    public AutoSubsystem autoSubsystem;
    public HardwareSubsystem robot;
    public Follower follower;

    public double xTolerance = 20, yTolerance = 20;
    public int scoreTime = 600, grabTime = 500, slideTime = 600, time = 1200;

    // -------- Manual “vision” inputs --------
    public static final double MAX_MANUAL_DISTANCE_IN = 48;
    public static double manualDistance1 = 0;
    public static double manualDistance2 = 0;
    public boolean dpadLeftPrev  = false, dpadRightPrev = false;
    public boolean dpadUpPrev    = false, dpadDownPrev  = false;

    public static final double DIST_STEP  = 1;   // 1 in per press

    public double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public boolean aPrev = false, bPrev = false;
    public int currentManual = 0;

    public static double manualHoriz1 = 0;
    public static double manualHoriz2 = 0;
    public static final double MAX_MANUAL_HORIZ_IN = 48;
    public static final double HORIZ_STEP = 1.0;

    @Override
    public void init_loop() {
        super.init_loop();
        // —— select deposit on A/B rising edge ——
        if (gamepad1.a && !aPrev) currentManual = (currentManual == 1 ? 0 : 1);
        aPrev = gamepad1.a;
        if (gamepad1.b && !bPrev) currentManual = (currentManual == 2 ? 0 : 2);
        bPrev = gamepad1.b;

        // —— adjust selected deposit’s horiz & forward dist with D-pad ——
        if (currentManual == 1) {
            if (gamepad1.dpad_left  && !dpadLeftPrev)  manualHoriz1    = clamp(manualHoriz1 - HORIZ_STEP, -MAX_MANUAL_HORIZ_IN, MAX_MANUAL_HORIZ_IN);
            if (gamepad1.dpad_right && !dpadRightPrev) manualHoriz1    = clamp(manualHoriz1 + HORIZ_STEP, -MAX_MANUAL_HORIZ_IN, MAX_MANUAL_HORIZ_IN);
            if (gamepad1.dpad_up    && !dpadUpPrev)    manualDistance1 = clamp(manualDistance1 + DIST_STEP, -MAX_MANUAL_DISTANCE_IN, MAX_MANUAL_DISTANCE_IN);
            if (gamepad1.dpad_down  && !dpadDownPrev)  manualDistance1 = clamp(manualDistance1 - DIST_STEP, -MAX_MANUAL_DISTANCE_IN, MAX_MANUAL_DISTANCE_IN);

        } else if (currentManual == 2) {
            if (gamepad1.dpad_left  && !dpadLeftPrev)  manualHoriz2    = clamp(manualHoriz2 - HORIZ_STEP, -MAX_MANUAL_HORIZ_IN, MAX_MANUAL_HORIZ_IN);
            if (gamepad1.dpad_right && !dpadRightPrev) manualHoriz2    = clamp(manualHoriz2 + HORIZ_STEP, -MAX_MANUAL_HORIZ_IN, MAX_MANUAL_HORIZ_IN);
            if (gamepad1.dpad_up    && !dpadUpPrev)    manualDistance2 = clamp(manualDistance2 + DIST_STEP, -MAX_MANUAL_DISTANCE_IN, MAX_MANUAL_DISTANCE_IN);
            if (gamepad1.dpad_down  && !dpadDownPrev)  manualDistance2 = clamp(manualDistance2 - DIST_STEP, -MAX_MANUAL_DISTANCE_IN, MAX_MANUAL_DISTANCE_IN);
        }

        dpadLeftPrev  = gamepad1.dpad_left;
        dpadRightPrev = gamepad1.dpad_right;
        dpadUpPrev    = gamepad1.dpad_up;
        dpadDownPrev  = gamepad1.dpad_down;

        telemetry.addLine("Press A/B to pick deposit 1/2");
        telemetry.addLine("DPAD \u25C0\u25B6 = horizontal, \u25B2\u25BC = forward dist");
        telemetry.addData("Sel",   currentManual == 0 ? "none" : "dep " + currentManual);
        telemetry.addData("1 h,d", String.format("(%.1f, %.1f)", manualHoriz1, manualDistance1));
        telemetry.addData("2 h,d", String.format("(%.1f, %.1f)", manualHoriz2, manualDistance2));
        telemetry.update();

        // Update poses
        Paths.sub1 = new Pose(
                Paths.control.getX() + manualHoriz1,
                Paths.control.getY() + manualDistance1,
                Math.toRadians(-90)
        );
        Paths.sub2 = new Pose(
                Paths.control.getX() + manualHoriz2,
                Paths.control.getY() + manualDistance2,
                Math.toRadians(-90)
        );
    }

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
                new RunCommand(telemetry::update),
                new RunCommand(follower::update),
                new SequentialCommandGroup(
                        new InstantCommand(() -> autoSubsystem.autoState = Subsystem.AutoState.Score),
                        // Score preload
                        cmd.followPath(follower, Paths.score1()).setSpeed(0.8),
                        new WaitUntilCommand(() -> follower.atPose(Paths.score, xTolerance, yTolerance)),
                        new InstantCommand(() -> autoSubsystem.nextAutoCycle()).andThen(new WaitCommand(scoreTime))
                                .andThen(new InstantCommand(() -> autoSubsystem.nextAutoCycle()))
                                .andThen(new WaitCommand(slideTime)).andThen(new InstantCommand(() -> autoSubsystem.nextAutoCycle())),

                        // Grab 2nd sample
                        cmd.followPath(follower, Paths.grab2()).setSpeed(0.8),
                        new WaitUntilCommand(() -> follower.atPose(Paths.second, xTolerance, yTolerance)).andThen(new WaitCommand(200)),
                        new InstantCommand(() -> autoSubsystem.nextAutoCycle()).andThen(new WaitCommand(grabTime))
                                .andThen(new InstantCommand(() -> autoSubsystem.nextAutoCycle())),

                        // Score 2nd sample
                        cmd.followPath(follower, Paths.score2()).setSpeed(0.8),
                        new WaitUntilCommand(() -> follower.atPose(Paths.score, xTolerance, yTolerance)),
                        new InstantCommand(() -> autoSubsystem.nextAutoCycle()).andThen(new WaitCommand(scoreTime))
                                .andThen(new InstantCommand(() -> autoSubsystem.nextAutoCycle()))
                                .andThen(new WaitCommand(slideTime)).andThen(new InstantCommand(() -> autoSubsystem.nextAutoCycle())),

                        // Grab 3rd sample
                        cmd.followPath(follower, Paths.grab3()).setSpeed(0.8),
                        new WaitUntilCommand(() -> follower.atPose(Paths.third, xTolerance, yTolerance)).andThen(new WaitCommand(200)),
                        new InstantCommand(() -> autoSubsystem.nextAutoCycle()).andThen(new WaitCommand(grabTime))
                                .andThen(new InstantCommand(() -> autoSubsystem.nextAutoCycle())),


                        // Score 3rd sample
                        cmd.followPath(follower, Paths.score3()).setSpeed(0.8),
                        new WaitUntilCommand(() -> follower.atPose(Paths.score, xTolerance, yTolerance)),
                        new InstantCommand(() -> autoSubsystem.nextAutoCycle()).andThen(new WaitCommand(scoreTime))
                                .andThen(new InstantCommand(() -> autoSubsystem.nextAutoCycle()))
                                .andThen(new WaitCommand(slideTime)).andThen(new InstantCommand(() -> autoSubsystem.nextAutoCycle())),

                        // Grab 4th sample
                        cmd.followPath(follower, Paths.grab4()).setSpeed(0.8),
                        new WaitUntilCommand(() -> follower.atPose(Paths.fourth, xTolerance, yTolerance)).andThen(new WaitCommand(200)),
                        new InstantCommand(() -> autoSubsystem.nextAutoCycle()).andThen(new WaitCommand(grabTime))
                                .andThen(new InstantCommand(() -> autoSubsystem.nextAutoCycle())),

                        // Score 4th sample
                        cmd.followPath(follower, Paths.score4()).setSpeed(0.8),
                        new WaitUntilCommand(() -> follower.atPose(Paths.score, xTolerance, yTolerance)),
                        new InstantCommand(() -> autoSubsystem.nextAutoCycle()).andThen(new WaitCommand(scoreTime))
                                .andThen(new InstantCommand(() -> autoSubsystem.nextAutoCycle()))
                                .andThen(new WaitCommand(slideTime)).andThen(new InstantCommand(() -> autoSubsystem.nextAutoCycle())),

                        //Sub Cycle 1
                        cmd.followPath(follower, Paths.sub1()),
                        new WaitUntilCommand(() -> follower.atPose(Paths.sub1, xTolerance, yTolerance)),
                        new InstantCommand(() -> autoSubsystem.nextAutoCycle()).andThen(new WaitCommand(grabTime)).andThen(new InstantCommand(() -> autoSubsystem.autoState = Subsystem.AutoState.armHover)),

                        //Score 5th
                        cmd.followPath(follower, Paths.score5()).alongWith(new WaitCommand(time).andThen(new InstantCommand(() -> autoSubsystem.autoState = Subsystem.AutoState.Score))),
                        new WaitUntilCommand(() -> follower.atPose(Paths.score, xTolerance, yTolerance)),
                        new InstantCommand(() -> autoSubsystem.nextAutoCycle()).andThen(new WaitCommand(scoreTime))
                                .andThen(new InstantCommand(() -> autoSubsystem.nextAutoCycle()))
                                .andThen(new WaitCommand(slideTime)).andThen(new InstantCommand(() -> autoSubsystem.nextAutoCycle())),

                        //Sub Cycle 2
                        cmd.followPath(follower, Paths.sub2()),
                        new WaitUntilCommand(() -> follower.atPose(Paths.sub2, xTolerance, yTolerance)),
                        new InstantCommand(() -> autoSubsystem.nextAutoCycle()).andThen(new WaitCommand(grabTime)).andThen(new InstantCommand(() -> autoSubsystem.autoState = Subsystem.AutoState.armHover)),

                        //Score 6th
                        cmd.followPath(follower, Paths.score6()).alongWith(new WaitCommand(time).andThen(new InstantCommand(() -> autoSubsystem.autoState = Subsystem.AutoState.Score))),
                        new WaitUntilCommand(() -> follower.atPose(Paths.score, xTolerance, yTolerance)),
                        new InstantCommand(() -> autoSubsystem.nextAutoCycle()).andThen(new WaitCommand(scoreTime))
                                .andThen(new InstantCommand(() -> autoSubsystem.nextAutoCycle()))

                        //cmd.followPath(follower, Paths.park())

                        //Sub Cycle 3
//                        cmd.followPath(follower, Paths.sub3()),
//                        new InstantCommand(() -> autoSubsystem.nextAutoCycle()).andThen(new WaitCommand(grabTime)),
//
//                        //Score 7th
//                        cmd.followPath(follower, Paths.score7()),
//                        new WaitUntilCommand(() -> follower.atPose(Paths.score, xTolerance, yTolerance)),
//                        new InstantCommand(() -> autoSubsystem.nextAutoCycle())
                )
        );
    }
}

class Paths {
    public static Pose start = new Pose(6.25, 114, Math.toRadians(270));
    public static Pose score = new Pose(15, 134, Math.toRadians(-45));
    public static Pose chainScore = new Pose(14, 130, Math.toRadians(-45));
    public static Pose second = new Pose (30.5, 121, Math.toRadians(0));
    public static Pose third = new Pose(30.5, 130, Math.toRadians(0));
    public static Pose fourth = new Pose(33 , 134, Math.toRadians(22));
    public static Pose sub1 = new Pose(62, 94, Math.toRadians(-90));
    public static Pose sub2 = new Pose(62, 94, Math.toRadians(-90));
    public static Pose control = new Pose(61, 94, Math.toRadians(-90));
    public static Pose controlSub = new Pose(54, 122, Math.toRadians(270));
    public static Pose park = new Pose(57.25, 94.5, Math.toRadians(270));
    public static PathChain score1() {
        return new PathBuilder()
                .addPath(
                        new BezierLine(
                                start, chainScore
                        )
                )
                .setLinearHeadingInterpolation(start.getHeading(), chainScore.getHeading())
                .build();
    }

    public static PathChain grab2() {
        return new PathBuilder()
                .addPath(
                        new BezierLine(
                                chainScore, second
                        )
                )
                .setLinearHeadingInterpolation(chainScore.getHeading(), second.getHeading())
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
                                score, controlSub, sub1
                        )
                )
                .setLinearHeadingInterpolation(score.getHeading(), sub1.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();
    }

    public static PathChain score5() {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                sub1, controlSub, chainScore
                        )
                )
                .setLinearHeadingInterpolation(sub1.getHeading(), chainScore.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();
    }

    public static PathChain sub2() {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                chainScore, controlSub, sub2
                        )
                )
                .setLinearHeadingInterpolation(chainScore.getHeading(), sub2.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();
    }

    public static PathChain score6() {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                sub2, controlSub, chainScore
                        )
                )
                .setLinearHeadingInterpolation(sub2.getHeading(), chainScore.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();
    }

    public static PathChain sub3() {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                score, sub2
                        )
                )
                .setLinearHeadingInterpolation(score.getHeading(), sub2.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();
    }

    public static PathChain score7() {
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


    public static PathChain park() {
        return new PathBuilder()
                .addPath(
                        new BezierLine(
                                score, park
                        )
                )
                .setLinearHeadingInterpolation(score.getHeading(), park.getHeading())
                .setZeroPowerAccelerationMultiplier(1)
                .build();
    }
}


