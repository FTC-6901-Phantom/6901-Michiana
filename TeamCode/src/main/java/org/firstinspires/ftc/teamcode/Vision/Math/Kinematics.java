package org.firstinspires.ftc.teamcode.Vision.Math;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.Vision.Limelight;


public class Kinematics {

    @Config
    public static class LimelightInformation {
        public static double limelightHeight = 3.97;
        public static double limelightAngle = 74;
        public static double forwardOffset = 0.5;

        public static double forwardScalarForLateral = 0.01;
        public static double forwardOffsetForLateral = 0.2661;

        public static double constantXOffset = 4.9;

        public static double a = 24.32983;
        public static double b = 0.0310553;
        public static double c = -8.773041;
        public static double e = Math.E;
    }
    public RobotPosition absoluteRobotTarget;
    public double absoluteClawRotation;

    public Kinematics(Limelight.SampleState buffer) {

        double forwards = (LimelightInformation.a * (
                Math.pow(LimelightInformation.e, LimelightInformation.b * buffer.center.y)
        ) + LimelightInformation.c) / 2.54;
        double ty = 8;

        double x = forwards * 2.54;

        double m = -0.0119154 * x - 0.487525;
        double c = -8.06;

        double lateralCM = m * buffer.center.x + c;

        this.absoluteRobotTarget = RobotPosition.relativePosition(
                new RobotPosition(
                        buffer.robotPosition,
                        buffer.robotRotation
                ),
                Vector.cartesian(
                        lateralCM / 2.54, ty
                )
        );
        this.absoluteClawRotation = buffer.angle;
    }

    public PathChain instantPath(Follower follower) {
        Pose pose = follower.getPose();
        double cx = pose.getX();
        double cy = pose.getY();

        PathBuilder builder = new PathBuilder();
        builder.addPath(
                new BezierLine(
                        new Point(cx, cy, Point.CARTESIAN),
                        new Point(
                                this.absoluteRobotTarget.position.x,
                                this.absoluteRobotTarget.position.y,
                                Point.CARTESIAN
                        )
                )
        ).setConstantHeadingInterpolation(
                this.absoluteRobotTarget.rotation
        );

        return builder.build();
    }
}