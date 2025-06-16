package org.firstinspires.ftc.teamcode.OpMode.Auto;

import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class AutoPath {

    public static Point point(double x, double y) {
        return new Point(y, x, Point.CARTESIAN);
    }

    public static PathChain simpleLine(Point a, Point b, double h) {
        Path path = new Path(
                new BezierLine(a, b)
        );
        path.setConstantHeadingInterpolation(Math.toRadians(h));
        PathBuilder pathBuilder = new PathBuilder();
        pathBuilder.addPath(path);
        return pathBuilder.build();
    }

    public static PathChain simpleCurve(Point a, Point c, Point b) {
        Path path = new Path(
                new BezierCurve(a, c, b)
        );
        path.setTangentHeadingInterpolation();
        PathBuilder pathBuilder = new PathBuilder();
        pathBuilder.addPath(path);
        return pathBuilder.build();
    }

    public static PathChain simpleReverseCurve(Point a, Point c, Point c2, Point b) {
        Path path = new Path(
                new BezierCurve(a, c, c2, b)
        );
        path.setTangentHeadingInterpolation();
        path.setReversed(true);
        PathBuilder pathBuilder = new PathBuilder();
        pathBuilder.addPath(path);
        return pathBuilder.build();
    }

    public static Point start = point(7.3, 113.1);

    public static Point Basket = point(12, 131);
    public static Point FirstSample = point(30, 121);
    public static Point SecondSample = point(30, 132);
    public static Point ThirdSample = point(32, 136);

    public static PathChain preload() {
        return simpleLine(start, Basket, -45);
    }

    public static PathChain firstSamp() {
        return simpleLine(Basket, FirstSample, 0);
    }

    public static PathChain firstSampScore() {
        return simpleLine(FirstSample, Basket, -45);
    }

    public static PathChain secondSamp() {
        return simpleLine(Basket, SecondSample, 0);
    }

    public static PathChain secondSampScore() {
        return simpleLine(SecondSample, Basket, -45);
    }

    public static PathChain thirdSamp() {
        return simpleLine(Basket, ThirdSample, 25);
    }

    public static PathChain thirdSampScore() {
        return simpleLine(ThirdSample, Basket, -45);
    }
}
