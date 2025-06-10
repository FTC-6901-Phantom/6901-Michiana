package org.firstinspires.ftc.teamcode.Vision;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.Vision.Math.Vector;


public class Limelight {
    private Limelight3A hardware;

    private double position = 0;

    public enum Targets {
        YellowOnly,
        RedAndYellow,
        BlueAndYellow
    }


    public Limelight (HardwareMap hardwareMap, Targets targets) {
        this.hardware = hardwareMap.get(Limelight3A.class, "limelight");
        switch (targets) {
            case YellowOnly:
                this.hardware.pipelineSwitch(1);
                break;
            case RedAndYellow:
                this.hardware.pipelineSwitch(2);
                break;
            case BlueAndYellow:
                this.hardware.pipelineSwitch(3);
                break;
        }

        this.hardware.updatePythonInputs(0,0,0,0,0,0,0,0);
    }

    public void enable() {
        hardware.start();
    }
    public void logStatus(Telemetry telemetry) {
        telemetry.addData("LL3A STATUS", hardware.getStatus().toString());
    }
    public void disable() { hardware.stop(); }
    public void shutdown() { hardware.shutdown(); }
    public void load(int id) {
        this.hardware.pipelineSwitch(id);
    }

    public static class SamplePair {
        public SampleState optimal;
        public SampleState cached;

        public SamplePair(SampleState optimal, SampleState cached) {
            this.optimal = optimal;
            this.cached = cached;
        }
    }

    public static class SampleState {
        public double angle;
        public Vector center;

        public Vector robotPosition;
        public double robotRotation;

        public double armPosition;
        public double Tilt;

        public SampleState(double angle, Vector center, Vector robotPosition, double robotRotation, double armPosition, double Tilt) {
            this.angle = angle;
            this.center = center;
            this.robotPosition = robotPosition;
            this.robotRotation = robotRotation;
            this.armPosition = armPosition;
            this.Tilt = Tilt;
        }

        public SampleState() {
            this.angle = 0;
            this.center = Vector.cartesian(0, 0);
            this.robotPosition = Vector.cartesian(0, 0);
            this.robotRotation = 0;
            this.armPosition = 0;
        }

        public void reset() {
            this.angle = 0;
            this.center = Vector.cartesian(0, 0);
            this.robotPosition = Vector.cartesian(0, 0);
            this.robotRotation = 0;
            this.armPosition = 0;
        }
    }

    public SampleState query(Telemetry telemetry, Follower follower, HardwareSubsystem arm) {
        LLResult result = hardware.getLatestResult();

        telemetry.addData("SOMERESULT", result == null);
        if (result == null) return null;

        double[] result_array = result.getPythonOutput();
        telemetry.addData("SOMEPYTHON", result_array == null);

        if (result_array == null) return null;
        if (result_array.length == 0) return null;
        double angle = result_array[0];

        // This is POSSIBLE, but so unlikely it can be treated as an error
        if (angle == 0) return null;

        Vector center = Vector.cartesian(result_array[1], result_array[2]);
        Pose current = follower.getPose();
        return new SampleState(angle, center, Vector.cartesian(current.getX(),
                current.getY()), current.getHeading(), arm.getArm(),
                arm.getTilt());
    }

    public SamplePair query_two(Telemetry telemetry, Follower follower, HardwareSubsystem arm) {
        LLResult result = hardware.getLatestResult();

        telemetry.addData("SOMERESULT", result == null);
        if (result == null) return null;

        double[] result_array = result.getPythonOutput();
        telemetry.addData("SOMEPYTHON", result_array == null);

        if (result_array == null) return null;
        if (result_array.length == 0) return null;
        double angle = result_array[0];
        double angle2 = result_array[3];

        // This is POSSIBLE, but so unlikely it can be treated as an error
        if (angle == 0) return null;

        Vector center = Vector.cartesian(result_array[1], result_array[2]);
        Vector center2 = Vector.cartesian(result_array[4], result_array[5]);
        Pose current = follower.getPose();

        SampleState optimal = new SampleState(angle, center, Vector.cartesian(current.getX(),
                current.getY()), current.getHeading(), arm.getArm(),
                arm.getTilt());

        if (angle2 == 0) return new SamplePair(optimal, null);

        SampleState cached = new SampleState(angle2, center2, Vector.cartesian(current.getX(),
                current.getY()), current.getHeading(), arm.getArm(),
                arm.getTilt());

        return new SamplePair(optimal, cached);
    }
}
