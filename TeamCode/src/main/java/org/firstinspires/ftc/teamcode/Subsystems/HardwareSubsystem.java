package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.Controls;

@Config
public class HardwareSubsystem extends SubsystemBase {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Servo LeftLift, RightLift;
    public Servo Pitch, Wrist, Claw;
    public Subsystem.ClimbState climbState;
    public Subsystem.SlideState slideState;
    public Subsystem.ClawState clawState;
    public Subsystem.PitchState pitchState;
    public Subsystem.ArmState armState;
    public Subsystem.WristState wristState;

    public DcMotor LeftSlide, RightSlide, LeftHang, RightHang;

    //Arm Pose
    public static double intakePose = 0.15, autoPose = 0.15, scorePose = 0.95, hoverPose = 0.25, nuetral = 0.7, specPose = 0.4, AutoScore = 0.9;

    //Claw Pose
    public static double openPose = 0.4, grabPose = 0.65;

    //Wrist Pose
    public static double nuetralPose = 0.3, horizontalPose = 0.65;

    //Pitch Pose
    public static double pitch = 0.8, Nan = 0.2, spec = 1;

    //Slide Pose
    public static int slideMin = -5;
    public static int slideMax = 2000;

    public static int High = 1550;
    public static int Spec = 400;
    public static int Reset = 0;
    public int slideCurrent = 0;

    //Hang Pose
    public static int hangMin = -5;
    public static int hangMax = 1450;

    public static int lineUp = 200;
    public static int climb = 0;
    public int hangCurrent = 0;

    public boolean isIntake() {
        return LeftLift.getPosition() == autoPose;
    }
    public boolean isGrabbed() {
        return Claw.getPosition() == grabPose;
    }

    public HardwareSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        //Servos
        LeftLift = (Servo) hardwareMap.get("LeftLift");
        RightLift = (Servo) hardwareMap.get("RightLift");
        Pitch = (Servo) hardwareMap.get("Pitch");
        Wrist = (Servo) hardwareMap.get("Wrist");
        Claw = (Servo) hardwareMap.get("Claw");

        RightLift.setDirection(Servo.Direction.REVERSE);

        //Motors
        LeftSlide = (DcMotor) hardwareMap.get("LeftSlide");
        RightSlide = (DcMotor) hardwareMap.get("RightSlide");
        LeftHang = (DcMotor) hardwareMap.get("LeftHang");
        RightHang = (DcMotor) hardwareMap.get("RightHang");

        //Slides
        LeftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        RightSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        LeftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Hang
        LeftHang.setDirection(DcMotorSimple.Direction.FORWARD);
        RightHang.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //Claw
    public void ClawSetState(Subsystem.ClawState clawState) {
        this.clawState = clawState;

        double position = 0;

        switch (clawState) {
            case Open:
                position = openPose;
                break;
            case Closed:
                position = grabPose;
                break;
        }
        this.Claw.setPosition(position);
    }
    public double getClaw() {
        return Claw.getPosition();
    }

    //Arm
    public double getArm() {
        return LeftLift.getPosition() + RightLift.getPosition() / 2;
    }
    public void ArmSetState(Subsystem.ArmState armState ) {
        this.armState = armState;

        double position = 0;
        switch (armState) {
            case Hover:
                position = hoverPose;
                break;
            case Intake:
                position = intakePose;
                break;
            case Score:
                position = scorePose;
                break;
            case Reset:
                position = nuetral;
                break;
            case Spec:
                position = specPose;
                break;
            case Auto:
                position = autoPose;
                break;
            case AutoScore:
                position = AutoScore;
                break;
        }
        this.LeftLift.setPosition(position);
        this.RightLift.setPosition(position);
    }

    //Wrist
    public void WristSetState(Subsystem.WristState wristState) {
        this.wristState = wristState;

        double position = 0;

        switch (wristState) {
            case Neutral:
                position = nuetralPose;
                break;
            case Horizontal:
                position = horizontalPose;
                break;
        }

        this.Wrist.setPosition(position);
    }
    public void setTilt(double newPose) {
        Wrist.setPosition(newPose);
    }
    public double getTilt() {
        return Wrist.getPosition();
    }

    //Pitch
    public void PitchSetState(Subsystem.PitchState pitchState) {
        this.pitchState = pitchState;

        double position = 0;

        switch (pitchState) {
            case Intake:
                position = Nan;
                break;
            case Score:
                position = pitch;
                break;
            case Spec:
                position = spec;
                break;
        }

        this.Pitch.setPosition(position);
    }

    //Slides
    public void setLiftPose(int pos) {
        if (pos <= slideMax && pos >= slideMin) slideCurrent = pos;
        System.out.println(slideCurrent);
        setLift();
    }
    public void setLift() {
        LeftSlide.setTargetPosition(slideCurrent);
        RightSlide.setTargetPosition(slideCurrent);
        LeftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftSlide.setPower(1);
        RightSlide.setPower(1);
    }
    public void SlideSetState(Subsystem.SlideState slideState) {
        this.slideState = slideState;

        int position = 0;

        switch (slideState) {
            case Retracted:
                LeftSlide.setPower(0.55);
                RightSlide.setPower(0.55);
                position = Reset;
                break;
            case Score:
                position = High;
                break;
            case Spec:
                position = Spec;
                break;
        }
        setLiftPose(position);
    }

    //Hang
    public void setHangPose(int pos) {
        if (pos <= hangMax && pos >= hangMin) hangCurrent = pos;
        System.out.println(hangCurrent);
        setHang();
    }
    public void setHang() {
        LeftHang.setTargetPosition(hangCurrent);
        RightHang.setTargetPosition(hangCurrent);
        LeftHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftHang.setPower(1);
        RightHang.setPower(1);
    }
    public void ClimbSetState(Subsystem.ClimbState climbState) {
        this.climbState = climbState;

        int position = 0;

        switch (climbState) {
            case Retract:
                position = climb;
                break;
            case Hang:
                position = lineUp;
                break;
        }
        setHangPose(position);
    }

    //Auto Stuff
    public void raiseSlides() {
        this.ArmSetState(Subsystem.ArmState.Score);
        this.SlideSetState(Subsystem.SlideState.Score);
    }
    public void grabSample() {
        this.ClawSetState(Subsystem.ClawState.Closed);
    }

    public void init() {
        ClawSetState(Subsystem.ClawState.Closed);
        WristSetState(Subsystem.WristState.Neutral);
        //ClimbSetState(Subsystem.ClimbState.Retract);
        SlideSetState(Subsystem.SlideState.Retracted);
        PitchSetState(Subsystem.PitchState.Intake);
        ArmSetState(Subsystem.ArmState.Score);
    }
}
