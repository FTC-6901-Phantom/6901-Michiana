package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.Subsystem;

@Config
public class HardwareSubsystem {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Servo LeftLift, RightLift;
    public Servo Pitch, Wrist, Claw;

    public DcMotor LeftSlide, RightSlide, LeftHang, RightHang;

    //Arm Pose
    public static double intakePose = 0.05, scorePose = 0.7, hoverPose = 0.15, nuetral = 0.5;

    //Claw Pose
    public static double openPose = 0.4, grabPose = 0.7;

    //Wrist Pose
    public static double nuetralPose = 0.3, horizontalPose = 0.65;

    //Pitch Pose
    public static double pitch = 0.6, Nan = 0;

    //Slide Pose
    public static int slideMin = -5;
    public static int slideMax = 1450;

    public static int High = 1450;
    public static int Reset = 0;
    public int slideCurrent = 0;

    //Hang Pose
    public static int hangMin = -5;
    public static int hangMax = 1450;

    public static int lineUp = 1450;
    public static int climb = 0;
    public int hangCurrent = 0;

    public HardwareSubsystem(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;

        //Servos
        LeftLift = (Servo) opMode.hardwareMap.get("LeftLift");
        RightLift = (Servo) opMode.hardwareMap.get("RightLift");
        Pitch = (Servo) opMode.hardwareMap.get("Pitch");
        Wrist = (Servo) opMode.hardwareMap.get("Wrist");
        Claw = (Servo) opMode.hardwareMap.get("Claw");

        RightLift.setDirection(Servo.Direction.REVERSE);

        //Motors
        LeftSlide = (DcMotor) opMode.hardwareMap.get("LeftSlide");
        RightSlide = (DcMotor) opMode.hardwareMap.get("RightSlide");
        LeftHang = (DcMotor) opMode.hardwareMap.get("LeftHang");
        RightHang = (DcMotor) opMode.hardwareMap.get("RightHang");

        //Slides
        LeftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        RightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

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
    public void ClawGrab() {
        Claw.setPosition(grabPose);
    }
    public void ClawOpen() {
        Claw.setPosition(openPose);
    }

    //Arm
    public void Bucket() {
        LeftLift.setPosition(scorePose);
        RightLift.setPosition(scorePose);
    }
    public void Intake() {
        LeftLift.setPosition(intakePose);
        RightLift.setPosition(intakePose);
    }
    public void Hover() {
        LeftLift.setPosition(hoverPose);
        RightLift.setPosition(hoverPose);
    }
    public void Nuetral() {
        LeftLift.setPosition(nuetral);
        RightLift.setPosition(nuetral);
    }
    public double getArm() {
        return LeftLift.getPosition() + RightLift.getPosition() / 2;
    }

    //Wrist
    public void wristNuetral() {
        Wrist.setPosition(nuetralPose);
    }
    public void wristHorizontal() {
        Wrist.setPosition(horizontalPose);
    }
    public void setTilt(double newPose) {
        Wrist.setPosition(newPose);
    }
    public double getTilt() {
        return Wrist.getPosition();
    }

    //Pitch
    public void Score() {
        Pitch.setPosition(pitch);
    }
    public void pitchReset() {
        Pitch.setPosition(Nan);
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

    public void High() {
        setLiftPose(High);
    }
    public void Reset() {
        setLiftPose(Reset);
    }

    //Hang
    public void setHangPose(int pos) {
        if (pos <= hangMax && pos >= hangMin) hangCurrent = pos;
        System.out.println(hangCurrent);
        setHang();
    }
    public void setHang() {
        LeftHang.setTargetPosition(hangCurrent);
        LeftHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftHang.setPower(1);
        RightHang.setTargetPosition(hangCurrent);
        RightHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightHang.setPower(1);
    }

    public void LineUp() {
        setHangPose(lineUp);
    }
    public void Climb() {
        setHangPose(climb);
    }

    public void init() {
        ClawOpen();
        Climb();
        Reset();
        Score();
        wristNuetral();
        Nuetral();
    }
}
