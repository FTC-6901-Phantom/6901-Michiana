package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class HardwareSubsystem {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Servo LeftLift, RightLift;
    public Servo Pitch, Wrist, Claw;

    public DcMotor LeftSlide, RightSlide, LeftHang, RightHang;

    //Arm Pose
    public static final double intakePose = 0.05,
                                scorePose = 0.9;

    //Claw Pose
    public static final double openPose = 0;

    public HardwareSubsystem(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;

        //Servos
        this.LeftLift = (Servo) hardwareMap.get("LeftLift");
        this.RightLift = (Servo) hardwareMap.get("RightLift");
        this.Pitch = (Servo) hardwareMap.get("Pitch");
        this.Wrist = (Servo) hardwareMap.get("Wrist");
        this.Claw = (Servo) hardwareMap.get("Claw");

        RightLift.setDirection(Servo.Direction.REVERSE);

        //Motors
        this.LeftSlide = (DcMotor) hardwareMap.get("LeftSlide");
        this.RightSlide = (DcMotor) hardwareMap.get("RightSlide");
        this.LeftHang = (DcMotor) hardwareMap.get("LeftHang");
        this.RightHang = (DcMotor) hardwareMap.get("RightHang");

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
}
