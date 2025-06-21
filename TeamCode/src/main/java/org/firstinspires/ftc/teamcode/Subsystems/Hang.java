package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Hang {
    public final DcMotor LeftClimb;
    public final DcMotor RightClimb;

    private final HardwareMap hardwareMap;
    private final Gamepad Driver1;
    public final Telemetry telemetry;

    public Hang(OpMode opMode) {
        Driver1 = opMode.gamepad1;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;

        LeftClimb = hardwareMap.get(DcMotor.class, "LClimb");
        LeftClimb.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftClimb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RightClimb = hardwareMap.get(DcMotor.class, "RClimb");
        RightClimb.setDirection(DcMotorSimple.Direction.FORWARD);
        RightClimb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders to start at 0
        LeftClimb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightClimb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void teleOp() {
        if (Driver1.dpad_left) moveByTicks(150);
        else if (Driver1.dpad_right) moveByTicks(-150);
        else Stop();
    }

    public void moveByTicks(int ticks) {
        // Calculate new target positions by adding ticks
        int newLeftTarget = LeftClimb.getCurrentPosition() + ticks;
        int newRightTarget = RightClimb.getCurrentPosition() + ticks;

        LeftClimb.setTargetPosition(newLeftTarget);
        RightClimb.setTargetPosition(newRightTarget);

        LeftClimb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightClimb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftClimb.setPower(1);
        RightClimb.setPower(1);
    }

    public void Stop() {
        // Stop power but hold position
        LeftClimb.setPower(0);
        RightClimb.setPower(0);
    }
}