package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoSubsystem extends SubsystemBase {

    //Subsystems
    public HardwareSubsystem robot;
    public Telemetry telemetry;

    public ElapsedTime time;

    //Internal Subsystem State
    public Subsystem.AutoState autoState;
    public Subsystem.WristState wristState;

    public AutoSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.autoState = Subsystem.AutoState.Score;
        this.wristState = Subsystem.WristState.Neutral;
        this.robot = new HardwareSubsystem(hardwareMap, telemetry);
        this.telemetry = telemetry;
        this.time = new ElapsedTime();

        robot.init();
    }

    public void nextAutoCycle() {
        switch (autoState) {
            case Hover:
                this.autoState = Subsystem.AutoState.Intake;
                break;
            case Intake:
                this.autoState = Subsystem.AutoState.Grab;
                break;
            case Grab:
                this.autoState = Subsystem.AutoState.Score;
                break;
            case Score:
                this.autoState = Subsystem.AutoState.Finish;
                break;
            case Finish:
                this.autoState = Subsystem.AutoState.Hover;
                break;
        }
    }
    public void nextAutoWrist() {
        switch (wristState) {
            case Horizontal:
                this.wristState = Subsystem.WristState.Neutral;
                break;
            case Neutral:
                this.wristState = Subsystem.WristState.Horizontal;
                break;
        }
    }

    @Override
    public void periodic() {
        telemetry.addData("AutoState", this.autoState.toString());
        telemetry.addData("Wrist", this.wristState.toString());

        switch (autoState) {
            case Hover:
                time.reset();
                robot.SlideSetState(Subsystem.SlideState.Retracted);
                robot.ArmSetState(Subsystem.ArmState.Hover);
                robot.ClawSetState(Subsystem.ClawState.Open);
                robot.WristSetState(Subsystem.WristState.Neutral);
                robot.PitchSetState(Subsystem.PitchState.Intake);
                break;
            case Intake:
                time.startTime();
                robot.ArmSetState(Subsystem.ArmState.Intake);
                if (robot.isIntake()) nextAutoCycle();
                break;
            case Grab:
                if (time.seconds() >= 0.2) {
                    robot.ClawSetState(Subsystem.ClawState.Closed);
                }
                if (robot.isGrabbed() && robot.isIntake()) nextAutoCycle();
                break;
            case Score:
                robot.SlideSetState(Subsystem.SlideState.Score);
                robot.ArmSetState(Subsystem.ArmState.Score);
                time.reset();
                break;
            case Finish:
                time.startTime();
                robot.PitchSetState(Subsystem.PitchState.Score);
                if (time.seconds() >= 0.3) robot.ClawSetState(Subsystem.ClawState.Open);
                if (time.seconds() >= 0.6) {
                    robot.ArmSetState(Subsystem.ArmState.Reset);
                    robot.SlideSetState(Subsystem.SlideState.Retracted);
                }
                break;
        }

        switch (wristState) {
            case Neutral:
                robot.WristSetState(Subsystem.WristState.Neutral);
                break;
            case Horizontal:
                robot.WristSetState(Subsystem.WristState.Horizontal);
                break;
        }
    }
}
