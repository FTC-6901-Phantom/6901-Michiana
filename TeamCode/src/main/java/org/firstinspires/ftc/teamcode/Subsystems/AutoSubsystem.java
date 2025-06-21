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
        this.autoState = Subsystem.AutoState.Start;
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
                this.autoState = Subsystem.AutoState.Line;
                break;
            case Line:
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
            case Start:
                break;
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
                robot.ArmSetState(Subsystem.ArmState.Auto);
                if (robot.isIntake()) nextAutoCycle();
                break;
            case Grab:
                if (time.seconds() >= 0.3) {
                    robot.ClawSetState(Subsystem.ClawState.Closed);
                }
                break;
            case Score:
                robot.SlideSetState(Subsystem.SlideState.Score);
                robot.PitchSetState(Subsystem.PitchState.Score);
                robot.ArmSetState(Subsystem.ArmState.Reset);
                time.reset();
                break;
            case Line:
                time.startTime();
                robot.ArmSetState(Subsystem.ArmState.AutoScore);
                if (time.seconds() >= 0.3) robot.ClawSetState(Subsystem.ClawState.Open);
                if (robot.Claw.getPosition() == HardwareSubsystem.grabPose) nextAutoCycle();
                break;
            case Finish:
                if (time.seconds() >= 0.5) {
                    robot.ArmSetState(Subsystem.ArmState.Reset);
                    robot.SlideSetState(Subsystem.SlideState.Retracted);
                }
                break;
            case armHover:
                robot.ArmSetState(Subsystem.ArmState.Hover);
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
