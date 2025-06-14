package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {

    //Subsystems
    public HardwareSubsystem robot;
    public Telemetry telemetry;

    //Internal Subsystem State
    public Subsystem.CycleState cycleState;
    public Subsystem.SlideState slideState;
    public Subsystem.ClawState clawState;
    public Subsystem.PitchState pitchState;
    public Subsystem.ArmState armState;
    public Subsystem.ClimbState climbState;
    public Subsystem.WristState wristState;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.wristState = Subsystem.WristState.Neutral;
        this.cycleState = Subsystem.CycleState.Score;
        this.robot = new HardwareSubsystem(hardwareMap, telemetry);
        this.telemetry = telemetry;

        robot.init();
    }

    public void nextCycle() {
        switch (this.cycleState) {
            case Hover:
                this.cycleState = Subsystem.CycleState.Intake;
                break;
            case Intake:
                this.cycleState = Subsystem.CycleState.Score;
                break;
            case Score:
                this.cycleState = Subsystem.CycleState.Hover;
                break;
        }
    }

    public void nextWrist() {
        switch (this.wristState) {
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
        telemetry.addData("Cycle", this.cycleState.toString());
        telemetry.addData("Wrist", this.wristState.toString());

        switch (this.cycleState) {
            case Hover:
                robot.ArmSetState(this.robot.hardwareMap, Subsystem.ArmState.Hover);
                robot.ClawSetState(this.robot.hardwareMap, Subsystem.ClawState.Open);
                robot.WristSetState(this.robot.hardwareMap, Subsystem.WristState.Neutral);
                break;
            case Intake:
                robot.ArmSetState(this.robot.hardwareMap, Subsystem.ArmState.Intake);
                if (robot.getArm() == HardwareSubsystem.intakePose) {
                    robot.ClawSetState(this.robot.hardwareMap, Subsystem.ClawState.Closed);
                } if (robot.getClaw() == HardwareSubsystem.grabPose) {
                    robot.ArmSetState(this.robot.hardwareMap, Subsystem.ArmState.Score);
                    robot.PitchSetState(this.robot.hardwareMap, Subsystem.PitchState.Score);
                }
                break;
            case Score:
                robot.ClawSetState(this.robot.hardwareMap, Subsystem.ClawState.Open);
                if (robot.getClaw() == HardwareSubsystem.openPose) {
                    robot.SlideSetState(this.robot.hardwareMap, Subsystem.SlideState.Retracted);
                    robot.ArmSetState(this.robot.hardwareMap, Subsystem.ArmState.Reset);
                }
                break;
        }

        switch (this.wristState) {
            case Neutral:
                robot.WristSetState(this.robot.hardwareMap, Subsystem.WristState.Neutral);
                break;
            case Horizontal:
                robot.WristSetState(this.robot.hardwareMap, Subsystem.WristState.Horizontal);
                break;
        }
    }
}
