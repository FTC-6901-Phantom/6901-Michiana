package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Util.cmd.sleep;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.cmd;

public class IntakeSubsystem extends SubsystemBase {

    //Subsystems
    public HardwareSubsystem robot;
    public Telemetry telemetry;

    public ElapsedTime time;

    //Internal Subsystem State
    public Subsystem.CycleState cycleState;
    public Subsystem.WristState wristState;
    public Subsystem.SpecState specState;
    public Subsystem.TeleState teleState;
    public Subsystem.ClimbState climbState;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.wristState = Subsystem.WristState.Neutral;
        this.cycleState = Subsystem.CycleState.Finish;
        this.teleState = Subsystem.TeleState.Sample;
        this.specState = Subsystem.SpecState.Intake;
        this.climbState = Subsystem.ClimbState.Retract;
        this.robot = new HardwareSubsystem(hardwareMap, telemetry);
        this.telemetry = telemetry;
        this.time = new ElapsedTime();

        robot.init();
    }

    public void nextCycle() {
        switch (this.cycleState) {
            case Hover:
                this.cycleState = Subsystem.CycleState.Intake;
                break;
            case Intake:
                this.cycleState = Subsystem.CycleState.Grab;
                break;
            case Grab:
                this.cycleState = Subsystem.CycleState.Return;
                break;
            case Return:
                this.cycleState = Subsystem.CycleState.Score;
                break;
            case Score:
                this.cycleState = Subsystem.CycleState.Finish;
                break;
            case Finish:
                this.cycleState = Subsystem.CycleState.Hover;
        }
    }

    public void swapTele() {
        switch (teleState) {
            case Spec:
                this.teleState = Subsystem.TeleState.Sample;
                break;
            case Sample:
                this.teleState = Subsystem.TeleState.Spec;
                break;
        }
    }

    public void Reset() {
        this.cycleState = Subsystem.CycleState.Hover;
    }

    public void nextSpec() {
        switch (specState) {
            case Spec:
                this.specState = Subsystem.SpecState.Intake;
                break;
            case Intake:
                this.specState = Subsystem.SpecState.Spec;
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

    public void nextClimb() {
        switch (this.climbState) {
            case Hang:
                this.climbState = Subsystem.ClimbState.Retract;
                break;
            case Retract:
                this.climbState = Subsystem.ClimbState.Hang;
                break;
        }
    }

    @Override
    public void periodic() {
        telemetry.addData("Cycle", this.cycleState.toString());
        telemetry.addData("Wrist", this.wristState.toString());
        telemetry.addData("TeleState", this.teleState.toString());

        telemetry.addLine();
        telemetry.addData("ClawPose", robot.getClaw());
        telemetry.addData("ArmPose", robot.getArm());


        switch (this.teleState) {
            case Sample:
                switch (this.cycleState) {
                    case Nothing:
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
                        robot.ArmSetState(Subsystem.ArmState.Intake);
                        if (robot.isIntake()) nextCycle();
                        break;
                    case Grab:
                        if (time.seconds() >= 0.2) {
                            robot.ClawSetState(Subsystem.ClawState.Closed);
                        }
                        if (time.seconds() >= 0.4) robot.ArmSetState(Subsystem.ArmState.Hover);
                        break;
                    case Return:
                        robot.ArmSetState(Subsystem.ArmState.Reset);
                        robot.PitchSetState(Subsystem.PitchState.Score);
                        break;
                    case Score:
                        robot.SlideSetState(Subsystem.SlideState.Score);
                        robot.ArmSetState(Subsystem.ArmState.Score);
                        time.reset();
                        break;
                    case Finish:
                        time.startTime();
                        robot.ClawSetState(Subsystem.ClawState.Open);
                        if (time.seconds() > 0.4) {
                            robot.SlideSetState(Subsystem.SlideState.Retracted);
                            robot.ArmSetState(Subsystem.ArmState.Reset);
                        }
                        break;
                }
                break;
            case Spec:
                switch (specState) {
                    case Nothing:
                        break;
                    case Intake:
                        robot.ArmSetState(Subsystem.ArmState.Score);
                        robot.ClawSetState(Subsystem.ClawState.Open);
                        robot.SlideSetState(Subsystem.SlideState.Retracted);
                        robot.PitchSetState(Subsystem.PitchState.Score);
                        time.reset();
                    case Spec:
                        time.startTime();
                        robot.ClawSetState(Subsystem.ClawState.Closed);
                        if (time.seconds() > 0.3) {
                            robot.ArmSetState(Subsystem.ArmState.Spec);
                            robot.PitchSetState(Subsystem.PitchState.Spec);
                            robot.SlideSetState(Subsystem.SlideState.Spec);
                        }
                }
                break;
        }

//        switch (this.climbState) {
//            case Hang:
//                robot.ClimbSetState(Subsystem.ClimbState.Hang);
//                break;
//            case Retract:
//                robot.ClimbSetState(Subsystem.ClimbState.Retract);
//                break;
//        }

        switch (this.wristState) {
            case Neutral:
                robot.WristSetState(Subsystem.WristState.Neutral);
                break;
            case Horizontal:
                robot.WristSetState(Subsystem.WristState.Horizontal);
                break;
        }
    }
}
