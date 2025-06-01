package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.HardwareSubsystem;

import java.util.List;

public class LiftCommand {
    public HardwareSubsystem robot;
    public HardwareMap hardwareMap;
    private boolean wasInputPressed = false;

    enum CycleState {
        GRABBING,
        SCORE,
        DROPOFF
    }

    enum LiftState {
        EXTENDED,
        RESET
    }

    enum WristState {
        NUETRAL,
        HORIZONTAL
    }

    CycleState cycleState = CycleState.GRABBING;
    LiftState liftState = LiftState.EXTENDED;
    WristState wristState = WristState.NUETRAL;

    public LiftCommand(OpMode opMode) {
        robot = new HardwareSubsystem(opMode);
    }

    public void SlideTele(List<Action> runningActions, FtcDashboard ftcDashboard, boolean input) {

        if (input && !wasInputPressed) {
            switch (liftState) {
                case EXTENDED:
                    runningActions.add(
                            new SequentialAction(
                                    new InstantAction(robot::High)
                            )
                    );
                    liftState = LiftState.RESET;
                    break;
                case RESET:
                    runningActions.add(
                            new SequentialAction(
                                    new InstantAction(robot::Reset)
                            )
                    );
                    liftState = LiftState.EXTENDED;
                    break;
            }
        }
        wasInputPressed = input;
    }

    public void WristTele(List<Action> runningActions, FtcDashboard ftcDashboard, boolean input) {

        if (input && !wasInputPressed) {
            switch (wristState) {
                case NUETRAL:
                    runningActions.add(
                            new SequentialAction(
                                    new InstantAction(robot::wristHorizontal)
                            )
                    );
                    wristState = WristState.HORIZONTAL;
                    break;
                case HORIZONTAL:
                    runningActions.add(
                            new SequentialAction(
                                    new InstantAction(robot::wristNuetral)
                            )
                    );
                    wristState = WristState.NUETRAL;
                    break;
            }
        }
        wasInputPressed = input;
    }
}

