package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.HardwareSubsystem;

import java.util.List;

public class WristCommand {
    public HardwareSubsystem robot;
    public HardwareMap hardwareMap;
    private boolean wasInputPressed = false;

    enum WristState {
        NUETRAL,
        HORIZONTAL
    }
    WristState wristState = WristState.NUETRAL;

    public WristCommand(OpMode opMode) {
        robot = new HardwareSubsystem(opMode);
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

