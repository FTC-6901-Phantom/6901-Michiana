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

public class ArmCommand {
    public HardwareSubsystem robot;
    public HardwareMap hardwareMap;
    private boolean wasInputPressed = false;

    enum CycleState {
        GRABBING,
        SCORE,
        DROPOFF
    }

    CycleState cycleState = CycleState.GRABBING;

    public ArmCommand(OpMode opMode) {
        robot = new HardwareSubsystem(opMode);
    }

    public void CycleTele(List<Action> runningActions, FtcDashboard dashboard, boolean input) {

        if (input && !wasInputPressed) {

            switch (cycleState) {
                case GRABBING:
                    runningActions.add(
                            new SequentialAction(
                                    new InstantAction(robot::Hover),
                                    new InstantAction(robot::ClawOpen),
                                    new InstantAction(robot::pitchReset)
                            )
                    );
                    cycleState = CycleState.SCORE;
                    break;

                case SCORE:
                    runningActions.add(
                            new SequentialAction(
                                    new InstantAction(robot::Intake),
                                    new SleepAction(0.1),
                                    new InstantAction(robot::ClawGrab),
                                    new SleepAction(0.1),
                                    new InstantAction(robot::Bucket),
                                    new InstantAction(robot::wristNuetral),
                                    new InstantAction(robot::Score)
                            )
                    );
                    cycleState = CycleState.DROPOFF;
                    break;
                case DROPOFF:
                    runningActions.add(
                            new SequentialAction(
                                    new InstantAction(robot::ClawOpen),
                                    new SleepAction(0.1),
                                    new InstantAction(robot::Nuetral),
                                    new SleepAction(0.2),
                                    new InstantAction(robot::Reset)
                            )
                    );
                    cycleState = CycleState.GRABBING;
                    break;
            }
        }
        wasInputPressed = input;
    }
}

