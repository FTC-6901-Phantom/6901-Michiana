package org.firstinspires.ftc.teamcode.Subsystems;

public class Subsystem {

    // Wrist states
    public enum WristState {
        Neutral,
        Horizontal
    }

    //Slide states
    public enum SlideState {
        Retracted,
        Score
    }

    //Claw states
    public enum ClawState {
        Open,
        Closed
    }

    //Pitch States
    public enum PitchState {
        Intake,
        Score
    }

    //Arm States
    public enum ArmState {
        Hover,
        Intake,
        Score,
        Reset
    }

    //Climb States
    public enum ClimbState {
        Hang,
        Retract
    }

    //Cycle states
    public enum CycleState {
        Hover,
        Intake,
        Score
    }

    //Spec states
    public enum SpecState {
        Intake,
        Spec
    }

    //Tele states
    public enum TeleState {
        Spec,
        Sample
    }
}