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
        Score,
        Spec,
        Auto
    }

    //Claw states
    public enum ClawState {
        Open,
        Closed
    }

    //Pitch States
    public enum PitchState {
        Intake,
        Score,
        Spec
    }

    //Arm States
    public enum ArmState {
        Hover,
        Intake,
        Score,
        Reset,
        Spec,
        Auto,
        AutoScore
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
        Grab,
        Return,
        Score,
        Finish,
        Nothing
    }

    //Auto Cycle states
    public enum AutoState {
        Start,
        Hover,
        Intake,
        Grab,
        Score,
        Line,
        Finish,
        armHover
    }

    //Spec states
    public enum SpecState {
        Intake,
        Spec,
        Nothing
    }

    //Tele states
    public enum TeleState {
        Spec,
        Sample
    }
}