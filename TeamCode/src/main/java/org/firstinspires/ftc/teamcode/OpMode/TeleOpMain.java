package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.Commands.ArmCommand;
import org.firstinspires.ftc.teamcode.Commands.LiftCommand;
import org.firstinspires.ftc.teamcode.Commands.WristCommand;
import org.firstinspires.ftc.teamcode.Subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.Util.Constants.FConstants;
import org.firstinspires.ftc.teamcode.Util.Constants.LConstants;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class TeleOpMain extends OpMode {
    private Follower follower;
    public ArmCommand arm;
    public LiftCommand lift;
    public WristCommand wrist;
    public HardwareSubsystem hardware;
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private final Pose startPose = new Pose(0,0,0);

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        arm = new ArmCommand(this);
        wrist = new WristCommand(this);
        lift = new LiftCommand(this);
        hardware = new HardwareSubsystem(this);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        hardware.init();
    }

    @Override
    public void loop() {
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();

        arm.CycleTele(runningActions, dash, gamepad1.left_bumper);
        lift.SlideTele(runningActions, dash, gamepad1.b);
        wrist.WristTele(runningActions, dash, gamepad1.right_bumper);

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
        telemetry.update();

        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            TelemetryPacket packet = new TelemetryPacket();
            action.preview(packet.fieldOverlay());
            if (!action.run(packet)) {
                continue;
            }
            newActions.add(action);
            dash.sendTelemetryPacket(packet);
        }
        runningActions = newActions;
    }

}
