package org.firstinspires.ftc.teamcode.OpMode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;

import org.firstinspires.ftc.teamcode.Util.Constants.FConstants;
import org.firstinspires.ftc.teamcode.Util.Constants.LConstants;

import static org.firstinspires.ftc.teamcode.OpMode.Auto.AutoPath.preload;

@Autonomous(name = "Auto Path Only")
public class PathOnlyAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        Constants.setConstants(FConstants.class, LConstants.class);

        Follower follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        Pose startPose = new Pose(7.2, 113.1, Math.toRadians(0));  // Adjust heading if needed
        follower.setStartingPose(startPose);

        waitForStart();

        follower.followPath(preload());

        while (opModeIsActive()) {
            follower.update();

            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }

        // Stop the robot when done
    }
}
