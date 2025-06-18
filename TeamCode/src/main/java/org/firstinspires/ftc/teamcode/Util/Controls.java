package org.firstinspires.ftc.teamcode.Util;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.DoubleSupplier;

public class Controls {
    public static class Buttons {
        public GamepadEx gamepad1;

        //specific commands
        public GamepadButton Cycle;
        public GamepadButton Wrist;
        public GamepadButton Lift;
        public GamepadButton Swap;

        public DoubleSupplier driveY;
        public DoubleSupplier driveX;
        public DoubleSupplier yaw;

        public Buttons(Gamepad gamepad1) {
            this.gamepad1 = new GamepadEx(gamepad1);

            this.Cycle = this.gamepad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);
            this.Wrist = this.gamepad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
            this.Lift = this.gamepad1.getGamepadButton(GamepadKeys.Button.B);
            this.Swap = this.gamepad1.getGamepadButton(GamepadKeys.Button.Y);

            // Method reference to avoid creating a lambda
            this.driveY = this.gamepad1::getLeftY;
            this.driveX = this.gamepad1::getLeftX;
            this.yaw = this.gamepad1::getRightX;
        }
    }
}
