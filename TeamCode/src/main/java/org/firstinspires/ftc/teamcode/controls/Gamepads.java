package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.controls.commands.DriveCommands;
import org.firstinspires.ftc.teamcode.controls.commands.OdometryCommands;
import org.firstinspires.ftc.teamcode.controls.commands.OuttakeCommands;
import org.firstinspires.ftc.teamcode.definitions.Subsystems;

public class Gamepads
{
    public GamepadEx driverGamepadEx;
    public GamepadEx coDriverGamepadEx;

    public Subsystems subsystems;

    public Driver driver;

    public Gamepads(Gamepad driver, Gamepad coDriver, Subsystems subsystems)
    {
        this.driverGamepadEx = new GamepadEx(driver);
        this.coDriverGamepadEx = new GamepadEx(coDriver);

        this.subsystems = subsystems;

        this.driver = new Driver(driverGamepadEx);
        this.driver.bind();
    }

    public class Driver
    {
        public final Gamepad gamepad;

        private Button leftBumper;
        private Button rightBumper;

        public Button dpadUp;
        public Button dpadDown;
        private Button dpadLeft;
        private Button dpadRight;

        private Button A;
        private Button B;
        private Button X;
        private Button Y;

        private Button leftStick;
        private Button rightStick;

        private Trigger leftTrigger;
        private Trigger rightTrigger;

        public Driver(GamepadEx gamepadEx)
        {
            this.gamepad = gamepadEx.gamepad;

            leftBumper = gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);
            rightBumper = gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);

            dpadUp = gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP);
            dpadDown = gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);
            dpadLeft = gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
            dpadRight = gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);

            A = gamepadEx.getGamepadButton(GamepadKeys.Button.A);
            B = gamepadEx.getGamepadButton(GamepadKeys.Button.B);
            X = gamepadEx.getGamepadButton(GamepadKeys.Button.X);
            Y = gamepadEx.getGamepadButton(GamepadKeys.Button.Y);
        }

        public void bind()
        {
            leftBumper.whenPressed(new DriveCommands.DecreaseSpeed(subsystems.drive));
            rightBumper.whenPressed(new DriveCommands.IncreaseSpeed(subsystems.drive));

            A.toggleWhenPressed(new OuttakeCommands.On(subsystems.outtake), new OuttakeCommands.Off(subsystems.outtake));
            B.whenPressed(new OdometryCommands.SetForwardAngle(subsystems.odometry));
            dpadUp.whenPressed(new OuttakeCommands.ChangeTargetRPM(subsystems.outtake, 100));
            dpadDown.whenPressed(new OuttakeCommands.ChangeTargetRPM(subsystems.outtake, -100));
        }
    }
}