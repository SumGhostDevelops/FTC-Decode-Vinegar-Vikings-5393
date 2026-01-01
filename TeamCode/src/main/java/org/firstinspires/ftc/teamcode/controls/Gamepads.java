package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.controls.commands.DriveCommands;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

public class Gamepads
{
    public GamepadEx driverGamepadEx;
    public GamepadEx coDriverGamepadEx;

    public org.firstinspires.ftc.teamcode.subsystems.Drive drive;
    public Intake intake;
    public Transfer transfer;

    public Driver driver;

    public Gamepads(Gamepad driver, Gamepad coDriver, org.firstinspires.ftc.teamcode.subsystems.Drive drive, Intake intake, Transfer transfer)
    {
        this.driverGamepadEx = new GamepadEx(driver);
        this.coDriverGamepadEx = new GamepadEx(coDriver);

        this.drive = drive;
        this.intake = intake;
        this.transfer = transfer;

        this.driver = new Driver(driverGamepadEx);
        this.driver.bind();
    }

    public class Driver
    {
        public final Gamepad gamepad;

        private Button leftBumper;
        private Button rightBumper;

        private Button dpadUp;
        private Button dpadDown;
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
            leftBumper.whenPressed(new DriveCommands.DecreaseSpeed(drive));
            rightBumper.whenPressed(new DriveCommands.IncreaseSpeed(drive));
        }
    }
}