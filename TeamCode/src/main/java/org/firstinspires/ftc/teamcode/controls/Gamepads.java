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
    public GamepadEx driver;
    public GamepadEx coDriver;

    public Gamepads(Gamepad driver, Gamepad coDriver)
    {
        this.driver = new GamepadEx(driver);
        this.coDriver = new GamepadEx(coDriver);
    }
}