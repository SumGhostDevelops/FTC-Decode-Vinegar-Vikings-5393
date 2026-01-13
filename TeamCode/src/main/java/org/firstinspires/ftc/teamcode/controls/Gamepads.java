package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

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