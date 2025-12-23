package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

public class Gamepads
{
    public GamepadEx gamepad1;
    public GamepadEx gamepad2;

    public Gamepads(Gamepad gamepad1, Gamepad gamepad2)
    {
        this.gamepad1 = new GamepadEx(gamepad1);
        this.gamepad2 = new GamepadEx(gamepad2);
    }
}
