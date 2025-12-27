package org.firstinspires.ftc.teamcode.opmodes.experiments;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.controls.Gamepads;
import org.firstinspires.ftc.teamcode.controls.commands.turret.Aim0;
import org.firstinspires.ftc.teamcode.controls.commands.turret.Aim180;
import org.firstinspires.ftc.teamcode.controls.commands.turret.Aim270;
import org.firstinspires.ftc.teamcode.controls.commands.turret.Aim90;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "Turret Testing", group = "Experimental")
public class TurretTesting extends CommandOpMode
{
    private Turret turret;
    private Aim0 aim0;
    private Aim90 aim90;
    private Aim180 aim180;
    private Aim270 aim270;

    @Override
    public void initialize()
    {
        RobotHardware hw = new RobotHardware(hardwareMap, telemetry);
        turret = new Turret(hw.turret, RobotConstants.Turret.FORWARD_ANGLE);

        register(turret);

        aim0 = new Aim0(turret);
        aim90 = new Aim90(turret);
        aim180 = new Aim180(turret);
        aim270 = new Aim270(turret);

        Gamepads gamepads = new Gamepads(gamepad1, gamepad2);
        Button dpadUp = new GamepadButton(gamepads.driver, GamepadKeys.Button.DPAD_UP
        ).whenPressed(aim0);
        Button dpadDown = new GamepadButton(gamepads.driver, GamepadKeys.Button.DPAD_DOWN
        ).whenPressed(aim180);
        Button dpadLeft = new GamepadButton(gamepads.driver, GamepadKeys.Button.DPAD_LEFT
        ).whenPressed(aim90);
        Button dpadRight = new GamepadButton(gamepads.driver, GamepadKeys.Button.DPAD_RIGHT
        ).whenPressed(aim270);
    }

    @Override
    public void run()
    {
        super.run();
        telemetry.addData("Relative Heading", turret.getRelativeHeading(UnnormalizedAngleUnit.DEGREES));
    }
}