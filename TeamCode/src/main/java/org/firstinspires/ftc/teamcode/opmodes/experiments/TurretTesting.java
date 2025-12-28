package org.firstinspires.ftc.teamcode.opmodes.experiments;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.controls.Gamepads;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.measure.Angle;

@TeleOp(name = "Turret Testing", group = "Experimental")
public class TurretTesting extends CommandOpMode
{
    private Turret turret;

    @Override
    public void initialize()
    {
        RobotHardware hw = new RobotHardware(hardwareMap, telemetry);
        turret = new Turret(hw.turret, RobotConstants.Turret.FORWARD_ANGLE);

        register(turret);

        Gamepads gamepads = new Gamepads(gamepad1, gamepad2);
        Button dpadUp = new GamepadButton(gamepads.driver, GamepadKeys.Button.DPAD_UP);
        Button dpadLeft = new GamepadButton(gamepads.driver, GamepadKeys.Button.DPAD_LEFT);
        Button dpadDown = new GamepadButton(gamepads.driver, GamepadKeys.Button.DPAD_DOWN);
        Button dpadRight = new GamepadButton(gamepads.driver, GamepadKeys.Button.DPAD_RIGHT);

        dpadUp.whenPressed(turret.turnToCommand(new Angle(0, AngleUnit.DEGREES)));
        dpadLeft.whenPressed(turret.turnToCommand(new Angle(90, AngleUnit.DEGREES)));
        dpadDown.whenPressed(turret.turnToCommand(new Angle(180, AngleUnit.DEGREES)));
        dpadRight.whenPressed(turret.turnToCommand(new Angle(270, AngleUnit.DEGREES)));
    }

    @Override
    public void run()
    {
        super.run();
        telemetry.addData("Relative Heading", turret.getRelativeUnnormalizedAngle().toUnit(UnnormalizedAngleUnit.DEGREES));
    }
}