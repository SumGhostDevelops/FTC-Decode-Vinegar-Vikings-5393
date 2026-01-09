package org.firstinspires.ftc.teamcode.opmodes.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.definitions.RobotContext;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;

@TeleOp(name = "Turret Testing", group = "Experimental")
@Disabled
public class TurretTesting extends CommandOpMode
{
    private Turret turret;

    @Override
    public void initialize()
    {
        RobotHardware hw = new RobotHardware(hardwareMap, telemetry);
        RobotContext robot = new RobotContext(Team.BLUE, hardwareMap, telemetry, gamepad1, gamepad2);

        register(turret);

        Button dpadUp = new GamepadButton(robot.gamepads.driver, GamepadKeys.Button.DPAD_UP);
        Button dpadLeft = new GamepadButton(robot.gamepads.driver, GamepadKeys.Button.DPAD_LEFT);
        Button dpadDown = new GamepadButton(robot.gamepads.driver, GamepadKeys.Button.DPAD_DOWN);
        Button dpadRight = new GamepadButton(robot.gamepads.driver, GamepadKeys.Button.DPAD_RIGHT);

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