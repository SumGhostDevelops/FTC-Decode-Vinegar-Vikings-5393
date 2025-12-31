package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.controls.InputHandler;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.controls.Gamepads;
import org.firstinspires.ftc.teamcode.definitions.RobotContext;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;

import java.util.function.DoubleSupplier;

/**
 * This class is for implementing and testing new but unstable features
 */
public abstract class BaseUnstable extends CommandOpMode
{
    protected Team team;
    protected RobotContext robot;
    protected InputHandler input;

    protected DoubleSupplier axial;
    protected DoubleSupplier lateral;
    protected DoubleSupplier yaw;

    @Override
    public void initialize()
    {
        axial = () -> -gamepad1.left_stick_y;
        lateral = () -> gamepad1.left_stick_x;
        yaw = () -> gamepad1.right_stick_x;
        Gamepads gamepads = new Gamepads(gamepad1, gamepad2);

        robot = new RobotContext(team, hardwareMap, telemetry, gamepads, new DoubleSupplier[]{axial, lateral, yaw});

        robot.gamepads.driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(
                () -> robot.turret.setTargetRelative(new Angle(0, AngleUnit.DEGREES))
                ));
        robot.gamepads.driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(
                        () -> robot.turret.setTargetRelative(new Angle(90, AngleUnit.DEGREES))
                ));
        robot.gamepads.driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(
                        () -> robot.turret.setTargetRelative(new Angle(180, AngleUnit.DEGREES))
                ));
        robot.gamepads.driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(
                        () -> robot.turret.setTargetRelative(new Angle(270, AngleUnit.DEGREES))
                ));

        robot.gamepads.driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(
                        () -> robot.drive.decreaseSpeed()
                ));
        robot.gamepads.driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(
                        () -> robot.drive.increaseSpeed()
                ));

        telemetry.setAutoClear(RobotConstants.TELEMETRY_SET_AUTOCLEAR);
        telemetry.addData("Status", "Initialized for " + team);
        displayTelemetry();
        telemetry.update();
    }

    /**
     * For customizing what shows up in the telemetry
     */
    protected void displayTelemetry()
    {
        telemetry.addData("Team", team);
        telemetry.addData("Drive Speed", robot.drive.getSpeed());
        telemetry.addData("Turret Heading", robot.turret.getRelativeAngle());
        telemetry.addData("Heading", robot.odometry.getAngle().toUnit(AngleUnit.DEGREES).getUnsignedAngle(AngleUnit.DEGREES));
        telemetry.addLine("----------");
    }

    /**
     * For updating important systems, like the Telemetry or Localization
     */
    protected void update()
    {
        telemetry.update();
        input.update();
    }

    public void run()
    {
        displayTelemetry();
        robot.turret.setTargetAbsolute(new Angle(90, AngleUnit.DEGREES), robot.odometry.getAngle());

        // robot.turret.setTargetAbsolute(robot.odometry.getFieldCoord().angleTo(team.goal.coord), robot.odometry.getAngle());
    }

    @Override
    public void end()
    {
        robot.odometry.close();
    }
}