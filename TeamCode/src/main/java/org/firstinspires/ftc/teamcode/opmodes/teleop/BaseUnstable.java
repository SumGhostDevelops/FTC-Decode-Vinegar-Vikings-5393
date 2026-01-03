package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.controls.commands.DriveCommands;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.definitions.RobotContext;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * This class is for implementing and testing new but unstable features
 */
public abstract class BaseUnstable extends CommandOpMode
{
    protected Team team;
    protected RobotContext robot;

    @Override
    public void initialize()
    {
        robot = new RobotContext(team, hardwareMap, telemetry, gamepad1, gamepad2);

        register(robot.drive, robot.intake, robot.transfer, robot.turret, robot.outtake, robot.odometry);


        DoubleSupplier axial = () -> gamepad1.left_stick_y;
        DoubleSupplier lateral = () -> gamepad1.left_stick_x;
        DoubleSupplier yaw = () -> gamepad1.right_stick_x;
        Supplier<Angle> driverHeading = () -> robot.odometry.getDriverHeading();

        robot.drive.setDefaultCommand(new DriveCommands.Manuever(robot.drive, lateral, axial, yaw, driverHeading));

        telemetry.setAutoClear(RobotConstants.Telemetry.SET_AUTOCLEAR);
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
        displayTelemetry();
        telemetry.update();
    }

    @Override
    public void run()
    {
        update();
        super.run();
    }

    @Override
    public void end()
    {
        robot.odometry.close();
    }
}