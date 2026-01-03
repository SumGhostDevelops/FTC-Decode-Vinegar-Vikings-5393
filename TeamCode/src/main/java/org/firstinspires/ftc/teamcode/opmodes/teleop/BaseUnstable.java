package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

        register(robot.subsystems.drive, robot.subsystems.intake, robot.subsystems.transfer, robot.subsystems.turret, robot.subsystems.outtake, robot.subsystems.odometry);


        DoubleSupplier axial = () -> gamepad1.left_stick_y;
        DoubleSupplier lateral = () -> gamepad1.left_stick_x;
        DoubleSupplier yaw = () -> gamepad1.right_stick_x;
        Supplier<Angle> driverHeading = () -> robot.subsystems.odometry.getDriverHeading();

        robot.subsystems.drive.setDefaultCommand(new DriveCommands.Manuever(robot.subsystems.drive, lateral, axial, yaw, driverHeading));

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
        telemetry.addLine("--- Odometry ---");
        telemetry.addData("Relative Heading (deg)", robot.subsystems.odometry.getDriverHeading().getUnsignedAngle(AngleUnit.DEGREES));
        telemetry.addData("Absolute Heading (deg)", robot.subsystems.odometry.getAngle().getUnsignedAngle(AngleUnit.DEGREES));
        telemetry.addData("x (inches)", robot.subsystems.odometry.getFieldCoord().x.toUnit(DistanceUnit.INCH));
        telemetry.addData("y (inches)", robot.subsystems.odometry.getFieldCoord().y.toUnit(DistanceUnit.INCH));
        telemetry.addLine("--- Drive ---");
        telemetry.addData("Speed (power)", robot.subsystems.drive.getSpeed());
        telemetry.addLine("--- Outtake ---");
        telemetry.addData("Target RPM", robot.subsystems.outtake.getTargetRPM());
        telemetry.addData("True RPM", robot.subsystems.outtake.getRPM());
        telemetry.addLine("--- Turret ---");
        telemetry.addData("Relative Heading (deg)", robot.subsystems.turret.getRelativeAngle().getUnsignedAngle(AngleUnit.DEGREES));
        telemetry.addData("Absolute Heading (deg)", robot.subsystems.turret.getAbsoluteAngle(robot.subsystems.odometry.getAngle()).getUnsignedAngle(AngleUnit.DEGREES));
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
        robot.subsystems.odometry.close();
    }
}