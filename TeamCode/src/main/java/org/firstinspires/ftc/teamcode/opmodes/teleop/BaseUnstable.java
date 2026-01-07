package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.controls.commands.DriveCommands;
import org.firstinspires.ftc.teamcode.controls.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.controls.commands.OdometryCommands;
import org.firstinspires.ftc.teamcode.controls.commands.OuttakeCommands;
import org.firstinspires.ftc.teamcode.controls.commands.TransferCommands;
import org.firstinspires.ftc.teamcode.controls.commands.TurretCommands;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.Subsystems;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.definitions.RobotContext;
import org.firstinspires.ftc.teamcode.util.dashboard.FieldDrawing;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;

import java.util.function.BooleanSupplier;
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

        // Initialize Panels Field with FTC Standard coordinates
        FieldDrawing.init();

        register(robot.subsystems.drive, robot.subsystems.intake, robot.subsystems.transfer, robot.subsystems.turret, robot.subsystems.outtake, robot.subsystems.odometry);


        DoubleSupplier axial = () -> gamepad1.left_stick_y;
        DoubleSupplier lateral = () -> gamepad1.left_stick_x;
        DoubleSupplier yaw = () -> gamepad1.right_stick_x;
        Supplier<Angle> driverHeading = () -> robot.subsystems.odometry.getDriverHeading();

        robot.subsystems.drive.setDefaultCommand(new DriveCommands.Manuever(robot.subsystems.drive, lateral, axial, yaw, driverHeading));

        bindKeys();

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
        telemetry.addData("Raw Yaw", robot.hw.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Relative Heading (deg)", robot.subsystems.odometry.getDriverHeading().getUnsignedAngle(AngleUnit.DEGREES));
        telemetry.addData("Absolute Heading (deg)", robot.subsystems.odometry.getAngle().getUnsignedAngle(AngleUnit.DEGREES));
        telemetry.addData("x (inches)", robot.subsystems.odometry.getFieldCoord().x.toUnit(DistanceUnit.INCH));
        telemetry.addData("y (inches)", robot.subsystems.odometry.getFieldCoord().y.toUnit(DistanceUnit.INCH));
        telemetry.addLine("--- Drive ---");
        telemetry.addData("Speed (power)", robot.subsystems.drive.getSpeed());
        telemetry.addLine("--- Outtake ---");
        telemetry.addData("Target RPM", robot.subsystems.outtake.getTargetRPM());
        telemetry.addData("True RPM", robot.subsystems.outtake.getRPM());
        telemetry.addData("Is Stable", robot.subsystems.outtake.isReady());
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
        robot.hw.clearHubCache();

        // Draw robot position on Panels Dashboard Field panel
        FieldDrawing.drawRobot(
                robot.subsystems.odometry.getPose(),
                robot.subsystems.turret.getAbsoluteAngle(robot.subsystems.odometry.getAngle()).getUnsignedAngle(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES),
                robot.team.goal.coord
        );
        FieldDrawing.sendPacket();
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

    public void bindKeys()
    {
        GamepadEx driver = robot.gamepads.driver;
        Subsystems subsystems = robot.subsystems;

        Trigger opModeIsActive = new Trigger(this::opModeIsActive);

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> telemetry.log().add("LEFT_BUMPER pressed"))
                .whenPressed(new DriveCommands.DecreaseSpeed(subsystems.drive));
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> telemetry.log().add("RIGHT_BUMPER pressed"))
                .whenPressed(new DriveCommands.IncreaseSpeed(subsystems.drive));

        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> telemetry.log().add("B pressed"))
                .whenPressed(new OdometryCommands.SetDriverForwardFromCurrent(subsystems.odometry));

        // X button: Attempt AprilTag localization
        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> telemetry.log().add("X pressed"))
                .whenPressed(new OdometryCommands.Localize(subsystems.odometry, telemetry));

        // Y button (held): Auto-aim turret to team's goal
        driver.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> telemetry.log().add("Y pressed - auto-aim enabled"))
                .toggleWhenPressed(new TurretCommands.AimToGoal(subsystems.turret, robot.team.goal.coord, () -> subsystems.odometry.getPose()));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> telemetry.log().add("DPAD_UP pressed"))
                .whenPressed(new OuttakeCommands.ChangeTargetRPM(subsystems.outtake, 100));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> telemetry.log().add("DPAD_DOWN pressed"))
                .whenPressed(new OuttakeCommands.ChangeTargetRPM(subsystems.outtake, -100));

        Trigger driverLeftTrigger = new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.25);
        Trigger driverRightTrigger = new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.25);

        Trigger outtakeReady = new Trigger(subsystems.outtake::isReady);

        Command intakeCommand = new IntakeCommands.In(subsystems.intake, () -> RobotConstants.Intake.intakePower);
        Command transferCommand = new IntakeCommands.In(subsystems.intake, () -> RobotConstants.Intake.transferPower);
        Command openTransfer = new TransferCommands.Open(subsystems.transfer);
        Command outtakeOn = new OuttakeCommands.On(subsystems.outtake, () -> RobotConstants.Outtake.IDLE_WHEN_END);

        if (RobotConstants.Intake.automaticBehavior)
        {
            // Intake mode: Automatic
            opModeIsActive
                    .and(driverLeftTrigger.negate())
                    .whenActive(() -> telemetry.log().add("Passive intake enabled"))
                    .whileActiveOnce(intakeCommand);
        }
        else
        {
            // Intake mode: Left trigger held, right trigger NOT pressed
            // Runs intake continuously while conditions are met
            driverLeftTrigger
                    .and(driverRightTrigger.negate())
                    .whenActive(() -> telemetry.log().add("Intake mode"))
                    .whileActiveContinuous(intakeCommand);
        }

        // Transfer mode: Both triggers pressed AND outtake is ready
        // Runs both intake and transfer while all conditions are met
        driverLeftTrigger
                .and(driverRightTrigger)
                .whileActiveContinuous(openTransfer)
                .and(outtakeReady)
                .whenActive(() -> telemetry.log().add("Outtake ready; transferring..."))
                .whileActiveOnce(transferCommand);

        // When outtake becomes not ready, close transfer for a short duration to prevent accidental shots
        outtakeReady.whenInactive(new TransferCommands.CloseForDuration(subsystems.transfer, RobotConstants.Transfer.autoPauseMs)); // 500ms

        // Outtake: Right trigger spins up flywheel
        driverRightTrigger
                .whenActive(() -> telemetry.log().add("Right trigger activated"))
                .whileActiveOnce(outtakeOn);
    }
}