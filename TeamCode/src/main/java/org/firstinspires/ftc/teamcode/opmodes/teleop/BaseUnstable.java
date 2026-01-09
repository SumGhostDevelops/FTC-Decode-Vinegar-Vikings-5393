package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.seattlesolvers.solverslib.command.Command;
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

        DoubleSupplier x = () -> gamepad1.left_stick_x; // Counteract imperfect strafing
        DoubleSupplier y = () -> -gamepad1.left_stick_y; // Y is inverted

        DoubleSupplier rx = () -> gamepad1.right_stick_x;
        Supplier<Angle> driverHeading = () -> robot.subsystems.odometry.getYaw(AngleUnit.DEGREES);

        robot.subsystems.drive.setDefaultCommand(new DriveCommands.Manuever(robot.subsystems.drive, x, y, rx, driverHeading));

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
        telemetry.clear();
        telemetry.addData("Team", team);
        telemetry.addData("Distance to Goal (meters)", robot.subsystems.odometry.getFieldCoord().distanceTo(team.goal.coord).toUnit(DistanceUnit.METER).magnitude);
        telemetry.addData("Distance to Goal (inches)", robot.subsystems.odometry.getFieldCoord().distanceTo(team.goal.coord).toUnit(DistanceUnit.INCH).magnitude);
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
        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
    }

    /**
     * For updating important systems, like the Telemetry or Localization
     */
    protected void update()
    {
        displayTelemetry();
        telemetry.update();

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
                .whenPressed(new DriveCommands.DecreaseSpeed(subsystems.drive));
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new DriveCommands.IncreaseSpeed(subsystems.drive));

        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new OdometryCommands.SetDriverForwardFromCurrent(subsystems.odometry));

        // X button: Attempt AprilTag localization
        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new OdometryCommands.Localize(subsystems.odometry, telemetry));

        // Y button (held): Auto-aim turret to team's goal
        driver.getGamepadButton(GamepadKeys.Button.Y)
                .toggleWhenPressed(new TurretCommands.AimToGoal(subsystems.turret, robot.team.goal.coord, () -> subsystems.odometry.getPose()));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new OuttakeCommands.ChangeTargetRPM(subsystems.outtake, 25));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new OuttakeCommands.ChangeTargetRPM(subsystems.outtake, -25));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whileActiveOnce(new IntakeCommands.Out(subsystems.intake, () -> RobotConstants.Intake.outtakePower));

        Trigger driverLeftTrigger = new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.25);
        Trigger driverRightTrigger = new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.25);

        Trigger outtakeReady = new Trigger(subsystems.outtake::isReady);

        Command intakeIntake = new IntakeCommands.In(subsystems.intake, () -> RobotConstants.Intake.intakePower);
        Command intakeTransfer = new IntakeCommands.In(subsystems.intake, () -> RobotConstants.Intake.transferPassPower);
        Command transferOpen = new TransferCommands.OpenTransfer(subsystems.transfer);
        Command outtakeOn = new OuttakeCommands.On(subsystems.outtake, () -> RobotConstants.Outtake.IDLE_WHEN_END);

        if (RobotConstants.Intake.automaticBehavior)
        {
            // Intake mode: Automatic
            // If the op mode is active, and we are not holding down the left trigger, turn the intake on
            opModeIsActive
                .and(driverLeftTrigger.negate())
                .whileActiveOnce(intakeIntake);
        }
        else
        {
            // Intake mode: Left trigger held, right trigger NOT pressed
            // Runs intake continuously while conditions are met
            // While we are holding down the left trigger, and we are not holding down the right trigger, turn the intake on
            driverLeftTrigger
                .and(driverRightTrigger.negate())
                .whileActiveOnce(intakeIntake);
        }

        // Intake Mode: Set transfer to the intake/blocking mode
        // If we are holding down the left trigger, and we are not holding down the right trigger, set the transfer to a blocking state
        driverLeftTrigger
                .and(driverRightTrigger.negate())
                .whileActiveOnce(new TransferCommands.CloseIntake(subsystems.transfer));

        // Transfer mode: Both triggers pressed AND outtake is ready
        // Runs both intake and transfer while all conditions are met
        // If the left trigger, right trigger, and outtake are ready, open the transfer and set the intake to a transfer mode
        driverLeftTrigger
                .and(driverRightTrigger)
                .whileActiveOnce(intakeTransfer)
                //.and(outtakeReady)
                .whileActiveOnce(transferOpen);

        // When outtake becomes not ready, close transfer for a short duration and run intake in reverse to prevent accidental shots
        // When the outtake goes from ready -> not ready, forcibly turn the intake in reverse and put the transfer in a block-allow state
        //outtakeReady.whenInactive(new IntakeCommands.TransferPreventForDuration(subsystems.intake, RobotConstants.Intake.transferPreventPower, RobotConstants.Intake.transferPreventDurationMs), true).whenInactive(new TransferCommands.CloseTransferForDuration(subsystems.transfer, RobotConstants.Transfer.autoCloseMs), true);

        // Outtake: Right trigger spins up flywheel
        // While the right trigger is held down, turn the intake on
        driverRightTrigger
                .whileActiveOnce(outtakeOn);
    }
}