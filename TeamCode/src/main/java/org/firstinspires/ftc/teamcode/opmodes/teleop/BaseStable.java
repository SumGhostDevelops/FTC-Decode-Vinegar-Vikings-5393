package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.Timing.Timer;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.controls.commands.DriveCommands;
import org.firstinspires.ftc.teamcode.controls.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.controls.commands.OdometryCommands;
import org.firstinspires.ftc.teamcode.controls.commands.OuttakeCommands;
import org.firstinspires.ftc.teamcode.controls.commands.TransferCommands;
import org.firstinspires.ftc.teamcode.controls.commands.TurretCommands;
import org.firstinspires.ftc.teamcode.definitions.constants.ConstantsPresets;
import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.hardware.Subsystems;
import org.firstinspires.ftc.teamcode.definitions.constants.Team;
import org.firstinspires.ftc.teamcode.definitions.hardware.RobotContext;
import org.firstinspires.ftc.teamcode.util.dashboard.FieldDrawing;
import org.firstinspires.ftc.teamcode.definitions.localization.CornersCoordinates;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;

import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * This class is for implementing and testing new but unstable features
 */
public abstract class BaseStable extends CommandOpMode
{
    protected Team team;
    protected RobotContext robot;
    private final Timer timer = new Timer(120, TimeUnit.SECONDS);

    @Override
    public void initialize()
    {
        ConstantsPresets.applyPreset();
        robot = new RobotContext(team, hardwareMap, telemetry, gamepad1, gamepad2);

        // Initialize Panels Field with FTC Standard coordinates
        FieldDrawing.init();

        register(robot.subsystems.drive, robot.subsystems.intake, robot.subsystems.transfer, robot.subsystems.turret, robot.subsystems.outtake, robot.subsystems.odometry);

        DoubleSupplier x = () -> gamepad1.left_stick_x; // Counteract imperfect strafing
        DoubleSupplier y = () -> -gamepad1.left_stick_y; // Y is inverted
        DoubleSupplier rx = () -> gamepad1.right_stick_x;
        Supplier<Angle> driverHeading = () -> robot.subsystems.odometry.getDriverHeading();

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
        if (RobotConstants.Telemetry.SET_AUTOCLEAR) telemetry.clear();
        if (RobotConstants.Telemetry.SET_AUTOCLEAR_LOGS) telemetry.log().clear();

        switch (RobotConstants.General.PRESET_OPTION)
        {
            case DEFAULT:
            case COMPETITION:
                telemetry.addLine("--- Co Driver Keybinds ---");
                telemetry.addLine("DPAD UP: BLUE GOAL");
                telemetry.addLine("DPAD RIGHT: RED GOAL");
                telemetry.addLine("DPAD DOWN: BLUE LOADING ZONE");
                telemetry.addLine("DPAD LEFT: RED LOADING ZONE");
                telemetry.addLine("RIGHT BUMPER: SMALL/FAR TRIANGLE");
                telemetry.addLine("------");
                telemetry.addData("Team", team);
                telemetry.addData("Remaining Time", timer.remainingTime() + "/120");
                telemetry.addData("Distance to Goal (inches)", robot.subsystems.odometry.getFieldCoord().distanceTo(team.goal.coord).toUnit(DistanceUnit.INCH));
                telemetry.addLine("--- Odometry ---");
                telemetry.addData("Absolute Heading (deg)", robot.subsystems.odometry.getAngle().getUnsignedAngle(AngleUnit.DEGREES));
                telemetry.addLine("--- Drive ---");
                telemetry.addData("Speed (power)", robot.subsystems.drive.getSpeed());
                telemetry.addLine("--- Outtake ---");
                telemetry.addData("Target RPM", robot.subsystems.outtake.getTargetRPM());
                telemetry.addData("True RPM", robot.subsystems.outtake.getRPM());
                telemetry.addData("Is Stable", robot.subsystems.outtake.isReady());
                telemetry.addLine("--- Turret ---");
                telemetry.addData("Relative Heading (deg)", robot.subsystems.turret.getRelativeAngle().getUnsignedAngle(AngleUnit.DEGREES));
                telemetry.addData("Absolute Heading (deg)", robot.subsystems.turret.getAbsoluteAngle(robot.subsystems.odometry.getAngle()).getUnsignedAngle(AngleUnit.DEGREES));
            case TESTING:
                telemetry.addLine("--- Co Driver Keybinds ---");
                telemetry.addLine("DPAD UP: BLUE GOAL");
                telemetry.addLine("DPAD RIGHT: RED GOAL");
                telemetry.addLine("DPAD DOWN: BLUE LOADING ZONE");
                telemetry.addLine("DPAD LEFT: RED LOADING ZONE");
                telemetry.addLine("RIGHT BUMPER: SMALL/FAR TRIANGLE");
                telemetry.addLine("------");
                telemetry.addData("Team", team);
                telemetry.addData("Distance to Goal", robot.subsystems.odometry.getFieldCoord().distanceTo(team.goal.coord).toUnit(DistanceUnit.INCH));
                telemetry.addLine("--- Odometry ---");
                telemetry.addData("Relative Heading (deg)", robot.subsystems.odometry.getDriverHeading().getUnsignedAngle(AngleUnit.DEGREES));
                telemetry.addData("Absolute Heading (deg)", robot.subsystems.odometry.getAngle().getUnsignedAngle(AngleUnit.DEGREES));
                telemetry.addLine("--- Drive ---");
                telemetry.addData("Speed (power)", robot.subsystems.drive.getSpeed());
                telemetry.addLine("--- Outtake ---");
                telemetry.addData("Target RPM", robot.subsystems.outtake.getTargetRPM());
                telemetry.addData("RPM", robot.subsystems.outtake.getRPM());
                telemetry.addData("Acceleration", robot.subsystems.outtake.getRPMAcceleration());
                telemetry.addData("Is Stable", robot.subsystems.outtake.isReady());
                telemetry.addLine("--- Turret ---");
                telemetry.addData("Relative Heading (deg)", robot.subsystems.turret.getRelativeAngle().getUnsignedAngle(AngleUnit.DEGREES));
                telemetry.addData("Absolute Heading (deg)", robot.subsystems.turret.getAbsoluteAngle(robot.subsystems.odometry.getAngle()).getUnsignedAngle(AngleUnit.DEGREES));
                telemetry.addData("Bearing to Target", robot.subsystems.turret.bearingToTarget());
                telemetry.addData("Strafe Encoder", robot.hw.dwStrf.getPosition());
                telemetry.addData("Fwd Encoder", robot.hw.dwFwd.getPosition());
                telemetry.addData("Raw Data", robot.subsystems.odometry.getRawAprilTagData());
        }
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
        if (!timer.isTimerOn())
        {
            timer.start();
        }
        if (timer.done())
        {
            timer.pause();
        }
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
        GamepadEx coDriver = robot.gamepads.coDriver;
        Subsystems subsystems = robot.subsystems;

        Trigger opModeIsActive = new Trigger(this::opModeIsActive);
        Trigger outtakeReady = new Trigger(subsystems.outtake::isReady);

        Trigger driverLeftTrigger = new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.25);
        Trigger driverRightTrigger = new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.25);

        Command intakeIntake = new IntakeCommands.In(subsystems.intake, () -> RobotConstants.Intake.intakeRPM);
        Command intakeTransfer = new IntakeCommands.In(subsystems.intake, () -> RobotConstants.Intake.transferPassRPM);
        Command transferOpen = new TransferCommands.OpenTransfer(subsystems.transfer);
        Command transferShoot = new TransferCommands.ShootingTransfer(subsystems.transfer);
        Command transferCloseIntake = new TransferCommands.CloseIntake(subsystems.transfer);
        Command transferCloseTransfer = new TransferCommands.CloseTransfer(subsystems.transfer);
        Command outtakeOn = new OuttakeCommands.On(subsystems.outtake, () -> RobotConstants.Outtake.IDLE_BY_DEFAULT);

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new DriveCommands.DecreaseSpeed(subsystems.drive));
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new DriveCommands.IncreaseSpeed(subsystems.drive));

        if (RobotConstants.General.PRESET_OPTION.equals(ConstantsPresets.Preset.TESTING))
        {
            driver.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(subsystems.transfer::close, subsystems.transfer::open);
        }
        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new OdometryCommands.SetDriverForwardFromCurrent(subsystems.odometry));

        // X button: Attempt AprilTag localization

        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new OdometryCommands.Localize(subsystems.odometry, telemetry));

        if (RobotConstants.Turret.autoAimToGoal)
        {
            opModeIsActive.whileActiveContinuous(new TurretCommands.AimToGoal(subsystems.turret, robot.team.goal.coord, () -> subsystems.odometry.getPose()));
        }
        else
        {
            // Y button (held): Auto-aim turret to team's goal
            driver.getGamepadButton(GamepadKeys.Button.Y)
                    .toggleWhenPressed(new TurretCommands.AimToGoal(subsystems.turret, robot.team.goal.coord, () -> subsystems.odometry.getPose()));
        }

        if (RobotConstants.General.REGRESSION_TESTING_MODE)
        {
            driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                    .whileHeld(new OuttakeCommands.ChangeTargetRPM(subsystems.outtake, 25));
            driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                    .whileHeld(new OuttakeCommands.ChangeTargetRPM(subsystems.outtake, -25));
        }
        else
        {
            opModeIsActive.whileActiveContinuous(new OuttakeCommands.UpdateRPMBasedOnDistance(subsystems.outtake, () -> robot.subsystems.odometry.getFieldCoord().distanceTo(team.goal.coord)));
        }

        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whileHeld(new IntakeCommands.Out(subsystems.intake, () -> RobotConstants.Intake.outtakePower));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whileHeld(new TransferCommands.CloseIntake(subsystems.transfer));

        if (RobotConstants.Intake.INTAKE_BY_DEFAULT)
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
                .whileActiveOnce(transferCloseTransfer);

        driverLeftTrigger
                .and(driverRightTrigger)
                //.and(outtakeReady)
                //.whileActiveOnce(transferOpen)
                .whileActiveContinuous(intakeTransfer)
                .whileActiveOnce(transferOpen);


        // Outtake: Right trigger spins up flywheel
        // While the right trigger is held down, turn the intake on
        driverRightTrigger
                .whileActiveOnce(outtakeOn);

        // Co driver localization
        coDriver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> subsystems.odometry.updateReferencePose(CornersCoordinates.BLUE_GOAL));
        coDriver.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> subsystems.odometry.updateReferencePose(CornersCoordinates.RED_GOAL));
        coDriver.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> subsystems.odometry.updateReferencePose(CornersCoordinates.BLUE_LOADING_ZONE));
        coDriver.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> subsystems.odometry.updateReferencePose(CornersCoordinates.RED_LOADING_ZONE));
        coDriver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> subsystems.odometry.updateReferencePose(CornersCoordinates.SMALL_TRIANGLE));
    }
}