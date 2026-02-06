package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.Timing;
import com.seattlesolvers.solverslib.util.Timing.Timer;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.controls.commands.*;
import org.firstinspires.ftc.teamcode.definitions.constants.ConstantsPresets;
import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.constants.Team;
import org.firstinspires.ftc.teamcode.definitions.hardware.RobotContext;
import org.firstinspires.ftc.teamcode.definitions.hardware.Subsystems;
import org.firstinspires.ftc.teamcode.util.dashboard.FieldDrawing;
import org.firstinspires.ftc.teamcode.util.dashboard.Graph;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.CoordinateSystem;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.motors.VelocityMotor;

import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Refactored BaseStable:
 * - Optimized telemetry for object allocation
 * - Simplified binding logic
 * - Cleaned up subsystem access
 */
public abstract class BaseStable extends CommandOpMode
{
    protected Team team;
    protected RobotContext robot;

    private final BooleanSupplier setAutoclear = RobotConstants.Telemetry.SET_AUTOCLEAR;
    private final BooleanSupplier setAutoclearLogs = RobotConstants.Telemetry.SET_AUTOCLEAR_LOGS;
    private final DoubleSupplier logAutoclearDelay = RobotConstants.Telemetry.LOG_AUTOCLEAR_DELAY;
    private final Supplier<ConstantsPresets.Preset> presetOption = RobotConstants.General.PRESET_OPTION;
    private final BooleanSupplier enableFieldDrawing = RobotConstants.Telemetry.ENABLE_FIELD_DRAWING;
    private final BooleanSupplier enableGraphOutput = RobotConstants.Telemetry.ENABLE_GRAPH_OUTPUT;
    private final BooleanSupplier autoAimToGoal = RobotConstants.Turret.AUTO_AIM_TO_GOAL;
    private final BooleanSupplier regressionTestingMode = RobotConstants.General.REGRESSION_TESTING_MODE;
    private final BooleanSupplier autoDistanceAdjustment = RobotConstants.Outtake.AUTO_DISTANCE_ADJUSMENT;
    private final DoubleSupplier intakePower = RobotConstants.Intake.intakePower;
    private final DoubleSupplier transferPower = RobotConstants.Intake.transferPower;
    private final DoubleSupplier outtakePower = RobotConstants.Intake.outtakePower;
    private final BooleanSupplier intakeByDefault = RobotConstants.Intake.INTAKE_BY_DEFAULT;
    private final BooleanSupplier outtakeOnByDefault = RobotConstants.Outtake.ON_BY_DEFAULT;

    // Timers
    private final Timer matchTimer = new Timer(120, TimeUnit.SECONDS);
    private final ElapsedTime logTimer = new ElapsedTime();
    private final ElapsedTime dashboardTimer = new ElapsedTime();
    private Timing.Stopwatch loopStopwatch;

    @Override
    public void initialize()
    {
        loopStopwatch = new Timing.Stopwatch(TimeUnit.MILLISECONDS);
        loopStopwatch.start();

        ConstantsPresets.applyPreset();

        // 1. Initialize Hardware
        robot = new RobotContext(team, hardwareMap, telemetry, gamepad1, gamepad2);

        // 2. Register Subsystems
        Subsystems s = robot.subsystems;
        register(s.drive, s.intake, s.transfer, s.turret, s.outtake, s.odometry);

        // 3. Set Defaults & Init Commands
        // s.transfer.setDefaultCommand(new TransferCommands.CloseOnce(s.transfer));

        // Drive Control Suppliers
        DoubleSupplier x = () -> gamepad1.left_stick_x;
        DoubleSupplier y = () -> -gamepad1.left_stick_y;
        DoubleSupplier rx = () -> gamepad1.right_stick_x;
        Supplier<Angle> driverHeading = s.odometry::getDriverHeading;

        s.drive.setDefaultCommand(new DriveCommands.Manuever(s.drive, x, y, rx, driverHeading));

        // 4. Bind Controls
        bindKeys();

        // 5. Finalize Telemetry
        telemetry.log().add("Initialized in " + loopStopwatch.deltaTime() + "ms");
        telemetry.setAutoClear(setAutoclear.getAsBoolean());
        telemetry.addData("Status", "Initialized for " + team);

        // Initialize Dashboard tools
        FieldDrawing.init();

        // Push initial state
        update();
    }

    @Override
    public void run()
    {
        robot.hw.clearHubCache();
        robot.hw.readBattery();
        manageMatchTimer();
        update();
        super.run();
    }

    private void manageMatchTimer()
    {
        if (!matchTimer.isTimerOn())
            matchTimer.start();
        if (matchTimer.done())
            matchTimer.pause();
    }

    protected void update()
    {
        // Cache the subsystem reference for this loop iteration
        Subsystems s = robot.subsystems;

        // Pre-fetch pose once per loop to avoid re-calculating it for every telemetry
        // line
        Pose2d loopPose = s.odometry.getPose();

        displayTelemetry(s, loopPose);

        // Throttle dashboard updates to save bandwidth (~20Hz is plenty for
        // visualization)
        if (dashboardTimer.milliseconds() > 50)
        {
            updateDashboard(s, loopPose);
            dashboardTimer.reset();
        }
    }

    /**
     * Optimized telemetry display using cached data
     */
    protected void displayTelemetry(Subsystems s, Pose2d pose)
    {
        long loopTime = loopStopwatch.deltaTime();

        telemetry.addData("Loop Time", loopTime + "ms");
        Graph.put("Loop Time (ms)", loopTime);

        if (setAutoclearLogs.getAsBoolean() && logTimer.seconds() > logAutoclearDelay.getAsDouble())
        {
            telemetry.log().clear();
            logTimer.reset();
        }

        if (presetOption.get() != ConstantsPresets.Preset.TESTING)
        {
            telemetry.update();
            return;
        }

        // --- Testing Telemetry ---
        telemetry.addData("Team", team);
        telemetry.addData("Dist. to Goal", pose.coord.distanceTo(team.goal.coord).toUnit(DistanceUnit.INCH));

        telemetry.addLine("--- Odometry ---");
        telemetry.addData("Coord (Pedro)", pose.toCoordinateSystem(CoordinateSystem.DECODE_PEDROPATH));
        telemetry.addData("Coord (FTC)", pose.toCoordinateSystem(CoordinateSystem.DECODE_FTC));
        telemetry.addData("Heading (Field)", s.odometry.getFieldHeading().toSystem(CoordinateSystem.DECODE_PEDROPATH));
        telemetry.addData("Heading (Driver)", s.odometry.getDriverHeading());
        telemetry.addData("IMU Yaw", s.odometry.getIMUYaw());

        telemetry.addLine("--- Hardware ---");
        telemetry.addData("Drive Speed", s.drive.getSpeed());
        telemetry.addData("Intake RPM", s.intake.getRPM());
        telemetry.addData("Outtake", s.outtake.toString());
        telemetry.addData("Outtake Stable", s.outtake.isStable());

        telemetry.addLine("--- Turret ---");
        telemetry.addData("At Target", s.turret.isAtTarget());
        telemetry.addData("Err/Tol", String.format("%.2f / %.2f", s.turret.bearingToTarget().getDegrees(), s.turret.getTolerance().getDegrees()));
        telemetry.addData("Heading (Abs)", s.turret.getFieldHeading(s.odometry.getFieldHeading()).toUnnormalized().toUnit(UnnormalizedAngleUnit.DEGREES));

        telemetry.update();
    }

    private void updateDashboard(Subsystems s, Pose2d pose)
    {
        if (enableFieldDrawing.getAsBoolean())
        {
            FieldDrawing.draw(
                    pose,
                    null,
                    s.turret.getFieldHeading(s.odometry.getFieldHeading()),
                    robot.team.goal.coord);
            FieldDrawing.update();
        }

        if (enableGraphOutput.getAsBoolean())
        {
            Graph.put("Outtake (Motor RPM)", s.outtake.getMotorRPM());
            Graph.put("Outtake (Flywheel RPM)", s.outtake.getFlywheelRPM());
            Graph.put("Outtake (Flywheel RPM Target)", robot.hw.outtake.getOutputTargetRPM());
            Graph.put("Outtake (Motor Target RPM)", robot.hw.outtake.getMotorTargetRPM());
            Graph.put("Outtake (Power)", robot.hw.outtake.getPower());
            Graph.put("Outtake Ready", s.outtake.isStable() ? 1 : 0);

            Graph.put("Turret (Degrees)", s.turret.getRelativeUnnormalizedAngle().getDegrees());
            Graph.put("Turret (Target Degrees)", s.turret.getTargetAngleDegrees());
            Graph.put("Turret (Bearing Degrees)", s.turret.bearingToTarget().getDegrees());

            Graph.put("Intake (RPM)", s.intake.getRPM());
            Graph.put("Intake (Power)", robot.hw.intake.getPower());

            Graph.put("Battery (Voltage)", robot.hw.battery.getVoltage());

            Graph.update();
        }
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
        Subsystems s = robot.subsystems;

        Trigger opModeActive = new Trigger(this::opModeIsActive);

        // Controls
        bindDriveControls(driver, s);
        bindTurretControls(opModeActive, driver, s);
        bindOuttakeControls(opModeActive, driver, s);
        bindIntakeAndTransferLogic(opModeActive, driver, coDriver, s);
        bindCoDriverControls(coDriver, s);
    }

    private void bindDriveControls(GamepadEx driver, Subsystems s)
    {
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new DriveCommands.DecreaseSpeed(s.drive));
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new DriveCommands.IncreaseSpeed(s.drive));
        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new OdometryCommands.SetDriverForwardFromCurrent(s.odometry));
        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new OdometryCommands.LocalizeWithDebugTelemetry(s.odometry, telemetry));
    }

    private void bindTurretControls(Trigger opModeActive, GamepadEx driver, Subsystems s)
    {
        Supplier<Pose2d> turretPose = s.odometry::getPose;
        Command aimCmd = new TurretCommands.AimToCoordinate(s.turret, robot.team.goal.coord, turretPose);

        if (autoAimToGoal.getAsBoolean())
        {
            opModeActive.whileActiveContinuous(aimCmd);
        }
        else
        {
            driver.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(aimCmd);
        }
    }

    private void bindOuttakeControls(Trigger active, GamepadEx driver, Subsystems s)
    {
        if (regressionTestingMode.getAsBoolean())
        {
            driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(new OuttakeCommands.ChangeTargetRPM(s.outtake, 50));
            driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(new OuttakeCommands.ChangeTargetRPM(s.outtake, -50));
        }
        if (autoDistanceAdjustment.getAsBoolean())
        {
            // Update RPM dynamically based on distance to goal
            active.whileActiveContinuous(new OuttakeCommands.UpdateRPMBasedOnDistance(
                    s.outtake,
                    () -> s.odometry.getPose().distanceTo(team.goal.coord)));
        }
    }

    private void bindIntakeAndTransferLogic(Trigger active, GamepadEx driver, GamepadEx coDriver, Subsystems s)
    {
        // --- Triggers ---
        Trigger intakeBtn = new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1);
        Trigger shootBtn = new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1);
        Trigger reverseBtn = new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_LEFT));

        Trigger systemsReady = new Trigger(s.turret::isAtTarget).and(new Trigger(s.outtake::isStable));

        // --- Commands ---
        Command intakeIn = new IntakeCommands.In(s.intake, intakePower.getAsDouble());
        Command intakeScore = new IntakeCommands.In(s.intake, transferPower.getAsDouble());
        Command reverseIntake = new IntakeCommands.Reverse(s.intake, outtakePower.getAsDouble());
        Command closeTransfer = new TransferCommands.CloseOnce(s.transfer);
        Command openTransfer = new TransferCommands.OpenOnce(s.transfer);

        // --- Logic ---

        // 1. Intake Logic: (Auto OR Manual) AND Not Shooting AND Not Reversing
        Trigger shouldIntake = (intakeByDefault.getAsBoolean() ? active : intakeBtn)
                .and(shootBtn.negate())
                .and(reverseBtn.negate());

        // Use whileActiveContinuous instead of whileActiveOnce to fix the command
        // scheduling bug
        // whileActiveContinuous ensures the command is properly re-scheduled each time
        // the trigger becomes active
        shouldIntake.whileActiveContinuous(intakeIn).whenActive(closeTransfer);
        reverseBtn.whileActiveContinuous(reverseIntake);

        // 2. Scoring Logic
        boolean semiAuto = intakeByDefault.getAsBoolean() || outtakeOnByDefault.getAsBoolean();

        // "Can Score" = User wants to shoot + Systems are ready ( + Manual intake hold
        // if not semi-auto)
        Trigger canScore = semiAuto
                ? shootBtn.and(systemsReady)
                : intakeBtn.and(shootBtn).and(systemsReady);

        // "Keep Scoring" = Hysteresis to ensure we finish the shot even if turret
        // jitters slightly
        Trigger keepScoring = semiAuto
                ? shootBtn
                : intakeBtn.and(shootBtn); // Ideally this might need systemsReady too, but kept close to original logic

        // Action: When ready, run intake (feeder) and open transfer
        canScore.whenActive(intakeScore).whenActive(openTransfer);

        // Exit: When we stop holding shoot (or intake), cancel the feeding commands
        keepScoring.negate().cancelWhenActive(intakeScore).cancelWhenActive(openTransfer);

        // 3. Flywheel Logic
        Trigger flywheelActive = outtakeOnByDefault.getAsBoolean() ? active : shootBtn;
        flywheelActive.whileActiveContinuous(new OuttakeCommands.On(s.outtake, () -> false));
    }

    private void bindCoDriverControls(GamepadEx coDriver, Subsystems s)
    {
        // Add Co-Driver controls here
    }
}