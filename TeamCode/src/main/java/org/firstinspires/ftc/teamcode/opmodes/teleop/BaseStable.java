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

import java.util.concurrent.TimeUnit;
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

    private final boolean setAutoclear = RobotConstants.Telemetry.SET_AUTOCLEAR;
    private final boolean setAutoclearLogs = RobotConstants.Telemetry.SET_AUTOCLEAR_LOGS;
    private final double logAutoclearDelay = RobotConstants.Telemetry.LOG_AUTOCLEAR_DELAY;
    private final ConstantsPresets.Preset presetOption = RobotConstants.General.PRESET_OPTION;
    private final boolean enableFieldDrawing = RobotConstants.Telemetry.ENABLE_FIELD_DRAWING;
    private final boolean enableGraphOutput = RobotConstants.Telemetry.ENABLE_GRAPH_OUTPUT;
    private final boolean autoAimToGoal = RobotConstants.Turret.AUTO_AIM_TO_GOAL;
    private final boolean regressionTestingMode = RobotConstants.General.REGRESSION_TESTING_MODE;
    private final boolean autoDistanceAdjustment = RobotConstants.Outtake.AUTO_DISTANCE_ADJUSMENT;
    private final double intakePower = RobotConstants.Intake.intakePower;
    private final double transferPower = RobotConstants.Intake.transferPower;
    private final double outtakePower = RobotConstants.Intake.outtakePower;
    private final boolean intakeByDefault = RobotConstants.Intake.INTAKE_BY_DEFAULT;
    private final boolean outtakeOnByDefault = RobotConstants.Outtake.ON_BY_DEFAULT;

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
        telemetry.setAutoClear(setAutoclear);
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

        if (setAutoclearLogs && logTimer.seconds() > logAutoclearDelay)
        {
            telemetry.log().clear();
            logTimer.reset();
        }

        if (presetOption != ConstantsPresets.Preset.TESTING)
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
        telemetry.addData("Outtake State", s.outtake.getState());
        telemetry.addData("Outtake RPM", s.outtake.getRPM());
        telemetry.addData("Outtake Target", s.outtake.getTargetRPM());
        telemetry.addData("Outtake Stable", s.outtake.isStable());

        telemetry.addLine("--- Turret ---");
        telemetry.addData("At Target", s.turret.isAtTarget());
        telemetry.addData("Err/Tol", String.format("%.2f / %.2f", s.turret.bearingToTarget().getDegrees(), s.turret.getTolerance().getDegrees()));
        telemetry.addData("Heading (Abs)", s.turret.getFieldHeading(s.odometry.getFieldHeading()).toUnnormalized().toUnit(UnnormalizedAngleUnit.DEGREES));

        telemetry.update();
    }

    private void updateDashboard(Subsystems s, Pose2d pose)
    {
        if (enableFieldDrawing)
        {
            FieldDrawing.draw(
                    pose,
                    null,
                    s.turret.getFieldHeading(s.odometry.getFieldHeading()),
                    robot.team.goal.coord);
            FieldDrawing.update();
        }

        if (enableGraphOutput)
        {
            Graph.put("Turret Pos", s.turret.getRelativeUnnormalizedAngle().getDegrees());
            Graph.put("Turret Target", s.turret.getTargetAngleDegrees());
            Graph.put("Outtake RPM", s.outtake.getRPM());
            Graph.put("Outtake Target", s.outtake.getTargetRPM());
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
                .whenPressed(new OdometryCommands.LocalizeWithRumble(s.odometry, telemetry, driver.gamepad));
    }

    private void bindTurretControls(Trigger opModeActive, GamepadEx driver, Subsystems s)
    {
        // turretPose supplier no longer needed, we pass odometry directly
        Command aimCmd = new TurretCommands.AimToCoordinate(s.turret, robot.team.goal.coord, s.odometry);

        if (autoAimToGoal)
        {
            opModeActive.whileActiveContinuous(aimCmd);
        } else
        {
            driver.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(aimCmd);
        }
    }

    private void bindOuttakeControls(Trigger active, GamepadEx driver, Subsystems s)
    {
        if (regressionTestingMode)
        {
            driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(new OuttakeCommands.ChangeTargetRPM(s.outtake, 25));
            driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(new OuttakeCommands.ChangeTargetRPM(s.outtake, -25));
        }
        if (autoDistanceAdjustment)
        {
            // Update RPM dynamically based on distance to goal, optionally using FuturePose
            active.whileActiveContinuous(new OuttakeCommands.UpdateRPMBasedOnDistance(
                    s.outtake,
                    team.goal.coord,
                    s.odometry));
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
        Command intakeIn = new IntakeCommands.In(s.intake, intakePower);
        Command intakeScore = new IntakeCommands.In(s.intake, transferPower);
        Command reverseIntake = new IntakeCommands.Reverse(s.intake, outtakePower);
        Command closeTransfer = new TransferCommands.CloseOnce(s.transfer);
        Command openTransfer = new TransferCommands.OpenOnce(s.transfer);

        // --- Logic ---

        // 1. Intake Logic: (Auto OR Manual) AND Not Shooting AND Not Reversing
        Trigger shouldIntake = (intakeByDefault ? active : intakeBtn)
                .and(shootBtn.negate())
                .and(reverseBtn.negate());

        // Use whileActiveContinuous instead of whileActiveOnce to fix the command
        // scheduling bug
        // whileActiveContinuous ensures the command is properly re-scheduled each time
        // the trigger becomes active
        shouldIntake.whileActiveContinuous(intakeIn).whenActive(closeTransfer);
        reverseBtn.whileActiveContinuous(reverseIntake);

        // 2. Scoring Logic
        boolean semiAuto = intakeByDefault || outtakeOnByDefault;

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
        Trigger flywheelActive = outtakeOnByDefault ? active : shootBtn;
        flywheelActive.whileActiveContinuous(new OuttakeCommands.On(s.outtake, () -> false));
    }

    private void bindCoDriverControls(GamepadEx coDriver, Subsystems s)
    {
        // Add Co-Driver controls here
    }
}