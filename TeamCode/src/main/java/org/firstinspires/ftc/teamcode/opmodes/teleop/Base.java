package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.Timing;
import com.seattlesolvers.solverslib.util.Timing.Timer;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.controls.commands.*;
import org.firstinspires.ftc.teamcode.definitions.constants.ConstantsPresets;
import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.constants.Team;
import org.firstinspires.ftc.teamcode.definitions.hardware.RobotContext;
import org.firstinspires.ftc.teamcode.definitions.hardware.Subsystems;
import org.firstinspires.ftc.teamcode.definitions.localization.Hardpoints;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.odometry.Odometry;
import org.firstinspires.ftc.teamcode.util.dashboard.FieldDrawing;
import org.firstinspires.ftc.teamcode.util.dashboard.Graph;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.CoordinateSystem;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
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
public abstract class Base extends CommandOpMode
{
    protected Team team;
    protected RobotContext robot;

    private final BooleanSupplier setAutoclear = () -> RobotConstants.Telemetry.SET_AUTOCLEAR;
    private final BooleanSupplier setAutoclearLogs = () -> RobotConstants.Telemetry.SET_AUTOCLEAR_LOGS;
    private final DoubleSupplier logAutoclearDelay = () -> RobotConstants.Telemetry.LOG_AUTOCLEAR_DELAY;
    private final Supplier<ConstantsPresets.Preset> presetOption = () -> RobotConstants.General.PRESET_OPTION;
    private final BooleanSupplier enableFieldDrawing = () -> RobotConstants.Telemetry.ENABLE_FIELD_DRAWING;
    private final BooleanSupplier enableGraphOutput = () -> RobotConstants.Telemetry.ENABLE_GRAPH_OUTPUT;
    private final BooleanSupplier autoAimToGoal = () -> RobotConstants.Turret.AUTO_AIM_TO_GOAL;
    private final BooleanSupplier regressionTestingMode = () -> RobotConstants.General.REGRESSION_TESTING_MODE;
    private final DoubleSupplier intakePower = () -> RobotConstants.Intake.intakePower;
    private final DoubleSupplier minimumTransferPower = () -> RobotConstants.Intake.minimumTransferPower;
    private final DoubleSupplier maximumTransferPower = () -> RobotConstants.Intake.maximumTransferPower;
    private final DoubleSupplier outtakePower = () -> RobotConstants.Intake.outtakePower;
    private final BooleanSupplier intakeByDefault = () -> RobotConstants.Intake.INTAKE_BY_DEFAULT;
    private final BooleanSupplier outtakeOnByDefault = () -> RobotConstants.Outtake.ON_BY_DEFAULT;

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

        RobotConstants.Odometry.RESET_PINPOINT = false;

        // 1. Initialize Hardware
        robot = new RobotContext(team, hardwareMap, telemetry, gamepad1, gamepad2);

        // 2. Register Subsystems
        Subsystems s = robot.subsystems;

        Pose2d savedPose = getSavedAutonomousPose(RobotConstants.Autonomous.AUTONOMOUS_POSE_TIMEOUT_MS);

        if (savedPose != null && s.odometry != null)
        {
            s.odometry.setReferencePose(savedPose);
        }

        // Register only non-null subsystems to prevent CommandScheduler NPE
        if (s.drive != null)    register(s.drive);
        if (s.intake != null)   register(s.intake);
        if (s.transfer != null) register(s.transfer);
        if (s.turret != null)   register(s.turret);
        if (s.outtake != null)  register(s.outtake);
        if (s.odometry != null) register(s.odometry);

        // 3. Set Defaults & Init Commands
        // s.transfer.setDefaultCommand(new TransferCommands.CloseOnce(s.transfer));

        // Drive Control Suppliers
        DoubleSupplier x = () -> gamepad1.left_stick_x;
        DoubleSupplier y = () -> -gamepad1.left_stick_y;
        DoubleSupplier rx = () -> gamepad1.right_stick_x;

        if (s.drive != null && s.odometry != null)
        {
            Supplier<Angle> driverHeading = s.odometry::getDriverHeading;
            s.drive.setDefaultCommand(new DriveCommands.Manuever(s.drive, x, y, rx, driverHeading));
        }

        // 4. Bind Controls
        bindKeys();

        // 5. Finalize Telemetry
        telemetry.setAutoClear(setAutoclear.getAsBoolean());

        // Initialize Dashboard tools
        FieldDrawing.init();

        final long timeToInit = loopStopwatch.deltaTime();

        do
        {
            robot.hw.clearHubCache();
            robot.hw.readBattery();

            // Continually attempt to load/configure/calibrate Pinpoint until ready
            if (s.odometry != null)
                s.odometry.periodic();

            telemetry.addData("Status", "Initialized for " + team);
            telemetry.addLine("Initialized in " + timeToInit + "ms");
            telemetry.addLine(savedPose != null ? "Loaded Autonomous Pose" : "No Autonomous Pose Loaded");
            telemetry.addData("Reference Pose Set", s.odometry != null ? s.odometry.referencePoseWasSet() : "N/A");
            telemetry.addData("Pose", s.odometry.getPose());
            telemetry.addData("Pinpoint Status", robot.hw.pinpoint != null ? robot.hw.pinpoint.getDeviceStatus() : "NULL");
            telemetry.update();
        } while (!isStarted() && opModeInInit());
    }

    @Override
    public void run()
    {
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
        // Reset the cache and battery
        robot.hw.clearHubCache();
        robot.hw.readBattery();

        // Cache the subsystem reference for this loop iteration
        Subsystems s = robot.subsystems;

        // Pre-fetch pose once per loop to avoid re-calculating it for every telemetry line
        Pose2d loopPose = s.odometry != null ? s.odometry.getPose() : null;

        displayTelemetry(s, loopPose);

        // Throttle dashboard updates to save bandwidth (~20Hz is plenty for visualization)
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

        if (presetOption.get() == ConstantsPresets.Preset.COMPETITION)
        {
            telemetry.update();
            return;
        }

        // --- Testing Telemetry ---
        telemetry.addData("Team", team);

        if (pose != null)
        {
            telemetry.addData("Dist. to Goal", pose.distanceTo(team.goalFromClose.coord).toUnit(DistanceUnit.INCH));
            telemetry.addData("Turret Dist. to Goal", s.turret.getTurretPose(pose).distanceTo(getGoal()).toUnit(DistanceUnit.INCH));

            telemetry.addLine("--- Odometry ---");
            telemetry.addData("Coord (Pedro)", pose.toCoordinateSystem(CoordinateSystem.DECODE_PEDROPATH));
        }

        if (s.odometry != null)
        {
            telemetry.addData("Heading (Field)", s.odometry.getFieldHeading().toCoordinateSystem(CoordinateSystem.DECODE_PEDROPATH));
            telemetry.addData("IMU Yaw", s.odometry.getIMUYaw());
        }

        telemetry.addLine("--- Hardware ---");
        if (s.drive != null)    telemetry.addData("Drive Speed", s.drive.getSpeed());
        if (s.intake != null)   telemetry.addData("Intake RPM", s.intake.getRPM());
        if (s.outtake != null)
        {
            telemetry.addData("Outtake", s.outtake.toString());
            telemetry.addData("Outtake Stable", s.outtake.isStable());
        }

        if (s.turret != null)
        {
            telemetry.addLine("--- Turret ---");
            telemetry.addData("At Target", s.turret.isAtTarget());
            telemetry.addData("Err/Tol", String.format("%.2f / %.2f", s.turret.bearingToTarget().getDegrees(), s.turret.getTolerance().getDegrees()));
            if (s.odometry != null)
                telemetry.addData("Heading (Abs)", s.turret.getFieldHeading(s.odometry.getFieldHeading()).toUnnormalized().toUnit(UnnormalizedAngleUnit.DEGREES));
        }

        telemetry.update();
    }

    private void updateDashboard(Subsystems s, Pose2d pose)
    {
        if (enableFieldDrawing.getAsBoolean() && pose != null && s.turret != null && s.odometry != null)
        {
            FieldDrawing.draw(
                    () -> pose,
                    () -> null,
                    () -> s.turret.getFieldHeading(s.odometry.getFieldHeading()),
                    this::getGoal);
            FieldDrawing.update();
        }

        if (enableGraphOutput.getAsBoolean())
        {
            if (s.outtake != null && robot.hw.outtake != null)
            {
                Graph.put("Outtake (Motor RPM)", s.outtake.getMotorRPM());
                Graph.put("Outtake (Motor RPM^2", s.outtake.getMotorRPMAcceleration());
                Graph.put("Outtake (Flywheel RPM)", s.outtake.getFlywheelRPM());
                Graph.put("Outtake (Flywheel RPM^2", s.outtake.getFlywheelRPMAcceleration());
                Graph.put("Outtake (Flywheel RPM Target)", robot.hw.outtake.getOutputTargetRPM());
                Graph.put("Outtake (Motor Target RPM)", robot.hw.outtake.getMotorTargetRPM());
                Graph.put("Outtake (Residual RPM)", robot.hw.outtake.getMotorRPM() - robot.hw.outtake.getMotorTargetRPM());
                Graph.put("Outtake (Power)", robot.hw.outtake.getPower());
                Graph.put("Outtake Ready", s.outtake.isStable() ? 1 : 0);
            }

            if (s.turret != null && robot.hw.turret != null)
            {
                Graph.put("Turret (Degrees)", s.turret.getRelativeUnnormalizedAngle().getDegrees());
                Graph.put("Turret (Target Degrees)", s.turret.getTargetAngleDegrees());
                Graph.put("Turret (Bearing Degrees)", s.turret.bearingToTarget().getDegrees());
                Graph.put("Turret (Power)", robot.hw.turret.getPower());
            }

            if (s.intake != null && robot.hw.intake != null)
            {
                Graph.put("Intake (RPM)", s.intake.getRPM());
                Graph.put("Intake (Target RPM)", robot.hw.intake.getMotorTargetRPM());
                Graph.put("Intake (Power)", robot.hw.intake.getPower());
            }

            if (robot.hw.transfer != null) Graph.put("Transfer (Angle)", robot.hw.transfer.get());
            if (robot.hw.battery != null)  Graph.put("Battery (Voltage)", robot.hw.battery.getVoltage());

            Graph.update();
        }
    }

    @Override
    public void end()
    {
        if (robot.subsystems.odometry != null)
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

        // Bind corner localization
        if (RobotConstants.Odometry.ENABLE_CORNER_LOCALIZATION && s.odometry != null)
        {
            GamepadButton driverB = driver.getGamepadButton(GamepadKeys.Button.B);
            GamepadButton driverA = driver.getGamepadButton(GamepadKeys.Button.A);

            switch (team)
            {
                case RED:
                    driverB.whenPressed(new OdometryCommands.SetReferencePose(s.odometry, Hardpoints.Poses.RED_GOAL));
                    driverA.whenPressed(new OdometryCommands.SetReferencePose(s.odometry, Hardpoints.Poses.RED_LOADING_ZONE));
                    break;
                case BLUE:
                    driverB.whenPressed(new OdometryCommands.SetReferencePose(s.odometry, Hardpoints.Poses.BLUE_GOAL));
                    driverA.whenPressed(new OdometryCommands.SetReferencePose(s.odometry, Hardpoints.Poses.BLUE_LOADING_ZONE));
            }
        }
    }

    private void bindDriveControls(GamepadEx driver, Subsystems s)
    {
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new DriveCommands.DecreaseSpeed(s.drive));
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new DriveCommands.IncreaseSpeed(s.drive));
        if (s.odometry != null)
        {
            /*
            driver.getGamepadButton(GamepadKeys.Button.B)
                    .whenPressed(new OdometryCommands.SetDriverForwardFromCurrent(s.odometry));
            driver.getGamepadButton(GamepadKeys.Button.X)
                    .whenPressed(new OdometryCommands.LocalizeWithDebugTelemetry(s.odometry, telemetry));

             */
        }
    }

    // Turret manual aiming state
    private boolean turretEnabled = true;

    private void bindTurretControls(Trigger opModeActive, GamepadEx driver, Subsystems s)
    {
        if (s.turret == null || s.odometry == null) return;

        Supplier<Pose2d> turretPose = s.odometry::getPose;
        Command aimCmd = new TurretCommands.AimToCoordinate(s.turret, this::getGoal, turretPose);
        Command off = new InstantCommand(() -> s.turret.setState(Turret.State.OFF));
        Command aimCenter = new InstantCommand(s.turret::center);

        Trigger intakeBtn = new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1);
        Trigger shootBtn = new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1);

        // Scoring Logic
        boolean semiAuto = intakeByDefault.getAsBoolean() || outtakeOnByDefault.getAsBoolean();

        // "Intend Shoot" = User wants to shoot ( + Manual intake hold if not semi-auto)
        Trigger intendShoot = semiAuto
                ? shootBtn
                : intakeBtn.and(shootBtn);

        if (autoAimToGoal.getAsBoolean())
        {
            if (RobotConstants.Turret.TARGET_ONLY_WHEN_INTENDING_TO_SHOOT)
            {
                opModeActive.and(intendShoot).toggleWhenActive(aimCmd, aimCenter);
            }
            else
            {
                opModeActive.whileActiveContinuous(aimCmd);
            }
        }
        else
        {
            // Manual aiming mode: Y toggles turret on/off
            Trigger turretEnabledTrigger = new Trigger(() -> turretEnabled);

            // Toggle turret enabled state with Y button
            driver.getGamepadButton(GamepadKeys.Button.Y)
                    .whenPressed(new InstantCommand(() -> turretEnabled = !turretEnabled));

            if (RobotConstants.Turret.TARGET_ONLY_WHEN_INTENDING_TO_SHOOT)
            {
                // When turret is enabled AND intending to shoot -> aim to target
                // When turret is enabled AND NOT intending to shoot -> aim forward
                // When turret is disabled -> turn off
                turretEnabledTrigger.and(intendShoot).whileActiveContinuous(aimCmd);
                turretEnabledTrigger.and(intendShoot.negate()).whileActiveContinuous(aimCenter);
                turretEnabledTrigger.negate().whenActive(off);
            }
            else
            {
                // Original behavior: Y toggles between aiming and off
                turretEnabledTrigger.whileActiveContinuous(aimCmd);
                turretEnabledTrigger.negate().whenActive(off);
            }
        }
    }

    private void bindOuttakeControls(Trigger active, GamepadEx driver, Subsystems s)
    {
        if (s.outtake == null) return;

        if (regressionTestingMode.getAsBoolean())
        {
            driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(new OuttakeCommands.ChangeTargetRPM(s.outtake, 5));
            driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(new OuttakeCommands.ChangeTargetRPM(s.outtake, -5));
        }

        if (s.odometry != null)
        {
            // Always bind the command - the actual adjustment check happens dynamically inside Outtake.setTargetRPM(Distance)
            active.whileActiveContinuous(new OuttakeCommands.UpdateRPMBasedOnDistance(
                    s.outtake,
                    () -> s.odometry.getPose().distanceTo(getGoal())));
        }
    }

    private void bindIntakeAndTransferLogic(Trigger active, GamepadEx driver, GamepadEx coDriver, Subsystems s)
    {
        if (s.intake == null || s.transfer == null || s.outtake == null || s.turret == null) return;

        // --- Triggers ---
        Trigger intakeBtn = new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1);
        Trigger shootBtn = new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1);
        Trigger reverseBtn = new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_LEFT));

        Trigger systemsReady = new Trigger(s.turret::isAtTarget).and(new Trigger(s.outtake::isStable));

        // --- Commands ---
        Command intakeIn = new IntakeCommands.In(s.intake, intakePower.getAsDouble());
        Command intakeScore = new IntakeCommands.Transfer(s.intake, this::getIntakeTransferPower);
        Command reverseIntake = new IntakeCommands.Reverse(s.intake, outtakePower.getAsDouble());
        //Command closeTransfer = new TransferCommands.CloseOnce(s.transfer);
        Command openTransfer = new TransferCommands.Open(s.transfer);

        // --- Logic ---

        // 1. Intake Logic: (Auto OR Manual) AND Not Shooting AND Not Reversing
        Trigger shouldIntake = (intakeByDefault.getAsBoolean() ? active : intakeBtn)
                .and(shootBtn.negate())
                .and(reverseBtn.negate());

        // Use whileActiveContinuous instead of whileActiveOnce to fix the command
        // scheduling bug
        // whileActiveContinuous ensures the command is properly re-scheduled each time
        // the trigger becomes active
        shouldIntake.whileActiveContinuous(intakeIn).cancelWhenActive(openTransfer);
        reverseBtn.whileActiveContinuous(reverseIntake);

        // 2. Scoring Logic
        boolean semiAuto = intakeByDefault.getAsBoolean() || outtakeOnByDefault.getAsBoolean();

        // "Can Score" = User wants to shoot + Systems are ready ( + Manual intake hold
        // if not semi-auto)
        Trigger intendShoot = semiAuto
                ? shootBtn
                : intakeBtn.and(shootBtn);

        Trigger canScore = intendShoot.and(systemsReady);

        // "Keep Scoring" = Hysteresis to ensure we finish the shot even if turret
        // jitters slightly
        Trigger keepScoring = semiAuto
                ? shootBtn
                : intakeBtn.and(shootBtn);

        // Action: If intending to shoot, open the transfer
        intendShoot.whenActive(openTransfer);

        // Action: When ready to shoot, run the intake
        canScore.whenActive(intakeScore);

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

    private FieldCoordinate getGoal()
    {
        Odometry odometry = robot.subsystems.odometry;

        if (odometry == null)
            return robot.team.goalFromFar.coord;

        if (odometry.getFieldCoord().toCoordinateSystem(CoordinateSystem.DECODE_PEDROPATH).y.getInch() > 48)
        {
            return robot.team.goalFromClose.coord;
        }
        else
        {
            return robot.team.goalFromFar.coord;
        }
    }

    private double getIntakeTransferPower()
    {
        Odometry odometry = robot.subsystems.odometry;

        if (odometry == null)
            return minimumTransferPower.getAsDouble();

        if (odometry.getFieldCoord().toCoordinateSystem(CoordinateSystem.DECODE_PEDROPATH).y.getInch() > 48)
        {
            return maximumTransferPower.getAsDouble();
        }
        else
        {
            return minimumTransferPower.getAsDouble();
        }
    }

    /**
     * @return The autonomous pose or null if it does not exist.
     *
     * @param timeout The maximum acceptable file age in milliseconds.
     */
    private Pose2d getSavedAutonomousPose(long timeout)
    {
        // get the file reference
        File file = AppUtil.getInstance().getSettingsFile(RobotConstants.Autonomous.AUTONOMOUS_POSE_FILE_NAME);

        // check if the file exists
        if (!file.exists())
        {
            return null;
        }

        // check if the file is fresh enough
        long fileAge = System.currentTimeMillis() - file.lastModified();
        if (fileAge > timeout)
        {
            return null;
        }

        // read the data; pedropath, inches, degrees
        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
            String line = br.readLine();
            if (line == null) return null;

            String[] parts = line.split(",");

            // x, y, heading, coordinate system
            Pose2d pose = new Pose2d(
                    new Distance(Double.parseDouble(parts[0]), DistanceUnit.INCH),
                    new Distance(Double.parseDouble(parts[1]), DistanceUnit.INCH),
                    new Angle(Double.parseDouble(parts[2]), AngleUnit.DEGREES),
                    CoordinateSystem.valueOf(parts[3])
            );

            // Delete the file after successfully reading it
            if (RobotConstants.Autonomous.DELETE_AUTONOMOUS_POSE_AFTER_FIRST_READ)
            {
                file.delete();
            }

            return pose;
        }
        catch (Exception e)
        {
            return null;
        }
    }
}