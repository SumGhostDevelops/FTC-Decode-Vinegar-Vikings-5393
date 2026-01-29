package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.Timing;
import com.seattlesolvers.solverslib.util.Timing.Timer;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
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
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.CoordinateSystem;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;

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
    private final ElapsedTime logTimer = new ElapsedTime();
    private double lastCleared = logTimer.seconds();
    private Timing.Stopwatch stopwatch;

    @Override
    public void initialize()
    {
        stopwatch = new Timing.Stopwatch(TimeUnit.MILLISECONDS);
        stopwatch.start();
        ConstantsPresets.applyPreset();
        robot = new RobotContext(team, hardwareMap, telemetry, gamepad1, gamepad2);
        telemetry.log().add("RobotContext initialized in " + stopwatch.deltaTime() + "ms");
        telemetry.update();

        // Initialize Panels Field with FTC Standard coordinates
        FieldDrawing.init();

        register(robot.subsystems.drive, robot.subsystems.intake, robot.subsystems.transfer, robot.subsystems.turret, robot.subsystems.outtake, robot.subsystems.odometry);

        // Set the default command for transfer to maintain its initial position
        robot.subsystems.transfer.setDefaultCommand(new TransferCommands.DefaultPosition(robot.subsystems.transfer));

        DoubleSupplier x = () -> gamepad1.left_stick_x; // Counteract imperfect strafing
        DoubleSupplier y = () -> -gamepad1.left_stick_y; // Y is inverted
        DoubleSupplier rx = () -> gamepad1.right_stick_x;
        Supplier<Angle> driverHeading = () -> robot.subsystems.odometry.getDriverHeading();

        robot.subsystems.drive.setDefaultCommand(new DriveCommands.Manuever(robot.subsystems.drive, x, y, rx, driverHeading));

        bindKeys();

        telemetry.log().add("Subsystems initialized in " + stopwatch.deltaTime() + "ms");
        telemetry.update();

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
        telemetry.addData("Loop Time", stopwatch.deltaTime() + "ms");

        if (RobotConstants.Telemetry.SET_AUTOCLEAR) telemetry.clear();
        if (RobotConstants.Telemetry.SET_AUTOCLEAR_LOGS && Math.abs(logTimer.seconds() - lastCleared) > RobotConstants.Telemetry.LOG_AUTOCLEAR_DELAY)
        {
            telemetry.log().clear();
            lastCleared = logTimer.seconds();
        }

        switch (RobotConstants.General.PRESET_OPTION)
        {
            case DEFAULT:
            case COMPETITION:
                /*
                telemetry.addData("Team", team);
                telemetry.addData("Remaining Time", timer.remainingTime() + "/120");
                telemetry.addData("Distance to Goal (inches)", robot.subsystems.odometry.getFieldCoord().distanceTo(team.goal.coord).toUnit(DistanceUnit.INCH));
                telemetry.addLine("--- Odometry ---");
                telemetry.addData("Absolute Heading (deg)", robot.subsystems.odometry.getFieldAngle().toSystem(CoordinateSystem.DECODE_PEDROPATH).angle.getUnsignedAngle(AngleUnit.DEGREES));
                telemetry.addLine("--- Drive ---");
                telemetry.addData("Speed (power)", robot.subsystems.drive.getSpeed());
                telemetry.addLine("--- Outtake ---");
                telemetry.addData("Target RPM", robot.subsystems.outtake.getTargetRPM());
                telemetry.addData("True RPM", robot.subsystems.outtake.getRPM());
                telemetry.addData("Is Stable", robot.subsystems.outtake.isReady());
                telemetry.addLine("--- Turret ---");
                telemetry.addData("Relative Heading (deg)", robot.subsystems.turret.getRelativeAngle().getUnsignedAngle(AngleUnit.DEGREES));
                telemetry.addData("Absolute Heading (deg)", robot.subsystems.turret.getFieldHeading(robot.subsystems.odometry.getFieldAngle()).toUnnormalized());

                 */
            case TESTING:
                telemetry.addData("Team", team);
                telemetry.addData("Distance to Goal", robot.subsystems.odometry.getFieldCoord().distanceTo(team.goal.coord).toUnit(DistanceUnit.INCH));
                telemetry.addLine("--- Odometry ---");
                telemetry.addData("Coordinate", robot.subsystems.odometry.getPose().toCoordinateSystem(CoordinateSystem.DECODE_PEDROPATH));
                telemetry.addData("Coordinate", robot.subsystems.odometry.getPose().toCoordinateSystem(CoordinateSystem.DECODE_FTC));
                telemetry.addData("IMU Yaw", robot.subsystems.odometry.getIMUYaw());
                telemetry.addData("Driver Heading", robot.subsystems.odometry.getDriverHeading());
                telemetry.addData("Field Heading", robot.subsystems.odometry.getFieldHeading().toSystem(CoordinateSystem.DECODE_PEDROPATH));
                telemetry.addData("Velocity (in/sec)", robot.subsystems.odometry.getVelocity().toCoordinateSystem(CoordinateSystem.DECODE_PEDROPATH));
                telemetry.addLine("--- Drive ---");
                telemetry.addData("Speed (power)", robot.subsystems.drive.getSpeed());
                telemetry.addLine("--- Intake ---");
                telemetry.addData("RPM", robot.subsystems.intake.getRPM());
                telemetry.addLine("--- Outtake ---");
                telemetry.addData("Target RPM", robot.subsystems.outtake.getTargetRPM());
                telemetry.addData("RPM", robot.subsystems.outtake.getRPM());
                telemetry.addData("Acceleration", robot.subsystems.outtake.getRPMAcceleration());
                telemetry.addData("Is Stable", robot.subsystems.outtake.isReady());
                telemetry.addLine("--- Turret ---");
                telemetry.addData("Is At Target", robot.subsystems.turret.isAtTarget());
                telemetry.addData("Relative Heading", robot.subsystems.turret.getRelativeAngle().toUnnormalized().toUnit(UnnormalizedAngleUnit.DEGREES));
                telemetry.addData("Absolute Heading", robot.subsystems.turret.getFieldHeading(robot.subsystems.odometry.getFieldHeading()).toUnnormalized().toUnit(UnnormalizedAngleUnit.DEGREES));
                telemetry.addData("Bearing to Target", robot.subsystems.turret.bearingToTarget());
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
        FieldDrawing.draw(
                robot.subsystems.odometry.getPose(),
                null,
                robot.subsystems.turret.getFieldHeading(robot.subsystems.odometry.getFieldHeading()),
                robot.team.goal.coord
        );
        FieldDrawing.sendPacket();
        robot.hw.clearHubCache();
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
        // 1. Setup local references and shared triggers
        GamepadEx driver = robot.gamepads.driver;
        GamepadEx coDriver = robot.gamepads.coDriver;
        Subsystems s = robot.subsystems;

        Trigger opModeIsActive = new Trigger(this::opModeIsActive);
        Trigger turretReady = new Trigger(() -> Math.abs(s.turret.bearingToTarget().toUnit(AngleUnit.DEGREES).measure) < 8.5);
        Trigger outtakeReady = new Trigger(s.outtake::isReady);

        // Define Raw Triggers
        Trigger dLT = new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.25);
        Trigger dRT = new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.25);
        Trigger cdLT = new Trigger(() -> coDriver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.25);
        Trigger cdRT = new Trigger(() -> coDriver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.25);

        // 2. Delegate to specialized binding methods
        bindDriveControls(driver, s);
        bindTurretControls(opModeIsActive, driver, s);
        bindOuttakeControls(opModeIsActive, driver, s);

        // Pass all triggers to the logic method
        bindIntakeAndTransferLogic(opModeIsActive, dLT, dRT, cdLT, cdRT, turretReady, outtakeReady, s);
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

    private void bindTurretControls(Trigger active, GamepadEx driver, Subsystems s)
    {
        Supplier<Pose2d> turretPose = getPoseSupplier(RobotConstants.Turret.USE_FUTURE_POSE, RobotConstants.Turret.FUTURE_POSE_TIME, s);
        Command aimCmd = new TurretCommands.AimToGoal(s.turret, robot.team.goal.coord, turretPose);

        if (RobotConstants.Turret.autoAimToGoal)
        {
            active.whileActiveContinuous(aimCmd);
        }
        else
        {
            driver.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(aimCmd);
        }
    }

    private void bindOuttakeControls(Trigger active, GamepadEx driver, Subsystems s)
    {
        if (RobotConstants.General.REGRESSION_TESTING_MODE)
        {
            driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(new OuttakeCommands.ChangeTargetRPM(s.outtake, 25));
            driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(new OuttakeCommands.ChangeTargetRPM(s.outtake, -25));
        }
        if (RobotConstants.Outtake.AUTO_DISTANCE_ADJUSMENT)
        {
            Supplier<Pose2d> outtakePose = getPoseSupplier(RobotConstants.Outtake.USE_FUTURE_POSE, RobotConstants.Outtake.FUTURE_POSE_TIME, s);
            active.whileActiveContinuous(new OuttakeCommands.UpdateRPMBasedOnDistance(s.outtake, () -> outtakePose.get().distanceTo(team.goal.coord)));
        }
    }

    private void bindIntakeAndTransferLogic(Trigger active, Trigger dLT, Trigger dRT, Trigger cdLT, Trigger cdRT, Trigger turretReady, Trigger outtakeReady, Subsystems s)
    {
        boolean twoPerson = RobotConstants.General.TWO_PERSON_OPERATION;

        // Abstract the Controls
        Trigger intakeTrigger = twoPerson ? cdLT : dLT;
        Trigger shootTrigger  = twoPerson ? cdRT : dRT;

        // Shared Command Instances
        Command intakeIn = new IntakeCommands.In(s.intake, () -> RobotConstants.Intake.intakeRPM);
        Command intakeTransfer = new IntakeCommands.In(s.intake, () -> RobotConstants.Intake.transferPassRPM);
        Command transferOpen = new TransferCommands.OpenTransfer(s.transfer);
        Command transferClose = new TransferCommands.CloseTransfer(s.transfer);

        /* --- 1. INTAKE LOGIC --- */
        // Trigger if INTAKE_BY_DEFAULT is on, OR if the manual intake trigger is held.
        // We add .and(shootTrigger.negate()) to ensure intake stops when we try to score.
        Trigger shouldIntake = (RobotConstants.Intake.INTAKE_BY_DEFAULT ? active : intakeTrigger)
                .and(shootTrigger.negate());

        shouldIntake.whileActiveContinuous(intakeIn).whileActiveContinuous(transferClose);

        /* --- 2. THE START CONDITION (canScore) --- */
        boolean isSemiAuto = RobotConstants.Intake.INTAKE_BY_DEFAULT || RobotConstants.Outtake.ON_BY_DEFAULT;

        Trigger canScore = isSemiAuto
                ? shootTrigger.and(turretReady).and(outtakeReady)
                : intakeTrigger.and(shootTrigger).and(turretReady).and(outtakeReady);

        /* --- 3. THE PERSISTENCE/EXIT CONDITION (keepScoring) --- */
        Trigger keepScoring = isSemiAuto
                ? shootTrigger.and(turretReady)
                : intakeTrigger.and(shootTrigger).and(turretReady);

        // Bindings for scoring
        canScore.whenActive(intakeTransfer).whenActive(transferOpen);
        keepScoring.negate().cancelWhenActive(intakeTransfer).cancelWhenActive(transferOpen);

        /* --- 4. FLYWHEEL POWER --- */
        if (RobotConstants.Outtake.ON_BY_DEFAULT)
        {
            active.whileActiveContinuous(new OuttakeCommands.On(s.outtake, () -> false));
        }
        else
        {
            shootTrigger.whileActiveContinuous(new OuttakeCommands.On(s.outtake, () -> false));
        }

        /* --- 5. MANUAL OVERRIDES (Driver D-Pad) --- */
        robot.gamepads.driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whileHeld(new IntakeCommands.Out(s.intake, () -> RobotConstants.Intake.outtakePower));
    }

    private void bindCoDriverControls(GamepadEx coDriver, Subsystems s)
    {
    }

    /**
     * Helper to avoid duplicating FuturePose logic
     */
    private Supplier<Pose2d> getPoseSupplier(boolean useFuture, double time, Subsystems s)
    {
        //return useFuture ? () -> s.odometry.getFuturePose(time) : s.odometry::getPose;

        return s.odometry::getPose;
    }
}