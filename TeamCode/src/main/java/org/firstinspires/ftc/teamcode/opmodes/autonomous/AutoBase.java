package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.constants.Team;
import org.firstinspires.ftc.teamcode.definitions.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.definitions.hardware.Subsystems;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.CoordinateSystem;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintWriter;

/**
 * Common base class for all autonomous modes with ball flow methods.
 * Provides intake, transfer, outtake, turret aiming and shooting logic.
 */
public abstract class AutoBase extends LinearOpMode
{
    protected Team team;
    protected RobotHardware hw;
    protected Subsystems subsystems;
    protected Follower follower;
    protected boolean finishedAutonomous = false; // flag for if the robot should end the teleop mode early

    // Constants from RobotConstants
    private final double intakePower = RobotConstants.Intake.intakePower;
    private final double transferPower = RobotConstants.Intake.minimumTransferPower;

    protected enum AutoStrat
    {
        BASIC, REGULAR, GATE
    }

    // Common autonomous state - shared by all subclasses
    protected Timer pathStateTimer;
    protected Timer opModeTimer;
    protected AutoStrat autoStrat = AutoStrat.REGULAR;
    protected Object currentPathState;

    /**
     * Strategy selection configuration for customization
     */
    protected static class StrategyOption
    {
        public final AutoStrat strat;
        public final String button;
        public final String description;

        public StrategyOption(AutoStrat strat, String button, String description)
        {
            this.strat = strat;
            this.button = button;
            this.description = description;
        }
    }

    // ========================
    // Template Methods - Override in subclasses as needed
    // ========================

    /**
     * Get strategy options for selection menu.
     * Override to customize available strategies and their descriptions.
     */
    protected StrategyOption[] getStrategyOptions()
    {
        return new StrategyOption[]{
                new StrategyOption(AutoStrat.GATE, "DPAD UP", "GATE METHOD (GATE)"),
                new StrategyOption(AutoStrat.REGULAR, "DPAD RIGHT", "12 Ball (REGULAR)"),
                new StrategyOption(AutoStrat.BASIC, "DPAD DOWN", "3 Ball (BASIC)")
        };
    }

    /**
     * Create and initialize paths object.
     * Subclasses must implement this to return their specific Paths instance.
     */
    protected abstract Object createPaths(Follower follower, AutoStrat autoStrat);

    /**
     * Get the starting pose for the autonomous.
     * Subclasses must implement this to return the starting pose from their Paths object.
     */
    protected abstract Pose getStartingPose(Object paths);

    /**
     * Handle pathing logic based on current state.
     * Default implementation delegates to strategy-specific methods.
     * Override for custom behavior.
     */
    protected void handlePathing()
    {
        switch (autoStrat)
        {
            case BASIC:
                PathBasic();
                break;
            case REGULAR:
                PathRegular();
                break;
            case GATE:
                PathGate();
                break;
        }
    }

    /**
     * Path logic for BASIC strategy.
     * Override in subclasses.
     */
    protected void PathBasic()
    {
        // Default: do nothing
    }

    /**
     * Path logic for REGULAR strategy.
     * Override in subclasses.
     */
    protected void PathRegular()
    {
        // Default: do nothing
    }

    /**
     * Path logic for GATE strategy.
     * Override in subclasses.
     */
    protected void PathGate()
    {
        // Default: do nothing
    }

    // ========================
    // Common Helper Methods
    // ========================

    /**
     * Sets the current path state and resets the state timer.
     */
    protected void setPathState(Object pathState)
    {
        currentPathState = pathState;
        if (pathStateTimer != null)
        {
            pathStateTimer.resetTimer();
        }
    }

    /**
     * Display strategy selection menu during init.
     * Handles gamepad input and shows available options.
     * @return selected strategy
     */
    protected AutoStrat selectStrategy()
    {
        AutoStrat selected = AutoStrat.REGULAR;
        StrategyOption[] options = getStrategyOptions();

        while (opModeInInit())
        {
            // Handle gamepad input
            if (gamepad1.dpad_up)
            {
                for (StrategyOption opt : options)
                {
                    if (opt.button.contains("UP"))
                    {
                        selected = opt.strat;
                        break;
                    }
                }
            }
            else if (gamepad1.dpad_right)
            {
                for (StrategyOption opt : options)
                {
                    if (opt.button.contains("RIGHT"))
                    {
                        selected = opt.strat;
                        break;
                    }
                }
            }
            else if (gamepad1.dpad_down)
            {
                for (StrategyOption opt : options)
                {
                    if (opt.button.contains("DOWN"))
                    {
                        selected = opt.strat;
                        break;
                    }
                }
            }
            else if (gamepad1.dpad_left)
            {
                for (StrategyOption opt : options)
                {
                    if (opt.button.contains("LEFT"))
                    {
                        selected = opt.strat;
                        break;
                    }
                }
            }

            // Display menu
            telemetry.addLine("--- SELECT AUTO STRATEGY ---");
            telemetry.addData("Selected", selected);
            telemetry.addLine("\nControls:");
            for (StrategyOption opt : options)
            {
                telemetry.addLine(opt.button + ": " + opt.description);
            }
            telemetry.update();
        }

        return selected;
    }

    /**
     * Initialize autonomous with common setup.
     * Creates timers, follower, paths, and initializes robot hardware.
     */
    protected void initAuto(Object paths)
    {
        // Initialize Timers
        pathStateTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        // Set starting pose and update follower
        follower.setStartingPose(getStartingPose(paths));
        follower.update();

        // Init Robot Hardware
        initRobot();
    }

    /**
     * Main autonomous loop template method.
     * Handles the common structure: start outtake, run paths, update robot, cleanup.
     * Call this from runOpMode() after initialization.
     */
    protected void runAutoLoop()
    {
        if (opModeIsActive() && !isStopRequested())
        {
            opModeTimer.resetTimer();
            startOuttake();

            while (opModeIsActive() && !isStopRequested() && !finishedAutonomous)
            {
                handlePathing();
                follower.update();
                updateRobot();
            }
        }

        stopSubsystems();
        writePoseToFile();
    }

    /**
     * Default telemetry display with common autonomous info.
     * Override to add custom telemetry.
     */
    protected void displayTelemetry()
    {
        if (pathStateTimer == null || opModeTimer == null || follower == null)
            return;

        telemetry.addData("Current State", currentPathState);
        telemetry.addData("State Time (s)", pathStateTimer.getElapsedTimeSeconds());
        telemetry.addData("OpMode Time (s)", opModeTimer.getElapsedTimeSeconds());
        telemetry.addData("Heading", follower.getHeading());
        telemetry.addLine("-----");
        telemetry.addData("Pose", getPose2d());
        telemetry.addData("Velocity", follower.getVelocity());
        telemetry.addData("Distance to Goal", getPose2d().distanceTo(getGoal()).toUnit(DistanceUnit.INCH));

        if (subsystems != null && subsystems.outtake != null)
        {
            telemetry.addData("Outtake RPM", subsystems.outtake.getMotorRPM());
        }

        if (subsystems != null && subsystems.turret != null)
        {
            telemetry.addData("Turret Angle", subsystems.turret.getRelativeAngle().toUnit(AngleUnit.DEGREES));
            telemetry.addData("Turret Target Angle", subsystems.turret.getTargetAngleDegrees());
        }

        telemetry.update();
    }

    // ========================
    // Initialization
    // ========================

    /**
     * Initialize robot hardware and subsystems.
     * Call this at the start of runOpMode() after setting team.
     */
    protected void initRobot()
    {
        robot = new RobotContext(team, hardwareMap, telemetry, gamepad1, gamepad2);
        hw = new RobotHardware.Builder(hardwareMap, telemetry)
                .withIntake()
                .withTransfer()
                .withTurret()
                .withOuttake()
                .build();

        subsystems = new Subsystems.Builder(hw, team)
                .withIntake()
                .withTransfer()
                .withTurret()
                .withOuttake()
                .build();

        assert intakeExists() : "Intake is null";
        assert transferExists() : "Transfer is null";
        assert turretExists() : "Turret is null";
        assert outtakeExists() : "Outtake is null";
        assert !odometryDoesNotExist() : "Odometry is not null";

        telemetry.addData("Status", "Initialized for " + team);
        telemetry.update();
    }


    /**
     * Set the Pedro follower reference so we can get pose from it.
     */
    protected void setFollower(Follower follower)
    {
        this.follower = follower;
    }

    // ========================
    // Pose Conversion
    // ========================

    /**
     * Converts Pedro Pose to our Pose2d using DECODE_PEDROPATH coordinate system.
     */
    protected Pose2d getPose2d()
    {
        if (follower == null)
        {
            // Fallback if no follower set
            return new Pose2d(
                    new Distance(0, DistanceUnit.INCH),
                    new Distance(0, DistanceUnit.INCH),
                    new Angle(0, AngleUnit.RADIANS),
                    CoordinateSystem.DECODE_PEDROPATH);
        }
        Pose p = follower.getPose();
        return new Pose2d(
                new Distance(p.getX(), DistanceUnit.INCH),
                new Distance(p.getY(), DistanceUnit.INCH),
                new Angle(p.getHeading(), AngleUnit.RADIANS),
                CoordinateSystem.DECODE_PEDROPATH);
    }

    // ========================
    // Subsystem Updates
    // ========================

    /**
     * Updates subsystem PIDF controllers and manages state machine.
     * Call this in wait loops.
     *
     * State machine logic (matching BaseStable):
     * - Turret: Aims when enabled, otherwise off
     * - Outtake: RPM set based on distance to goal
     * - Transfer: Opens when intending to shoot AND intake is not running
     * - Intake: Only transfers when turret aimed AND outtake stable
     */
    protected void updateSubsystems()
    {
        if (hw == null)
            return;

        // Clear bulk cache to get fresh sensor readings (required for MANUAL caching mode)
        hw.clearHubCache();
        hw.readBattery();

        updateOuttakeRPM(); // update the outtake rpm constantly

        // --- PIDF Updates ---
        subsystems.turret.periodic();
        subsystems.outtake.periodic();
    }

    /**
     * Updates subsystems and telemetry
     */
    protected void updateRobot()
    {
        updateSubsystems();
        displayTelemetry();
    }

    /**
     * Stops the intake & outtake and closes the transfer.
     */
    protected void stopSubsystems()
    {
        stopIntake();
        stopOuttake();
        closeTransfer();
    }

    // ========================
    // Intake Control
    // ========================

    /**
     * Starts the intake motor at default power.
     * Sets isIntaking flag to keep transfer closed.
     */
    protected void startIntake()
    {
        subsystems.intake.intake(intakePower);
    }

    protected void transferIntake()
    {
        subsystems.intake.intake(getIntakeTransferPower());
    }

    /**
     * Stops the intake motor.
     * Clears isIntaking flag to allow transfer to open.
     */
    protected void stopIntake()
    {
        subsystems.intake.stop();
    }

    // ========================
    // Transfer Control
    // ========================

    /**
     * Closes the transfer gate to hold balls.
     */
    protected void closeTransfer()
    {
        subsystems.transfer.close();
    }

    /**
     * Opens the transfer gate to release balls into outtake.
     */
    protected void openTransfer()
    {
        subsystems.transfer.open();
    }

    // ========================
    // Outtake Control
    // ========================

    /**
     * Starts the outtake flywheel.
     */
    protected void startOuttake()
    {
        updateOuttakeRPM();
        subsystems.outtake.on();
    }

    /**
     * Stops the outtake flywheel.
     */
    protected void stopOuttake()
    {
        subsystems.outtake.off();
    }

    /**
     * Updates outtake target RPM based on distance to goal.
     */
    protected void updateOuttakeRPM()
    {
        Pose2d pose = getPose2d();
        Distance distToGoal = pose.distanceTo(getGoal());
        subsystems.outtake.setTargetRPM(distToGoal);
    }

    // ========================
    // Turret Control
    // ========================

    /**
     * Aims the turret at the goal using current pose.
     */
    protected void aimTurret()
    {
        Pose2d pose = getPose2d();
        FieldCoordinate goalCoord = getGoal();
        subsystems.turret.setState(Turret.State.ON);
        subsystems.turret.aimToCoordinate(goalCoord, pose);
    }

    protected void centerTurret()
    {
        subsystems.turret.center();
    }

    // ========================
    // Ready Checks
    // ========================

    /**
     * @return true if turret is aimed and outtake is stable (ready to shoot)
     */
    protected boolean isReadyToShoot()
    {
        Subsystems s = subsystems;

        boolean turretAtTarget = s.turret.isAtTarget();
        boolean outtakeStable = s.outtake.isStable();
        boolean notMoving = follower.getVelocity().getMagnitude() < 1;

        return turretAtTarget && outtakeStable && notMoving;
    }

    /**
     * Blocks until systems are ready to shoot, or timeout expires.
     * Continuously updates subsystems while waiting.
     * 
     * @param timeoutMs
     *            maximum time to wait in milliseconds
     * @return true if ready, false if timed out
     */
    protected boolean waitForSystemsReady(long timeoutMs)
    {

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && !isStopRequested() && timer.milliseconds() < timeoutMs)
        {
            // Update follower pose
            if (follower != null)
            {
                follower.update();
            }

            // Update subsystems (PIDF loops)
            updateRobot();

            // Continuously aim and adjust RPM
            aimTurret();
            updateOuttakeRPM();

            // Check if ready
            if (isReadyToShoot())
            {
                return true;
            }

            // Small sleep to not hog CPU
            sleep(10);
        }
        return isReadyToShoot();
    }

    // ========================
    // Shooting
    // ========================

    /**
     * Continues shooting for the specified duration while updating subsystems.
     * Assumes intendingToShoot is already true and systems are ready.
     * The state machine in updateSubsystems() handles intake power.
     *
     * @param durationMs
     *            how long to continue feeding balls through
     */
    protected void waitForDuration(long durationMs)
    {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && !isStopRequested() && timer.milliseconds() < durationMs)
        {
            // Update follower pose
            if (follower != null)
            {
                follower.update();
            }

            // State machine handles transfer and intake
            updateRobot();
            sleep(10);
        }
    }

    /**
     * Complete shoot sequence using state machine logic.
     * 1. Ensures outtake is on, opens transfer, and aims turret
     * 2. Waits for systems to be ready
     * 3. If ready, transfer & shoot for duration
     * 4. Stop the intake, close the transfer, and center the turret
     */
    public void Shoot()
    {
        openTransfer();
        startOuttake(); // start outtake if not already running
        aimTurret();

        // Wait up to 2 seconds for systems to be ready
        // updateSubsystems() will handle transfer and intake state
        boolean isReady = waitForSystemsReady(RobotConstants.Autonomous.WAIT_SYS_READY_MS);

        // If ready, let it score for the duration
        if (isReady)
        {
            transferIntake();
            waitForDuration(RobotConstants.Autonomous.SHOOT_LENGTH_MS);
        }

        stopIntake();
        closeTransfer();
        centerTurret();
    }

    // ========================
    // Goal Selection (matching BaseStable logic)
    // ========================

    /**
     * Gets the appropriate goal based on field position.
     * Uses close goal if Y > 48 inches, otherwise far goal.
     *
     * @return the target goal coordinate
     */
    protected FieldCoordinate getGoal()
    {
        Pose2d pose = getPose2d();

        if (pose.toCoordinateSystem(CoordinateSystem.DECODE_PEDROPATH).coord.y.getInch() > 48)
        {
            return team.goalFromClose.coord;
        }
        else
        {
            return team.goalFromFar.coord;
        }
    }

    /**
     * Gets the appropriate intake transfer power based on field position.
     * Uses full power if Y > 48 inches, otherwise minimum transfer power.
     *
     * @return the intake power to use for transferring balls
     */
    protected double getIntakeTransferPower()
    {
        Pose2d pose = getPose2d();

        if (pose.toCoordinateSystem(CoordinateSystem.DECODE_PEDROPATH).coord.y.getInch() > 48)
        {
            return 1.0;
        }
        else
        {
            return transferPower;
        }
    }

    /**
     * Writes a pose to a file in the PedroPath coordinate system, in inches, in degrees.
     */
    protected void writePoseToFile()
    {
        if (!RobotConstants.Autonomous.SAVE_END_AUTONOMOUS_POSE)
        {
            return;
        }

        Pose2d pose = getPose2d().toCoordinateSystem(CoordinateSystem.DECODE_PEDROPATH).toDistanceUnit(DistanceUnit.INCH).toAngleUnit(AngleUnit.DEGREES);

        String coordinateSystem = String.valueOf(pose.coord.coordSys);
        String x = String.valueOf(pose.coord.x.getInch());
        String y = String.valueOf(pose.coord.y.getInch());
        String heading = String.valueOf(pose.heading.angle.measure);

        File file = AppUtil.getInstance().getSettingsFile(RobotConstants.Autonomous.AUTONOMOUS_POSE_FILE_NAME);
        String data = x + "," + y + "," + heading + "," + coordinateSystem;

        try (PrintWriter out = new PrintWriter(new FileOutputStream(file)))
        {
            out.println(data);
            telemetry.addData("Pose", "Saved: %s", data);
        }
        catch (Exception e)
        {
            telemetry.addData("Error", "Save failed: %s", e.getMessage());
        }
    }

    private boolean intakeExists()
    {
        return hw.intake != null && subsystems.intake != null;
    }

    private boolean transferExists()
    {
        return hw.transfer != null && subsystems.transfer != null;
    }

    private boolean turretExists()
    {
        return hw.turret != null && subsystems.turret != null;
    }

    private boolean outtakeExists()
    {
        return hw.outtake != null && subsystems.outtake != null;
    }

    private boolean odometryDoesNotExist()
    {
        return hw.pinpoint == null && subsystems.odometry == null;
    }
}
