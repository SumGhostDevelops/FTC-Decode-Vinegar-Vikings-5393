package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.constants.Team;
import org.firstinspires.ftc.teamcode.definitions.hardware.RobotContext;
import org.firstinspires.ftc.teamcode.definitions.hardware.Subsystems;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.CoordinateSystem;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

/**
 * Common base class for all autonomous modes with ball flow methods.
 * Provides intake, transfer, outtake, turret aiming and shooting logic.
 */
public abstract class AutoBase extends LinearOpMode
{
    protected Team team;
    protected RobotContext robot;
    protected Follower follower;

    // Constants from RobotConstants
    private final double intakePower = RobotConstants.Intake.intakePower;
    private final double transferPower = RobotConstants.Intake.minimumTransferPower;

    // Turret state (matching BaseStable logic)
    protected boolean turretEnabled = true;

    // Subsystem state flags (matching BaseStable logic)
    protected boolean intendingToShoot = false;
    protected boolean isScoring = false;
    protected boolean isIntaking = false;

    protected enum AutoStrat
    {
        BASIC, REGULAR, GATE
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
        if (robot == null)
            return;
        Subsystems s = robot.subsystems;

        // --- Turret Logic ---
        if (turretEnabled)
        {
            s.turret.setState(Turret.State.ON);
            s.turret.aimToCoordinate(getGoal(), getPose2d());
        }
        else
        {
            s.turret.setState(Turret.State.OFF);
        }

        // --- Outtake Logic ---
        s.outtake.setTargetRPM(getPose2d().distanceTo(getGoal()));

        // --- Transfer Logic ---
        // Transfer opens ONLY when intending to shoot AND intake is not running
        // Otherwise, transfer stays closed
        if (intendingToShoot && !isIntaking)
        {
            s.transfer.open();
        }
        else
        {
            s.transfer.close();
        }

        // --- Intake/Scoring Logic ---
        // Only transfer balls when systems are ready
        boolean systemsReady = s.turret.isAtTarget() && s.outtake.isStable();

        if (intendingToShoot && !isIntaking && systemsReady)
        {
            // Start scoring - feed balls through
            isScoring = true;
            s.intake.intake(getIntakeTransferPower());
        }
        else if (!intendingToShoot)
        {
            // Stop scoring when we stop intending to shoot
            isScoring = false;
        }
        // Note: If intending to shoot but not ready, intake keeps its current state

        // --- PIDF Updates ---
        s.turret.periodic();
        s.outtake.periodic();
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
        if (robot == null)
            return;
        isIntaking = true;
        robot.subsystems.intake.intake(intakePower);
    }

    /**
     * Stops the intake motor.
     * Clears isIntaking flag to allow transfer to open.
     */
    protected void stopIntake()
    {
        if (robot == null)
            return;
        isIntaking = false;
        robot.subsystems.intake.stop();
    }

    // ========================
    // Transfer Control
    // ========================

    /**
     * Closes the transfer gate to hold balls.
     */
    protected void closeTransfer()
    {
        if (robot == null)
            return;
        robot.subsystems.transfer.close();
    }

    /**
     * Opens the transfer gate to release balls into outtake.
     */
    protected void openTransfer()
    {
        if (robot == null)
            return;
        robot.subsystems.transfer.open();
    }

    // ========================
    // Outtake Control
    // ========================

    /**
     * Starts the outtake flywheel.
     */
    protected void startOuttake()
    {
        if (robot == null)
            return;
        robot.subsystems.outtake.on();
    }

    /**
     * Stops the outtake flywheel.
     */
    protected void stopOuttake()
    {
        if (robot == null)
            return;
        robot.subsystems.outtake.off();
    }

    /**
     * Updates outtake target RPM based on distance to goal.
     */
    protected void updateOuttakeRPM()
    {
        if (robot == null)
            return;
        Pose2d pose = getPose2d();
        Distance distToGoal = pose.distanceTo(getGoal());
        robot.subsystems.outtake.setTargetRPM(distToGoal);
    }

    // ========================
    // Turret Control
    // ========================

    /**
     * Aims the turret at the goal using current pose.
     */
    protected void aimTurret()
    {
        if (robot == null)
            return;
        Pose2d pose = getPose2d();
        FieldCoordinate goalCoord = getGoal();
        robot.subsystems.turret.setState(Turret.State.ON);
        robot.subsystems.turret.aimToCoordinate(goalCoord, pose);
    }

    // ========================
    // Ready Checks
    // ========================

    /**
     * @return true if turret is aimed and outtake is stable (ready to shoot)
     */
    protected boolean isReadyToShoot()
    {
        if (robot == null)
            return false;
        Subsystems s = robot.subsystems;
        return s.turret.isAtTarget() && s.outtake.isStable();
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
            updateSubsystems();

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
    protected void shootForDuration(long durationMs)
    {
        if (robot == null)
            return;

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && !isStopRequested() && timer.milliseconds() < durationMs)
        {
            // Update follower pose
            if (follower != null)
            {
                follower.update();
            }

            // State machine handles transfer and intake
            updateSubsystems();
            sleep(10);
        }
    }

    /**
     * Complete shoot sequence using state machine logic.
     * 1. Stops intake and sets intendingToShoot = true (opens transfer, starts aiming)
     * 2. Waits for systems to be ready
     * 3. Scoring happens automatically when ready (via updateSubsystems)
     * 4. Shoots for duration, then resets state
     */
    public void Shoot()
    {
        if (robot == null)
            return;

        // Start outtake if not already running
        startOuttake();

        // Stop intake so transfer can open
        isIntaking = false;
        robot.subsystems.intake.stop();

        // Signal intent to shoot - this opens transfer and prepares systems
        intendingToShoot = true;
        isScoring = false;

        // Wait up to 2 seconds for systems to be ready
        // updateSubsystems() will handle transfer and intake state
        waitForSystemsReady(2000);

        // If ready, let it score for the duration
        if (isReadyToShoot())
        {
            shootForDuration(500);
        }

        // Reset state
        intendingToShoot = false;
        isScoring = false;
        closeTransfer();
        stopIntake();
    }

    /**
     * Complete intake sequence: runs intake and closes transfer.
     * Resets shooting state to allow normal intake operation.
     */
    public void Intake()
    {
        if (robot == null)
            return;

        // Reset shooting state
        intendingToShoot = false;
        isScoring = false;

        closeTransfer();
        startIntake();
    }

    // ========================
    // State Control
    // ========================

    /**
     * Signals intent to shoot. This will:
     * - Open the transfer gate (ready position)
     * - Continue aiming turret
     * - Spin up outtake
     * Once systems are ready, scoring will start automatically.
     */
    protected void setIntendingToShoot(boolean intending)
    {
        this.intendingToShoot = intending;
        if (!intending)
        {
            this.isScoring = false;
        }
    }

    /**
     * @return true if currently intending to shoot
     */
    protected boolean isIntendingToShoot()
    {
        return intendingToShoot;
    }

    /**
     * @return true if currently actively scoring (feeding balls)
     */
    protected boolean isScoring()
    {
        return isScoring;
    }

    /**
     * Enables or disables the turret.
     */
    protected void setTurretEnabled(boolean enabled)
    {
        this.turretEnabled = enabled;
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
}
