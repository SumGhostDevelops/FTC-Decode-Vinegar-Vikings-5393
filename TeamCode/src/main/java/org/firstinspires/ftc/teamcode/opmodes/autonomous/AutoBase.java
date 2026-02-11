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
    private final double transferPower = RobotConstants.Intake.transferPower;

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
     * Updates subsystem PIDF controllers. Call this in wait loops.
     */
    protected void updateSubsystems()
    {
        if (robot == null)
            return;
        Subsystems s = robot.subsystems;

        s.turret.aimToCoordinate(team.goalFromClose.coord, getPose2d());
        s.outtake.setTargetRPM(getPose2d().distanceTo(team.goalFromClose.coord));

        s.turret.periodic();
        s.outtake.periodic();
    }

    // ========================
    // Intake Control
    // ========================

    /**
     * Starts the intake motor at default power.
     */
    protected void startIntake()
    {
        if (robot == null)
            return;
        robot.subsystems.intake.intake(intakePower);
    }

    /**
     * Stops the intake motor.
     */
    protected void stopIntake()
    {
        if (robot == null)
            return;
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
        Distance distToGoal = pose.distanceTo(team.goalFromClose.coord);
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
        FieldCoordinate goalCoord = team.goalFromClose.coord;
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
     * Shoots by opening transfer and running intake at transfer power.
     * Waits for the specified duration while updating subsystems.
     * 
     * @param durationMs
     *            how long to feed balls through
     */
    protected void shootForDuration(long durationMs)
    {
        if (robot == null)
            return;

        openTransfer();
        robot.subsystems.intake.intake(transferPower);

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && !isStopRequested() && timer.milliseconds() < durationMs)
        {
            // Update follower pose
            if (follower != null)
            {
                follower.update();
            }

            updateSubsystems();
            aimTurret();
            sleep(10);
        }

        closeTransfer();
        stopIntake();
    }

    /**
     * Complete shoot sequence: waits for ready, then shoots.
     * Wraps waitForSystemsReady() and shootForDuration().
     */
    public void Shoot()
    {
        if (robot == null)
            return;

        // Start outtake if not already running
        startOuttake();
        closeTransfer();

        // Wait up to 2 seconds for systems to be ready
        waitForSystemsReady(2000);

        // Shoot for 500ms
        if (isReadyToShoot())
        {
            shootForDuration(500);
        }
    }

    /**
     * Complete intake sequence: runs intake and closes transfer.
     */
    public void Intake()
    {
        if (robot == null)
            return;

        closeTransfer();
        startIntake();
    }
}
