package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotMath;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
import org.firstinspires.ftc.teamcode.util.measure.angle.UnnormalizedAngle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;

public class Turret extends SubsystemBase
{
    public final MotorEx turretMotor;
    private final double degreePerTick;
    private Angle initialRelativeAngle;

    public int targetPosition = 0;

    /**
     * If true, the turret will continuously aim to the last {@link Turret#targetPosition} specified when {@link Turret#periodic()} is called.
     */
    public boolean lockToPosition = true;

    public final Angle tolerance = new Angle(2, AngleUnit.DEGREES);

    public Turret(MotorEx turretMotor, Angle initialRelativeAngle, boolean shouldLock)
    {
        this(turretMotor, initialRelativeAngle);
        this.lockToPosition = shouldLock;
    }
    public Turret(MotorEx turretMotor, Angle initialRelativeAngle)
    {
        this.turretMotor = turretMotor;
        this.initialRelativeAngle = initialRelativeAngle;
        //degreePerTick = 360.0 / (turretMotor.getCPR() * RobotConstants.Turret.GEAR_RATIO);
        degreePerTick = 360.0 / RobotConstants.Turret.TICKS_PER_REV;
    }

    /**
     * Aim absolutely to the field, around the reported {@code initialRelativeAngle}
     * @param targetAngle The absolute target angle
     * @param robotAngle The robot's absolute angle
     * @see #aimRelative(Angle targetAngle)
     */
    public void aimAbsolute(Angle targetAngle, Angle robotAngle)
    {
        aimRelative(targetAngle.minus(robotAngle));
    }

    /**
     * Aim relatively to the robot, around the reported {@code initialRelativeAngle}
     * @param targetAngle The desired turret angle relative to the robot's orientation
     * @see #aimAbsolute(Angle targetAngle, Angle robotAngle)
     */
    public void aimRelative(Angle targetAngle)
    {
        // Subtract initialRelativeAngle to convert from "relative to robot" coordinates
        // to "relative to motor zero position" coordinates
        Angle adjustedTarget = targetAngle.minus(initialRelativeAngle);
        int targetTicks = resolveSolution(turretMotor.getCurrentPosition(), adjustedTarget);

        // Get turn limits
        UnnormalizedAngle[] turnLimits = RobotConstants.Turret.TURN_LIMITS;

        // Final safety clamp (should not be needed if logic above is correct)
        int minTicks = angleToTicks(turnLimits[0]);
        int maxTicks = angleToTicks(turnLimits[1]);
        this.targetPosition = Math.max(minTicks, Math.min(maxTicks, targetTicks));

        if (isAtTarget())
        {
            return;
        }

        aim();
    }

    /**
     * Aims the turret toward a target field coordinate given the robot's current pose.
     * Uses the bearing from the robot's position to the target to compute the relative turret angle.
     * @param target The target {@link FieldCoordinate} to aim at (e.g., a goal position)
     * @param robotPose The robot's current {@link Pose2d} (position and heading)
     */
    public void aimToCoordinate(FieldCoordinate target, Pose2d robotPose)
    {
        // bearingTo computes the relative angle from the robot's heading to the target
        Angle relativeBearing = robotPose.bearingTo(target);
        aimRelative(relativeBearing);
    }

    /**
     * Aims to the tick specified in {@link Turret#targetPosition}
     */
    private void aim()
    {
        // Use SDK's RUN_TO_POSITION for built-in deceleration and position holding
        turretMotor.motorEx.setTargetPosition(targetPosition);
        if (!turretMotor.motorEx.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)) turretMotor.motorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.motorEx.setVelocity(2500);
    }

    public void reset()
    {
        aimRelative(RobotConstants.Turret.FORWARD_ANGLE);
    }

    /**
     * @param robotAngle
     * @return The absolute {@link Angle} of the turret based on the relative {@link Angle} and the robot's {@link Angle}
     * @see #getAbsoluteUnnormalizedAngle(UnnormalizedAngle)
     */
    public Angle getAbsoluteAngle(Angle robotAngle)
    {
        return getRelativeAngle().plus(robotAngle).minus(initialRelativeAngle);
    }

    /**
     *
     * @param robotAngle
     * @return The absolute {@link UnnormalizedAngle} of the turret based on the relative {@link UnnormalizedAngle} and the robot's {@link UnnormalizedAngle}
     * @see #getAbsoluteAngle(Angle)
     */
    public UnnormalizedAngle getAbsoluteUnnormalizedAngle(UnnormalizedAngle robotAngle)
    {
        return getRelativeUnnormalizedAngle().plus(robotAngle);
    }

    /**
     * @return The relative {@link Angle} of the turret
     * @see #getRelativeUnnormalizedAngle()
     */
    public Angle getRelativeAngle()
    {
        return getRelativeUnnormalizedAngle().toNormalized();
    }

    /**
     * @return The {@link UnnormalizedAngle} of the relative turret heading in the raw {@code UnnormalizedAngleUnit.DEGREES}
     * @see #getRelativeAngle()
     */
    public UnnormalizedAngle getRelativeUnnormalizedAngle()
    {
        double motorPosition = turretMotor.getCurrentPosition(); // in ticks
        double turretAngle = degreePerTick * motorPosition; // in degrees

        return initialRelativeAngle.toUnit(UnnormalizedAngleUnit.DEGREES) // convert to an unnormalized angle
                .plus(new UnnormalizedAngle(turretAngle, UnnormalizedAngleUnit.DEGREES)); // add the turret angle to the initial angle
    }

    public void setForward()
    {
        initialRelativeAngle = RobotConstants.Turret.FORWARD_ANGLE;
    }

    public Command turnToCommand(Angle angle)
    {
        return new InstantCommand(() -> this.aimRelative(angle), this);
    }

    /**
     * Resolves the best turret encoder position (in ticks) to reach the requested angle
     * while respecting the configured turn limits.
     *
     * @param currentTicks The current turret position in encoder ticks.
     * @param targetAngle  The desired turret angle to turn to.
     * @return The solution, in ticks
     */
    private int resolveSolution(int currentTicks, Angle targetAngle)
    {
        double currentDegrees = degreePerTick * currentTicks;
        double targetDegrees = targetAngle.getAngle(AngleUnit.DEGREES);

        // Find the 'n' (number of rotations) that gets us closest to current position
        double closestN = Math.round((currentDegrees - targetDegrees) / 360.0);

        // Get turn limits in degrees
        UnnormalizedAngle[] turnLimits = RobotConstants.Turret.TURN_LIMITS;
        double minDegrees = turnLimits[0].getAngle(UnnormalizedAngleUnit.DEGREES);
        double maxDegrees = turnLimits[1].getAngle(UnnormalizedAngleUnit.DEGREES);

        int bestTicks = currentTicks;
        double minDistance = Double.MAX_VALUE;

        // Check the nearest 3 possible rotations (current, one back, one forward)
        for (double n = closestN - 1; n <= closestN + 1; n++) {
            double candidate = targetDegrees + (n * 360.0);

            // Check limits (if applicable)
            if (candidate >= minDegrees && candidate <= maxDegrees) {
                int candidateTicks = (int) Math.round(candidate / degreePerTick);
                double dist = Math.abs(candidateTicks - currentTicks);
                if (dist < minDistance) {
                    minDistance = dist;
                    bestTicks = candidateTicks;
                }
            }
        }
        return bestTicks;
    }

    private int angleToTicks(Angle angle)
    {
        return RobotMath.Motor.angleToTicks(angle, turretMotor.getCPR(), RobotConstants.Turret.GEAR_RATIO);
    }

    private int angleToTicks(UnnormalizedAngle angle)
    {
        return RobotMath.Motor.angleToTicks(angle, turretMotor.getCPR(), RobotConstants.Turret.GEAR_RATIO);
    }

    public Angle bearingToTarget()
    {
        return RobotMath.Motor.ticksToAngle(turretMotor.motorEx.getCurrentPosition() - targetPosition, turretMotor.getCPR(), RobotConstants.Turret.GEAR_RATIO);
    }

    public boolean isAtTarget()
    {
        return Math.abs(turretMotor.motorEx.getCurrentPosition() - targetPosition) < RobotConstants.Turret.TOLERANCE;
    }

    @Override
    public void periodic()
    {
        if (!turretMotor.motorEx.isBusy()) // check if the motor is currently going to a target
        {
            if (this.lockToPosition) // if not, and we should be "locked" to a position
            {
                aim(); // aim to that position
            }
            else // if the motor is done and we shouldn't continuously aim
            {
                turretMotor.motorEx.setPower(0); // set the turret power to 0
            }
        }

        if (Math.abs(bearingToTarget().toUnit(AngleUnit.DEGREES).measure) < tolerance.toUnit(AngleUnit.DEGREES).measure)
        {
            turretMotor.motorEx.setPower(0);
        }
    }
}
