package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.util.measure.angle.field.FieldHeading;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.UnnormalizedAngle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.motors.modern.PositionMotor;

public class Turret extends SubsystemBase
{
    private final PositionMotor motor;
    private final Angle initialRelativeAngle;

    private double targetAngleDegrees;

    /** If true, continuously locks to the target angle in {@link #periodic()}. */
    public boolean lockToPosition;

    public Turret(PositionMotor motor, Angle initialRelativeAngle, boolean shouldLock)
    {
        this.motor = motor;
        this.initialRelativeAngle = initialRelativeAngle;
        this.lockToPosition = shouldLock;

        // Initialize target to current position
        this.targetAngleDegrees = motor.getDistance();
    }

    /**
     * Aims the turret absolutely to a field heading.
     * @param targetAngle The absolute target angle on the field
     * @param robotAngle The robot's current heading
     */
    public void aimAbsolute(Angle targetAngle, Angle robotAngle)
    {
        aimRelative(targetAngle.minus(robotAngle));
    }

    /**
     * Aims the turret relatively to the robot's forward direction.
     * @param targetAngle The desired turret angle relative to the robot
     */
    public void aimRelative(Angle targetAngle)
    {
        // Convert from robot-relative to motor-zero-relative coordinates
        Angle adjustedTarget = targetAngle.minus(initialRelativeAngle);
        double targetDegrees = adjustedTarget.getDegrees();

        // Resolve the best rotation to minimize travel while respecting limits
        targetDegrees = resolveBestRotation(motor.getDistance(), targetDegrees);

        // Clamp to turn limits
        UnnormalizedAngle[] limits = RobotConstants.Turret.TURN_LIMITS;
        double minDegrees = limits[0].getDegrees();
        double maxDegrees = limits[1].getDegrees();
        this.targetAngleDegrees = Math.max(minDegrees, Math.min(maxDegrees, targetDegrees));

        motor.setTargetDistance(this.targetAngleDegrees);
    }

    /**
     * Aims the turret toward a target field coordinate.
     * @param target The target position on the field
     * @param robotPose The robot's current pose
     */
    public void aimToCoordinate(FieldCoordinate target, Pose2d robotPose)
    {
        aimRelative(robotPose.bearingTo(target));
    }

    /** Resets the turret to forward position. */
    public void reset()
    {
        aimRelative(RobotConstants.Turret.FORWARD_ANGLE);
    }

    /**
     * Gets the turret's absolute heading on the field.
     * @param robotHeading The robot's field heading
     * @return The turret's absolute field heading
     */
    public FieldHeading getFieldHeading(FieldHeading robotHeading)
    {
        Angle turretOffset = getRelativeAngle().minus(initialRelativeAngle);
        return new FieldHeading(robotHeading.angle.plus(turretOffset), robotHeading.system);
    }

    /**
     * Gets the turret's current angle relative to the robot's forward.
     * @return The relative turret angle (normalized)
     */
    public Angle getRelativeAngle()
    {
        return getRelativeUnnormalizedAngle().toNormalized();
    }

    /**
     * Gets the turret's current unnormalized angle relative to the robot.
     * @return The relative turret angle (unnormalized)
     */
    public UnnormalizedAngle getRelativeUnnormalizedAngle()
    {
        double currentDegrees = motor.getDistance();
        return initialRelativeAngle.toUnit(UnnormalizedAngleUnit.DEGREES)
                .plus(new UnnormalizedAngle(currentDegrees, UnnormalizedAngleUnit.DEGREES));
    }

    /** @return true if the turret has reached its target angle */
    public boolean isAtTarget()
    {
        return motor.atSetPoint();
    }

    /**
     * Gets the angular difference between the current position and the target.
     * @return The bearing (error) to the target position as an Angle
     */
    public Angle bearingToTarget()
    {
        double errorDegrees = targetAngleDegrees - motor.getDistance();
        return new Angle(errorDegrees, AngleUnit.DEGREES);
    }

    /**
     * Resolves the best rotation (adding/subtracting 360°) to minimize travel within limits.
     */
    private double resolveBestRotation(double currentDegrees, double targetDegrees)
    {
        UnnormalizedAngle[] limits = RobotConstants.Turret.TURN_LIMITS;
        double minDegrees = limits[0].getDegrees();
        double maxDegrees = limits[1].getDegrees();

        double closestN = Math.round((currentDegrees - targetDegrees) / 360.0);
        double bestCandidate = targetDegrees;
        double minDistance = Double.MAX_VALUE;

        for (double n = closestN - 1; n <= closestN + 1; n++)
        {
            double candidate = targetDegrees + (n * 360.0);
            if (candidate >= minDegrees && candidate <= maxDegrees)
            {
                double dist = Math.abs(candidate - currentDegrees);
                if (dist < minDistance)
                {
                    minDistance = dist;
                    bestCandidate = candidate;
                }
            }
        }
        return bestCandidate;
    }

    @Override
    public void periodic()
    {
        if (lockToPosition)
        {
            motor.update();
        }
        else if (motor.atSetPoint())
        {
            motor.stopMotor();
        }
    }
}


