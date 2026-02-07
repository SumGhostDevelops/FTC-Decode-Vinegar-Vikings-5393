package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.util.measure.angle.field.FieldHeading;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.UnnormalizedAngle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.firstinspires.ftc.teamcode.util.motors.PositionMotor;

import java.util.function.DoubleSupplier;

public class Turret extends SubsystemBase
{
    private final PositionMotor motor;
    private final Angle initialRelativeAngle;
    private State state = State.ON;

    private final double gearRatio = RobotConstants.Turret.GEAR_RATIO;
    private final Angle forwardAngle = RobotConstants.Turret.FORWARD_ANGLE;
    private final Angle tolerance = RobotConstants.Turret.TOLERANCE;
    private final Distance linearTolerance = RobotConstants.Turret.LINEAR_TOLERANCE;
    private final boolean useDynamicTolerance = RobotConstants.Turret.USE_DYNAMIC_TOLERANCE;
    private final boolean rotationCompensationEnabled = RobotConstants.Turret.ROTATION_COMPENSATION_ENABLED;
    private final double rotationCompensationFF = RobotConstants.Turret.ROTATION_COMPENSATION_FF;
    private final UnnormalizedAngle[] turnLimits = RobotConstants.Turret.TURN_LIMITS;
    private final Angle safetyMargin = RobotConstants.Turret.SAFETY_MARGIN;

    private double targetAngleDegrees;

    // Add a variable to track the active tolerance, defaulting to the local
    // tolerance
    private double currentToleranceDegrees = tolerance.getDegrees();

    // Supplier for robot angular velocity (deg/s) for feedforward compensation
    private DoubleSupplier angularVelocitySupplier = () -> 0.0;

    public enum State { ON, OFF }

    public Turret(PositionMotor motor, Angle initialRelativeAngle)
    {
        this.motor = motor;
        this.initialRelativeAngle = initialRelativeAngle;

        // Initialize target to current position
        this.targetAngleDegrees = motor.getDistance();
    }

    public void setState(State state)
    {
        this.state = state;
    }

    public State getState()
    {
        return this.state;
    }

    /**
     * Sets the angular velocity supplier for rotation compensation.
     * Configures the motor's feedforward to counteract robot rotation.
     * 
     * @param supplier
     *            A supplier that returns robot angular velocity in deg/s
     */
    public void setAngularVelocitySupplier(DoubleSupplier supplier)
    {
        this.angularVelocitySupplier = supplier != null ? supplier : () -> 0.0;

        // Configure motor feedforward for rotation compensation
        // When robot rotates at ω deg/s, turret motor needs -ω × GEAR_RATIO × gain
        // power
        motor.setFeedforwardSupplier(() ->
        {
            if (!rotationCompensationEnabled)
            {
                return 0.0;
            }
            double robotAngularVelocity = angularVelocitySupplier.getAsDouble();

            // Negative because turret must counter-rotate against robot rotation
            return -robotAngularVelocity * gearRatio * rotationCompensationFF;
        });
    }

    /**
     * Aims the turret absolutely to a field heading.
     * Resets tolerance to the default fixed value.
     */
    public void aimAbsolute(Angle targetAngle, Angle robotAngle)
    {
        aimRelative(targetAngle.minus(robotAngle));
    }

    /**
     * Aims the turret relatively to the robot's forward direction.
     * Resets tolerance to the default fixed value.
     */
    public void aimRelative(Angle targetAngle)
    {
        // Reset to default tolerance when using standard aiming
        this.currentToleranceDegrees = tolerance.getDegrees();
        setTargetRelative(targetAngle);
    }

    /**
     * Internal helper to handle the motor math.
     * Extracted from aimRelative so we can call it without resetting tolerance.
     */
    private void setTargetRelative(Angle targetAngle)
    {
        // Convert from robot-relative to motor-zero-relative coordinates
        Angle adjustedTarget = targetAngle.minus(initialRelativeAngle);
        double targetDegrees = adjustedTarget.getDegrees();

        // Resolve the best rotation to minimize travel while respecting limits
        targetDegrees = resolveBestRotation(motor.getDistance(), targetDegrees);

        // Clamp to turn limits
        double minDegrees = turnLimits[0].getDegrees();
        double maxDegrees = turnLimits[1].getDegrees();
        this.targetAngleDegrees = Math.max(minDegrees, Math.min(maxDegrees, targetDegrees));

        motor.setTargetDistance(this.targetAngleDegrees);
    }

    private double resolveBestRotation(double currentDegrees, double targetDegrees)
    {
        double minDegrees = turnLimits[0].getDegrees();
        double maxDegrees = turnLimits[1].getDegrees();

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

    /**
     * Aims the turret toward a target coordinate with dynamic tolerance.
     * Tolerance tightens as the robot gets farther away, maintaining a virtual "hit
     * box" size.
     * * @param target The target position on the field
     *
     * @param robotPose
     *            The robot's current pose
     * @param linearToleranceRadius
     *            The allowable error radius at the target (e.g.,
     *            3 inches)
     * @param minAngularTolerance
     *            The minimum angle (degrees) the tolerance can
     *            shrink to
     */
    public void aimToCoordinate(FieldCoordinate target, Pose2d robotPose, Distance linearToleranceRadius,
            Angle minAngularTolerance)
    {
        if (!useDynamicTolerance)
        {
            aimToCoordinate(target, robotPose);
            return;
        }

        // Calculate distance to target
        double distance = robotPose.coord.distanceTo(target).getInch();

        // Calculate dynamic tolerance (Inverse Tangent of radius / distance)
        if (distance > 1e-6)
        { // Avoid division by zero
            double dynamicTolerance = Math.toDegrees(Math.atan2(linearToleranceRadius.getInch(), distance));
            // Ensure we don't demand impossible precision
            this.currentToleranceDegrees = Math.max(dynamicTolerance, minAngularTolerance.getDegrees());
        } else
        {
            // If we are ON the target, tolerance is effectively infinite
            this.currentToleranceDegrees = 180.0;
        }

        // Aim using the calculated bearing without resetting the tolerance we just set
        setTargetRelative(robotPose.bearingTo(target));
    }

    /**
     * Overload for aimToCoordinate that defaults to standard behavior if no MOA
     * params are provided.
     */
    public void aimToCoordinate(FieldCoordinate target, Pose2d robotPose)
    {
        aimRelative(robotPose.bearingTo(target));
    }

    public void reset()
    {
        aimRelative(forwardAngle);
    }

    public FieldHeading getFieldHeading(FieldHeading robotHeading)
    {
        Angle turretOffset = getRelativeAngle().minus(initialRelativeAngle);
        return new FieldHeading(robotHeading.angle.plus(turretOffset), robotHeading.system);
    }

    public Angle getRelativeAngle()
    {
        return getRelativeUnnormalizedAngle().toNormalized();
    }

    public UnnormalizedAngle getRelativeUnnormalizedAngle()
    {
        double currentDegrees = motor.getDistance();
        return initialRelativeAngle.toUnit(UnnormalizedAngleUnit.DEGREES)
                .plus(new UnnormalizedAngle(currentDegrees, UnnormalizedAngleUnit.DEGREES));
    }

    public double getDistance()
    {
        return motor.getDistance();
    }

    public double getTargetAngleDegrees()
    {
        return targetAngleDegrees;
    }

    /**
     * @return true if the turret has reached its target angle within the current
     *         tolerance
     */
    public boolean isAtTarget()
    {
        // Use the active dynamic tolerance instead of the static constant
        return Math.abs(bearingToTarget().getDegrees()) < this.currentToleranceDegrees;
    }

    public boolean exceedingTurnLimits()
    {
        double currentDegrees = motor.getDistance();
        double minDegrees = turnLimits[0].getDegrees();
        double maxDegrees = turnLimits[1].getDegrees();
        double safetyMarginDegrees = safetyMargin.getDegrees();

        // If we are significantly outside the limits, stop the motor immediately to
        // prevent damage
        return (currentDegrees < minDegrees - safetyMarginDegrees) || (currentDegrees > maxDegrees + safetyMarginDegrees);
    }

    public Angle bearingToTarget()
    {
        double errorDegrees = targetAngleDegrees - motor.getDistance();
        return new Angle(errorDegrees, AngleUnit.DEGREES);
    }

    public Angle getTolerance()
    {
        return new Angle(currentToleranceDegrees, AngleUnit.DEGREES);
    }

    @Override
    public void periodic()
    {
        // If we are significantly outside the limits, stop the motor immediately to
        // prevent damage
        if (exceedingTurnLimits() || state == State.OFF)
        {
            motor.stopMotor();
            return;
        }

        motor.setPositionTolerance(currentToleranceDegrees);

        // Always update motor to apply feedforward compensation for robot rotation
        // even when at target position (to maintain position during rotation)
        motor.update();
    }
}