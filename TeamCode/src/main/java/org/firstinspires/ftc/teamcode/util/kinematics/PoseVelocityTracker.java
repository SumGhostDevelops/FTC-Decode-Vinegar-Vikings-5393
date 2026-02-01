package org.firstinspires.ftc.teamcode.util.kinematics;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.field.FieldHeading;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.firstinspires.ftc.teamcode.util.measure.geometry.Vector2d;

import java.util.function.DoubleSupplier;

/**
 * Tracks Pose2d and Velocity data from Pinpoint and computes acceleration.
 * Optimized for low overhead and loop time safety.
 */
public class PoseVelocityTracker
{
    private static final DistanceUnit INTERNAL_DIST_UNIT = DistanceUnit.INCH;
    private static final AngleUnit INTERNAL_ANGLE_UNIT = AngleUnit.RADIANS;

    // Smoothing factor for Acceleration (0.0 = no new data, 1.0 = no smoothing)
    private static final double ACCEL_SMOOTHING = 0.5;

    private double lastTimestamp = 0;

    private double lastVX = 0;
    private double lastVY = 0;
    private double lastVHeading = 0;

    private double lastAX = 0;
    private double lastAY = 0;
    private double lastAHeading = 0;

    private final DoubleSupplier timeSupplier;

    public PoseVelocityTracker(DoubleSupplier timeSupplier)
    {
        this.timeSupplier = timeSupplier;
    }

    /**
     * Updates the tracker with new pose and velocity data.
     * 
     * @param pose
     *            The current Pose2d
     * @param velocity
     *            The current Velocity Vector2d
     * @param headingVelocity
     *            The current Angular Velocity (rad/s preferred)
     */
    public void update(Pose2d pose, Vector2d velocity, double headingVelocity)
    {
        double currentTime = timeSupplier.getAsDouble();
        if (lastTimestamp == 0)
        {
            lastTimestamp = currentTime;
            lastVX = velocity.toDistanceUnit(INTERNAL_DIST_UNIT).x.magnitude;
            lastVY = velocity.toDistanceUnit(INTERNAL_DIST_UNIT).y.magnitude;
            lastVHeading = headingVelocity;
            return;
        }

        double dt = currentTime - lastTimestamp;
        if (dt <= 0)
            return;

        double currentVX = velocity.toDistanceUnit(INTERNAL_DIST_UNIT).x.magnitude;
        double currentVY = velocity.toDistanceUnit(INTERNAL_DIST_UNIT).y.magnitude;
        double currentVHeading = headingVelocity;

        // Calculate raw acceleration
        double rawAX = (currentVX - lastVX) / dt;
        double rawAY = (currentVY - lastVY) / dt;
        double rawAHeading = (currentVHeading - lastVHeading) / dt;

        // Apply Low-Pass Filter
        lastAX = (ACCEL_SMOOTHING * rawAX) + ((1 - ACCEL_SMOOTHING) * lastAX);
        lastAY = (ACCEL_SMOOTHING * rawAY) + ((1 - ACCEL_SMOOTHING) * lastAY);
        lastAHeading = (ACCEL_SMOOTHING * rawAHeading) + ((1 - ACCEL_SMOOTHING) * lastAHeading);

        // Update state
        lastVX = currentVX;
        lastVY = currentVY;
        lastVHeading = currentVHeading;
        lastTimestamp = currentTime;
    }

    /**
     * Predicts the future pose after a given time delta.
     */
    public Pose2d getFuturePose(double seconds, Pose2d currentPose)
    {
        Pose2d normalizedPose = currentPose
                .toDistanceUnit(INTERNAL_DIST_UNIT)
                .toAngleUnit(INTERNAL_ANGLE_UNIT);

        double x = normalizedPose.coord.x.magnitude;
        double y = normalizedPose.coord.y.magnitude;
        double h = normalizedPose.heading.angle.measure;

        double t = seconds;
        double t2 = 0.5 * t * t;

        double futureX = x + lastVX * t + lastAX * t2;
        double futureY = y + lastVY * t + lastAY * t2;
        double futureH = h + lastVHeading * t + lastAHeading * t2;

        return new Pose2d(
                new FieldCoordinate(
                        new Distance(futureX, INTERNAL_DIST_UNIT),
                        new Distance(futureY, INTERNAL_DIST_UNIT),
                        currentPose.coord.coordSys),
                new FieldHeading(new Angle(futureH, INTERNAL_ANGLE_UNIT), currentPose.coord.coordSys));
    }

    public Vector2d getVelocity()
    {
        return new Vector2d(new Distance(lastVX, INTERNAL_DIST_UNIT), new Distance(lastVY, INTERNAL_DIST_UNIT), null);
    }

    /**
     * Predicts the velocity after a given time delta.
     */
    public Vector2d getFutureVelocity(double seconds)
    {
        double t = seconds;
        double futureVX = lastVX + lastAX * t;
        double futureVY = lastVY + lastAY * t;

        return new Vector2d(new Distance(futureVX, INTERNAL_DIST_UNIT), new Distance(futureVY, INTERNAL_DIST_UNIT), null);
    }

    public double getAngularVelocity()
    {
        return lastVHeading;
    }

    public double getAccelerationX()
    {
        return lastAX;
    }

    public double getAccelerationY()
    {
        return lastAY;
    }

    public double getAngularAcceleration()
    {
        return lastAHeading;
    }

    public void reset()
    {
        lastTimestamp = 0;
        lastVX = 0;
        lastVY = 0;
        lastVHeading = 0;
        lastAX = 0;
        lastAY = 0;
        lastAHeading = 0;
    }
}
