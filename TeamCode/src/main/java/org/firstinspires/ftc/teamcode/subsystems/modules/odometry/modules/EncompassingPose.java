package org.firstinspires.ftc.teamcode.subsystems.modules.odometry.modules;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.modules.odometry.Odometry;

public class EncompassingPose
{
    public final Pose2D position;
    public final Pose2D velocity;
    public final Pose2D acceleration;
    public final double heading;
    public final long timestampSec;

    public enum AngleType
    {
        SIGNED, // -180 to 180 or -pi to pi
        UNSIGNED, // 0 to 360 or 0 to 2pi
    }

    /**
     * Stores state in base units (meters, radians, seconds).
     */
    public EncompassingPose(Pose2D position, Pose2D velocity, Pose2D acceleration, double heading, long timestampSec)
    {
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.heading = heading;
        this.timestampSec = timestampSec;
    }

    public Pose2D getPositionPose(DistanceUnit distanceUnit, AngleUnit angleUnit)
    {
        return convert(position, distanceUnit, angleUnit);
    }

    public Pose2D getVelocityPose(DistanceUnit distanceUnit, AngleUnit angleUnit)
    {
        return convert(velocity, distanceUnit, angleUnit);
    }

    public Pose2D getAccelerationPose(DistanceUnit distanceUnit, AngleUnit angleUnit)
    {
        return convert(acceleration, distanceUnit, angleUnit);
    }

    public double getVelocityResultant(DistanceUnit distanceUnit)
    {
        return Math.hypot(velocity.getX(distanceUnit), velocity.getY(distanceUnit));
    }

    public double getAccelerationResultant(DistanceUnit distanceUnit)
    {
        return Math.hypot(acceleration.getX(distanceUnit), acceleration.getY(distanceUnit));
    }

    public double getVelocityAngle(AngleUnit angleUnit, AngleType angleType)
    {
        return convertAngle(velocity.getHeading(angleUnit), angleUnit, angleType);
    }

    public double getAccelerationAngle(AngleUnit angleUnit, AngleType angleType)
    {
        return convertAngle(acceleration.getHeading(angleUnit), angleUnit, angleType);
    }

    public double getHeading(AngleUnit angleUnit, AngleType angleType)
    {
        return convertAngle(heading, angleUnit, angleType);
    }

    private Pose2D convert(Pose2D pose, DistanceUnit distanceUnit, AngleUnit angleUnit)
    {
        return new Pose2D(distanceUnit,
                distanceUnit.fromMeters(pose.getX(DistanceUnit.METER)),
                distanceUnit.fromMeters(pose.getY(DistanceUnit.METER)),
                angleUnit,
                angleUnit.fromRadians(pose.getHeading(AngleUnit.RADIANS))
        );
    }

    private double convertAngle(double angle, AngleUnit angleUnit, AngleType angleType)
    {
        // Protect against NaN/Infinite
        if (Double.isNaN(angle) || Double.isInfinite(angle)) return angle;

        /**
         * double fullCircle;
         * if (angleUnit == AngleUnit.DEGREES)
         * {
         *     fullCircle = 360.0;
         * }
         * else
         * {
         *     fullCircle = 2.0 * Math.PI;
         * }
         */

        double fullCircle = (angleUnit == AngleUnit.DEGREES) ? 360.0 : 2.0 * Math.PI;

        if (angleType == AngleType.SIGNED)
        {
            // Pose2D already provides a normalized signed heading; return as-is to preserve
            // boundary behavior (e.g. -180 vs +180 as Pose2D defines it).
            return angle;
        }
        else // UNSIGNED
        {
            // Convert signed heading to unsigned [0, fullCircle).
            double unsigned = angle;
            if (unsigned < 0)
            {
                unsigned += fullCircle;
            }
            // Guard against inputs outside expected range; ensure in [0, fullCircle)
            unsigned = ((unsigned % fullCircle) + fullCircle) % fullCircle;
            return unsigned;
        }
    }
}