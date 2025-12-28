package org.firstinspires.ftc.teamcode.util.measure.coordinate;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

/**
 * Describes the {@link Vector2d#x} and {@link Vector2d#y} components which form a {@link Vector2d}.
 * Enforces the {@link DistanceUnit} of {@link Vector2d#x} and {@link AngleUnit#RADIANS} of {@link Vector2d#x} by default.
 */
public class Vector2d
{
    public final DistanceUnit distUnit;
    public final AngleUnit angUnit;

    public final Distance x;
    public final Distance y;

    public Vector2d(Distance x, Distance y)
    {
        this(x, y, x.unit);
    }

    public Vector2d(Distance x, Distance y, DistanceUnit distUnit)
    {
        this(x, y, distUnit, AngleUnit.RADIANS);
    }

    public Vector2d(Distance x, Distance y, AngleUnit angUnit)
    {
        this(x, y, x.unit, angUnit);
    }

    public Vector2d(Distance x, Distance y, DistanceUnit distUnit, AngleUnit angUnit)
    {
        this.distUnit = distUnit;
        this.angUnit = angUnit;

        // x and y will always be the same set DistanceUnit
        this.x = x.toUnit(distUnit);
        this.y = y.toUnit(distUnit);
    }

    /**
     * @return The {@link Distance} of the {@link Vector2d} vector
     */
    public Distance getDistance()
    {
        return new Distance(Math.hypot(x.magnitude, y.magnitude), distUnit);
    }

    /**
     * @return The {@link Angle} of the {@link Vector2d} vector
     */
    public Angle getDirection()
    {
        Angle angleRads = new Angle(Math.atan2(y.magnitude, x.magnitude), AngleUnit.RADIANS);

        return angleRads.toUnit(angUnit);
    }

    /**
     * @param distanceUnit
     * @return The {@link Vector2d} with {@code x}, {@code y}, and relevant calculations converted to the {@link DistanceUnit}
     */
    public Vector2d toDistanceUnit(DistanceUnit distanceUnit)
    {
        if (isDistanceUnit(distanceUnit))
        {
            return this;
        }

        return new Vector2d(x, y, distanceUnit);
    }

    /**
     * @param angleUnit
     * @return The {@link Vector2d} with relevant calculations converted to the {@link AngleUnit}
     */
    public Vector2d toAngleUnit(AngleUnit angleUnit)
    {
        if (isAngleUnit(angleUnit))
        {
            return this;
        }

        return new Vector2d(x, y, angleUnit);
    }

    /**
     * @return The inversed {@link Vector2d}, where the {@code x} and {@code y} magnitudes are inversed.
     */
    public Vector2d inverse()
    {
        return new Vector2d(new Distance(-x.magnitude, x.unit), new Distance(-y.magnitude, y.unit), distUnit, angUnit);
    }

    /**
     * Calculates this {@link Vector2d} + the other {@link Vector2d}
     * @param b
     * @return The calculated {@link Vector2d}
     */
    public Vector2d plus(Vector2d b)
    {
        Distance xResult = this.x.plus(b.x);
        Distance yResult = this.y.plus(b.y);

        return new Vector2d(xResult, yResult, xResult.unit, this.angUnit);
    }

    /**
     * Calculates this {@link Vector2d} - the other {@link Vector2d}
     * @param b
     * @return The calculated {@link Vector2d}
     */
    public Vector2d minus(Vector2d b)
    {
        return plus(b.inverse());
    }

    public boolean isDistanceUnit(DistanceUnit distanceUnit)
    {
        return (this.distUnit == distanceUnit);
    }

    public boolean isAngleUnit(AngleUnit angleUnit)
    {
        return (this.angUnit == angleUnit);
    }
}