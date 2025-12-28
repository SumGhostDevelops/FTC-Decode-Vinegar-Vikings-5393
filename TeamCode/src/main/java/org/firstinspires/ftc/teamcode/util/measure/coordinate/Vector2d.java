package org.firstinspires.ftc.teamcode.util.measure.coordinate;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

/**
 * Describes the {@code x} and {@code y} components which form a {@link Vector2d}.
 * {@code DistanceUnit.METER} and {@code AngleUnit.RADIANS} by default,
 * and there shouldn't be any reason to change them (because you can always change their unit after the fact), though you have the option to.
 */
public class Vector2d
{
    public final DistanceUnit distUnit;
    public final AngleUnit angUnit;

    public final Distance x;
    public final Distance y;

    public Vector2d(Distance x, Distance y)
    {
        this(x, y, DistanceUnit.METER);
    }

    public Vector2d(Distance x, Distance y, DistanceUnit distUnit)
    {
        this(x, y, distUnit, AngleUnit.RADIANS);
    }

    public Vector2d(Distance x, Distance y, AngleUnit angUnit)
    {
        this(x, y, DistanceUnit.METER, angUnit);
    }

    public Vector2d(Distance x, Distance y, DistanceUnit distUnit, AngleUnit angUnit)
    {
        this.distUnit = distUnit;
        this.angUnit = angUnit;

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
    public Angle getAngle()
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
        return new Vector2d(x, y, distanceUnit);
    }

    /**
     * @param angleUnit
     * @return The {@link Vector2d} with relevant calculations converted to the {@link AngleUnit}
     */
    public Vector2d toAngleUnit(AngleUnit angleUnit)
    {
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
}