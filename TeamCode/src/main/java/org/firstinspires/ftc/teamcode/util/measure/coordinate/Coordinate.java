package org.firstinspires.ftc.teamcode.util.measure.coordinate;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.firstinspires.ftc.teamcode.util.measure.geometry.Vector2d;

/**
 * Describes a generic {@link Coordinate}, with methods to compare it to other {@link Coordinate}s
 */
public class Coordinate
{
    public final Distance x;
    public final Distance y;

    public Coordinate(Distance x, Distance y)
    {
        this.x = x;
        this.y = y;
    }

    /**
     * Returns the vector between the two coordinates. Assumes that they are in the same system.
     *
     * @param otherCoord
     * @return The {@link Vector2d} vector to the other {@link Coordinate}
     */
    public Vector2d vectorTo(Coordinate otherCoord)
    {
        Distance dx = otherCoord.x.minus(this.x).toUnit(DistanceUnit.METER);
        Distance dy = otherCoord.y.minus(this.y).toUnit(DistanceUnit.METER);

        // use GENERIC system for raw vectors
        return new Vector2d(dx, dy, CoordinateSystem.GENERIC);
    }

    /**
     * @param otherCoord
     * @return The {@link Distance} to the other {@link Coordinate}
     */
    public Distance distanceTo(Coordinate otherCoord)
    {
        return vectorTo(otherCoord).getLength();
    }

    /**
     * @param otherCoord
     * @return The {@link Angle} to the other {@link Coordinate}
     */
    public Angle angleTo(Coordinate otherCoord)
    {
        return vectorTo(otherCoord).getDirection();
    }

    /**
     * @param distanceUnit
     * @return The converted {@link Coordinate} with the specified {@link DistanceUnit}
     */
    public Coordinate toDistanceUnit(DistanceUnit distanceUnit)
    {
        return new Coordinate(x.toUnit(distanceUnit), y.toUnit(distanceUnit));
    }

    /**
     * @param translation
     * @return The translated {@link Coordinate}
     */
    public Coordinate translate(Vector2d translation)
    {
        Distance newX = x.plus(translation.x);
        Distance newY = y.plus(translation.y);

        return new Coordinate(newX, newY);
    }

    /**
     * @param distanceUnit
     * @return If both {@link Distance}s are in the specified {@link DistanceUnit}
     */
    public boolean isDistanceUnit(DistanceUnit distanceUnit)
    {
        return this.x.unit == distanceUnit && this.y.unit == distanceUnit;
    }

    @Override
    public boolean equals(Object obj)
    {
        if (this == obj)
        {
            return true;
        }

        if (obj == null || getClass() != obj.getClass())
        {
            return false;
        }

        Coordinate otherCoord = (Coordinate) obj;
        return this.x.equals(otherCoord.x) && this.y.equals(otherCoord.y);
    }

    @Override
    public String toString()
    {
        return "x=" + x.toString() + ", y=" + y.toString();
    }
}