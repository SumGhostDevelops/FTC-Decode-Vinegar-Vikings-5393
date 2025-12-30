package org.firstinspires.ftc.teamcode.util.measure.distance;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * {@link Distance} represents a distance and its unit of measure.
 */
public class Distance
{
    public final double magnitude;
    public final DistanceUnit unit;

    public Distance(double magnitude, DistanceUnit unit)
    {
        this.magnitude = magnitude;
        this.unit = unit;
    }

    /**
     * @param newUnit
     * @return The {@link Distance} in the new {@link DistanceUnit}
     */
    public Distance toUnit(DistanceUnit newUnit)
    {
        if (isUnit(newUnit))
        {
            return this;
        }

        return new Distance(getDistance(newUnit), newUnit);
    }

    /**
     * @param newUnit
     * @return The quantity of the {@link Distance} in the specified {@link DistanceUnit}
     */
    public double getDistance(DistanceUnit newUnit)
    {
        return newUnit.fromUnit(this.unit, magnitude);
    }

    /**
     * Calculates this {@link Distance} + another {@link Distance}
     * @param b The second Distance
     * @return The resulting {@link Distance}, in this {@link Distance}'s {@link DistanceUnit}
     */
    public Distance plus(Distance b)
    {
        double bDist = b.getDistance(this.unit);

        return new Distance(this.magnitude + bDist, this.unit);
    }

    /**
     * Calculates this {@link Distance} - another {@link Distance}
     * @param b The second Distance
     * @return The resulting {@link Distance}, in this {@link Distance}'s {@link DistanceUnit}
     */
    public Distance minus(Distance b)
    {
        double bDist = b.getDistance(this.unit);

        return new Distance(this.magnitude - bDist, this.unit);
    }

    /**
     * Calculates this {@link Distance} * a scalar
     * @param scalar The scalar to multiply by
     * @return The resulting {@link Distance}
     */
    public Distance multiply(double scalar)
    {
        return new Distance(this.magnitude * scalar, this.unit);
    }

    /**
     * Calculates this {@link Distance} / a scalar
     * @param scalar The scalar to divide by
     * @return The resulting {@link Distance}
     */
    public Distance divide(double scalar)
    {
        return new Distance(this.magnitude / scalar, this.unit);
    }

    public boolean isUnit(DistanceUnit distanceUnit)
    {
        return (this.unit == distanceUnit);
    }

    private Distance toComparableStandard()
    {
        return this.toUnit(DistanceUnit.METER);
    }

    public boolean equals(Distance otherDist)
    {
        return compareTo(otherDist) == 0;
    }

    public double compareTo(Distance otherDist)
    {
        return this.toComparableStandard().magnitude - otherDist.toComparableStandard().magnitude;
    }

    @Override
    public String toString()
    {
        return unit.toString(magnitude);
    }
}