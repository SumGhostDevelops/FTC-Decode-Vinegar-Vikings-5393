package org.firstinspires.ftc.teamcode.util.measure.distance;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * {@link Distance} represents a distance and its unit of measure.
 */
public class Distance
{
    public static final Distance ZERO = new Distance(0, DistanceUnit.INCH);

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

    public double getMM()
    {
        return getDistance(DistanceUnit.MM);
    }

    public double getCM()
    {
        return getDistance(DistanceUnit.CM);
    }

    public double getInch()
    {
        return getDistance(DistanceUnit.INCH);
    }

    public double getMeter()
    {
        return getDistance(DistanceUnit.METER);
    }

    /**
     * Calculates this {@link Distance} + another {@link Distance}
     *
     * @param b The second Distance
     * @return The resulting {@link Distance}, in this {@link Distance}'s {@link DistanceUnit}
     */
    public Distance plus(Distance b)
    {
        if (b.magnitude == 0) return this;

        double bDist = b.getDistance(this.unit);
        return new Distance(this.magnitude + bDist, this.unit);
    }

    /**
     * Calculates this {@link Distance} - another {@link Distance}
     *
     * @param b The second Distance
     * @return The resulting {@link Distance}, in this {@link Distance}'s {@link DistanceUnit}
     */
    public Distance minus(Distance b)
    {
        if (b.magnitude == 0) return this;

        double bDist = b.getDistance(this.unit);

        return new Distance(this.magnitude - bDist, this.unit);
    }

    /**
     * Calculates this {@link Distance} * a scalar
     *
     * @param scalar The scalar to multiply by
     * @return The resulting {@link Distance}
     */
    public Distance multiply(double scalar)
    {
        return new Distance(this.magnitude * scalar, this.unit);
    }

    /**
     * Calculates this {@link Distance} / a scalar
     *
     * @param scalar The scalar to divide by
     * @return The resulting {@link Distance}
     * @throws IllegalArgumentException if {@code scalar} is zero
     */
    public Distance divide(double scalar)
    {
        if (scalar == 0.0)
        {
            throw new IllegalArgumentException("Scalar divisor must not be zero.");
        }
        return new Distance(this.magnitude / scalar, this.unit);
    }

    public Distance negative()
    {
        return multiply(-1.0);
    }

    public boolean isUnit(DistanceUnit distanceUnit)
    {
        return (this.unit == distanceUnit);
    }

    private Distance toComparableStandard()
    {
        return this.toUnit(DistanceUnit.METER);
    }

    @Override
    public boolean equals(Object otherDist)
    {
        if (!(otherDist instanceof Distance))
        {
            return false;
        }

        return Math.abs(compareTo((Distance) otherDist)) < 1e-9;
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