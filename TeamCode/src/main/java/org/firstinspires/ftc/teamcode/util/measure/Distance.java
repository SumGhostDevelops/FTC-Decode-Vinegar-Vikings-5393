package org.firstinspires.ftc.teamcode.util.measure;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Distance
{
    public final double distance;
    public final DistanceUnit unit;

    public Distance(double distance, DistanceUnit unit)
    {
        this.distance = distance;
        this.unit = unit;
    }

    /**
     * Converts THIS distance to the final unit
     * @param finalUnit
     * @return The final Distance
     */
    public Distance toUnit(DistanceUnit finalUnit)
    {
        if (this.unit == finalUnit)
        {
            return this;
        }

        return new Distance(toUnitOnlyDistance(finalUnit), finalUnit);
    }

    /**
     * Converts THIS distance to the final unit
     * @param finalUnit
     * @return The final distance (quantity)
     */
    public double toUnitOnlyDistance(DistanceUnit finalUnit)
    {
        return finalUnit.fromUnit(this.unit, distance);
    }

    /**
     * Calculates THIS distance + b
     * @param b The second Distance
     * @return The result in THIS distance's unit
     */
    public Distance add(Distance b)
    {
        return add(b, this.unit);
    }

    /**
     * Calculates THIS distance + b
     * @param b The second Distance
     * @param finalUnit The unit to return the result in
     * @return The result in the specified unit
     */
    public Distance add(Distance b, DistanceUnit finalUnit)
    {
        double aDist = this.toUnitOnlyDistance(finalUnit);
        double bDist = b.toUnitOnlyDistance(finalUnit);

        return new Distance(aDist + bDist, finalUnit);
    }

    /**
     * Calculates THIS distance - b
     * @param b The second Distance
     * @return The result in the specified unit
     */
    public Distance subtract(Distance b)
    {
        return subtract(b, this.unit);
    }

    /**
     * Calculates THIS distance - b
     * @param b The second Distance
     * @param finalUnit The unit to return the result in
     * @return The result in the specified unit
     */
    public Distance subtract(Distance b, DistanceUnit finalUnit)
    {
        double aDist = this.toUnitOnlyDistance(finalUnit);
        double bDist = b.toUnitOnlyDistance(finalUnit);

        return new Distance(aDist - bDist, finalUnit);
    }

    public Distance multiply(double scalar)
    {
        return new Distance(this.distance * scalar, this.unit);
    }
}