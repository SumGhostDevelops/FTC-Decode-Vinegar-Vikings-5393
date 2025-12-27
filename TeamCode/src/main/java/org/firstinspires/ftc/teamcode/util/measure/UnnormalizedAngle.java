package org.firstinspires.ftc.teamcode.util.measure;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class UnnormalizedAngle
{
    public final double angle;
    public final UnnormalizedAngleUnit unit;

    public UnnormalizedAngle(double angle, UnnormalizedAngleUnit unit)
    {
        this.angle = angle;
        this.unit = unit;
    }

    /**
     * Converts THIS angle to the final unit
     * @param finalUnit
     * @return The final Angle
     */
    public UnnormalizedAngle toUnit(UnnormalizedAngleUnit finalUnit)
    {
        if (this.unit == finalUnit)
        {
            return this;
        }

        return new UnnormalizedAngle(toUnitOnlyAngle(finalUnit), finalUnit);
    }

    /**
     * Converts THIS angle to the final normalized unit
     * @param finalUnit
     * @return The final normalized Angle
     */
    public Angle toNormalizedAngle(AngleUnit finalUnit)
    {
        AngleUnit normalizedUnit = this.unit.getNormalized(); // get the normalized version of the current unit
        Angle normalized = new Angle(this.angle, normalizedUnit); // create a new angle, constructor handles normalization

        return normalized.toUnit(finalUnit);
    }

    /**
     * Converts THIS angle to the final unit
     * @param finalUnit
     * @return This final angle (quantity)
     */
    public double toUnitOnlyAngle(UnnormalizedAngleUnit finalUnit)
    {
        return finalUnit.fromUnit(this.unit, angle);
    }

    /**
     * Calculates THIS angle + b
     * @param b The second Angle
     * @return The result in THIS angle's unit
     */
    public UnnormalizedAngle add(UnnormalizedAngle b)
    {
        return add(b, this.unit);
    }

    /**
     * Calculates THIS angle + b
     * @param b The second Angle
     * @param finalUnit The unit to return the result in
     * @return The result in the specified unit
     */
    public UnnormalizedAngle add(UnnormalizedAngle b, UnnormalizedAngleUnit finalUnit)
    {
        double aAngle = this.toUnitOnlyAngle(finalUnit);
        double bAngle = b.toUnitOnlyAngle(finalUnit);

        return new UnnormalizedAngle(aAngle + bAngle, finalUnit);
    }

    /**
     * Calculates THIS angle - b
     * @param b The second angle
     * @return The result in THIS angle's unit
     */
    public UnnormalizedAngle subtract(UnnormalizedAngle b)
    {
        return subtract(b, this.unit);
    }

    /**
     * Calculates THIS angle - b
     * @param b The second angle
     * @param finalUnit The unit to return the result in
     * @return The result in the specified unit
     */
    public UnnormalizedAngle subtract(UnnormalizedAngle b, UnnormalizedAngleUnit finalUnit)
    {
        double aAngle = this.toUnitOnlyAngle(finalUnit);
        double bAngle = b.toUnitOnlyAngle(finalUnit);

        return new UnnormalizedAngle(aAngle - bAngle, finalUnit);
    }
}
