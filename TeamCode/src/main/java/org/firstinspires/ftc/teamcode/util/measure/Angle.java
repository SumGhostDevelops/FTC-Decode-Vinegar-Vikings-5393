package org.firstinspires.ftc.teamcode.util.measure;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Angle
{
    public final double angle;
    public final AngleUnit unit;

    public Angle(double angle, AngleUnit unit)
    {
        this.angle = unit.normalize(angle);
        this.unit = unit;
    }

    /**
     * Converts THIS angle to the final unit
     * @param finalUnit
     * @return The final Angle
     */
    public Angle toUnit(AngleUnit finalUnit)
    {
        if (this.unit == finalUnit)
        {
            return this;
        }

        return new Angle(toUnitOnlyAngle(finalUnit), finalUnit);
    }

    /**
     * Converts THIS angle to the final unit
     * @param finalUnit
     * @return This final angle (quantity)
     */
    public double toUnitOnlyAngle(AngleUnit finalUnit)
    {
        return finalUnit.fromUnit(this.unit, angle);
    }

    // These calculations may need to be optimized because of the number of conversion steps

    /**
     * Calculates THIS angle + b
     * @param b The second Angle
     * @return The result in THIS angle's unit
     */
    public Angle add(Angle b)
    {
        return add(b, this.unit);
    }

    /**
     * Calculates THIS angle + b
     * @param b The second Angle
     * @param finalUnit The unit to return the result in
     * @return The result in the specified unit
     */
    public Angle add(Angle b, AngleUnit finalUnit)
    {
        double aAngle = this.toUnitOnlyAngle(finalUnit);
        double bAngle = b.toUnitOnlyAngle(finalUnit);

        return new Angle(aAngle + bAngle, finalUnit);
    }

    /**
     * Calculates THIS angle - b
     * @param b The second angle
     * @return The result in THIS angle's unit
     */
    public Angle subtract(Angle b)
    {
        return subtract(b, this.unit);
    }

    /**
     * Calculates THIS angle - b
     * @param b The second angle
     * @param finalUnit The unit to return the result in
     * @return The result in the specified unit
     */
    public Angle subtract(Angle b, AngleUnit finalUnit)
    {
        double aAngle = this.toUnitOnlyAngle(finalUnit);
        double bAngle = b.toUnitOnlyAngle(finalUnit);

        return new Angle(aAngle - bAngle, finalUnit);
    }
}