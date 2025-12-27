package org.firstinspires.ftc.teamcode.util.measure;

import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

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
     * Converts THIS angle to the final Unnormalized unit
     * @param finalUnit
     * @return The final UnnormalizedAngle
     */
    public UnnormalizedAngle toUnit(UnnormalizedAngleUnit finalUnit)
    {
        UnnormalizedAngleUnit unnormalizedUnit = unit.getUnnormalized(); // get unnormalized unit
        double unnormalizedAngle = unnormalizedUnit.fromUnit(this.unit, this.angle); // convert to unnormalized angle

        return new UnnormalizedAngle(unnormalizedAngle, unnormalizedUnit).toUnit(finalUnit); // create unnormalizedangle, convert to final unit
    }

    /**
     * Converts THIS angle to the final unit
     * @param finalUnit
     * @return The final angle (quantity)
     */
    public double toUnitOnlyAngle(AngleUnit finalUnit)
    {
        return finalUnit.fromUnit(this.unit, angle);
    }

    /**
     * Converts THIS angle to the normalized unnormalized [0, 360) / [0, 2pi) angle and final unit
     * @param finalUnit
     * @return The final normalized unnormalized angle (quantity) in the specified unit
     */
    public double toNormalUnnormal(AngleUnit finalUnit)
    {
        double convertedAngle = toUnitOnlyAngle(finalUnit);
        return MathUtils.normalizeAngle(convertedAngle, true, finalUnit);
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

    // Convenience aliases

    /**
     * Calculates THIS angle + b
     * @param b The second Angle
     * @return The result in THIS angle's unit
     */
    public Angle plus(Angle b)
    {
        return add(b);
    }

    /**
     * Calculates THIS angle + b
     * @param b The second Angle
     * @param finalUnit The unit to return the result in
     * @return The result in the specified unit
     */
    public Angle plus(Angle b, AngleUnit finalUnit)
    {
        return add(b, finalUnit);
    }

    /**
     * Calculates THIS angle - b
     * @param b The second angle
     * @return The result in THIS angle's unit
     */
    public Angle minus(Angle b)
    {
        return subtract(b);
    }

    /**
     * Calculates THIS angle - b
     * @param b The second angle
     * @param finalUnit The unit to return the result in
     * @return The result in the specified unit
     */
    public Angle minus(Angle b, AngleUnit finalUnit)
    {
        return subtract(b, finalUnit);
    }
}