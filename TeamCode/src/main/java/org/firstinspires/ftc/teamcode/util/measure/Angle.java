package org.firstinspires.ftc.teamcode.util.measure;

import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/**
 * {@link Angle} represents a normalized angle ({@code [-half circle, half circle)}) and its unit of measure.
 */
public class Angle
{
    public final double angle;
    public final AngleUnit unit;

    public Angle(double angle, AngleUnit unit)
    {
        this.angle = unit.normalize(angle);
        this.unit = unit;
    }

    // Converters
    /**
     * @param newUnit
     * @return The {@link Angle} in the new {@link AngleUnit}
     */
    public Angle toUnit(AngleUnit newUnit)
    {
        if (this.unit == newUnit)
        {
            return this;
        }

        return new Angle(getAngle(newUnit), newUnit);
    }

    /**
     * Converts an {@link Angle} to an {@link UnnormalizedAngle}
     * @param newUnit
     * @return The {@link UnnormalizedAngle} in the new {@link UnnormalizedAngleUnit}
     */
    public UnnormalizedAngle toUnit(UnnormalizedAngleUnit newUnit)
    {
        UnnormalizedAngleUnit unnormalizedUnit = unit.getUnnormalized(); // get unnormalized unit
        double unnormalizedAngle = unnormalizedUnit.fromUnit(this.unit, this.angle); // convert to unnormalized angle

        return new UnnormalizedAngle(unnormalizedAngle, unnormalizedUnit).toUnit(newUnit); // create unnormalizedangle, convert to final unit
    }

    /**
     * @see #toUnit(UnnormalizedAngleUnit)
     * @return The {@link UnnormalizedAngle} in the {@link Angle}'s equivalent {@link UnnormalizedAngleUnit}
     */
    public UnnormalizedAngle toUnnormalized()
    {
        return toUnit(getUnnormalizedUnit(this.unit));
    }

    /**
     * @param newUnit
     * @return The quantity of the {@link Angle} in the specified {@link AngleUnit}
     */
    public double getAngle(AngleUnit newUnit)
    {
        if (this.unit == newUnit)
        {
            return this.angle;
        }

        return newUnit.fromUnit(this.unit, angle);
    }

    /**
     * Converts the {@link Angle}'s signed quantity to an unsigned quantity [0, 360) / [0, 2Pi)
     * @return The unsigned quantity of the {@link Angle} in the {@link Angle}'s {@link AngleUnit}
     */
    public double getUnsignedAngle()
    {
        return getUnsignedAngle(this.unit);
    }

    /**
     * Converts the {@link Angle}'s signed quantity to an unsigned quantity [0, 360) / [0, 2Pi)
     * @param newUnit
     * @return The unsigned quantity of the {@link Angle} in the specified {@link AngleUnit}
     */
    public double getUnsignedAngle(AngleUnit newUnit)
    {
        double convertedAngle = getAngle(newUnit);
        return MathUtils.normalizeAngle(convertedAngle, true, newUnit);
    }

    // Calculations

    /**
     * Calculates this {@link Angle} + another {@link Angle}
     * @param b The second {@link Angle}
     * @return The resulting {@link Angle}, in this {@link Angle}'s {@link AngleUnit}
     */
    public Angle plus(Angle b)
    {
        double bAngle = b.getAngle(this.unit);

        return new Angle(this.angle + bAngle, this.unit);
    }

    /**
     * Calculates this {@link Angle} - another {@link Angle}
     * @param b The second {@link Angle}
     * @return The resulting {@link Angle}, in this {@link Angle}'s {@link AngleUnit}
     */
    public Angle minus(Angle b)
    {
        double bAngle = b.getAngle(this.unit);

        return new Angle(this.angle - bAngle, this.unit);
    }

    /**
     * @see AngleUnit#getUnnormalized()
     * @param angleUnit
     * @return The unnormalized {@link UnnormalizedAngleUnit} version of the {@link AngleUnit}
     */
    public static UnnormalizedAngleUnit getUnnormalizedUnit(AngleUnit angleUnit)
    {
        return angleUnit.getUnnormalized();
    }
}