package org.firstinspires.ftc.teamcode.util.measure.angle;

import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;

/**
 * {@link Angle} represents a normalized angle ({@code [-half circle, half circle)}) and its unit of measure.
 */
public class Angle
{
    public final double measure;
    public final AngleUnit unit;

    public Angle(double measure, AngleUnit unit)
    {
        this.measure = unit.normalize(measure);
        this.unit = unit;
    }

    // Converters

    /**
     * @param angleUnit The angle unit to get the unnormalized version of
     * @return The unnormalized {@link UnnormalizedAngleUnit} version of the {@link AngleUnit}
     * @see AngleUnit#getUnnormalized()
     */
    public static UnnormalizedAngleUnit getUnnormalizedUnit(AngleUnit angleUnit)
    {
        return angleUnit.getUnnormalized();
    }

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
     *
     * @param newUnit
     * @return The {@link UnnormalizedAngle} in the new {@link UnnormalizedAngleUnit}
     */
    public UnnormalizedAngle toUnit(UnnormalizedAngleUnit newUnit)
    {
        UnnormalizedAngleUnit unnormalizedUnit = unit.getUnnormalized(); // get unnormalized unit
        double unnormalizedAngle = unnormalizedUnit.fromUnit(this.unit, this.measure); // convert to unnormalized angle

        return new UnnormalizedAngle(unnormalizedAngle, unnormalizedUnit).toUnit(newUnit); // create unnormalizedangle, convert to final unit
    }

    /**
     * @return The {@link UnnormalizedAngle} in the {@link Angle}'s equivalent {@link UnnormalizedAngleUnit}
     * @see #toUnit(UnnormalizedAngleUnit)
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
            return this.measure;
        }

        return newUnit.fromUnit(this.unit, measure);
    }

    public double getDegrees()
    {
        return getAngle(AngleUnit.DEGREES);
    }

    public double getRadians()
    {
        return getAngle(AngleUnit.RADIANS);
    }

    /**
     * Converts the {@link Angle}'s signed quantity to an unsigned quantity [0, 360) / [0, 2Pi)
     *
     * @return The unsigned quantity of the {@link Angle} in the {@link Angle}'s {@link AngleUnit}
     */
    public double getUnsignedAngle()
    {
        return getUnsignedAngle(this.unit);
    }

    // Calculations

    /**
     * Converts the {@link Angle}'s signed quantity to an unsigned quantity [0, 360) / [0, 2Pi)
     *
     * @param newUnit
     * @return The unsigned quantity of the {@link Angle} in the specified {@link AngleUnit}
     */
    public double getUnsignedAngle(AngleUnit newUnit)
    {
        double convertedAngle = getAngle(newUnit);
        return MathUtils.normalizeAngle(convertedAngle, true, newUnit);
    }

    /**
     * Calculates this {@link Angle} + another {@link Angle}
     *
     * @return The resulting {@link Angle}, in this {@link Angle}'s {@link AngleUnit}
     * @* @param other The second {@link Angle}
     */
    public Angle plus(Angle other)
    {
        if (other == null)
        {
            throw new IllegalStateException("Angle.plus(): other Angle is null");
        }
        if (other.unit == null)
        {
            throw new IllegalStateException("Angle.plus(): other.unit is null");
        }

        double otherAngle = other.getAngle(this.unit);
        return new Angle(this.measure + otherAngle, this.unit);
    }

    /**
     * Calculates this {@link Angle} - another {@link Angle}
     *
     * @param other The second {@link Angle}
     * @return The resulting {@link Angle}, in this {@link Angle}'s {@link AngleUnit}
     */
    public Angle minus(Angle other)
    {
        double otherAngle = other.getAngle(this.unit);

        return new Angle(this.measure - otherAngle, this.unit);
    }

    public boolean isUnit(AngleUnit angleUnit)
    {
        return this.unit == angleUnit;
    }

    @Override
    public String toString()
    {
        switch (this.unit)
        {
            default:
            case DEGREES:
                return String.format(Locale.getDefault(), "%.3f degrees", measure);
            case RADIANS:
                return String.format(Locale.getDefault(), "%.3f radians", measure);
        }
    }
}