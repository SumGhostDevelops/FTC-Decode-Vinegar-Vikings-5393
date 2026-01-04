package org.firstinspires.ftc.teamcode.util.measure.angle;

import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/**
 * {@link UnnormalizedAngle} represents an unnormalized angle ({@code (-infinity, infinity) based on a full circle}) and its unit of measure.
 */
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
     * @param finalUnit
     * @return The {@link UnnormalizedAngle} in the new {@link UnnormalizedAngleUnit}
     */
    public UnnormalizedAngle toUnit(UnnormalizedAngleUnit finalUnit)
    {
        if (this.unit == finalUnit)
        {
            return this;
        }

        return new UnnormalizedAngle(getAngle(finalUnit), finalUnit);
    }

    /**
     * Converts an {@link UnnormalizedAngle} to an {@link Angle}
     * @param newUnit
     * @return The {@link Angle} in the new {@link AngleUnit}
     */
    public Angle toUnit(AngleUnit newUnit)
    {
        AngleUnit normalizedUnit = this.unit.getNormalized(); // get the normalized version of the current unit
        Angle normalized = new Angle(this.angle, normalizedUnit); // create a new angle, constructor handles normalization

        return normalized.toUnit(newUnit);
    }

    public Angle toNormalized()
    {
        return toUnit(getNormalizedUnit(this.unit));
    }

    /**
     * @param newUnit
     * @return The quantity of the {@link UnnormalizedAngle} in the specified {@link UnnormalizedAngleUnit}
     */
    public double getAngle(UnnormalizedAngleUnit newUnit)
    {
        return newUnit.fromUnit(this.unit, angle);
    }

    /**
     * @return The quantity of the wrapped {@link UnnormalizedAngle} in the {@link UnnormalizedAngle}'s {@link UnnormalizedAngleUnit}
     */
    public double getWrappedAngle()
    {
        return getWrappedAngle(this.unit);
    }

    /**
     * @param newUnit
     * @return The quantity of the wrapped {@link UnnormalizedAngle} in the specified {@link UnnormalizedAngleUnit}
     */
    public double getWrappedAngle(UnnormalizedAngleUnit newUnit)
    {
        double convertedAngle = getAngle(newUnit);
        return MathUtils.normalizeAngle(convertedAngle, true, getNormalizedUnit(newUnit));
    }

    /**
     * Calculates this {@link UnnormalizedAngle} + another {@link UnnormalizedAngle}
     * @param b The second {@link UnnormalizedAngle}
     * @return The resulting {@link UnnormalizedAngle}, in this {@link UnnormalizedAngle}'s {@link UnnormalizedAngleUnit}
     */
    public UnnormalizedAngle plus(UnnormalizedAngle b)
    {
        double bAngle = b.getAngle(this.unit);

        return new UnnormalizedAngle(this.angle + bAngle, this.unit);
    }

    /**
     * Calculates this {@link UnnormalizedAngle} - another {@link UnnormalizedAngle}
     * @param b The second {@link UnnormalizedAngle}
     * @return The resulting {@link UnnormalizedAngle}, in this {@link UnnormalizedAngle}'s {@link UnnormalizedAngleUnit}
     */
    public UnnormalizedAngle minus(UnnormalizedAngle b)
    {
        double bAngle = b.getAngle(this.unit);

        return new UnnormalizedAngle(this.angle - bAngle, this.unit);
    }

    /**
     * @param angleUnit
     * @return The normalized {@link AngleUnit} version of the {@link UnnormalizedAngleUnit}
     */
    public static AngleUnit getNormalizedUnit(UnnormalizedAngleUnit angleUnit)
    {
        switch (angleUnit)
        {
            default:
            case DEGREES: return AngleUnit.DEGREES;
            case RADIANS: return AngleUnit.RADIANS;
        }
    }
}