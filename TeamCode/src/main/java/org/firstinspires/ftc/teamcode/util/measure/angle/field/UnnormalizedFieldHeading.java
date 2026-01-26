package org.firstinspires.ftc.teamcode.util.measure.angle.field;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.UnnormalizedAngle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.CoordinateSystem;

public class UnnormalizedFieldHeading
{
    public final UnnormalizedAngle angle;
    public final CoordinateSystem system;

    public UnnormalizedFieldHeading(double measure, UnnormalizedAngleUnit unit, CoordinateSystem system)
    {
        this(new UnnormalizedAngle(measure, unit), system);
    }

    public UnnormalizedFieldHeading(UnnormalizedAngle angle, CoordinateSystem system)
    {
        this.angle = angle;
        this.system = system;
    }

    /**
     * Converts this heading to a different coordinate system while preserving the total rotation (winding).
     *
     * @param targetSystem The new coordinate system to represent this heading in.
     * @return A new UnnormalizedFieldHeading relative to the target system.
     */
    public UnnormalizedFieldHeading toSystem(CoordinateSystem targetSystem)
    {
        if (this.system == targetSystem) return this;

        // 1. Convert Local Angle -> Universal Angle (Radians)
        // Universal = Local + SystemOffset
        // Note: We use raw addition/subtraction on the unnormalized value to preserve winding.
        double myOffset = this.system.getRotationOffsetRadians();
        double universalRadians = this.angle.getRadians() + myOffset;

        // 2. Convert Universal Angle -> Target Local Angle
        // Target = Universal - TargetOffset
        double targetOffset = targetSystem.getRotationOffsetRadians();
        double targetRadians = universalRadians - targetOffset;

        // Create the result in Radians first, then convert back to the original preference (Degrees/Radians)
        UnnormalizedAngle resultAngle = new UnnormalizedAngle(targetRadians, UnnormalizedAngleUnit.RADIANS);

        return new UnnormalizedFieldHeading(
                resultAngle.toUnit(this.angle.unit),
                targetSystem
        );
    }

    /**
     * Converts the internal unit of measure (e.g., Degrees -> Radians).
     */
    public UnnormalizedFieldHeading toUnit(UnnormalizedAngleUnit newUnit)
    {
        if (this.angle.unit == newUnit) return this;

        return new UnnormalizedFieldHeading(this.angle.toUnit(newUnit), this.system);
    }

    /**
     * Normalizes the heading into a standard [-180, 180) or [0, 2pi) range
     * wrapped in a {@link FieldHeading}.
     */
    public FieldHeading toNormalized()
    {
        return new FieldHeading(this.angle.toNormalized(), this.system);
    }

    public UnnormalizedFieldHeading plus(UnnormalizedFieldHeading other)
    {
        // Ensure both headings are in the same coordinate system before adding
        other = other.toSystem(this.system);

        return new UnnormalizedFieldHeading(this.angle.plus(other.angle), this.system);
    }

    public UnnormalizedFieldHeading minus(UnnormalizedFieldHeading other)
    {
        // Ensure both headings are in the same coordinate system before subtracting
        other = other.toSystem(this.system);

        return new UnnormalizedFieldHeading(this.angle.minus(other.angle), this.system);
    }

    @Override
    public String toString()
    {
        return String.format("%s (%s)", angle, system.name());
    }
}
