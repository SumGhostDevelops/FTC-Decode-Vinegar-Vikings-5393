package org.firstinspires.ftc.teamcode.util.measure.angle.field;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.CoordinateSystem;

public class FieldHeading
{
    public final Angle angle;
    public final CoordinateSystem system;

    public FieldHeading(double heading, AngleUnit angleUnit, CoordinateSystem coordinateSystem)
    {
        this(new Angle(heading, angleUnit), coordinateSystem);
    }

    public FieldHeading(Angle angle, CoordinateSystem system)
    {
        this.angle = angle;
        this.system = system;
    }

    public FieldHeading toSystem(CoordinateSystem targetSystem)
    {
        if (this.system == targetSystem) return this;

        // 1. Convert Local Angle -> Universal Angle
        // Universal = Local + SystemOffset
        double myOffset = this.system.getRotationOffsetRadians();
        double universalRadians = this.angle.getRadians() + myOffset;

        // 2. Convert Universal Angle -> Target Local Angle
        // Target = Universal - TargetOffset
        double targetOffset = targetSystem.getRotationOffsetRadians();
        double targetRadians = universalRadians - targetOffset;

        return new FieldHeading(
                new Angle(targetRadians, AngleUnit.RADIANS).toUnit(this.angle.unit),
                targetSystem
        );
    }

    public FieldHeading toAngleUnit(AngleUnit angleUnit)
    {
        if (this.angle.unit == angleUnit) return this;

        return new FieldHeading(this.angle.toUnit(angleUnit), this.system);
    }

    public UnnormalizedFieldHeading toUnnormalized()
    {
        return new UnnormalizedFieldHeading(this.angle.toUnnormalized(), this.system);
    }

    public FieldHeading plus(FieldHeading other)
    {
        other = other.toSystem(this.system);

        return new FieldHeading(this.angle.plus(other.angle), this.system);
    }

    public FieldHeading minus(FieldHeading other)
    {
        other = other.toSystem(this.system);

        return new FieldHeading(this.angle.minus(other.angle), this.system);
    }

    @Override
    public String toString()
    {
        return String.format("%s (%s)", angle, system.name());
    }
}