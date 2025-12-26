package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Angle
{
    public final AngleUnit unit;
    public final double angle;

    public Angle(double angle, AngleUnit unit)
    {
        this.unit = unit;
        this.angle = angle;
    }

    public Angle toUnit(AngleUnit newUnit)
    {
        double newAngle = newUnit.fromUnit(unit, angle);
        return new Angle(newAngle, newUnit);
    }
}