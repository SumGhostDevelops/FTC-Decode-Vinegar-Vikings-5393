package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Distance
{
    public final DistanceUnit unit;
    public final double distance;

    public Distance(double distance, DistanceUnit unit)
    {
        this.unit = unit;
        this.distance = distance;
    }

    public Distance toUnit(DistanceUnit newUnit)
    {
        double newDistance = newUnit.fromUnit(unit, distance);
        return new Distance(newDistance, newUnit);
    }
}