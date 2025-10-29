package org.firstinspires.ftc.teamcode.util;

import java.lang.Math;
import java.util.List;

public class RobotMath
{
    public static double Calculate(double degrees)
    {
        double timeToTurnNinety = 0.204;
        double Lx = 0.410;
        double Ly = 0.336;
        double r = 0.052;

        return Math.abs(timeToTurnNinety * (degrees / 90.0));
    }

    public static double angleAddition(double currentHeading, double headingToCorrect)
    {
        double sum = currentHeading + headingToCorrect;

        // Use the modulo operator to handle wrapping around 360 degrees
        double normalized = sum % 360.0;

        // If the result is negative, add 360 to bring it into the positive range
        if (normalized < 0)
        {
            normalized += 360.0;
        }

        return normalized;
    }

    public static double angleAddition(List<Double> headings)
    {
        double sum = 0;
        for (double heading : headings)
        {
            sum += heading;
        }

        // Use the modulo operator to handle wrapping around 360 degrees
        double normalized = sum % 360.0;

        // If the result is negative, add 360 to bring it into the positive range
        if (normalized < 0)
        {
            normalized += 360.0;
        }

        return normalized;
    }
}