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

    public static double angleAddition(double currentHeading, double headingToCorrectBy)
    {
        double sum = currentHeading + headingToCorrectBy;

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

    /**
     * Normalizes an angle to be within the -180 to +180 degree range.
     * @param angle The angle to normalize.
     * @return The angle, wrapped to the -180 to +180 range.
     */
    public static double normalizeAngle(double angle) {
        while (angle > 180.0) {
            angle -= 360.0;
        }
        while (angle <= -180.0) {
            angle += 360.0;
        }
        return angle;
    }
}