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

    public static double normalize360Angle(double angle)
    {
        // 1. Calculate the remainder using modulo
        double normalized = angle % 360.0;

        // 2. If the result is negative, add 360 to bring it into the 0-360 range
        if (normalized < 0) {
            normalized += 360.0;
        }

        return normalized;
    }

    /**
     * Normalizes an absolute angle to be within the -180 to +180 degree range.
     * @param angle The angle to normalize.
     * @return The angle, wrapped to the -180 to +180 range.
     */
    public static double convert360AngleTo180(double angle)
    {
        // Map the angle to the 0 to 360 range
        double normalized = normalize360Angle(angle);

        if (normalized > 180)
        {
            normalized -= 360;
        }

        return normalized;
    }

    public static double clampPower(double motorPower)
    {
        return Math.max(-1.0, Math.min(1.0, motorPower));
    }
}