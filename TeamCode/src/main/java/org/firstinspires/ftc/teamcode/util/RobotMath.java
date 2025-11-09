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

    public double normalizeAbsoluteAngle(double angle)
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

    public static double clampPower(double motorPower)
    {
        return Math.max(-1.0, Math.min(1.0, motorPower));
    }
}