package org.firstinspires.ftc.teamcode.util;

public class MotorMath
{
    public static double tpsToRpm(double tps, double ppr)
    {
        return (tps / ppr) * 60.0;
    }

    public static double rpmToTps(double rpm, double ppr)
    {
        return (rpm * ppr) / 60.0;
    }

    public static double tps2ToRpm2(double tps2, double ppr)
    {
        // Square the conversion factor (60/ppr)
        double factor = 60.0 / ppr;
        return tps2 * (factor * factor);
    }

    public static double rpm2ToTps2(double rpm2, double ppr)
    {
        // Square the inverse conversion factor (ppr/60)
        double factor = ppr / 60.0;
        return rpm2 * (factor * factor);
    }
}