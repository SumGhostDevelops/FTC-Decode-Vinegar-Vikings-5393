package org.firstinspires.ftc.teamcode.util;

import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
import org.firstinspires.ftc.teamcode.util.measure.angle.UnnormalizedAngle;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

public class RobotMath
{
    public static double clamp(double value, double minValue, double maxValue)
    {
        return Math.max(minValue, Math.min(value, maxValue));
    }

    public static int clamp(int value, int minValue, int maxValue)
    {
        return clamp(value, minValue, maxValue);
    }

    public static class Motor
    {
        /**
         * Convert ticks per second (TPS) to revolutions per minute (RPM)
         * @param tps The ticks per second
         * @param ppr The PPR of the motor
         * @return
         */
        public static double tpsToRpm(double tps, double ppr)
        {
            if (ppr == 0.0) return 0.0;
            return (tps / ppr) * 60.0;
        }

        /**
         * Convert revolutions per minute (RPM) to ticks per second
         * @param rpm The revolutions per minute
         * @param ppr The PPR of the motor
         * @return
         */
        public static double rpmToTps(double rpm, double ppr)
        {
            return (rpm * ppr) / 60.0;
        }

        /**
         * Convert ticks per second squared (T/S^2) to revolutions per minute squared (R/M^2). Acceleration -> acceleration.
         * @param tps2 The ticks per second squared
         * @param ppr The PPR of the motor
         * @return
         */
        public static double tps2ToRpm2(double tps2, double ppr)
        {
            // Square the conversion factor (60/ppr)
            double factor = 60.0 / ppr;
            return tps2 * (factor * factor);
        }

        /**
         * Convert revolutions per minute squared (R/M^2) to ticks per second squared (T/S^2). Acceleration -> acceleration.
         * @param rpm2 The revolutions per minute squared
         * @param ppr The PPR of the motor
         * @return
         */
        public static double rpm2ToTps2(double rpm2, double ppr)
        {
            // Square the inverse conversion factor (ppr/60)
            double factor = ppr / 60.0;
            return rpm2 * (factor * factor);
        }

        /**
         * Converts an angle to the # of ticks necessary to turn that angle
         * @param angle
         * @param PPR Pulses per Revolution
         * @param gearRatio Ratio from input to Output (ex: 20:1 = 20.0)
         * @return
         */
        public static int angleToTicks(Angle angle, double PPR, double gearRatio)
        {
            angle = angle.toUnit(AngleUnit.DEGREES);

            double tickPerDegree = (PPR * gearRatio) / 360.0;

            return Math.toIntExact(Math.round(angle.measure * tickPerDegree));
        }

        /**
         * Converts an unnormalized angle to the # of ticks necessary to turn that angle
         * @param angle
         * @param PPR Pulses per Revolution
         * @param gearRatio Ratio from input to Output (ex: 20:1 = 20.0)
         * @return
         */
        public static int angleToTicks(UnnormalizedAngle angle, double PPR, double gearRatio)
        {
            angle = angle.toUnit(UnnormalizedAngleUnit.DEGREES);

            double tickPerDegree = (PPR * gearRatio) / 360.0;

            return Math.toIntExact(Math.round(angle.measure * tickPerDegree));
        }

        /**
         * Convert encoder ticks to a normalized {@link Angle} ([-180,180) degrees).
         *
         * @param ticks   encoder tick count
         * @param PPR     pulses per revolution of the encoder
         * @param gearRatio gear ratio from motor/encoder input to output (e.g. 20.0 for 20:1)
         * @return an {@link Angle} in degrees (normalized)
         */
        public static Angle ticksToAngle(int ticks, double PPR, double gearRatio)
        {
            double tickPerDegree = (PPR * gearRatio) / 360.0;

            if (tickPerDegree == 0.0)
            {
                return new Angle(0.0, AngleUnit.DEGREES);
            }

            double degrees = ticks / tickPerDegree;

            return new Angle(degrees, AngleUnit.DEGREES);
        }

        /**
         * Convert encoder ticks to an {@link UnnormalizedAngle} (unbounded angle quantity).
         * Useful when you want the raw continuous angle without normalization.
         *
         * @param ticks   encoder tick count
         * @param PPR     pulses per revolution of the encoder
         * @param gearRatio gear ratio from motor/encoder input to output (e.g. 20.0 for 20:1)
         * @return an {@link UnnormalizedAngle} in degrees (not normalized/wrapped)
         */
        public static UnnormalizedAngle ticksToUnnormalizedAngle(int ticks, int PPR, double gearRatio)
        {
            double tickPerDegree = (PPR * gearRatio) / 360.0;

            if (tickPerDegree == 0.0)
            {
                return new UnnormalizedAngle(0.0, UnnormalizedAngleUnit.DEGREES);
            }

            double degrees = ticks / tickPerDegree;

            return new UnnormalizedAngle(degrees, UnnormalizedAngleUnit.DEGREES);
        }
    }

    public static class Outtake
    {
        private static final InterpLUT lut = new InterpLUT();
        private static boolean lutInitialized = false;
        private static final DistanceUnit dUnit = DistanceUnit.INCH;

        /**
         * Initialize the look up table.
         */
        public static void initLUT()
        {
            // make sure all distances have .toUnit(dUnit)
            // distance, rpm
            // inches
            lut.add(0, 4100);
            lut.add(50.32, 4100);
            lut.add(59.63, 4300);
            lut.add(70.25, 4400);
            lut.add(86.79, 4800);
            lut.add(97.81, 5000);
            lut.add(108.49, 5400);
            lut.add(Math.hypot(144, 144), 5400);
            lut.createLUT();
            lutInitialized = true;
        }

        /**
         * Get
         * @param dist
         * @return
         */
        public static double rpmLUT(Distance dist)
        {
            if (!lutInitialized)
            {
                initLUT();
            }

            return lut.get(dist.toUnit(dUnit).magnitude);

            //double distance = dist.toUnit(dUnit).magnitude;

            //return 0.172932 * distance * distance - 6.07764 * distance + 3996.30357;
        }
    }
}