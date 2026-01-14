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
        return  Math.max(minValue, Math.min(value, maxValue));
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
        public static int angleToTicks(Angle angle, int PPR, double gearRatio)
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
        public static int angleToTicks(UnnormalizedAngle angle, int PPR, double gearRatio)
        {
            angle = angle.toUnit(UnnormalizedAngleUnit.DEGREES);

            double tickPerDegree = (PPR * gearRatio) / 360.0;

            return Math.toIntExact(Math.round(angle.angle * tickPerDegree));
        }

        /**
         * Convert encoder ticks to a normalized {@link Angle} ([-180,180) degrees).
         *
         * @param ticks   encoder tick count
         * @param PPR     pulses per revolution of the encoder
         * @param gearRatio gear ratio from motor/encoder input to output (e.g. 20.0 for 20:1)
         * @return an {@link Angle} in degrees (normalized)
         */
        public static Angle ticksToAngle(int ticks, int PPR, double gearRatio)
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
        private static final DistanceUnit dUnit = DistanceUnit.METER;

        /**
         * Initialize the look up table.
         */
        public static void initLUT()
        {
            // make sure all distances have .toUnit(dUnit)
            // distance, rpm
            //Adding each val with a key
            /*
            lut.add(1.529, 2250);
            lut.add(1.911, 2400);
            lut.add(3.872, 3300);
            lut.add(2.934, 2950);


             */
            //generating final equation
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
            double meters = dist.toUnit(DistanceUnit.METER).magnitude;
            return -223.05528*meters*meters + 1691.10697*meters + 34.64716;
        }
    }
}