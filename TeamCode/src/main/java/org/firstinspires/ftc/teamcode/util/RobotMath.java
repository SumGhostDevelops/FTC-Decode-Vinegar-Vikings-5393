package org.firstinspires.ftc.teamcode.util;

import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.util.measure.Angle;
import org.firstinspires.ftc.teamcode.util.measure.UnnormalizedAngle;

public class RobotMath
{
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

            return Math.toIntExact(Math.round(angle.angle * tickPerDegree));
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
    }

    public static class Outtake
    {
        private static final InterpLUT lut = new InterpLUT();
        private static boolean lutInitialized = false;

        /**
         * Initialize the look up table.
         */
        public static void initLUT()
        {
            /* example stuff
            //Adding each val with a key
            lut.add(1.1, 0.2);
            lut.add(2.7, .5);
            lut.add(3.6, 0.75);
            lut.add(4.1, 0.9);
            lut.add(5, 1);
            //generating final equation
            lut.createLUT();
             */
            lutInitialized = true;
        }

        /**
         * Get
         * @param distance
         * @return
         */
        public static double rpmLUT(double distance)
        {
            if (!lutInitialized)
            {
                initLUT();
            }

            return lut.get(distance);
        }
    }
}