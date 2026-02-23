package org.firstinspires.ftc.teamcode.util.math;

import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.UnnormalizedAngle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.firstinspires.ftc.teamcode.util.measure.geometry.Vector2d;

public class MathUtil
{
    public static double clamp(double value, double minValue, double maxValue)
    {
        return Range.clip(value, minValue, maxValue);
    }

    public static int clamp(int value, int minValue, int maxValue)
    {
        return Range.clip(value, minValue, maxValue);
    }

    public static class Decode
    {
        /*
         * futurePose = currentPose // vector (x, y)
         * repeat 3 times:
         * distance = ||targetPos - futurePose|| // Euclidean distance
         * flightTime = LUT(distance) // or physics formula
         * newFuturePose = currentPose + robotVelocity * flightTime
         * futurePose = (futurePose + newFuturePose) / 2 // optional averaging
         *
         * //all of the following will probably be outside the function, just wanted to
         * put it somewhere
         * distance = ||targetPos - futurePose|| // final distance
         * turretAngle = atan2(target_y - future_y, target_x - future_x)
         * flywheelRPM = LUT(distance)
         */

        public static Pose2d getFuturePose(Pose2d currentPose, Vector2d robotVelocity, FieldCoordinate targetPos, InterpLUT flightTimeLUT)
        {
            return getFuturePose(currentPose, robotVelocity, targetPos, flightTimeLUT, 3);
        }

        public static Pose2d getFuturePose(Pose2d currentPose, Vector2d robotVelocity, FieldCoordinate targetPos, InterpLUT flightTimeLUT, int iterations)
        {
            return getFuturePose(currentPose, robotVelocity, targetPos, flightTimeLUT, iterations, true);
        }

        public static Pose2d getFuturePose(Pose2d currentPose, Vector2d robotVelocity, FieldCoordinate targetPos, InterpLUT flightTimeLUT, int iterations, boolean useDampening)
        {
            // We track our "guess" for where we will be
            double futureX = currentPose.coord.x.getInch();
            double futureY = currentPose.coord.y.getInch();

            for (int i = 0; i < iterations; i++)
            {
                // 1. Calculate distance from our LATEST guess to the target
                double dx = targetPos.x.getInch() - futureX;
                double dy = targetPos.y.getInch() - futureY;
                double distance = Math.sqrt(dx * dx + dy * dy);

                // 2. Get time of flight based on that distance
                double flightTime = flightTimeLUT.get(distance);

                // 3. Calculate the "instant" prediction (starting from currentPose)
                double instantX = currentPose.coord.x.getInch() + (robotVelocity.x.getInch() * flightTime);
                double instantY = currentPose.coord.y.getInch() + (robotVelocity.y.getInch() * flightTime);

                // 4. Update the guess
                if (useDampening)
                {
                    futureX = (futureX + instantX) / 2.0;
                    futureY = (futureY + instantY) / 2.0;
                }
                else
                {
                    futureX = instantX;
                    futureY = instantY;
                }
            }

            // Convert back to your Distance units (assuming a .fromInch or similar constructor)
            return new Pose2d(
                    new Distance(futureX, DistanceUnit.INCH),
                    new Distance(futureY, DistanceUnit.INCH),
                    currentPose.heading.angle,
                    currentPose.coord.coordSys
            );
        }
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
}