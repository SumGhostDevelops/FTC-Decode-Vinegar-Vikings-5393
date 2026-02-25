package org.firstinspires.ftc.teamcode.definitions.localization;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.configurables.annotations.Sorter;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.field.FieldHeading;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.CoordinateSystem;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

/**
 * All coordinates have their pose facing up or toward the goal
 */
@Configurable
public class Hardpoints
{
    private static final DistanceUnit INCH = DistanceUnit.INCH;
    private static final AngleUnit DEGREES = AngleUnit.DEGREES;

    public static class Coordinates
    {
        public static final FieldCoordinate CENTER_FIELD = new FieldCoordinate(Distance.ZERO, Distance.ZERO, CoordinateSystem.DECODE_FTC);
        public static final FieldCoordinate RED_BASE = new FieldCoordinate(new Distance(39, INCH), new Distance(33, INCH), CoordinateSystem.DECODE_PEDROPATH);
        public static final FieldCoordinate BLUE_BASE = new FieldCoordinate(new Distance(105, INCH), new Distance(33, INCH), CoordinateSystem.DECODE_PEDROPATH);
    }

    @Configurable
    public static class Poses
    {
        @Sorter(sort = 0)
        public static final Pose2d BLUE_LOADING_ZONE = new Pose2d(coord(135.0396, 8.3375), deg(0));
        @Sorter(sort = 1)
        public static final Pose2d RED_LOADING_ZONE = new Pose2d(coord(8.9604, 8.3375), deg(180));
        @Sorter(sort = 2)
        public static final Pose2d BLUE_GOAL = new Pose2d(coord(44, 135.0396), deg(90));
        @Sorter(sort = 3)
        public static final Pose2d RED_GOAL = new Pose2d(coord(100.0875, 135.0396), deg(90));
    }

    /**
     * inches and in pedropath coordinate system
     * @param x
     * @param y
     * @return
     */
    private static FieldCoordinate coord(double x, double y)
    {
        return new FieldCoordinate(
                new Distance(x, DistanceUnit.INCH),
                new Distance(y, DistanceUnit.INCH),
                CoordinateSystem.DECODE_PEDROPATH
        );
    }

    /**
     * degrees and in pedropath coordinate system
     * @param degrees
     * @return
     */
    private static FieldHeading deg(double degrees)
    {
        return new FieldHeading(new Angle(degrees, AngleUnit.DEGREES), CoordinateSystem.DECODE_PEDROPATH);
    }
}