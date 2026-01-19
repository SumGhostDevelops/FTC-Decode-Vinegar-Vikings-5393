package org.firstinspires.ftc.teamcode.definitions.localization;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
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

    public static class Poses
    {

        public static final Pose2d BLUE_LOADING_ZONE = new Pose2d(new FieldCoordinate(new Distance(136, INCH), new Distance(8.625, INCH), CoordinateSystem.DECODE_PEDROPATH), new Angle(180, DEGREES));
        public static final Pose2d RED_LOADING_ZONE = new Pose2d(new FieldCoordinate(new Distance(8, INCH), new Distance(8.625, INCH), CoordinateSystem.DECODE_PEDROPATH), new Angle(0, DEGREES));
        public static final Pose2d BLUE_GOAL = new Pose2d(new FieldCoordinate(new Distance(23.35, INCH), new Distance(127.41, INCH), CoordinateSystem.DECODE_PEDROPATH), new Angle(144, DEGREES));
        public static final Pose2d RED_GOAL = new Pose2d(new FieldCoordinate(new Distance(120.74, INCH), new Distance(127.41, INCH), CoordinateSystem.DECODE_PEDROPATH), new Angle(36, DEGREES));
        public static final Pose2d SMALL_TRIANGLE = new Pose2d(new FieldCoordinate(new Distance(72, INCH), new Distance(8.625, INCH), CoordinateSystem.DECODE_PEDROPATH), new Angle(90, DEGREES));
    }
}