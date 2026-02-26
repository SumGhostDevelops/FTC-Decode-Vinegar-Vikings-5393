package org.firstinspires.ftc.teamcode.definitions.localization;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.field.FieldHeading;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.CoordinateSystem;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

public class Waypoints
{
    private static Distance BOTTOM_SPIKE_Y = inch(36);
    private static Distance MIDDLE_SPIKE_Y = inch(60);
    private static Distance TOP_SPIKE_Y = inch(84);
    private static Distance LOADING_SPIKE_Y = inch(25.3);

    private static Distance GATE_Y = inch(70);

    public static class Blue
    {
        private static Distance GATE_X = inch(9);
        private static Distance SPIKE_X = inch(24);
        private static Distance LOADING_SPIKE_X = inch(2.85);

        public static FieldCoordinate BOTTOM_SPIKE = coord(SPIKE_X, BOTTOM_SPIKE_Y);
        public static FieldCoordinate MIDDLE_SPIKE = coord(SPIKE_X, MIDDLE_SPIKE_Y);
        public static FieldCoordinate TOP_SPIKE = coord(SPIKE_X, TOP_SPIKE_Y);
        public static FieldCoordinate LOADING_SPIKE = coord(LOADING_SPIKE_X, LOADING_SPIKE_Y);

        public static FieldCoordinate GATE = coord(GATE_X, GATE_Y);
    }

    public static class Red
    {
        public static FieldCoordinate BOTTOM_SPIKE = mirror(Blue.BOTTOM_SPIKE);
        public static FieldCoordinate MIDDLE_SPIKE = mirror(Blue.MIDDLE_SPIKE);
        public static FieldCoordinate TOP_SPIKE = mirror(Blue.TOP_SPIKE);
        public static FieldCoordinate LOADING_SPIKE = mirror(Blue.LOADING_SPIKE);

        public static FieldCoordinate GATE = mirror(Blue.GATE);
    }

    private static Distance inch(double inches)
    {
        return new Distance(inches, DistanceUnit.INCH);
    }

    private static FieldCoordinate coord(double x, double y)
    {
        return coord(inch(x), inch(y));
    }

    private static FieldCoordinate coord(Distance x, Distance y)
    {
        return new FieldCoordinate(x, y, CoordinateSystem.DECODE_PEDROPATH);
    }

    private static Angle deg(double degrees)
    {
        return new Angle(degrees, AngleUnit.DEGREES);
    }

    private static FieldHeading degHeading(double degrees)
    {
        return new FieldHeading(deg(degrees), CoordinateSystem.DECODE_PEDROPATH);
    }

    private static Distance mirror(Distance distance)
    {
        return inch(144 - distance.getInch());
    }

    private static FieldCoordinate mirror(FieldCoordinate coordinate)
    {
        return new FieldCoordinate(mirror(coordinate.x), coordinate.y, coordinate.coordSys);
    }
}
