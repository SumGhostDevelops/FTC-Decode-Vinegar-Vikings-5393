package org.firstinspires.ftc.teamcode.definitions.constants;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.definitions.localization.Hardpoints;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;
import org.firstinspires.ftc.teamcode.util.measure.angle.field.FieldHeading;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.CoordinateSystem;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

/**
 * Represents a team (RED or BLUE) with associated Goal and Base locations.
 */
public enum Team
{

    // Enum constants.
    // You can change these default coordinates and IDs as needed.
    RED(
            "red",
            new Goal(24, coord(134, 134)),
            new Goal(24, coord(144, 144)),
            new Base(coord(38.5, 33.5)),
            new Pose2d(Hardpoints.Coordinates.RED_BASE, deg(90)),
            deg(0)),

    BLUE(
            "blue",
            new Goal(20, coord(10, 134)),
            new Goal(20, coord(0, 144)),
            new Base(coord(105.25, 33.5)),
            new Pose2d(Hardpoints.Coordinates.BLUE_BASE, deg(90)),
            deg(180)),

    NONE(
            "none",
            new Goal(-1, coord(72, 140.5)),
            new Goal(-1, coord(72, 140.5)),
            new Base(coord(72, 33.5)),
            new Pose2d(coord(72, 9), deg(90)),
            deg(90));

    // --- Enum Fields and Methods ---
    public final String color;
    public final Goal goalFromClose;
    public final Goal goalFromFar;
    public final Base base;
    public final Pose2d initialPose; // currently configured for an 18in x 18in robot, starting in the small triangle corner
    public final FieldHeading forwardAngle;

    // --- Nested Data Structures ---

    /**
     * Private constructor for the enum.
     *
     * @param color         The team's color
     * @param goalFromClose   The team's goal
     * @param goalFromFar
     * @param base          The team's base
     */
    Team(String color, Goal goalFromClose, Goal goalFromFar, Base base, Pose2d initialPose, FieldHeading forwardAngle)
    {
        this.color = color;
        this.goalFromClose = goalFromClose;
        this.goalFromFar = goalFromFar;
        this.base = base;
        this.initialPose = initialPose;
        this.forwardAngle = forwardAngle;
    }

    // --- Helper Methods to reduce visual noise ---

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

    /**
     * A class to hold Goal data.
     */
    public static class Goal
    {
        public final int id;
        public final FieldCoordinate coord;

        /**
         * Constructor for the Goal.
         *
         * @param id    The AprilTag ID of the goal
         * @param coord The {@link FieldCoordinate} of the Goal
         */
        public Goal(int id, FieldCoordinate coord)
        {
            this.id = id;
            this.coord = coord;
        }
    }

    /**
     * A class to hold Base data.
     */
    public static class Base
    {
        public final FieldCoordinate coord;

        /**
         * Constructor for the Base.
         *
         * @param coord The {@link FieldCoordinate} of the Base
         */
        public Base(FieldCoordinate coord)
        {
            this.coord = coord;
        }
    }
}