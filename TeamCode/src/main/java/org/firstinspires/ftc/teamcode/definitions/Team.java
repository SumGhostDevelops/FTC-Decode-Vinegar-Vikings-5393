package org.firstinspires.ftc.teamcode.definitions;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

/**
 * Represents a team (RED or BLUE) with associated Goal and Base locations.
 */
public enum Team
{

    // Enum constants.
    // You can change these default coordinates and IDs as needed.
    RED("red", new Goal(24, new FieldCoordinate(
            new Distance(134.5, DistanceUnit.INCH),
            new Distance(140.5, DistanceUnit.INCH),
            FieldCoordinate.CoordinateSystem.RIGHT_HAND)),

            new Base(new FieldCoordinate(
                    new Distance(38.5, DistanceUnit.INCH),
                    new Distance(33.5, DistanceUnit.INCH),
                    FieldCoordinate.CoordinateSystem.RIGHT_HAND))),

    BLUE("blue", new Goal(20, new FieldCoordinate(
            new Distance(9.5, DistanceUnit.INCH),
            new Distance(140.5, DistanceUnit.INCH),
            FieldCoordinate.CoordinateSystem.RIGHT_HAND)),

            new Base(new FieldCoordinate(
                    new Distance(105.25, DistanceUnit.INCH),
                    new Distance(33.5, DistanceUnit.INCH),
                    FieldCoordinate.CoordinateSystem.RIGHT_HAND))),

    NONE("none", new Goal(-1, new FieldCoordinate(
            new Distance(72, DistanceUnit.INCH),
            new Distance(140.5, DistanceUnit.INCH),
            FieldCoordinate.CoordinateSystem.RIGHT_HAND)),

            new Base(new FieldCoordinate(
                    new Distance(72, DistanceUnit.INCH),
                    new Distance(33.5, DistanceUnit.INCH),
                    FieldCoordinate.CoordinateSystem.RIGHT_HAND)));

    // --- Enum Fields and Methods ---
    public final String color;
    public final Goal goal;
    public final Base base;

    // --- Nested Data Structures ---

    /**
     * Private constructor for the enum.
     *
     * @param goal  The team's goal
     * @param base  The team's base
     * @param color The team's color
     */
    Team(String color, Goal goal, Base base)
    {
        this.color = color;
        this.goal = goal;
        this.base = base;
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
         * @param id The AprilTag ID of the goal
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