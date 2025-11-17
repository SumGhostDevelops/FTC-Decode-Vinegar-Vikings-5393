package org.firstinspires.ftc.teamcode;

/**
 * Represents a team (RED or BLUE) with associated Goal and Base locations.
 */
public enum Team {

    // Enum constants.
    // You can change these default coordinates and IDs as needed.
    RED("red", new Goal(1, 100.0, 50.0), new Base(10.0, 50.0)),
    BLUE("blue", new Goal(2, -100.0, 50.0), new Base(-10.0, 50.0));

    // --- Enum Fields and Methods ---
    public final String color;
    public final Goal goal;
    public final Base base;

    // --- Nested Data Structures ---
    /**
     * A class to hold Goal data.
     */
    public static class Goal {
        public final int id;
        public final double x;
        public final double y;

        /**
         * Constructor for the Goal.
         * @param id The unique identifier for the goal
         * @param x The x-coordinate of the goal
         * @param y The y-coordinate of the goal
         */
        public Goal(int id, double x, double y) {
            this.id = id;
            this.x = x;
            this.y = y;
        }
    }

    /**
     * A class to hold Base data.
     */
    public static class Base {
        public final double x;
        public final double y;

        /**
         * Constructor for the Base.
         * @param x The x-coordinate of the base
         * @param y The y-coordinate of the base
         */
        public Base(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    /**
     * Private constructor for the enum.
     *
     * @param goal The team's goal
     * @param base The team's base
     * @param color The team's color
     */
    Team(String color, Goal goal, Base base) {
        this.color = color;
        this.goal = goal;
        this.base = base;
    }
}