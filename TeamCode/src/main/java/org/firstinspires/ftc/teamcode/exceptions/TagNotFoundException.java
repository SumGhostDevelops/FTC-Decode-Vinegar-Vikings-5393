package org.firstinspires.ftc.teamcode.exceptions;

public class TagNotFoundException extends RuntimeException
{
    /**
     * "The AprilTag was not found."
     */
    public TagNotFoundException()
    {
        super("The AprilTag was not found.");
    }

    /**
     * "The AprilTag with ID " + id + " was not found."
     * @param id The ID of the AprilTag that was not found.
     */
    public TagNotFoundException(int id)
    {
        super("The AprilTag with ID " + id + " was not found.");
    }
}