package org.firstinspires.ftc.teamcode.exceptions;

public class TagNotFoundException extends RuntimeException
{
    /**
     * "The AprilTag was not detected."
     */
    public TagNotFoundException()
    {
        super("The AprilTag was not detected.");
    }

    /**
     * "The AprilTag with ID " + id + " was not detected."
     * @param id The ID of the AprilTag that was not detected.
     */
    public TagNotFoundException(int id)
    {
        super("The AprilTag with ID " + id + " was not detected.");
    }
}