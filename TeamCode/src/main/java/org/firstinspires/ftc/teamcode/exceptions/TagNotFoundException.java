package org.firstinspires.ftc.teamcode.exceptions;

public class TagNotFoundException extends RuntimeException
{
    public TagNotFoundException()
    {
        super("The AprilTag was not detected.");
    }

    public TagNotFoundException(int id)
    {
        super("The AprilTag with ID " + id + " was not detected.");
    }
}