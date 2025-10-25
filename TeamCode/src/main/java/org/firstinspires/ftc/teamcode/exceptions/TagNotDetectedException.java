package org.firstinspires.ftc.teamcode.exceptions;

public class TagNotDetectedException extends RuntimeException
{
    public TagNotDetectedException()
    {
        super("The AprilTag was not detected.");
    }

    public TagNotDetectedException(int id)
    {
        super("The AprilTag with ID " + id + " was not detected.");
    }
}