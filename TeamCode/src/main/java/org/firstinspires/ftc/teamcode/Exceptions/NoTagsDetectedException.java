package org.firstinspires.ftc.teamcode.Exceptions;

public class NoTagsDetectedException extends Exception
{
    public NoTagsDetectedException()
    {
        super("No AprilTags were detected.");
    }
}