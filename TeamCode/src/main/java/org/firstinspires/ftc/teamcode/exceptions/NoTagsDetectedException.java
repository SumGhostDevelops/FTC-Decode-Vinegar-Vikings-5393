package org.firstinspires.ftc.teamcode.exceptions;

public class NoTagsDetectedException extends Exception
{
    public NoTagsDetectedException()
    {
        super("No AprilTags were detected.");
    }
}