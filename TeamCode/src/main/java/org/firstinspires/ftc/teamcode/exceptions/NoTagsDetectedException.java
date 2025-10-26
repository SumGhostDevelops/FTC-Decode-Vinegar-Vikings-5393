package org.firstinspires.ftc.teamcode.exceptions;

public class NoTagsDetectedException extends Exception
{
    /**
     * "No AprilTags were detected."
     */
    public NoTagsDetectedException()
    {
        super("No AprilTags were detected.");
    }
}