package org.firstinspires.ftc.teamcode.Exceptions;

public class MultipleTagsDetectedException extends Exception
{
    public MultipleTagsDetectedException()
    {
        super("More than one AprilTag was detected.");
    }

    public MultipleTagsDetectedException(int tagCount)
    {
        super("Expected 1 tag, but detected " + tagCount + ".");
    }
}