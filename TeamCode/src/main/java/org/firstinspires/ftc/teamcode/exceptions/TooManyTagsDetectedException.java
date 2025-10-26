package org.firstinspires.ftc.teamcode.exceptions;

public class TooManyTagsDetectedException extends Exception
{
    /**
     * "Too many AprilTags were detected."
     */
    public TooManyTagsDetectedException()
    {
        super("Too many AprilTags were detected.");
    }

    /**
     * "Too many AprilTags (" + actualTagCount + ") were detected. Expected " + expectedTagCount + "."
     * @param expectedTagCount The expected number of AprilTags
     * @param actualTagCount The actual number of AprilTags
     */
    public TooManyTagsDetectedException(int expectedTagCount, int actualTagCount)
    {
        super("Too many AprilTags (" + actualTagCount + ") were detected. Expected " + expectedTagCount + ".");
    }
}