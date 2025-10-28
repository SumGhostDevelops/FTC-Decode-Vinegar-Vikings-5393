package org.firstinspires.ftc.teamcode.exceptions;

import java.util.List;

public class UnexpectedTagIDException extends RuntimeException
{
    /**
     * "The AprilTag found is not a valid ID."
     */
    public UnexpectedTagIDException()
    {
        super("The AprilTag found is not a valid ID.");
    }

    /**
     * "The AprilTag with ID " + expectedId + " is not a valid ID."
     * @param expectedId The expected ID of the AprilTag
     */
    public UnexpectedTagIDException(int expectedId)
    {
        super("The AprilTag with ID " + expectedId + " is not a valid ID.");
    }

    /**
     * "The AprilTag with ID " + id + " is not a valid ID."
     * @param expectedId The expected ID of the AprilTag
     * @param actualId The actual ID of the AprilTag
     */
    public UnexpectedTagIDException(int expectedId, int actualId)
    {
        super("Expected an AprilTag with ID " + expectedId + ", but found an AprilTag with ID " + actualId + ".");
    }
}