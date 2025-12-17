package org.firstinspires.ftc.teamcode.controls;

public class BlockingCommands
{
    public static void sleep(double seconds)
    {
        try
        {
            Thread.sleep((long) seconds * 1000);
        }
        catch (InterruptedException ignored) {}
    }
}
