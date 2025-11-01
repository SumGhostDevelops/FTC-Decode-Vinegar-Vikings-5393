package org.firstinspires.ftc.teamcode.robot;

public class Launcher
{
    public static void Launch(Robot robot)
    {
        robot.hub.launcher.setPower(0.75);
        try
        {
            robot.hub.launcher.wait(1000);
        }
        catch (InterruptedException e)
        {
            robot.telemetry.log().add("Launcher waiting failed. Error: " + e.getMessage());
        }
        robot.hub.launcher.setPower(0);
    }
}
