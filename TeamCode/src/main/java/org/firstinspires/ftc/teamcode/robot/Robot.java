package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.*;

public class Robot
{
    private String teamColor;
    public String mode;
    private double speed;
    private double launcherSpeed;
    private double launcherRpm = 0;
    private double launcherRpmDerivative = 0;
    private int obeliskId;
    ElapsedTime timer;

    public Robot()
    {
        this("blue", 0.7, 0.7, -1);
    }

    public Robot(String teamColor)
    {
        this();
        this.teamColor = teamColor;
    }

    public Robot(String teamColor, double speed, double launcherSpeed, int obeliskId)
    {
        this.teamColor = teamColor;
        this.mode = "manual";
        this.speed = speed;
        this.launcherSpeed = launcherSpeed;
        this.obeliskId = obeliskId;
        timer = new ElapsedTime();
    }

    public String getTeamColor()
    {
        return teamColor;
    }

    public double getSpeed()
    {
        return speed;
    }

    public double getLauncherSpeed()
    {
        return launcherSpeed;
    }

    public int getObeliskId()
    {
        return obeliskId;
    }

    public boolean setTeamColor(String teamColor)
    {
        if (!(teamColor.equals("red") || teamColor.equals("blue")))
        {
            throw new IllegalArgumentException("Expected \"red\" or \"blue\", got \"" + teamColor + "\"");
        }

        if (teamColor.equals(this.teamColor))
        {
            return false;
        }

        this.teamColor = teamColor;
        return true;
    }

    public boolean setSpeed(double newSpeed)
    {
        double lowerSpeedLimit = 0.25;
        double upperSpeedLimit = 1;

        if (newSpeed > upperSpeedLimit || newSpeed < lowerSpeedLimit)
        {
            return false;
        }
        this.speed = newSpeed;
        return true;
    }

    public boolean setLauncherSpeed(double newSpeed)
    {
        double lowerSpeedLimit = 0.5;
        double upperSpeedLimit = 0.8;

        if (newSpeed > upperSpeedLimit || newSpeed < lowerSpeedLimit)
        {
            return false;
        }

        this.launcherSpeed = newSpeed;
        return true;
    }

    public boolean setObeliskId(int newObeliskId) throws IllegalArgumentException
    {
        if (newObeliskId < 21 || newObeliskId > 23)
        {
            throw new IllegalArgumentException("Obelisk ID must be between 21 and 23, got " + newObeliskId);
        }

        if (newObeliskId == this.obeliskId)
        {
            return false;
        }

        this.obeliskId = newObeliskId;
        return true;
    }


    public void updateTelemetry(RobotContext robot)
    {
        Telemetry telemetry = robot.telemetry;

        // --- Derivative Calculation ---

        // 1. Calculate the time elapsed since the last updateTelemetry call.
        double timeDelta = timer.seconds();
        timer.reset(); // Reset the timer for the next interval

        // 2. Store the previous RPM value before calculating the new one.
        double oldLauncherRpm = this.launcherRpm;

        double ticksPerSecond = robot.hub.launcher.getVelocity();
        launcherRpm = robot.hub.launcher.getVelocity(AngleUnit.DEGREES) / 6.0;

        // 4. Calculate the derivative (rate of change) in RPM per second.
        //    Avoid division by zero if the loop is extremely fast.
        if (timeDelta > 0)
        {
            this.launcherRpmDerivative = (this.launcherRpm - oldLauncherRpm) / timeDelta;
        }
        else
        {
            this.launcherRpmDerivative = 0;
        }

        // --- End of Derivative Calculation ---

        telemetry.clear();

        telemetry.addData("Team", teamColor);
        telemetry.addData("Mode", mode);
        telemetry.addData("Speed", speed);
        telemetry.addData("Launcher Speed", launcherSpeed);
        telemetry.addData("Launcher RPM", launcherRpm);
        telemetry.addData("Launcher RPM Rate (deriv)", launcherRpmDerivative);
        telemetry.addData("Obelisk Combination", obeliskId);

        telemetry.update();
    }

    public int getGoalId()
    {
        switch (this.teamColor)
        {
            case "blue":
                return 20;
            case "red":
                return 24;
            default:
                throw new IllegalStateException("Unexpected teamColor: " + this.teamColor);
        }
    }
}