package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.*;

public class Robot
{
    private String teamColor;
    public String mode;
    private double speed;
    private double launcherSpeed;
    private int obeliskId;
    public Map<String, String> extra;

    private Robot()
    {
        this("blue", 0, 0.7, -1, new HashMap<String, String>());
    }

    public Robot(String teamColor)
    {
        this();
        this.teamColor = teamColor;
    }

    public Robot(String teamColor, double speed, double launcherSpeed, int obeliskId, Map<String, String> extra)
    {
        this.teamColor = teamColor;
        this.mode = "manual";
        this.speed = speed;
        this.launcherSpeed = launcherSpeed;
        this.obeliskId = obeliskId;
        this.extra = extra;
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
        double lowerSpeedLimit = 0.05;
        double upperSpeedLimit = 0.75;

        if (newSpeed > upperSpeedLimit || newSpeed < lowerSpeedLimit)
        {
            return false;
        }
        this.speed = newSpeed;
        return true;
    }

    public boolean setLauncherSpeed(double newSpeed)
    {
        double lowerSpeedLimit = 0.7;
        double upperSpeedLimit = 1;

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


    public void updateTelemetry(Telemetry telemetry)
    {
        telemetry.clear();

        telemetry.addData("Team", teamColor);
        telemetry.addData("Mode", mode);
        telemetry.addData("Speed", speed);
        telemetry.addData("Launcher Speed", launcherSpeed);
        telemetry.addData("Obelisk Combination", obeliskId);

        if (!extra.isEmpty())
        {
            for (Map.Entry<String, String> entry : extra.entrySet())
            {
                telemetry.addData(entry.getKey(), entry.getValue());
            }
        }

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