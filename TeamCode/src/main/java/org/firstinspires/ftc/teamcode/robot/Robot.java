package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.*;

public class Robot
{
    public String team;
    public String mode;
    public double speed;
    public int obeliskId;
    public Map<String, String> extra;

    public Robot()
    {
        this.team = "blue";
        this.mode = "manual";
        this.speed = 0;
        this.obeliskId = -1;
        this.extra = new HashMap<String, String>();
    }

    public Robot(String team)
    {
        this.team = team;
        this.mode = "manual";
        this.speed = 0;
        this.obeliskId = -1;
        this.extra = new HashMap<String, String>();
    }

    public Robot(String team, double speed, int obeliskId, Map<String, String> extra)
    {
        this.team = team;
        this.mode = "manual";
        this.speed = speed;
        this.obeliskId = obeliskId;
        this.extra = extra;
    }

    public void updateTelemetry(Telemetry telemetry)
    {
        telemetry.clear();

        telemetry.addData("Mode", mode);
        telemetry.addData("Speed", speed);
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
        if (team.equals("Blue"))
        {
            return 20;
        }
        return 24;
    }
}