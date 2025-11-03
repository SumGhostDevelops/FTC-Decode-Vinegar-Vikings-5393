package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.util.ObeliskHelper;

import java.util.*;

public class RobotStatus
{
    private String mode;
    private double speed;
    private int obeliskId;
    private Map<String, String> extra;

    public RobotStatus()
    {
        this.mode = "Manual";
        this.speed = 0;
        this.obeliskId = -1;
        this.extra = new HashMap<String, String>();
    }

    public RobotStatus(double speed, int obeliskId, Map<String, String> extra)
    {
        this.mode = "Manual";
        this.speed = speed;
        this.obeliskId = obeliskId;
        this.extra = extra;
    }

    public String getMode()
    {
        return mode;
    }

    public double getSpeed()
    {
        return speed;
    }

    public int getObeliskId()
    {
        return obeliskId;
    }

    public Map<String, String> getExtra()
    {
        return extra;
    }

    public void setMode(String mode)
    {
        this.mode = mode;
    }

    public void setSpeed(double speed)
    {
        this.speed = speed;
    }

    public void setObeliskId(int obeliskCombination)
    {
        this.obeliskId = obeliskCombination;
    }

    public void setExtra(Map<String, String> extra)
    {
        this.extra = extra;
    }

    public void addExtra(String key, String value)
    {
        extra.put(key, value);
    }

    public void clearExtra()
    {
        extra.clear();
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
}