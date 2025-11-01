package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.*;

public class TelemetryStatus
{
    private String mode;
    private double speed;
    private String obeliskCombination;
    private Map<String, String> extra;

    public TelemetryStatus()
    {
        this.mode = "Manual";
        this.speed = 0;
        this.obeliskCombination = "";
        this.extra = new HashMap<String, String>();
    }

    public TelemetryStatus(double speed, String obeliskCombination, Map<String, String> extra)
    {
        this.mode = "Manual";
        this.speed = speed;
        this.obeliskCombination = obeliskCombination;
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

    public String getObeliskCombination()
    {
        return obeliskCombination;
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

    public void setObeliskCombination(String obeliskCombination)
    {
        this.obeliskCombination = obeliskCombination;
    }

    public void setExtra(Map<String, String> extra)
    {
        this.extra = extra;
    }

    public void update(Telemetry telemetry)
    {
        telemetry.clear();

        telemetry.addData("Mode", mode);
        telemetry.addData("Speed", speed);
        telemetry.addData("Obelisk Combination", obeliskCombination);

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