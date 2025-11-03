package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.*;

public class RobotStatus
{
    private String mode;
    private double speedScalar;
    private int obeliskId;
    private Map<String, String> extra;

    public RobotStatus()
    {
        this.mode = "Manual";
        this.speedScalar = 0;
        this.obeliskId = -1;
        this.extra = new HashMap<String, String>();
    }

    public RobotStatus(double speedScalar, int obeliskId, Map<String, String> extra)
    {
        this.mode = "Manual";
        this.speedScalar = speedScalar;
        this.obeliskId = obeliskId;
        this.extra = extra;
    }

    public String getMode()
    {
        return mode;
    }

    public double getSpeedScalar()
    {
        return speedScalar;
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

    public void setSpeedScalar(double speedScalar)
    {
        this.speedScalar = speedScalar;
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
        telemetry.addData("Speed", speedScalar);
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