package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.*;

public class Robot
{
    public String mode;
    public double speed;
    public int obeliskId;
    public Map<String, String> extra;

    public Robot()
    {
        this.mode = "Manual";
        this.speed = 0;
        this.obeliskId = -1;
        this.extra = new HashMap<String, String>();
    }

    public Robot(double speed, int obeliskId, Map<String, String> extra)
    {
        this.mode = "Manual";
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
}