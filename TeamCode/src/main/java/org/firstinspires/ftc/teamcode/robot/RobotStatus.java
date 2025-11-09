package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.*;

public class RobotStatus
{
    public String mode;
    public double speedScalar;
    public int obeliskId;
    public Map<String, String> extra;

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