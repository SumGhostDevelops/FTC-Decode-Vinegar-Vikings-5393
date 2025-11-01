package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryStatus
{
    private String mode;
    private String obeliskCombination;
    private String extra = "";

    public TelemetryStatus(String mode, String obeliskCombination)
    {
        this.mode = mode;
        this.obeliskCombination = obeliskCombination;
    }

    public TelemetryStatus(String mode, String obeliskCombination, String extra)
    {
        this.mode = mode;
        this.obeliskCombination = obeliskCombination;
        this.extra = extra;
    }

    public void updateMode(String mode)
    {
        this.mode = mode;
    }

    public void updateObeliskCombination(String obeliskCombination)
    {
        this.obeliskCombination = obeliskCombination;
    }

    public void updateExtra(String extra)
    {
        this.extra = extra;
    }

    public void updateTelemetry(Telemetry telemetry)
    {
        telemetry.clear();
        telemetry.addData("Mode", mode);
        telemetry.addData("Obelisk Combination", obeliskCombination);

        if (extra != "")
        {
            telemetry.addData("Extra", extra);
        }

        telemetry.update();
    }
}
