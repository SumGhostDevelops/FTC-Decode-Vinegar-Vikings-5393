package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "REDVikingsTeleOp", group = "Vikings")
public class RedVikingsTeleOp extends VikingsTeleOpBase
{
    @Override
    public String getTeamColor()
    {
        return "red";
    }
}