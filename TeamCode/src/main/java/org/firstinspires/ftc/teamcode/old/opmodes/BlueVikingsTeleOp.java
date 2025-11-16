package org.firstinspires.ftc.teamcode.old.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BLUEVikingsTeleOp", group = "Vikings")
public class BlueVikingsTeleOp extends VikingsTeleOpBase
{
    @Override
    public String getTeamColor()
    {
        return "blue";
    }
}