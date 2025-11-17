package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Team;

@TeleOp(name = "RedVikingsTeleOp")
public class RedVikingsTeleOp extends VikingsTeleOp
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        team = Team.RED;
        super.runOpMode();
    }
}
