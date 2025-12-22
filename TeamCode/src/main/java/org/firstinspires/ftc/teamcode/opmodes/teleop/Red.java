package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.definitions.Team;

@TeleOp(name = "RedVikingsTeleOp", group = "Game")
public class Red extends BaseLOL
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        team = Team.RED;
        super.runOpMode();
    }
}