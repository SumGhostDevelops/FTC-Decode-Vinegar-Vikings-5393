package org.firstinspires.ftc.teamcode.opmodes.autonomous.basic;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.definitions.Team;

@Autonomous(name = "RedFullVikingsAutonomous", group = "Red", preselectTeleOp = "RedVikingsTeleOp")
public class Red extends Base
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        team = Team.RED;
        super.runOpMode();
    }
}