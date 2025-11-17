package org.firstinspires.ftc.teamcode.opmodes.auton.basic;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;

@Autonomous(name = "RedBasicVikingsAutonomous", group = "Basic", preselectTeleOp = "RedVikingsTeleOp")
public class RedBasicVikingsAutonomous extends BasicVikingsAutonomous
{
    public void runOpMode() throws InterruptedException
    {
        team = Team.RED;
        super.runOpMode();
    }
}