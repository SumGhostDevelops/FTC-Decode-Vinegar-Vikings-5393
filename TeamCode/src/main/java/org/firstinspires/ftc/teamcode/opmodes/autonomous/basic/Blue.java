package org.firstinspires.ftc.teamcode.opmodes.autonomous.basic;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.definitions.Team;

@Autonomous(name = "BlueBasicVikingsAutonomous", group = "Blue", preselectTeleOp = "BlueVikingsTeleOp")
public class Blue extends Base
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        team = Team.BLUE;
        super.runOpMode();
    }
}