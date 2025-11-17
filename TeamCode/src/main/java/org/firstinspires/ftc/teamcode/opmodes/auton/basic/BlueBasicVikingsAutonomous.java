package org.firstinspires.ftc.teamcode.opmodes.auton.basic;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;

@Autonomous(name = "BlueBasicVikingsAutonomous", group = "Basic", preselectTeleOp = "BlueVikingsTeleOp")
public class BlueBasicVikingsAutonomous extends BasicVikingsAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        team = Team.BLUE;
        super.runOpMode();
    }
}
