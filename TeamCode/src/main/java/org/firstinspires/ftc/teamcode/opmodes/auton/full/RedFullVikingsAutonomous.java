package org.firstinspires.ftc.teamcode.opmodes.auton.full;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;

@Autonomous(name = "RedFullVikingsAutonomous", group = "Full", preselectTeleOp = "RedVikingsTeleOp")
public class RedFullVikingsAutonomous extends FullVikingsAutonomous
{
    public void runOpMode() throws InterruptedException
    {
        team = Team.RED;
        super.runOpMode();
    }
}
