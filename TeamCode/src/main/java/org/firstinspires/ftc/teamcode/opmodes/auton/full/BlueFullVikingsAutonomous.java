package org.firstinspires.ftc.teamcode.opmodes.auton.full;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;

@Autonomous(name = "BlueFullVikingsAutonomous", group = "Blue", preselectTeleOp = "BlueVikingsTeleOp")
public class BlueFullVikingsAutonomous extends FullVikingsAutonomous
{
    public void runOpMode() throws InterruptedException
    {
        team = Team.BLUE;
        super.runOpMode();
    }
}
