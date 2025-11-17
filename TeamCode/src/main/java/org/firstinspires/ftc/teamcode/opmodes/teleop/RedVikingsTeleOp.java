package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.Team;

public class RedVikingsTeleOp extends VikingsTeleOp
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        team = Team.RED;
        super.runOpMode();
    }
}
