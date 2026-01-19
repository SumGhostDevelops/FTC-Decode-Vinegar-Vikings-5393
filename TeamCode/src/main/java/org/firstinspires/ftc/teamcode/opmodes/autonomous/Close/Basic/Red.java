package org.firstinspires.ftc.teamcode.opmodes.autonomous.Close.Basic;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.definitions.constants.Team;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.Close.Basic.Base;

@Autonomous(name = "RedMoveOnlyAuton", group = "Red", preselectTeleOp = "RedVikingsTeleOp")
public class Red extends Base
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        this.team = Team.RED;
        super.runOpMode();
    }
}