package org.firstinspires.ftc.teamcode.opmodes.autonomous.Far;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.definitions.constants.Team;

public abstract class Base extends LinearOpMode
{
    protected Team team;

    protected enum AutoStrat{
        BASIC,
        REGULAR,
        GATE

    }
    @Override
    public void runOpMode() throws InterruptedException
    {

    }
}