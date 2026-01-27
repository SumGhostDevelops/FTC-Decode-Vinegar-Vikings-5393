package org.firstinspires.ftc.teamcode.opmodes.autonomous.Close.Basic;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.definitions.constants.Team;
import org.firstinspires.ftc.teamcode.definitions.hardware.RobotContext;

public abstract class Base extends LinearOpMode
{
    protected Team team;

    @Override
    public void runOpMode() throws InterruptedException
    {
        RobotContext robot = new RobotContext(team, hardwareMap, telemetry, gamepad1, gamepad2);
        telemetry.log().add("Waiting 20 seconds before moving.");
        Thread.sleep(20000);
        robot.subsystems.drive.setDrivePowers(1, 1, 1, 1);
        telemetry.log().add("Moving for 1.5 seconds.");
        Thread.sleep(1500);
        robot.subsystems.drive.stop();
    }
}