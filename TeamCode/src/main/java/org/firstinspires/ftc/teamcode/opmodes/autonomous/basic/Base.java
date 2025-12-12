package org.firstinspires.ftc.teamcode.opmodes.autonomous.basic;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.odometry.Localization;

public abstract class Base extends LinearOpMode
{
    protected Team team;
    protected RobotHardware robot;
    protected Drive drive;
    protected Localization localization;
    protected Outtake outtake;
    protected Transfer intake;

    // Assume a PedroPath Follower object would be initialized here
    // protected Follower follower;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new RobotHardware(hardwareMap, telemetry);
        localization = new Localization(robot);
        drive = new Drive(robot, localization);
        outtake = new Outtake(robot);
        intake = new Transfer(robot);

        // follower = new Follower(hardwareMap); // PedroPath init

        waitForStart();

        if (opModeIsActive())
        {

        }

        localization.close();
    }
}