package org.firstinspires.ftc.teamcode.opmodes.autonomous.basic;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.LoaderIntake;
import org.firstinspires.ftc.teamcode.subsystems.Localization;

abstract class Base extends LinearOpMode
{
    protected Team team;
    protected RobotHardware robot;
    protected Drive drive;
    protected Localization localization;
    protected Launcher launcher;
    protected LoaderIntake intake;

    // Assume a PedroPath Follower object would be initialized here
    // protected Follower follower;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new RobotHardware(hardwareMap, telemetry);
        localization = new Localization(robot);
        drive = new Drive(robot, localization);
        launcher = new Launcher(robot);
        intake = new LoaderIntake(robot);

        // follower = new Follower(hardwareMap); // PedroPath init

        waitForStart();

        if (opModeIsActive())
        {

        }

        localization.close();
    }
}