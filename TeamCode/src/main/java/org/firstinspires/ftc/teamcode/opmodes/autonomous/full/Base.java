package org.firstinspires.ftc.teamcode.opmodes.autonomous.full;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.controls.Macros;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Gamepads;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.RobotContext;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.odometry.Localization;

public abstract class Base extends LinearOpMode
{
    protected Team team;

    @Override
    public void runOpMode() throws InterruptedException
    {
        RobotHardware hw = new RobotHardware(hardwareMap, telemetry);
        Localization localization = new Localization(hw);
        Drive drive = new Drive(hw, localization);
        Intake intake = new Intake(hw);
        Outtake outtake = new Outtake(hw);
        Transfer transfer = new Transfer(hw);
        Gamepads gamepads = new Gamepads(gamepad1, gamepad2);

        RobotContext robot = new RobotContext(team, hw, drive, intake, transfer, outtake, localization, gamepads, telemetry, this::opModeIsActive);
        Macros macros = new Macros(robot);

        waitForStart();

        while (!localization.webcam.tagIdExists(team.goal.id))
        {
            macros.sleep(1, "AprilTag (ID " + team.goal.id + " ) not found. Waiting 1 seocnd.");
        }

        telemetry.log().add("AprilTag (ID " + team.goal.id + " )found!");

        double distanceToTag = localization.webcam.getRangeToTag(team.goal.id);
        macros.autoSetOuttakeTargetRPM(distanceToTag);
        macros.aimToAprilTag(team.goal.id);

        for (int i = 0; i < 3; i++)
        {
            while (!outtake.isReadyToLaunch())
            {
                macros.sleep(0.5, "Waiting for the outtake to reach its target RPM (" + outtake.getRPM() + "/" + outtake.getTargetRPM());
            }
            while (outtake.isReadyToLaunch())
            {
                transfer.setPower(1);
            }
            transfer.stop();
        }

        outtake.stop();
        robot.drive.setDrivePowers(1, 0, 0);
        macros.sleep(0.6, "Letting the robot move out of the scoring zone");
        robot.drive.stop();

        localization.close();
    }
}