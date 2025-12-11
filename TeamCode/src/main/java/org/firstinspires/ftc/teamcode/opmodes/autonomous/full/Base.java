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
        localization.webcam.updateDetections();

        while (!localization.webcam.tagIdExists(team.goal.id))
        {
            localization.webcam.updateDetections();
            macros.sleep(1, "AprilTag (ID " + team.goal.id + " ) not found. Waiting 1 seocnd.");
        }

        telemetry.log().add("AprilTag (ID " + team.goal.id + " )found!");

        double distanceToTag = localization.webcam.getRangeToTag(team.goal.id);
        outtake.setRPMBasedOnDistance(distanceToTag);
        macros.aimToAprilTag(team.goal.id);

        for (int i = 0; i < 3; i++)
        {
            // Wait for the outtake to be ready
            while (!outtake.isReadyToLaunch())
            {
                macros.sleep(1, "Outtake is not ready.");
            }
            telemetry.log().add("Outtake is ready.");
            transfer.setPower(1);
            // Wait for the outtake to shoot
            while (outtake.isReadyToLaunch())
            {
                macros.sleep(1);
                if (!outtake.isReadyToLaunch())
                {
                    break;
                }
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