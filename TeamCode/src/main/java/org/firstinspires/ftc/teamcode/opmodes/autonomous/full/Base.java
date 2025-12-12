package org.firstinspires.ftc.teamcode.opmodes.autonomous.full;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.controls.Macros;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Gamepads;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.definitions.RobotContext;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.odometry.Localization;

import java.util.Optional;

public abstract class Base extends LinearOpMode
{
    protected Team team;

    RobotHardware hw;
    Localization localization;
    Drive drive;
    Intake intake;
    Outtake outtake;
    Transfer transfer;
    Gamepads gamepads;

    Macros macros;

    @Override
    public void runOpMode() throws InterruptedException
    {
        hw = new RobotHardware(hardwareMap, telemetry);
        localization = new Localization(hw);
        drive = new Drive(hw, localization);
        intake = new Intake(hw);
        outtake = new Outtake(hw);
        transfer = new Transfer(hw);
        gamepads = new Gamepads(gamepad1, gamepad2);

        RobotContext robot = new RobotContext(team, hw, drive, intake, transfer, outtake, localization, gamepads, telemetry, this::opModeIsActive);
        macros = new Macros(robot);

        waitForStart();
        if (opModeIsActive())
        {
            run();
        }

        localization.close();
    }

    public void run()
    {
        localization.webcam.updateDetections();

        while (!localization.webcam.tagIdExists(team.goal.id))
        {
            localization.webcam.updateDetections();
            macros.sleep(1, "AprilTag (ID " + team.goal.id + " ) not found. Waiting 1 seocnd.");
        }

        telemetry.log().add("AprilTag (ID " + team.goal.id + " )found!");

        Optional<Double> distanceToTag = localization.webcam.getRangeToTag(team.goal.id);

        if (distanceToTag.isPresent())
        {
            outtake.setRPMBasedOnDistance(distanceToTag.get());
        }
        else
        {
            telemetry.log().add("Could not find the distance for some reason.");
        }

        macros.aimToAprilTag(team.goal.id);

        for (int i = 0; opModeIsActive() && i < 3; i++)
        {
            while (!outtake.isReadyToLaunch()) // Wait for the outtake to be ready
            {
                macros.sleep(1, "Outtake is not ready.");
            }
            telemetry.log().add("Outtake is ready.");
            transfer.setPower(1);
            macros.sleep(0.4, "Letting the transfer move the ball."); // Move the ball into the outtake
            transfer.stop();
            macros.sleep(2);
        }

        outtake.stop();
        drive.setDrivePowers(1, 0, 0);
        macros.sleep(0.4, "Letting the robot move out of the scoring zone");
        drive.stop();
    }
}