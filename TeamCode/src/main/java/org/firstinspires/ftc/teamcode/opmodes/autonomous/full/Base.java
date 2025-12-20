package org.firstinspires.ftc.teamcode.opmodes.autonomous.full;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.controls.BlockingCommands;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.subsystems.modules.Drive;
import org.firstinspires.ftc.teamcode.subsystems.modules.Gamepads;
import org.firstinspires.ftc.teamcode.subsystems.modules.Intake;
import org.firstinspires.ftc.teamcode.subsystems.modules.Turret.Outtake;
import org.firstinspires.ftc.teamcode.definitions.RobotContext;
import org.firstinspires.ftc.teamcode.subsystems.modules.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.modules.odometry.Odometry;

import java.util.Optional;

public abstract class Base extends LinearOpMode
{
    protected Team team;

    RobotHardware hw;
    Odometry localization;
    Drive drive;
    Intake intake;
    Outtake outtake;
    Transfer transfer;
    Gamepads gamepads;

    @Override
    public void runOpMode() throws InterruptedException
    {
        hw = new RobotHardware(hardwareMap, telemetry);
        localization = new Odometry(hw, telemetry);
        drive = new Drive(hw, localization, telemetry);
        intake = new Intake(hw);
        outtake = new Outtake(hw);
        transfer = new Transfer(hw);
        gamepads = new Gamepads(gamepad1, gamepad2);


        RobotConstants.OUTTAKE_RPM_TOLERANCE = 75;

        waitForStart();
        if (opModeIsActive())
        {
            run();
        }

        localization.close();
    }

    public void run()
    {
        localization.updateWebcamDetections();

        while (!localization.tagIdExists(team.goal.id))
        {
            localization.updateWebcamDetections();
            telemetry.log().add("AprilTag (ID " + team.goal.id + " ) not found. Waiting 1 second.");
            BlockingCommands.sleep(1);
        }

        telemetry.log().add("AprilTag (ID " + team.goal.id + " ) found!");

        Optional<Double> distanceToTag = localization.getRangeToTag(team.goal.id);

        if (distanceToTag.isPresent())
        {
            outtake.setRPMBasedOnDistance(distanceToTag.get());
        }
        else
        {
            telemetry.log().add("Could not find the distance for some reason.");
        }

        //macros.aimToAprilTag(team.goal.id);

        for (int i = 0; opModeIsActive() && i < 4; i++)
        {
            while (!outtake.isReadyToLaunch()) // Wait for the outtake to be ready
            {
                telemetry.log().add("Outtake is not ready.");
                BlockingCommands.sleep(1);
            }
            telemetry.log().add("Outtake is ready.");
            transfer.setPower(1);
            telemetry.log().add("Letting the transfer move the ball.");
            BlockingCommands.sleep(0.4); // Move the ball into the outtake
            transfer.stop();
            BlockingCommands.sleep(2);
        }

        outtake.stop();
        drive.setDrivePowers(1, 0, 0);
        telemetry.log().add("Letting the robot move out of the scoring zone");
        BlockingCommands.sleep(0.4);
        drive.stop();
    }
}