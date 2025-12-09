package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.robot.Robot;

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

    private RobotHardware hw;
    private Localization localization;
    private Drive drive;
    private Intake intake;
    private Outtake outtake;
    private Transfer transfer;
    private Macros macros;
    private Gamepads gamepads;

    @Override
    public void runOpMode() throws InterruptedException
    {
        hw = new RobotHardware(hardwareMap, telemetry);
        localization = new Localization(hw);
        drive = new Drive(hw, localization);
        outtake = new Outtake(hw);
        intake = new Intake(hw);
        transfer = new Transfer(hw);

        telemetry.setAutoClear(true);
        telemetry.addData("Status", "Initialized for " + team);
        telemetry.update();

        gamepads = new Gamepads(gamepad1, gamepad2);

        RobotContext robotContext = new RobotContext(team, hw, drive, intake, transfer, outtake, localization, gamepads, telemetry, this::opModeIsActive);

        macros = new Macros(robotContext);

        waitForStart();

        while (opModeIsActive())
        {
            run();
        }

        localization.close();
    }

    private void run() throws InterruptedException
    {
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        handleA(gamepad1);
        handleB(gamepad1);
        handleX(gamepad1);
        handleY(gamepad1);

        handleDpadUp(gamepad1);
        handleDpadDown(gamepad1);
        handleDpadLeft(gamepad1);
        handleDpadRight(gamepad1);

        handleLeftBumper(gamepad1);
        handleRightBumper(gamepad1);
        handleLeftTrigger(gamepad1);
        handleRightTrigger(gamepad1);

        handleStart(gamepad1);
        handleBack(gamepad1);
        handleMode(gamepad1);

        // Drive
        drive.drive(axial, lateral, yaw);

        telemetry.addData("Drive Mode", drive.getMode());
        telemetry.addData("Speed", RobotConstants.DRIVE_SPEED_MULTIPLIER);
        telemetry.addData("Heading", localization.getHeading());
        telemetry.addData("Outtake Target RPM", outtake.getTargetRPM());
        telemetry.addData("Outtake RPM", outtake.getRPM());

        macros.update();
        localization.update();
        transfer.update();
        outtake.update();

        telemetry.update();
    }

    private void handleA(Gamepad gamepad) // Reset transfer outtake
    {
        if (gamepad.aWasPressed())
        {
            macros.resetTransferOuttakeNonFSM();
        }
        else if (gamepad.aWasReleased())
        {
            macros.stopResetTransferOuttakeNonFSM();
        }
    }

    private void handleB(Gamepad gamepad) // Reset IMU Yaw
    {
        if (gamepad.bWasPressed())
        {
            localization.resetHeading();
        }
    }

    private void handleX(Gamepad gamepad) // Debugging, get distance to tag
    {
        if (gamepad.xWasPressed())
        {
            localization.webcam.updateDetections();
            int tagId = localization.webcam.getAnyTagID();
            if (tagId < 0)
            {
                telemetry.log().add("No AprilTags were found.");
                return;
            }

            double distance = localization.webcam.getRangeToTag(tagId);
            telemetry.log().add("Distance to " + tagId + ":" + distance + " meters");
        }
    }

    private void handleY(Gamepad gamepad) // Auto Aim
    {
        if (gamepad.yWasPressed())
        {
            localization.webcam.updateDetections();

            if (!localization.webcam.tagIdExists(team.goal.id))
            {
                telemetry.log().add("Goal ID not found. Cancelling autoaim.");
                return;
            }

            double distanceToTag = localization.webcam.getRangeToTag(team.goal.id);

            // Here we would add a thing for RobotConstants.OUTTAKE_TARGET_RPM to be set to a value using a regression.
            macros.aimToAprilTag(team.goal.id);
        }
    }

    private void handleDpadUp(Gamepad gamepad)
    {
        if (gamepad.dpadUpWasPressed())
        {
            RobotConstants.OUTTAKE_TARGET_RPM += 100;
        }
    }

    private void handleDpadDown(Gamepad gamepad)
    {
        if (gamepad.dpadDownWasPressed())
        {
            RobotConstants.OUTTAKE_TARGET_RPM -= 100;
        }
    }

    private void handleDpadLeft(Gamepad gamepad)
    {
        if (gamepad.dpadLeftWasPressed())
        {
            RobotConstants.FORCED_ANGLE_OFFSET -= 0.25;
        }
    }

    private void handleDpadRight(Gamepad gamepad)
    {
        if (gamepad.dpadRightWasPressed())
        {
            RobotConstants.FORCED_ANGLE_OFFSET += 0.25;
        }
    }

    private void handleLeftBumper(Gamepad gamepad)
    {
        if (gamepad.leftBumperWasPressed())
        {
            double newMultiplier = RobotConstants.DRIVE_SPEED_MULTIPLIER - RobotConstants.DRIVE_SPEED_CHANGE;

            RobotConstants.DRIVE_SPEED_MULTIPLIER = Math.max(RobotConstants.DRIVE_SPEED_MINIMUM, newMultiplier);
            telemetry.log().add("New Speed: " + RobotConstants.DRIVE_SPEED_MULTIPLIER);
        }
    }

    private void handleRightBumper(Gamepad gamepad)
    {
        if (gamepad.rightBumperWasPressed())
        {
            double newMultiplier = RobotConstants.DRIVE_SPEED_MULTIPLIER + RobotConstants.DRIVE_SPEED_CHANGE;

            RobotConstants.DRIVE_SPEED_MULTIPLIER = Math.min(RobotConstants.DRIVE_SPEED_MAXIMUM, newMultiplier);
            telemetry.log().add("New Speed: " + RobotConstants.DRIVE_SPEED_MULTIPLIER);
        }
    }

    private void handleLeftTrigger(Gamepad gamepad)
    {
        if (gamepad.left_trigger > 0.25 && outtake.isReadyToLaunch())
        {
            transfer.setPower(1);
        }
        else
        {
            transfer.stop();
        }
    }

    private void handleRightTrigger(Gamepad gamepad)
    {
        if (gamepad.right_trigger > 0.25)
        {
            outtake.setRPM(RobotConstants.OUTTAKE_TARGET_RPM);
        }
        else
        {
            outtake.stop();
        }
    }

    private void handleStart(Gamepad gamepad)
    {
        if (gamepad.startWasPressed())
        {

        }
    }

    private void handleBack(Gamepad gamepad)
    {
        if (gamepad.backWasPressed())
        {

        }
    }

    private void handleMode(Gamepad gamepad) // Not sure what Mode on the G310 maps to in software
    {
        if (gamepad.guideWasPressed())
        {
            telemetry.log().add("Mode = Guide");
        }
        if (gamepad.optionsWasPressed())
        {
            telemetry.log().add("Mode = Options");
        }
    }
}