package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.controls.Macros;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Gamepads;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.RobotContext;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.odometry.Localization;
import org.firstinspires.ftc.teamcode.subsystems.odometry.Webcam;

public abstract class Base extends LinearOpMode
{

    // Change this manually or create a config selector OpMode
    protected Team team;

    private RobotHardware robot;
    private Localization localization;
    private Drive drive;
    private Intake intake;
    private Outtake outtake;
    private Transfer transfer;
    private Webcam webcam;
    private Macros macros;
    private Gamepads gamepads;

    private RobotContext robotContext;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new RobotHardware(hardwareMap, telemetry);
        localization = new Localization(robot);
        drive = new Drive(robot, localization);
        outtake = new Outtake(robot);
        intake = new Intake(robot);
        transfer = new Transfer(robot);
        webcam = new Webcam(robot);

        telemetry.addData("Status", "Initialized for " + team);
        telemetry.update();

        gamepads = new Gamepads(gamepad1, gamepad2);

        robotContext = new RobotContext(team, robot, drive, intake, transfer, outtake, webcam, localization, gamepads, telemetry, this::opModeIsActive);

        macros = new Macros(robotContext);

        waitForStart();

        boolean telemetryEnabled = true;

        while (opModeIsActive())
        {
            run();
        }
    }

    private void run()
    {
        telemetry.addData("Status", "Initialized for " + team);
        telemetry.update();

        waitForStart();

        boolean telemetryEnabled = true;

        while (opModeIsActive())
        {
            // --- Gamepad 1: Drive ---
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Mode Toggle
            if (gamepad1.bWasPressed())
            {
                drive.toggleDriveMode();
            }

            // Reset Heading
            if (gamepad1.optionsWasPressed())
            { // Options/Start button
                localization.resetHeading();
            }

            // Drive
            drive.drive(axial, lateral, yaw, 1.0);

            if (gamepad1.right_trigger > 0.5) // Toggle fire when ready
            {
                macros.toggleFireWhenReady();
            }

            if (gamepad2.xWasPressed()) // Reset transfer outtake
            {
                macros.resetTransferOuttake();
            }

            // --- Telemetry Toggle ---
            if (gamepad1.startWasPressed())
            {
                telemetryEnabled = !telemetryEnabled;
            }

            if (telemetryEnabled)
            {
                telemetry.addData("Drive Mode", drive.getMode());
                telemetry.addData("Heading", Math.toDegrees(localization.getHeading()));
                telemetry.addData("Launcher RPM", outtake.getRPM());
                telemetry.update();
            }

            macros.update();
            localization.update();
            transfer.update();
            outtake.update();
        }

        localization.close();
    }
}