package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.LoaderIntake;
import org.firstinspires.ftc.teamcode.subsystems.Localization;

abstract class Base extends LinearOpMode
{

    // Change this manually or create a config selector OpMode
    protected Team team;

    private RobotHardware robot;
    private Localization localization;
    private Drive drive;
    private Launcher launcher;
    private LoaderIntake intake;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new RobotHardware(hardwareMap, telemetry);
        localization = new Localization(robot);
        drive = new Drive(robot, localization);
        launcher = new Launcher(robot);
        intake = new LoaderIntake(robot);

        telemetry.addData("Status", "Initialized for " + team);
        telemetry.update();

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

            // --- Gamepad 2: Mechanisms ---
            if (gamepad2.right_trigger > 0.5) // TODO: Launcher should eventually be a toggle, and the right trigger only enables loading to happen, but we automatically handle if the loader should be on or off based on the launcher's RPM
            {
                launcher.launch();
                intake.loadBall(); // Feed into flywheel
            }
            else
            {
                launcher.idle();
                intake.stopLoader();
            }

            if (gamepad2.xWasPressed())
            {
                launcher.clearJam();
            }

            // Intake control
            if (gamepad2.aWasPressed()) intake.intakeForward();
            else if (gamepad2.bWasPressed()) intake.intakeReverse();
            else intake.stopIntake();

            // --- Telemetry Toggle ---
            if (gamepad1.startWasPressed())
            {
                telemetryEnabled = !telemetryEnabled;
            }

            if (telemetryEnabled)
            {
                telemetry.addData("Drive Mode", drive.getMode());
                telemetry.addData("Heading", Math.toDegrees(localization.getHeading()));
                telemetry.addData("Launcher RPM", launcher.getRPM());
                telemetry.update();
            }

            localization.update();
        }

        localization.close();
    }
}