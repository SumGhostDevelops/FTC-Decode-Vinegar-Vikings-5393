package org.firstinspires.ftc.teamcode.opmodes.experiments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.controls.Macros;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Gamepads;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.odometry.Localization;
import org.firstinspires.ftc.teamcode.subsystems.odometry.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "AprilTagComponentDistance", group = "Experiments")
public class AprilTagComponentDistance extends LinearOpMode
{
    private RobotHardware robot;
    private Localization localization;
    private Drive drive;
    private Intake intake;
    private Outtake outtake;
    private Transfer transfer;
    private Webcam webcam;
    private Macros macros;
    private Gamepads gamepads;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new RobotHardware(hardwareMap, telemetry);
        localization = new Localization(robot);
        drive = new Drive(robot, localization);
        outtake = new Outtake(robot);
        intake = new Intake(robot);
        transfer = new Transfer(robot);
        webcam = new Webcam(robot, DistanceUnit.INCH);

        gamepads = new Gamepads(gamepad1, gamepad2);

        waitForStart();

        while (opModeIsActive())
        {
            run();
        }
    }

    private void run()
    {
        waitForStart();

        boolean telemetryEnabled = true;
        // --- Gamepad 1: Drive ---
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        // EXPERIMENT: Component Distance from AprilTag
        if (gamepad1.aWasPressed())
        {
            webcam.updateDetections();
            int goalId = webcam.getAnyGoalId();

            if (goalId < 0)
            {
                telemetry.log().add("No Goal Tag Found");
                return;
            }

            telemetry.log().add("Updated Displacements");

            AprilTagDetection tag = webcam.getSingleDetection(goalId);

            double xDisplacement = tag.ftcPose.x;
            double yDisplacement = tag.ftcPose.y;

            telemetry.addData("x Displacement (inches)", xDisplacement);
            telemetry.addData("y Displacement (inches)", yDisplacement);
        }

        // Reset Heading
        if (gamepad1.optionsWasPressed())
        {
            localization.resetHeading();
        }

        if (gamepad1.yWasPressed())
        {
            drive.toggleDriveMode();
        }

        // Drive
        drive.drive(axial, lateral, yaw, 1.0);

        // --- Telemetry Toggle ---
        if (gamepad1.startWasPressed())
        {
            telemetryEnabled = !telemetryEnabled;
        }

        if (telemetryEnabled)
        {
            telemetry.addData("Drive Mode", drive.getMode());
            telemetry.addData("Heading", Math.toDegrees(localization.getHeading()));
            telemetry.addData("Outtake Target RPM", outtake.getTargetRPM());
            telemetry.addData("Outtake RPM", outtake.getRPM());
            telemetry.update();
        }
    }
}
