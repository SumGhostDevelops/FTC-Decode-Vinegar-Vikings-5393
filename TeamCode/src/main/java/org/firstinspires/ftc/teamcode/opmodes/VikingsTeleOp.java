package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.ControlHub;
import org.firstinspires.ftc.teamcode.exceptions.TooManyTagsDetectedException;
import org.firstinspires.ftc.teamcode.exceptions.NoTagsDetectedException;
import org.firstinspires.ftc.teamcode.util.TurningMath;
import org.firstinspires.ftc.teamcode.robot.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;


@TeleOp(name="VikingsTeleOp")
public class VikingsTeleOp extends LinearOpMode {
    // Arbritary values for stuff
    // TODO: Tune these speeds OR make them easily editable (FTC dashboard?)
    private final double upperMultiplierLimit = 0.75;
    private final double lowerMultiplierLimit = 0.05;
    private double powerMultiplier = 0.5; // initial power reduction value

    private int obeliskId;

    // Initialize some stuff
    ControlHub hub = new ControlHub();
    AprilTagWebcam aprilTagWebcam;
    WebcamName camera = hub.camera;


    @Override
    public void runOpMode() throws InterruptedException {
        hub.init(hardwareMap, new Pose2d(10, 10, Math.toRadians(Math.PI / 2)));
        aprilTagWebcam = new AprilTagWebcam(new double[]{1424.38, 1424.38, 637.325, 256.774}, hub.camera, true);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(

                // depending on what direction the logo is facing on the control hub would determine what orientation is.
                //change depending on what it actually is lmao

                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        hub.imu.initialize(parameters);
        hub.imu.resetYaw();

        waitForStart();
        while (opModeIsActive()) {
            motorAction(gamepad1);
        }

        aprilTagWebcam.close();
    }

    public void motorAction(Gamepad gamepad) throws InterruptedException {
        double y = -gamepad.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad.left_stick_x;
        double rx = gamepad.right_stick_x;
        double botHeading = hub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;


        // TODO: Add keybind system for different drivers

        if (gamepad.xWasPressed()) // Panic button; kills all power TODO: Remove later
        {
            frontLeftPower = 0;
            frontRightPower = 0;
            backLeftPower = 0;
            backRightPower = 0;
        }

        if (gamepad.yWasPressed()) // Auto aim to AprilTag
        {
            autoAim();
        }

        if (gamepad.aWasPressed()) // Scan Obelisk
        {
            AprilTagDetection tag;

            try
            {
                tag = aprilTagWebcam.scanObelisk();
            }
            catch (NoTagsDetectedException e)
            {
                telemetry.addLine("No obelisk tags detected!");
                telemetry.update();
                return;
            }

            telemetry.addLine("Old Obelisk ID: " + obeliskId);
            telemetry.addLine("New Obelisk ID: " + tag.id);
            telemetry.update();

            obeliskId = tag.id;
        }

        if (gamepad.bWasPressed())
        {
            hub.imu.resetYaw();
        }

        if (gamepad.dpadUpWasPressed())
        {

        }

        if (gamepad.dpadDownWasPressed())
        {

        }

        if (gamepad.dpadLeftWasPressed())
        {

        }

        if (gamepad.dpadRightWasPressed())
        {

        }

        if (gamepad.leftBumperWasPressed() && (powerMultiplier > lowerMultiplierLimit)) // Lower speed
        {
            powerMultiplier -= 0.05;
            telemetry.addData("Power Multiplier: ", powerMultiplier);
            telemetry.update();
        }

        if (gamepad.rightBumperWasPressed() && (powerMultiplier < upperMultiplierLimit)) // Increase speed
        {
            powerMultiplier += 0.05;
            telemetry.addData("Power Multiplier: ", powerMultiplier);
            telemetry.update();
        }

        if (gamepad.right_trigger > 0.25) // Shoot
        {

        }

        if (gamepad.left_trigger > 0.25)
        {

        }

        // Handle movement inputs
        hub.leftFront.setPower(frontLeftPower * powerMultiplier);
        hub.leftBack.setPower(backLeftPower * powerMultiplier);
        hub.rightFront.setPower(frontRightPower * powerMultiplier);
        hub.rightBack.setPower(backRightPower * powerMultiplier);

        telemetry.update();
    }

    private void autoAim()
    {
        AprilTagDetection tag;
        aprilTagWebcam.updateDetections();

        // Getting an AprilTag is a dangerous method, so simply restart the iteration if there is an error
        try
        {
            tag = aprilTagWebcam.getSingleDetection(); // TODO: Add code to only aim if the AprilTag ID is ours
        }
        catch (NoTagsDetectedException | TooManyTagsDetectedException e)
        {
            telemetry.addLine("Auto Aim command cancelled.");
            telemetry.addData("Error: ", e);
            telemetry.update();
            return; // Restart motorAction() if there is an error; should have negligible effect on driving
        }

        // Resolve the Yaw and time it takes to turn
        double yaw = tag.ftcPose.yaw;

        double time = TurningMath.Calculate(yaw) * 2.5;
        // TODO: Add a maximum limit for time for competition in case it is some ungodly high number
        telemetry.addData("Vision Yaw: ", yaw);
        telemetry.addData("Calculated Turn Time (s): ", time);
        telemetry.update();

        if (yaw > 0)
        {
            hub.leftFront.setPower(-powerMultiplier);
            hub.leftBack.setPower(-powerMultiplier);
            hub.rightFront.setPower(powerMultiplier);
            hub.rightBack.setPower(powerMultiplier);
        }
        else
        {
            hub.leftFront.setPower(powerMultiplier);
            hub.leftBack.setPower(powerMultiplier);
            hub.rightFront.setPower(-powerMultiplier);
            hub.rightBack.setPower(-powerMultiplier);
        }

        sleep((long) (time * 1000));

        // Stop motors after turning
        hub.leftFront.setPower(0);
        hub.leftBack.setPower(0);
        hub.rightFront.setPower(0);
        hub.rightBack.setPower(0);

        telemetry.addLine("Done turning!");
        telemetry.update();
    }
}