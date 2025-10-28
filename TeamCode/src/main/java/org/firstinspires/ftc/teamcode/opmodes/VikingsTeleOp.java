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

        if (gamepad.leftBumperWasPressed()) // Lower speed
        {
            if (powerMultiplier > lowerMultiplierLimit)
            {
                powerMultiplier -= 0.05;
            }
            telemetry.addData("Power Multiplier: ", powerMultiplier);
            telemetry.update();
        }

        if (gamepad.rightBumperWasPressed()) // Increase speed
        {
            if (powerMultiplier < upperMultiplierLimit)
            {
                powerMultiplier -= 0.05;
            }
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

    /*
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
        double yawToCorrect = tag.ftcPose.yaw;

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
     */

    private void autoAim()
    {
        AprilTagDetection tag;
        aprilTagWebcam.updateDetections();

        try
        {
            tag = aprilTagWebcam.getSingleDetection();
        }
        catch (NoTagsDetectedException | TooManyTagsDetectedException e)
        {
            telemetry.addLine("Auto Aim command cancelled.");
            telemetry.addData("Error: ", e);
            telemetry.update();
            return;
        }

        // Get the yaw from the AprilTag detection. This is how many degrees we need to turn.
        double yawToCorrect = tag.ftcPose.yaw;

        // Get the robot's current heading from the IMU.
        double currentBotHeading = hub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Calculate the absolute target angle for the robot to face.
        double targetAngle = currentBotHeading + yawToCorrect;

        telemetry.addData("Vision Yaw Correction: ", yawToCorrect);
        telemetry.addData("Current Heading: ", currentBotHeading);
        telemetry.addData("Target Heading: ", targetAngle);
        telemetry.update();

        // Call the new PID turning method
        turnToAnglePID(targetAngle);
    }

    private void turnToAnglePID(double targetAngle) {
        // A simple P-controller for turning. You can tune this value.
        double kP = 0.05; // Proportional gain - START with a small value and tune it.
        double error;
        double motorPower;
        double tolerance = 2.0; // The robot is "close enough" if it's within 2 degrees of the target.

        do {
            // The IMU gives us the current angle of the robot.
            double currentAngle = hub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // This is the "error" - how far we are from our target angle.
            error = targetAngle - currentAngle;

            // Calculate the motor power. This is the "Proportional" part of PID.
            // The power will be high when the error is large, and small as we get closer.
            motorPower = error * kP;

            // Apply power to the motors to turn the robot.
            // To turn right (positive error), left wheels go forward and right wheels go backward.
            hub.leftFront.setPower(motorPower);
            hub.leftBack.setPower(motorPower);
            hub.rightFront.setPower(-motorPower);
            hub.rightBack.setPower(-motorPower);

            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Error", error);
            telemetry.update();

            // The loop continues as long as the robot is not within the tolerance range and the opmode is active.
        } while (Math.abs(error) > tolerance && opModeIsActive());

        // Stop all motors once the turn is complete.
        hub.leftFront.setPower(0);
        hub.leftBack.setPower(0);
        hub.rightFront.setPower(0);
        hub.rightBack.setPower(0);

        telemetry.addLine("Finished turning!");
        telemetry.update();
    }

}