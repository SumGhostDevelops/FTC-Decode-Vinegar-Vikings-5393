package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Actions;
import org.firstinspires.ftc.teamcode.robot.ControlHub;
import org.firstinspires.ftc.teamcode.robot.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotStatus;
import org.firstinspires.ftc.teamcode.robot.Wheels;
import org.firstinspires.ftc.teamcode.util.RobotMath;


@TeleOp(name="VikingsTeleOp")
public class VikingsTeleOp extends LinearOpMode {
    // Arbritary values for stuff
    // TODO: Tune these speeds OR make them easily editable (FTC dashboard?)
    private final double upperSpeedLimit = 0.75;
    private final double lowerSpeedLimit = 0.05;
    private final int goalTagId = 24;

    // Initialize some stuff
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        ControlHub hub = new ControlHub();
        hub.init(hardwareMap, new Pose2d(10, 10, Math.toRadians(Math.PI / 2)));
        AprilTagWebcam aprilTagWebcam = new AprilTagWebcam(new double[]{1424.38, 1424.38, 637.325, 256.774}, hub.camera, true);
        robot = new Robot(hub, aprilTagWebcam, telemetry, gamepad1, this::opModeIsActive, new RobotStatus(), new Wheels());

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(

                // depending on what direction the logo is facing on the control hub would determine what orientation is.
                //change depending on what it actually is lmao

                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        robot.hub.imu.initialize(parameters);
        robot.hub.imu.resetYaw();

        telemetry.setAutoClear(false);

        waitForStart();
        while (opModeIsActive()) {
            motorAction(gamepad1);
        }

        robot.webcam.close();
    }

    public void motorAction(Gamepad gamepad) throws InterruptedException {
        double y = -gamepad.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad.left_stick_x;
        double rx = gamepad.right_stick_x;
        double botHeading = robot.hub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        robot.wheels.setLeftFrontPower((rotY + rotX + rx) / denominator);
        robot.wheels.setLeftBackPower((rotY - rotX + rx) / denominator);
        robot.wheels.setRightFrontPower((rotY - rotX - rx) / denominator);
        robot.wheels.setRightBackPower((rotY + rotX - rx) / denominator);

        // TODO: Add keybind system for different drivers

        if (gamepad.xWasPressed()) // Panic button; kills all power TODO: Remove later
        {
            robot.wheels.setAllWheelsPower(0);
        }

        if (gamepad.yWasPressed()) // Auto aim to opposite AprilTag
        {
            Actions.aimToAprilTag(robot, goalTagId);
        }

        if (gamepad.aWasPressed()) // Scan Obelisk
        {
            Actions.scanObelisk(robot);
        }

        if (gamepad.bWasPressed())
        {
            robot.hub.imu.resetYaw();
        }

        if (gamepad.dpadUpWasPressed())
        {

        }

        if (gamepad.dpadDownWasPressed()) // Mostly a demo. Can be removed later. Turns the bot around 180 degrees.
        {
            double currentBotHeading = robot.hub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            double newAngle = RobotMath.angleAddition(currentBotHeading, 180);

            Actions.turnToAngle(robot, newAngle);
        }

        if (gamepad.dpadLeftWasPressed())
        {

        }

        if (gamepad.dpadRightWasPressed())
        {

        }

        if (gamepad.leftBumperWasPressed()) // Lower speed
        {
            if (robot.status.getSpeed() - 0.05 >= lowerSpeedLimit)
            {
                robot.status.setSpeed(robot.status.getSpeed() - 0.05);
            }
            robot.status.updateTelemetry(telemetry);
        }

        if (gamepad.rightBumperWasPressed()) // Increase speed
        {
            if (robot.status.getSpeed() + 0.05 <= upperSpeedLimit)
            {
                robot.status.setSpeed(robot.status.getSpeed() + 0.05);
            }
            robot.status.updateTelemetry(telemetry);
        }

        if (gamepad.right_trigger > 0.25) // Shoot
        {
            Actions.launchBall(robot);
        }

        if (gamepad.left_trigger > 0.25)
        {

        }

        // Handle movement inputs
        Actions.move(robot);
        robot.telemetry.update();
    }
    /*
    private void autoAim()
    {
        AprilTagDetection tag;
        robot.webcam.updateDetections();

        // Getting an AprilTag is a dangerous method, so simply restart the iteration if there is an error
        try
        {
            tag = robot.webcam.getSingleDetection(); // TODO: Add code to only aim if the AprilTag ID is ours
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
            robot.hub.leftFront.setPower(-powerMultiplier);
            robot.hub.leftBack.setPower(-powerMultiplier);
            robot.hub.rightFront.setPower(powerMultiplier);
            robot.hub.rightBack.setPower(powerMultiplier);
        }
        else
        {
            robot.hub.leftFront.setPower(powerMultiplier);
            robot.hub.leftBack.setPower(powerMultiplier);
            robot.hub.rightFront.setPower(-powerMultiplier);
            robot.hub.rightBack.setPower(-powerMultiplier);
        }

        sleep((long) (time * 1000));

        // Stop motors after turning
        robot.hub.leftFront.setPower(0);
        robot.hub.leftBack.setPower(0);
        robot.hub.rightFront.setPower(0);
        robot.hub.rightBack.setPower(0);

        telemetry.addLine("Done turning!");
        telemetry.update();
    }
     */
}
