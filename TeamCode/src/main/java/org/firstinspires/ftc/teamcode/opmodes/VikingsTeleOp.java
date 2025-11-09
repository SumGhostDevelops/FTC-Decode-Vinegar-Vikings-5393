package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Actions;
import org.firstinspires.ftc.teamcode.robot.ControlHub;
import org.firstinspires.ftc.teamcode.robot.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.robot.RobotContext;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Wheels;


@TeleOp(name="VikingsTeleOp")
public class VikingsTeleOp extends LinearOpMode {
    // Arbritary values for stuff
    // TODO: Tune these speeds OR make them easily editable (FTC dashboard?)
    private final double upperSpeedLimit = 0.75;
    private final double lowerSpeedLimit = 0.05;
    private final int goalTagId = 24;
    // Initialize some stuff
    RobotContext robot;

    @Override
    public void runOpMode() throws InterruptedException {
        ControlHub hub = new ControlHub();
        hub.init(hardwareMap, new Pose2d(10, 10, Math.toRadians(Math.PI / 2)));
        AprilTagWebcam aprilTagWebcam = new AprilTagWebcam(new double[]{1424.38, 1424.38, 637.325, 256.774}, hub.camera, true);
        robot = new RobotContext(hub, aprilTagWebcam, telemetry, gamepad1, this::opModeIsActive, new Robot(), new Wheels());

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(

                // depending on what direction the logo is facing on the control hub would determine what orientation is.
                //change depending on what it actually is lmao

                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
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

        robot.wheels.leftFront = (rotY + rotX + rx) / denominator;
        robot.wheels.leftBack = (rotY - rotX + rx) / denominator;
        robot.wheels.rightFront = (rotY - rotX - rx) / denominator;
        robot.wheels.rightBack = (rotY + rotX - rx) / denominator;

        // TODO: Add keybind system for different drivers

        if (gamepad.xWasPressed()) // Panic button; kills all power TODO: Remove later
        {
            robot.wheels.setAllPower(0);
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
            Actions.changeLauncherPower(robot,0.01);
        }

        if (gamepad.dpadDownWasPressed()) // Mostly a demo. Can be removed later. Turns the bot around 180 degrees.
        {
            Actions.changeLauncherPower(robot,-0.01);
        }

        if (gamepad.dpadLeftWasPressed())
        {

        }

        if (gamepad.dpadRightWasPressed())
        {

        }

        if (gamepad.leftBumperWasPressed()) // Lower speed
        {
            if (robot.self.speed - 0.05 >= lowerSpeedLimit)
            {
                robot.self.speed -= 0.05;
            }
            robot.self.updateTelemetry(telemetry);
        }

        if (gamepad.rightBumperWasPressed()) // Increase speed
        {
            if (robot.self.speed + 0.05 <= upperSpeedLimit)
            {
                robot.self.speed += 0.05;
            }
            robot.self.updateTelemetry(telemetry);
        }

        if (gamepad.right_trigger > 0.25) // Shoot
        {
            Actions.manualLaunchBall(robot);
        }
        else
        {
            robot.hub.launcher.setPower(0);
        }

        if (gamepad.left_trigger > 0.25)
        {
            robot.hub.loader.setPower(1);
        }
        else
        {
            robot.hub.loader.setPower(0);
        }

        // Handle movement inputs
        Actions.move(robot);

        robot.telemetry.update();
    }
}
