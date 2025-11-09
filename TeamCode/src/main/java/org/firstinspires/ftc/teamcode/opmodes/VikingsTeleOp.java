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
    RobotContext robotContext;

    @Override
    public void runOpMode() throws InterruptedException {
        ControlHub hub = new ControlHub();
        hub.init(hardwareMap, new Pose2d(10, 10, Math.toRadians(Math.PI / 2)));
        AprilTagWebcam aprilTagWebcam = new AprilTagWebcam(new double[]{1424.38, 1424.38, 637.325, 256.774}, hub.camera, true);
        robotContext = new RobotContext(hub, aprilTagWebcam, telemetry, gamepad1, this::opModeIsActive, new Robot(), new Wheels());

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(

                // depending on what direction the logo is facing on the control hub would determine what orientation is.
                //change depending on what it actually is lmao

                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        robotContext.hub.imu.initialize(parameters);
        robotContext.hub.imu.resetYaw();

        telemetry.setAutoClear(false);

        waitForStart();
        while (opModeIsActive()) {
            motorAction(gamepad1);
        }

        robotContext.webcam.close();
    }

    public void motorAction(Gamepad gamepad) throws InterruptedException {
        double y = -gamepad.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad.left_stick_x;
        double rx = gamepad.right_stick_x;
        double botHeading = robotContext.hub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        robotContext.wheels.leftFront = (rotY + rotX + rx) / denominator;
        robotContext.wheels.leftBack = (rotY - rotX + rx) / denominator;
        robotContext.wheels.rightFront = (rotY - rotX - rx) / denominator;
        robotContext.wheels.rightBack = (rotY + rotX - rx) / denominator;

        // TODO: Add keybind system for different drivers

        if (gamepad.xWasPressed()) // Panic button; kills all power TODO: Remove later
        {
            robotContext.wheels.setAllPower(0);
        }

        if (gamepad.yWasPressed()) // Auto aim to opposite AprilTag
        {
            Actions.aimToAprilTag(robotContext, goalTagId);
        }

        if (gamepad.aWasPressed()) // Scan Obelisk
        {
            Actions.scanObelisk(robotContext);
        }

        if (gamepad.bWasPressed())
        {
            robotContext.hub.imu.resetYaw();
        }

        if (gamepad.dpadUpWasPressed())
        {
            Actions.changeLauncherPower(robotContext,0.01);
        }

        if (gamepad.dpadDownWasPressed()) // Mostly a demo. Can be removed later. Turns the bot around 180 degrees.
        {
            Actions.changeLauncherPower(robotContext,-0.01);
        }

        if (gamepad.dpadLeftWasPressed())
        {

        }

        if (gamepad.dpadRightWasPressed())
        {

        }

        if (gamepad.leftBumperWasPressed()) // Lower speed
        {
            if (robotContext.status.speedScalar - 0.05 >= lowerSpeedLimit)
            {
                robotContext.status.speedScalar -= 0.05;
            }
            robotContext.status.updateTelemetry(telemetry);
        }

        if (gamepad.rightBumperWasPressed()) // Increase speed
        {
            if (robotContext.status.speedScalar + 0.05 <= upperSpeedLimit)
            {
                robotContext.status.speedScalar += 0.05;
            }
            robotContext.status.updateTelemetry(telemetry);
        }

        if (gamepad.right_trigger > 0.05) // Shoot
        {
            Actions.manualLaunchBall(robotContext);
        }
        else
        {
            robotContext.hub.launcher.setPower(0);
        }

        if (gamepad.left_trigger > 0.25)
        {
            robotContext.hub.loader.setPower(1);
        }
        else
        {
            robotContext.hub.loader.setPower(0);
        }

        // Handle movement inputs
        Actions.move(robotContext);

        robotContext.telemetry.update();
    }
}
