package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.exceptions.NoTagsDetectedException;
import org.firstinspires.ftc.teamcode.exceptions.TagNotFoundException;
import org.firstinspires.ftc.teamcode.robot.Actions;
import org.firstinspires.ftc.teamcode.robot.ControlHub;
import org.firstinspires.ftc.teamcode.robot.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotStatus;
import org.firstinspires.ftc.teamcode.robot.Wheels;
import org.firstinspires.ftc.teamcode.util.RobotMath;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@TeleOp(name="TuneTurningTeleOp")
public class TuneTurningTeleOp extends LinearOpMode {
    // Arbritary values for stuff
    // TODO: Tune these speeds OR make them easily editable (FTC dashboard?)
    private final double upperSpeedLimit = 0.75;
    private final double lowerSpeedLimit = 0.05;
    private final int goalTagId = 24;
    // Initialize some stuff
    Robot robot;

    // for tuning PID values
    private double kP = 0.07;  // Proportional (The "gas") - Start with this higher
    private double kD = 0.002; // Derivative (The "brake") - Start small
    private double minTurnPower = 0.1; // Minimum power to overcome friction

    @Override
    public void runOpMode() throws InterruptedException {
        ControlHub hub = new ControlHub();
        hub.init(hardwareMap, new Pose2d(10, 10, Math.toRadians(Math.PI / 2)));
        AprilTagWebcam aprilTagWebcam = new AprilTagWebcam(new double[]{1424.38, 1424.38, 637.325, 256.774}, hub.camera, true);
        robot = new Robot(hub, aprilTagWebcam, telemetry, gamepad1, this::opModeIsActive, new RobotStatus(), new Wheels());

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(

                // depending on what direction the logo is facing on the control hub would determine what orientation is.
                //change depending on what it actually is lmao

                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        robot.hub.imu.initialize(parameters);
        robot.hub.imu.resetYaw();

        telemetry.setAutoClear(false);
        robot.telemetry.addLine("kP = bumpers, kD = dpad vertical, minTurnPower = dpad horizontal");
        robot.telemetry.addData("kP", kP);
        robot.telemetry.addData("kD", kD);
        robot.telemetry.addData("minTurnPower", minTurnPower);
        robot.telemetry.update();

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
            aimToAprilTag(robot, goalTagId);
        }

        if (gamepad.aWasPressed()) // Scan Obelisk
        {
        }

        if (gamepad.bWasPressed())
        {
            robot.hub.imu.resetYaw();
        }

        if (gamepad.dpadUpWasPressed()) // increase kD
        {
            kD += 0.001;
            robot.telemetry.addData("kD", kD);
            robot.telemetry.update();
        }

        if (gamepad.dpadDownWasPressed()) // decrease kD
        {
            kD -= 0.001;
            robot.telemetry.addData("kD", kD);
            robot.telemetry.update();
        }

        if (gamepad.dpadLeftWasPressed()) // decrease minTurnPower
        {
            minTurnPower -= 0.01;
            robot.telemetry.addData("minTurnPower", minTurnPower);
            robot.telemetry.update();
        }

        if (gamepad.dpadRightWasPressed()) // increase minTurnPower
        {
            minTurnPower += 0.01;
            robot.telemetry.addData("minTurnPower", minTurnPower);
            robot.telemetry.update();
        }

        if (gamepad.leftBumperWasPressed()) // increase kp
        {
            kD += 0.01;
            robot.telemetry.addData("kP", kP);
            robot.telemetry.update();
        }

        if (gamepad.rightBumperWasPressed()) // decrease kp
        {
            kD -= 0.01;
            robot.telemetry.addData("kP", kP);
            robot.telemetry.update();
        }

        if (gamepad.right_trigger > 0.05) // Shoot
        {
        }
        else
        {
        }

        if (gamepad.left_trigger > 0.25)
        {
        }
        else
        {
            robot.hub.loader.setPower(0);
        }

        // Handle movement inputs
        Actions.move(robot);

        robot.telemetry.update();
    }

    public static void aimToAprilTag(Robot robot, int tagId)
    {
        robot.telemetry.log().add("-aimToAprilTag---------");
        AprilTagDetection tag;
        robot.webcam.updateDetections();

        try
        {
            tag = robot.webcam.getSingleDetection(tagId);
        }
        catch (NoTagsDetectedException | TagNotFoundException e)
        {
            robot.telemetry.log().add("Auto Aim command cancelled. Error: " + e.getMessage());
            return;
        }

        aimToAprilTag(robot, tag);
    }

    private static void aimToAprilTag(Robot robot, AprilTagDetection tag)
    {
        // Get the yaw from the AprilTag detection. This is how many degrees we need to turn.
        double yawToCorrect = tag.ftcPose.yaw;

        // Get the robot's current heading from the IMU.
        double currentBotHeading = robot.hub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Calculate the absolute target angle for the robot to face.
        double targetAngle = RobotMath.normalizeAngle(currentBotHeading + yawToCorrect);

        robot.telemetry.log().add("Turning to AprilTag " + tag.id + ".");
        robot.telemetry.update();

        // Call the new PID turning method
        Actions.newTurnToAngle(robot, targetAngle);
    }
}
