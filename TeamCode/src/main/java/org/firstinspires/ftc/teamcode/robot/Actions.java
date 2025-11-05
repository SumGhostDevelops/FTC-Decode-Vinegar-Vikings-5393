package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.exceptions.NoTagsDetectedException;
import org.firstinspires.ftc.teamcode.exceptions.TagNotFoundException;
import org.firstinspires.ftc.teamcode.exceptions.TooManyTagsDetectedException;
import org.firstinspires.ftc.teamcode.exceptions.UnexpectedTagIDException;
import org.firstinspires.ftc.teamcode.util.ObeliskHelper;
import org.firstinspires.ftc.teamcode.util.RobotMath;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class Actions
{
    public static void turnToAngle(Robot robot, double targetAngle)
    {
        robot.status.clearExtra();

        robot.telemetry.log().add("-turnToAngle--------");
        // A simple P-controller for turning. You can tune this value.
        double kP = 0.05; // Proportional gain - START with a small value and tune it.
        double error;
        double motorPower;
        double tolerance = 2.0; // The robot is "close enough" if it's within 2 degrees of the target.

        do {
            // The IMU gives us the current angle of the robot.
            double currentAngle = robot.hub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // This is the "error" - how far we are from our target angle.
            error = targetAngle - currentAngle;

            // Calculate the motor power. This is the "Proportional" part of PID.
            // The power will be high when the error is large, and small as we get closer.
            motorPower = error * kP;

            // Apply power to the motors to turn the robot.
            // To turn right (positive error), left wheels go forward and right wheels go backward.
            robot.hub.leftFront.setPower(motorPower);
            robot.hub.leftBack.setPower(motorPower);
            robot.hub.rightFront.setPower(-motorPower);
            robot.hub.rightBack.setPower(-motorPower);

            robot.status.setMode("Automatic (Turning)");
            robot.status.addExtra("Current Angle", String.format("%.1f", motorPower));
            robot.status.addExtra("Target Angle", String.format("%.1f", targetAngle));
            robot.status.addExtra("Error", String.format("%.1f", error));

            robot.status.updateTelemetry(robot.telemetry);
            robot.telemetry.update();

            // The loop continues as long as the robot is not within the tolerance range and the opmode is active.
        } while (Math.abs(error) > tolerance && robot.opModeIsActive.get() && !gamepad1.yWasPressed());

        // Stop all motors once the turn is complete.
        robot.hub.leftFront.setPower(0);
        robot.hub.leftBack.setPower(0);
        robot.hub.rightFront.setPower(0);
        robot.hub.rightBack.setPower(0);

        robot.status.setMode("Manual");
        robot.status.clearExtra();
        robot.telemetry.log().add("Finished turning.");
        robot.status.updateTelemetry(robot.telemetry);
    }

    public static void scanObelisk(Robot robot)
    {
        AprilTagDetection tag;
        robot.telemetry.log().add("-scanObelisk---------");
        robot.webcam.updateDetections();

        try
        {
            tag = robot.webcam.scanObelisk();
        }
        catch (NoTagsDetectedException e)
        {
            robot.telemetry.log().add("No tags detected.");
            robot.telemetry.update();
            return;
        }
        catch (UnexpectedTagIDException e)
        {
            robot.telemetry.log().add("Tags were detected, but none were a valid obelisk tag.");
            robot.telemetry.update();
            return;
        }

        if (tag.id == robot.status.getObeliskId())
        {
            robot.telemetry.log().add("The same obelisk was detected.");
            robot.telemetry.update();
            return;
        }

        if (!ObeliskHelper.isObelisk(tag.id))
        {
            robot.telemetry.log().add("The detected tag was not a valid obelisk tag.");
            robot.telemetry.update();
            return;
        }

        robot.status.setMode("Manual");
        robot.status.setObeliskId(tag.id);
        robot.telemetry.log().add("New Obelisk ID: " + tag.id);
        robot.status.updateTelemetry(robot.telemetry);
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
        double targetAngle = RobotMath.angleAddition(currentBotHeading, yawToCorrect);

        robot.telemetry.log().add("Turning to AprilTag " + tag.id + ".");
        robot.telemetry.update();

        // Call the new PID turning method
        turnToAngle(robot, targetAngle);
    }

    public static void move(Robot robot)
    {
        Wheels wheels = robot.wheels;
        double speedScalar = robot.status.getSpeedScalar();

        robot.hub.leftFront.setPower(wheels.getLeftFrontPower() * speedScalar);
        robot.hub.leftBack.setPower(wheels.getLeftBackPower() * speedScalar);
        robot.hub.rightFront.setPower(wheels.getRightFrontPower() * speedScalar);
        robot.hub.rightBack.setPower(wheels.getRightBackPower() * speedScalar);
    }

    public static void launchBall(Robot robot)
    {
        robot.hub.launcher.setPower(0.75);
        try
        {
            robot.hub.launcher.wait(1000);
        }
        catch (InterruptedException e)
        {
            robot.telemetry.log().add("Launcher waiting failed. Error: " + e.getMessage());
        }
        robot.hub.launcher.setPower(0);
    }

    public static void manualLaunchBall(Robot robot)
    {
        robot.hub.launcher.setPower(robot.gamepad.right_trigger * 0.8);
    }
}