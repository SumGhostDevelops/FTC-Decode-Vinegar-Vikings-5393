package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.exceptions.NoTagsDetectedException;
import org.firstinspires.ftc.teamcode.exceptions.TooManyTagsDetectedException;
import org.firstinspires.ftc.teamcode.exceptions.UnexpectedTagIDException;
import org.firstinspires.ftc.teamcode.util.RobotMath;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class Actions
{
    public static void turnToAngle(Robot robot, double targetAngle)
    {
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

            robot.telemetry.addData("Current Angle", currentAngle);
            robot.telemetry.addData("Target Angle", targetAngle);
            robot.telemetry.addData("Error", error);
            robot.telemetry.update();

            // The loop continues as long as the robot is not within the tolerance range and the opmode is active.
        } while (Math.abs(error) > tolerance && robot.opModeIsActive.get() && !gamepad1.yWasPressed());

        // Stop all motors once the turn is complete.
        robot.hub.leftFront.setPower(0);
        robot.hub.leftBack.setPower(0);
        robot.hub.rightFront.setPower(0);
        robot.hub.rightBack.setPower(0);

        robot.telemetry.addLine("Finished turning!");
        robot.telemetry.update();
    }

    public static int scanObelisk(Robot robot)
    {
        AprilTagDetection tag;

        try
        {
            tag = robot.webcam.scanObelisk();
        }
        catch (NoTagsDetectedException e)
        {
            robot.telemetry.addLine("No tags detected.");
            robot.telemetry.update();
            return -1;
        }
        catch (UnexpectedTagIDException e)
        {
            robot.telemetry.addLine("Tags were detected, but none were a valid obelisk tag.");
            robot.telemetry.update();
            return -1;
        }

        robot.telemetry.addLine("New Obelisk ID: " + tag.id);
        robot.telemetry.update();

        return tag.id;
    }

    public static void aimToAprilTag(Robot robot)
    {
        AprilTagDetection tag;
        robot.webcam.updateDetections();

        try
        {
            tag = robot.webcam.getSingleDetection();
        }
        catch (NoTagsDetectedException | TooManyTagsDetectedException e)
        {
            robot.telemetry.addLine("Auto Aim command cancelled.");
            robot.telemetry.addData("Error: ", e);
            robot.telemetry.update();
            return;
        }

        // Get the yaw from the AprilTag detection. This is how many degrees we need to turn.
        double yawToCorrect = tag.ftcPose.yaw;

        // Get the robot's current heading from the IMU.
        double currentBotHeading = robot.hub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Calculate the absolute target angle for the robot to face.
        double targetAngle = RobotMath.angleAddition(currentBotHeading, yawToCorrect);

        robot.telemetry.addData("Vision Yaw Correction: ", yawToCorrect);
        robot.telemetry.addData("Current Heading: ", currentBotHeading);
        robot.telemetry.addData("Target Heading: ", targetAngle);
        robot.telemetry.update();

        // Call the new PID turning method
        turnToAngle(robot, targetAngle);
    }
}