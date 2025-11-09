package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.exceptions.NoTagsDetectedException;
import org.firstinspires.ftc.teamcode.exceptions.TagNotFoundException;
import org.firstinspires.ftc.teamcode.exceptions.UnexpectedTagIDException;
import org.firstinspires.ftc.teamcode.util.ObeliskHelper;
import org.firstinspires.ftc.teamcode.util.RobotMath;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Actions
{
    public static void turnToAngle(RobotContext robot, double targetAngle)
    {
        robot.self.extra.clear();
        robot.telemetry.log().add("-turnToAngle--------");

        // A simple P-controller for turning.
        double kP = 0.05; // Proportional gain
        double error;
        double motorPower;
        double tolerance = 0.5; // Stop when within this many degrees

        do {
            // The IMU gives us the current angle of the robot.
            double currentAngle = robot.hub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // Calculate the error, normalizing it to the -180 to +180 range
            // This finds the shortest path to the target angle.
            error = RobotMath.normalizeAngle(targetAngle - currentAngle);

            // Calculate the motor power.
            motorPower = error * kP;

            // Clamp the motor power to the valid range of -1.0 to 1.0
            motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

            // Apply power to the motors to turn the robot.
            robot.hub.leftFront.setPower(-motorPower);
            robot.hub.leftBack.setPower(-motorPower);
            robot.hub.rightFront.setPower(motorPower);
            robot.hub.rightBack.setPower(motorPower);

            // Telemetry
            robot.self.mode = "automatic";
            robot.self.extra.put("Current Angle", String.format("%.1f", currentAngle));
            robot.self.extra.put("Target Angle", String.format("%.1f", targetAngle));
            robot.self.extra.put("Error", String.format("%.1f", error));
            robot.self.updateTelemetry(robot.telemetry);
            robot.telemetry.update();

        } while (Math.abs(error) > tolerance && robot.opModeIsActive.get() && !robot.gamepad.yWasPressed());

        // Stop all motors
        stopMoving(robot);

        robot.self.mode = "manual";
        robot.self.extra.clear();
        robot.telemetry.log().add("Finished turning.");
        robot.self.updateTelemetry(robot.telemetry);
    }

    public static void newTurnToAngle(RobotContext robot, double targetAngle, double kP, double kD, double minTurnPower)
    {
        robot.self.extra.clear();
        robot.telemetry.log().add("-turnToAngle (PD)--------");

        /*
        // --- Tuning Gains (You MUST tune these) ---
        double kP = 0.07;  // Proportional (The "gas") - Start with this higher
        double kD = 0.002; // Derivative (The "brake") - Start small
        double minTurnPower = 0.1; // Minimum power to overcome friction
         */

        double error;
        double motorPower;
        double tolerance = 1.0; // We can be more precise now

        // Variables for the 'D' term
        double lastError = 0;
        double derivative;
        ElapsedTime timer = new ElapsedTime();

        do {
            double currentAngle = robot.hub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = RobotMath.normalizeAngle(targetAngle - currentAngle);

            // --- 'D' Term Calculation ---
            // This is the "rate of change" of the error, or how fast we're turning.
            derivative = (error - lastError) / timer.seconds();

            // --- PD Power Calculation ---
            // motorPower = (Proportional push) + (Derivative brake)
            motorPower = (error * kP) + (derivative * kD);

            // --- Minimum Power ---
            // If the calculated power is too small, boost it to the min
            // power to overcome static friction.
            if (Math.abs(error) > tolerance)
            {
                if (Math.abs(motorPower) < minTurnPower)
                {
                    motorPower = Math.signum(motorPower) * minTurnPower;
                }
            }

            // Clamp the final power
            motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

            // Apply power
            robot.hub.leftFront.setPower(-motorPower);
            robot.hub.leftBack.setPower(-motorPower);
            robot.hub.rightFront.setPower(motorPower);
            robot.hub.rightBack.setPower(motorPower);

            // Update loop variables for next cycle
            lastError = error;
            timer.reset();

            // Telemetry
            robot.self.mode = "automatic";
            robot.self.extra.put("Current Angle", String.format("%.1f", currentAngle));
            robot.self.extra.put("Target Angle", String.format("%.1f", targetAngle));
            robot.self.extra.put("Error", String.format("%.1f", error));
            robot.self.updateTelemetry(robot.telemetry);
            robot.telemetry.update();

        } while (Math.abs(error) > tolerance && robot.opModeIsActive.get() && !robot.gamepad.yWasPressed());

        // Stop all motors
        stopMoving(robot);

        robot.self.mode = "manual";
        robot.self.extra.clear();
        robot.telemetry.log().add("Finished turning.");
        robot.self.updateTelemetry(robot.telemetry);
    }

    public static void scanObelisk(RobotContext robot)
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

        if (tag.id == robot.self.obeliskId)
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

        robot.self.mode = "manual";
        robot.self.obeliskId = tag.id;
        robot.telemetry.log().add("New Obelisk ID: " + tag.id);
        robot.self.updateTelemetry(robot.telemetry);
    }

    public static void aimToAprilTag(RobotContext robot, int tagId)
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

    private static void aimToAprilTag(RobotContext robotContext, AprilTagDetection tag)
    {
        // Get the yaw from the AprilTag detection. This is how many degrees we need to turn.
        double yawToCorrect = tag.ftcPose.yaw;

        // Get the robot's current heading from the IMU.
        double currentBotHeading = robotContext.hub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Calculate the absolute target angle for the robot to face.
        double targetAngle = RobotMath.normalizeAngle(currentBotHeading + yawToCorrect);

        robotContext.telemetry.log().add("Turning to AprilTag " + tag.id + ".");
        robotContext.telemetry.update();

        // Call the new PID turning method
        turnToAngle(robotContext, targetAngle);
    }

    public static void move(RobotContext robot)
    {
        Wheels wheels = robot.wheels;
        double speedScalar = robot.self.speed;

        robot.hub.leftFront.setPower(RobotMath.clampPower(wheels.leftFront * speedScalar));
        robot.hub.leftBack.setPower(RobotMath.clampPower(wheels.leftBack * speedScalar));
        robot.hub.rightFront.setPower(RobotMath.clampPower(wheels.rightFront * speedScalar));
        robot.hub.rightBack.setPower(RobotMath.clampPower(wheels.rightBack * speedScalar));
    }

    public static void stopMoving(RobotContext robot)
    {
        robot.wheels.setAllPower(0);
        move(robot);
    }

    public static void launchBall(RobotContext robot)
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

    public static void manualLaunchBall(RobotContext robot)
    {
        robot.hub.launcher.setPower(robot.launcher.power);
    }

    public static void changeLauncherPower (RobotContext robot, Double change)
    {
        Launcher launcher = robot.launcher;

        launcher.power = Math.max(0.7, Math.min(1, launcher.power + change));
        robot.telemetry.log().add("New Launcher Power: " + launcher.power);
    }
}