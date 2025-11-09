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
    public static void turnToAngle(RobotContext robotContext, double targetAngle)
    {
        robotContext.status.extra.clear();
        robotContext.telemetry.log().add("-turnToAngle--------");

        // A simple P-controller for turning.
        double kP = 0.05; // Proportional gain
        double error;
        double motorPower;
        double tolerance = 0.5; // Stop when within this many degrees

        do {
            // The IMU gives us the current angle of the robot.
            double currentAngle = robotContext.hub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // Calculate the error, normalizing it to the -180 to +180 range
            // This finds the shortest path to the target angle.
            error = RobotMath.normalizeAngle(targetAngle - currentAngle);

            // Calculate the motor power.
            motorPower = error * kP;

            // Clamp the motor power to the valid range of -1.0 to 1.0
            motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

            // Apply power to the motors to turn the robot.
            robotContext.hub.leftFront.setPower(-motorPower);
            robotContext.hub.leftBack.setPower(-motorPower);
            robotContext.hub.rightFront.setPower(motorPower);
            robotContext.hub.rightBack.setPower(motorPower);

            // Telemetry
            robotContext.status.mode = "Automatic (Turning)";
            robotContext.status.extra.put("Current Angle", String.format("%.1f", currentAngle));
            robotContext.status.extra.put("Target Angle", String.format("%.1f", targetAngle));
            robotContext.status.extra.put("Error", String.format("%.1f", error));
            robotContext.status.updateTelemetry(robotContext.telemetry);
            robotContext.telemetry.update();

        } while (Math.abs(error) > tolerance && robotContext.opModeIsActive.get() && !robotContext.gamepad.yWasPressed());

        // Stop all motors
        stopMoving(robotContext);

        robotContext.status.mode = "Manual";
        robotContext.status.extra.clear();
        robotContext.telemetry.log().add("Finished turning.");
        robotContext.status.updateTelemetry(robotContext.telemetry);
    }

    public static void newTurnToAngle(RobotContext robotContext, double targetAngle, double kP, double kD, double minTurnPower)
    {
        robotContext.status.extra.clear();
        robotContext.telemetry.log().add("-turnToAngle (PD)--------");

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
            double currentAngle = robotContext.hub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
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
            robotContext.hub.leftFront.setPower(-motorPower);
            robotContext.hub.leftBack.setPower(-motorPower);
            robotContext.hub.rightFront.setPower(motorPower);
            robotContext.hub.rightBack.setPower(motorPower);

            // Update loop variables for next cycle
            lastError = error;
            timer.reset();

            // Telemetry
            robotContext.status.mode = "Automatic (Turning)";
            robotContext.status.extra.put("Current Angle", String.format("%.1f", currentAngle));
            robotContext.status.extra.put("Target Angle", String.format("%.1f", targetAngle));
            robotContext.status.extra.put("Error", String.format("%.1f", error));
            robotContext.status.updateTelemetry(robotContext.telemetry);
            robotContext.telemetry.update();

        } while (Math.abs(error) > tolerance && robotContext.opModeIsActive.get() && !robotContext.gamepad.yWasPressed());

        // Stop all motors
        stopMoving(robotContext);

        robotContext.status.mode = "Manual";
        robotContext.status.extra.clear();
        robotContext.telemetry.log().add("Finished turning.");
        robotContext.status.updateTelemetry(robotContext.telemetry);
    }

    public static void scanObelisk(RobotContext robotContext)
    {
        AprilTagDetection tag;
        robotContext.telemetry.log().add("-scanObelisk---------");
        robotContext.webcam.updateDetections();

        try
        {
            tag = robotContext.webcam.scanObelisk();
        }
        catch (NoTagsDetectedException e)
        {
            robotContext.telemetry.log().add("No tags detected.");
            robotContext.telemetry.update();
            return;
        }
        catch (UnexpectedTagIDException e)
        {
            robotContext.telemetry.log().add("Tags were detected, but none were a valid obelisk tag.");
            robotContext.telemetry.update();
            return;
        }

        if (tag.id == robotContext.status.obeliskId)
        {
            robotContext.telemetry.log().add("The same obelisk was detected.");
            robotContext.telemetry.update();
            return;
        }

        if (!ObeliskHelper.isObelisk(tag.id))
        {
            robotContext.telemetry.log().add("The detected tag was not a valid obelisk tag.");
            robotContext.telemetry.update();
            return;
        }

        robotContext.status.mode = "Manual";
        robotContext.status.obeliskId = tag.id;
        robotContext.telemetry.log().add("New Obelisk ID: " + tag.id);
        robotContext.status.updateTelemetry(robotContext.telemetry);
    }

    public static void aimToAprilTag(RobotContext robotContext, int tagId)
    {
        robotContext.telemetry.log().add("-aimToAprilTag---------");
        AprilTagDetection tag;
        robotContext.webcam.updateDetections();

        try
        {
            tag = robotContext.webcam.getSingleDetection(tagId);
        }
        catch (NoTagsDetectedException | TagNotFoundException e)
        {
            robotContext.telemetry.log().add("Auto Aim command cancelled. Error: " + e.getMessage());
            return;
        }

        aimToAprilTag(robotContext, tag);
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

    public static void move(RobotContext robotContext)
    {
        Wheels wheels = robotContext.wheels;
        double speedScalar = robotContext.status.speedScalar;

        robotContext.hub.leftFront.setPower(wheels.leftFront * speedScalar);
        robotContext.hub.leftBack.setPower(wheels.leftBack * speedScalar);
        robotContext.hub.rightFront.setPower(wheels.rightFront * speedScalar);
        robotContext.hub.rightBack.setPower(wheels.rightBack * speedScalar);
    }

    public static void stopMoving(RobotContext robotContext)
    {
        robotContext.wheels.setAllPower(0);
        move(robotContext);
    }

    public static void launchBall(RobotContext robotContext)
    {
        robotContext.hub.launcher.setPower(0.75);
        try
        {
            robotContext.hub.launcher.wait(1000);
        }
        catch (InterruptedException e)
        {
            robotContext.telemetry.log().add("Launcher waiting failed. Error: " + e.getMessage());
        }
        robotContext.hub.launcher.setPower(0);
    }

    public static void manualLaunchBall(RobotContext robotContext)
    {
        double launcherPower = Variables.getLauncherPower();
        robotContext.hub.launcher.setPower(launcherPower);
    }

    public static void changeLauncherPower (RobotContext robotContext, Double change)
    {
        Variables.setLauncherPower(Math.max(0.7, Math.min(1, Variables.getLauncherPower()+change)));
        robotContext.telemetry.log().add("New Launcher Power: " + Variables.getLauncherPower());
    }
}