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
    private RobotContext robot;

    public Actions(RobotContext robot)
    {
        this.robot = robot;
    }

    public void turnToAngle(double targetAngle)
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
            error = RobotMath.convert360AngleTo180(targetAngle - currentAngle);

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
            robot.telemetry.log().add("Current Angle: " + String.format("%.1f", currentAngle));
            robot.telemetry.log().add("Target Angle: " + String.format("%.1f", targetAngle));
            robot.telemetry.log().add("Error: " + String.format("%.1f", error));
            robot.self.updateTelemetry(robot);

        } while (Math.abs(error) > tolerance && robot.opModeIsActive.get() && !robot.gamepad.yWasPressed());

        // Stop all motors
        stopMoving();

        robot.self.mode = "manual";
        robot.self.extra.clear();
        robot.telemetry.log().add("Finished turning.");
        robot.self.updateTelemetry(robot);
    }

    public void newTurnToAngle(double targetAngle, double kP, double kD, double minTurnPower)
    {
        robot.self.extra.clear();
        robot.telemetry.log().add("-turnToAngle (PD)--------");

        double error;
        double motorPower;
        double tolerance = 1.0; // Angle tolerance in degrees

        // Variables for the 'D' term
        double lastError;
        double derivative;
        ElapsedTime timer = new ElapsedTime();

        // --- FIX: Initialize variables *before* the loop ---
        // Get the first error reading
        double currentAngle = robot.hub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        error = RobotMath.convert360AngleTo180(targetAngle - currentAngle);

        // Set lastError to the current error so the D-term is 0 on the first loop
        // This prevents the initial "derivative spike"
        lastError = error;

        // Reset the timer right before the loop starts
        timer.reset();
        // --- End of Fix ---

        do {
            // Get new angle and calculate error
            currentAngle = robot.hub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = RobotMath.convert360AngleTo180(targetAngle - currentAngle);

            // --- 'D' Term Calculation ---
            // Calculate the rate of change (how fast the error is changing)
            derivative = (error - lastError) / timer.seconds();

            // --- PD Power Calculation ---
            // motorPower = (Proportional push) + (Derivative brake)
            motorPower = (error * kP) + (derivative * kD);

            // --- Minimum Power ---
            // If the calculated power is too small but we're not at the target,
            // boost it to the min power to overcome static friction.
            if (Math.abs(error) > tolerance)
            {
                if (Math.abs(motorPower) < minTurnPower)
                {
                    // Apply the sign of the motorPower (or error) to the min power
                    motorPower = Math.signum(motorPower) * minTurnPower;
                }
            }

            // Clamp the final power to the -1.0 to 1.0 range
            motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

            // Apply power to motors for a point turn
            robot.hub.leftFront.setPower(-motorPower);
            robot.hub.leftBack.setPower(-motorPower);
            robot.hub.rightFront.setPower(motorPower);
            robot.hub.rightBack.setPower(motorPower);

            // Update loop variables for next cycle
            lastError = error;
            timer.reset();

            // Telemetry
            robot.self.mode = "automatic";
            robot.telemetry.log().add("Current Angle" + String.format("%.1f", currentAngle));
            robot.telemetry.log().add("Target Angle" + String.format("%.1f", targetAngle));
            robot.telemetry.log().add("Error" + String.format("%.1f", error));
            robot.telemetry.log().add("Power" + String.format("%.2f", motorPower));
            robot.telemetry.log().add("Deriv" + String.format("%.2f", derivative));
            robot.self.updateTelemetry(robot);

        } while (Math.abs(error) > tolerance && robot.opModeIsActive.get() && !robot.gamepad.yWasPressed());

        // Stop all motors
        stopMoving();

        robot.self.mode = "manual";
        robot.self.extra.clear();
        robot.telemetry.log().add("Finished turning.");
        robot.self.updateTelemetry(robot);
    }

    /**
     * Turns the robot to a specific target angle using an IMU and a PD controller.
     * @param targetAngle The desired angle, expected to be in the -180 to +180 range.
     * @param kP Proportional gain.
     * @param kD Derivative gain.
     * @param minTurnPower The minimum motor power to overcome static friction.
     */
    public void imuTurnToAngle(double targetAngle, double kP, double kD, double minTurnPower)
    {
        robot.telemetry.log().add("-imuTurnToAngle---------");
        robot.self.mode = "automatic";
        robot.self.updateTelemetry(robot);

        double tolerance = 1.0; // Angle tolerance in degrees (e.g., 1 degree)

        // Initialize variables outside the loop
        double error;
        double motorPower;
        double lastError;
        double derivative;

        // CRITICAL FIX: Use a dedicated timer for derivative calculation
        ElapsedTime loopTimer = new ElapsedTime();
        double loopTime;

        // Calculate initial error and save it
        double currentAngle = robot.hub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Assuming RobotMath.convert360AngleTo180 correctly wraps the error difference
        error = RobotMath.convert360AngleTo180(targetAngle - currentAngle);
        lastError = error;

        // Reset timer to start measuring the first loop iteration time
        loopTimer.reset();

        do
        {
            // 1. Get new angle and calculate time elapsed
            currentAngle = robot.hub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            loopTime = loopTimer.seconds();
            loopTimer.reset(); // Reset timer at the beginning of the loop

            // 2. Calculate the error (difference between target and current)
            error = RobotMath.convert360AngleTo180(targetAngle - currentAngle);

            // 3. Calculate Derivative Term (Handle division by zero safety)
            if (loopTime > 0.0001)
            { // Check for near-zero time to prevent division by zero/oversize D-term
                derivative = (error - lastError) / loopTime;
            }
            else
            {
                derivative = 0.0;
            }

            // 4. Calculate raw motor power (PD Control Law)
            motorPower = (error * kP) + (derivative * kD);

            // 5. Apply minTurnPower (Min Motor Power Logic Fix)
            // Only apply min power if the error is still outside tolerance
            if (Math.abs(error) > tolerance)
            {
                if (Math.abs(motorPower) < minTurnPower)
                {
                    // Boost power to minTurnPower to overcome friction, keeping the same sign/direction
                    motorPower = Math.signum(motorPower) * minTurnPower;
                }
            }

            // 6. Clamp power to the legal range [-1.0, 1.0]
            motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

            // 7. Apply power to motors (Assuming standard mecanum/tank drive)
            robot.hub.leftFront.setPower(-motorPower);
            robot.hub.leftBack.setPower(-motorPower);
            robot.hub.rightFront.setPower(motorPower);
            robot.hub.rightBack.setPower(motorPower);

            // 8. Update for next iteration
            lastError = error;

            // Telemetry update (Keep outside the critical timing loop for best performance)
            robot.telemetry.log().add("Current Angle" + String.format("%.1f", currentAngle));
            robot.telemetry.log().add("Error" + String.format("%.1f", error));
            robot.telemetry.log().add("Power" + String.format("%.2f", motorPower));
            robot.telemetry.log().add("Deriv" + String.format("%.2f", derivative));
            robot.self.updateTelemetry(robot);

        } while (Math.abs(error) > tolerance && robot.opModeIsActive.get() && !robot.gamepad.yWasPressed());

        // Stop and clean up
        stopMoving();

        robot.self.mode = "manual";
        robot.telemetry.log().add("Finished turning.");
        robot.telemetry.log().add("Final error: " + String.format("%.1f", error));
        robot.self.updateTelemetry(robot);
    }

    public void aimToAprilTag(int tagId)
    {
        aimToAprilTag(tagId, 0.01, 0.002, 0.1);
    }

    public void aimToAprilTag(int tagId, double kP, double kD, double minTurnPower)
    {
        AprilTagDetection tag;
        robot.telemetry.log().add("-aimToAprilTag---------");
        robot.webcam.updateDetections();
        try
        {
            tag = robot.webcam.getSingleDetection(tagId);
        }
        catch (NoTagsDetectedException | TagNotFoundException e)
        {
            robot.telemetry.log().add("Cancelling aiming command: " + e.getMessage());
            return;
        }

        aimToAprilTag(tag, kP, kD, minTurnPower);
    }

    private void aimToAprilTag(AprilTagDetection tag, double kP, double kD, double minTurnPower)
    {
        double offset = tag.ftcPose.bearing;

        double currentAngle = robot.hub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double targetAngle = RobotMath.convert360AngleTo180(currentAngle + offset);

        imuTurnToAngle(targetAngle, kP, kD, minTurnPower);
    }

    public void scanObelisk()
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

        if (tag.id == robot.self.getObeliskId())
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
        robot.self.setObeliskId(tag.id);
        robot.telemetry.log().add("New Obelisk ID: " + tag.id);
    }

    public void move()
    {
        Wheels wheels = robot.wheels;
        double speedScalar = robot.self.getSpeed();

        robot.hub.leftFront.setPower(RobotMath.clampPower(wheels.leftFront * speedScalar));
        robot.hub.leftBack.setPower(RobotMath.clampPower(wheels.leftBack * speedScalar));
        robot.hub.rightFront.setPower(RobotMath.clampPower(wheels.rightFront * speedScalar));
        robot.hub.rightBack.setPower(RobotMath.clampPower(wheels.rightBack * speedScalar));
    }

    public void stopMoving()
    {
        robot.wheels.setAllPower(0);
        move();
    }

    public void loadLaunchBall()
    {
        robot.hub.loader.setPower(1);
        sleep(0.5);
        robot.hub.loader.setPower(0);

        robot.self.setLauncherSpeed(0.75);
        robot.hub.launcher.setPower(robot.self.getLauncherSpeed());
        robot.self.updateTelemetry(robot);
        sleep(1);
        robot.hub.launcher.setPower(0);
        robot.self.updateTelemetry(robot);
    }

    public void manualLaunchBall()
    {
        robot.hub.launcher.setPower(robot.self.getLauncherSpeed());
        robot.self.updateTelemetry(robot);
    }

    public void sleep(double seconds)
    {
        long milliseconds = (long) (seconds * 1000);

        robot.telemetry.log().add("Sleeping for " + seconds + "seconds.");
        robot.telemetry.update();

        try
        {
            Thread.sleep(milliseconds);
        }
        catch (InterruptedException e)
        {
            robot.telemetry.log().add("Interrupted while sleeping: " + e.getMessage());
            Thread.currentThread().interrupt();
        }
    }
}