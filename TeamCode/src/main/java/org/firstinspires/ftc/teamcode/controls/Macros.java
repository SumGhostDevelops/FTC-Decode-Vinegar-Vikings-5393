package org.firstinspires.ftc.teamcode.controls;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Gamepads;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.RobotContext;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.odometry.Localization;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.function.Supplier;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Macros
{
    private final RobotContext robot;
    private List<Event> events;

    public Macros(RobotContext robot)
    {
        this.robot = robot;
        this.events = new ArrayList<>();
    }
    public void resetTransferOuttake()
    {
        Event event = new Event(Macro.RESET_TRANSFER_OUTTAKE, 2.0, Subsystem.TRANSFER, Subsystem.OUTTAKE);

        if (!handleNewEvent(event))
        {
            return;
        }

        robot.outtake.setPower(-0.5);
        robot.transfer.setPower(-0.5);
    }

    public void resetTransferOuttakeNonFSM()
    {
        robot.outtake.reset();
        robot.transfer.reset();
    }

    public void stopResetTransferOuttakeNonFSM()
    {
        robot.outtake.stopResetting();
        robot.transfer.stopResetting();
    }

    public void toggleFireWhenReady()
    {
        Event event = new Event(Macro.FIRE_WHEN_READY, -1, Subsystem.TRANSFER, Subsystem.OUTTAKE);

        if (!handleNewEvent(event))
        {
            return;
        }

        event.setPeriodicTask(() ->
        {
            if (robot.outtake.isReadyToLaunch())
            {
                robot.transfer.setPower(RobotConstants.TRANSFER_POWER_FWD);
            }
            else
            {
                robot.transfer.stop();
            }
        });
    }

    public void toggleAprilTagAim(int targetTagId)
    {
        // 1. Define the Event (Runs indefinitely until toggled off)
        Event event = new Event(Macro.AIM_AT_TAG, -1.0, Subsystem.DRIVE); // Uses Drive

        // 2. Handle Toggling
        if (!handleNewEvent(event)) {
            return;
        }

        // 3. Create a state container to hold "Memory" between loops
        // This effectively replaces the variables that used to be outside your do-while loop
        class AimControlState {
            ElapsedTime timer = new ElapsedTime();
            double lastError = 0;
            boolean targetFound = false;
        }

        AimControlState state = new AimControlState();

        // 4. Set the Periodic Task
        event.setPeriodicTask(() -> {
            robot.localization.webcam.updateDetections();
            // --- A. GET VISION DATA ---
            double currentHeading = 0;
            boolean tagVisible = false;

            // (Replace this line with your actual camera syntax)
            AprilTagDetection detection = robot.localization.webcam.getSingleDetection(robot.team.goal.id);

            if (detection != null) {
                // In FTC AprilTag coordinates, 'Bearing' is usually the turning error
                currentHeading = detection.ftcPose.bearing;
                tagVisible = true;
            }

            // --- B. CALCULATE PID (Only if tag is visible) ---
            double turnPower = 0;

            if (tagVisible) {
                double targetAngle = 0; // We want 0 degrees bearing (centered)

                // Calculate Error (Target - Current)
                // Note: AprilTag Bearing is usually -180 to 180 already.
                // If it isn't, use your RobotMath.convert360AngleTo180 here.
                double error = targetAngle - currentHeading;

                // Calculate Derivative
                double currentTime = state.timer.seconds();
                double derivative = 0;
                if (currentTime > 0.001) {
                    derivative = (error - state.lastError) / currentTime;
                }
                state.timer.reset(); // Reset for next loop
                state.lastError = error;

                // PID Constants (Tweak these!)
                double kP = 0.02;
                double kD = 0.002;
                double minTurn = 0.1;

                // Calculate Power
                turnPower = (error * kP) + (derivative * kD);

                // Apply Min Power Logic (Friction boost)
                if (Math.abs(turnPower) < minTurn && Math.abs(error) > 1.0) {
                    turnPower = Math.signum(turnPower) * minTurn;
                }

                // Clamp
                turnPower = Math.max(-1.0, Math.min(1.0, turnPower));
            }
            else {
                // Failsafe: If we lose the tag, stop turning (or hold last power)
                turnPower = 0;
                state.lastError = 0; // Reset derivative
            }

            // --- C. MIX INPUTS (Auto Turn + Manual Drive) ---
            // This allows you to strafe around the target while locked onto it!

            double drive = -robot.gamepads.gamepad1.left_stick_y; // Forward/Back
            double strafe = robot.gamepads.gamepad1.left_stick_x; // Left/Right
            // notice we IGNORE right_stick_x and use 'turnPower' instead

            // Send to Drive Subsystem
            robot.drive.setDrivePowers(drive, strafe, turnPower);
        });
    }

    public void aimToAprilTag(int id)
    {
        double maxDistance = 3.5; // meters, used for scaling the offset
        double distanceToTag = robot.localization.webcam.getRangeToTag(robot.team.goal.id);
        double angleOffset;

        switch (robot.team)
        {
            case BLUE:
            {
                angleOffset = RobotConstants.FORCED_ANGLE_OFFSET;
                break;
            }
            case RED:
            {
                angleOffset = -RobotConstants.FORCED_ANGLE_OFFSET;
                break;
            }
            default:
            {
                angleOffset = 0.0;
            }
        }

        angleOffset *= distanceToTag / maxDistance; // Linearly reduce the angleOffset as we get closer to the AprilTag

        aimToAprilTag(id, angleOffset);
    }

    public void aimToAprilTag(int id, double manualAngleOffset)
    {
        robot.telemetry.log().add("-aimToAprilTag---------");
        robot.localization.webcam.updateDetections();

        AprilTagDetection tag = robot.localization.webcam.getSingleDetection(id);

        if (tag == null)
        {
            robot.telemetry.log().add("AprilTag with ID " + id + " was not found.");
            return;
        }

        double offset = tag.ftcPose.bearing;
        double currentAngle = robot.localization.getHeading();
        double targetAngle = currentAngle + offset + manualAngleOffset; // No manual angleOffset

        aimToAngle(targetAngle);
    }

    private void aimToAngle(double targetAngle) // Blocking action/non-FSM
    {
        robot.telemetry.log().add("-aimToAngle---------");
        robot.telemetry.update();

        double tolerance = 1.5; // Angle tolerance in degrees (e.g., 1 degree)

        // Initialize variables outside the loop
        double error;
        double motorPower;
        double lastError;
        double derivative;

        double kP = 0.01;
        double kD = 0.002;
        double minTurnPower = 0.1;

        // CRITICAL FIX: Use a dedicated timer for derivative calculation
        ElapsedTime loopTimer = new ElapsedTime();
        double loopTime;

        // Calculate initial error and save it
        double currentAngle = robot.localization.getHeading();

        // Assuming RobotMath.convert360AngleTo180 correctly wraps the error difference
        error = targetAngle - currentAngle;
        lastError = error;

        // Reset timer to start measuring the first loop iteration time
        loopTimer.reset();

        do
        {
            // 1. Get new angle and calculate time elapsed
            currentAngle = robot.localization.getHeading();
            loopTime = loopTimer.seconds();
            loopTimer.reset(); // Reset timer at the beginning of the loop

            // 2. Calculate the error (difference between target and current)
            error = targetAngle - currentAngle;

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
            robot.drive.setDrivePowers(-motorPower, -motorPower, motorPower, motorPower);

            // 8. Update for next iteration
            lastError = error;
        } while (Math.abs(error) > tolerance && robot.opModeIsActive.get() && robot.gamepads.gamepad1.yWasPressed());

        // Stop and clean up
        robot.drive.stop();

        robot.telemetry.log().add("Finished turning.");
        robot.telemetry.log().add("Final error: " + String.format("%.1f", error));
        robot.telemetry.update();
    }

    /**
     * Changes the value of the target RPM and turns the flywheel on.
     * @param distanceToTag
     */
    public void autoSetOuttakeTargetRPM(double distanceToTag)
    {
        // RobotConstants.OUTTAKE_TARGET_RPM = regression
        robot.outtake.setRPM(RobotConstants.OUTTAKE_TARGET_RPM);
        robot.telemetry.log().add("Flywheel activated and set to a target RPM of " + RobotConstants.OUTTAKE_TARGET_RPM);
        robot.telemetry.update();
    }

    public void sleep(double seconds)
    {
        sleep(seconds, robot.telemetry);
    }

    public void sleep(double seconds, String reason)
    {
        sleep(seconds, reason, robot.telemetry);
    }

    public static void sleep(double seconds, Telemetry telemetry)
    {
        sleep(seconds, "", telemetry);
    }

    public static void sleep(double seconds, String reason, Telemetry telemetry)
    {
        long milliseconds = (long) (seconds * 1000);

        String result = "Sleeping for " + seconds + "seconds.";

        if (!reason.isEmpty())
        {
            result += " Reason: " + reason;
        }

        telemetry.log().add(result);
        telemetry.update();

        try
        {
            Thread.sleep(milliseconds);
        }
        catch (InterruptedException e)
        {
            telemetry.log().add("Interrupted while sleeping: " + e.getMessage());
            Thread.currentThread().interrupt();
        }

        telemetry.log().add("Done sleeping.");
        telemetry.update();
    }

    public void update()
    {
        // Use Iterator to safely remove items while looping
        Iterator<Event> iterator = events.iterator();
        while (iterator.hasNext())
        {
            Event e = iterator.next();

            if (e.hasPeriodicTask())
            {
                e.runPeriodicTask();
            }

            // Check if time is up OR if it was marked inactive by a conflict
            if (e.isOver() || !e.active)
            {
                stopSubsystemsInEvent(e);
                iterator.remove(); // Safely remove from list
            }
        }
    }

    /**
     * Handles the addition of a new event.
     * Cancels the event if an event of the same type is already running.
     * Otherwise, stops any currently running events if the new event requires a subsystem the currently running event is using and adds the event.
     *
     * @param event An event object to handle.
     * @return If the event was added or not.
     */
    private boolean handleNewEvent(Event event)
    {
        // Check if the new event already exists
        if (eventIsActive(event))
        {
            // Stops events associated with that subsystem
            stopSameEvents(event);
            return false; // Cancel the event
        }

        // Check if the new event uses subsystems already in use
        if (subsystemsInUse(event.subsystems))
        {
            // Stop the subsystems associated with that event
            stopEventsWithSubsystems(event.subsystems);
        }

        events.add(event);
        return true;
    }

    /**
     * Stops all of the subsystems in Event.subsystems
     *
     * @param event An event whose subsystems to stop.
     */
    private void stopSubsystemsInEvent(Event event)
    {
        for (Subsystem s : event.subsystems)
        {
            switch (s)
            {
                case INTAKE:
                    if (!robot.intake.isIdle()) robot.intake.stop();
                    break;
                case TRANSFER:
                    if (!robot.transfer.isIdle()) robot.transfer.stop();
                    break;
                case OUTTAKE:
                    if (!robot.outtake.isIdle()) robot.outtake.stop();
                    break;
            }
        }
    }

    /**
     * Checks if an event is already exists/is running
     *
     * @param event The event to check for
     * @return If the event already exists/is running
     */
    private boolean eventIsActive(Event event)
    {
        for (Event e : events)
        {
            if (e.name == event.name)
            {
                return true;
            }
        }
        return false;
    }

    private boolean eventIsActive(Macro event)
    {
        for (Event e: events)
        {
            if (e.name == event)
            {
                return true;
            }
        }
        return false;
    }

    /**
     * Checks if any subsystems in a provided list is already in use in any currently active event
     *
     * @param subsystems
     * @return
     */
    private boolean subsystemsInUse(List<Subsystem> subsystems)
    {
        for (Event e : events)
        {
            if (e.containsAnySubsystem(subsystems))
            {
                return true;
            }
        }
        return false;
    }

    /**
     * Stops any event which contains any subsystem
     *
     * @param subsystems A List of Subsystems
     */
    private void stopEventsWithSubsystems(List<Subsystem> subsystems)
    {
        for (Event e : events)
        {
            if (e.containsAnySubsystem(subsystems))
            {
                e.active = false;
                stopSubsystemsInEvent(e);
            }
        }
    }

    // Helper to stop a specific macro by name
    private void stopSameEvents(Event event)
    {
        Iterator<Event> iterator = events.iterator();
        while (iterator.hasNext())
        {
            Event e = iterator.next();
            if (e.name == event.name)
            {
                stopSubsystemsInEvent(e);
                iterator.remove();
            }
        }
    }
}