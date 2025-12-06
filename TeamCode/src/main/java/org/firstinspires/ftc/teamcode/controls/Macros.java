package org.firstinspires.ftc.teamcode.controls;

import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Gamepads;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.RobotContext;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.odometry.Localization;
import org.firstinspires.ftc.teamcode.subsystems.odometry.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Macros
{
    private final Team team;
    private List<Event> events;

    // Subsystems
    private final Drive drive;
    private final Intake intake;
    private final Transfer transfer;
    private final Outtake outtake;
    private final Webcam webcam;
    private final Localization localization;
    private Gamepads gamepads;

    public Macros(RobotContext robot)
    {
        this.drive = robot.drive;
        this.intake = robot.intake;
        this.transfer = robot.transfer;
        this.outtake = robot.outtake;
        this.localization = robot.localization;
        this.webcam = robot.webcam;
        this.gamepads = robot.gamepads;
        this.team = robot.team;
        this.events = new ArrayList<>();
    }
    public void resetTransferOuttake()
    {
        Event event = new Event(Macro.RESET_TRANSFER_OUTTAKE, 2.0, Subsystem.TRANSFER, Subsystem.OUTTAKE);

        if (!handleNewEvent(event))
        {
            return;
        }

        outtake.setPower(-0.5);
        transfer.setPower(-0.5);
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
            if (outtake.isReadyToLaunch())
            {
                transfer.setPower(RobotConstants.TRANSFER_POWER_FWD);
            }
            else
            {
                transfer.stop();
            }
        });
    }

    // Inside Macros.java

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
            webcam.updateDetections();
            // --- A. GET VISION DATA ---
            double currentHeading = 0;
            boolean tagVisible = false;

            // (Replace this line with your actual camera syntax)
            AprilTagDetection detection = webcam.getSingleDetection(team.goal.id);

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

            double drive = -gamepads.gamepad1.left_stick_y; // Forward/Back
            double strafe = gamepads.gamepad1.left_stick_x; // Left/Right
            // notice we IGNORE right_stick_x and use 'turnPower' instead

            // Send to Drive Subsystem
            this.drive.setDrivePowers(drive, strafe, turnPower);
        });
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
                    if (!intake.isIdle()) intake.stop();
                    break;
                case TRANSFER:
                    if (!transfer.isIdle()) transfer.stop();
                    break;
                case OUTTAKE:
                    if (!outtake.isIdle()) outtake.stop();
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