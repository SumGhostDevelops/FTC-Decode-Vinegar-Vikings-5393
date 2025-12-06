package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Event
{
    private final double duration;
    private final ElapsedTime timer;
    public final BasicEnum name; // Identifier for the Macro type
    public final List<Subsystem> subsystems; // Changed from BasicEnum to Subsystem
    public boolean active = true;
    private Runnable periodicTask = null;

    // Varargs (...) makes this easier to call
    public Event(BasicEnum name, Subsystem... subsystems)
    {
        this(name, Double.MAX_VALUE, subsystems);
    }

    public Event(BasicEnum name, double duration, Subsystem... subsystems)
    {
        this.name = name;
        this.duration = duration;
        this.timer = new ElapsedTime();
        this.subsystems = new ArrayList<>(Arrays.asList(subsystems));
    }

    /**
     * Sets a task to run every update loop while this event is active.
     */
    public void setPeriodicTask(Runnable task)
    {
        this.periodicTask = task;
    }

    /**
     * Runs the task if it exists.
     */
    public void runPeriodicTask()
    {
        if (periodicTask != null)
        {
            periodicTask.run();
        }
    }

    public boolean hasPeriodicTask()
    {
        return periodicTask != null;
    }

    public boolean isOver()
    {
        if (duration < 0)
        {
            return false;
        }

        return timer.seconds() >= duration;
    }

    public boolean containsAnySubsystem(List<Subsystem> newSubsystems)
    {
        for (Subsystem s : newSubsystems)
        {
            if (this.subsystems.contains(s))
            {
                return true;
            }
        }
        return false;
    }
}