package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Event
{
    public final BasicEnum name;
    private final double duration;
    public final List<Subsystem> subsystems;

    private final ElapsedTime timer;
    public boolean active = true; // If set inactive, the Event will be forcibly set for deletion
    public Runnable periodicTask = null; // If not null, the Task has logic that needs to be rechecked

    // Varargs (...) makes this easier to call
    public Event(BasicEnum name, Subsystem... subsystems)
    {
        this(name, Double.MAX_VALUE, subsystems);
    }

    /**
     *
     * @param name The name of the Event
     * @param duration How long the Event should run from the time it is initialized
     * @param subsystems The subsystems used in the event
     */
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