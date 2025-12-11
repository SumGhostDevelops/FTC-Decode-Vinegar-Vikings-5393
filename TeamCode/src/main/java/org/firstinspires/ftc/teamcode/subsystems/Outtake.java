package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

// Make sure this import points to where you saved the helper class I wrote earlier
import org.firstinspires.ftc.teamcode.controls.OuttakeTracker;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;

public class Outtake extends Actuator
{
    private final RobotHardware robot;

    // Timer runs continuously to provide a timeline for the regression math.
    // Do not reset this timer inside the loop.
    private final ElapsedTime stabilityTimer = new ElapsedTime();

    private int targetRPM = RobotConstants.OUTTAKE_INITIAL_TARGET_RPM;
    private double currentAcceleration = 0.0; // Stores the calculated acceleration (slope)

    // A buffer size of 7 is a good balance between smoothing and responsiveness
    private final OuttakeTracker tracker = new OuttakeTracker(7);

    // Stability Threshold: If acceleration is less than 500 RPM/s^2, we consider it stable.
    // You can lower this (e.g., 200) for more precision, or raise it (e.g., 800) for faster shots.
    private final double STABILITY_THRESHOLD = RobotConstants.OUTTAKE_STABILITY_TOLERANCE;

    public Outtake(RobotHardware robot)
    {
        super(robot.outtakeMotor);
        this.robot = robot;
    }

    /**
     * Reads velocity, updates the regression tracker, and returns current RPM.
     * Note: This updates the 'currentAcceleration' variable as a side effect.
     */
    public double getRPM()
    {
        // FIX: Removed the double tpsToRpm call.
        double currentRPM = tpsToRpm(robot.outtakeMotor.getVelocity());

        // Update the tracker with the current time and speed to get true acceleration
        currentAcceleration = tracker.updateAndGetAcceleration(stabilityTimer.seconds(), currentRPM);

        return currentRPM;
    }

    public int getTargetRPM()
    {
        return targetRPM;
    }

    public double getRPMAcceleration()
    {
        return currentAcceleration;
    }

    /**
     * Adds the provided RPM to the current target RPM
     */
    public void varyTargetRPM(int rpmChange)
    {
        int newRpm = targetRPM + rpmChange;

        if (newRpm > 6000)
        {
            robot.telemetry.log().add("RPM capped at 6000.");
            newRpm = 6000;
        }

        targetRPM = newRpm;
    }

    /**
     * Changes the internal target RPM without turning the flywheel on
     */
    public void modifyTargetRPM(int rpm)
    {
        if (rpm > 6000)
        {
            robot.telemetry.log().add("RPM capped at 6000.");
            rpm = 6000;
        }

        targetRPM = rpm;
    }

    public void modifyTargetRPMBasedOnDistance(double distance)
    {
        int newRpm = calculateTargetRPMFromDistance(distance);
        modifyTargetRPM(newRpm);
        robot.telemetry.log().add("Auto-adjusted RPM to " + newRpm);
    }

    /**
     * Enables the flywheel with its currently set target RPM
     */
    public void setRPM()
    {
        setRPM(targetRPM);
    }

    public void setRPM(int rpm)
    {
        if (isResetting())
        {
            return;
        }

        if (rpm > 6000)
        {
            rpm = 6000;
        }

        status = Status.FORWARD_ENABLED;
        targetRPM = rpm;

        // CRITICAL: Reset the tracker buffer when target changes.
        // We don't want history from the previous speed affecting the new acceleration calculation.
        tracker.reset();

        robot.outtakeMotor.setVelocity(rpmToTps(targetRPM));
    }

    /**
     * Sets the target RPM based on a distance regression and turns the flywheel on
     */
    public void setRPMBasedOnDistance(double distance)
    {
        setRPM(calculateTargetRPMFromDistance(distance));
    }

    public boolean isReadyToLaunch()
    {
        // 1. Get the current RPM (This also internally updates currentAcceleration)
        double actualRPM = getRPM();

        // 3. Check Stability
        // We are stable if the acceleration (slope) is flat (near zero)
        boolean rpm_is_stable = Math.abs(currentAcceleration) < STABILITY_THRESHOLD;

        // 4. Check Tolerance
        boolean rpm_within_tolerance = Math.abs(actualRPM - targetRPM) < RobotConstants.OUTTAKE_RPM_TOLERANCE;

        return rpm_is_stable && rpm_within_tolerance && !isResetting();
    }

    // tps = Ticks per Second; rpm = Revolutions per Minute
    private double tpsToRpm(double ticks)
    {
        return (ticks * 60.0) / RobotConstants.OUTTAKE_PPR;
    }

    private double rpmToTps(double rpm)
    {
        return (rpm * RobotConstants.OUTTAKE_PPR) / 60.0;
    }

    private int calculateTargetRPMFromDistance(double distance)
    {
        return Math.toIntExact(Math.round(93.48178 * Math.pow(distance, 3) - 807.53481 * Math.pow(distance, 2) + 2786.50082 * distance + 450.60475));
    }
}