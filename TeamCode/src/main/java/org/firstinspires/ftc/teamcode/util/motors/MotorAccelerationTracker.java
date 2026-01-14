package org.firstinspires.ftc.teamcode.util.motors;

import org.firstinspires.ftc.teamcode.util.math.DerivativeCalculator;

/**
 * Tracks motor RPM over time and calculates acceleration using linear regression.
 * This is a specialized wrapper around {@link DerivativeCalculator} for motor use cases.
 *
 * @see DerivativeCalculator for the underlying generic derivative calculation
 */
public class MotorAccelerationTracker
{
    private final DerivativeCalculator derivativeCalculator;

    /**
     * Creates a motor acceleration tracker with the specified buffer size.
     * @param bufferSize The number of RPM samples to keep in the buffer
     */
    public MotorAccelerationTracker(int bufferSize)
    {
        this.derivativeCalculator = new DerivativeCalculator(bufferSize);
    }

    /**
     * Adds a new RPM sample and calculates the acceleration.
     * @param currentTimeSeconds The current time in seconds
     * @param currentRPM The current motor RPM
     * @return The calculated acceleration (RPM per second)
     */
    public double updateAndGetAcceleration(double currentTimeSeconds, double currentRPM)
    {
        return derivativeCalculator.updateAndGetDerivative(currentTimeSeconds, currentRPM);
    }

    /**
     * @return The last calculated acceleration (RPM per second)
     */
    public double getAcceleration()
    {
        return derivativeCalculator.getDerivative();
    }

    /**
     * @return The last calculated jerk (rate of change of acceleration, RPM per second²)
     */
    public double getJerk()
    {
        return derivativeCalculator.getSecondDerivative();
    }

    public void reset()
    {
        derivativeCalculator.reset();
    }
}