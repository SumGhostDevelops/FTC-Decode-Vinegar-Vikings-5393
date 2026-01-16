package org.firstinspires.ftc.teamcode.util.kinematics;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

/**
 * Tracks Pose2d data points over time and computes velocity and acceleration.
 *
 * IMPROVEMENTS:
 * - Uses a time-bounded window (e.g., 100ms) for regression instead of full buffer capacity.
 * This dramatically reduces lag during starts/stops.
 * - Uses Low-Pass Filtered finite differences for acceleration to capture sudden impulses.
 */
public class PoseVelocityTracker
{
    private static final DistanceUnit INTERNAL_DIST_UNIT = DistanceUnit.INCH;
    private static final AngleUnit INTERNAL_ANGLE_UNIT = AngleUnit.RADIANS;

    // Time horizon for velocity calculation.
    // 0.1s (100ms) provides a good balance between noise rejection and fast reaction.
    // If the robot stops, the old motion data "falls off" this window in 0.1s.
    private static final double WINDOW_TIME_HORIZON = 0.1;

    // Minimum time interval between samples (seconds)
    // Lowered to 5ms to allow for high-frequency control loops.
    private final double MIN_UPDATE_INTERVAL = 0.005;

    // Smoothing factor for Acceleration (0.0 = no new data, 1.0 = no smoothing)
    // Acceleration is naturally noisy, so we smooth it slightly.
    private final double ACCEL_SMOOTHING = 0.6;

    private final int capacity;
    private final double[] times;
    private final double[] xPositions;
    private final double[] yPositions;
    private final double[] headings;

    private int size = 0;
    private int head = 0; // Points to the oldest element (start of window)

    private double lastAddTimestamp = 0;

    // Cached velocity values
    private double lastVX = 0;
    private double lastVY = 0;
    private double lastVHeading = 0;

    // Cached acceleration values
    private double lastAX = 0;
    private double lastAY = 0;
    private double lastAHeading = 0;

    public PoseVelocityTracker(int bufferSize)
    {
        this.capacity = bufferSize;
        this.times = new double[bufferSize];
        this.xPositions = new double[bufferSize];
        this.yPositions = new double[bufferSize];
        this.headings = new double[bufferSize];
    }

    /**
     * Adds a new pose sample and recalculates velocities and accelerations.
     *
     * @param currentTimeSeconds The current time in seconds
     * @param pose               The current Pose2d of the robot
     */
    public void update(double currentTimeSeconds, Pose2d pose)
    {
        // 1. Frequency Cap
        if (size > 0 && (currentTimeSeconds - lastAddTimestamp < MIN_UPDATE_INTERVAL))
        {
            return;
        }

        double dt = currentTimeSeconds - lastAddTimestamp; // Delta time since last accepted update
        lastAddTimestamp = currentTimeSeconds;

        // 2. Data Normalization
        Pose2d normalizedPose = pose
                .toDistanceUnit(INTERNAL_DIST_UNIT)
                .toAngleUnit(INTERNAL_ANGLE_UNIT);

        double x = normalizedPose.coord.x.magnitude;
        double y = normalizedPose.coord.y.magnitude;
        double heading = normalizedPose.heading.measure;

        // 3. Handle Heading Wraparound
        if (size > 0)
        {
            int lastIndex = (head + size - 1) % capacity;
            double lastHeading = headings[lastIndex];
            heading = unwrapAngle(lastHeading, heading);
        }

        // 4. Add to Circular Buffer
        if (size < capacity)
        {
            int index = (head + size) % capacity;
            times[index] = currentTimeSeconds;
            xPositions[index] = x;
            yPositions[index] = y;
            headings[index] = heading;
            size++;
        }
        else
        {
            times[head] = currentTimeSeconds;
            xPositions[head] = x;
            yPositions[head] = y;
            headings[head] = heading;
            head = (head + 1) % capacity;
        }

        if (size < 2) return;

        // 5. Determine Analysis Window
        // We only look back 'WINDOW_TIME_HORIZON' seconds to determine current velocity.
        // This ensures old data doesn't drag down the average.
        int windowCount = 0;
        for (int i = 0; i < size; i++)
        {
            int index = (head + size - 1 - i) % capacity; // Iterate backwards
            double sampleTime = times[index];
            windowCount++;
            if (currentTimeSeconds - sampleTime > WINDOW_TIME_HORIZON)
            {
                break;
            }
        }

        // Ensure we have at least 2 points for a line, but prefer at least 3 for regression
        windowCount = Math.max(2, windowCount);

        // 6. Calculate Velocity (Linear Regression on recent window)
        // Store previous velocities to calculate acceleration later
        double prevVX = lastVX;
        double prevVY = lastVY;
        double prevVHeading = lastVHeading;

        double[] vXResults = calculateLinearRegression(times, xPositions, windowCount);
        double[] vYResults = calculateLinearRegression(times, yPositions, windowCount);
        double[] vHResults = calculateLinearRegression(times, headings, windowCount);

        lastVX = vXResults[0];
        lastVY = vYResults[0];
        lastVHeading = vHResults[0];

        // 7. Calculate Acceleration (Finite Difference + Low Pass Filter)
        // Regression for acceleration is too slow/unstable for sudden stops.
        // We use the change in velocity over the change in time.
        if (size > 2 && dt > 0)
        {
            double rawAX = (lastVX - prevVX) / dt;
            double rawAY = (lastVY - prevVY) / dt;
            double rawAH = (lastVHeading - prevVHeading) / dt;

            // Apply Exponential Moving Average (Low Pass Filter)
            lastAX = (ACCEL_SMOOTHING * rawAX) + ((1 - ACCEL_SMOOTHING) * lastAX);
            lastAY = (ACCEL_SMOOTHING * rawAY) + ((1 - ACCEL_SMOOTHING) * lastAY);
            lastAHeading = (ACCEL_SMOOTHING * rawAH) + ((1 - ACCEL_SMOOTHING) * lastAHeading);
        }
    }

    /**
     * Unwraps an angle to avoid discontinuities at ±π.
     */
    private double unwrapAngle(double reference, double angle)
    {
        double diff = angle - reference;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        return reference + diff;
    }

    /**
     * Calculates linear regression (OLS) on the last 'count' elements of the buffer.
     * @return [slope, intercept]
     */
    private double[] calculateLinearRegression(double[] timeData, double[] valueData, int count)
    {
        double n = count;
        double sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;

        // Start index for the window
        // (head + size - count) points to the start of the recent window
        int startIndex = (head + size - count);
        // Fix negative modulo if count is close to capacity (though math handles it, this is safe)
        while(startIndex < 0) startIndex += capacity;
        startIndex %= capacity;

        // Use the oldest time in the WINDOW as t=0 to keep numbers small
        double t0 = timeData[startIndex];

        for (int i = 0; i < count; i++)
        {
            int index = (startIndex + i) % capacity;
            double x = timeData[index] - t0;
            double y = valueData[index];

            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumXX += x * x;
        }

        double denominator = (n * sumXX) - (sumX * sumX);
        if (Math.abs(denominator) < 1e-9)
        {
            // Fallback: If denominator is 0 (all times same), return 0 slope
            return new double[]{0, sumY / n};
        }

        double slope = ((n * sumXY) - (sumX * sumY)) / denominator;
        double intercept = (sumY - slope * sumX) / n;

        return new double[]{slope, intercept};
    }

    /**
     * Predicts the future pose after a given time delta.
     */
    public Pose2d getFuturePose(double seconds, Pose2d currentPose)
    {
        Pose2d normalizedPose = currentPose
                .toDistanceUnit(INTERNAL_DIST_UNIT)
                .toAngleUnit(INTERNAL_ANGLE_UNIT);

        double currentX = normalizedPose.coord.x.magnitude;
        double currentY = normalizedPose.coord.y.magnitude;
        double currentHeading = normalizedPose.heading.measure;

        double t = seconds;
        double t2 = t * t;

        double futureX = currentX + lastVX * t + 0.5 * lastAX * t2;
        double futureY = currentY + lastVY * t + 0.5 * lastAY * t2;
        double futureHeading = currentHeading + lastVHeading * t + 0.5 * lastAHeading * t2;

        return new Pose2d(
                new FieldCoordinate(
                        new Distance(futureX, INTERNAL_DIST_UNIT),
                        new Distance(futureY, INTERNAL_DIST_UNIT),
                        currentPose.coord.coordSys
                ),
                new Angle(futureHeading, INTERNAL_ANGLE_UNIT)
        );
    }

    // --- Getters ---

    public double getVelocityX() { return lastVX; }
    public double getVelocityY() { return lastVY; }
    public double getAngularVelocity() { return lastVHeading; }
    public double getSpeed() { return Math.sqrt(lastVX * lastVX + lastVY * lastVY); }

    public double getAccelerationX() { return lastAX; }
    public double getAccelerationY() { return lastAY; }
    public double getAngularAcceleration() { return lastAHeading; }
    public double getAccelerationMagnitude() { return Math.sqrt(lastAX * lastAX + lastAY * lastAY); }

    public void reset()
    {
        size = 0;
        head = 0;
        lastAddTimestamp = 0;
        lastVX = 0; lastVY = 0; lastVHeading = 0;
        lastAX = 0; lastAY = 0; lastAHeading = 0;
    }

    public int getSampleCount()
    {
        return size;
    }
}