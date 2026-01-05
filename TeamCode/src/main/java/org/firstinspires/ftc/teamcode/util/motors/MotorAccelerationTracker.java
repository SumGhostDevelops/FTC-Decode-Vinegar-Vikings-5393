package org.firstinspires.ftc.teamcode.util.motors;

public class MotorAccelerationTracker
{
    private final int capacity;

    // Simple arrays instead of ArrayList
    private final double[] times;
    private final double[] rpms;
    private int size = 0;
    private int head = 0;  // Points to oldest element

    // Time control variables
    private double lastAddTimestamp = 0;
    private double lastCalculatedAccel = 0;

    // 25ms is a safe bet. Most motors update velocity every 10ms-20ms.
    private final double MIN_UPDATE_INTERVAL = 0.025; // Seconds

    public MotorAccelerationTracker(int bufferSize)
    {
        this.capacity = bufferSize;
        this.times = new double[bufferSize];
        this.rpms = new double[bufferSize];
    }

    /**
     * Updates the buffer only if enough time has passed, then returns the slope.
     * If called too quickly, it returns the previously calculated slope.
     */
    public double updateAndGetAcceleration(double currentTimeSeconds, double currentRPM)
    {

        // 1. CHECK: Has enough time passed since the last sample?
        if (currentTimeSeconds - lastAddTimestamp < MIN_UPDATE_INTERVAL)
        {
            return lastCalculatedAccel;
        }

        // 2. Update Timestamp
        lastAddTimestamp = currentTimeSeconds;

        // 3. Add new data point to circular buffer
        if (size < capacity)
        {
            // Buffer not full yet, just append
            int index = (head + size) % capacity;
            times[index] = currentTimeSeconds;
            rpms[index] = currentRPM;
            size++;
        } else
        {
            // Buffer full, overwrite oldest
            times[head] = currentTimeSeconds;
            rpms[head] = currentRPM;
            head = (head + 1) % capacity;
        }

        // 4. Calculate Slope (Linear Regression)
        if (size < 2)
        {
            return 0.0;
        }

        double n = size;
        double sumX = 0;
        double sumY = 0;
        double sumXY = 0;
        double sumXX = 0;

        double startTime = times[head];

        for (int i = 0; i < size; i++)
        {
            int index = (head + i) % capacity;
            double x = times[index] - startTime;
            double y = rpms[index];

            sumX += x;
            sumY += y;
            sumXY += (x * y);
            sumXX += (x * x);
        }

        double denominator = (n * sumXX) - (sumX * sumX);

        if (Math.abs(denominator) < 1e-9)
        {
            lastCalculatedAccel = 0.0;
        } else
        {
            lastCalculatedAccel = ((n * sumXY) - (sumX * sumY)) / denominator;
        }

        return lastCalculatedAccel;
    }

    public void reset()
    {
        size = 0;
        head = 0;
        lastAddTimestamp = 0;
        lastCalculatedAccel = 0;
    }
}