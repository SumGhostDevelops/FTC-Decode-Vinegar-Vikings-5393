package org.firstinspires.ftc.teamcode.util.motors;

public class MotorAccelerationTracker
{
    private final int capacity;
    private final double[] times;
    private final double[] rpms;

    private int size = 0;
    private int head = 0; // Points to the oldest element (start of window)

    private double lastAddTimestamp = 0;
    private double lastCalculatedAccel = 0;

    // 25ms is good. If you run at 50Hz control loop, this guarantees 1 sample per loop.
    // If your loop is 20ms, you might miss every other sample.
    // Consider lowering to 0.015 or 0.020 if your loops are fast.
    private final double MIN_UPDATE_INTERVAL = 0.015;

    public MotorAccelerationTracker(int bufferSize)
    {
        this.capacity = bufferSize;
        this.times = new double[bufferSize];
        this.rpms = new double[bufferSize];
    }

    public double updateAndGetAcceleration(double currentTimeSeconds, double currentRPM)
    {
        // 1. CHECK: Has enough time passed?
        // Note: Added check for size == 0 to ensure we ALWAYS accept the very first point
        if (size > 0 && (currentTimeSeconds - lastAddTimestamp < MIN_UPDATE_INTERVAL))
        {
            return lastCalculatedAccel;
        }

        lastAddTimestamp = currentTimeSeconds;

        // 2. Add to Circular Buffer
        if (size < capacity)
        {
            int index = (head + size) % capacity;
            times[index] = currentTimeSeconds;
            rpms[index] = currentRPM;
            size++;
        }
        else
        {
            times[head] = currentTimeSeconds;
            rpms[head] = currentRPM;
            head = (head + 1) % capacity;
        }

        // 3. Need at least 3 points for a reliable regression, 2 is mathematically possible but noisy
        if (size < 3)
        {
            return 0.0;
        }

        // 4. Calculate Linear Regression (OLS)
        // O(N) operation - perfectly fine for N < 50
        double n = size;
        double sumX = 0;
        double sumY = 0;
        double sumXY = 0;
        double sumXX = 0;

        // "Normalizing" time to the start of the window preserves floating point precision
        double startTime = times[head];

        for (int i = 0; i < size; i++)
        {
            int index = (head + i) % capacity;
            double x = times[index] - startTime; // Time relative to window start
            double y = rpms[index];              // Velocity

            sumX += x;
            sumY += y;
            sumXY += (x * y);
            sumXX += (x * x);
        }

        double denominator = (n * sumXX) - (sumX * sumX);

        // Prevent division by zero (e.g., if all timestamps are identical, which shouldn't happen)
        if (Math.abs(denominator) < 1e-9)
        {
            // Keep previous value if math fails
            return lastCalculatedAccel;
        }

        // Slope formula
        lastCalculatedAccel = ((n * sumXY) - (sumX * sumY)) / denominator;

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