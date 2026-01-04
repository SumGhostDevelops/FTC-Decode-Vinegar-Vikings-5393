package org.firstinspires.ftc.teamcode.util.motors;

import java.util.ArrayList;

public class Tracker
{

    private static class DataPoint {
        double time;
        double rpm;

        public DataPoint(double time, double rpm) {
            this.time = time;
            this.rpm = rpm;
        }
    }

    private final int capacity;
    private final ArrayList<DataPoint> buffer;

    // Time control variables
    private double lastAddTimestamp = 0;
    private double lastCalculatedAccel = 0;

    // 25ms is a safe bet. Most motors update velocity every 10ms-20ms.
    // By setting this slightly higher than the motor update rate,
    // we ensure we never capture the same "stale" frame twice.
    private final double MIN_UPDATE_INTERVAL = 0.025; // Seconds

    public Tracker(int bufferSize) {
        this.capacity = bufferSize;
        this.buffer = new ArrayList<>();
    }

    /**
     * Updates the buffer only if enough time has passed, then returns the slope.
     * If called too quickly, it returns the previously calculated slope.
     */
    public double updateAndGetAcceleration(double currentTimeSeconds, double currentRPM) {

        // 1. CHECK: Has enough time passed since the last sample?
        if (currentTimeSeconds - lastAddTimestamp < MIN_UPDATE_INTERVAL) {
            // Not enough time has passed. The sensor data might be stale.
            // Return the last known valid acceleration.
            return lastCalculatedAccel;
        }

        // 2. Update Timestamp
        lastAddTimestamp = currentTimeSeconds;

        // 3. Add new data point
        buffer.add(new DataPoint(currentTimeSeconds, currentRPM));

        // 4. Manage Buffer Size
        if (buffer.size() > capacity) {
            buffer.remove(0);
        }

        // 5. Calculate Slope (Linear Regression)
        if (buffer.size() < 2) {
            return 0.0;
        }

        double n = buffer.size();
        double sumX = 0;
        double sumY = 0;
        double sumXY = 0;
        double sumXX = 0;

        double startTime = buffer.get(0).time;

        for (DataPoint point : buffer) {
            double x = point.time - startTime;
            double y = point.rpm;

            sumX += x;
            sumY += y;
            sumXY += (x * y);
            sumXX += (x * x);
        }

        double denominator = (n * sumXX) - (sumX * sumX);

        if (Math.abs(denominator) < 1e-9) {
            lastCalculatedAccel = 0.0;
        } else {
            lastCalculatedAccel = ((n * sumXY) - (sumX * sumY)) / denominator;
        }

        return lastCalculatedAccel;
    }

    public void reset() {
        buffer.clear();
        lastAddTimestamp = 0;
        lastCalculatedAccel = 0;
    }
}