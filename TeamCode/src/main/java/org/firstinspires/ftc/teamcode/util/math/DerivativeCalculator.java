package org.firstinspires.ftc.teamcode.util.math;

/**
 * A generic derivative calculator that uses linear regression (OLS) to compute
 * the first derivative of a time series of scalar values.
 *
 * This is a generalized version of what was previously specific to motor RPM tracking.
 * Can be used for any scalar quantity where you need to track rate of change.
 *
 * Usage examples:
 * - Motor acceleration (derivative of RPM)
 * - Position velocity (derivative of position)
 * - Any rate-of-change calculation
 */
public class DerivativeCalculator
{
    private final int capacity;
    private final double[] times;
    private final double[] values;

    private int size = 0;
    private int head = 0; // Points to the oldest element (start of window)

    private double lastAddTimestamp = 0;
    private double lastCalculatedDerivative = 0;
    private double lastCalculatedSecondDerivative = 0;

    // Minimum time interval between samples (seconds)
    private final double minUpdateInterval;

    /**
     * Creates a derivative calculator with the specified buffer size and default 15ms update interval.
     * @param bufferSize The number of samples to keep in the buffer
     */
    public DerivativeCalculator(int bufferSize)
    {
        this(bufferSize, 0.015);
    }

    /**
     * Creates a derivative calculator with the specified buffer size and update interval.
     * @param bufferSize The number of samples to keep in the buffer
     * @param minUpdateIntervalSeconds Minimum time between accepting new samples (seconds)
     */
    public DerivativeCalculator(int bufferSize, double minUpdateIntervalSeconds)
    {
        this.capacity = bufferSize;
        this.times = new double[bufferSize];
        this.values = new double[bufferSize];
        this.minUpdateInterval = minUpdateIntervalSeconds;
    }

    /**
     * Adds a new sample and calculates the derivative (rate of change).
     * @param currentTimeSeconds The current time in seconds
     * @param value The current value of the quantity being tracked
     * @return The calculated first derivative (rate of change per second)
     */
    public double updateAndGetDerivative(double currentTimeSeconds, double value)
    {
        // Check if enough time has passed (except for first sample)
        if (size > 0 && (currentTimeSeconds - lastAddTimestamp < minUpdateInterval))
        {
            return lastCalculatedDerivative;
        }

        lastAddTimestamp = currentTimeSeconds;

        // Add to circular buffer
        if (size < capacity)
        {
            int index = (head + size) % capacity;
            times[index] = currentTimeSeconds;
            values[index] = value;
            size++;
        }
        else
        {
            times[head] = currentTimeSeconds;
            values[head] = value;
            head = (head + 1) % capacity;
        }

        // Need at least 3 points for reliable regression
        if (size < 3)
        {
            return 0.0;
        }

        // Calculate first derivative using linear regression (OLS)
        lastCalculatedDerivative = calculateLinearSlope();

        // Calculate second derivative if we have enough points
        if (size >= 5)
        {
            lastCalculatedSecondDerivative = calculateQuadraticSecondDerivative();
        }

        return lastCalculatedDerivative;
    }

    /**
     * Calculates the slope using linear regression (first derivative).
     */
    private double calculateLinearSlope()
    {
        double n = size;
        double sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;

        double startTime = times[head];

        for (int i = 0; i < size; i++)
        {
            int index = (head + i) % capacity;
            double x = times[index] - startTime;
            double y = values[index];

            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumXX += x * x;
        }

        double denominator = (n * sumXX) - (sumX * sumX);

        if (Math.abs(denominator) < 1e-9)
        {
            return lastCalculatedDerivative;
        }

        return ((n * sumXY) - (sumX * sumY)) / denominator;
    }

    /**
     * Calculates the second derivative using quadratic regression.
     * Fits y = a*t² + b*t + c, second derivative = 2*a
     */
    private double calculateQuadraticSecondDerivative()
    {
        double n = size;
        double sumT = 0, sumT2 = 0, sumT3 = 0, sumT4 = 0;
        double sumY = 0, sumTY = 0, sumT2Y = 0;

        double startTime = times[head];

        for (int i = 0; i < size; i++)
        {
            int index = (head + i) % capacity;
            double t = times[index] - startTime;
            double y = values[index];

            double t2 = t * t;
            double t3 = t2 * t;
            double t4 = t2 * t2;

            sumT += t;
            sumT2 += t2;
            sumT3 += t3;
            sumT4 += t4;
            sumY += y;
            sumTY += t * y;
            sumT2Y += t2 * y;
        }

        // Solve the normal equations for quadratic regression using Cramer's rule
        double[][] A = {
                {sumT4, sumT3, sumT2},
                {sumT3, sumT2, sumT},
                {sumT2, sumT, n}
        };

        double detA = determinant3x3(A);
        if (Math.abs(detA) < 1e-12)
        {
            return 0;
        }

        double[][] A_a = {
                {sumT2Y, sumT3, sumT2},
                {sumTY, sumT2, sumT},
                {sumY, sumT, n}
        };

        double a = determinant3x3(A_a) / detA;

        return 2 * a;
    }

    private double determinant3x3(double[][] m)
    {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }

    /**
     * @return The last calculated first derivative (rate of change per second)
     */
    public double getDerivative()
    {
        return lastCalculatedDerivative;
    }

    /**
     * @return The last calculated second derivative (rate of change of rate of change per second²)
     */
    public double getSecondDerivative()
    {
        return lastCalculatedSecondDerivative;
    }

    /**
     * @return The number of samples currently in the buffer
     */
    public int getSampleCount()
    {
        return size;
    }

    public void reset()
    {
        size = 0;
        head = 0;
        lastAddTimestamp = 0;
        lastCalculatedDerivative = 0;
        lastCalculatedSecondDerivative = 0;
    }
}

