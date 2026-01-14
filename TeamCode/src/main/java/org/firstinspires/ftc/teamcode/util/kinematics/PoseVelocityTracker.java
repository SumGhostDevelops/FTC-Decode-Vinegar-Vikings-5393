package org.firstinspires.ftc.teamcode.util.kinematics;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

/**
 * Tracks Pose2d data points over time and computes velocity and acceleration
 * using linear regression (OLS). This is analogous to MotorAccelerationTracker
 * but operates on Pose2d instead of a single scalar.
 *
 * Velocities are computed as:
 * - vX: velocity in X direction (inches/second)
 * - vY: velocity in Y direction (inches/second)
 * - vHeading: angular velocity (radians/second)
 *
 * Accelerations are computed as the derivative of velocities.
 */
public class PoseVelocityTracker
{
    private static final DistanceUnit INTERNAL_DIST_UNIT = DistanceUnit.INCH;
    private static final AngleUnit INTERNAL_ANGLE_UNIT = AngleUnit.RADIANS;

    private final int capacity;
    private final double[] times;
    private final double[] xPositions;
    private final double[] yPositions;
    private final double[] headings;

    private int size = 0;
    private int head = 0; // Points to the oldest element (start of window)

    private double lastAddTimestamp = 0;

    // Cached velocity values (first derivative)
    private double lastVX = 0;
    private double lastVY = 0;
    private double lastVHeading = 0;

    // Cached acceleration values (second derivative)
    private double lastAX = 0;
    private double lastAY = 0;
    private double lastAHeading = 0;

    // Minimum time interval between samples (seconds)
    private final double MIN_UPDATE_INTERVAL = 0.015;

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
        // Check if enough time has passed (except for first sample)
        if (size > 0 && (currentTimeSeconds - lastAddTimestamp < MIN_UPDATE_INTERVAL))
        {
            return;
        }

        lastAddTimestamp = currentTimeSeconds;

        // Convert pose to internal units
        Pose2d normalizedPose = pose
                .toDistanceUnit(INTERNAL_DIST_UNIT)
                .toAngleUnit(INTERNAL_ANGLE_UNIT);

        double x = normalizedPose.coord.x.magnitude;
        double y = normalizedPose.coord.y.magnitude;
        double heading = normalizedPose.heading.measure;

        // Handle heading wraparound - unwrap heading relative to previous sample
        if (size > 0)
        {
            int lastIndex = (head + size - 1) % capacity;
            double lastHeading = headings[lastIndex];
            heading = unwrapAngle(lastHeading, heading);
        }

        // Add to circular buffer
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

        // Need at least 3 points for reliable regression
        if (size < 3)
        {
            return;
        }

        // Calculate velocities (first derivatives) using linear regression
        double[] regressionResults = calculateLinearRegression(times, xPositions);
        lastVX = regressionResults[0]; // slope is velocity

        regressionResults = calculateLinearRegression(times, yPositions);
        lastVY = regressionResults[0];

        regressionResults = calculateLinearRegression(times, headings);
        lastVHeading = regressionResults[0];

        // For acceleration, we need to track velocity over time
        // Use quadratic regression to get acceleration directly
        if (size >= 5) // Need more points for reliable second derivative
        {
            lastAX = calculateQuadraticAcceleration(times, xPositions);
            lastAY = calculateQuadraticAcceleration(times, yPositions);
            lastAHeading = calculateQuadraticAcceleration(times, headings);
        }
    }

    /**
     * Unwraps an angle to avoid discontinuities at ±π.
     * Returns the unwrapped angle that is closest to the reference.
     */
    private double unwrapAngle(double reference, double angle)
    {
        double diff = angle - reference;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        return reference + diff;
    }

    /**
     * Calculates linear regression (OLS) and returns [slope, intercept].
     */
    private double[] calculateLinearRegression(double[] timeData, double[] valueData)
    {
        double n = size;
        double sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;

        double startTime = timeData[head];

        for (int i = 0; i < size; i++)
        {
            int index = (head + i) % capacity;
            double x = timeData[index] - startTime;
            double y = valueData[index];

            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumXX += x * x;
        }

        double denominator = (n * sumXX) - (sumX * sumX);
        if (Math.abs(denominator) < 1e-9)
        {
            return new double[]{0, sumY / n}; // Return average as intercept if degenerate
        }

        double slope = ((n * sumXY) - (sumX * sumY)) / denominator;
        double intercept = (sumY - slope * sumX) / n;

        return new double[]{slope, intercept};
    }

    /**
     * Calculates acceleration using quadratic regression.
     * Fits y = a*t² + b*t + c, acceleration = 2*a
     */
    private double calculateQuadraticAcceleration(double[] timeData, double[] valueData)
    {
        // Use least squares for quadratic fit
        double n = size;
        double sumT = 0, sumT2 = 0, sumT3 = 0, sumT4 = 0;
        double sumY = 0, sumTY = 0, sumT2Y = 0;

        double startTime = timeData[head];

        for (int i = 0; i < size; i++)
        {
            int index = (head + i) % capacity;
            double t = timeData[index] - startTime;
            double y = valueData[index];

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

        // Solve the normal equations for quadratic regression
        // Using Cramer's rule for 3x3 system
        double[][] A = {
                {sumT4, sumT3, sumT2},
                {sumT3, sumT2, sumT},
                {sumT2, sumT, n}
        };
        double[] B = {sumT2Y, sumTY, sumY};

        double detA = determinant3x3(A);
        if (Math.abs(detA) < 1e-12)
        {
            return 0;
        }

        // Replace first column with B to get coefficient 'a'
        double[][] A_a = {
                {B[0], sumT3, sumT2},
                {B[1], sumT2, sumT},
                {B[2], sumT, n}
        };

        double a = determinant3x3(A_a) / detA;

        // Acceleration is 2*a (second derivative of at² + bt + c)
        return 2 * a;
    }

    private double determinant3x3(double[][] m)
    {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }

    /**
     * Predicts the future pose after a given time delta.
     * Uses current velocity and acceleration for prediction.
     *
     * @param seconds Time in the future to predict (seconds)
     * @param currentPose The current pose to extrapolate from
     * @return The predicted Pose2d
     */
    public Pose2d getFuturePose(double seconds, Pose2d currentPose)
    {
        Pose2d normalizedPose = currentPose
                .toDistanceUnit(INTERNAL_DIST_UNIT)
                .toAngleUnit(INTERNAL_ANGLE_UNIT);

        double currentX = normalizedPose.coord.x.magnitude;
        double currentY = normalizedPose.coord.y.magnitude;
        double currentHeading = normalizedPose.heading.measure;

        // Kinematic equation: x = x0 + v*t + 0.5*a*t²
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

    // Getters for velocities (in internal units: inches/sec, radians/sec)

    public double getVelocityX()
    {
        return lastVX;
    }

    public double getVelocityY()
    {
        return lastVY;
    }

    public double getAngularVelocity()
    {
        return lastVHeading;
    }

    /**
     * @return The linear speed (magnitude of velocity vector) in inches/second
     */
    public double getSpeed()
    {
        return Math.sqrt(lastVX * lastVX + lastVY * lastVY);
    }

    // Getters for accelerations (in internal units: inches/sec², radians/sec²)

    public double getAccelerationX()
    {
        return lastAX;
    }

    public double getAccelerationY()
    {
        return lastAY;
    }

    public double getAngularAcceleration()
    {
        return lastAHeading;
    }

    /**
     * @return The linear acceleration magnitude in inches/second²
     */
    public double getAccelerationMagnitude()
    {
        return Math.sqrt(lastAX * lastAX + lastAY * lastAY);
    }

    public void reset()
    {
        size = 0;
        head = 0;
        lastAddTimestamp = 0;
        lastVX = 0;
        lastVY = 0;
        lastVHeading = 0;
        lastAX = 0;
        lastAY = 0;
        lastAHeading = 0;
    }

    public int getSampleCount()
    {
        return size;
    }
}

