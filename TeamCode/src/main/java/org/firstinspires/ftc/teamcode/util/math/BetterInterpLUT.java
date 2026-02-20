package org.firstinspires.ftc.teamcode.util.math;

import com.seattlesolvers.solverslib.util.InterpLUT;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * An improved {@link InterpLUT}
 * @see Builder
 */
public class BetterInterpLUT extends InterpLUT
{

    /**
     * Private constructor used by the Builder.
     * It populates the parent class immediately, ensuring this object is valid from the moment it exists.
     */
    private BetterInterpLUT(List<Point> sortedPoints)
    {
        super(); // Initialize parent

        // 1. Populate the parent class using the sorted data
        for (Point p : sortedPoints)
        {
            super.add(p.x, p.y);
        }

        // 3. Finalize the LUT immediately.
        // Since the constructor is private, we know this is safe to do here.
        super.createLUT();
    }

    /**
     * Helper to start the Builder chain comfortably.
     * Usage: BetterInterpLUT.builder().add(...).build();
     */
    public static Builder builder()
    {
        return new Builder();
    }

    // ==================================================================================
    // Safety Overrides (Immutability)
    // ==================================================================================

    /**
     * Throws an error to prevent modifying the LUT after it has been built.
     */
    @Override
    public InterpLUT add(double input, double output)
    {
        throw new UnsupportedOperationException("BetterInterpLUT is immutable. Please use the Builder to add points.");
    }

    /**
     * No-op or Error. Since we call this in the constructor, calling it again is redundant but harmless.
     * We block it to maintain the strict "Immutable" contract.
     */
    @Override
    public InterpLUT createLUT()
    {
        throw new UnsupportedOperationException("LUT is already created and immutable.");
    }

    // ==================================================================================
    // The Builder Class
    // ==================================================================================

    public static class Builder
    {
        private final List<Point> points = new ArrayList<>();

        /**
         * Convenience method to add a list of inputs and outputs at once.
         */
        public Builder add(List<Double> inputs, List<Double> outputs)
        {
            if (inputs.size() != outputs.size())
            {
                throw new IllegalArgumentException("Input and Output lists must be the same size.");
            }
            for (int i = 0; i < inputs.size(); i++)
            {
                add(inputs.get(i), outputs.get(i));
            }
            return this;
        }

        public Builder add(double input, double output)
        {
            points.add(new Point(input, output));
            return this;
        }

        public BetterInterpLUT build()
        {
            if (points.size() < 2)
            {
                throw new IllegalStateException("At least two control points are required.");
            }

            // Sort logic: Crucial for Spline interpolation
            points.sort(Comparator.comparingDouble(p -> p.x));

            return new BetterInterpLUT(points);
        }
    }

    /**
     * Simple internal data holder
     */
    private static class Point
    {
        final double x, y;

        Point(double x, double y)
        {
            this.x = x;
            this.y = y;
        }
    }
}