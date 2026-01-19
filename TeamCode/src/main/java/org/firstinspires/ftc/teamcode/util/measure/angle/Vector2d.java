package org.firstinspires.ftc.teamcode.util.measure.angle;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.CoordinateSystem; // Import your Enum
import org.firstinspires.ftc.teamcode.util.measure.coordinate.CoordinateSystem.Direction;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

/**
 * Describes the {@link Vector2d#x} and {@link Vector2d#y} components which form a {@link Vector2d}.
 * Enforces the {@link DistanceUnit} of {@link Vector2d#x} and {@link AngleUnit#RADIANS} of the {@link Vector2d} direction by default.
 * <p>
 * Note: Unlike FieldCoordinate, Vector2d represents a displacement or quantity.
 * Coordinate System conversions only affect rotation/orientation, not origin translation.
 */
public class Vector2d
{
    public final Distance x;
    public final Distance y;
    public final CoordinateSystem coordSys; // Added Coordinate System

    public final DistanceUnit distUnit;
    public final AngleUnit angUnit = AngleUnit.RADIANS;

    // --- Constructors ---

    public Vector2d(Distance x, Distance y, CoordinateSystem coordSys)
    {
        this.coordSys = coordSys;

        // x and y will always be the same set DistanceUnit
        this.x = x;
        this.y = y.toUnit(x.unit);
        this.distUnit = x.unit;
    }

    // --- Standard Getters ---

    /**
     * @return The magnitude of the vector
     */
    public Distance getLength()
    {
        return new Distance(Math.hypot(x.magnitude, y.magnitude), x.unit);
    }

    /**
     * @return The angle of the vector relative to its Coordinate System's positive X-axis
     */
    public Angle getDirection()
    {
        return new Angle(Math.atan2(y.magnitude, x.magnitude), angUnit);
    }

    // --- Unit Converters ---

    public Vector2d toDistanceUnit(DistanceUnit distanceUnit)
    {
        if (isDistanceUnit(distanceUnit)) return this;
        return new Vector2d(x.toUnit(distanceUnit), y.toUnit(distanceUnit), coordSys);
    }

    // --- Coordinate System Conversion (The Core Logic) ---

    /**
     * Converts this Vector to a different Coordinate System.
     * <p>
     * NOTE: This performs a ROTATION only. Vectors have magnitude and direction, not position.
     * The origin offset of the CoordinateSystem is ignored.
     */
    public Vector2d toCoordinateSystem(CoordinateSystem targetSys)
    {
        if (this.coordSys == targetSys) return this;

        // 1. Convert Local (This) -> Universal Vector
        // V_global = (x * AxisX_global) + (y * AxisY_global)
        double[] xBasis = getUniversalBasis(this.coordSys.positiveX);
        double[] yBasis = getUniversalBasis(this.coordSys.positiveY);

        double xMag = this.x.magnitude; // Use raw magnitude in current unit
        double yMag = this.y.magnitude;

        double globalX = (xMag * xBasis[0]) + (yMag * yBasis[0]);
        double globalY = (xMag * xBasis[1]) + (yMag * yBasis[1]);

        // 2. Convert Universal -> Target Vector
        // Project Global onto Target Axes (Dot Product)
        // V_target = (V_global . TargetAxisX, V_global . TargetAxisY)
        double[] targetXBasis = getUniversalBasis(targetSys.positiveX);
        double[] targetYBasis = getUniversalBasis(targetSys.positiveY);

        double newXMag = (globalX * targetXBasis[0]) + (globalY * targetXBasis[1]);
        double newYMag = (globalX * targetYBasis[0]) + (globalY * targetYBasis[1]);

        return new Vector2d(
                new Distance(newXMag, x.unit),
                new Distance(newYMag, y.unit),
                targetSys
        );
    }

    // --- Math Operations ---

    public Vector2d inverse()
    {
        return new Vector2d(x.multiply(-1), y.multiply(-1), coordSys);
    }

    /**
     * Calculates this + b.
     * Automatically converts 'b' to this CoordinateSystem before adding.
     */
    public Vector2d plus(Vector2d b)
    {
        // Ensure systems match
        Vector2d convertedB = b.toCoordinateSystem(this.coordSys);

        Distance xResult = this.x.plus(convertedB.x);
        Distance yResult = this.y.plus(convertedB.y);

        return new Vector2d(xResult, yResult, this.coordSys);
    }

    /**
     * Calculates this - b.
     * Automatically converts 'b' to this CoordinateSystem before subtracting.
     */
    public Vector2d minus(Vector2d b)
    {
        return plus(b.inverse());
    }

    /**
     * Helper to get Universal Basis Vectors (Matches CoordinateSystem logic).
     * Universal Frame: X=Audience, Y=Blue
     */
    private double[] getUniversalBasis(Direction dir)
    {
        switch (dir)
        {
            case AUDIENCE:  return new double[]{ 1.0,  0.0};
            case BACKSTAGE: return new double[]{-1.0,  0.0};
            case BLUE:      return new double[]{ 0.0,  1.0};
            case RED:       return new double[]{ 0.0, -1.0};
            default:        return new double[]{ 0.0,  0.0};
        }
    }

    public DistanceUnit getDistanceUnit()
    {
        return distUnit;
    }

    public boolean isDistanceUnit(DistanceUnit unit)
    {
        return distUnit.equals(unit);
    }

    @Override
    public boolean equals(Object obj)
    {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;

        Vector2d other = (Vector2d) obj;

        // Convert other to THIS system for apples-to-apples comparison
        Vector2d convertedOther = other.toCoordinateSystem(this.coordSys).toDistanceUnit(x.unit);

        // Use Distance.equals for fuzzy comparison
        return this.x.equals(convertedOther.x) && this.y.equals(convertedOther.y);
    }

    @Override
    public String toString()
    {
        return String.format("%s; %s (%s)", getLength(), getDirection(), coordSys.name());
    }
}