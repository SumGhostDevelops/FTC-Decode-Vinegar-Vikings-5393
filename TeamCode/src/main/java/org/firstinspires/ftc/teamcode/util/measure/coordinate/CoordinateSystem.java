package org.firstinspires.ftc.teamcode.util.measure.coordinate;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

public enum CoordinateSystem
{
    // FTC Standard: 0,0 at field center. X+ points to Audience, Y+ points to Blue.
    DECODE_FTC(Direction.AUDIENCE, Direction.BLUE, Direction.UP,
            new Coordinate(new Distance(0, DistanceUnit.INCH), new Distance(0, DistanceUnit.INCH))),

    // PedroPath: X+ points Blue, Y+ points Backstage. Field center is at (72, 72).
    DECODE_PEDROPATH(Direction.BLUE, Direction.BACKSTAGE, Direction.UP,
            new Coordinate(new Distance(72, DistanceUnit.INCH), new Distance(72, DistanceUnit.INCH))),

    GENERIC(Direction.UP, Direction.UP, Direction.UP, null);

    public enum Direction
    {
        RED, BLUE, AUDIENCE, BACKSTAGE, UP, DOWN
    }

    public final Direction positiveX;
    public final Direction positiveY;
    public final Direction positiveZ;
    public final Coordinate center; // The field center position in THIS coordinate system's own coordinates

    CoordinateSystem(Direction positiveX, Direction positiveY, Direction positiveZ, Coordinate center)
    {
        this.positiveX = positiveX;
        this.positiveY = positiveY;
        this.positiveZ = positiveZ;
        this.center = center;
    }

    /**
     * Converts a Local Coordinate (in THIS system) -> Universal Field Coordinate (FTC Standard)
     * Logic:
     * 1. Calculate offset from field center in local coordinates
     * 2. Rotate the offset vector to universal axes
     * 3. Universal center is (0,0), so the rotated offset IS the universal coordinate
     */
    public Coordinate toUniversal(Distance localX, Distance localY)
    {
        double[] xBasis = getUniversalUnitVector(this.positiveX);
        double[] yBasis = getUniversalUnitVector(this.positiveY);

        // 1. Calculate offset from field center in local coordinates
        Distance offsetX = localX.minus(center.x);
        Distance offsetY = localY.minus(center.y);

        // 2. Rotate offset to universal axes
        // globalX = offsetX * xBasis[0] + offsetY * yBasis[0]
        // globalY = offsetX * xBasis[1] + offsetY * yBasis[1]
        Distance globalX = offsetX.multiply(xBasis[0]).plus(offsetY.multiply(yBasis[0]));
        Distance globalY = offsetX.multiply(xBasis[1]).plus(offsetY.multiply(yBasis[1]));

        return new Coordinate(globalX, globalY);
    }

    /**
     * Converts a Universal Field Coordinate -> Local Coordinate (in THIS system)
     * Logic:
     * 1. Universal coords are already offset from universal center (0,0)
     * 2. Rotate universal offset to local axes (inverse rotation)
     * 3. Add local center to get local coordinates
     */
    public Coordinate fromUniversal(Distance globalX, Distance globalY)
    {
        double[] xBasis = getUniversalUnitVector(this.positiveX);
        double[] yBasis = getUniversalUnitVector(this.positiveY);

        // Inverse rotation: project universal offset onto local axes
        // localOffsetX = globalX * xBasis[0] + globalY * xBasis[1]
        // localOffsetY = globalX * yBasis[0] + globalY * yBasis[1]
        Distance localOffsetX = globalX.multiply(xBasis[0]).plus(globalY.multiply(xBasis[1]));
        Distance localOffsetY = globalX.multiply(yBasis[0]).plus(globalY.multiply(yBasis[1]));

        // Add local center to get local coordinates
        Distance localX = localOffsetX.plus(center.x);
        Distance localY = localOffsetY.plus(center.y);

        return new Coordinate(localX, localY);
    }

    public Coordinate fromUniversal(Coordinate globalCoordinate)
    {
        return fromUniversal(globalCoordinate.x, globalCoordinate.y);
    }

    /**
     * Defines the Universal Basis Vectors (FTC Standard Definition)
     * X+ = Audience, Y+ = Blue
     */
    private double[] getUniversalUnitVector(Direction dir)
    {
        // If Generic, we treat it as a standard identity vector (1,0) for internal math
        // But realistically, we should prevent converting GENERIC <-> FIELD
        if (this == GENERIC) return new double[]{1.0, 0.0};

        switch (dir)
        {
            case AUDIENCE:  return new double[]{ 1.0,  0.0}; // Universal X
            case BACKSTAGE: return new double[]{-1.0,  0.0}; // -Universal X
            case BLUE:      return new double[]{ 0.0,  1.0}; // Universal Y
            case RED:       return new double[]{ 0.0, -1.0}; // -Universal Y
            default:        return new double[]{ 0.0,  0.0};
        }
    }
}