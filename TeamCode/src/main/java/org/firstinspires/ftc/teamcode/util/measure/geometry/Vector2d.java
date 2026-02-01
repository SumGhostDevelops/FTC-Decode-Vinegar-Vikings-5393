package org.firstinspires.ftc.teamcode.util.measure.geometry;

import static org.firstinspires.ftc.teamcode.util.measure.coordinate.CoordinateSystem.calculateBasis;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.CoordinateSystem;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

public class Vector2d
{
    public final Distance x;
    public final Distance y;
    public final CoordinateSystem coordSys;
    public final DistanceUnit distUnit;
    public final AngleUnit angUnit = AngleUnit.DEGREES;

    public Vector2d(Distance x, Distance y, CoordinateSystem coordSys)
    {
        this.coordSys = coordSys;
        this.x = x;
        this.y = y.toUnit(x.unit);
        this.distUnit = x.unit;
    }

    // ... (Keep standard Getters: getLength, getDirection) ...
    public Distance getLength()
    {
        return new Distance(Math.hypot(x.magnitude, y.magnitude), x.unit);
    }

    public Angle getDirection()
    {
        // Math.atan2 returns radians, so we must use AngleUnit.RADIANS here
        return new Angle(Math.atan2(y.magnitude, x.magnitude), AngleUnit.RADIANS);
    }

    public Vector2d toDistanceUnit(DistanceUnit distanceUnit)
    {
        if (isDistanceUnit(distanceUnit)) return this;
        return new Vector2d(x.toUnit(distanceUnit), y.toUnit(distanceUnit), coordSys);
    }

    public Vector2d toCoordinateSystem(CoordinateSystem targetSys)
    {
        // Optimization: Immediate return if no change is needed
        if (this.coordSys == targetSys) return this;

        // Optimization: Access the pre-calculated basis vectors from CoordinateSystem
        // instead of calling a helper method or switch statement.

        // 1. Local -> Universal
        // Note: Vector rotation does not care about origin offsets, only axes.
        double xMag = this.x.magnitude;
        double yMag = this.y.magnitude;

        // Access private fields via public getters or if in same package (package-private)
        // Assuming we rely on the basis logic defined in CoordinateSystem logic:
        double[] currentXBasis = calculateBasis(this.coordSys.positiveX);
        double[] currentYBasis = calculateBasis(this.coordSys.positiveY);

        double globalX = (xMag * currentXBasis[0]) + (yMag * currentYBasis[0]);
        double globalY = (xMag * currentXBasis[1]) + (yMag * currentYBasis[1]);

        // 2. Universal -> Target
        double[] targetXBasis = calculateBasis(targetSys.positiveX);
        double[] targetYBasis = calculateBasis(targetSys.positiveY);

        double newXMag = (globalX * targetXBasis[0]) + (globalY * targetXBasis[1]);
        double newYMag = (globalX * targetYBasis[0]) + (globalY * targetYBasis[1]);

        return new Vector2d(
                new Distance(newXMag, x.unit),
                new Distance(newYMag, x.unit), // Use x.unit to preserve original unit
                targetSys
        );
    }

    // Small optimization: check for empty addition
    public Vector2d plus(Vector2d b)
    {
        if (b.x.magnitude == 0 && b.y.magnitude == 0) return this;

        Vector2d convertedB = b.toCoordinateSystem(this.coordSys);
        return new Vector2d(this.x.plus(convertedB.x), this.y.plus(convertedB.y), this.coordSys);
    }

    public Vector2d inverse()
    {
        return new Vector2d(x.multiply(-1), y.multiply(-1), coordSys);
    }

    public Vector2d minus(Vector2d b)
    {
        return plus(b.inverse());
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

        // Optimization: If they are the exact same coordinate system and unit,
        // we can skip the heavy 'toCoordinateSystem' conversion logic.
        if (this.coordSys == other.coordSys && this.distUnit == other.distUnit)
        {
            return this.x.equals(other.x) && this.y.equals(other.y);
        }

        // Otherwise, convert other to THIS system for apples-to-apples comparison
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