package org.firstinspires.ftc.teamcode.util.measure.coordinate;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.firstinspires.ftc.teamcode.util.measure.geometry.Vector2d;

/**
 * Describes a {@link FieldCoordinate} in context of the FTC field
 */
public class FieldCoordinate extends Coordinate
{
    public final CoordinateSystem coordSys;

    /**
     * RIGHT_HAND sets the red alliance loading zone to (0, 0). FTC_STD (STANDARD)
     * sets the center of the field to (0, 0). Both coordinate systems are audience
     * down/goals up.
     */

    public FieldCoordinate(Distance x, Distance y)
    {
        this(x, y, CoordinateSystem.DECODE_PEDROPATH); // default to right-hand origin
    }

    public FieldCoordinate(Distance x, Distance y, CoordinateSystem coordSys)
    {
        super(x, y);
        this.coordSys = coordSys;
    }

    public FieldCoordinate(double xInch, double yInch, CoordinateSystem coordSys)
    {
        this(new Distance(xInch, DistanceUnit.INCH), new Distance(yInch, DistanceUnit.INCH), coordSys);
    }

    public double[] getCoordinate(DistanceUnit unit, CoordinateSystem coordSys)
    {
        return toCoordinateSystem(coordSys).getCoordinate(unit);
    }

    /**
     * @param unit
     * @return The transformed {@link FieldCoordinate} to the specified
     *         {@link DistanceUnit}
     */
    public FieldCoordinate toDistanceUnit(DistanceUnit unit)
    {
        if (this.isDistanceUnit(unit))
            return this;

        return new FieldCoordinate(x.toUnit(unit), y.toUnit(unit), coordSys);
    }

    /**
     * @param targetSys
     * @return The transformed {@link FieldCoordinate} to the specified
     *         {@link CoordinateSystem}
     */
    public FieldCoordinate toCoordinateSystem(CoordinateSystem targetSys)
    {
        if (this.coordSys == targetSys)
            return this;

        // 1. Convert THIS -> Universal
        Coordinate universalPoint = this.coordSys.toUniversal(this.x, this.y);

        // 2. Convert Universal -> TARGET
        Coordinate targetPoint = targetSys.fromUniversal(universalPoint);

        // Preserve the original distance unit
        return new FieldCoordinate(targetPoint.x.toUnit(this.x.unit), targetPoint.y.toUnit(this.y.unit), targetSys);
    }

    @Override
    public FieldCoordinate translate(Vector2d translation)
    {
        return new FieldCoordinate(x.plus(translation.x), y.plus(translation.y), coordSys);
    }

    public Vector2d vectorTo(FieldCoordinate otherCoord)
    {
        return super.vectorTo(otherCoord.toCoordinateSystem(this.coordSys));
    }

    /**
     * Calculates the {@link Distance} to the other {@link FieldCoordinate}, with
     * consideration to {@link DistanceUnit} and/or {@link CoordinateSystem}
     * mismatches.
     *
     * @param otherCoord
     * @return The {@link Distance} to the other {@link FieldCoordinate}
     */
    public Distance distanceTo(FieldCoordinate otherCoord)
    {
        // Convert the other FieldCoordinate to our FieldCoordinate's coordinate system
        return super.distanceTo(otherCoord.toCoordinateSystem(this.coordSys));
    }

    /**
     * Calculates the {@link Angle} to the other {@link FieldCoordinate}, with
     * consideration to {@link DistanceUnit} and/or {@link CoordinateSystem}
     * mismatches.
     *
     * @param otherCoord
     * @return The {@link Angle} to the other {@link FieldCoordinate}
     */
    public Angle angleTo(FieldCoordinate otherCoord)
    {
        // Convert the other FieldCoordinate to our FieldCoordinate's coordinate system
        return super.angleTo(otherCoord.toCoordinateSystem(this.coordSys));
    }

    public Pose toPedro()
    {
        FieldCoordinate toPedro = this.toCoordinateSystem(CoordinateSystem.DECODE_PEDROPATH);

        return new Pose(toPedro.x.getInch(), toPedro.y.getInch());
    }

    public boolean isCoordinateSystem(CoordinateSystem coordSys)
    {
        return this.coordSys == coordSys;
    }

    @Override
    public boolean equals(Object obj)
    {
        if (this == obj)
            return true;
        if (!(obj instanceof FieldCoordinate))
            return false;
        FieldCoordinate other = (FieldCoordinate) obj;
        // Convert both to universal (FTC standard)
        FieldCoordinate thisUniv = this.toCoordinateSystem(CoordinateSystem.DECODE_FTC);
        FieldCoordinate otherUniv = other.toCoordinateSystem(CoordinateSystem.DECODE_FTC);
        return thisUniv.x.equals(otherUniv.x) && thisUniv.y.equals(otherUniv.y);
    }

    @Override
    public String toString()
    {
        return coordSys.toString() + ": " + super.toString();
    }
}