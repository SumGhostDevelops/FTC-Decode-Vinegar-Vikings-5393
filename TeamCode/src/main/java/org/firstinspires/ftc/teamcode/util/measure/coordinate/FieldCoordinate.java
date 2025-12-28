package org.firstinspires.ftc.teamcode.util.measure.coordinate;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

/**
 * Describes a {@link FieldCoordinate} in context of the FTC field
 */
public class FieldCoordinate
{
    public final Coordinate coord;
    public final CoordinateSystem coordSys;

    /**
     * RIGHT_HAND sets the red alliance loading zone to (0, 0). FTC_STD (STANDARD) sets the center of the field to (0, 0). Both coordinate systems are audience down/goals up.
     */
    public enum CoordinateSystem
    {
        RIGHT_HAND,
        FTC_STD,
    }

    public FieldCoordinate(Coordinate coord, CoordinateSystem coordSys)
    {
        this.coord = coord;
        this.coordSys = coordSys;
    }

    /**
     * @param distanceUnit
     * @return The transformed {@link FieldCoordinate} to the specified {@link DistanceUnit}
     */
    public FieldCoordinate toDistanceUnit(DistanceUnit distanceUnit)
    {
        if (isDistanceUnit(distanceUnit))
        {
            return this;
        }

        return new FieldCoordinate(coord.toUnit(distanceUnit), coordSys);
    }

    /**
     * @param coordSys
     * @return The transformed {@link FieldCoordinate} to the specified {@link CoordinateSystem}
     */
    public FieldCoordinate toCoordinateSystem(CoordinateSystem coordSys)
    {
        if (isCoordinateSystem(coordSys))
        {
            return this;
        }

        Distance offset = new Distance(72, DistanceUnit.INCH);
        Vector2d translation = new Vector2d(offset, offset);

        switch (coordSys)
        {
            case RIGHT_HAND: // convert FTC_STD to RIGHT_HAND
                return new FieldCoordinate(coord.translate(translation), coordSys);
            case FTC_STD: // convert RIGHT_HAND to FTC_STD
                translation = translation.inverse(); // instead of offsetting positive 72,72, offset negative 72,72
                return new FieldCoordinate(coord.translate(translation), coordSys);
            default:
                throw new IllegalArgumentException(coordSys.toString() + " is not a valid CoordinateSystem");
        }
    }

    /**
     * Calculates the {@link Distance} to the other {@link FieldCoordinate}, with consideration to {@link DistanceUnit} and/or {@link CoordinateSystem} mismatches.
     * @param otherCoord
     * @return The {@link Distance} to the other {@link FieldCoordinate}
     */
    public Distance distanceTo(FieldCoordinate otherCoord)
    {
        // Convert the other FieldCoordinate to our FieldCoordinate's coordinate system
        otherCoord = otherCoord.toCoordinateSystem(this.coordSys);

        return this.coord.distanceTo(otherCoord.coord);
    }

    /**
     * Calculates the {@link Angle} to the other {@link FieldCoordinate}, with consideration to {@link DistanceUnit} and/or {@link CoordinateSystem} mismatches.
     * @param otherCoord
     * @return The {@link Angle} to the other {@link FieldCoordinate}
     */
    public Angle angleTo(FieldCoordinate otherCoord)
    {
        // Convert the other FieldCoordinate to our FieldCoordiante's coordinate system
        otherCoord = otherCoord.toCoordinateSystem(this.coordSys);

        return this.coord.angleTo(otherCoord.coord);
    }

    public boolean isDistanceUnit(DistanceUnit distanceUnit)
    {
        return this.coord.isDistanceUnit(distanceUnit);
    }

    public boolean isCoordinateSystem(CoordinateSystem coordSys)
    {
        return this.coordSys == coordSys;
    }
}