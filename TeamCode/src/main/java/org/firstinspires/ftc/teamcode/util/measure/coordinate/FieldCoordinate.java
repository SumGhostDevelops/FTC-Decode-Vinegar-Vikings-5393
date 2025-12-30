package org.firstinspires.ftc.teamcode.util.measure.coordinate;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

/**
 * Describes a {@link FieldCoordinate} in context of the FTC field
 */
public class FieldCoordinate extends Coordinate
{
    public final CoordinateSystem coordSys;

    /**
     * RIGHT_HAND sets the red alliance loading zone to (0, 0). FTC_STD (STANDARD) sets the center of the field to (0, 0). Both coordinate systems are audience down/goals up.
     */
    public enum CoordinateSystem
    {
        RIGHT_HAND,
        FTC_STD,
    }

    public FieldCoordinate(Distance x, Distance y)
    {
        this(x, y, CoordinateSystem.RIGHT_HAND); // right hand cause ftc standard is dumb
    }

    public FieldCoordinate(Distance x, Distance y, CoordinateSystem coordSys)
    {
        super(x, y);
        this.coordSys = coordSys;
    }

    /**
     * @param distanceUnit
     * @return The transformed {@link FieldCoordinate} to the specified {@link DistanceUnit}
     */
    public FieldCoordinate toDistanceUnit(DistanceUnit distanceUnit)
    {
        if (this.isDistanceUnit(distanceUnit)) return this;

        return new FieldCoordinate(x.toUnit(distanceUnit), y.toUnit(distanceUnit), coordSys);
    }

    /**
     * @param coordSys
     * @return The transformed {@link FieldCoordinate} to the specified {@link CoordinateSystem}
     */
    public FieldCoordinate toCoordinateSystem(CoordinateSystem coordSys)
    {
        if (this.isCoordinateSystem(coordSys)) return this;

        Distance offset = new Distance(72, DistanceUnit.INCH); // see the javadoc for CoordinateSystem
        Vector2d translation = (coordSys == CoordinateSystem.RIGHT_HAND)
                ? new Vector2d(offset, offset) // FTC -> right_hand, add 72, 72
                : new Vector2d(offset, offset).inverse(); // right_hand -> ftc, subtract 72, 72

        return new FieldCoordinate(x.plus(translation.x), y.plus(translation.y), coordSys);
    }

    @Override
    public FieldCoordinate translate(Vector2d translation)
    {
        return new FieldCoordinate(x.plus(translation.x), y.plus(translation.y), coordSys);
    }

    private FieldCoordinate toComparableStandard()
    {
        return this
                .toDistanceUnit(DistanceUnit.METER)
                .toCoordinateSystem(CoordinateSystem.RIGHT_HAND);
    }

    /**
     * Calculates the {@link Distance} to the other {@link FieldCoordinate}, with consideration to {@link DistanceUnit} and/or {@link CoordinateSystem} mismatches.
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
     * Calculates the {@link Angle} to the other {@link FieldCoordinate}, with consideration to {@link DistanceUnit} and/or {@link CoordinateSystem} mismatches.
     *
     * @param otherCoord
     * @return The {@link Angle} to the other {@link FieldCoordinate}
     */
    public Angle angleTo(FieldCoordinate otherCoord)
    {
        // Convert the other FieldCoordinate to our FieldCoordiante's coordinate system
        return super.angleTo(otherCoord.toCoordinateSystem(this.coordSys));
    }

    public boolean isCoordinateSystem(CoordinateSystem coordSys)
    {
        return this.coordSys == coordSys;
    }

    public boolean equals(FieldCoordinate otherCoord)
    {
        FieldCoordinate thisCoord = this.toDistanceUnit(DistanceUnit.METER).toCoordinateSystem(CoordinateSystem.RIGHT_HAND);
        otherCoord = otherCoord.toDistanceUnit(DistanceUnit.METER).toCoordinateSystem(CoordinateSystem.RIGHT_HAND);

        return thisCoord.x.equals(otherCoord.x) && thisCoord.y.equals(otherCoord.y);
    }
}