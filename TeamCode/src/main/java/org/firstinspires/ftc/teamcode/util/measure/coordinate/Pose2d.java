package org.firstinspires.ftc.teamcode.util.measure.coordinate;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

public class Pose2d
{
    public final FieldCoordinate coord;
    public final Angle heading;

    /**
     * @param x       the x-coordinate distance component of the pose
     * @param y       the y-coordinate distance component of the pose
     * @param heading the orientation angle of the pose
     * @see FieldCoordinate#FieldCoordinate(Distance, Distance)
     */
    public Pose2d(Distance x, Distance y, Angle heading)
    {
        this.coord = new FieldCoordinate(x, y);
        this.heading = heading;
    }

    public Pose2d(FieldCoordinate coord, Angle heading)
    {
        this.coord = coord;
        this.heading = heading;
    }

    public static Pose2d fromPose2D(Pose2D pose, CoordinateSystem coordSys)
    {
        FieldCoordinate coord = new FieldCoordinate(
                new Distance(pose.getX(DistanceUnit.INCH), DistanceUnit.INCH),
                new Distance(pose.getY(DistanceUnit.INCH), DistanceUnit.INCH),
                coordSys
        );

        Angle angle = new Angle(pose.getHeading(AngleUnit.DEGREES), AngleUnit.DEGREES);

        return new Pose2d(coord, angle);
    }

    /**
     * Converts a {@link Pose3D} into a more useful {@link Pose2d}
     *
     * @param pose     The 3D pose to convert to a 2D pose
     * @param coordSys The coordinate system to use for the 2D pose
     * @return The converted {@link Pose2d}
     */
    public static Pose2d fromPose3D(Pose3D pose, CoordinateSystem coordSys)
    {
        FieldCoordinate coord = new FieldCoordinate(
                new Distance(pose.getPosition().x, pose.getPosition().unit),
                new Distance(pose.getPosition().y, pose.getPosition().unit),
                coordSys);

        return new Pose2d(coord, new Angle((pose.getOrientation().getYaw(AngleUnit.RADIANS)), AngleUnit.RADIANS));
    }

    public Pose2d toDistanceUnit(DistanceUnit distanceUnit)
    {
        if (coord.isDistanceUnit(distanceUnit))
        {
            return this;
        }

        return new Pose2d(coord.toDistanceUnit(distanceUnit), heading);
    }

    public Pose2d toCoordinateSystem(CoordinateSystem coordSys)
    {
        if (coord.isCoordinateSystem(coordSys))
        {
            return this;
        }

        return new Pose2d(coord.toCoordinateSystem(coordSys), heading);
    }

    public Pose2d toAngleUnit(AngleUnit angleUnit)
    {
        if (heading.isUnit(angleUnit))
        {
            return this;
        }

        return new Pose2d(coord, heading.toUnit(angleUnit));
    }

    /**
     * @param otherPose
     * @return The straight line {@link Distance} to another {@link Pose2d}'s {@link Pose2d#coord}
     */
    public Distance distanceTo(Pose2d otherPose)
    {
        return distanceTo(otherPose.coord);
    }

    /**
     * @param coord The target field coordinate to calculate distance to
     * @return The straight line {@link Distance} to another {@link FieldCoordinate}
     */
    public Distance distanceTo(FieldCoordinate coord)
    {
        return this.coord.distanceTo(coord);
    }

    /**
     * @param otherPose
     * @return The {@link Angle} to another {@link Pose2d}'s {@link Pose2d#coord}, taking into account this {@link Pose2d}'s {@link Pose2d#heading}; the relative {@link Angle}
     */
    public Angle bearingTo(Pose2d otherPose)
    {
        return bearingTo(otherPose.coord);
    }

    /**
     * @param otherCoord The target {@link FieldCoordinate} to calculate the relative bearing to
     * @return The {@link Angle} to another {@link FieldCoordinate}, taking into account this {@link Pose2d}'s {@link Pose2d#heading}; the relative {@link Angle}
     */
    public Angle bearingTo(FieldCoordinate otherCoord)
    {
        return angleTo(otherCoord).minus(this.heading);
    }

    /**
     * @param otherPose The target {@link Pose2d} whose {@link Pose2d#coord} is used to compute the absolute {@link Angle}
     * @return The {@link Angle} to another {@link Pose2d}'s {@link Pose2d#coord} from this {@link Pose2d}'s {@link Pose2d#coord}; the absolute {@link Angle}
     */
    public Angle angleTo(Pose2d otherPose)
    {
        return angleTo(otherPose.coord);
    }

    /**
     * @param otherCoord
     * @return The {@link Angle} to another {@link FieldCoordinate} from this {@link Pose2d}'s {@link Pose2d#coord}; the absolute {@link Angle}
     */
    public Angle angleTo(FieldCoordinate otherCoord)
    {
        return this.coord.angleTo(otherCoord);
    }

    /**
     * Convert our custom {@link Pose2d} to the FTC Standard {@link Pose2D}
     */
    public Pose2D toPose2D()
    {
        // 1. Convert Heading directly
        double headingRad = this.heading.getAngle(AngleUnit.RADIANS);

        // 2. Convert Coordinate to Universal (FTC Standard) directly
        Coordinate univCoord = this.coord.coordSys.toUniversal(this.coord.x, this.coord.y);

        // 3. Extract Meters directly
        double xMeter = univCoord.x.getDistance(DistanceUnit.METER);
        double yMeter = univCoord.y.getDistance(DistanceUnit.METER);

        return new Pose2D(DistanceUnit.METER, xMeter, yMeter, AngleUnit.RADIANS, headingRad);
    }

    public Pose2d toCoordSys(CoordinateSystem targetCoordSys)
    {
        return new Pose2d(this.coord.toCoordinateSystem(targetCoordSys), this.heading);
    }

    @Override
    public String toString()
    {
        return coord.toString() + "; " + heading.toString();
    }
}