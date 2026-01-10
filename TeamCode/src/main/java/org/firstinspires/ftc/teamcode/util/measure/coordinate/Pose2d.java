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

    public Pose2d toDistanceUnit(DistanceUnit distanceUnit)
    {
        if (coord.isDistanceUnit(distanceUnit))
        {
            return this;
        }

        return new Pose2d(coord.toDistanceUnit(distanceUnit), heading);
    }

    public Pose2d toCoordinateSystem(FieldCoordinate.CoordinateSystem coordSys)
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
     * @return The converted {@link Pose2D}
     */
    public Pose2D toPose2D()
    {
        // make sure all of the values are even and as they should be (toAngleUnit()) is technically overkill but why not
        Pose2d verifiedPose = this.toAngleUnit(AngleUnit.RADIANS).toDistanceUnit(DistanceUnit.METER).toCoordinateSystem(FieldCoordinate.CoordinateSystem.FTC_STD);

        return new Pose2D(verifiedPose.coord.x.unit, verifiedPose.coord.x.magnitude, verifiedPose.coord.y.magnitude, verifiedPose.heading.unit, verifiedPose.heading.measure);
    }

    /**
     * Converts a {@link Pose3D} into a more useful {@link Pose2d}
     * @param pose The 3D pose to convert to a 2D pose
     * @return The converted {@link Pose2d}
     */
    public static Pose2d fromPose3D(Pose3D pose)
    {
        // AprilTag robotPose X/Y are in field coordinates but need to be negated
        // to match the expected behavior (distance decreases as robot approaches tag)
        FieldCoordinate coord = new FieldCoordinate(
                new Distance(-pose.getPosition().x, pose.getPosition().unit),
                new Distance(-pose.getPosition().y, pose.getPosition().unit),
                FieldCoordinate.CoordinateSystem.FTC_STD);

        return new Pose2d(coord, new Angle(pose.getOrientation().getYaw(AngleUnit.RADIANS), AngleUnit.RADIANS));
    }
}