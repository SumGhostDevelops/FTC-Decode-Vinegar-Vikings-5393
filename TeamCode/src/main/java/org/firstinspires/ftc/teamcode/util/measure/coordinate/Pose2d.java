package org.firstinspires.ftc.teamcode.util.measure.coordinate;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

public class Pose2d
{
    public final FieldCoordinate coord;
    public final Angle heading;

    /**
     * @param x
     * @param y
     * @param heading
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
     * @param coord
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
     * @param otherCoord
     * @return The {@link Angle} to another {@link FieldCoordinate}, taking into account this {@link Pose2d}'s {@link Pose2d#heading}; the relative {@link Angle}
     */
    public Angle bearingTo(FieldCoordinate otherCoord)
    {
        return angleTo(otherCoord).minus(this.heading);
    }

    /**
     * @param otherPose
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
     * Converts a {@link Pose3D} into a more useful {@link Pose2d}
     * @param pose
     * @return The converted {@link Pose2d}
     */
    public static Pose2d fromPose3D(Pose3D pose)
    {
        FieldCoordinate coord = new FieldCoordinate(
                new Distance(pose.getPosition().x, pose.getPosition().unit),
                new Distance(pose.getPosition().y, pose.getPosition().unit),
                FieldCoordinate.CoordinateSystem.FTC_STD);

        return new Pose2d(coord, new Angle(pose.getOrientation().getYaw(AngleUnit.RADIANS), AngleUnit.RADIANS));
    }
}