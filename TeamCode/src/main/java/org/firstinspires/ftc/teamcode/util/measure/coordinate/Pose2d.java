package org.firstinspires.ftc.teamcode.util.measure.coordinate;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.measure.angle.field.FieldHeading;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.firstinspires.ftc.teamcode.util.measure.geometry.Vector2d;

public class Pose2d
{
    public final FieldCoordinate coord;
    public final FieldHeading heading;

    /**
     * @param x       the x-coordinate distance component of the pose
     * @param y       the y-coordinate distance component of the pose
     * @param heading the orientation angle of the pose
     * @see FieldCoordinate#FieldCoordinate(Distance, Distance)
     */
    public Pose2d(Distance x, Distance y, Angle heading, CoordinateSystem coordinateSystem)
    {
        this.coord = new FieldCoordinate(x, y, coordinateSystem);
        this.heading = new FieldHeading(heading, coordinateSystem);
    }

    public Pose2d(FieldCoordinate coord, FieldHeading heading)
    {
        this.coord = coord;
        this.heading = heading.toCoordinateSystem(coord.coordSys);
    }

    public static Pose2d fromPose2D(Pose2D pose, CoordinateSystem coordSys)
    {
        FieldCoordinate coord = new FieldCoordinate(
                new Distance(pose.getX(DistanceUnit.INCH), DistanceUnit.INCH),
                new Distance(pose.getY(DistanceUnit.INCH), DistanceUnit.INCH),
                coordSys);

        FieldHeading heading = new FieldHeading(pose.getHeading(AngleUnit.DEGREES), AngleUnit.DEGREES, coordSys);

        return new Pose2d(coord, heading);
    }

    /**
     * Converts a {@link Pose3D} into a more useful {@link Pose2d}
     */
    public static Pose2d fromPose3D(Pose3D pose, CoordinateSystem coordSys)
    {
        FieldCoordinate coord = new FieldCoordinate(
                new Distance(pose.getPosition().x, pose.getPosition().unit),
                new Distance(pose.getPosition().y, pose.getPosition().unit),
                coordSys);

        return new Pose2d(coord,
                new FieldHeading((pose.getOrientation().getYaw(AngleUnit.DEGREES)), AngleUnit.DEGREES, coordSys));
    }

    /**
     * Converts AprilTag SDK's robotPose to our Pose2d.
     * SDK has FTC coordinates, but a PedroPath-like heading.
     *
     * @param pose The robotPose from AprilTagDetection
     * @return A Pose2d
     */
    public static Pose2d fromAprilTagRobotPose(Pose3D pose)
    {
        FieldCoordinate coord = new FieldCoordinate(
                new Distance(pose.getPosition().x, pose.getPosition().unit),
                new Distance(pose.getPosition().y, pose.getPosition().unit),
                CoordinateSystem.DECODE_FTC);

        return new Pose2d(coord,
                new FieldHeading((pose.getOrientation().getYaw(AngleUnit.DEGREES)), AngleUnit.DEGREES, CoordinateSystem.DECODE_PEDROPATH));
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

        // Fix: Explicitly convert the heading to the target system as well
        return new Pose2d(
                coord.toCoordinateSystem(coordSys),
                heading.toCoordinateSystem(coordSys));
    }

    public Pose2d toAngleUnit(AngleUnit angleUnit)
    {
        if (heading.angle.isUnit(angleUnit))
        {
            return this;
        }

        return new Pose2d(coord, heading.toAngleUnit(angleUnit));
    }

    /**
     * Creates a new Pose2d shifted by the given local coordinates relative to this pose.
     * Useful for determining the position of components (turrets, cameras, intakes)
     * mounted at a fixed offset from the robot center.
     *
     * @param offset The forward (positive) or backward (negative) X and left (positive) or right (negative) Y distances from the center.
     * @return A new Pose2d representing the translated point.
     */
    public Pose2d transform(Vector2d offset)
    {
        return transform(offset.x, offset.y);
    }

    /**
     * Creates a new Pose2d shifted by a polar offset relative to this pose.
     * The {@code radius} is the distance from the robot center to the target point,
     * and {@code bearingOffset} is the angle measured from the robot's forward direction,
     * counter-clockwise positive (left of forward = positive).
     *
     * <p>This is equivalent to converting the polar offset to local Cartesian coordinates
     * and then calling {@link #transform(Distance, Distance)}.
     *
     * @param radius        The radial distance from the robot center to the offset point.
     * @param bearingOffset The angle from the robot's forward axis to the offset direction,
     *                      counter-clockwise positive.
     * @return A new Pose2d representing the translated point.
     */
    public Pose2d transform(Distance radius, Angle bearingOffset)
    {
        double r   = radius.getInch();
        double phi = bearingOffset.getRadians();

        // Convert polar (r, phi) to local Cartesian: forward = x-axis, left = y-axis
        Distance forwardOffset = new Distance(r * Math.cos(phi), DistanceUnit.INCH);
        Distance strafeOffset  = new Distance(r * Math.sin(phi), DistanceUnit.INCH);

        return transform(forwardOffset, strafeOffset);
    }

    /**
     * Creates a new Pose2d shifted by the given local coordinates relative to this pose.
     * Useful for determining the position of components (turrets, cameras, intakes)
     * mounted at a fixed offset from the robot center.
     *
     * @param forwardOffset The distance forward (positive) or backward (negative) from the center.
     * @param strafeOffset  The distance left (positive) or right (negative) from the center.
     * @return A new Pose2d representing the translated point.
     */
    public Pose2d transform(Distance forwardOffset, Distance strafeOffset)
    {
        double xLocal = forwardOffset.getInch();
        double yLocal = strafeOffset.getInch();

        double theta = this.heading.angle.getRadians();

        // Apply Rotation Matrix to convert Local(Robot) -> Global(Field)
        // dx_global = x_local * cos(theta) - y_local * sin(theta)
        // dy_global = x_local * sin(theta) + y_local * cos(theta)
        double dxField = xLocal * Math.cos(theta) - yLocal * Math.sin(theta);
        double dyField = xLocal * Math.sin(theta) + yLocal * Math.cos(theta);

        // Add global deltas to current global coordinates
        double newX = this.coord.x.getDistance(DistanceUnit.INCH) + dxField;
        double newY = this.coord.y.getDistance(DistanceUnit.INCH) + dyField;

        FieldCoordinate newCoord = new FieldCoordinate(
                new Distance(newX, DistanceUnit.INCH),
                new Distance(newY, DistanceUnit.INCH),
                this.coord.coordSys
        );

        return new Pose2d(newCoord, this.heading);
    }

    /**
     * Converts a global field coordinate into a local vector relative to the robot's center and heading.
     * +X is forward distance, +Y is left (strafe) distance.
     * * @param targetCoord The global point on the field.
     * @return A Vector2d representing the local offsets required to reach the target from the robot.
     */
    public Vector2d toLocalRelative(FieldCoordinate targetCoord)
    {
        // 1. Get raw global deltas in the shared coordinate system
        Vector2d globalDelta = this.coord.vectorTo(targetCoord);
        double dxField = globalDelta.x.getInch();
        double dyField = globalDelta.y.getInch();

        double theta = this.heading.angle.getRadians();

        // 2. Apply Inverse Rotation Matrix (transpose) to map Global(Field) -> Local(Robot)
        // x_local =  dx_global * cos(theta) + dy_global * sin(theta)
        // y_local = -dx_global * sin(theta) + dy_global * cos(theta)
        double localX = dxField * Math.cos(theta) + dyField * Math.sin(theta);
        double localY = -dxField * Math.sin(theta) + dyField * Math.cos(theta);

        return new Vector2d(
                new Distance(localX, DistanceUnit.INCH),
                new Distance(localY, DistanceUnit.INCH),
                CoordinateSystem.GENERIC
        );
    }

    /**
     * @param otherPose
     * @return The straight line {@link Distance} to another {@link Pose2d}'s
     * {@link Pose2d#coord}
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
     * @return The {@link Angle} to another {@link Pose2d}'s {@link Pose2d#coord},
     * taking into account this {@link Pose2d}'s {@link Pose2d#heading}; the
     * relative {@link Angle}
     */
    public Angle bearingTo(Pose2d otherPose)
    {
        return bearingTo(otherPose.coord);
    }

    /**
     * @param otherCoord The target {@link FieldCoordinate} to calculate the
     *                   relative bearing to
     * @return The {@link Angle} to another {@link FieldCoordinate}, taking into
     * account this {@link Pose2d}'s {@link Pose2d#heading}; the relative
     * {@link Angle}
     */
    public Angle bearingTo(FieldCoordinate otherCoord)
    {
        return angleTo(otherCoord).minus(this.heading.angle);
    }

    /**
     * @param otherCoord
     * @return The {@link Angle} to another {@link FieldCoordinate} from this
     * {@link Pose2d}'s {@link Pose2d#coord}; the absolute {@link Angle}
     */
    public Angle angleTo(FieldCoordinate otherCoord)
    {
        return this.coord.angleTo(otherCoord);
    }

    /**
     * @param otherPose The target {@link Pose2d} whose {@link Pose2d#coord} is used
     *                  to compute the absolute {@link Angle}
     * @return The {@link Angle} to another {@link Pose2d}'s {@link Pose2d#coord}
     * from this {@link Pose2d}'s {@link Pose2d#coord}; the absolute
     * {@link Angle}
     */
    public Angle angleTo(Pose2d otherPose)
    {
        return angleTo(otherPose.coord);
    }

    public Pose2D toPose2D()
    {
        double headingRad = this.heading.angle.getRadians();
        Coordinate univCoord = this.coord.coordSys.toUniversal(this.coord.x, this.coord.y);
        double xMeter = univCoord.x.getDistance(DistanceUnit.METER);
        double yMeter = univCoord.y.getDistance(DistanceUnit.METER);

        return new Pose2D(DistanceUnit.METER, xMeter, yMeter, AngleUnit.RADIANS, headingRad);
    }

    public Pose toPedro()
    {
        Pose2d pedroPose = this.toCoordinateSystem(CoordinateSystem.DECODE_PEDROPATH);

        return new Pose(pedroPose.coord.x.getInch(), pedroPose.coord.y.getInch());
    }

    @Override
    public String toString()
    {
        return coord.toString() + "; " + heading.toString();
    }
}