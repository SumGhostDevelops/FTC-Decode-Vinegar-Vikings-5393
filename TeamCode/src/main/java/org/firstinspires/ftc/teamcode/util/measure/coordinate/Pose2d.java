package org.firstinspires.ftc.teamcode.util.measure.coordinate;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.measure.angle.field.FieldHeading;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

public class Pose2d {
    public final FieldCoordinate coord;
    public final FieldHeading heading;

    /**
     * @param x       the x-coordinate distance component of the pose
     * @param y       the y-coordinate distance component of the pose
     * @param heading the orientation angle of the pose
     * @see FieldCoordinate#FieldCoordinate(Distance, Distance)
     */
    public Pose2d(Distance x, Distance y, Angle heading, CoordinateSystem coordinateSystem) {
        this.coord = new FieldCoordinate(x, y, coordinateSystem);
        this.heading = new FieldHeading(heading, coordinateSystem);
    }

    public Pose2d(FieldCoordinate coord, FieldHeading heading) {
        this.coord = coord;
        this.heading = heading.toSystem(coord.coordSys);
    }

    public static Pose2d fromPose2D(Pose2D pose, CoordinateSystem coordSys) {
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
    public static Pose2d fromPose3D(Pose3D pose, CoordinateSystem coordSys) {
        FieldCoordinate coord = new FieldCoordinate(
                new Distance(pose.getPosition().x, pose.getPosition().unit),
                new Distance(pose.getPosition().y, pose.getPosition().unit),
                coordSys);

        return new Pose2d(coord,
                new FieldHeading((pose.getOrientation().getYaw(AngleUnit.RADIANS)), AngleUnit.RADIANS, coordSys));
    }

    /**
     * Converts AprilTag SDK's robotPose to our Pose2d.
     * SDK outputs in (+X=Audience, +Y=Blue). We transform with (y, -x) to get
     * (+X=Blue, +Y=Backstage) in APRILTAG_SDK, then convert to DECODE_FTC.
     *
     * @param robotPose The robotPose from AprilTagDetection
     * @return A Pose2d in DECODE_FTC coordinate system
     */
    public static Pose2d fromAprilTagRobotPose(Pose3D robotPose) {
        // Transform SDK coordinates: (y, -x) rotates from (Audience/Blue) to
        // (Blue/Backstage)
        FieldCoordinate coord = new FieldCoordinate(
                new Distance(robotPose.getPosition().y, robotPose.getPosition().unit),
                new Distance(-robotPose.getPosition().x, robotPose.getPosition().unit),
                CoordinateSystem.APRILTAG_SDK);

        FieldHeading heading = new FieldHeading(
                robotPose.getOrientation().getYaw(AngleUnit.RADIANS),
                AngleUnit.RADIANS,
                CoordinateSystem.APRILTAG_SDK);

        // Convert from APRILTAG_SDK to DECODE_FTC
        return new Pose2d(coord, heading).toCoordinateSystem(CoordinateSystem.DECODE_FTC);
    }

    public Pose2d toDistanceUnit(DistanceUnit distanceUnit) {
        if (coord.isDistanceUnit(distanceUnit)) {
            return this;
        }

        return new Pose2d(coord.toDistanceUnit(distanceUnit), heading);
    }

    public Pose2d toCoordinateSystem(CoordinateSystem coordSys) {
        if (coord.isCoordinateSystem(coordSys)) {
            return this;
        }

        // Fix: Explicitly convert the heading to the target system as well
        return new Pose2d(
                coord.toCoordinateSystem(coordSys),
                heading.toSystem(coordSys));
    }

    public Pose2d toAngleUnit(AngleUnit angleUnit) {
        if (heading.angle.isUnit(angleUnit)) {
            return this;
        }

        return new Pose2d(coord, heading.toAngleUnit(angleUnit));
    }

    /**
     * @param otherPose
     * @return The straight line {@link Distance} to another {@link Pose2d}'s
     *         {@link Pose2d#coord}
     */
    public Distance distanceTo(Pose2d otherPose) {
        return distanceTo(otherPose.coord);
    }

    /**
     * @param coord The target field coordinate to calculate distance to
     * @return The straight line {@link Distance} to another {@link FieldCoordinate}
     */
    public Distance distanceTo(FieldCoordinate coord) {
        return this.coord.distanceTo(coord);
    }

    /**
     * @param otherPose
     * @return The {@link Angle} to another {@link Pose2d}'s {@link Pose2d#coord},
     *         taking into account this {@link Pose2d}'s {@link Pose2d#heading}; the
     *         relative {@link Angle}
     */
    public Angle bearingTo(Pose2d otherPose) {
        return bearingTo(otherPose.coord);
    }

    /**
     * @param otherCoord The target {@link FieldCoordinate} to calculate the
     *                   relative bearing to
     * @return The {@link Angle} to another {@link FieldCoordinate}, taking into
     *         account this {@link Pose2d}'s {@link Pose2d#heading}; the relative
     *         {@link Angle}
     */
    public Angle bearingTo(FieldCoordinate otherCoord) {
        return angleTo(otherCoord).minus(this.heading.angle);
    }

    /**
     * @param otherPose The target {@link Pose2d} whose {@link Pose2d#coord} is used
     *                  to compute the absolute {@link Angle}
     * @return The {@link Angle} to another {@link Pose2d}'s {@link Pose2d#coord}
     *         from this {@link Pose2d}'s {@link Pose2d#coord}; the absolute
     *         {@link Angle}
     */
    public Angle angleTo(Pose2d otherPose) {
        return angleTo(otherPose.coord);
    }

    /**
     * @param otherCoord
     * @return The {@link Angle} to another {@link FieldCoordinate} from this
     *         {@link Pose2d}'s {@link Pose2d#coord}; the absolute {@link Angle}
     */
    public Angle angleTo(FieldCoordinate otherCoord) {
        return this.coord.angleTo(otherCoord);
    }

    public Pose2D toPose2D() {
        double headingRad = this.heading.angle.getRadians();
        Coordinate univCoord = this.coord.coordSys.toUniversal(this.coord.x, this.coord.y);
        double xMeter = univCoord.x.getDistance(DistanceUnit.METER);
        double yMeter = univCoord.y.getDistance(DistanceUnit.METER);

        return new Pose2D(DistanceUnit.METER, xMeter, yMeter, AngleUnit.RADIANS, headingRad);
    }

    @Override
    public String toString() {
        return coord.toString() + "; " + heading.toString();
    }
}