package org.firstinspires.ftc.teamcode.util.measure.coordinate;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;
import org.firstinspires.ftc.teamcode.util.measure.angle.field.FieldHeading;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.junit.Test;
import static org.junit.Assert.*;

/**
 * Comprehensive unit tests for the {@link Pose2d} class.
 */
public class Pose2dUnitTests
{
    private static final double DELTA = 1e-9;

    // ==================== Constructor Tests ====================

    @Test
    public void constructor_fourArgs_createsFieldCoordinate()
    {
        Distance x = new Distance(3.0, DistanceUnit.METER);
        Distance y = new Distance(4.0, DistanceUnit.METER);
        Angle heading = new Angle(45.0, AngleUnit.DEGREES);

        Pose2d pose = new Pose2d(x, y, heading, CoordinateSystem.DECODE_PEDROPATH);

        assertEquals(3.0, pose.coord.x.magnitude, DELTA);
        assertEquals(4.0, pose.coord.y.magnitude, DELTA);
        assertEquals(45.0, pose.heading.angle.measure, DELTA);
        assertEquals(CoordinateSystem.DECODE_PEDROPATH, pose.coord.coordSys);
        assertEquals(CoordinateSystem.DECODE_PEDROPATH, pose.heading.system);
    }

    @Test
    public void constructor_twoArgs_fieldCoordinateAndHeading()
    {
        FieldCoordinate coord = new FieldCoordinate(
                new Distance(3.0, DistanceUnit.METER),
                new Distance(4.0, DistanceUnit.METER),
                CoordinateSystem.DECODE_FTC
        );
        FieldHeading heading = new FieldHeading(90.0, AngleUnit.DEGREES, CoordinateSystem.DECODE_FTC);

        Pose2d pose = new Pose2d(coord, heading);

        assertEquals(3.0, pose.coord.x.magnitude, DELTA);
        assertEquals(4.0, pose.coord.y.magnitude, DELTA);
        assertEquals(90.0, pose.heading.angle.measure, DELTA);
        assertEquals(CoordinateSystem.DECODE_FTC, pose.coord.coordSys);
    }

    @Test
    public void constructor_zeroValues()
    {
        Distance x = new Distance(0.0, DistanceUnit.METER);
        Distance y = new Distance(0.0, DistanceUnit.METER);
        Angle heading = new Angle(0.0, AngleUnit.DEGREES);

        Pose2d pose = new Pose2d(x, y, heading, CoordinateSystem.DECODE_PEDROPATH);

        assertEquals(0.0, pose.coord.x.magnitude, DELTA);
        assertEquals(0.0, pose.coord.y.magnitude, DELTA);
        assertEquals(0.0, pose.heading.angle.measure, DELTA);
    }

    @Test
    public void constructor_negativeValues()
    {
        Distance x = new Distance(-3.0, DistanceUnit.METER);
        Distance y = new Distance(-4.0, DistanceUnit.METER);
        Angle heading = new Angle(-45.0, AngleUnit.DEGREES);

        Pose2d pose = new Pose2d(x, y, heading, CoordinateSystem.DECODE_PEDROPATH);

        assertEquals(-3.0, pose.coord.x.magnitude, DELTA);
        assertEquals(-4.0, pose.coord.y.magnitude, DELTA);
        assertEquals(-45.0, pose.heading.angle.measure, DELTA);
    }

    @Test
    public void constructor_radianHeading()
    {
        Distance x = new Distance(1.0, DistanceUnit.METER);
        Distance y = new Distance(2.0, DistanceUnit.METER);
        Angle heading = new Angle(Math.PI / 4, AngleUnit.RADIANS);

        Pose2d pose = new Pose2d(x, y, heading, CoordinateSystem.DECODE_PEDROPATH);

        assertEquals(Math.PI / 4, pose.heading.angle.measure, DELTA);
        assertEquals(AngleUnit.RADIANS, pose.heading.angle.unit);
    }

    // ==================== toDistanceUnit() Tests ====================

    @Test
    public void toDistanceUnit_sameUnit_returnsSameInstance()
    {
        Pose2d pose = new Pose2d(
                new Distance(1.0, DistanceUnit.METER),
                new Distance(2.0, DistanceUnit.METER),
                new Angle(45.0, AngleUnit.DEGREES),
                CoordinateSystem.DECODE_PEDROPATH
        );

        Pose2d converted = pose.toDistanceUnit(DistanceUnit.METER);
        assertSame(pose, converted);
    }

    @Test
    public void toDistanceUnit_meterToCm()
    {
        Pose2d pose = new Pose2d(
                new Distance(1.0, DistanceUnit.METER),
                new Distance(2.0, DistanceUnit.METER),
                new Angle(45.0, AngleUnit.DEGREES),
                CoordinateSystem.DECODE_PEDROPATH
        );

        Pose2d converted = pose.toDistanceUnit(DistanceUnit.CM);
        assertEquals(100.0, converted.coord.x.magnitude, DELTA);
        assertEquals(200.0, converted.coord.y.magnitude, DELTA);
        assertEquals(DistanceUnit.CM, converted.coord.x.unit);
    }

    @Test
    public void toDistanceUnit_preservesHeading()
    {
        Pose2d pose = new Pose2d(
                new Distance(1.0, DistanceUnit.METER),
                new Distance(2.0, DistanceUnit.METER),
                new Angle(45.0, AngleUnit.DEGREES),
                CoordinateSystem.DECODE_PEDROPATH
        );

        Pose2d converted = pose.toDistanceUnit(DistanceUnit.CM);
        assertEquals(45.0, converted.heading.angle.measure, DELTA);
        assertEquals(AngleUnit.DEGREES, converted.heading.angle.unit);
    }

    @Test
    public void toDistanceUnit_preservesCoordinateSystem()
    {
        FieldCoordinate coord = new FieldCoordinate(
                new Distance(1.0, DistanceUnit.METER),
                new Distance(2.0, DistanceUnit.METER),
                CoordinateSystem.DECODE_FTC
        );
        Pose2d pose = new Pose2d(coord, new FieldHeading(45.0, AngleUnit.DEGREES, CoordinateSystem.DECODE_FTC));

        Pose2d converted = pose.toDistanceUnit(DistanceUnit.CM);
        assertEquals(CoordinateSystem.DECODE_FTC, converted.coord.coordSys);
    }

    // ==================== toCoordinateSystem() Tests ====================

    @Test
    public void toCoordinateSystem_sameSystem_returnsSameInstance()
    {
        Pose2d pose = new Pose2d(
                new Distance(1.0, DistanceUnit.METER),
                new Distance(2.0, DistanceUnit.METER),
                new Angle(45.0, AngleUnit.DEGREES),
                CoordinateSystem.DECODE_PEDROPATH
        );

        Pose2d converted = pose.toCoordinateSystem(CoordinateSystem.DECODE_PEDROPATH);
        assertSame(pose, converted);
    }

    @Test
    public void toCoordinateSystem_pedroToFtcStd()
    {
        // Pedro Center (72, 72) -> FTC (0, 0)
        Pose2d pose = new Pose2d(
                new Distance(72.0, DistanceUnit.INCH),
                new Distance(72.0, DistanceUnit.INCH),
                new Angle(45.0, AngleUnit.DEGREES),
                CoordinateSystem.DECODE_PEDROPATH
        );

        Pose2d converted = pose.toCoordinateSystem(CoordinateSystem.DECODE_FTC);

        // Coordinates verify
        assertEquals(0.0, converted.coord.x.magnitude, DELTA);
        assertEquals(0.0, converted.coord.y.magnitude, DELTA);
        assertEquals(CoordinateSystem.DECODE_FTC, converted.coord.coordSys);

        // Heading Verify:
        // Pedro 0 deg = Blue = FTC 90 deg
        // Pedro 45 deg = 45 deg Left of Blue
        // FTC = 90 + 45 = 135 deg
        assertEquals(135.0, converted.heading.angle.measure, DELTA);
    }

    @Test
    public void toCoordinateSystem_preservesPhysicalHeadingNotNumber()
    {
        // Pedro 0 degrees = Facing Blue Alliance
        Pose2d pose = new Pose2d(
                new Distance(72.0, DistanceUnit.INCH),
                new Distance(72.0, DistanceUnit.INCH),
                new Angle(0.0, AngleUnit.DEGREES),
                CoordinateSystem.DECODE_PEDROPATH
        );

        Pose2d converted = pose.toCoordinateSystem(CoordinateSystem.DECODE_FTC);

        // In FTC system, Facing Blue is 90 degrees (Audience is 0)
        assertEquals(90.0, converted.heading.angle.measure, DELTA);
    }

    // ==================== toAngleUnit() Tests ====================

    @Test
    public void toAngleUnit_sameUnit_returnsSameInstance()
    {
        Pose2d pose = new Pose2d(
                new Distance(1.0, DistanceUnit.METER),
                new Distance(2.0, DistanceUnit.METER),
                new Angle(45.0, AngleUnit.DEGREES),
                CoordinateSystem.DECODE_PEDROPATH
        );

        Pose2d converted = pose.toAngleUnit(AngleUnit.DEGREES);
        assertSame(pose, converted);
    }

    @Test
    public void toAngleUnit_degreesToRadians()
    {
        Pose2d pose = new Pose2d(
                new Distance(1.0, DistanceUnit.METER),
                new Distance(2.0, DistanceUnit.METER),
                new Angle(90.0, AngleUnit.DEGREES),
                CoordinateSystem.DECODE_PEDROPATH
        );

        Pose2d converted = pose.toAngleUnit(AngleUnit.RADIANS);
        assertEquals(Math.PI / 2, converted.heading.angle.measure, DELTA);
        assertEquals(AngleUnit.RADIANS, converted.heading.angle.unit);
    }

    @Test
    public void toAngleUnit_preservesCoordinates()
    {
        Pose2d pose = new Pose2d(
                new Distance(3.0, DistanceUnit.METER),
                new Distance(4.0, DistanceUnit.METER),
                new Angle(45.0, AngleUnit.DEGREES),
                CoordinateSystem.DECODE_PEDROPATH
        );

        Pose2d converted = pose.toAngleUnit(AngleUnit.RADIANS);
        assertEquals(3.0, converted.coord.x.magnitude, DELTA);
        assertEquals(4.0, converted.coord.y.magnitude, DELTA);
    }

    // ==================== distanceTo(Pose2d) Tests ====================

    @Test
    public void distanceTo_pose_samePose_returnsZero()
    {
        Pose2d pose = new Pose2d(
                new Distance(3.0, DistanceUnit.METER),
                new Distance(4.0, DistanceUnit.METER),
                new Angle(45.0, AngleUnit.DEGREES),
                CoordinateSystem.DECODE_PEDROPATH
        );

        Distance distance = pose.distanceTo(pose);
        assertEquals(0.0, distance.magnitude, DELTA);
    }

    @Test
    public void distanceTo_pose_345Triangle()
    {
        Pose2d a = new Pose2d(
                new Distance(0.0, DistanceUnit.METER),
                new Distance(0.0, DistanceUnit.METER),
                new Angle(0.0, AngleUnit.DEGREES),
                CoordinateSystem.DECODE_PEDROPATH
        );
        Pose2d b = new Pose2d(
                new Distance(3.0, DistanceUnit.METER),
                new Distance(4.0, DistanceUnit.METER),
                new Angle(90.0, AngleUnit.DEGREES),
                CoordinateSystem.DECODE_PEDROPATH
        );

        Distance distance = a.distanceTo(b);
        assertEquals(5.0, distance.magnitude, DELTA);
    }

    // ==================== angleTo(Pose2d) Tests ====================

    @Test
    public void angleTo_pose_positiveXAxis()
    {
        Pose2d a = new Pose2d(
                new Distance(0.0, DistanceUnit.METER),
                new Distance(0.0, DistanceUnit.METER),
                new Angle(0.0, AngleUnit.DEGREES),
                CoordinateSystem.DECODE_PEDROPATH
        );
        Pose2d b = new Pose2d(
                new Distance(5.0, DistanceUnit.METER),
                new Distance(0.0, DistanceUnit.METER),
                new Angle(90.0, AngleUnit.DEGREES),
                CoordinateSystem.DECODE_PEDROPATH
        );

        // Vector is (5, 0) in Pedro. Angle is 0.
        Angle angle = a.angleTo(b);
        assertEquals(0.0, angle.measure, DELTA);
    }

    // ==================== bearingTo(Pose2d) Tests ====================

    @Test
    public void bearingTo_pose_sameHeading()
    {
        Pose2d a = new Pose2d(
                new Distance(0.0, DistanceUnit.METER),
                new Distance(0.0, DistanceUnit.METER),
                new Angle(0.0, AngleUnit.RADIANS),
                CoordinateSystem.DECODE_PEDROPATH
        );
        Pose2d b = new Pose2d(
                new Distance(1.0, DistanceUnit.METER),
                new Distance(0.0, DistanceUnit.METER),
                new Angle(0.0, AngleUnit.RADIANS),
                CoordinateSystem.DECODE_PEDROPATH
        );

        // Target at 0 radians (relative to Pedro). Robot facing 0 radians. Relative = 0.
        Angle bearing = a.bearingTo(b);
        assertEquals(0.0, bearing.measure, DELTA);
    }

    @Test
    public void bearingTo_pose_targetToTheLeft()
    {
        Pose2d pose = new Pose2d(
                new Distance(0.0, DistanceUnit.METER),
                new Distance(0.0, DistanceUnit.METER),
                new Angle(0.0, AngleUnit.RADIANS),
                CoordinateSystem.DECODE_PEDROPATH
        );
        Pose2d target = new Pose2d(
                new Distance(0.0, DistanceUnit.METER),
                new Distance(1.0, DistanceUnit.METER),
                new Angle(0.0, AngleUnit.RADIANS),
                CoordinateSystem.DECODE_PEDROPATH
        );

        // Target is at (0, 1) in Pedro. Angle is +90 (PI/2).
        // Robot facing 0. Relative = +90.
        Angle bearing = pose.bearingTo(target);
        assertEquals(Math.PI / 2, bearing.measure, DELTA);
    }
}