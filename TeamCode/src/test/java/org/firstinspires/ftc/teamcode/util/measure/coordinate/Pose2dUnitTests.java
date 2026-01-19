package org.firstinspires.ftc.teamcode.util.measure.coordinate;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
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
    public void constructor_threeArgs_createsFieldCoordinate()
    {
        Distance x = new Distance(3.0, DistanceUnit.METER);
        Distance y = new Distance(4.0, DistanceUnit.METER);
        Angle heading = new Angle(45.0, AngleUnit.DEGREES);

        Pose2d pose = new Pose2d(x, y, heading);

        assertEquals(3.0, pose.coord.x.magnitude, DELTA);
        assertEquals(4.0, pose.coord.y.magnitude, DELTA);
        assertEquals(45.0, pose.heading.measure, DELTA);
        assertEquals(CoordinateSystem.DECODE_PEDROPATH, pose.coord.coordSys);
    }

    @Test
    public void constructor_twoArgs_fieldCoordinateAndHeading()
    {
        FieldCoordinate coord = new FieldCoordinate(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER),
            CoordinateSystem.DECODE_FTC
        );
        Angle heading = new Angle(90.0, AngleUnit.DEGREES);

        Pose2d pose = new Pose2d(coord, heading);

        assertEquals(3.0, pose.coord.x.magnitude, DELTA);
        assertEquals(4.0, pose.coord.y.magnitude, DELTA);
        assertEquals(90.0, pose.heading.measure, DELTA);
        assertEquals(CoordinateSystem.DECODE_FTC, pose.coord.coordSys);
    }

    @Test
    public void constructor_zeroValues()
    {
        Distance x = new Distance(0.0, DistanceUnit.METER);
        Distance y = new Distance(0.0, DistanceUnit.METER);
        Angle heading = new Angle(0.0, AngleUnit.DEGREES);

        Pose2d pose = new Pose2d(x, y, heading);

        assertEquals(0.0, pose.coord.x.magnitude, DELTA);
        assertEquals(0.0, pose.coord.y.magnitude, DELTA);
        assertEquals(0.0, pose.heading.measure, DELTA);
    }

    @Test
    public void constructor_negativeValues()
    {
        Distance x = new Distance(-3.0, DistanceUnit.METER);
        Distance y = new Distance(-4.0, DistanceUnit.METER);
        Angle heading = new Angle(-45.0, AngleUnit.DEGREES);

        Pose2d pose = new Pose2d(x, y, heading);

        assertEquals(-3.0, pose.coord.x.magnitude, DELTA);
        assertEquals(-4.0, pose.coord.y.magnitude, DELTA);
        assertEquals(-45.0, pose.heading.measure, DELTA);
    }

    @Test
    public void constructor_radianHeading()
    {
        Distance x = new Distance(1.0, DistanceUnit.METER);
        Distance y = new Distance(2.0, DistanceUnit.METER);
        Angle heading = new Angle(Math.PI / 4, AngleUnit.RADIANS);

        Pose2d pose = new Pose2d(x, y, heading);

        assertEquals(Math.PI / 4, pose.heading.measure, DELTA);
        assertEquals(AngleUnit.RADIANS, pose.heading.unit);
    }

    // ==================== toDistanceUnit() Tests ====================

    @Test
    public void toDistanceUnit_sameUnit_returnsSameInstance()
    {
        Pose2d pose = new Pose2d(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(2.0, DistanceUnit.METER),
            new Angle(45.0, AngleUnit.DEGREES)
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
            new Angle(45.0, AngleUnit.DEGREES)
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
            new Angle(45.0, AngleUnit.DEGREES)
        );

        Pose2d converted = pose.toDistanceUnit(DistanceUnit.CM);
        assertEquals(45.0, converted.heading.measure, DELTA);
        assertEquals(AngleUnit.DEGREES, converted.heading.unit);
    }

    @Test
    public void toDistanceUnit_preservesCoordinateSystem()
    {
        FieldCoordinate coord = new FieldCoordinate(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(2.0, DistanceUnit.METER),
            CoordinateSystem.DECODE_FTC
        );
        Pose2d pose = new Pose2d(coord, new Angle(45.0, AngleUnit.DEGREES));

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
            new Angle(45.0, AngleUnit.DEGREES)
        );

        Pose2d converted = pose.toCoordinateSystem(CoordinateSystem.DECODE_PEDROPATH);
        assertSame(pose, converted);
    }

    @Test
    public void toCoordinateSystem_rightHandToFtcStd()
    {
        Pose2d pose = new Pose2d(
            new Distance(72.0, DistanceUnit.INCH),
            new Distance(72.0, DistanceUnit.INCH),
            new Angle(45.0, AngleUnit.DEGREES)
        );

        Pose2d converted = pose.toCoordinateSystem(CoordinateSystem.DECODE_FTC);
        assertEquals(0.0, converted.coord.x.magnitude, DELTA);
        assertEquals(0.0, converted.coord.y.magnitude, DELTA);
        assertEquals(CoordinateSystem.DECODE_FTC, converted.coord.coordSys);
    }

    @Test
    public void toCoordinateSystem_preservesHeading()
    {
        Pose2d pose = new Pose2d(
            new Distance(72.0, DistanceUnit.INCH),
            new Distance(72.0, DistanceUnit.INCH),
            new Angle(90.0, AngleUnit.DEGREES)
        );

        Pose2d converted = pose.toCoordinateSystem(CoordinateSystem.DECODE_FTC);
        assertEquals(90.0, converted.heading.measure, DELTA);
    }

    // ==================== toAngleUnit() Tests ====================

    @Test
    public void toAngleUnit_sameUnit_returnsSameInstance()
    {
        Pose2d pose = new Pose2d(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(2.0, DistanceUnit.METER),
            new Angle(45.0, AngleUnit.DEGREES)
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
            new Angle(90.0, AngleUnit.DEGREES)
        );

        Pose2d converted = pose.toAngleUnit(AngleUnit.RADIANS);
        assertEquals(Math.PI / 2, converted.heading.measure, DELTA);
        assertEquals(AngleUnit.RADIANS, converted.heading.unit);
    }

    @Test
    public void toAngleUnit_radiansToDegrees()
    {
        Pose2d pose = new Pose2d(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(2.0, DistanceUnit.METER),
            new Angle(Math.PI / 4, AngleUnit.RADIANS)
        );

        Pose2d converted = pose.toAngleUnit(AngleUnit.DEGREES);
        assertEquals(45.0, converted.heading.measure, DELTA);
        assertEquals(AngleUnit.DEGREES, converted.heading.unit);
    }

    @Test
    public void toAngleUnit_preservesCoordinates()
    {
        Pose2d pose = new Pose2d(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER),
            new Angle(45.0, AngleUnit.DEGREES)
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
            new Angle(45.0, AngleUnit.DEGREES)
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
            new Angle(0.0, AngleUnit.DEGREES)
        );
        Pose2d b = new Pose2d(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER),
            new Angle(90.0, AngleUnit.DEGREES)
        );

        Distance distance = a.distanceTo(b);
        assertEquals(5.0, distance.magnitude, DELTA);
    }

    @Test
    public void distanceTo_pose_ignoresHeading()
    {
        Pose2d a = new Pose2d(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER),
            new Angle(0.0, AngleUnit.DEGREES)
        );
        Pose2d b1 = new Pose2d(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER),
            new Angle(0.0, AngleUnit.DEGREES)
        );
        Pose2d b2 = new Pose2d(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER),
            new Angle(180.0, AngleUnit.DEGREES)
        );

        assertEquals(a.distanceTo(b1).magnitude, a.distanceTo(b2).magnitude, DELTA);
    }

    // ==================== distanceTo(FieldCoordinate) Tests ====================

    @Test
    public void distanceTo_fieldCoordinate_345Triangle()
    {
        Pose2d pose = new Pose2d(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER),
            new Angle(45.0, AngleUnit.DEGREES)
        );
        FieldCoordinate coord = new FieldCoordinate(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER),
            CoordinateSystem.DECODE_PEDROPATH
        );

        Distance distance = pose.distanceTo(coord);
        assertEquals(5.0, distance.magnitude, DELTA);
    }

    // ==================== angleTo(Pose2d) Tests ====================

    @Test
    public void angleTo_pose_positiveXAxis()
    {
        Pose2d a = new Pose2d(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER),
            new Angle(0.0, AngleUnit.DEGREES)
        );
        Pose2d b = new Pose2d(
            new Distance(5.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER),
            new Angle(90.0, AngleUnit.DEGREES)
        );

        Angle angle = a.angleTo(b);
        assertEquals(0.0, angle.measure, DELTA);
    }

    @Test
    public void angleTo_pose_firstQuadrant()
    {
        Pose2d a = new Pose2d(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER),
            new Angle(0.0, AngleUnit.DEGREES)
        );
        Pose2d b = new Pose2d(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(1.0, DistanceUnit.METER),
            new Angle(0.0, AngleUnit.DEGREES)
        );

        Angle angle = a.angleTo(b);
        assertEquals(Math.PI / 4, angle.measure, DELTA);
    }

    // ==================== angleTo(FieldCoordinate) Tests ====================

    @Test
    public void angleTo_fieldCoordinate_firstQuadrant()
    {
        Pose2d pose = new Pose2d(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER),
            new Angle(0.0, AngleUnit.DEGREES)
        );
        FieldCoordinate coord = new FieldCoordinate(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(1.0, DistanceUnit.METER),
            CoordinateSystem.DECODE_PEDROPATH
        );

        Angle angle = pose.angleTo(coord);
        assertEquals(Math.PI / 4, angle.measure, DELTA);
    }

    // ==================== bearingTo(Pose2d) Tests ====================

    @Test
    public void bearingTo_pose_sameHeading()
    {
        Pose2d a = new Pose2d(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER),
            new Angle(0.0, AngleUnit.RADIANS)
        );
        Pose2d b = new Pose2d(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER),
            new Angle(0.0, AngleUnit.RADIANS)
        );

        // Target is at 0 radians, pose heading is 0, so relative bearing is 0
        Angle bearing = a.bearingTo(b);
        assertEquals(0.0, bearing.measure, DELTA);
    }

    @Test
    public void bearingTo_pose_targetToTheLeft()
    {
        Pose2d pose = new Pose2d(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER),
            new Angle(0.0, AngleUnit.RADIANS)
        );
        Pose2d target = new Pose2d(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(1.0, DistanceUnit.METER),
            new Angle(0.0, AngleUnit.RADIANS)
        );

        // Target is at 90 degrees (PI/2), pose heading is 0, so relative bearing is PI/2
        Angle bearing = pose.bearingTo(target);
        assertEquals(Math.PI / 2, bearing.measure, DELTA);
    }

    @Test
    public void bearingTo_pose_withNonZeroHeading()
    {
        Pose2d pose = new Pose2d(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER),
            new Angle(Math.PI / 4, AngleUnit.RADIANS) // 45 degrees heading
        );
        Pose2d target = new Pose2d(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(1.0, DistanceUnit.METER),
            new Angle(0.0, AngleUnit.RADIANS)
        );

        // Absolute angle to target is PI/4 (45 degrees)
        // Pose heading is PI/4, so relative bearing is 0
        Angle bearing = pose.bearingTo(target);
        assertEquals(0.0, bearing.measure, DELTA);
    }

    @Test
    public void bearingTo_pose_targetBehind()
    {
        Pose2d pose = new Pose2d(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER),
            new Angle(0.0, AngleUnit.RADIANS)
        );
        Pose2d target = new Pose2d(
            new Distance(-1.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER),
            new Angle(0.0, AngleUnit.RADIANS)
        );

        // Target is at PI radians (180 degrees), pose heading is 0
        // Relative bearing should be PI or -PI
        Angle bearing = pose.bearingTo(target);
        assertTrue(Math.abs(bearing.measure - Math.PI) < DELTA || Math.abs(bearing.measure + Math.PI) < DELTA);
    }

    // ==================== bearingTo(FieldCoordinate) Tests ====================

    @Test
    public void bearingTo_fieldCoordinate_targetStraightAhead()
    {
        Pose2d pose = new Pose2d(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER),
            new Angle(0.0, AngleUnit.RADIANS)
        );
        FieldCoordinate target = new FieldCoordinate(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER),
            CoordinateSystem.DECODE_PEDROPATH
        );

        Angle bearing = pose.bearingTo(target);
        assertEquals(0.0, bearing.measure, DELTA);
    }

    @Test
    public void bearingTo_fieldCoordinate_targetToRight()
    {
        Pose2d pose = new Pose2d(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER),
            new Angle(0.0, AngleUnit.RADIANS)
        );
        FieldCoordinate target = new FieldCoordinate(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(-1.0, DistanceUnit.METER),
            CoordinateSystem.DECODE_PEDROPATH
        );

        // Target is at -90 degrees (-PI/2)
        Angle bearing = pose.bearingTo(target);
        assertEquals(-Math.PI / 2, bearing.measure, DELTA);
    }

    // ==================== toPose2D() Tests ====================

    @Test
    public void toPose2D_convertsToFtcStandard()
    {
        Pose2d pose = new Pose2d(
            new Distance(72.0, DistanceUnit.INCH),
            new Distance(72.0, DistanceUnit.INCH),
            new Angle(45.0, AngleUnit.DEGREES)
        );

        org.firstinspires.ftc.robotcore.external.navigation.Pose2D ftcPose = pose.toPose2D();

        // (72, 72) in RIGHT_HAND = (0, 0) in FTC_STD
        assertEquals(0.0, ftcPose.getX(DistanceUnit.METER), DELTA);
        assertEquals(0.0, ftcPose.getY(DistanceUnit.METER), DELTA);
    }

    @Test
    public void toPose2D_convertsHeadingToRadians()
    {
        Pose2d pose = new Pose2d(
            new Distance(72.0, DistanceUnit.INCH),
            new Distance(72.0, DistanceUnit.INCH),
            new Angle(90.0, AngleUnit.DEGREES)
        );

        org.firstinspires.ftc.robotcore.external.navigation.Pose2D ftcPose = pose.toPose2D();

        assertEquals(Math.PI / 2, ftcPose.getHeading(AngleUnit.RADIANS), DELTA);
    }

    // ==================== fromPose3D() Tests ====================

    @Test
    public void fromPose3D_createsCorrectPose()
    {
        org.firstinspires.ftc.robotcore.external.navigation.Position position =
            new org.firstinspires.ftc.robotcore.external.navigation.Position(
                DistanceUnit.METER, 1.0, 2.0, 0.0, 0
            );
        org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles orientation =
            new org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles(
                AngleUnit.RADIANS, Math.PI / 4, 0.0, 0.0, 0
            );
        org.firstinspires.ftc.robotcore.external.navigation.Pose3D pose3D =
            new org.firstinspires.ftc.robotcore.external.navigation.Pose3D(position, orientation);

        Pose2d pose2d = Pose2d.fromPose3D(pose3D, CoordinateSystem.DECODE_FTC);

        assertEquals(1.0, pose2d.coord.x.magnitude, DELTA);
        assertEquals(2.0, pose2d.coord.y.magnitude, DELTA);
        assertEquals(Math.PI / 4, pose2d.heading.measure, DELTA);
        assertEquals(CoordinateSystem.DECODE_FTC, pose2d.coord.coordSys);
    }

    // ==================== Edge Cases ====================

    @Test
    public void edgeCase_chainedConversions()
    {
        Pose2d original = new Pose2d(
            new Distance(36.0, DistanceUnit.INCH),
            new Distance(48.0, DistanceUnit.INCH),
            new Angle(45.0, AngleUnit.DEGREES)
        );

        Pose2d result = original
            .toDistanceUnit(DistanceUnit.CM)
            .toAngleUnit(AngleUnit.RADIANS)
            .toCoordinateSystem(CoordinateSystem.DECODE_FTC)
            .toDistanceUnit(DistanceUnit.METER)
            .toAngleUnit(AngleUnit.DEGREES)
            .toCoordinateSystem(CoordinateSystem.DECODE_PEDROPATH)
            .toDistanceUnit(DistanceUnit.INCH);

        assertEquals(36.0, result.coord.x.magnitude, 1e-6);
        assertEquals(48.0, result.coord.y.magnitude, 1e-6);
        assertEquals(45.0, result.heading.measure, 1e-6);
    }

    @Test
    public void edgeCase_fullRotationHeading()
    {
        Pose2d pose = new Pose2d(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(2.0, DistanceUnit.METER),
            new Angle(360.0, AngleUnit.DEGREES)
        );

        assertEquals(0.0, pose.heading.measure, DELTA);
    }

    @Test
    public void edgeCase_negativeHeading()
    {
        Pose2d pose = new Pose2d(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(2.0, DistanceUnit.METER),
            new Angle(-90.0, AngleUnit.DEGREES)
        );

        assertEquals(-90.0, pose.heading.measure, DELTA);
    }

    @Test
    public void edgeCase_bearingSymmetry()
    {
        Pose2d pose = new Pose2d(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER),
            new Angle(Math.PI / 4, AngleUnit.RADIANS)
        );

        // Target directly in front of the pose's heading
        FieldCoordinate targetAhead = new FieldCoordinate(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(1.0, DistanceUnit.METER),
            CoordinateSystem.DECODE_PEDROPATH
        );

        Angle bearing = pose.bearingTo(targetAhead);
        assertEquals(0.0, bearing.measure, DELTA);
    }
}

