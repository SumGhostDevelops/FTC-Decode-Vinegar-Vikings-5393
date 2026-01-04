package org.firstinspires.ftc.teamcode.util.measure.coordinate;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
import org.firstinspires.ftc.teamcode.util.measure.angle.Vector2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.junit.Test;
import static org.junit.Assert.*;

/**
 * Comprehensive unit tests for the {@link FieldCoordinate} class.
 */
public class FieldCoordinateUnitTests
{
    private static final double DELTA = 1e-9;

    // ==================== Constructor Tests ====================

    @Test
    public void constructor_twoArgs_defaultsToRightHand()
    {
        Distance x = new Distance(3.0, DistanceUnit.METER);
        Distance y = new Distance(4.0, DistanceUnit.METER);
        FieldCoordinate coord = new FieldCoordinate(x, y);

        assertEquals(3.0, coord.x.magnitude, DELTA);
        assertEquals(4.0, coord.y.magnitude, DELTA);
        assertEquals(FieldCoordinate.CoordinateSystem.RIGHT_HAND, coord.coordSys);
    }

    @Test
    public void constructor_threeArgs_rightHand()
    {
        Distance x = new Distance(3.0, DistanceUnit.METER);
        Distance y = new Distance(4.0, DistanceUnit.METER);
        FieldCoordinate coord = new FieldCoordinate(x, y, FieldCoordinate.CoordinateSystem.RIGHT_HAND);

        assertEquals(3.0, coord.x.magnitude, DELTA);
        assertEquals(4.0, coord.y.magnitude, DELTA);
        assertEquals(FieldCoordinate.CoordinateSystem.RIGHT_HAND, coord.coordSys);
    }

    @Test
    public void constructor_threeArgs_ftcStd()
    {
        Distance x = new Distance(3.0, DistanceUnit.METER);
        Distance y = new Distance(4.0, DistanceUnit.METER);
        FieldCoordinate coord = new FieldCoordinate(x, y, FieldCoordinate.CoordinateSystem.FTC_STD);

        assertEquals(3.0, coord.x.magnitude, DELTA);
        assertEquals(4.0, coord.y.magnitude, DELTA);
        assertEquals(FieldCoordinate.CoordinateSystem.FTC_STD, coord.coordSys);
    }

    @Test
    public void constructor_zeroCoordinate()
    {
        Distance x = new Distance(0.0, DistanceUnit.METER);
        Distance y = new Distance(0.0, DistanceUnit.METER);
        FieldCoordinate coord = new FieldCoordinate(x, y);

        assertEquals(0.0, coord.x.magnitude, DELTA);
        assertEquals(0.0, coord.y.magnitude, DELTA);
    }

    @Test
    public void constructor_negativeCoordinates()
    {
        Distance x = new Distance(-3.0, DistanceUnit.METER);
        Distance y = new Distance(-4.0, DistanceUnit.METER);
        FieldCoordinate coord = new FieldCoordinate(x, y);

        assertEquals(-3.0, coord.x.magnitude, DELTA);
        assertEquals(-4.0, coord.y.magnitude, DELTA);
    }

    // ==================== toDistanceUnit() Tests ====================

    @Test
    public void toDistanceUnit_sameUnit_returnsSameInstance()
    {
        FieldCoordinate coord = new FieldCoordinate(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(2.0, DistanceUnit.METER)
        );

        FieldCoordinate converted = coord.toDistanceUnit(DistanceUnit.METER);
        assertSame(coord, converted);
    }

    @Test
    public void toDistanceUnit_meterToCm()
    {
        FieldCoordinate coord = new FieldCoordinate(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(2.0, DistanceUnit.METER)
        );

        FieldCoordinate converted = coord.toDistanceUnit(DistanceUnit.CM);
        assertEquals(100.0, converted.x.magnitude, DELTA);
        assertEquals(200.0, converted.y.magnitude, DELTA);
        assertEquals(DistanceUnit.CM, converted.x.unit);
        assertEquals(DistanceUnit.CM, converted.y.unit);
    }

    @Test
    public void toDistanceUnit_preservesCoordinateSystem()
    {
        FieldCoordinate coord = new FieldCoordinate(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(2.0, DistanceUnit.METER),
            FieldCoordinate.CoordinateSystem.FTC_STD
        );

        FieldCoordinate converted = coord.toDistanceUnit(DistanceUnit.CM);
        assertEquals(FieldCoordinate.CoordinateSystem.FTC_STD, converted.coordSys);
    }

    // ==================== toCoordinateSystem() Tests ====================

    @Test
    public void toCoordinateSystem_sameSystem_returnsSameInstance()
    {
        FieldCoordinate coord = new FieldCoordinate(
            new Distance(0.0, DistanceUnit.INCH),
            new Distance(0.0, DistanceUnit.INCH),
            FieldCoordinate.CoordinateSystem.RIGHT_HAND
        );

        FieldCoordinate converted = coord.toCoordinateSystem(FieldCoordinate.CoordinateSystem.RIGHT_HAND);
        assertSame(coord, converted);
    }

    @Test
    public void toCoordinateSystem_rightHandToFtcStd()
    {
        // In RIGHT_HAND, (72, 72) inches is the center of the field
        // In FTC_STD, that should be (0, 0)
        FieldCoordinate coord = new FieldCoordinate(
            new Distance(72.0, DistanceUnit.INCH),
            new Distance(72.0, DistanceUnit.INCH),
            FieldCoordinate.CoordinateSystem.RIGHT_HAND
        );

        FieldCoordinate converted = coord.toCoordinateSystem(FieldCoordinate.CoordinateSystem.FTC_STD);
        assertEquals(0.0, converted.x.magnitude, DELTA);
        assertEquals(0.0, converted.y.magnitude, DELTA);
        assertEquals(FieldCoordinate.CoordinateSystem.FTC_STD, converted.coordSys);
    }

    @Test
    public void toCoordinateSystem_ftcStdToRightHand()
    {
        // In FTC_STD, (0, 0) is the center of the field
        // In RIGHT_HAND, that should be (72, 72) inches
        FieldCoordinate coord = new FieldCoordinate(
            new Distance(0.0, DistanceUnit.INCH),
            new Distance(0.0, DistanceUnit.INCH),
            FieldCoordinate.CoordinateSystem.FTC_STD
        );

        FieldCoordinate converted = coord.toCoordinateSystem(FieldCoordinate.CoordinateSystem.RIGHT_HAND);
        assertEquals(72.0, converted.x.magnitude, DELTA);
        assertEquals(72.0, converted.y.magnitude, DELTA);
        assertEquals(FieldCoordinate.CoordinateSystem.RIGHT_HAND, converted.coordSys);
    }

    @Test
    public void toCoordinateSystem_roundTrip()
    {
        FieldCoordinate original = new FieldCoordinate(
            new Distance(36.0, DistanceUnit.INCH),
            new Distance(48.0, DistanceUnit.INCH),
            FieldCoordinate.CoordinateSystem.RIGHT_HAND
        );

        FieldCoordinate roundTrip = original
            .toCoordinateSystem(FieldCoordinate.CoordinateSystem.FTC_STD)
            .toCoordinateSystem(FieldCoordinate.CoordinateSystem.RIGHT_HAND);

        assertEquals(36.0, roundTrip.x.magnitude, DELTA);
        assertEquals(48.0, roundTrip.y.magnitude, DELTA);
    }

    @Test
    public void toCoordinateSystem_preservesDistanceUnit()
    {
        FieldCoordinate coord = new FieldCoordinate(
            new Distance(100.0, DistanceUnit.CM),
            new Distance(200.0, DistanceUnit.CM),
            FieldCoordinate.CoordinateSystem.RIGHT_HAND
        );

        FieldCoordinate converted = coord.toCoordinateSystem(FieldCoordinate.CoordinateSystem.FTC_STD);
        assertEquals(DistanceUnit.CM, converted.x.unit);
        assertEquals(DistanceUnit.CM, converted.y.unit);
    }

    // ==================== translate() Tests ====================

    @Test
    public void translate_positive()
    {
        FieldCoordinate coord = new FieldCoordinate(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(2.0, DistanceUnit.METER)
        );
        Vector2d translation = new Vector2d(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER)
        );

        FieldCoordinate translated = coord.translate(translation);
        assertEquals(4.0, translated.x.magnitude, DELTA);
        assertEquals(6.0, translated.y.magnitude, DELTA);
    }

    @Test
    public void translate_preservesCoordinateSystem()
    {
        FieldCoordinate coord = new FieldCoordinate(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(2.0, DistanceUnit.METER),
            FieldCoordinate.CoordinateSystem.FTC_STD
        );
        Vector2d translation = new Vector2d(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(1.0, DistanceUnit.METER)
        );

        FieldCoordinate translated = coord.translate(translation);
        assertEquals(FieldCoordinate.CoordinateSystem.FTC_STD, translated.coordSys);
    }

    @Test
    public void translate_negative()
    {
        FieldCoordinate coord = new FieldCoordinate(
            new Distance(5.0, DistanceUnit.METER),
            new Distance(5.0, DistanceUnit.METER)
        );
        Vector2d translation = new Vector2d(
            new Distance(-2.0, DistanceUnit.METER),
            new Distance(-3.0, DistanceUnit.METER)
        );

        FieldCoordinate translated = coord.translate(translation);
        assertEquals(3.0, translated.x.magnitude, DELTA);
        assertEquals(2.0, translated.y.magnitude, DELTA);
    }

    // ==================== distanceTo() Tests ====================

    @Test
    public void distanceTo_sameCoordinateSystem()
    {
        FieldCoordinate a = new FieldCoordinate(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER),
            FieldCoordinate.CoordinateSystem.RIGHT_HAND
        );
        FieldCoordinate b = new FieldCoordinate(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER),
            FieldCoordinate.CoordinateSystem.RIGHT_HAND
        );

        Distance distance = a.distanceTo(b);
        assertEquals(5.0, distance.magnitude, DELTA);
    }

    @Test
    public void distanceTo_differentCoordinateSystems()
    {
        // (72, 72) in RIGHT_HAND is (0, 0) in FTC_STD
        // So distance from (72, 72) RIGHT_HAND to (0, 0) FTC_STD should be 0
        FieldCoordinate a = new FieldCoordinate(
            new Distance(72.0, DistanceUnit.INCH),
            new Distance(72.0, DistanceUnit.INCH),
            FieldCoordinate.CoordinateSystem.RIGHT_HAND
        );
        FieldCoordinate b = new FieldCoordinate(
            new Distance(0.0, DistanceUnit.INCH),
            new Distance(0.0, DistanceUnit.INCH),
            FieldCoordinate.CoordinateSystem.FTC_STD
        );

        Distance distance = a.distanceTo(b);
        assertEquals(0.0, distance.magnitude, DELTA);
    }

    @Test
    public void distanceTo_symmetricAcrossCoordinateSystems()
    {
        FieldCoordinate a = new FieldCoordinate(
            new Distance(10.0, DistanceUnit.INCH),
            new Distance(10.0, DistanceUnit.INCH),
            FieldCoordinate.CoordinateSystem.RIGHT_HAND
        );
        FieldCoordinate b = new FieldCoordinate(
            new Distance(-50.0, DistanceUnit.INCH),
            new Distance(-50.0, DistanceUnit.INCH),
            FieldCoordinate.CoordinateSystem.FTC_STD
        );

        Distance distanceAB = a.distanceTo(b);
        Distance distanceBA = b.distanceTo(a);
        assertEquals(distanceAB.magnitude, distanceBA.magnitude, DELTA);
    }

    // ==================== angleTo() Tests ====================

    @Test
    public void angleTo_sameCoordinateSystem()
    {
        FieldCoordinate origin = new FieldCoordinate(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER),
            FieldCoordinate.CoordinateSystem.RIGHT_HAND
        );
        FieldCoordinate other = new FieldCoordinate(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(1.0, DistanceUnit.METER),
            FieldCoordinate.CoordinateSystem.RIGHT_HAND
        );

        Angle angle = origin.angleTo(other);
        assertEquals(Math.PI / 4, angle.measure, DELTA);
    }

    @Test
    public void angleTo_differentCoordinateSystems()
    {
        FieldCoordinate origin = new FieldCoordinate(
            new Distance(72.0, DistanceUnit.INCH),
            new Distance(72.0, DistanceUnit.INCH),
            FieldCoordinate.CoordinateSystem.RIGHT_HAND
        );
        // (1, 1) in FTC_STD = (73, 73) in RIGHT_HAND
        FieldCoordinate other = new FieldCoordinate(
            new Distance(1.0, DistanceUnit.INCH),
            new Distance(1.0, DistanceUnit.INCH),
            FieldCoordinate.CoordinateSystem.FTC_STD
        );

        Angle angle = origin.angleTo(other);
        assertEquals(Math.PI / 4, angle.measure, DELTA);
    }

    // ==================== isCoordinateSystem() Tests ====================

    @Test
    public void isCoordinateSystem_sameSystem_returnsTrue()
    {
        FieldCoordinate coord = new FieldCoordinate(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(2.0, DistanceUnit.METER),
            FieldCoordinate.CoordinateSystem.RIGHT_HAND
        );

        assertTrue(coord.isCoordinateSystem(FieldCoordinate.CoordinateSystem.RIGHT_HAND));
    }

    @Test
    public void isCoordinateSystem_differentSystem_returnsFalse()
    {
        FieldCoordinate coord = new FieldCoordinate(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(2.0, DistanceUnit.METER),
            FieldCoordinate.CoordinateSystem.RIGHT_HAND
        );

        assertFalse(coord.isCoordinateSystem(FieldCoordinate.CoordinateSystem.FTC_STD));
    }

    // ==================== equals() Tests ====================

    @Test
    public void equals_sameInstance_returnsTrue()
    {
        FieldCoordinate coord = new FieldCoordinate(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER)
        );

        assertTrue(coord.equals(coord));
    }

    @Test
    public void equals_equalCoordinates_sameSystem_returnsTrue()
    {
        FieldCoordinate a = new FieldCoordinate(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER),
            FieldCoordinate.CoordinateSystem.RIGHT_HAND
        );
        FieldCoordinate b = new FieldCoordinate(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER),
            FieldCoordinate.CoordinateSystem.RIGHT_HAND
        );

        assertTrue(a.equals(b));
    }

    @Test
    public void equals_samePosition_differentSystems_returnsTrue()
    {
        // (72, 72) in RIGHT_HAND equals (0, 0) in FTC_STD
        FieldCoordinate a = new FieldCoordinate(
            new Distance(72.0, DistanceUnit.INCH),
            new Distance(72.0, DistanceUnit.INCH),
            FieldCoordinate.CoordinateSystem.RIGHT_HAND
        );
        FieldCoordinate b = new FieldCoordinate(
            new Distance(0.0, DistanceUnit.INCH),
            new Distance(0.0, DistanceUnit.INCH),
            FieldCoordinate.CoordinateSystem.FTC_STD
        );

        assertTrue(a.equals(b));
    }

    @Test
    public void equals_differentUnits_samePosition_returnsTrue()
    {
        FieldCoordinate a = new FieldCoordinate(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(2.0, DistanceUnit.METER),
            FieldCoordinate.CoordinateSystem.RIGHT_HAND
        );
        FieldCoordinate b = new FieldCoordinate(
            new Distance(100.0, DistanceUnit.CM),
            new Distance(200.0, DistanceUnit.CM),
            FieldCoordinate.CoordinateSystem.RIGHT_HAND
        );

        assertTrue(a.equals(b));
    }

    @Test
    public void equals_differentPosition_returnsFalse()
    {
        FieldCoordinate a = new FieldCoordinate(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER)
        );
        FieldCoordinate b = new FieldCoordinate(
            new Distance(5.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER)
        );

        assertFalse(a.equals(b));
    }

    @Test
    public void equals_null_returnsFalse()
    {
        FieldCoordinate coord = new FieldCoordinate(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER)
        );

        assertFalse(coord.equals(null));
    }

    @Test
    public void equals_baseCoordinate_returnsFalse()
    {
        FieldCoordinate fieldCoord = new FieldCoordinate(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER)
        );
        Coordinate baseCoord = new Coordinate(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER)
        );

        // FieldCoordinate.equals requires the other object to be a FieldCoordinate
        assertFalse(fieldCoord.equals(baseCoord));
    }

    // ==================== Edge Cases ====================

    @Test
    public void edgeCase_originInBothSystems()
    {
        // Origin in RIGHT_HAND
        FieldCoordinate rightHandOrigin = new FieldCoordinate(
            new Distance(0.0, DistanceUnit.INCH),
            new Distance(0.0, DistanceUnit.INCH),
            FieldCoordinate.CoordinateSystem.RIGHT_HAND
        );

        // Convert to FTC_STD
        FieldCoordinate ftcStd = rightHandOrigin.toCoordinateSystem(FieldCoordinate.CoordinateSystem.FTC_STD);
        assertEquals(-72.0, ftcStd.x.magnitude, DELTA);
        assertEquals(-72.0, ftcStd.y.magnitude, DELTA);
    }

    @Test
    public void edgeCase_chainedConversions()
    {
        FieldCoordinate original = new FieldCoordinate(
            new Distance(36.0, DistanceUnit.INCH),
            new Distance(48.0, DistanceUnit.INCH),
            FieldCoordinate.CoordinateSystem.RIGHT_HAND
        );

        FieldCoordinate result = original
            .toDistanceUnit(DistanceUnit.CM)
            .toCoordinateSystem(FieldCoordinate.CoordinateSystem.FTC_STD)
            .toDistanceUnit(DistanceUnit.METER)
            .toCoordinateSystem(FieldCoordinate.CoordinateSystem.RIGHT_HAND)
            .toDistanceUnit(DistanceUnit.INCH);

        assertEquals(36.0, result.x.magnitude, 1e-6);
        assertEquals(48.0, result.y.magnitude, 1e-6);
    }

    @Test
    public void edgeCase_fieldBoundaries()
    {
        // Full field is 144 x 144 inches
        // In RIGHT_HAND, corners are (0,0), (144,0), (0,144), (144,144)
        // In FTC_STD, corners are (-72,-72), (72,-72), (-72,72), (72,72)

        FieldCoordinate rightHandCorner = new FieldCoordinate(
            new Distance(144.0, DistanceUnit.INCH),
            new Distance(144.0, DistanceUnit.INCH),
            FieldCoordinate.CoordinateSystem.RIGHT_HAND
        );

        FieldCoordinate ftcStdCorner = rightHandCorner.toCoordinateSystem(FieldCoordinate.CoordinateSystem.FTC_STD);
        assertEquals(72.0, ftcStdCorner.x.magnitude, DELTA);
        assertEquals(72.0, ftcStdCorner.y.magnitude, DELTA);
    }
}

