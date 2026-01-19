package org.firstinspires.ftc.teamcode.util.measure.coordinate;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.firstinspires.ftc.teamcode.util.measure.geometry.Vector2d;
import org.junit.Test;
import static org.junit.Assert.*;

/**
 * Comprehensive unit tests for the {@link Coordinate} class.
 */
public class CoordinateUnitTests
{
    private static final double DELTA = 1e-9;

    // ==================== Constructor Tests ====================

    @Test
    public void constructor_setsXAndY()
    {
        Distance x = new Distance(3.0, DistanceUnit.METER);
        Distance y = new Distance(4.0, DistanceUnit.METER);
        Coordinate coord = new Coordinate(x, y);
        
        assertEquals(3.0, coord.x.magnitude, DELTA);
        assertEquals(4.0, coord.y.magnitude, DELTA);
    }

    @Test
    public void constructor_differentUnits()
    {
        Distance x = new Distance(100.0, DistanceUnit.CM);
        Distance y = new Distance(2.0, DistanceUnit.METER);
        Coordinate coord = new Coordinate(x, y);
        
        assertEquals(100.0, coord.x.magnitude, DELTA);
        assertEquals(DistanceUnit.CM, coord.x.unit);
        assertEquals(2.0, coord.y.magnitude, DELTA);
        assertEquals(DistanceUnit.METER, coord.y.unit);
    }

    @Test
    public void constructor_zeroCoordinate()
    {
        Distance x = new Distance(0.0, DistanceUnit.METER);
        Distance y = new Distance(0.0, DistanceUnit.METER);
        Coordinate coord = new Coordinate(x, y);
        
        assertEquals(0.0, coord.x.magnitude, DELTA);
        assertEquals(0.0, coord.y.magnitude, DELTA);
    }

    @Test
    public void constructor_negativeCoordinates()
    {
        Distance x = new Distance(-3.0, DistanceUnit.METER);
        Distance y = new Distance(-4.0, DistanceUnit.METER);
        Coordinate coord = new Coordinate(x, y);
        
        assertEquals(-3.0, coord.x.magnitude, DELTA);
        assertEquals(-4.0, coord.y.magnitude, DELTA);
    }

    // ==================== distanceTo() Tests ====================

    @Test
    public void distanceTo_sameCoordinate_returnsZero()
    {
        Distance x = new Distance(3.0, DistanceUnit.METER);
        Distance y = new Distance(4.0, DistanceUnit.METER);
        Coordinate coord = new Coordinate(x, y);
        
        Distance distance = coord.distanceTo(coord);
        assertEquals(0.0, distance.magnitude, DELTA);
    }

    @Test
    public void distanceTo_345Triangle()
    {
        Coordinate origin = new Coordinate(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER)
        );
        Coordinate other = new Coordinate(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER)
        );
        
        Distance distance = origin.distanceTo(other);
        assertEquals(5.0, distance.magnitude, DELTA);
    }

    @Test
    public void distanceTo_horizontalDistance()
    {
        Coordinate a = new Coordinate(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER)
        );
        Coordinate b = new Coordinate(
            new Distance(10.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER)
        );
        
        Distance distance = a.distanceTo(b);
        assertEquals(10.0, distance.magnitude, DELTA);
    }

    @Test
    public void distanceTo_verticalDistance()
    {
        Coordinate a = new Coordinate(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER)
        );
        Coordinate b = new Coordinate(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(10.0, DistanceUnit.METER)
        );
        
        Distance distance = a.distanceTo(b);
        assertEquals(10.0, distance.magnitude, DELTA);
    }

    @Test
    public void distanceTo_differentUnits()
    {
        Coordinate a = new Coordinate(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER)
        );
        Coordinate b = new Coordinate(
            new Distance(300.0, DistanceUnit.CM),
            new Distance(400.0, DistanceUnit.CM)
        );
        
        Distance distance = a.distanceTo(b);
        assertEquals(5.0, distance.magnitude, DELTA);
        assertEquals(DistanceUnit.METER, distance.unit);
    }

    @Test
    public void distanceTo_negativeCoordinates()
    {
        Coordinate a = new Coordinate(
            new Distance(-3.0, DistanceUnit.METER),
            new Distance(-4.0, DistanceUnit.METER)
        );
        Coordinate b = new Coordinate(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER)
        );
        
        Distance distance = a.distanceTo(b);
        assertEquals(5.0, distance.magnitude, DELTA);
    }

    @Test
    public void distanceTo_isSymmetric()
    {
        Coordinate a = new Coordinate(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(2.0, DistanceUnit.METER)
        );
        Coordinate b = new Coordinate(
            new Distance(4.0, DistanceUnit.METER),
            new Distance(6.0, DistanceUnit.METER)
        );
        
        Distance distanceAB = a.distanceTo(b);
        Distance distanceBA = b.distanceTo(a);
        assertEquals(distanceAB.magnitude, distanceBA.magnitude, DELTA);
    }

    // ==================== angleTo() Tests ====================

    @Test
    public void angleTo_positiveXAxis()
    {
        Coordinate origin = new Coordinate(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER)
        );
        Coordinate other = new Coordinate(
            new Distance(5.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER)
        );
        
        Angle angle = origin.angleTo(other);
        assertEquals(0.0, angle.measure, DELTA);
    }

    @Test
    public void angleTo_positiveYAxis()
    {
        Coordinate origin = new Coordinate(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER)
        );
        Coordinate other = new Coordinate(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(5.0, DistanceUnit.METER)
        );
        
        Angle angle = origin.angleTo(other);
        assertEquals(Math.PI / 2, angle.measure, DELTA);
    }

    @Test
    public void angleTo_negativeXAxis()
    {
        Coordinate origin = new Coordinate(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER)
        );
        Coordinate other = new Coordinate(
            new Distance(-5.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER)
        );
        
        Angle angle = origin.angleTo(other);
        assertTrue(Math.abs(angle.measure - Math.PI) < DELTA || Math.abs(angle.measure + Math.PI) < DELTA);
    }

    @Test
    public void angleTo_negativeYAxis()
    {
        Coordinate origin = new Coordinate(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER)
        );
        Coordinate other = new Coordinate(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(-5.0, DistanceUnit.METER)
        );
        
        Angle angle = origin.angleTo(other);
        assertEquals(-Math.PI / 2, angle.measure, DELTA);
    }

    @Test
    public void angleTo_firstQuadrant()
    {
        Coordinate origin = new Coordinate(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER)
        );
        Coordinate other = new Coordinate(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(1.0, DistanceUnit.METER)
        );
        
        Angle angle = origin.angleTo(other);
        assertEquals(Math.PI / 4, angle.measure, DELTA);
    }

    @Test
    public void angleTo_differentUnits()
    {
        Coordinate origin = new Coordinate(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER)
        );
        Coordinate other = new Coordinate(
            new Distance(100.0, DistanceUnit.CM),
            new Distance(100.0, DistanceUnit.CM)
        );
        
        Angle angle = origin.angleTo(other);
        assertEquals(Math.PI / 4, angle.measure, DELTA);
    }

    // ==================== vectorTo() Tests ====================

    @Test
    public void vectorTo_sameCoordinate_returnsZeroVector()
    {
        Coordinate coord = new Coordinate(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER)
        );
        
        Vector2d vector = coord.vectorTo(coord);
        assertEquals(0.0, vector.x.magnitude, DELTA);
        assertEquals(0.0, vector.y.magnitude, DELTA);
    }

    @Test
    public void vectorTo_positive()
    {
        Coordinate a = new Coordinate(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(2.0, DistanceUnit.METER)
        );
        Coordinate b = new Coordinate(
            new Distance(4.0, DistanceUnit.METER),
            new Distance(6.0, DistanceUnit.METER)
        );
        
        Vector2d vector = a.vectorTo(b);
        assertEquals(3.0, vector.x.magnitude, DELTA);
        assertEquals(4.0, vector.y.magnitude, DELTA);
    }

    @Test
    public void vectorTo_negative()
    {
        Coordinate a = new Coordinate(
            new Distance(4.0, DistanceUnit.METER),
            new Distance(6.0, DistanceUnit.METER)
        );
        Coordinate b = new Coordinate(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(2.0, DistanceUnit.METER)
        );
        
        Vector2d vector = a.vectorTo(b);
        assertEquals(-3.0, vector.x.magnitude, DELTA);
        assertEquals(-4.0, vector.y.magnitude, DELTA);
    }

    @Test
    public void vectorTo_differentUnits()
    {
        Coordinate a = new Coordinate(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER)
        );
        Coordinate b = new Coordinate(
            new Distance(100.0, DistanceUnit.CM),
            new Distance(200.0, DistanceUnit.CM)
        );
        
        Vector2d vector = a.vectorTo(b);
        assertEquals(1.0, vector.x.magnitude, DELTA);
        assertEquals(2.0, vector.y.magnitude, DELTA);
        assertEquals(DistanceUnit.METER, vector.x.unit);
    }

    // ==================== toDistanceUnit() Tests ====================

    @Test
    public void toDistanceUnit_meterToCm()
    {
        Coordinate coord = new Coordinate(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(2.0, DistanceUnit.METER)
        );
        
        Coordinate converted = coord.toDistanceUnit(DistanceUnit.CM);
        assertEquals(100.0, converted.x.magnitude, DELTA);
        assertEquals(200.0, converted.y.magnitude, DELTA);
        assertEquals(DistanceUnit.CM, converted.x.unit);
        assertEquals(DistanceUnit.CM, converted.y.unit);
    }

    @Test
    public void toDistanceUnit_cmToMeter()
    {
        Coordinate coord = new Coordinate(
            new Distance(100.0, DistanceUnit.CM),
            new Distance(200.0, DistanceUnit.CM)
        );
        
        Coordinate converted = coord.toDistanceUnit(DistanceUnit.METER);
        assertEquals(1.0, converted.x.magnitude, DELTA);
        assertEquals(2.0, converted.y.magnitude, DELTA);
        assertEquals(DistanceUnit.METER, converted.x.unit);
        assertEquals(DistanceUnit.METER, converted.y.unit);
    }

    @Test
    public void toDistanceUnit_mixedUnits()
    {
        Coordinate coord = new Coordinate(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(50.0, DistanceUnit.CM)
        );
        
        Coordinate converted = coord.toDistanceUnit(DistanceUnit.MM);
        assertEquals(1000.0, converted.x.magnitude, DELTA);
        assertEquals(500.0, converted.y.magnitude, DELTA);
        assertEquals(DistanceUnit.MM, converted.x.unit);
        assertEquals(DistanceUnit.MM, converted.y.unit);
    }

    // ==================== translate() Tests ====================

    @Test
    public void translate_positive()
    {
        Coordinate coord = new Coordinate(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(2.0, DistanceUnit.METER)
        );
        Vector2d translation = new Vector2d(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER),
            CoordinateSystem.DECODE_FTC
        );
        
        Coordinate translated = coord.translate(translation);
        assertEquals(4.0, translated.x.magnitude, DELTA);
        assertEquals(6.0, translated.y.magnitude, DELTA);
    }

    @Test
    public void translate_negative()
    {
        Coordinate coord = new Coordinate(
            new Distance(5.0, DistanceUnit.METER),
            new Distance(5.0, DistanceUnit.METER)
        );
        Vector2d translation = new Vector2d(
            new Distance(-2.0, DistanceUnit.METER),
            new Distance(-3.0, DistanceUnit.METER),
            CoordinateSystem.DECODE_FTC
        );
        
        Coordinate translated = coord.translate(translation);
        assertEquals(3.0, translated.x.magnitude, DELTA);
        assertEquals(2.0, translated.y.magnitude, DELTA);
    }

    @Test
    public void translate_zeroVector()
    {
        Coordinate coord = new Coordinate(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER)
        );
        Vector2d translation = new Vector2d(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER),
            CoordinateSystem.DECODE_FTC
        );
        
        Coordinate translated = coord.translate(translation);
        assertEquals(3.0, translated.x.magnitude, DELTA);
        assertEquals(4.0, translated.y.magnitude, DELTA);
    }

    @Test
    public void translate_differentUnits()
    {
        Coordinate coord = new Coordinate(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(1.0, DistanceUnit.METER)
        );
        Vector2d translation = new Vector2d(
            new Distance(100.0, DistanceUnit.CM),
            new Distance(200.0, DistanceUnit.CM),
            CoordinateSystem.DECODE_FTC
        );
        
        Coordinate translated = coord.translate(translation);
        assertEquals(2.0, translated.x.magnitude, DELTA);
        assertEquals(3.0, translated.y.magnitude, DELTA);
    }

    // ==================== isDistanceUnit() Tests ====================

    @Test
    public void isDistanceUnit_sameUnit_returnsTrue()
    {
        Coordinate coord = new Coordinate(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(2.0, DistanceUnit.METER)
        );
        
        assertTrue(coord.isDistanceUnit(DistanceUnit.METER));
    }

    @Test
    public void isDistanceUnit_differentUnit_returnsFalse()
    {
        Coordinate coord = new Coordinate(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(2.0, DistanceUnit.METER)
        );
        
        assertFalse(coord.isDistanceUnit(DistanceUnit.CM));
    }

    @Test
    public void isDistanceUnit_mixedUnits_returnsFalse()
    {
        Coordinate coord = new Coordinate(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(200.0, DistanceUnit.CM)
        );
        
        assertFalse(coord.isDistanceUnit(DistanceUnit.METER));
        assertFalse(coord.isDistanceUnit(DistanceUnit.CM));
    }

    // ==================== equals() Tests ====================

    @Test
    public void equals_sameInstance_returnsTrue()
    {
        Coordinate coord = new Coordinate(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER)
        );
        
        assertTrue(coord.equals(coord));
    }

    @Test
    public void equals_equalCoordinates_returnsTrue()
    {
        Coordinate a = new Coordinate(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER)
        );
        Coordinate b = new Coordinate(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER)
        );
        
        assertTrue(a.equals(b));
    }

    @Test
    public void equals_differentUnits_samePosition_returnsTrue()
    {
        Coordinate a = new Coordinate(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(2.0, DistanceUnit.METER)
        );
        Coordinate b = new Coordinate(
            new Distance(100.0, DistanceUnit.CM),
            new Distance(200.0, DistanceUnit.CM)
        );
        
        assertTrue(a.equals(b));
    }

    @Test
    public void equals_differentX_returnsFalse()
    {
        Coordinate a = new Coordinate(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER)
        );
        Coordinate b = new Coordinate(
            new Distance(5.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER)
        );
        
        assertFalse(a.equals(b));
    }

    @Test
    public void equals_differentY_returnsFalse()
    {
        Coordinate a = new Coordinate(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER)
        );
        Coordinate b = new Coordinate(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(5.0, DistanceUnit.METER)
        );
        
        assertFalse(a.equals(b));
    }

    @Test
    public void equals_null_returnsFalse()
    {
        Coordinate coord = new Coordinate(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER)
        );
        
        assertFalse(coord.equals(null));
    }

    @Test
    public void equals_differentClass_returnsFalse()
    {
        Coordinate coord = new Coordinate(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER)
        );
        
        assertFalse(coord.equals("not a coordinate"));
    }

    @Test
    public void equals_zeroCoordinates_returnsTrue()
    {
        Coordinate a = new Coordinate(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER)
        );
        Coordinate b = new Coordinate(
            new Distance(0.0, DistanceUnit.CM),
            new Distance(0.0, DistanceUnit.CM)
        );
        
        assertTrue(a.equals(b));
    }

    // ==================== Edge Cases ====================

    @Test
    public void edgeCase_veryLargeCoordinates()
    {
        Coordinate a = new Coordinate(
            new Distance(1e10, DistanceUnit.METER),
            new Distance(1e10, DistanceUnit.METER)
        );
        Coordinate b = new Coordinate(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER)
        );
        
        Distance distance = a.distanceTo(b);
        assertEquals(Math.sqrt(2) * 1e10, distance.magnitude, 1e2);
    }

    @Test
    public void edgeCase_verySmallCoordinates()
    {
        Coordinate a = new Coordinate(
            new Distance(1e-10, DistanceUnit.METER),
            new Distance(1e-10, DistanceUnit.METER)
        );
        Coordinate b = new Coordinate(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER)
        );
        
        Distance distance = a.distanceTo(b);
        assertEquals(Math.sqrt(2) * 1e-10, distance.magnitude, 1e-18);
    }

    @Test
    public void edgeCase_chainedTranslations()
    {
        Coordinate coord = new Coordinate(
            new Distance(0.0, DistanceUnit.METER),
            new Distance(0.0, DistanceUnit.METER)
        );
        Vector2d translation1 = new Vector2d(
            new Distance(1.0, DistanceUnit.METER),
            new Distance(2.0, DistanceUnit.METER),
            CoordinateSystem.DECODE_FTC
        );
        Vector2d translation2 = new Vector2d(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER),
            CoordinateSystem.DECODE_FTC
        );
        
        Coordinate result = coord.translate(translation1).translate(translation2);
        assertEquals(4.0, result.x.magnitude, DELTA);
        assertEquals(6.0, result.y.magnitude, DELTA);
    }

    @Test
    public void edgeCase_translateAndBack()
    {
        Coordinate coord = new Coordinate(
            new Distance(5.0, DistanceUnit.METER),
            new Distance(5.0, DistanceUnit.METER)
        );
        Vector2d translation = new Vector2d(
            new Distance(3.0, DistanceUnit.METER),
            new Distance(4.0, DistanceUnit.METER),
            CoordinateSystem.DECODE_FTC
        );
        
        Coordinate result = coord.translate(translation).translate(translation.inverse());
        assertEquals(5.0, result.x.magnitude, DELTA);
        assertEquals(5.0, result.y.magnitude, DELTA);
    }
}

