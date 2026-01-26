package org.firstinspires.ftc.teamcode.util.measure.angle;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.CoordinateSystem;
import org.firstinspires.ftc.teamcode.util.measure.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.junit.Test;
import static org.junit.Assert.*;

/**
 * Comprehensive unit tests for the {@link Vector2d} class.
 */
public class Vector2dUnitTests
{
    private static final double DELTA = 1e-9;

    // ==================== Constructor Tests ====================

    @Test
    public void constructor_twoArgs_setsXYAndDefaultUnits()
    {
        Distance x = new Distance(3.0, DistanceUnit.METER);
        Distance y = new Distance(4.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        assertEquals(3.0, v.x.magnitude, DELTA);
        assertEquals(4.0, v.y.magnitude, DELTA);
        assertEquals(DistanceUnit.METER, v.distUnit);
        assertEquals(AngleUnit.RADIANS, v.angUnit);
    }

    @Test
    public void constructor_threeArgs_distanceUnit()
    {
        Distance x = new Distance(3.0, DistanceUnit.METER);
        Distance y = new Distance(4.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);
        Vector2d converted = v.toDistanceUnit(DistanceUnit.CM);

        assertEquals(300.0, converted.x.magnitude, DELTA);
        assertEquals(400.0, converted.y.magnitude, DELTA);
        assertEquals(DistanceUnit.CM, converted.distUnit);
        assertEquals(AngleUnit.RADIANS, converted.angUnit);
    }

    @Test
    public void constructor_threeArgs_angleUnit()
    {
        Distance x = new Distance(3.0, DistanceUnit.METER);
        Distance y = new Distance(4.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        assertEquals(3.0, v.x.magnitude, DELTA);
        assertEquals(4.0, v.y.magnitude, DELTA);
        assertEquals(DistanceUnit.METER, v.distUnit);
        assertEquals(AngleUnit.RADIANS, v.angUnit);
    }

    @Test
    public void constructor_fourArgs()
    {
        Distance x = new Distance(3.0, DistanceUnit.METER);
        Distance y = new Distance(4.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC).toDistanceUnit(DistanceUnit.CM);

        assertEquals(300.0, v.x.magnitude, DELTA);
        assertEquals(400.0, v.y.magnitude, DELTA);
        assertEquals(DistanceUnit.CM, v.distUnit);
        assertEquals(AngleUnit.RADIANS, v.angUnit);
    }

    @Test
    public void constructor_convertsXYToSameUnit()
    {
        Distance x = new Distance(1.0, DistanceUnit.METER);
        Distance y = new Distance(200.0, DistanceUnit.CM);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC).toDistanceUnit(DistanceUnit.METER);

        assertEquals(1.0, v.x.magnitude, DELTA);
        assertEquals(2.0, v.y.magnitude, DELTA);
        assertEquals(DistanceUnit.METER, v.x.unit);
        assertEquals(DistanceUnit.METER, v.y.unit);
    }

    @Test
    public void constructor_zeroVector()
    {
        Distance x = new Distance(0.0, DistanceUnit.METER);
        Distance y = new Distance(0.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        assertEquals(0.0, v.x.magnitude, DELTA);
        assertEquals(0.0, v.y.magnitude, DELTA);
    }

    @Test
    public void constructor_negativeComponents()
    {
        Distance x = new Distance(-3.0, DistanceUnit.METER);
        Distance y = new Distance(-4.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        assertEquals(-3.0, v.x.magnitude, DELTA);
        assertEquals(-4.0, v.y.magnitude, DELTA);
    }

    // ==================== getDistance() Tests ====================

    @Test
    public void getLength_345Triangle()
    {
        Distance x = new Distance(3.0, DistanceUnit.METER);
        Distance y = new Distance(4.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        Distance distance = v.getLength();
        assertEquals(5.0, distance.magnitude, DELTA);
        assertEquals(DistanceUnit.METER, distance.unit);
    }

    @Test
    public void getLength_xOnly()
    {
        Distance x = new Distance(5.0, DistanceUnit.METER);
        Distance y = new Distance(0.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        Distance distance = v.getLength();
        assertEquals(5.0, distance.magnitude, DELTA);
    }

    @Test
    public void getLength_yOnly()
    {
        Distance x = new Distance(0.0, DistanceUnit.METER);
        Distance y = new Distance(5.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        Distance distance = v.getLength();
        assertEquals(5.0, distance.magnitude, DELTA);
    }

    @Test
    public void getLength_zeroVector()
    {
        Distance x = new Distance(0.0, DistanceUnit.METER);
        Distance y = new Distance(0.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        Distance distance = v.getLength();
        assertEquals(0.0, distance.magnitude, DELTA);
    }

    @Test
    public void getLength_negativeComponents()
    {
        Distance x = new Distance(-3.0, DistanceUnit.METER);
        Distance y = new Distance(-4.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        Distance distance = v.getLength();
        assertEquals(5.0, distance.magnitude, DELTA);
    }

    @Test
    public void getLength_differentUnits()
    {
        Distance x = new Distance(300.0, DistanceUnit.CM);
        Distance y = new Distance(400.0, DistanceUnit.CM);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        Distance distance = v.getLength();
        assertEquals(500.0, distance.magnitude, DELTA);
        assertEquals(DistanceUnit.CM, distance.unit);
    }

    // ==================== getDirection() Tests ====================

    @Test
    public void getDirection_firstQuadrant()
    {
        Distance x = new Distance(1.0, DistanceUnit.METER);
        Distance y = new Distance(1.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        Angle direction = v.getDirection();
        assertEquals(Math.PI / 4, direction.measure, DELTA);
        assertEquals(AngleUnit.RADIANS, direction.unit);
    }

    @Test
    public void getDirection_secondQuadrant()
    {
        Distance x = new Distance(-1.0, DistanceUnit.METER);
        Distance y = new Distance(1.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        Angle direction = v.getDirection();
        assertEquals(3 * Math.PI / 4, direction.measure, DELTA);
    }

    @Test
    public void getDirection_thirdQuadrant()
    {
        Distance x = new Distance(-1.0, DistanceUnit.METER);
        Distance y = new Distance(-1.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        Angle direction = v.getDirection();
        assertEquals(-3 * Math.PI / 4, direction.measure, DELTA);
    }

    @Test
    public void getDirection_fourthQuadrant()
    {
        Distance x = new Distance(1.0, DistanceUnit.METER);
        Distance y = new Distance(-1.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        Angle direction = v.getDirection();
        assertEquals(-Math.PI / 4, direction.measure, DELTA);
    }

    @Test
    public void getDirection_positiveXAxis()
    {
        Distance x = new Distance(5.0, DistanceUnit.METER);
        Distance y = new Distance(0.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        Angle direction = v.getDirection();
        assertEquals(0.0, direction.measure, DELTA);
    }

    @Test
    public void getDirection_positiveYAxis()
    {
        Distance x = new Distance(0.0, DistanceUnit.METER);
        Distance y = new Distance(5.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        Angle direction = v.getDirection();
        assertEquals(Math.PI / 2, direction.measure, DELTA);
    }

    @Test
    public void getDirection_negativeXAxis()
    {
        Distance x = new Distance(-5.0, DistanceUnit.METER);
        Distance y = new Distance(0.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        Angle direction = v.getDirection();
        assertTrue(Math.abs(direction.measure - Math.PI) < DELTA || Math.abs(direction.measure + Math.PI) < DELTA);
    }

    @Test
    public void getDirection_negativeYAxis()
    {
        Distance x = new Distance(0.0, DistanceUnit.METER);
        Distance y = new Distance(-5.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        Angle direction = v.getDirection();
        assertEquals(-Math.PI / 2, direction.measure, DELTA);
    }

    @Test
    public void getDirection_withDegreesUnit()
    {
        Distance x = new Distance(1.0, DistanceUnit.METER);
        Distance y = new Distance(1.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        Angle direction = v.getDirection();
        assertEquals(Math.PI / 4, direction.measure, DELTA);
        assertEquals(AngleUnit.RADIANS, direction.unit);
    }

    // ==================== toDistanceUnit() Tests ====================

    @Test
    public void toDistanceUnit_sameUnit_returnsSameInstance()
    {
        Distance x = new Distance(3.0, DistanceUnit.METER);
        Distance y = new Distance(4.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        Vector2d converted = v.toDistanceUnit(DistanceUnit.METER);
        assertSame(v, converted);
    }

    @Test
    public void toDistanceUnit_meterToCm()
    {
        Distance x = new Distance(1.0, DistanceUnit.METER);
        Distance y = new Distance(2.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        Vector2d converted = v.toDistanceUnit(DistanceUnit.CM);
        assertEquals(100.0, converted.x.magnitude, DELTA);
        assertEquals(200.0, converted.y.magnitude, DELTA);
        assertEquals(DistanceUnit.CM, converted.distUnit);
    }

    @Test
    public void toDistanceUnit_cmToMeter()
    {
        Distance x = new Distance(100.0, DistanceUnit.CM);
        Distance y = new Distance(200.0, DistanceUnit.CM);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        Vector2d converted = v.toDistanceUnit(DistanceUnit.METER);
        assertEquals(1.0, converted.x.magnitude, DELTA);
        assertEquals(2.0, converted.y.magnitude, DELTA);
        assertEquals(DistanceUnit.METER, converted.distUnit);
    }

    // ==================== inverse() Tests ====================

    @Test
    public void inverse_positiveComponents()
    {
        Distance x = new Distance(3.0, DistanceUnit.METER);
        Distance y = new Distance(4.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        Vector2d inverted = v.inverse();
        assertEquals(-3.0, inverted.x.magnitude, DELTA);
        assertEquals(-4.0, inverted.y.magnitude, DELTA);
    }

    @Test
    public void inverse_negativeComponents()
    {
        Distance x = new Distance(-3.0, DistanceUnit.METER);
        Distance y = new Distance(-4.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        Vector2d inverted = v.inverse();
        assertEquals(3.0, inverted.x.magnitude, DELTA);
        assertEquals(4.0, inverted.y.magnitude, DELTA);
    }

    @Test
    public void inverse_zeroVector()
    {
        Distance x = new Distance(0.0, DistanceUnit.METER);
        Distance y = new Distance(0.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        Vector2d inverted = v.inverse();
        assertEquals(0.0, inverted.x.magnitude, DELTA);
        assertEquals(0.0, inverted.y.magnitude, DELTA);
    }

    @Test
    public void inverse_preservesUnits()
    {
        Distance x = new Distance(3.0, DistanceUnit.CM);
        Distance y = new Distance(4.0, DistanceUnit.CM);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC).toDistanceUnit(DistanceUnit.CM);

        Vector2d inverted = v.inverse();
        assertEquals(DistanceUnit.CM, inverted.distUnit);
        assertEquals(AngleUnit.RADIANS, inverted.angUnit);
    }

    @Test
    public void inverse_doubleInverse_returnsOriginal()
    {
        Distance x = new Distance(3.0, DistanceUnit.METER);
        Distance y = new Distance(4.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        Vector2d inverted = v.inverse();
        Vector2d invertedAgain = inverted.inverse();

        assertEquals(v.x.magnitude, invertedAgain.x.magnitude, DELTA);
        assertEquals(v.y.magnitude, invertedAgain.y.magnitude, DELTA);
    }

    // ==================== plus() Tests ====================

    @Test
    public void plus_sameUnit()
    {
        Distance x1 = new Distance(1.0, DistanceUnit.METER);
        Distance y1 = new Distance(2.0, DistanceUnit.METER);
        Vector2d v1 = new Vector2d(x1, y1, CoordinateSystem.DECODE_FTC);

        Distance x2 = new Distance(3.0, DistanceUnit.METER);
        Distance y2 = new Distance(4.0, DistanceUnit.METER);
        Vector2d v2 = new Vector2d(x2, y2, CoordinateSystem.DECODE_FTC);

        Vector2d result = v1.plus(v2);
        assertEquals(4.0, result.x.magnitude, DELTA);
        assertEquals(6.0, result.y.magnitude, DELTA);
    }

    @Test
    public void plus_differentUnits()
    {
        Distance x1 = new Distance(1.0, DistanceUnit.METER);
        Distance y1 = new Distance(2.0, DistanceUnit.METER);
        Vector2d v1 = new Vector2d(x1, y1, CoordinateSystem.DECODE_FTC);

        Distance x2 = new Distance(100.0, DistanceUnit.CM);
        Distance y2 = new Distance(200.0, DistanceUnit.CM);
        Vector2d v2 = new Vector2d(x2, y2, CoordinateSystem.DECODE_FTC);

        Vector2d result = v1.plus(v2);
        assertEquals(2.0, result.x.magnitude, DELTA);
        assertEquals(4.0, result.y.magnitude, DELTA);
        assertEquals(DistanceUnit.METER, result.distUnit);
    }

    @Test
    public void plus_withZeroVector()
    {
        Distance x1 = new Distance(3.0, DistanceUnit.METER);
        Distance y1 = new Distance(4.0, DistanceUnit.METER);
        Vector2d v1 = new Vector2d(x1, y1, CoordinateSystem.DECODE_FTC);

        Distance x2 = new Distance(0.0, DistanceUnit.METER);
        Distance y2 = new Distance(0.0, DistanceUnit.METER);
        Vector2d v2 = new Vector2d(x2, y2, CoordinateSystem.DECODE_FTC);

        Vector2d result = v1.plus(v2);
        assertEquals(3.0, result.x.magnitude, DELTA);
        assertEquals(4.0, result.y.magnitude, DELTA);
    }

    @Test
    public void plus_negativeComponents()
    {
        Distance x1 = new Distance(5.0, DistanceUnit.METER);
        Distance y1 = new Distance(5.0, DistanceUnit.METER);
        Vector2d v1 = new Vector2d(x1, y1, CoordinateSystem.DECODE_FTC);

        Distance x2 = new Distance(-3.0, DistanceUnit.METER);
        Distance y2 = new Distance(-2.0, DistanceUnit.METER);
        Vector2d v2 = new Vector2d(x2, y2, CoordinateSystem.DECODE_FTC);

        Vector2d result = v1.plus(v2);
        assertEquals(2.0, result.x.magnitude, DELTA);
        assertEquals(3.0, result.y.magnitude, DELTA);
    }

    // ==================== minus() Tests ====================

    @Test
    public void minus_sameUnit()
    {
        Distance x1 = new Distance(5.0, DistanceUnit.METER);
        Distance y1 = new Distance(6.0, DistanceUnit.METER);
        Vector2d v1 = new Vector2d(x1, y1, CoordinateSystem.DECODE_FTC);

        Distance x2 = new Distance(2.0, DistanceUnit.METER);
        Distance y2 = new Distance(3.0, DistanceUnit.METER);
        Vector2d v2 = new Vector2d(x2, y2, CoordinateSystem.DECODE_FTC);

        Vector2d result = v1.minus(v2);
        assertEquals(3.0, result.x.magnitude, DELTA);
        assertEquals(3.0, result.y.magnitude, DELTA);
    }

    @Test
    public void minus_differentUnits()
    {
        Distance x1 = new Distance(2.0, DistanceUnit.METER);
        Distance y1 = new Distance(3.0, DistanceUnit.METER);
        Vector2d v1 = new Vector2d(x1, y1, CoordinateSystem.DECODE_FTC);

        Distance x2 = new Distance(100.0, DistanceUnit.CM);
        Distance y2 = new Distance(100.0, DistanceUnit.CM);
        Vector2d v2 = new Vector2d(x2, y2, CoordinateSystem.DECODE_FTC);

        Vector2d result = v1.minus(v2);
        assertEquals(1.0, result.x.magnitude, DELTA);
        assertEquals(2.0, result.y.magnitude, DELTA);
    }

    @Test
    public void minus_resultIsNegative()
    {
        Distance x1 = new Distance(1.0, DistanceUnit.METER);
        Distance y1 = new Distance(1.0, DistanceUnit.METER);
        Vector2d v1 = new Vector2d(x1, y1, CoordinateSystem.DECODE_FTC);

        Distance x2 = new Distance(3.0, DistanceUnit.METER);
        Distance y2 = new Distance(4.0, DistanceUnit.METER);
        Vector2d v2 = new Vector2d(x2, y2, CoordinateSystem.DECODE_FTC);

        Vector2d result = v1.minus(v2);
        assertEquals(-2.0, result.x.magnitude, DELTA);
        assertEquals(-3.0, result.y.magnitude, DELTA);
    }

    @Test
    public void minus_withZeroVector()
    {
        Distance x1 = new Distance(3.0, DistanceUnit.METER);
        Distance y1 = new Distance(4.0, DistanceUnit.METER);
        Vector2d v1 = new Vector2d(x1, y1, CoordinateSystem.DECODE_FTC);

        Distance x2 = new Distance(0.0, DistanceUnit.METER);
        Distance y2 = new Distance(0.0, DistanceUnit.METER);
        Vector2d v2 = new Vector2d(x2, y2, CoordinateSystem.DECODE_FTC);

        Vector2d result = v1.minus(v2);
        assertEquals(3.0, result.x.magnitude, DELTA);
        assertEquals(4.0, result.y.magnitude, DELTA);
    }

    // ==================== isDistanceUnit() Tests ====================

    @Test
    public void isDistanceUnit_sameUnit_returnsTrue()
    {
        Distance x = new Distance(1.0, DistanceUnit.METER);
        Distance y = new Distance(1.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        assertTrue(v.isDistanceUnit(DistanceUnit.METER));
    }

    @Test
    public void isDistanceUnit_differentUnit_returnsFalse()
    {
        Distance x = new Distance(1.0, DistanceUnit.METER);
        Distance y = new Distance(1.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        assertFalse(v.isDistanceUnit(DistanceUnit.CM));
        assertFalse(v.isDistanceUnit(DistanceUnit.MM));
        assertFalse(v.isDistanceUnit(DistanceUnit.INCH));
    }

    // ==================== isAngleUnit() Tests ====================

    @Test
    public void isAngleUnit_sameUnit_returnsTrue()
    {
        Distance x = new Distance(1.0, DistanceUnit.METER);
        Distance y = new Distance(1.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC); // default is RADIANS

        assertEquals(AngleUnit.RADIANS, v.angUnit);
    }

    @Test
    public void isAngleUnit_differentUnit_returnsFalse()
    {
        Distance x = new Distance(1.0, DistanceUnit.METER);
        Distance y = new Distance(1.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC); // default is RADIANS

        assertNotEquals(AngleUnit.DEGREES, v.angUnit);
    }

    // ==================== equals() Tests ====================

    @Test
    public void equals_sameVector_returnsTrue()
    {
        Distance x = new Distance(3.0, DistanceUnit.METER);
        Distance y = new Distance(4.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        assertTrue(v.equals(v));
    }

    @Test
    public void equals_equalVectors_returnsTrue()
    {
        Distance x1 = new Distance(3.0, DistanceUnit.METER);
        Distance y1 = new Distance(4.0, DistanceUnit.METER);
        Vector2d v1 = new Vector2d(x1, y1, CoordinateSystem.DECODE_FTC);

        Distance x2 = new Distance(3.0, DistanceUnit.METER);
        Distance y2 = new Distance(4.0, DistanceUnit.METER);
        Vector2d v2 = new Vector2d(x2, y2, CoordinateSystem.DECODE_FTC);

        assertTrue(v1.equals(v2));
    }

    @Test
    public void equals_differentUnits_sameDistance_returnsTrue()
    {
        Distance x1 = new Distance(1.0, DistanceUnit.METER);
        Distance y1 = new Distance(2.0, DistanceUnit.METER);
        Vector2d v1 = new Vector2d(x1, y1, CoordinateSystem.DECODE_FTC);

        Distance x2 = new Distance(100.0, DistanceUnit.CM);
        Distance y2 = new Distance(200.0, DistanceUnit.CM);
        Vector2d v2 = new Vector2d(x2, y2, CoordinateSystem.DECODE_FTC);

        assertTrue(v1.equals(v2));
    }

    @Test
    public void equals_differentX_returnsFalse()
    {
        Distance x1 = new Distance(3.0, DistanceUnit.METER);
        Distance y1 = new Distance(4.0, DistanceUnit.METER);
        Vector2d v1 = new Vector2d(x1, y1, CoordinateSystem.DECODE_FTC);

        Distance x2 = new Distance(5.0, DistanceUnit.METER);
        Distance y2 = new Distance(4.0, DistanceUnit.METER);
        Vector2d v2 = new Vector2d(x2, y2, CoordinateSystem.DECODE_FTC);

        assertFalse(v1.equals(v2));
    }

    @Test
    public void equals_differentY_returnsFalse()
    {
        Distance x1 = new Distance(3.0, DistanceUnit.METER);
        Distance y1 = new Distance(4.0, DistanceUnit.METER);
        Vector2d v1 = new Vector2d(x1, y1, CoordinateSystem.DECODE_FTC);

        Distance x2 = new Distance(3.0, DistanceUnit.METER);
        Distance y2 = new Distance(5.0, DistanceUnit.METER);
        Vector2d v2 = new Vector2d(x2, y2, CoordinateSystem.DECODE_FTC);

        assertFalse(v1.equals(v2));
    }

    @Test
    public void equals_null_returnsFalse()
    {
        Distance x = new Distance(3.0, DistanceUnit.METER);
        Distance y = new Distance(4.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        assertFalse(v.equals(null));
    }

    @Test
    public void equals_differentClass_returnsFalse()
    {
        Distance x = new Distance(3.0, DistanceUnit.METER);
        Distance y = new Distance(4.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        assertFalse(v.equals("not a vector"));
    }

    @Test
    public void equals_zeroVectors_returnsTrue()
    {
        Distance x1 = new Distance(0.0, DistanceUnit.METER);
        Distance y1 = new Distance(0.0, DistanceUnit.METER);
        Vector2d v1 = new Vector2d(x1, y1, CoordinateSystem.DECODE_FTC);

        Distance x2 = new Distance(0.0, DistanceUnit.CM);
        Distance y2 = new Distance(0.0, DistanceUnit.CM);
        Vector2d v2 = new Vector2d(x2, y2, CoordinateSystem.DECODE_FTC);

        assertTrue(v1.equals(v2));
    }

    // ==================== Edge Cases ====================

    @Test
    public void edgeCase_unitVector_xAxis()
    {
        Distance x = new Distance(1.0, DistanceUnit.METER);
        Distance y = new Distance(0.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        assertEquals(1.0, v.getLength().magnitude, DELTA);
        assertEquals(0.0, v.getDirection().measure, DELTA);
    }

    @Test
    public void edgeCase_unitVector_yAxis()
    {
        Distance x = new Distance(0.0, DistanceUnit.METER);
        Distance y = new Distance(1.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        assertEquals(1.0, v.getLength().magnitude, DELTA);
        assertEquals(Math.PI / 2, v.getDirection().measure, DELTA);
    }

    @Test
    public void edgeCase_veryLargeComponents()
    {
        Distance x = new Distance(1e10, DistanceUnit.METER);
        Distance y = new Distance(1e10, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        Distance distance = v.getLength();
        assertEquals(Math.sqrt(2) * 1e10, distance.magnitude, 1e2);
    }

    @Test
    public void edgeCase_verySmallComponents()
    {
        Distance x = new Distance(1e-10, DistanceUnit.METER);
        Distance y = new Distance(1e-10, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        Distance distance = v.getLength();
        assertEquals(Math.sqrt(2) * 1e-10, distance.magnitude, 1e-18);
    }

    @Test
    public void edgeCase_chainedOperations()
    {
        Distance x1 = new Distance(1.0, DistanceUnit.METER);
        Distance y1 = new Distance(2.0, DistanceUnit.METER);
        Vector2d v1 = new Vector2d(x1, y1, CoordinateSystem.DECODE_FTC);

        Distance x2 = new Distance(3.0, DistanceUnit.METER);
        Distance y2 = new Distance(4.0, DistanceUnit.METER);
        Vector2d v2 = new Vector2d(x2, y2, CoordinateSystem.DECODE_FTC);

        Vector2d result = v1.plus(v2).minus(v1);
        assertEquals(3.0, result.x.magnitude, DELTA);
        assertEquals(4.0, result.y.magnitude, DELTA);
    }

    @Test
    public void edgeCase_plusInverse_returnsZero()
    {
        Distance x = new Distance(3.0, DistanceUnit.METER);
        Distance y = new Distance(4.0, DistanceUnit.METER);
        Vector2d v = new Vector2d(x, y, CoordinateSystem.DECODE_FTC);

        Vector2d result = v.plus(v.inverse());
        assertEquals(0.0, result.x.magnitude, DELTA);
        assertEquals(0.0, result.y.magnitude, DELTA);
    }
}

