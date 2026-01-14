package org.firstinspires.ftc.teamcode.util.measure.angle;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.junit.Test;
import static org.junit.Assert.*;

/**
 * Comprehensive unit tests for the {@link Angle} class.
 */
public class AngleUnitTests
{
    private static final double DELTA = 1e-9;

    // ==================== Constructor Tests ====================

    @Test
    public void constructor_setsMeasureAndUnit_degrees()
    {
        Angle a = new Angle(45.0, AngleUnit.DEGREES);
        assertEquals(45.0, a.measure, DELTA);
        assertEquals(AngleUnit.DEGREES, a.unit);
    }

    @Test
    public void constructor_setsMeasureAndUnit_radians()
    {
        Angle a = new Angle(Math.PI / 4, AngleUnit.RADIANS);
        assertEquals(Math.PI / 4, a.measure, DELTA);
        assertEquals(AngleUnit.RADIANS, a.unit);
    }

    @Test
    public void constructor_normalizesAngle_degrees_positive()
    {
        // 370 degrees should normalize to 10 degrees
        Angle a = new Angle(370.0, AngleUnit.DEGREES);
        assertEquals(10.0, a.measure, DELTA);
    }

    @Test
    public void constructor_normalizesAngle_degrees_negative()
    {
        // -370 degrees should normalize to -10 degrees
        Angle a = new Angle(-370.0, AngleUnit.DEGREES);
        assertEquals(-10.0, a.measure, DELTA);
    }

    @Test
    public void constructor_normalizesAngle_radians_positive()
    {
        // 3*PI should normalize to -PI (since range is [-PI, PI))
        Angle a = new Angle(3 * Math.PI, AngleUnit.RADIANS);
        // PI normalizes to -PI in the range [-PI, PI)
        assertTrue(Math.abs(a.measure - Math.PI) < DELTA || Math.abs(a.measure + Math.PI) < DELTA);
    }

    @Test
    public void constructor_normalizesAngle_radians_negative()
    {
        // -3*PI should normalize to -PI
        Angle a = new Angle(-3 * Math.PI, AngleUnit.RADIANS);
        assertEquals(-Math.PI, a.measure, DELTA);
    }

    @Test
    public void constructor_zeroAngle()
    {
        Angle a = new Angle(0.0, AngleUnit.DEGREES);
        assertEquals(0.0, a.measure, DELTA);
    }

    @Test
    public void constructor_halfCircle_degrees()
    {
        Angle a = new Angle(180.0, AngleUnit.DEGREES);
        assertTrue(Math.abs(a.measure) == 180.0 || Math.abs(a.measure) < DELTA);
    }

    @Test
    public void constructor_halfCircle_radians()
    {
        Angle a = new Angle(Math.PI, AngleUnit.RADIANS);
        assertTrue(Math.abs(a.measure - Math.PI) < DELTA || Math.abs(a.measure + Math.PI) < DELTA);
    }

    // ==================== toUnit(AngleUnit) Tests ====================

    @Test
    public void toUnit_sameUnit_returnsSameInstance()
    {
        Angle a = new Angle(45.0, AngleUnit.DEGREES);
        Angle converted = a.toUnit(AngleUnit.DEGREES);
        assertSame(a, converted);
    }

    @Test
    public void toUnit_degreesToRadians()
    {
        Angle a = new Angle(90.0, AngleUnit.DEGREES);
        Angle converted = a.toUnit(AngleUnit.RADIANS);
        assertEquals(Math.PI / 2, converted.measure, DELTA);
        assertEquals(AngleUnit.RADIANS, converted.unit);
    }

    @Test
    public void toUnit_radiansToDegrees()
    {
        Angle a = new Angle(Math.PI / 2, AngleUnit.RADIANS);
        Angle converted = a.toUnit(AngleUnit.DEGREES);
        assertEquals(90.0, converted.measure, DELTA);
        assertEquals(AngleUnit.DEGREES, converted.unit);
    }

    @Test
    public void toUnit_zeroDegrees()
    {
        Angle a = new Angle(0.0, AngleUnit.DEGREES);
        Angle converted = a.toUnit(AngleUnit.RADIANS);
        assertEquals(0.0, converted.measure, DELTA);
    }

    @Test
    public void toUnit_zeroRadians()
    {
        Angle a = new Angle(0.0, AngleUnit.RADIANS);
        Angle converted = a.toUnit(AngleUnit.DEGREES);
        assertEquals(0.0, converted.measure, DELTA);
    }

    @Test
    public void toUnit_negativeAngle()
    {
        Angle a = new Angle(-90.0, AngleUnit.DEGREES);
        Angle converted = a.toUnit(AngleUnit.RADIANS);
        assertEquals(-Math.PI / 2, converted.measure, DELTA);
    }

    // ==================== toUnit(UnnormalizedAngleUnit) Tests ====================

    @Test
    public void toUnnormalizedUnit_degreesToUnnormalizedDegrees()
    {
        Angle a = new Angle(45.0, AngleUnit.DEGREES);
        UnnormalizedAngle converted = a.toUnit(UnnormalizedAngleUnit.DEGREES);
        assertEquals(45.0, converted.measure, DELTA);
        assertEquals(UnnormalizedAngleUnit.DEGREES, converted.unit);
    }

    @Test
    public void toUnnormalizedUnit_radiansToUnnormalizedRadians()
    {
        Angle a = new Angle(Math.PI / 4, AngleUnit.RADIANS);
        UnnormalizedAngle converted = a.toUnit(UnnormalizedAngleUnit.RADIANS);
        assertEquals(Math.PI / 4, converted.measure, DELTA);
        assertEquals(UnnormalizedAngleUnit.RADIANS, converted.unit);
    }

    @Test
    public void toUnnormalizedUnit_degreesToUnnormalizedRadians()
    {
        Angle a = new Angle(90.0, AngleUnit.DEGREES);
        UnnormalizedAngle converted = a.toUnit(UnnormalizedAngleUnit.RADIANS);
        assertEquals(Math.PI / 2, converted.measure, DELTA);
        assertEquals(UnnormalizedAngleUnit.RADIANS, converted.unit);
    }

    // ==================== toUnnormalized() Tests ====================

    @Test
    public void toUnnormalized_degrees()
    {
        Angle a = new Angle(45.0, AngleUnit.DEGREES);
        UnnormalizedAngle converted = a.toUnnormalized();
        assertEquals(45.0, converted.measure, DELTA);
        assertEquals(UnnormalizedAngleUnit.DEGREES, converted.unit);
    }

    @Test
    public void toUnnormalized_radians()
    {
        Angle a = new Angle(Math.PI / 4, AngleUnit.RADIANS);
        UnnormalizedAngle converted = a.toUnnormalized();
        assertEquals(Math.PI / 4, converted.measure, DELTA);
        assertEquals(UnnormalizedAngleUnit.RADIANS, converted.unit);
    }

    // ==================== getAngle() Tests ====================

    @Test
    public void getAngle_sameUnit()
    {
        Angle a = new Angle(45.0, AngleUnit.DEGREES);
        assertEquals(45.0, a.getAngle(AngleUnit.DEGREES), DELTA);
    }

    @Test
    public void getAngle_degreesToRadians()
    {
        Angle a = new Angle(180.0, AngleUnit.DEGREES);
        assertEquals(Math.PI, Math.abs(a.getAngle(AngleUnit.RADIANS)), DELTA);
    }

    @Test
    public void getAngle_radiansToDegrees()
    {
        Angle a = new Angle(Math.PI / 6, AngleUnit.RADIANS);
        assertEquals(30.0, a.getAngle(AngleUnit.DEGREES), DELTA);
    }

    // ==================== getUnsignedAngle() Tests ====================

    @Test
    public void getUnsignedAngle_positiveAngle()
    {
        Angle a = new Angle(45.0, AngleUnit.DEGREES);
        assertEquals(45.0, a.getUnsignedAngle(), DELTA);
    }

    @Test
    public void getUnsignedAngle_negativeAngle_degrees()
    {
        Angle a = new Angle(-45.0, AngleUnit.DEGREES);
        assertEquals(315.0, a.getUnsignedAngle(), DELTA);
    }

    @Test
    public void getUnsignedAngle_negativeAngle_radians()
    {
        Angle a = new Angle(-Math.PI / 2, AngleUnit.RADIANS);
        assertEquals(3 * Math.PI / 2, a.getUnsignedAngle(), DELTA);
    }

    @Test
    public void getUnsignedAngle_zeroAngle()
    {
        Angle a = new Angle(0.0, AngleUnit.DEGREES);
        assertEquals(0.0, a.getUnsignedAngle(), DELTA);
    }

    @Test
    public void getUnsignedAngle_withNewUnit()
    {
        Angle a = new Angle(-90.0, AngleUnit.DEGREES);
        assertEquals(3 * Math.PI / 2, a.getUnsignedAngle(AngleUnit.RADIANS), DELTA);
    }

    // ==================== plus() Tests ====================

    @Test
    public void plus_sameUnit()
    {
        Angle a = new Angle(30.0, AngleUnit.DEGREES);
        Angle b = new Angle(45.0, AngleUnit.DEGREES);
        Angle result = a.plus(b);
        assertEquals(75.0, result.measure, DELTA);
        assertEquals(AngleUnit.DEGREES, result.unit);
    }

    @Test
    public void plus_differentUnits()
    {
        Angle a = new Angle(90.0, AngleUnit.DEGREES);
        Angle b = new Angle(Math.PI / 2, AngleUnit.RADIANS);
        Angle result = a.plus(b);
        // 90 + 90 = 180, normalized could be -180 or 180
        assertTrue(Math.abs(result.measure) == 180.0 || Math.abs(result.measure) < DELTA);
        assertEquals(AngleUnit.DEGREES, result.unit);
    }

    @Test
    public void plus_withZero()
    {
        Angle a = new Angle(45.0, AngleUnit.DEGREES);
        Angle b = new Angle(0.0, AngleUnit.DEGREES);
        Angle result = a.plus(b);
        assertEquals(45.0, result.measure, DELTA);
    }

    @Test
    public void plus_wrapsAround()
    {
        Angle a = new Angle(170.0, AngleUnit.DEGREES);
        Angle b = new Angle(20.0, AngleUnit.DEGREES);
        Angle result = a.plus(b);
        // 170 + 20 = 190, normalized to -170
        assertEquals(-170.0, result.measure, DELTA);
    }

    @Test
    public void plus_negativeAngles()
    {
        Angle a = new Angle(-30.0, AngleUnit.DEGREES);
        Angle b = new Angle(-45.0, AngleUnit.DEGREES);
        Angle result = a.plus(b);
        assertEquals(-75.0, result.measure, DELTA);
    }

    // ==================== minus() Tests ====================

    @Test
    public void minus_sameUnit()
    {
        Angle a = new Angle(75.0, AngleUnit.DEGREES);
        Angle b = new Angle(30.0, AngleUnit.DEGREES);
        Angle result = a.minus(b);
        assertEquals(45.0, result.measure, DELTA);
    }

    @Test
    public void minus_differentUnits()
    {
        Angle a = new Angle(180.0, AngleUnit.DEGREES);
        Angle b = new Angle(Math.PI / 2, AngleUnit.RADIANS);
        Angle result = a.minus(b);
        assertEquals(90.0, result.measure, DELTA);
        assertEquals(AngleUnit.DEGREES, result.unit);
    }

    @Test
    public void minus_withZero()
    {
        Angle a = new Angle(45.0, AngleUnit.DEGREES);
        Angle b = new Angle(0.0, AngleUnit.DEGREES);
        Angle result = a.minus(b);
        assertEquals(45.0, result.measure, DELTA);
    }

    @Test
    public void minus_resultIsNegative()
    {
        Angle a = new Angle(30.0, AngleUnit.DEGREES);
        Angle b = new Angle(75.0, AngleUnit.DEGREES);
        Angle result = a.minus(b);
        assertEquals(-45.0, result.measure, DELTA);
    }

    @Test
    public void minus_wrapsAround()
    {
        Angle a = new Angle(-170.0, AngleUnit.DEGREES);
        Angle b = new Angle(20.0, AngleUnit.DEGREES);
        Angle result = a.minus(b);
        // -170 - 20 = -190, normalized to 170
        assertEquals(170.0, result.measure, DELTA);
    }

    // ==================== getUnnormalizedUnit() Tests ====================

    @Test
    public void getUnnormalizedUnit_degrees()
    {
        UnnormalizedAngleUnit result = Angle.getUnnormalizedUnit(AngleUnit.DEGREES);
        assertEquals(UnnormalizedAngleUnit.DEGREES, result);
    }

    @Test
    public void getUnnormalizedUnit_radians()
    {
        UnnormalizedAngleUnit result = Angle.getUnnormalizedUnit(AngleUnit.RADIANS);
        assertEquals(UnnormalizedAngleUnit.RADIANS, result);
    }

    // ==================== isUnit() Tests ====================

    @Test
    public void isUnit_sameUnit_returnsTrue()
    {
        Angle a = new Angle(45.0, AngleUnit.DEGREES);
        assertTrue(a.isUnit(AngleUnit.DEGREES));
    }

    @Test
    public void isUnit_differentUnit_returnsFalse()
    {
        Angle a = new Angle(45.0, AngleUnit.DEGREES);
        assertFalse(a.isUnit(AngleUnit.RADIANS));
    }

    // ==================== toString() Tests ====================

    @Test
    public void toString_degrees()
    {
        Angle a = new Angle(45.0, AngleUnit.DEGREES);
        String result = a.toString();
        assertNotNull(result);
        assertTrue(result.contains("45"));
        assertTrue(result.contains("degrees"));
    }

    @Test
    public void toString_radians()
    {
        Angle a = new Angle(1.0, AngleUnit.RADIANS);
        String result = a.toString();
        assertNotNull(result);
        assertTrue(result.contains("1"));
        assertTrue(result.contains("radians"));
    }

    // ==================== Edge Cases ====================

    @Test
    public void edgeCase_fullCircle_degrees()
    {
        Angle a = new Angle(360.0, AngleUnit.DEGREES);
        assertEquals(0.0, a.measure, DELTA);
    }

    @Test
    public void edgeCase_fullCircle_radians()
    {
        Angle a = new Angle(2 * Math.PI, AngleUnit.RADIANS);
        assertEquals(0.0, a.measure, DELTA);
    }

    @Test
    public void edgeCase_multipleRotations()
    {
        Angle a = new Angle(720.0 + 45.0, AngleUnit.DEGREES);
        assertEquals(45.0, a.measure, DELTA);
    }

    @Test
    public void edgeCase_negativeMultipleRotations()
    {
        Angle a = new Angle(-720.0 - 45.0, AngleUnit.DEGREES);
        assertEquals(-45.0, a.measure, DELTA);
    }

    @Test
    public void edgeCase_chainedConversions()
    {
        Angle a = new Angle(45.0, AngleUnit.DEGREES);
        Angle result = a.toUnit(AngleUnit.RADIANS).toUnit(AngleUnit.DEGREES);
        assertEquals(45.0, result.measure, DELTA);
    }

    @Test
    public void edgeCase_chainedArithmetic()
    {
        Angle a = new Angle(30.0, AngleUnit.DEGREES);
        Angle result = a.plus(new Angle(20.0, AngleUnit.DEGREES))
                        .minus(new Angle(10.0, AngleUnit.DEGREES));
        assertEquals(40.0, result.measure, DELTA);
    }
}

