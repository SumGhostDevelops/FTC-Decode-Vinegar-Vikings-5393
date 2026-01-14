package org.firstinspires.ftc.teamcode.util.measure.angle;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.junit.Test;
import static org.junit.Assert.*;

/**
 * Comprehensive unit tests for the {@link UnnormalizedAngle} class.
 */
public class UnnormalizedAngleUnitTests
{
    private static final double DELTA = 1e-9;

    // ==================== Constructor Tests ====================

    @Test
    public void constructor_setsAngleAndUnit_degrees()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(45.0, UnnormalizedAngleUnit.DEGREES);
        assertEquals(45.0, a.measure, DELTA);
        assertEquals(UnnormalizedAngleUnit.DEGREES, a.unit);
    }

    @Test
    public void constructor_setsAngleAndUnit_radians()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(Math.PI / 4, UnnormalizedAngleUnit.RADIANS);
        assertEquals(Math.PI / 4, a.measure, DELTA);
        assertEquals(UnnormalizedAngleUnit.RADIANS, a.unit);
    }

    @Test
    public void constructor_doesNotNormalize_largePositive()
    {
        // 720 degrees should remain 720 (unnormalized)
        UnnormalizedAngle a = new UnnormalizedAngle(720.0, UnnormalizedAngleUnit.DEGREES);
        assertEquals(720.0, a.measure, DELTA);
    }

    @Test
    public void constructor_doesNotNormalize_largeNegative()
    {
        // -720 degrees should remain -720 (unnormalized)
        UnnormalizedAngle a = new UnnormalizedAngle(-720.0, UnnormalizedAngleUnit.DEGREES);
        assertEquals(-720.0, a.measure, DELTA);
    }

    @Test
    public void constructor_zeroAngle()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(0.0, UnnormalizedAngleUnit.DEGREES);
        assertEquals(0.0, a.measure, DELTA);
    }

    @Test
    public void constructor_negativeAngle()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(-45.0, UnnormalizedAngleUnit.DEGREES);
        assertEquals(-45.0, a.measure, DELTA);
    }

    @Test
    public void constructor_multipleRotations()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(1080.0, UnnormalizedAngleUnit.DEGREES);
        assertEquals(1080.0, a.measure, DELTA);
    }

    // ==================== toUnit(UnnormalizedAngleUnit) Tests ====================

    @Test
    public void toUnit_sameUnit_returnsSameInstance()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(45.0, UnnormalizedAngleUnit.DEGREES);
        UnnormalizedAngle converted = a.toUnit(UnnormalizedAngleUnit.DEGREES);
        assertSame(a, converted);
    }

    @Test
    public void toUnit_degreesToRadians()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(180.0, UnnormalizedAngleUnit.DEGREES);
        UnnormalizedAngle converted = a.toUnit(UnnormalizedAngleUnit.RADIANS);
        assertEquals(Math.PI, converted.measure, DELTA);
        assertEquals(UnnormalizedAngleUnit.RADIANS, converted.unit);
    }

    @Test
    public void toUnit_radiansToDegrees()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(Math.PI, UnnormalizedAngleUnit.RADIANS);
        UnnormalizedAngle converted = a.toUnit(UnnormalizedAngleUnit.DEGREES);
        assertEquals(180.0, converted.measure, DELTA);
        assertEquals(UnnormalizedAngleUnit.DEGREES, converted.unit);
    }

    @Test
    public void toUnit_preservesMultipleRotations_degrees()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(720.0, UnnormalizedAngleUnit.DEGREES);
        UnnormalizedAngle converted = a.toUnit(UnnormalizedAngleUnit.RADIANS);
        assertEquals(4 * Math.PI, converted.measure, DELTA);
    }

    @Test
    public void toUnit_preservesMultipleRotations_radians()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(4 * Math.PI, UnnormalizedAngleUnit.RADIANS);
        UnnormalizedAngle converted = a.toUnit(UnnormalizedAngleUnit.DEGREES);
        assertEquals(720.0, converted.measure, DELTA);
    }

    @Test
    public void toUnit_negativeAngle()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(-90.0, UnnormalizedAngleUnit.DEGREES);
        UnnormalizedAngle converted = a.toUnit(UnnormalizedAngleUnit.RADIANS);
        assertEquals(-Math.PI / 2, converted.measure, DELTA);
    }

    // ==================== toUnit(AngleUnit) Tests ====================

    @Test
    public void toNormalizedUnit_degreesToDegrees()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(45.0, UnnormalizedAngleUnit.DEGREES);
        Angle converted = a.toUnit(AngleUnit.DEGREES);
        assertEquals(45.0, converted.measure, DELTA);
        assertEquals(AngleUnit.DEGREES, converted.unit);
    }

    @Test
    public void toNormalizedUnit_degreesToRadians()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(90.0, UnnormalizedAngleUnit.DEGREES);
        Angle converted = a.toUnit(AngleUnit.RADIANS);
        assertEquals(Math.PI / 2, converted.measure, DELTA);
        assertEquals(AngleUnit.RADIANS, converted.unit);
    }

    @Test
    public void toNormalizedUnit_normalizesLargeAngle()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(370.0, UnnormalizedAngleUnit.DEGREES);
        Angle converted = a.toUnit(AngleUnit.DEGREES);
        assertEquals(10.0, converted.measure, DELTA);
    }

    @Test
    public void toNormalizedUnit_normalizesNegativeAngle()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(-370.0, UnnormalizedAngleUnit.DEGREES);
        Angle converted = a.toUnit(AngleUnit.DEGREES);
        assertEquals(-10.0, converted.measure, DELTA);
    }

    // ==================== toNormalized() Tests ====================

    @Test
    public void toNormalized_degrees()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(45.0, UnnormalizedAngleUnit.DEGREES);
        Angle converted = a.toNormalized();
        assertEquals(45.0, converted.measure, DELTA);
        assertEquals(AngleUnit.DEGREES, converted.unit);
    }

    @Test
    public void toNormalized_radians()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(Math.PI / 4, UnnormalizedAngleUnit.RADIANS);
        Angle converted = a.toNormalized();
        assertEquals(Math.PI / 4, converted.measure, DELTA);
        assertEquals(AngleUnit.RADIANS, converted.unit);
    }

    @Test
    public void toNormalized_largeAngle()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(720.0 + 45.0, UnnormalizedAngleUnit.DEGREES);
        Angle converted = a.toNormalized();
        assertEquals(45.0, converted.measure, DELTA);
    }

    // ==================== getAngle() Tests ====================

    @Test
    public void getAngle_sameUnit()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(45.0, UnnormalizedAngleUnit.DEGREES);
        assertEquals(45.0, a.getAngle(UnnormalizedAngleUnit.DEGREES), DELTA);
    }

    @Test
    public void getAngle_degreesToRadians()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(180.0, UnnormalizedAngleUnit.DEGREES);
        assertEquals(Math.PI, a.getAngle(UnnormalizedAngleUnit.RADIANS), DELTA);
    }

    @Test
    public void getAngle_radiansToDegrees()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(Math.PI / 6, UnnormalizedAngleUnit.RADIANS);
        assertEquals(30.0, a.getAngle(UnnormalizedAngleUnit.DEGREES), DELTA);
    }

    @Test
    public void getAngle_largeAngle()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(720.0, UnnormalizedAngleUnit.DEGREES);
        assertEquals(4 * Math.PI, a.getAngle(UnnormalizedAngleUnit.RADIANS), DELTA);
    }

    // ==================== getWrappedAngle() Tests ====================

    @Test
    public void getWrappedAngle_withinRange()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(45.0, UnnormalizedAngleUnit.DEGREES);
        assertEquals(45.0, a.getWrappedAngle(), DELTA);
    }

    @Test
    public void getWrappedAngle_largePositive()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(370.0, UnnormalizedAngleUnit.DEGREES);
        assertEquals(10.0, a.getWrappedAngle(), DELTA);
    }

    @Test
    public void getWrappedAngle_largeNegative()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(-370.0, UnnormalizedAngleUnit.DEGREES);
        // Wrapped to unsigned: -10 becomes 350
        double wrapped = a.getWrappedAngle();
        assertTrue(wrapped == 350.0 || Math.abs(wrapped - 350.0) < DELTA);
    }

    @Test
    public void getWrappedAngle_withNewUnit()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(720.0 + 90.0, UnnormalizedAngleUnit.DEGREES);
        assertEquals(Math.PI / 2, a.getWrappedAngle(UnnormalizedAngleUnit.RADIANS), DELTA);
    }

    // ==================== plus() Tests ====================

    @Test
    public void plus_sameUnit()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(30.0, UnnormalizedAngleUnit.DEGREES);
        UnnormalizedAngle b = new UnnormalizedAngle(45.0, UnnormalizedAngleUnit.DEGREES);
        UnnormalizedAngle result = a.plus(b);
        assertEquals(75.0, result.measure, DELTA);
        assertEquals(UnnormalizedAngleUnit.DEGREES, result.unit);
    }

    @Test
    public void plus_differentUnits()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(90.0, UnnormalizedAngleUnit.DEGREES);
        UnnormalizedAngle b = new UnnormalizedAngle(Math.PI / 2, UnnormalizedAngleUnit.RADIANS);
        UnnormalizedAngle result = a.plus(b);
        assertEquals(180.0, result.measure, DELTA);
        assertEquals(UnnormalizedAngleUnit.DEGREES, result.unit);
    }

    @Test
    public void plus_withZero()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(45.0, UnnormalizedAngleUnit.DEGREES);
        UnnormalizedAngle b = new UnnormalizedAngle(0.0, UnnormalizedAngleUnit.DEGREES);
        UnnormalizedAngle result = a.plus(b);
        assertEquals(45.0, result.measure, DELTA);
    }

    @Test
    public void plus_preservesMultipleRotations()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(360.0, UnnormalizedAngleUnit.DEGREES);
        UnnormalizedAngle b = new UnnormalizedAngle(360.0, UnnormalizedAngleUnit.DEGREES);
        UnnormalizedAngle result = a.plus(b);
        assertEquals(720.0, result.measure, DELTA);
    }

    @Test
    public void plus_negativeAngles()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(-30.0, UnnormalizedAngleUnit.DEGREES);
        UnnormalizedAngle b = new UnnormalizedAngle(-45.0, UnnormalizedAngleUnit.DEGREES);
        UnnormalizedAngle result = a.plus(b);
        assertEquals(-75.0, result.measure, DELTA);
    }

    // ==================== minus() Tests ====================

    @Test
    public void minus_sameUnit()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(75.0, UnnormalizedAngleUnit.DEGREES);
        UnnormalizedAngle b = new UnnormalizedAngle(30.0, UnnormalizedAngleUnit.DEGREES);
        UnnormalizedAngle result = a.minus(b);
        assertEquals(45.0, result.measure, DELTA);
        assertEquals(UnnormalizedAngleUnit.DEGREES, result.unit);
    }

    @Test
    public void minus_differentUnits()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(180.0, UnnormalizedAngleUnit.DEGREES);
        UnnormalizedAngle b = new UnnormalizedAngle(Math.PI / 2, UnnormalizedAngleUnit.RADIANS);
        UnnormalizedAngle result = a.minus(b);
        assertEquals(90.0, result.measure, DELTA);
        assertEquals(UnnormalizedAngleUnit.DEGREES, result.unit);
    }

    @Test
    public void minus_withZero()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(45.0, UnnormalizedAngleUnit.DEGREES);
        UnnormalizedAngle b = new UnnormalizedAngle(0.0, UnnormalizedAngleUnit.DEGREES);
        UnnormalizedAngle result = a.minus(b);
        assertEquals(45.0, result.measure, DELTA);
    }

    @Test
    public void minus_resultIsNegative()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(30.0, UnnormalizedAngleUnit.DEGREES);
        UnnormalizedAngle b = new UnnormalizedAngle(75.0, UnnormalizedAngleUnit.DEGREES);
        UnnormalizedAngle result = a.minus(b);
        assertEquals(-45.0, result.measure, DELTA);
    }

    @Test
    public void minus_preservesMultipleRotations()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(720.0, UnnormalizedAngleUnit.DEGREES);
        UnnormalizedAngle b = new UnnormalizedAngle(360.0, UnnormalizedAngleUnit.DEGREES);
        UnnormalizedAngle result = a.minus(b);
        assertEquals(360.0, result.measure, DELTA);
    }

    // ==================== getNormalizedUnit() Tests ====================

    @Test
    public void getNormalizedUnit_degrees()
    {
        AngleUnit result = UnnormalizedAngle.getNormalizedUnit(UnnormalizedAngleUnit.DEGREES);
        assertEquals(AngleUnit.DEGREES, result);
    }

    @Test
    public void getNormalizedUnit_radians()
    {
        AngleUnit result = UnnormalizedAngle.getNormalizedUnit(UnnormalizedAngleUnit.RADIANS);
        assertEquals(AngleUnit.RADIANS, result);
    }

    // ==================== Edge Cases ====================

    @Test
    public void edgeCase_veryLargeAngle()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(36000.0, UnnormalizedAngleUnit.DEGREES);
        assertEquals(36000.0, a.measure, DELTA);
        Angle normalized = a.toNormalized();
        assertEquals(0.0, normalized.measure, DELTA);
    }

    @Test
    public void edgeCase_verySmallAngle()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(1e-15, UnnormalizedAngleUnit.RADIANS);
        assertEquals(1e-15, a.measure, 1e-24);
    }

    @Test
    public void edgeCase_chainedConversions()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(720.0, UnnormalizedAngleUnit.DEGREES);
        UnnormalizedAngle result = a.toUnit(UnnormalizedAngleUnit.RADIANS)
                                    .toUnit(UnnormalizedAngleUnit.DEGREES);
        assertEquals(720.0, result.measure, DELTA);
    }

    @Test
    public void edgeCase_chainedArithmetic()
    {
        UnnormalizedAngle a = new UnnormalizedAngle(360.0, UnnormalizedAngleUnit.DEGREES);
        UnnormalizedAngle result = a.plus(new UnnormalizedAngle(180.0, UnnormalizedAngleUnit.DEGREES))
                                    .minus(new UnnormalizedAngle(90.0, UnnormalizedAngleUnit.DEGREES));
        assertEquals(450.0, result.measure, DELTA);
    }
}

