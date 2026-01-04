package org.firstinspires.ftc.teamcode.util.measure.distance;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.junit.Test;
import static org.junit.Assert.*;

/**
 * Comprehensive unit tests for the {@link Distance} class.
 */
public class DistanceUnitTests
{
    private static final double DELTA = 1e-9;

    // ==================== Constructor Tests ====================

    @Test
    public void constructor_setsMagnitudeAndUnit_meter()
    {
        Distance d = new Distance(5.0, DistanceUnit.METER);
        assertEquals(5.0, d.magnitude, DELTA);
        assertEquals(DistanceUnit.METER, d.unit);
    }

    @Test
    public void constructor_setsMagnitudeAndUnit_cm()
    {
        Distance d = new Distance(100.0, DistanceUnit.CM);
        assertEquals(100.0, d.magnitude, DELTA);
        assertEquals(DistanceUnit.CM, d.unit);
    }

    @Test
    public void constructor_setsMagnitudeAndUnit_mm()
    {
        Distance d = new Distance(1000.0, DistanceUnit.MM);
        assertEquals(1000.0, d.magnitude, DELTA);
        assertEquals(DistanceUnit.MM, d.unit);
    }

    @Test
    public void constructor_setsMagnitudeAndUnit_inch()
    {
        Distance d = new Distance(12.0, DistanceUnit.INCH);
        assertEquals(12.0, d.magnitude, DELTA);
        assertEquals(DistanceUnit.INCH, d.unit);
    }

    @Test
    public void constructor_zeroMagnitude()
    {
        Distance d = new Distance(0.0, DistanceUnit.METER);
        assertEquals(0.0, d.magnitude, DELTA);
    }

    @Test
    public void constructor_negativeMagnitude()
    {
        Distance d = new Distance(-5.0, DistanceUnit.METER);
        assertEquals(-5.0, d.magnitude, DELTA);
    }

    // ==================== toUnit() Tests ====================

    @Test
    public void toUnit_sameUnit_returnsSameInstance()
    {
        Distance d = new Distance(5.0, DistanceUnit.METER);
        Distance converted = d.toUnit(DistanceUnit.METER);
        assertSame(d, converted);
    }

    @Test
    public void toUnit_meterToCm()
    {
        Distance d = new Distance(1.0, DistanceUnit.METER);
        Distance converted = d.toUnit(DistanceUnit.CM);
        assertEquals(100.0, converted.magnitude, DELTA);
        assertEquals(DistanceUnit.CM, converted.unit);
    }

    @Test
    public void toUnit_meterToMm()
    {
        Distance d = new Distance(1.0, DistanceUnit.METER);
        Distance converted = d.toUnit(DistanceUnit.MM);
        assertEquals(1000.0, converted.magnitude, DELTA);
        assertEquals(DistanceUnit.MM, converted.unit);
    }

    @Test
    public void toUnit_meterToInch()
    {
        Distance d = new Distance(1.0, DistanceUnit.METER);
        Distance converted = d.toUnit(DistanceUnit.INCH);
        assertEquals(39.3700787401574803, converted.magnitude, 1e-6);
        assertEquals(DistanceUnit.INCH, converted.unit);
    }

    @Test
    public void toUnit_cmToMeter()
    {
        Distance d = new Distance(100.0, DistanceUnit.CM);
        Distance converted = d.toUnit(DistanceUnit.METER);
        assertEquals(1.0, converted.magnitude, DELTA);
        assertEquals(DistanceUnit.METER, converted.unit);
    }

    @Test
    public void toUnit_mmToMeter()
    {
        Distance d = new Distance(1000.0, DistanceUnit.MM);
        Distance converted = d.toUnit(DistanceUnit.METER);
        assertEquals(1.0, converted.magnitude, DELTA);
        assertEquals(DistanceUnit.METER, converted.unit);
    }

    @Test
    public void toUnit_inchToMeter()
    {
        Distance d = new Distance(39.3700787401574803, DistanceUnit.INCH);
        Distance converted = d.toUnit(DistanceUnit.METER);
        assertEquals(1.0, converted.magnitude, 1e-6);
        assertEquals(DistanceUnit.METER, converted.unit);
    }

    @Test
    public void toUnit_inchToCm()
    {
        Distance d = new Distance(1.0, DistanceUnit.INCH);
        Distance converted = d.toUnit(DistanceUnit.CM);
        assertEquals(2.54, converted.magnitude, DELTA);
        assertEquals(DistanceUnit.CM, converted.unit);
    }

    @Test
    public void toUnit_inchToMm()
    {
        Distance d = new Distance(1.0, DistanceUnit.INCH);
        Distance converted = d.toUnit(DistanceUnit.MM);
        assertEquals(25.4, converted.magnitude, DELTA);
        assertEquals(DistanceUnit.MM, converted.unit);
    }

    @Test
    public void toUnit_cmToMm()
    {
        Distance d = new Distance(1.0, DistanceUnit.CM);
        Distance converted = d.toUnit(DistanceUnit.MM);
        assertEquals(10.0, converted.magnitude, DELTA);
        assertEquals(DistanceUnit.MM, converted.unit);
    }

    @Test
    public void toUnit_mmToCm()
    {
        Distance d = new Distance(10.0, DistanceUnit.MM);
        Distance converted = d.toUnit(DistanceUnit.CM);
        assertEquals(1.0, converted.magnitude, DELTA);
        assertEquals(DistanceUnit.CM, converted.unit);
    }

    // ==================== getDistance() Tests ====================

    @Test
    public void getDistance_sameUnit()
    {
        Distance d = new Distance(5.0, DistanceUnit.METER);
        assertEquals(5.0, d.getDistance(DistanceUnit.METER), DELTA);
    }

    @Test
    public void getDistance_meterToCm()
    {
        Distance d = new Distance(2.5, DistanceUnit.METER);
        assertEquals(250.0, d.getDistance(DistanceUnit.CM), DELTA);
    }

    @Test
    public void getDistance_cmToInch()
    {
        Distance d = new Distance(2.54, DistanceUnit.CM);
        assertEquals(1.0, d.getDistance(DistanceUnit.INCH), DELTA);
    }

    // ==================== plus() Tests ====================

    @Test
    public void plus_sameUnit()
    {
        Distance a = new Distance(5.0, DistanceUnit.METER);
        Distance b = new Distance(3.0, DistanceUnit.METER);
        Distance result = a.plus(b);
        assertEquals(8.0, result.magnitude, DELTA);
        assertEquals(DistanceUnit.METER, result.unit);
    }

    @Test
    public void plus_differentUnits_meterPlusCm()
    {
        Distance a = new Distance(1.0, DistanceUnit.METER);
        Distance b = new Distance(50.0, DistanceUnit.CM);
        Distance result = a.plus(b);
        assertEquals(1.5, result.magnitude, DELTA);
        assertEquals(DistanceUnit.METER, result.unit);
    }

    @Test
    public void plus_differentUnits_cmPlusMeter()
    {
        Distance a = new Distance(50.0, DistanceUnit.CM);
        Distance b = new Distance(1.0, DistanceUnit.METER);
        Distance result = a.plus(b);
        assertEquals(150.0, result.magnitude, DELTA);
        assertEquals(DistanceUnit.CM, result.unit);
    }

    @Test
    public void plus_differentUnits_inchPlusCm()
    {
        Distance a = new Distance(1.0, DistanceUnit.INCH);
        Distance b = new Distance(2.54, DistanceUnit.CM);
        Distance result = a.plus(b);
        assertEquals(2.0, result.magnitude, DELTA);
        assertEquals(DistanceUnit.INCH, result.unit);
    }

    @Test
    public void plus_withZero()
    {
        Distance a = new Distance(5.0, DistanceUnit.METER);
        Distance b = new Distance(0.0, DistanceUnit.METER);
        Distance result = a.plus(b);
        assertEquals(5.0, result.magnitude, DELTA);
    }

    @Test
    public void plus_withNegative()
    {
        Distance a = new Distance(5.0, DistanceUnit.METER);
        Distance b = new Distance(-3.0, DistanceUnit.METER);
        Distance result = a.plus(b);
        assertEquals(2.0, result.magnitude, DELTA);
    }

    // ==================== minus() Tests ====================

    @Test
    public void minus_sameUnit()
    {
        Distance a = new Distance(5.0, DistanceUnit.METER);
        Distance b = new Distance(3.0, DistanceUnit.METER);
        Distance result = a.minus(b);
        assertEquals(2.0, result.magnitude, DELTA);
        assertEquals(DistanceUnit.METER, result.unit);
    }

    @Test
    public void minus_differentUnits_meterMinusCm()
    {
        Distance a = new Distance(1.0, DistanceUnit.METER);
        Distance b = new Distance(50.0, DistanceUnit.CM);
        Distance result = a.minus(b);
        assertEquals(0.5, result.magnitude, DELTA);
        assertEquals(DistanceUnit.METER, result.unit);
    }

    @Test
    public void minus_differentUnits_cmMinusMeter()
    {
        Distance a = new Distance(150.0, DistanceUnit.CM);
        Distance b = new Distance(1.0, DistanceUnit.METER);
        Distance result = a.minus(b);
        assertEquals(50.0, result.magnitude, DELTA);
        assertEquals(DistanceUnit.CM, result.unit);
    }

    @Test
    public void minus_resultIsNegative()
    {
        Distance a = new Distance(3.0, DistanceUnit.METER);
        Distance b = new Distance(5.0, DistanceUnit.METER);
        Distance result = a.minus(b);
        assertEquals(-2.0, result.magnitude, DELTA);
    }

    @Test
    public void minus_withZero()
    {
        Distance a = new Distance(5.0, DistanceUnit.METER);
        Distance b = new Distance(0.0, DistanceUnit.METER);
        Distance result = a.minus(b);
        assertEquals(5.0, result.magnitude, DELTA);
    }

    // ==================== multiply() Tests ====================

    @Test
    public void multiply_positiveScalar()
    {
        Distance d = new Distance(5.0, DistanceUnit.METER);
        Distance result = d.multiply(3.0);
        assertEquals(15.0, result.magnitude, DELTA);
        assertEquals(DistanceUnit.METER, result.unit);
    }

    @Test
    public void multiply_byZero()
    {
        Distance d = new Distance(5.0, DistanceUnit.METER);
        Distance result = d.multiply(0.0);
        assertEquals(0.0, result.magnitude, DELTA);
    }

    @Test
    public void multiply_negativeScalar()
    {
        Distance d = new Distance(5.0, DistanceUnit.METER);
        Distance result = d.multiply(-2.0);
        assertEquals(-10.0, result.magnitude, DELTA);
    }

    @Test
    public void multiply_fractionalScalar()
    {
        Distance d = new Distance(10.0, DistanceUnit.CM);
        Distance result = d.multiply(0.5);
        assertEquals(5.0, result.magnitude, DELTA);
        assertEquals(DistanceUnit.CM, result.unit);
    }

    @Test
    public void multiply_byOne()
    {
        Distance d = new Distance(5.0, DistanceUnit.METER);
        Distance result = d.multiply(1.0);
        assertEquals(5.0, result.magnitude, DELTA);
    }

    // ==================== divide() Tests ====================

    @Test
    public void divide_positiveScalar()
    {
        Distance d = new Distance(15.0, DistanceUnit.METER);
        Distance result = d.divide(3.0);
        assertEquals(5.0, result.magnitude, DELTA);
        assertEquals(DistanceUnit.METER, result.unit);
    }

    @Test
    public void divide_negativeScalar()
    {
        Distance d = new Distance(10.0, DistanceUnit.METER);
        Distance result = d.divide(-2.0);
        assertEquals(-5.0, result.magnitude, DELTA);
    }

    @Test
    public void divide_fractionalScalar()
    {
        Distance d = new Distance(5.0, DistanceUnit.CM);
        Distance result = d.divide(0.5);
        assertEquals(10.0, result.magnitude, DELTA);
        assertEquals(DistanceUnit.CM, result.unit);
    }

    @Test
    public void divide_byOne()
    {
        Distance d = new Distance(5.0, DistanceUnit.METER);
        Distance result = d.divide(1.0);
        assertEquals(5.0, result.magnitude, DELTA);
    }

    @Test(expected = IllegalArgumentException.class)
    public void divide_byZero_throwsException()
    {
        Distance d = new Distance(5.0, DistanceUnit.METER);
        d.divide(0.0);
    }

    // ==================== isUnit() Tests ====================

    @Test
    public void isUnit_sameUnit_returnsTrue()
    {
        Distance d = new Distance(5.0, DistanceUnit.METER);
        assertTrue(d.isUnit(DistanceUnit.METER));
    }

    @Test
    public void isUnit_differentUnit_returnsFalse()
    {
        Distance d = new Distance(5.0, DistanceUnit.METER);
        assertFalse(d.isUnit(DistanceUnit.CM));
        assertFalse(d.isUnit(DistanceUnit.MM));
        assertFalse(d.isUnit(DistanceUnit.INCH));
    }

    // ==================== equals() Tests ====================

    @Test
    public void equals_sameUnitSameMagnitude_returnsTrue()
    {
        Distance a = new Distance(5.0, DistanceUnit.METER);
        Distance b = new Distance(5.0, DistanceUnit.METER);
        assertTrue(a.equals(b));
    }

    @Test
    public void equals_differentUnitSameDistance_returnsTrue()
    {
        Distance a = new Distance(1.0, DistanceUnit.METER);
        Distance b = new Distance(100.0, DistanceUnit.CM);
        assertTrue(a.equals(b));
    }

    @Test
    public void equals_differentUnitSameDistance_inchAndCm_returnsTrue()
    {
        Distance a = new Distance(1.0, DistanceUnit.INCH);
        Distance b = new Distance(2.54, DistanceUnit.CM);
        assertTrue(a.equals(b));
    }

    @Test
    public void equals_differentMagnitude_returnsFalse()
    {
        Distance a = new Distance(5.0, DistanceUnit.METER);
        Distance b = new Distance(6.0, DistanceUnit.METER);
        assertFalse(a.equals(b));
    }

    @Test
    public void equals_nearlyEqualWithinTolerance_returnsTrue()
    {
        Distance a = new Distance(1.0, DistanceUnit.METER);
        Distance b = new Distance(1.0 + 1e-10, DistanceUnit.METER);
        assertTrue(a.equals(b));
    }

    @Test
    public void equals_nearlyEqualOutsideTolerance_returnsFalse()
    {
        Distance a = new Distance(1.0, DistanceUnit.METER);
        Distance b = new Distance(1.0 + 1e-8, DistanceUnit.METER);
        assertFalse(a.equals(b));
    }

    @Test
    public void equals_nonDistanceObject_returnsFalse()
    {
        Distance d = new Distance(5.0, DistanceUnit.METER);
        assertFalse(d.equals("5.0 meters"));
        assertFalse(d.equals(5.0));
        assertFalse(d.equals(null));
    }

    @Test
    public void equals_zeroInDifferentUnits_returnsTrue()
    {
        Distance a = new Distance(0.0, DistanceUnit.METER);
        Distance b = new Distance(0.0, DistanceUnit.INCH);
        assertTrue(a.equals(b));
    }

    // ==================== compareTo() Tests ====================

    @Test
    public void compareTo_equal_returnsZero()
    {
        Distance a = new Distance(5.0, DistanceUnit.METER);
        Distance b = new Distance(5.0, DistanceUnit.METER);
        assertEquals(0.0, a.compareTo(b), DELTA);
    }

    @Test
    public void compareTo_equalDifferentUnits_returnsZero()
    {
        Distance a = new Distance(1.0, DistanceUnit.METER);
        Distance b = new Distance(100.0, DistanceUnit.CM);
        assertEquals(0.0, a.compareTo(b), DELTA);
    }

    @Test
    public void compareTo_greater_returnsPositive()
    {
        Distance a = new Distance(6.0, DistanceUnit.METER);
        Distance b = new Distance(5.0, DistanceUnit.METER);
        assertTrue(a.compareTo(b) > 0);
    }

    @Test
    public void compareTo_lesser_returnsNegative()
    {
        Distance a = new Distance(4.0, DistanceUnit.METER);
        Distance b = new Distance(5.0, DistanceUnit.METER);
        assertTrue(a.compareTo(b) < 0);
    }

    @Test
    public void compareTo_differentUnits_greater()
    {
        Distance a = new Distance(2.0, DistanceUnit.METER);
        Distance b = new Distance(100.0, DistanceUnit.CM);
        assertTrue(a.compareTo(b) > 0);
    }

    @Test
    public void compareTo_differentUnits_lesser()
    {
        Distance a = new Distance(50.0, DistanceUnit.CM);
        Distance b = new Distance(1.0, DistanceUnit.METER);
        assertTrue(a.compareTo(b) < 0);
    }

    // ==================== toString() Tests ====================

    @Test
    public void toString_meter()
    {
        Distance d = new Distance(5.0, DistanceUnit.METER);
        String result = d.toString();
        assertNotNull(result);
        assertTrue(result.contains("5"));
    }

    @Test
    public void toString_cm()
    {
        Distance d = new Distance(100.0, DistanceUnit.CM);
        String result = d.toString();
        assertNotNull(result);
        assertTrue(result.contains("100"));
    }

    @Test
    public void toString_mm()
    {
        Distance d = new Distance(1000.0, DistanceUnit.MM);
        String result = d.toString();
        assertNotNull(result);
        assertTrue(result.contains("1000"));
    }

    @Test
    public void toString_inch()
    {
        Distance d = new Distance(12.0, DistanceUnit.INCH);
        String result = d.toString();
        assertNotNull(result);
        assertTrue(result.contains("12"));
    }

    // ==================== Edge Cases ====================

    @Test
    public void edgeCase_veryLargeMagnitude()
    {
        Distance d = new Distance(1e15, DistanceUnit.METER);
        assertEquals(1e15, d.magnitude, 1e6);
        Distance converted = d.toUnit(DistanceUnit.MM);
        assertEquals(1e18, converted.magnitude, 1e9);
    }

    @Test
    public void edgeCase_verySmallMagnitude()
    {
        Distance d = new Distance(1e-15, DistanceUnit.METER);
        assertEquals(1e-15, d.magnitude, 1e-24);
        Distance converted = d.toUnit(DistanceUnit.MM);
        assertEquals(1e-12, converted.magnitude, 1e-21);
    }

    @Test
    public void edgeCase_chainedConversions()
    {
        Distance d = new Distance(1.0, DistanceUnit.METER);
        Distance result = d.toUnit(DistanceUnit.CM)
                          .toUnit(DistanceUnit.MM)
                          .toUnit(DistanceUnit.INCH)
                          .toUnit(DistanceUnit.METER);
        assertEquals(1.0, result.magnitude, 1e-6);
    }

    @Test
    public void edgeCase_chainedArithmetic()
    {
        Distance d = new Distance(10.0, DistanceUnit.METER);
        Distance result = d.plus(new Distance(5.0, DistanceUnit.METER))
                          .minus(new Distance(3.0, DistanceUnit.METER))
                          .multiply(2.0)
                          .divide(4.0);
        assertEquals(6.0, result.magnitude, DELTA);
    }

    @Test
    public void edgeCase_infinityMagnitude()
    {
        Distance d = new Distance(Double.POSITIVE_INFINITY, DistanceUnit.METER);
        assertEquals(Double.POSITIVE_INFINITY, d.magnitude, 0);
    }

    @Test
    public void edgeCase_nanMagnitude()
    {
        Distance d = new Distance(Double.NaN, DistanceUnit.METER);
        assertTrue(Double.isNaN(d.magnitude));
    }
}

