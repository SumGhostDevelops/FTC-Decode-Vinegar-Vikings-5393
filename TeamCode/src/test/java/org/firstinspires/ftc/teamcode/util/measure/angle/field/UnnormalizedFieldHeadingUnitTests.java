package org.firstinspires.ftc.teamcode.util.measure.angle.field;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.CoordinateSystem;
import org.junit.Test;
import static org.junit.Assert.*;

public class UnnormalizedFieldHeadingUnitTests
{
    private static final double DELTA = 1e-9;

    @Test
    public void toSystem_preservesWinding()
    {
        // 360 degrees in Pedro (Full rotation, facing Blue)
        UnnormalizedFieldHeading pedro = new UnnormalizedFieldHeading(
                360, UnnormalizedAngleUnit.DEGREES, CoordinateSystem.DECODE_PEDROPATH
        );

        UnnormalizedFieldHeading ftc = pedro.toSystem(CoordinateSystem.DECODE_FTC);

        // Pedro 0 -> FTC 90.
        // Pedro 360 -> FTC 360 + 90 = 450.
        assertEquals(450.0, ftc.angle.measure, DELTA);
    }

    @Test
    public void toSystem_preservesNegativeWinding()
    {
        // -360 degrees in Pedro (Full rotation CW, facing Blue)
        UnnormalizedFieldHeading pedro = new UnnormalizedFieldHeading(
                -360, UnnormalizedAngleUnit.DEGREES, CoordinateSystem.DECODE_PEDROPATH
        );

        UnnormalizedFieldHeading ftc = pedro.toSystem(CoordinateSystem.DECODE_FTC);

        // Pedro 0 -> FTC 90.
        // Pedro -360 -> FTC -360 + 90 = -270.
        assertEquals(-270.0, ftc.angle.measure, DELTA);
    }

    @Test
    public void plus_preservesWindingAcrossSystems()
    {
        UnnormalizedFieldHeading base = new UnnormalizedFieldHeading(
                0, UnnormalizedAngleUnit.DEGREES, CoordinateSystem.DECODE_PEDROPATH
        );

        // FTC 720 (2 rotations facing Audience)
        // FTC 0 -> Pedro -90.
        // FTC 720 -> Pedro 720 - 90 = 630.
        UnnormalizedFieldHeading add = new UnnormalizedFieldHeading(
                720, UnnormalizedAngleUnit.DEGREES, CoordinateSystem.DECODE_FTC
        );

        // 0 + 630 = 630.
        UnnormalizedFieldHeading result = base.plus(add);
        assertEquals(630.0, result.angle.measure, DELTA);
    }
}