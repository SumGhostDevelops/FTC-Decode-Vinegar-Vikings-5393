package org.firstinspires.ftc.teamcode.util.measure.angle.field;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.CoordinateSystem;
import org.junit.Test;
import static org.junit.Assert.*;

public class FieldHeadingUnitTests
{
    private static final double DELTA = 1e-9;

    @Test
    public void constructor_createsCorrectStructure()
    {
        FieldHeading fh = new FieldHeading(90, AngleUnit.DEGREES, CoordinateSystem.DECODE_FTC);
        assertEquals(90.0, fh.angle.measure, DELTA);
        assertEquals(CoordinateSystem.DECODE_FTC, fh.system);
    }

    @Test
    public void toSystem_PedroToFTC_ZeroHeading()
    {
        // Pedro 0 = Facing Blue
        FieldHeading pedroHeading = new FieldHeading(0, AngleUnit.DEGREES, CoordinateSystem.DECODE_PEDROPATH);

        FieldHeading ftcHeading = pedroHeading.toSystem(CoordinateSystem.DECODE_FTC);

        // FTC 90 = Facing Blue (since FTC 0 = Audience)
        assertEquals(90.0, ftcHeading.angle.measure, DELTA);
        assertEquals(CoordinateSystem.DECODE_FTC, ftcHeading.system);
    }

    @Test
    public void toSystem_FTCToPedro_ZeroHeading()
    {
        // FTC 0 = Facing Audience
        FieldHeading ftcHeading = new FieldHeading(0, AngleUnit.DEGREES, CoordinateSystem.DECODE_FTC);

        FieldHeading pedroHeading = ftcHeading.toSystem(CoordinateSystem.DECODE_PEDROPATH);

        // Pedro Y = Backstage. Audience is -Backstage (-Y).
        // Pedro X = Blue.
        // Facing Audience is -Y in Pedro.
        // -90 degrees.
        assertEquals(-90.0, pedroHeading.angle.measure, DELTA);
    }

    @Test
    public void plus_handlesDifferentSystems()
    {
        // 10 deg in Pedro (slightly Left of Blue)
        FieldHeading base = new FieldHeading(10, AngleUnit.DEGREES, CoordinateSystem.DECODE_PEDROPATH);

        // 10 deg in FTC (slightly Blue of Audience)
        // FTC 10 -> Pedro (-90 + 10) = -80.
        FieldHeading addend = new FieldHeading(10, AngleUnit.DEGREES, CoordinateSystem.DECODE_FTC);

        // Result in Pedro: 10 + (-80) = -70.
        FieldHeading result = base.plus(addend);

        assertEquals(CoordinateSystem.DECODE_PEDROPATH, result.system);
        assertEquals(-70.0, result.angle.measure, DELTA);
    }

    @Test
    public void toAngleUnit_convertsInternalAngle()
    {
        FieldHeading deg = new FieldHeading(180, AngleUnit.DEGREES, CoordinateSystem.DECODE_FTC);
        FieldHeading rad = deg.toAngleUnit(AngleUnit.RADIANS);

        assertEquals(Math.PI, rad.angle.measure, DELTA);
        assertEquals(AngleUnit.RADIANS, rad.angle.unit);
    }
}