package org.firstinspires.ftc.teamcode.util.measure.coordinate;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.firstinspires.ftc.teamcode.util.measure.geometry.Vector2d;
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
    public void constructor_twoArgs_defaultsToGenericOrProvided()
    {
        Distance x = new Distance(3.0, DistanceUnit.METER);
        Distance y = new Distance(4.0, DistanceUnit.METER);
        // Default behavior depends on impl, but usually we use the explicit 3-arg constructor in tests
        FieldCoordinate coord = new FieldCoordinate(x, y, CoordinateSystem.DECODE_PEDROPATH);

        assertEquals(3.0, coord.x.magnitude, DELTA);
        assertEquals(4.0, coord.y.magnitude, DELTA);
        assertEquals(CoordinateSystem.DECODE_PEDROPATH, coord.coordSys);
    }

    // ==================== toCoordinateSystem() Tests ====================

    @Test
    public void toCoordinateSystem_pedroToFtcStd_Center()
    {
        // In PEDRO, (72, 72) inches is the center of the field
        // In FTC_STD, that should be (0, 0)
        FieldCoordinate coord = new FieldCoordinate(
                new Distance(72.0, DistanceUnit.INCH),
                new Distance(72.0, DistanceUnit.INCH),
                CoordinateSystem.DECODE_PEDROPATH
        );

        FieldCoordinate converted = coord.toCoordinateSystem(CoordinateSystem.DECODE_FTC);
        assertEquals(0.0, converted.x.magnitude, DELTA);
        assertEquals(0.0, converted.y.magnitude, DELTA);
        assertEquals(CoordinateSystem.DECODE_FTC, converted.coordSys);
    }

    @Test
    public void toCoordinateSystem_ftcStdToPedro_Center()
    {
        // In FTC_STD, (0, 0) is the center of the field
        // In PEDRO, that should be (72, 72) inches
        FieldCoordinate coord = new FieldCoordinate(
                new Distance(0.0, DistanceUnit.INCH),
                new Distance(0.0, DistanceUnit.INCH),
                CoordinateSystem.DECODE_FTC
        );

        FieldCoordinate converted = coord.toCoordinateSystem(CoordinateSystem.DECODE_PEDROPATH);
        assertEquals(72.0, converted.x.magnitude, DELTA);
        assertEquals(72.0, converted.y.magnitude, DELTA);
        assertEquals(CoordinateSystem.DECODE_PEDROPATH, converted.coordSys);
    }

    @Test
    public void toCoordinateSystem_BasisRotationCheck()
    {
        // Pedro Basis: X = Blue, Y = Backstage
        // FTC Basis: X = Audience, Y = Blue

        // A vector (10, 0) in Pedro means "10 inches towards Blue"
        // In FTC, "Towards Blue" is +Y. So we expect (0, 10) roughly (ignoring center offset)

        FieldCoordinate originPedro = new FieldCoordinate(
                new Distance(72, DistanceUnit.INCH),
                new Distance(72, DistanceUnit.INCH),
                CoordinateSystem.DECODE_PEDROPATH
        ); // Center

        FieldCoordinate tenInchesTowardsBlue = new FieldCoordinate(
                new Distance(82, DistanceUnit.INCH), // 72 + 10
                new Distance(72, DistanceUnit.INCH),
                CoordinateSystem.DECODE_PEDROPATH
        );

        FieldCoordinate converted = tenInchesTowardsBlue.toCoordinateSystem(CoordinateSystem.DECODE_FTC);

        // FTC Center is (0,0). "Towards Blue" is +Y.
        assertEquals(0.0, converted.x.magnitude, DELTA);
        assertEquals(10.0, converted.y.magnitude, DELTA);
    }

    // ==================== angleTo() Tests ====================

    @Test
    public void angleTo_differentCoordinateSystems()
    {
        FieldCoordinate origin = new FieldCoordinate(
                new Distance(72.0, DistanceUnit.INCH),
                new Distance(72.0, DistanceUnit.INCH),
                CoordinateSystem.DECODE_PEDROPATH
        );
        // (1, 1) in FTC_STD (1 inch Right, 1 inch Forward/Blue)
        // Center relative to Center.
        FieldCoordinate other = new FieldCoordinate(
                new Distance(1.0, DistanceUnit.INCH),
                new Distance(1.0, DistanceUnit.INCH),
                CoordinateSystem.DECODE_FTC
        );

        // We convert 'other' to Pedro to perform the angle calc in the 'origin' system.
        // FTC (1, 1) -> Pedro:
        // X (Audience) = -Y (Pedro Backstage inverse?) -> Actually X (FTC) is Audience.
        // Pedro Y is Backstage. Audience is -Backstage. So FTC X = -Pedro Y.
        // FTC Y (Blue) = Pedro X.
        // So FTC(1, 1) vector corresponds to Pedro(1, -1) vector.

        // Angle of (1, -1) in Pedro system = atan2(-1, 1) = -45 degrees (-PI/4).
        Angle angle = origin.angleTo(other);
        assertEquals(-Math.PI / 4, angle.measure, DELTA);
    }

    @Test
    public void distanceTo_CrossSystem_CenterToCenter()
    {
        // FTC Center is (0, 0)
        FieldCoordinate centerFTC = new FieldCoordinate(
                new Distance(0, DistanceUnit.INCH),
                new Distance(0, DistanceUnit.INCH),
                CoordinateSystem.DECODE_FTC
        );

        // Pedro Center is (72, 72)
        FieldCoordinate centerPedro = new FieldCoordinate(
                new Distance(72, DistanceUnit.INCH),
                new Distance(72, DistanceUnit.INCH),
                CoordinateSystem.DECODE_PEDROPATH
        );

        // Distance should be 0, regardless of raw values
        assertEquals(0.0, centerFTC.distanceTo(centerPedro).magnitude, DELTA);
        assertEquals(0.0, centerPedro.distanceTo(centerFTC).magnitude, DELTA);
    }

    @Test
    public void angleTo_CrossSystem_Quadrants()
    {
        // Origin at Field Center
        FieldCoordinate center = new FieldCoordinate(
                new Distance(0, DistanceUnit.INCH),
                new Distance(0, DistanceUnit.INCH),
                CoordinateSystem.DECODE_FTC
        );

        // Target: 10 inches towards Blue Alliance (FTC +Y)
        // In Pedro, Blue is +X. Center is (72,72). So target is (82, 72).
        FieldCoordinate targetBlue = new FieldCoordinate(
                new Distance(82, DistanceUnit.INCH),
                new Distance(72, DistanceUnit.INCH),
                CoordinateSystem.DECODE_PEDROPATH
        );

        // Angle in FTC system: Towards +Y is 90 degrees (PI/2)
        Angle angleInFTC = center.angleTo(targetBlue);
        assertEquals(Math.PI / 2, angleInFTC.getRadians(), DELTA);

        // Angle in Pedro system: Towards +X is 0 degrees
        // center.toSystem(PEDRO) -> (72,72). targetBlue -> (82,72).
        // Vector is (10, 0). Angle is 0.
        Angle angleInPedro = center.toCoordinateSystem(CoordinateSystem.DECODE_PEDROPATH)
                .angleTo(targetBlue);
        assertEquals(0.0, angleInPedro.getRadians(), DELTA);
    }
}