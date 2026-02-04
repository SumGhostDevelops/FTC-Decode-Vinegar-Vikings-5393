package org.firstinspires.ftc.teamcode.util.measure.coordinate;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

public enum CoordinateSystem {
    // FTC Standard: 0,0 at field center.
    DECODE_FTC(Direction.AUDIENCE, Direction.BLUE, Direction.UP,
            new Coordinate(new Distance(0, DistanceUnit.INCH), new Distance(0, DistanceUnit.INCH))),

    // PedroPath: Field center is at (72, 72).
    DECODE_PEDROPATH(Direction.BLUE, Direction.BACKSTAGE, Direction.UP,
            new Coordinate(new Distance(72, DistanceUnit.INCH), new Distance(72, DistanceUnit.INCH))),

    // AprilTag SDK output after (y, -x) transform: +X = Blue, +Y = Backstage,
    // center at (0,0)
    // Same axes as PedroPath, used as intermediate for AprilTag data conversion
    APRILTAG_SDK(Direction.BLUE, Direction.BACKSTAGE, Direction.UP,
            new Coordinate(new Distance(0, DistanceUnit.INCH), new Distance(0, DistanceUnit.INCH))),

    // Safe default for center to avoid NPEs if used accidentally
    GENERIC(Direction.UP, Direction.UP, Direction.UP,
            new Coordinate(new Distance(0, DistanceUnit.INCH), new Distance(0, DistanceUnit.INCH)));

    public final Direction positiveX;
    public final Direction positiveY;
    public final Direction positiveZ;
    public final Coordinate center;
    // Cache basis vectors to avoid switch statements at runtime
    private final double[] xBasis;
    private final double[] yBasis;

    CoordinateSystem(Direction positiveX, Direction positiveY, Direction positiveZ, Coordinate center) {
        this.positiveX = positiveX;
        this.positiveY = positiveY;
        this.positiveZ = positiveZ;
        this.center = center;

        // Pre-calculate basis vectors
        this.xBasis = calculateBasis(positiveX);
        this.yBasis = calculateBasis(positiveY);
    }

    public static double[] calculateBasis(Direction dir) {
        switch (dir) {
            case AUDIENCE:
                return new double[] { 1.0, 0.0 };
            case BACKSTAGE:
                return new double[] { -1.0, 0.0 };
            case BLUE:
                return new double[] { 0.0, 1.0 };
            case RED:
                return new double[] { 0.0, -1.0 };
            default:
                return new double[] { 0.0, 0.0 }; // Default identity
        }
    }

    public Coordinate toUniversal(Distance localX, Distance localY) {
        if (this == GENERIC)
            return new Coordinate(localX, localY);

        // Optimization: Use raw double math to avoid creating 3-4 intermediate Distance
        // objects
        // We normalize to INCHES for the math, then wrap at the end.
        double xInch = localX.getDistance(DistanceUnit.INCH);
        double yInch = localY.getDistance(DistanceUnit.INCH);
        double centerX = center.x.getDistance(DistanceUnit.INCH);
        double centerY = center.y.getDistance(DistanceUnit.INCH);

        // 1. Offset
        double offX = xInch - centerX;
        double offY = yInch - centerY;

        // 2. Rotate (Dot Product)
        double globalXVal = offX * xBasis[0] + offY * yBasis[0];
        double globalYVal = offX * xBasis[1] + offY * yBasis[1];

        return new Coordinate(
                new Distance(globalXVal, DistanceUnit.INCH),
                new Distance(globalYVal, DistanceUnit.INCH));
    }

    public Coordinate fromUniversal(Distance globalX, Distance globalY) {
        if (this == GENERIC)
            return new Coordinate(globalX, globalY);

        double gX = globalX.getDistance(DistanceUnit.INCH);
        double gY = globalY.getDistance(DistanceUnit.INCH);
        double centerX = center.x.getDistance(DistanceUnit.INCH);
        double centerY = center.y.getDistance(DistanceUnit.INCH);

        // Inverse rotation
        double localOffX = gX * xBasis[0] + gY * xBasis[1];
        double localOffY = gX * yBasis[0] + gY * yBasis[1];

        // Add Center
        return new Coordinate(
                new Distance(localOffX + centerX, DistanceUnit.INCH),
                new Distance(localOffY + centerY, DistanceUnit.INCH));
    }

    public Coordinate fromUniversal(Coordinate globalCoordinate) {
        return fromUniversal(globalCoordinate.x, globalCoordinate.y);
    }

    public double getRotationOffsetRadians() {
        // atan2(y component of X-basis, x component of X-basis)
        // This tells us the angle of this system's X-axis relative to the Universal
        // X-axis
        return Math.atan2(xBasis[1], xBasis[0]);
    }

    public enum Direction {
        RED, BLUE, AUDIENCE, BACKSTAGE, UP, DOWN
    }
}