package org.firstinspires.ftc.teamcode.definitions.localization;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

/**
 * All coordinates have their pose facing up or toward the goal
 */
@Configurable
public class CornersCoordinates
{
    private static final DistanceUnit INCH = DistanceUnit.INCH;
    private static final AngleUnit DEGREES = AngleUnit.DEGREES;

    public static final Pose2d BLUE_LOADING_ZONE = new Pose2d(new FieldCoordinate(new Distance(136, INCH), new Distance(8.625, INCH), FieldCoordinate.CoordinateSystem.RIGHT_HAND), new Angle(180, DEGREES));
    public static final Pose2d RED_LOADING_ZONE = new Pose2d(new FieldCoordinate(new Distance(8, INCH), new Distance(8.625, INCH), FieldCoordinate.CoordinateSystem.RIGHT_HAND), new Angle(0, DEGREES));

    public static final Pose2d BLUE_GOAL = new Pose2d(new FieldCoordinate(new Distance(23.35, INCH), new Distance(127.41, INCH), FieldCoordinate.CoordinateSystem.RIGHT_HAND), new Angle(144, DEGREES));
    public static final Pose2d RED_GOAL = new Pose2d(new FieldCoordinate(new Distance(120.74, INCH), new Distance(127.41, INCH), FieldCoordinate.CoordinateSystem.RIGHT_HAND), new Angle(36, DEGREES));

    public static final Pose2d SMALL_TRIANGLE = new Pose2d(new FieldCoordinate(new Distance(72, INCH), new Distance(8.625, INCH), FieldCoordinate.CoordinateSystem.RIGHT_HAND), new Angle(90, DEGREES));
}