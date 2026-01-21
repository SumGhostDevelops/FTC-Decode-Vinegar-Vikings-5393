package org.firstinspires.ftc.teamcode.util.measure.geometry;

import org.firstinspires.ftc.teamcode.util.measure.coordinate.Coordinate;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

import java.util.Arrays;

public class Polygon
{
    public final Coordinate[] points;

    public Polygon(Coordinate... point)
    {
        points = (Coordinate[]) Arrays.stream(point).toArray();
    }

    public Coordinate getCenter()
    {
        Distance averageX = Distance.ZERO;
        Distance averageY = Distance.ZERO;

        for (Coordinate c: points)
        {
            averageX.plus(c.x);
            averageY.plus(c.y);
        }

        averageX = averageX.divide(points.length);
        averageY = averageY.divide(points.length);

        return new Coordinate(averageX, averageY);
    }

    public BoundingBox getBoundingBox()
    {
        return null;
    }

    public boolean intersects(Polygon other)
    {
        return false;
    }
}
