package org.firstinspires.ftc.teamcode.util.measure.geometry;

import org.firstinspires.ftc.teamcode.util.measure.coordinate.Coordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

public class Rectangle extends Polygon
{
    public final Distance length;
    public final Distance width;

    public Rectangle(Distance length, Distance width)
    {
        super(new Coordinate(Distance.ZERO, Distance.ZERO), new Coordinate(width, Distance.ZERO), new Coordinate(width, length), new Coordinate(Distance.ZERO, length));
        this.length = length;
        this.width = width;
    }
}
