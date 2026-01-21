package org.firstinspires.ftc.teamcode.util.measure.geometry;

import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

public class Square extends Rectangle
{
    public Square(Distance length, Pose2d orientation)
    {
        super(length, length);
    }
}
