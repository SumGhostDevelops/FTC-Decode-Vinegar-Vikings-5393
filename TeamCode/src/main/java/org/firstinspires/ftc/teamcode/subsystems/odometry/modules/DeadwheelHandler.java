package org.firstinspires.ftc.teamcode.subsystems.odometry.modules;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

public class DeadwheelHandler
{
    // Constants
    private final double ENCODER_TICKS_PER_REVOLUTION = RobotConstants.Odometry.Deadwheels.COUNTS_PER_REVOLUTION; // ticks measured after
    // one full revolution of the deadwheel
    private final double ENCODER_WHEEL_CIRCUMFERENCE = RobotConstants.Odometry.Deadwheels.WHEEL_CIRCUMFERENCE.toUnit(DistanceUnit.INCH).magnitude;

    // Variables
    private Pose2d pose;
    private double lastLeftEnc = 0, lastNormalEnc = 0;

    public DeadwheelHandler(Pose2d pose)
    {
        this.pose = pose.toDistanceUnit(DistanceUnit.INCH).toAngleUnit(AngleUnit.DEGREES);
    }

    /**
     * New constructor that allows providing initial encoder baselines so the first update uses deltas correctly.
     */
    public DeadwheelHandler(Pose2d pose, double leftEncBaseline, double normalEncBaseline)
    {
        this(pose);
        this.lastLeftEnc = leftEncBaseline;
        this.lastNormalEnc = normalEncBaseline;
    }

    /**
     * Optional reset of encoder baselines at runtime.
     */
    public void resetEncoders(double leftEncBaseline, double normalEncBaseline)
    {
        this.lastLeftEnc = leftEncBaseline;
        this.lastNormalEnc = normalEncBaseline;
    }

    // Two Deadwheel Odo

    /**
     * l = ticks from the parallel odometry wheel
     * r = ticks from the perpendicular odometry wheel
     * ang = robot's angle (Angle)
     */
    public void updatePose(double l, double n, Angle angle)
    {
        double ang = angle.toUnit(AngleUnit.DEGREES).measure;
        double dL = l - lastLeftEnc;
        double dN = n - lastNormalEnc;
        lastNormalEnc = n;
        lastLeftEnc = l;

        double leftDist = -dL * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;
        double dyR = leftDist;
        double dxR = -dN * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;

        double cos = Math.cos((Math.toRadians(ang)));
        double sin = Math.sin((Math.toRadians(ang)));
        double dx = (dxR * sin) + (dyR * cos);
        double dy = (-dxR * cos) + (dyR * sin);

        pose = new Pose2d(new FieldCoordinate(pose.coord.x.plus(new Distance(dx, DistanceUnit.INCH)), pose.coord.y.plus(new Distance(dy, DistanceUnit.INCH)), pose.coord.coordSys), new Angle(ang, AngleUnit.DEGREES));
    }

    public Pose2d getPose()
    {
        return pose;
    }

    public FieldCoordinate getCoord()
    {
        return pose.coord;
    }
}
