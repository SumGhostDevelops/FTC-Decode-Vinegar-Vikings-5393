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

    // Distance from the center to the FORWARD wheel along the Y-axis (LATERAL)
    private final double FORWARD_WHEEL_Y_OFFSET = RobotConstants.Odometry.Deadwheels.Forward.OFFSET.toUnit(DistanceUnit.INCH).magnitude;

    // Distance from the center to the STRAFE wheel along the X-axis (LONGITUDINAL)
    private final double STRAFE_WHEEL_X_OFFSET = RobotConstants.Odometry.Deadwheels.Strafe.OFFSET.toUnit(DistanceUnit.INCH).magnitude;

    // Variables
    private Pose2d pose;
    private double lastLeftEnc = 0, lastNormalEnc = 0;
    private double lastAngle = 0;

    public DeadwheelHandler(Pose2d pose)
    {
        this.pose = pose.toDistanceUnit(DistanceUnit.INCH).toAngleUnit(AngleUnit.DEGREES);
        this.lastAngle = this.pose.heading.toUnit(AngleUnit.DEGREES).measure;
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
     * Parallel encoder measures forward/backward (Y in robot frame)
     * Perpendicular encoder measures left/right (X in robot frame)
     *
     * @param parallelEncoderTicks = ticks from the parallel (forward) odometry wheel
     * @param perpendicularEncoderTicks = ticks from the perpendicular (strafe) odometry wheel
     * @param angle = robot's heading angle (from IMU)
     */
    public void updatePose(double parallelEncoderTicks, double perpendicularEncoderTicks, Angle angle)
    {
        double ang = angle.toUnit(AngleUnit.DEGREES).measure;
        double dL = parallelEncoderTicks - lastLeftEnc; // Parallel delta
        double dN = perpendicularEncoderTicks - lastNormalEnc; // Perpendicular delta
        double dTheta = Math.toRadians(ang - lastAngle); // Heading change in radians

        lastNormalEnc = perpendicularEncoderTicks;
        lastLeftEnc = parallelEncoderTicks;
        lastAngle = ang;

        // 1. Convert encoder ticks to distance
        // Note: Removed the negative signs unless your encoders are mounted inverted
        double rawParallelDist = dL * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;
        double rawPerpDist = dN * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;

        // 2. Arc compensation (The r * theta fix)
        // dxR should be the Forward wheel corrected by its Y-offset
        // dyR should be the Strafe wheel corrected by its X-offset
        double dxR = rawParallelDist - (FORWARD_WHEEL_Y_OFFSET * dTheta);
        double dyR = rawPerpDist - (STRAFE_WHEEL_X_OFFSET * dTheta);

        // 3. Convert from robot-relative to field-relative coordinates
        // Standard Rotation Matrix for X (Forward) and Y (Strafe)
        double angRad = Math.toRadians(ang);
        double cos = Math.cos(angRad);
        double sin = Math.sin(angRad);

        // Field X = (RobotX * cos) - (RobotY * sin)
        // Field Y = (RobotX * sin) + (RobotY * cos)
        double dx = (dxR * cos) - (dyR * sin);
        double dy = (dxR * sin) + (dyR * cos);

        pose = new Pose2d(
                new FieldCoordinate(
                        pose.coord.x.plus(new Distance(dx, DistanceUnit.INCH)),
                        pose.coord.y.plus(new Distance(dy, DistanceUnit.INCH)),
                        pose.coord.coordSys
                ),
                new Angle(ang, AngleUnit.DEGREES)
        );
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
