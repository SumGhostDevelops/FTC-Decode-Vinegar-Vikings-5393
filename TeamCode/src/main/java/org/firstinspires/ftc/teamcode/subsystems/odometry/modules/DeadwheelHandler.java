package org.firstinspires.ftc.teamcode.subsystems.odometry.modules;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

public class DeadwheelHandler {
    // Constants
    private final double ENCODER_TICKS_PER_REVOLUTION = RobotConstants.Odometry.Deadwheels.COUNTS_PER_REVOLUTION;
    private final double ENCODER_WHEEL_CIRCUMFERENCE = RobotConstants.Odometry.Deadwheels.WHEEL_CIRCUMFERENCE.toUnit(DistanceUnit.INCH).magnitude;

    // Distance from the center to the FORWARD wheel along the Y-axis (LATERAL)
    // Positive if LEFT of center, Negative if RIGHT of center
    private final double FORWARD_WHEEL_Y_OFFSET = RobotConstants.Odometry.Deadwheels.Forward.OFFSET.toUnit(DistanceUnit.INCH).magnitude;

    // Distance from the center to the STRAFE wheel along the X-axis (LONGITUDINAL)
    // Positive if FORWARD of center, Negative if BACKWARD of center
    private final double STRAFE_WHEEL_X_OFFSET = RobotConstants.Odometry.Deadwheels.Strafe.OFFSET.toUnit(DistanceUnit.INCH).magnitude;

    // Variables
    private Pose2d pose;
    private double lastForwardEnc = 0, lastStrafeEnc = 0;
    private double lastAngleDeg = 0;

    public DeadwheelHandler(Pose2d pose) {
        this.pose = pose.toDistanceUnit(DistanceUnit.INCH).toAngleUnit(AngleUnit.DEGREES);
        this.lastAngleDeg = this.pose.heading.toUnit(AngleUnit.DEGREES).measure;
    }

    public DeadwheelHandler(Pose2d pose, double forwardEncBaseline, double strafeEncBaseline) {
        this(pose);
        this.lastForwardEnc = forwardEncBaseline;
        this.lastStrafeEnc = strafeEncBaseline;
    }

    public void resetEncoders(double forwardEncBaseline, double strafeEncBaseline) {
        this.lastForwardEnc = forwardEncBaseline;
        this.lastStrafeEnc = strafeEncBaseline;
    }

    /**
     * Updates the robot's pose based on encoder deltas and IMU heading.
     *
     * @param forwardEncoderTicks Ticks from the Forward/Backward pod
     * @param strafeEncoderTicks  Ticks from the Left/Right (Strafe) pod
     * @param currentAngle        Current absolute heading from IMU
     */
    public void updatePose(double forwardEncoderTicks, double strafeEncoderTicks, Angle currentAngle) {
        double currentAngleDeg = currentAngle.toUnit(AngleUnit.DEGREES).measure;

        // 1. Calculate Deltas
        double dForwardTicks = forwardEncoderTicks - lastForwardEnc;
        double dStrafeTicks = strafeEncoderTicks - lastStrafeEnc;

        // Calculate angle delta handling wrapping (-180 to 180)
        double deltaAngleDeg = AngleUnit.normalizeDegrees(currentAngleDeg - lastAngleDeg);
        double deltaAngleRad = Math.toRadians(deltaAngleDeg);

        // Update state for next loop
        lastForwardEnc = forwardEncoderTicks;
        lastStrafeEnc = strafeEncoderTicks;
        lastAngleDeg = currentAngleDeg;

        // 2. Convert encoder ticks to distance (inches)
        double rawForwardDist = dForwardTicks * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;
        double rawStrafeDist = dStrafeTicks * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;

        // 3. Arc Compensation [CORRECTED]
        // Forward Wheel (Standard: Left is positive Offset)
        // We ADD the offset component because the wheel spins backward during positive rotation.
        double robotDeltaX = rawForwardDist + (FORWARD_WHEEL_Y_OFFSET * deltaAngleRad);

        // Strafe Wheel (Standard: Forward is positive Offset)
        // We SUBTRACT the offset component because the wheel spins positive (left) during positive rotation.
        double robotDeltaY = rawStrafeDist - (STRAFE_WHEEL_X_OFFSET * deltaAngleRad);


        // 4. Field Centric Conversion using Pose Exponential
        // This provides mathematically exact integration for constant curvature motion
        double fieldDeltaX, fieldDeltaY;

        // Always use start heading for consistency (pose at beginning of this update)
        double startHeadingRad = Math.toRadians(currentAngleDeg - deltaAngleDeg);

        // Compute sinc(θ) and cosc(θ) with Taylor series for small angles
        // This avoids numerical instability near zero while maintaining accuracy
        double sincTheta, coscTheta;
        double halfDelta = deltaAngleRad / 2.0;

        if (Math.abs(deltaAngleRad) < 1e-4) {
            // Taylor series approximations for small angles (more terms for accuracy)
            // sinc(θ) ≈ 1 - θ²/6 + θ⁴/120
            // cosc(θ) ≈ θ/2 - θ³/24 + θ⁵/720
            double theta2 = deltaAngleRad * deltaAngleRad;
            double theta4 = theta2 * theta2;
            sincTheta = 1.0 - theta2 / 6.0 + theta4 / 120.0;
            coscTheta = halfDelta * (1.0 - theta2 / 12.0 + theta4 / 360.0);
        } else {
            // Standard computation for larger angles
            sincTheta = Math.sin(deltaAngleRad) / deltaAngleRad;
            coscTheta = (1.0 - Math.cos(deltaAngleRad)) / deltaAngleRad;
        }

        // Transform robot-frame deltas through the exponential map
        // This accounts for the curved path during rotation
        double rotatedX = (sincTheta * robotDeltaX) - (coscTheta * robotDeltaY);
        double rotatedY = (coscTheta * robotDeltaX) + (sincTheta * robotDeltaY);

        // Rotate from robot frame to field frame using starting heading
        double sin = Math.sin(startHeadingRad);
        double cos = Math.cos(startHeadingRad);

        fieldDeltaX = (rotatedX * cos) - (rotatedY * sin);
        fieldDeltaY = (rotatedX * sin) + (rotatedY * cos);

        // 5. Update Pose
        pose = new Pose2d(
                new FieldCoordinate(
                        pose.coord.x.plus(new Distance(fieldDeltaX, DistanceUnit.INCH)),
                        pose.coord.y.plus(new Distance(fieldDeltaY, DistanceUnit.INCH)),
                        pose.coord.coordSys
                ),
                new Angle(currentAngleDeg, AngleUnit.DEGREES)
        );
    }

    public Pose2d getPose() {
        return pose;
    }

    public FieldCoordinate getCoord() {
        return pose.coord;
    }
}