package org.firstinspires.ftc.teamcode.subsystems.odometry;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.constants.Team;
import org.firstinspires.ftc.teamcode.subsystems.odometry.modules.DeadwheelHandler;
import org.firstinspires.ftc.teamcode.subsystems.odometry.modules.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.odometry.modules.Webcam;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
import org.firstinspires.ftc.teamcode.util.measure.angle.UnnormalizedAngle;
import org.firstinspires.ftc.teamcode.util.measure.angle.Vector2d;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.CoordinateSystem;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Optional;

public class OdometryPinpoint extends SubsystemBase
{
    private final Pinpoint pinpoint;
    private final Webcam webcam;

    private static final AngleUnit aUnit = AngleUnit.DEGREES;
    private static final DistanceUnit dUnit = DistanceUnit.INCH;

    /**
     * Offset added to IMU yaw to get field-absolute heading.
     * After localize(): headingOffset = AprilTag heading - IMU yaw at that moment
     */
    private Angle headingOffset;

    /**
     * The field-absolute heading that the driver considers "forward" for field-centric driving.
     */
    private Angle driverForward;

    public OdometryPinpoint(Pinpoint pinpoint, Webcam webcam)
    {
        this(pinpoint, webcam, new Pose2d(new Distance(0, DistanceUnit.INCH), new Distance(0, DistanceUnit.INCH), new Angle(90, AngleUnit.DEGREES)));
    }

    public OdometryPinpoint(Pinpoint pinpoint, Webcam webcam, Pose2d referencePose)
    {
        this.pinpoint = pinpoint;
        this.webcam = webcam;

        this.pinpoint.setPosition(referencePose.toCoordinateSystem(CoordinateSystem.DECODE_FTC).toPose2D());
    }

    /**
     * @return The yaw, as reported directly by the Pinpoint
     */
    public Angle getIMUYaw()
    {
        return new Angle(pinpoint.getHeading(aUnit), aUnit);
    }

    /**
     * @return The absolute heading of the robot
     */
    public Angle getAngle()
    {
        return getIMUYaw().plus(headingOffset);
    }

    /**
     * @return A heading where forward is 0
     */
    public Angle getDriverHeading()
    {
        return getAngle().minus(driverForward);
    }

    /**
     * @return The field coordinate of the robot
     */
    public FieldCoordinate getFieldCoord()
    {
        return new FieldCoordinate(
                new Distance(pinpoint.getPosX(dUnit), dUnit),
                new Distance(pinpoint.getPosY(dUnit), dUnit),
                CoordinateSystem.DECODE_FTC
        );
    }

    /**
     * @return The pose of the robot
     */
    public Pose2d getPose()
    {
        return Pose2d.fromPose2D(pinpoint.getPosition(), CoordinateSystem.DECODE_FTC);
    }

    /**
     * @return A vector of the velocity
     */
    public Vector2d getVelocity()
    {
        return new Vector2d(
                new Distance(pinpoint.getVelX(dUnit), dUnit),
                new Distance(pinpoint.getVelY(dUnit), dUnit),
                CoordinateSystem.DECODE_FTC
        );
    }

    /**
     * @return The velocity of the robot
     */
    public UnnormalizedAngle getHeadingVelocity()
    {
        return new UnnormalizedAngle(pinpoint.getHeadingVelocity(aUnit.getUnnormalized()), aUnit.getUnnormalized());
    }

    /**
     * Sets the current field-absolute heading as the driver's "forward" direction.
     * Call this AFTER localization to set which direction the driver considers forward
     * for field-centric driving, without affecting the absolute heading calibration.
     */
    public void setDriverForwardFromCurrent()
    {
        driverForward = getAngle();
    }

    public void updateReferencePose(Pose2d referencePose)
    {
        headingOffset = referencePose.heading;
        driverForward = referencePose.heading;

        pinpoint.setPosition(referencePose.toPose2D());
    }

    /**
     * Localizes using an AprilTag
     * @return If the localization was successful or not
     */
    public boolean localizeWithAprilTag()
    {
        // verify the apriltag exists
        webcam.updateDetections();

        Optional<AprilTagDetection> possibleTag = webcam.goal.getAny();
        if (possibleTag.isEmpty()) return false;

        // get the apriltag and the pose it has estimated
        // Note: robotPose already accounts for camera offset since we configured
        // the AprilTagProcessor with setCameraPose() in the Webcam class
        AprilTagDetection tag = possibleTag.get();
        Pose2d estimatedPose = Pose2d.fromPose3DWebcam(tag.robotPose);

        // Preserve driver's relative heading before we change headingOffset
        // currentDriverHeading = getAngle() - driverForward
        Angle currentDriverHeading = getAngle().minus(driverForward);

        // Compute new headingOffset so getAngle() returns true field-absolute heading
        // headingOffset = estimatedPose.heading - currentIMUYaw
        Angle currentImuYaw = getIMUYaw();
        headingOffset = estimatedPose.heading.minus((currentImuYaw));

        // Update driverForward so getDriverHeading() still returns the same value
        // getDriverHeading() = getAngle() - driverForward = currentDriverHeading
        // driverForward = getAngle() - currentDriverHeading = estimatedPose.heading - currentDriverHeading
        driverForward = estimatedPose.heading.toUnit(aUnit).minus(currentDriverHeading);

        // set pinpoint to track from new estimated pose
        pinpoint.setPosition(estimatedPose.toPose2D());

        return true;
    }

    /**
     * Localizes using an AprilTag, and automatically sets the driver forward direction (if enabled)
     * @param team
     * @return If localization was successful or not
     */
    public boolean localizeWithAprilTag(Team team)
    {
        if (!localizeWithAprilTag())
        {
            return false;
        }

        if (RobotConstants.Odometry.SET_FORWARD_DIRECTION_BASED_ON_TEAM)
        {
            driverForward = team.forwardAngle;
        }

        return true;
    }

    @Override
    public void periodic()
    {
        pinpoint.update();
    }

    public void close()
    {
        webcam.close();
    }
}