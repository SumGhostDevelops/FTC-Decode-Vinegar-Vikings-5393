package org.firstinspires.ftc.teamcode.subsystems.odometry;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Thread.sleep;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.constants.Team;
import org.firstinspires.ftc.teamcode.subsystems.odometry.modules.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.odometry.modules.Webcam;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;
import org.firstinspires.ftc.teamcode.util.measure.angle.field.FieldHeading;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.UnnormalizedAngle;
import org.firstinspires.ftc.teamcode.util.measure.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.CoordinateSystem;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Optional;

public class Odometry extends SubsystemBase
{
    private final Pinpoint pinpoint;
    private final Webcam webcam;

    private static final AngleUnit aUnit = AngleUnit.DEGREES;
    private static final DistanceUnit dUnit = DistanceUnit.INCH;

    /**
     * The field-absolute heading that the driver considers "forward" for field-centric driving.
     */
    private FieldHeading driverForward;

    public Odometry(Pinpoint pinpoint, WebcamName webcam)
    {
        this(pinpoint, webcam, new Pose2d(new Distance(72, DistanceUnit.INCH), new Distance(72, DistanceUnit.INCH), new Angle(90, AngleUnit.DEGREES), CoordinateSystem.DECODE_PEDROPATH));
    }

    public Odometry(Pinpoint pinpoint, WebcamName webcam, Pose2d referencePose)
    {
        this.pinpoint = pinpoint;
        this.webcam = new Webcam(webcam);

        long startTime = System.currentTimeMillis();
        while (this.pinpoint.getDeviceStatus() != Pinpoint.DeviceStatus.READY
                && (System.currentTimeMillis() - startTime) < 1000)
        {
            this.pinpoint.update();
        }

        this.pinpoint.setPosition(referencePose.toCoordinateSystem(CoordinateSystem.DECODE_FTC).toPose2D());
        this.pinpoint.update();

        this.driverForward = referencePose.heading;
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
    public FieldHeading getFieldAngle()
    {
        return new FieldHeading(getIMUYaw(), CoordinateSystem.DECODE_FTC);
    }

    /**
     * @return A heading where forward is 0
     */
    public Angle getDriverHeading()
    {
        return getFieldAngle().minus(driverForward).angle.minus(new Angle(90, AngleUnit.DEGREES));
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
        driverForward = getFieldAngle();
    }

    public void updateReferencePose(Pose2d referencePose)
    {
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

        AprilTagDetection tag = possibleTag.get();
        Pose2d estimatedPose = Pose2d.fromPose3D(tag.robotPose, CoordinateSystem.DECODE_FTC);

        // Preserve driver's relative heading before resetting hardware
        FieldHeading currentDriverHeading = getFieldAngle().minus(driverForward);

        // Update driverForward based on the new absolute estimate
        driverForward = estimatedPose.heading.minus(currentDriverHeading);

        // Set hardware to the new estimated pose (translation and rotation)
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