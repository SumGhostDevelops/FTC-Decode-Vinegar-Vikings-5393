package org.firstinspires.ftc.teamcode.subsystems.odometry;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.constants.Team;
import org.firstinspires.ftc.teamcode.subsystems.odometry.modules.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.odometry.modules.Webcam;
import org.firstinspires.ftc.teamcode.util.measure.angle.field.FieldHeading;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.UnnormalizedAngle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.CoordinateSystem;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.firstinspires.ftc.teamcode.util.measure.geometry.Vector2d;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Optional;
import java.util.function.BooleanSupplier;

public class Odometry extends SubsystemBase
{
    private final Pinpoint pinpoint;
    private final Webcam webcam;

    private final BooleanSupplier setForwardBasedOnTeam = () -> RobotConstants.Odometry.SET_FORWARD_DIRECTION_BASED_ON_TEAM;

    private static final AngleUnit aUnit = AngleUnit.DEGREES;
    private static final DistanceUnit dUnit = DistanceUnit.INCH;

    /**
     * The field-absolute heading that the driver considers "forward" for
     * field-centric driving.
     */
    private FieldHeading driverForward;
    private Pose2d cachedPose;

    private boolean referencePoseWasSet = false;
    private Pose2D referencePose;

    public Odometry(Pinpoint pinpoint, WebcamName webcam)
    {
        this(pinpoint, webcam, new Pose2d(new Distance(72, DistanceUnit.INCH), new Distance(72, DistanceUnit.INCH),
                new Angle(90, AngleUnit.DEGREES), CoordinateSystem.DECODE_PEDROPATH));
    }

    public Odometry(Pinpoint pinpoint, WebcamName webcam, Pose2d referencePose)
    {
        this.pinpoint = pinpoint;
        this.webcam = new Webcam(webcam);

        this.referencePose = referencePose.toCoordinateSystem(CoordinateSystem.DECODE_FTC).toPose2D();

        if (!referencePoseWasSet && this.pinpoint.getDeviceStatus() == Pinpoint.DeviceStatus.READY)
        {
            this.pinpoint.setPosition(this.referencePose);
            this.pinpoint.update();
            referencePoseWasSet = true;
        }

        this.cachedPose = referencePose;
        this.driverForward = referencePose.heading;
    }

    /**
     * @return The yaw, as reported directly by the Pinpoint
     */
    public Angle getIMUYaw()
    {
        return cachedPose.heading.toCoordinateSystem(CoordinateSystem.DECODE_FTC).angle;
    }

    /**
     * @return The absolute heading of the robot
     */
    public FieldHeading getFieldHeading()
    {
        return cachedPose.heading;
    }

    /**
     * @return A heading where 0 is conceptually "Forward" (aligned with Pedro
     *         X-Axis/Blue Alliance)
     */
    public Angle getDriverHeading()
    {
        FieldHeading currentPedro = getFieldHeading().toCoordinateSystem(CoordinateSystem.DECODE_PEDROPATH);
        FieldHeading startPedro = driverForward.toCoordinateSystem(CoordinateSystem.DECODE_PEDROPATH);

        return currentPedro.minus(startPedro).angle;
    }

    /**
     * @return The field coordinate of the robot
     */
    public FieldCoordinate getFieldCoord()
    {
        return cachedPose.coord;
    }

    /**
     * @return The pose of the robot
     */
    public Pose2d getPose()
    {
        return cachedPose;
    }

    /**
     * @return A vector of the velocity
     */
    public Vector2d getVelocity()
    {
        return new Vector2d(
                new Distance(pinpoint.getVelX(dUnit), dUnit),
                new Distance(pinpoint.getVelY(dUnit), dUnit),
                CoordinateSystem.DECODE_FTC);
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
     * Call this AFTER localization to set which direction the driver considers
     * forward
     * for field-centric driving, without affecting the absolute heading
     * calibration.
     */
    public void setDriverForwardFromCurrent()
    {
        driverForward = getFieldHeading();
    }

    /**
     * @param referencePose The reference pose to update the Pinpoint with.
     */
    public void setReferencePose(Pose2d referencePose)
    {
        referencePoseWasSet = false; // set the flag to false

        // update the desired reference pose
        this.referencePose = referencePose.toCoordinateSystem(CoordinateSystem.DECODE_FTC).toPose2D();

        // update the poses that get used
        cachedPose = referencePose;
        driverForward = referencePose.heading;

        setReferencePose();
    }

    /**
     * Attempt to send the reference pose to the Pinpoint
     */
    private void setReferencePose()
    {
        // don't update the pinpoint's reference pose if we already did it
        if (referencePoseWasSet)
        {
            return;
        }

        // don't try if the pinpoint isn't ready
        if (this.pinpoint.getDeviceStatus() != Pinpoint.DeviceStatus.READY)
        {
            return;
        }

        this.pinpoint.setPosition(this.referencePose); // set the reference pose on the pinpoint
        this.pinpoint.update(); // update the pinpoint
        this.referencePoseWasSet = true; // set the flag
    }

    /**
     * Localizes using an AprilTag
     *
     * @return If the localization was successful or not
     */
    public boolean localizeWithAprilTag()
    {
        // verify the apriltag exists
        webcam.updateDetections();

        Optional<AprilTagDetection> possibleTag = webcam.goal.getAny();
        if (possibleTag.isEmpty())
            return false;

        AprilTagDetection tag = possibleTag.get();
        Pose2d estimatedPose = Pose2d.fromAprilTagRobotPose(tag.robotPose);

        // Preserve driver's relative heading before resetting hardware
        FieldHeading currentDriverHeading = getFieldHeading().minus(driverForward);

        // Update driverForward based on the new absolute estimate
        driverForward = estimatedPose.heading.minus(currentDriverHeading);

        // Set hardware to the new estimated pose (translation and rotation)
        pinpoint.setPosition(estimatedPose.toPose2D());

        return true;
    }

    /**
     * Localizes using an AprilTag, and automatically sets the driver forward
     * direction (if enabled)
     *
     * @param team
     * @return If localization was successful or not
     */
    public boolean localizeWithAprilTag(Team team)
    {
        if (!localizeWithAprilTag())
        {
            return false;
        }

        if (setForwardBasedOnTeam.getAsBoolean())
        {
            driverForward = team.forwardAngle;
        }

        return true;
    }

    @Override
    public void periodic()
    {
        setReferencePose(); // attempt to update the reference pose on the pinpoint

        pinpoint.update();

        // if we have sent the reference pose to the pinpoint, use the pinpoint
        if (referencePoseWasSet)
        {
            cachedPose = Pose2d.fromPose2D(pinpoint.getPosition(), CoordinateSystem.DECODE_FTC);
        }
        // otherwise
        else
        {
            cachedPose = Pose2d.fromPose2D(referencePose, CoordinateSystem.DECODE_FTC);
        }
    }

    public void close()
    {
        webcam.close();
    }
}