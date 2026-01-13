package org.firstinspires.ftc.teamcode.subsystems.odometry;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.odometry.modules.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.odometry.modules.Webcam;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Optional;

public class OdometryPinpoint extends SubsystemBase
{
    public final Webcam webcam;
    private final Pinpoint pinpoint;

    private static final DistanceUnit dUnit = DistanceUnit.METER;
    private static final AngleUnit aUnit = AngleUnit.RADIANS;

    /**
     * Offset added to Pinpoint heading to get field-absolute heading.
     * After localize(): headingOffset = AprilTag heading - Pinpoint heading at that moment
     */
    private Angle headingOffset = new Angle(0, aUnit);

    /**
     * The field-absolute heading that the driver considers "forward" for field-centric driving.
     */
    private Angle driverForward = new Angle(0, aUnit);

    /**
     * Initializes the odometry, sets the position to the center of the field, and the forward-angle to the robot's current heading
     * @param webcam
     * @param pinpoint
     */
    public OdometryPinpoint(WebcamName webcam, Pinpoint pinpoint)
    {
        this(webcam, pinpoint, new Pose2d(new FieldCoordinate(new Distance(0, DistanceUnit.INCH), new Distance(0, DistanceUnit.INCH), FieldCoordinate.CoordinateSystem.FTC_STD), new Angle(pinpoint.getHeading(aUnit), aUnit)));
    }

    /**
     * @param webcam
     * @param pinpoint
     * @param referencePose The reference (could be the initial) pose of the robot, to determine its absolute position and heading
     */
    public OdometryPinpoint(WebcamName webcam, Pinpoint pinpoint, Pose2d referencePose)
    {
        this.webcam = new Webcam(webcam);
        this.pinpoint = pinpoint;
        pinpoint.setPosition(referencePose.toPose2D());
    }

    /**
     * @return The field-absolute heading of the robot (Pinpoint heading + headingOffset)
     */
    public Angle getAngle()
    {
        return new Angle(pinpoint.getHeading(aUnit), aUnit).plus(headingOffset);
    }

    /**
     * @return A version of the robot heading where straight-ahead (in the view of the driver) is 0 degrees.
     */
    public Angle getDriverHeading()
    {
        return getAngle().minus(driverForward);
    }

    public FieldCoordinate getFieldCoord()
    {
        Distance x = new Distance(pinpoint.getPosX(dUnit), dUnit);
        Distance y = new Distance(pinpoint.getPosY(dUnit), dUnit);

        return new FieldCoordinate(x, y);
    }

    public Pose2d getPose()
    {
        return new Pose2d(getFieldCoord(), getAngle());
    }

    /**
     * Resets the current position to 0,0,0 and recalibrates the Odometry Computer's internal IMU. <br><br>
     * <strong> Robot MUST be stationary </strong> <br><br>
     * Device takes a large number of samples, and uses those as the gyroscope zero-offset. This takes approximately 0.25 seconds.
     */
    public void resetPosAndHeading()
    {
        pinpoint.resetPosAndIMU();
    }

    /**
     * Sets the driver's forward angle to the robot's current absolute heading.
     * Call this when the robot is facing the direction the driver considers "forward".
     * Note: This assumes the robot is facing field-forward (0°) for absolute heading purposes.
     * If not, call localize() afterward to correct the absolute heading.
     */
    public void setForwardAngle()
    {
        headingOffset = new Angle(0, aUnit);
        driverForward = new Angle(pinpoint.getHeading(aUnit), aUnit);
    }

    /**
     * Resets the Pinpoint IMU so that, when facing the Angle immediately prior to this being called, the Angle returned is 0.
     */
    public void resetAngle()
    {
        pinpoint.recalibrateIMU();
    }

    /**
     * Attempts to relocalize the robot. Camera must be facing a goal AprilTag to work.
     * This sets the robot's absolute position AND heading on the field based on AprilTag detection.
     * The driver's relative forward reference is preserved so field-centric driving remains consistent.
     * @return If the re-localization was successful or not
     */
    public boolean localize()
    {
        // verify the apriltag exists
        webcam.updateDetections();

        Optional<AprilTagDetection> possibleTag = webcam.goal.getAny();
        if (possibleTag.isEmpty()) return false;

        // get the apriltag and the pose it has estimated
        AprilTagDetection tag = possibleTag.get();
        Pose2d estimatedPose = Pose2d.fromPose3D(tag.robotPose);

        // Preserve driver's relative heading before we change headingOffset
        Angle currentDriverHeading = getAngle().minus(driverForward);

        // Compute new headingOffset so getAngle() returns true field-absolute heading
        // headingOffset = estimatedPose.heading - currentPinpointHeading
        Angle currentPinpointHeading = new Angle(pinpoint.getHeading(aUnit), aUnit);
        headingOffset = estimatedPose.heading.toUnit(aUnit).minus(currentPinpointHeading);

        // Update driverForward so getDriverHeading() still returns the same value
        driverForward = estimatedPose.heading.toUnit(aUnit).minus(currentDriverHeading);

        // Set the position to the pinpoint
        pinpoint.setPosition(estimatedPose.toPose2D());


        return true;
    }

    @Override
    public void periodic()
    {
        pinpoint.update();
    }

    /**
     * Closes the {@link Webcam}
     */
    public void close()
    {
        webcam.close();
    }
}