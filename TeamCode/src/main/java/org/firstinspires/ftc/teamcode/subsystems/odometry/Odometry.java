package org.firstinspires.ftc.teamcode.subsystems.odometry;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.subsystems.odometry.modules.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.odometry.modules.Webcam;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Optional;

public class Odometry
{
    public final Webcam webcam;
    private final Pinpoint pinpoint;

    private static DistanceUnit dUnit = DistanceUnit.METER;
    private static AngleUnit aUnit = AngleUnit.RADIANS;

    private Angle driverForward;

    /**
     * Initializes the odometry, sets the position to the center of the field, and the forward-angle to the robot's current heading
     * @param webcam
     * @param pinpoint
     */
    public Odometry(WebcamName webcam, Pinpoint pinpoint)
    {
        this(webcam, pinpoint, new Pose2d(new FieldCoordinate(new Distance(0, DistanceUnit.INCH), new Distance(0, DistanceUnit.INCH), FieldCoordinate.CoordinateSystem.FTC_STD), new Angle(pinpoint.getHeading(aUnit), aUnit)));
    }

    /**
     * Initializes the odometry
     * @param webcam
     * @param pinpoint
     * @param referencePose What the robot's current pose is
     */
    public Odometry(WebcamName webcam, Pinpoint pinpoint, Pose2d referencePose)
    {
        this(webcam, pinpoint, referencePose, referencePose.heading);
    }

    /**
     * @param webcam
     * @param pinpoint
     * @param referencePose The reference (could be the initial) pose of the robot, to determine its absolute position and heading
     * @param driverForward The absolute {@link Angle} where it is forward from the driver's perspective
     */
    public Odometry(WebcamName webcam, Pinpoint pinpoint, Pose2d referencePose, Angle driverForward)
    {
        this.webcam = new Webcam(webcam);
        this.pinpoint = pinpoint;
        pinpoint.setPosition(referencePose.toPose2D());
        this.driverForward = driverForward;
    }

    /**
     * @return The (hopefully) absolute {@link Angle} of the robot on the field
     */
    public Angle getAngle()
    {
        return new Angle(pinpoint.getHeading(aUnit), aUnit);
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
     * Resets the IMU heading. The robot must be perpendicular to the Blue alliance to work without errors.
     */
    public void resetHeading()
    {
        pinpoint.recalibrateIMU();
    }

    /**
     * Updates the {@link Odometry#driverForward} so that calls to {@link Odometry#getDriverHeading()} are treated as 0-degrees forward.
     */
    public void resetDriverHeading()
    {
        driverForward = getAngle();
    }

    /**
     * Attempts to relocalize the robot. Camera must be facing a goal AprilTag to work.
     * @return If the re-localization was successful or not
     */
    public boolean localize()
    {
        // verify the apriltag exists
        webcam.updateDetections();

        Optional<AprilTagDetection> possibleTag = webcam.goal.getAny();
        {
            if (possibleTag.isEmpty()) return false;
        }

        // get the apriltag and the pose it has estimated, and set it to the pinpoint
        AprilTagDetection tag = possibleTag.get();
        Pose2d estimatedPose = Pose2d.fromPose3D(tag.robotPose);
        pinpoint.setPosition(estimatedPose.toPose2D());

        return true;
    }

    /**
     * Same thing as {@link Odometry#localize()} but forces the driver-forward angle to be based on specified {@link Team} rather than where the robot is currently facing.
     * @param team
     * @return If the re-localization was successful or not
     * @see Odometry#localize
     */
    public boolean localize(Team team)
    {
        if (!localize()) return false;

        driverForward = team.forwardAngle;

        return true;
    }

    /**
     * Closes the {@link Webcam}
     */
    public void close()
    {
        webcam.close();
    }
}