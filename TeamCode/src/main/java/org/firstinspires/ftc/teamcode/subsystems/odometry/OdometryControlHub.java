package org.firstinspires.ftc.teamcode.subsystems.odometry;

import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor.Encoder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.subsystems.odometry.modules.DeadwheelHandler;
import org.firstinspires.ftc.teamcode.subsystems.odometry.modules.Webcam;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Optional;

public class OdometryControlHub extends SubsystemBase
{
    private final Webcam webcam;
    private final Encoder dwPar;
    private final Encoder dwPerp;
    private final IMU imu;

    private static final DistanceUnit dUnit = DistanceUnit.METER;
    private static final AngleUnit aUnit = AngleUnit.RADIANS;

    private DeadwheelHandler dwHandler;

    /**
     * Offset added to IMU yaw to get field-absolute heading.
     * After localize(): headingOffset = AprilTag heading - IMU yaw at that moment
     */
    private Angle headingOffset;

    /**
     * The field-absolute heading that the driver considers "forward" for field-centric driving.
     */
    private Angle driverForward;

    public OdometryControlHub(WebcamName webcam, IMU imu, Encoder dwPar, Encoder dwPerp)
    {
        this(webcam, imu, dwPar, dwPerp, new Pose2d(new FieldCoordinate(new Distance(0, DistanceUnit.INCH), new Distance(0, DistanceUnit.INCH), FieldCoordinate.CoordinateSystem.FTC_STD), new Angle(90, AngleUnit.DEGREES)));
    }

    public OdometryControlHub(WebcamName webcam, IMU imu, Encoder dwPar, Encoder dwPerp, Pose2d referencePose)
    {
        this.webcam = new Webcam(webcam);
        this.imu = imu;
        this.dwPar = dwPar;
        this.dwPerp = dwPerp;

        // reset encoders so deadwheel handler's initial encoder baseline is known
        dwPar.reset();
        dwPerp.reset();

        // Initialize headingOffset so getAngle() returns the reference pose heading
        // Since IMU yaw starts at 0 after reset, headingOffset = referencePose.heading - 0 = referencePose.heading
        this.headingOffset = referencePose.heading.toUnit(aUnit);

        // Initialize driverForward to the reference pose heading so getDriverHeading() starts at 0
        this.driverForward = referencePose.heading.toUnit(aUnit);

        // create handler with current encoder baselines (likely zero after reset)
        this.dwHandler = new DeadwheelHandler(referencePose, dwPar.getPosition(), dwPerp.getPosition());
    }

    public Angle getIMUYaw(AngleUnit angleUnit)
    {
        return new Angle(imu.getRobotYawPitchRollAngles().getYaw(angleUnit), angleUnit);
        //-180 to 180
    }

    /**
     * @return The field-absolute heading of the robot (IMU yaw + headingOffset)
     */
    public Angle getAngle()
    {
        return getIMUYaw(aUnit).plus(headingOffset);
    }

    /**
     * @return A version of the robot heading where the driver's chosen forward is 0 (for field-centric driving)
     */
    public Angle getDriverHeading()
    {
        return getAngle().minus(driverForward);
    }

    public FieldCoordinate getFieldCoord()
    {
        return dwHandler.getCoord();
    }

    public Pose2d getPose()
    {
        return new Pose2d(getFieldCoord(), getAngle());
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
        // Note: robotPose already accounts for camera offset since we configured
        // the AprilTagProcessor with setCameraPose() in the Webcam class
        AprilTagDetection tag = possibleTag.get();
        Pose2d estimatedPose = Pose2d.fromPose3D(tag.robotPose);

        // Preserve driver's relative heading before we change headingOffset
        // currentDriverHeading = getAngle() - driverForward
        Angle currentDriverHeading = getAngle().minus(driverForward);

        // Compute new headingOffset so getAngle() returns true field-absolute heading
        // headingOffset = estimatedPose.heading - currentIMUYaw
        Angle currentImuYaw = getIMUYaw(aUnit);
        headingOffset = estimatedPose.heading.toUnit(aUnit).minus(currentImuYaw);

        // Update driverForward so getDriverHeading() still returns the same value
        // getDriverHeading() = getAngle() - driverForward = currentDriverHeading
        // driverForward = getAngle() - currentDriverHeading = estimatedPose.heading - currentDriverHeading
        driverForward = estimatedPose.heading.toUnit(aUnit).minus(currentDriverHeading);

        // reset encoders and re-create handler with current baseline and absolute pose
        dwPar.reset();
        dwPerp.reset();
        dwHandler = new DeadwheelHandler(estimatedPose, dwPar.getPosition(), dwPerp.getPosition());

        return true;
    }

    /**
     * Manually sets the robot's position and heading based on a known reference pose.
     * Use this when the robot is placed at a known location (e.g., from CornersCoordinates).
     * The driver's relative forward reference is preserved so field-centric driving remains consistent.
     * @param referencePose The known pose of the robot on the field
     */
    public void updateReferencePose(Pose2d referencePose)
    {
        // Preserve driver's relative heading before we change headingOffset
        Angle currentDriverHeading = getAngle().minus(driverForward);

        // Compute new headingOffset so getAngle() returns the reference pose heading
        // headingOffset = referencePose.heading - currentIMUYaw
        Angle currentImuYaw = getIMUYaw(aUnit);
        headingOffset = referencePose.heading.toUnit(aUnit).minus(currentImuYaw);

        // Update driverForward so getDriverHeading() still returns the same value
        driverForward = referencePose.heading.toUnit(aUnit).minus(currentDriverHeading);

        // reset encoders and re-create handler with current baseline and the new reference pose
        dwPar.reset();
        dwPerp.reset();
        dwHandler = new DeadwheelHandler(referencePose, dwPar.getPosition(), dwPerp.getPosition());
    }

    /**
     * Gets raw AprilTag detection data for debugging. Does not modify robot state.
     * @return String with raw AprilTag pose data, or null if no tag detected
     */
    public String getRawAprilTagData()
    {
        webcam.updateDetections();
        Optional<AprilTagDetection> possibleTag = webcam.goal.getAny();
        if (possibleTag.isEmpty()) return null;

        AprilTagDetection tag = possibleTag.get();
        return String.format(
            "ID=%d | Raw XYZ: (%.2f, %.2f, %.2f) %s | Yaw: %.1f° | Converted Pose: (%.2f, %.2f) in, range: %.2f, heading: %.1f°, yaw: %.2f",
            tag.id,
            tag.robotPose.getPosition().x,
            tag.robotPose.getPosition().y,
            tag.robotPose.getPosition().z,
            tag.robotPose.getPosition().unit.toString(),
            tag.robotPose.getOrientation().getYaw(AngleUnit.DEGREES),
            Pose2d.fromPose3D(tag.robotPose).coord.x.getDistance(DistanceUnit.INCH),
            Pose2d.fromPose3D(tag.robotPose).coord.y.getDistance(DistanceUnit.INCH),
            tag.ftcPose.range,
            Pose2d.fromPose3D(tag.robotPose).heading.getAngle(AngleUnit.DEGREES),
                tag.ftcPose.yaw
        );
    }

    @Override
    public void periodic()
    {
        // pass the ABSOLUTE heading into the deadwheel handler for correct field-relative position integration
        dwHandler.updatePose(dwPar.getPosition(), dwPerp.getPosition(), getAngle());
    }

    /**
     * Closes the {@link Webcam}
     */
    public void close()
    {
        webcam.close();
    }
}
