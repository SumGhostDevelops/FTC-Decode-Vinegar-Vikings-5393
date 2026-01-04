package org.firstinspires.ftc.teamcode.subsystems.odometry;

import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor.Encoder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    private Angle headingOffset = new Angle(0, aUnit);

    /**
     * The field-absolute heading that the driver considers "forward" for field-centric driving.
     */
    private Angle driverForward = new Angle(0, aUnit);

    public OdometryControlHub(WebcamName webcam, IMU imu, Encoder dwPar, Encoder dwPerp)
    {
        this(webcam, imu, dwPar, dwPerp, new Pose2d(new FieldCoordinate(new Distance(0, DistanceUnit.INCH), new Distance(0, DistanceUnit.INCH), FieldCoordinate.CoordinateSystem.FTC_STD), new Angle(0, AngleUnit.DEGREES)));
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

        // create handler with current encoder baselines (likely zero after reset)
        this.dwHandler = new DeadwheelHandler(referencePose, dwPar.getPosition(), dwPerp.getPosition());
    }

    public Angle getYaw(AngleUnit angleUnit)
    {
        return new Angle(imu.getRobotYawPitchRollAngles().getYaw(angleUnit), angleUnit);
    }

    /**
     * @return The field-absolute heading of the robot (IMU yaw + headingOffset)
     */
    public Angle getAngle()
    {
        return getYaw(aUnit).plus(headingOffset);
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
     * Resets the IMU yaw and sets the driver's forward to the current heading.
     * Call this when the robot is facing the direction the driver wants to be "forward".
     * Note: This assumes the robot is facing field-forward (0°) for absolute heading purposes.
     * If not, call localize() afterward to correct the absolute heading.
     */
    public void setForwardAngle()
    {
        imu.resetYaw();
        headingOffset = new Angle(0, aUnit);
        driverForward = new Angle(0, aUnit);
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
        AprilTagDetection tag = possibleTag.get();
        Pose2d estimatedPose = Pose2d.fromPose3D(tag.robotPose);

        // Preserve driver's relative heading before we change headingOffset
        // currentDriverHeading = getAngle() - driverForward
        Angle currentDriverHeading = getAngle().minus(driverForward);

        // Compute new headingOffset so getAngle() returns true field-absolute heading
        // headingOffset = estimatedPose.heading - currentIMUYaw
        Angle currentImuYaw = getYaw(aUnit);
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