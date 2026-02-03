package org.firstinspires.ftc.teamcode.subsystems.odometry;

import static java.lang.Thread.sleep;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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

public class Odometry extends SubsystemBase {
    private final Pinpoint pinpoint;
    private final Webcam webcam;

    private final boolean setForwardBasedOnTeam = RobotConstants.Odometry.SET_FORWARD_DIRECTION_BASED_ON_TEAM;

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

    public Odometry(Pinpoint pinpoint, WebcamName webcam) {
        this(pinpoint, webcam, new Pose2d(new Distance(72, DistanceUnit.INCH), new Distance(72, DistanceUnit.INCH),
                new Angle(90, AngleUnit.DEGREES), CoordinateSystem.DECODE_PEDROPATH));
    }

    public Odometry(Pinpoint pinpoint, WebcamName webcam, Pose2d referencePose) {
        this.pinpoint = pinpoint;
        this.webcam = new Webcam(webcam);

        this.referencePose = referencePose.toCoordinateSystem(CoordinateSystem.DECODE_FTC).toPose2D();

        if (!referencePoseWasSet && this.pinpoint.getDeviceStatus() == Pinpoint.DeviceStatus.READY) {
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
    public Angle getIMUYaw() {
        return cachedPose.heading.toSystem(CoordinateSystem.DECODE_FTC).angle;
    }

    /**
     * @return The absolute heading of the robot
     */
    public FieldHeading getFieldHeading() {
        return cachedPose.heading;
    }

    /**
     * @return A heading where 0 is conceptually "Forward" (aligned with Pedro
     *         X-Axis/Blue Alliance)
     */
    public Angle getDriverHeading() {
        FieldHeading currentPedro = getFieldHeading().toSystem(CoordinateSystem.DECODE_PEDROPATH);
        FieldHeading startPedro = driverForward.toSystem(CoordinateSystem.DECODE_PEDROPATH);

        return currentPedro.minus(startPedro).angle;
    }

    /**
     * @return The field coordinate of the robot
     */
    public FieldCoordinate getFieldCoord() {
        return cachedPose.coord;
    }

    /**
     * @return The pose of the robot
     */
    public Pose2d getPose() {
        return cachedPose;
    }

    /**
     * @return A vector of the velocity
     */
    public Vector2d getVelocity() {
        return new Vector2d(
                new Distance(pinpoint.getVelX(dUnit), dUnit),
                new Distance(pinpoint.getVelY(dUnit), dUnit),
                CoordinateSystem.DECODE_FTC);
    }

    /**
     * @return The velocity of the robot
     */
    public UnnormalizedAngle getHeadingVelocity() {
        return new UnnormalizedAngle(pinpoint.getHeadingVelocity(aUnit.getUnnormalized()), aUnit.getUnnormalized());
    }

    /**
     * Sets the current field-absolute heading as the driver's "forward" direction.
     * Call this AFTER localization to set which direction the driver considers
     * forward
     * for field-centric driving, without affecting the absolute heading
     * calibration.
     */
    public void setDriverForwardFromCurrent() {
        driverForward = getFieldHeading();
    }

    public void updateReferencePose(Pose2d referencePose) {
        driverForward = referencePose.heading;

        pinpoint.setPosition(referencePose.toPose2D());
    }

    /**
     * Localizes using an AprilTag
     * 
     * @return If the localization was successful or not
     */
    public boolean localizeWithAprilTag() {
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
    public boolean localizeWithAprilTag(Team team) {
        if (!localizeWithAprilTag()) {
            return false;
        }

        if (setForwardBasedOnTeam) {
            driverForward = team.forwardAngle;
        }

        return true;
    }

    /**
     * Localizes using an AprilTag and prints raw robotPose values to telemetry for
     * debugging.
     * 
     * @param telemetry
     *                  Telemetry object to log debug values
     * @return If the localization was successful or not
     */
    public boolean localizeWithDebugTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        // verify the apriltag exists
        webcam.updateDetections();

        Optional<AprilTagDetection> possibleTag = webcam.goal.getAny();
        if (possibleTag.isEmpty()) {
            telemetry.log().add("Localization: No AprilTag detected");
            return false;
        }

        AprilTagDetection tag = possibleTag.get();

        // Log raw robotPose values for debugging
        double rawX = tag.robotPose.getPosition().x;
        double rawY = tag.robotPose.getPosition().y;
        double rawYaw = tag.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
        String unit = tag.robotPose.getPosition().unit.toString();

        telemetry.log().add(String.format("Raw robotPose: x=%.2f, y=%.2f (%s), yaw=%.1f°", rawX, rawY, unit, rawYaw));

        Pose2d estimatedPose = Pose2d.fromAprilTagRobotPose(tag.robotPose);

        // Preserve driver's relative heading before resetting hardware
        FieldHeading currentDriverHeading = getFieldHeading().minus(driverForward);

        // Update driverForward based on the new absolute estimate
        driverForward = estimatedPose.heading.minus(currentDriverHeading);

        // Set hardware to the new estimated pose (translation and rotation)
        pinpoint.setPosition(estimatedPose.toPose2D());

        // Log converted pose values
        telemetry.log().add(String.format("Converted Pose2d: %s", estimatedPose.toString()));

        return true;
    }

    @Override
    public void periodic() {
        if (!this.referencePoseWasSet && pinpoint.getDeviceStatus() == Pinpoint.DeviceStatus.READY) {
            this.pinpoint.setPosition(this.referencePose);
            this.referencePoseWasSet = true;
        }

        pinpoint.update();
        cachedPose = Pose2d.fromPose2D(pinpoint.getPosition(), CoordinateSystem.DECODE_FTC);
    }

    public void close() {
        webcam.close();
    }
}