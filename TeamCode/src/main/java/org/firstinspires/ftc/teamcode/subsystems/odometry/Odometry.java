package org.firstinspires.ftc.teamcode.subsystems.odometry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.odometry.modules.EncompassingPose;
import org.firstinspires.ftc.teamcode.subsystems.odometry.modules.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.odometry.modules.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.Optional;

public class Odometry
{
    private final Telemetry telemetry;
    private final Webcam webcam;
    private final Pinpoint pinpoint;

    // Current State
    private EncompassingPose currentPose;

    // Acceleration Calculation
    private Pose2D previousVelocity = new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.RADIANS, 0);
    private long previousTimeNs = System.nanoTime();

    // Low Pass Filter (0 = infinite smoothing, 1 = no smoothing)
    private final double ACCEL_FILTER_ALPHA = 0.2;
    private Pose2D previousAcceleration = new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.RADIANS, 0);

    AprilTagLibrary library = AprilTagGameDatabase.getDecodeTagLibrary();

    public Odometry(RobotHardware hw, Telemetry telemetry)
    {
        this.telemetry = telemetry;
        this.webcam = new Webcam(hw.webcam);
        this.pinpoint = hw.pinpoint;

        update();
    }

    public void update() {
        pinpoint.update();
        long currentTimeNs = System.nanoTime();

        // 1. GET RAW DATA (Always in Meters/Radians for math)
        Pose2D rawPos = pinpoint.getPosition();
        Pose2D rawVel = pinpoint.getVelocity();

        // Ensure strictly Meters/Radians
        Pose2D currPos = convertPose(rawPos, DistanceUnit.METER, AngleUnit.RADIANS);
        Pose2D currVel = convertPose(rawVel, DistanceUnit.METER, AngleUnit.RADIANS);

        // 2. CALCULATE ACCELERATION
        double dt = (currentTimeNs - previousTimeNs) / 1.0e9; // Convert Nanoseconds to Seconds

        // Avoid divide by zero on first loop or super fast updates
        if (dt < 1.0e-4) dt = 1.0e-4;

        double rawAccelX = (currVel.getX(DistanceUnit.METER) - previousVelocity.getX(DistanceUnit.METER)) / dt;
        double rawAccelY = (currVel.getY(DistanceUnit.METER) - previousVelocity.getY(DistanceUnit.METER)) / dt;
        double rawAccelH = (currVel.getHeading(AngleUnit.RADIANS) - previousVelocity.getHeading(AngleUnit.RADIANS)) / dt;

        // 3. LOW PASS FILTER (Smoothing)
        double filteredAccelX = (ACCEL_FILTER_ALPHA * rawAccelX) + ((1 - ACCEL_FILTER_ALPHA) * previousAcceleration.getX(DistanceUnit.METER));
        double filteredAccelY = (ACCEL_FILTER_ALPHA * rawAccelY) + ((1 - ACCEL_FILTER_ALPHA) * previousAcceleration.getY(DistanceUnit.METER));
        double filteredAccelH = (ACCEL_FILTER_ALPHA * rawAccelH) + ((1 - ACCEL_FILTER_ALPHA) * previousAcceleration.getHeading(AngleUnit.RADIANS));

        Pose2D currAccel = new Pose2D(DistanceUnit.METER, filteredAccelX, filteredAccelY, AngleUnit.RADIANS, filteredAccelH);

        // 4. UPDATE STATE
        currentPose = new EncompassingPose(currPos, currVel, currAccel, pinpoint.getHeading(AngleUnit.RADIANS), currentTimeNs);

        previousVelocity = currVel;
        previousAcceleration = currAccel;
        previousTimeNs = currentTimeNs;
    }

    public EncompassingPose getPose()
    {
        return currentPose;
    }

    public void updatePosition(Pose2D pose)
    {
        pinpoint.setPosition(pose);
    }

    public void setStartPosFromTag(int tagId) {
        if (tagId == 21 || tagId == 22 || tagId == 23)
        {
            throw new IllegalArgumentException("Tag Id " + tagId + " should not be used to localize the odometry.");
        }

        // 1. Look up the tag in the SDK Database
        AprilTagMetadata tagMetadata = library.lookupTag(tagId);

        if (tagMetadata == null) {
            telemetry.addData("Error", "Tag ID %d not found in database", tagId);
            return;
        }

        // 2. Get Field Data (The DB uses INCHES and QUATERNIONS)
        // We need to convert Position to Meters
        double tagFieldX = tagMetadata.distanceUnit.toMeters(tagMetadata.fieldPosition.get(0));
        double tagFieldY = tagMetadata.distanceUnit.toMeters(tagMetadata.fieldPosition.get(1));

        // We need to convert Orientation (Quaternion) to a standard Z-axis Heading (Radians)
        // The SDK provides a helper to convert Quaternion to YAW-PITCH-ROLL
        // AxesReference.EXTRINSIC, AxesOrder.XYZ is standard for extracting Z rotation
        double tagFieldHeading = tagMetadata.fieldOrientation.toOrientation(
                org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC,
                org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ,
                org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS
        ).thirdAngle; // thirdAngle corresponds to Z (Yaw) in XYZ order

        // 3. Get the Detection from the Camera
        findTagPose(tagId).ifPresent(tagDetection -> {
            // --- LOGIC FROM BEFORE ---

            // A. Calculate Robot Heading
            // RobotHeading = TagFieldHeading - CameraYaw
            double cameraYaw = Math.toRadians(tagDetection.yaw); // Ensure radians
            double robotHeading = tagFieldHeading - cameraYaw;

            // B. Rotation Matrix for Position
            // Camera X = Right, Y = Forward (Check your specific camera config!)
            double relativeX = tagDetection.x;
            double relativeY = tagDetection.y;

            // Rotate camera vector to match field vector
            double rotatedX = relativeX * Math.cos(robotHeading) - relativeY * Math.sin(robotHeading);
            double rotatedY = relativeX * Math.sin(robotHeading) + relativeY * Math.cos(robotHeading);

            // C. Calculate Absolute Position
            double finalX = tagFieldX - rotatedX;
            double finalY = tagFieldY - rotatedY;

            // D. Update Pinpoint
            Pose2D newStart = new Pose2D(DistanceUnit.METER, finalX, finalY, AngleUnit.RADIANS, robotHeading);
            pinpoint.setPosition(newStart);

            // Force internal update
            update();

            telemetry.addData("Relocalized", "Pos: %.2f, %.2f Heading: %.2f", finalX, finalY, Math.toDegrees(robotHeading));
        });
    }

    public double getHeading(AngleUnit angleUnit, EncompassingPose.AngleType angleType)
    {
        return currentPose.getHeading(angleUnit, angleType);
    }

    /**
     * Resets the heading, but only if the robot is stationary.
     */
    public void resetHeading()
    {
        if (currentPose.getVelocityResultant(DistanceUnit.METER) > 0)
        {
            telemetry.log().add("Not updating the heading because the robot is not still.");
        }
        pinpoint.recalibrateIMU();
    }

    /**
     * @return The resultant velocity of the robot.
     */
    public double getVelocity(DistanceUnit distanceUnit)
    {
        return currentPose.getVelocityResultant(distanceUnit);
    }

    /**
     * @return The resultant acceleration of the robot.
     */
    public double getAcceleration(DistanceUnit distanceUnit)
    {
        return currentPose.getAccelerationResultant(distanceUnit);
    }

    private Pose2D convertPose(Pose2D pose, DistanceUnit distanceUnit, AngleUnit angleUnit)
    {
        return new Pose2D(distanceUnit, pose.getX(distanceUnit), pose.getY(distanceUnit), angleUnit, pose.getHeading(angleUnit));
    }

    // --- Vision Logic ---

    public void updateWebcamDetections()
    {
        webcam.updateDetections();
    }

    public boolean tagIdExists(int id)
    {
        return webcam.tagIdExists(id);
    }

    public Optional<Double> getRangeToTag(int id)
    {
        return webcam.getDetection(id).map(detection -> detection.ftcPose.range);
    }

    public Optional<Integer> findObeliskId()
    {
        webcam.updateDetections();

        return webcam.obelisk.getId();
    }

    private Optional<AprilTagPoseFtc> findTagPose(int id)
    {
        webcam.updateDetections();

        return webcam.getDetection(id).map(aprilTagDetection -> aprilTagDetection.ftcPose);
    }

    public void close()
    {
        webcam.close();
    }
}