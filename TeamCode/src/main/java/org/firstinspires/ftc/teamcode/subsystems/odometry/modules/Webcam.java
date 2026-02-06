package org.firstinspires.ftc.teamcode.subsystems.odometry.modules;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

public class Webcam
{
    private final AprilTagProcessor tagProcessor;
    private final VisionPortal visionPortal;

    private final DoubleSupplier xOffset = () -> RobotConstants.Odometry.Webcam.Offset.X.get().getDistance(DistanceUnit.INCH);
    private final DoubleSupplier yOffset = () -> RobotConstants.Odometry.Webcam.Offset.Y.get().getDistance(DistanceUnit.INCH);
    private final DoubleSupplier zOffset = () -> RobotConstants.Odometry.Webcam.Offset.Z.get().getDistance(DistanceUnit.INCH);
    private final DoubleSupplier yawOffset = () -> RobotConstants.Odometry.Webcam.Offset.YAW.get().getAngle(AngleUnit.DEGREES);
    private final DoubleSupplier pitchOffset = () -> RobotConstants.Odometry.Webcam.Offset.PITCH.get().getAngle(AngleUnit.DEGREES);
    private final DoubleSupplier rollOffset = () -> RobotConstants.Odometry.Webcam.Offset.ROLL.get().getAngle(AngleUnit.DEGREES);

    private final DoubleSupplier lensFX = RobotConstants.Odometry.Webcam.Lens.LENS_FX;
    private final DoubleSupplier lensFY = RobotConstants.Odometry.Webcam.Lens.LENS_FY;
    private final DoubleSupplier lensCX = RobotConstants.Odometry.Webcam.Lens.LENS_CX;
    private final DoubleSupplier lensCY = RobotConstants.Odometry.Webcam.Lens.LENS_CY;

    private final Supplier<int[]> goalIds = RobotConstants.AprilTags.GOAL_IDS;
    private final Supplier<int[]> obeliskIds = RobotConstants.AprilTags.OBELISK_IDS;

    private List<AprilTagDetection> cachedTagDetections = new ArrayList<>();

    public final Obelisk obelisk = new Obelisk();
    public final Goal goal = new Goal();

    public Webcam(WebcamName webcam)
    {
        this(webcam, DistanceUnit.INCH, true);
    }

    protected Webcam(WebcamName webcam, DistanceUnit unit)
    {
        this(webcam, unit, true);
    }

    protected Webcam(WebcamName webcam, DistanceUnit unit, boolean showLiveView)
    {
        // Camera position relative to robot center
        Position cameraPosition = new Position(DistanceUnit.INCH,
                xOffset.getAsDouble(),
                yOffset.getAsDouble(),
                zOffset.getAsDouble(),
                0);

        // Camera orientation (yaw, pitch, roll)
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                yawOffset.getAsDouble(),
                pitchOffset.getAsDouble(),
                rollOffset.getAsDouble(),
                0);

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(showLiveView)
                .setDrawCubeProjection(showLiveView)
                .setDrawTagID(showLiveView)
                .setDrawTagOutline(showLiveView)
                .setLensIntrinsics(lensFX.getAsDouble(), lensFY.getAsDouble(), lensCX.getAsDouble(), lensCY.getAsDouble())
                .setCameraPose(cameraPosition, cameraOrientation)
                .setOutputUnits(unit, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(webcam)
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(showLiveView)
                .build();
    }

    /**
     * Detections are cached for efficiency. Calling this method updates the cache
     * to the latest frame.
     */
    public void updateDetections()
    {
        this.cachedTagDetections = tagProcessor.getDetections();
    }

    /**
     * Checks if an AprilTag with a certain ID is in the list of cached detections.
     * 
     * @param id
     *            The ID of the AprilTag to check.
     * @return Whether the AprilTag with the requested ID is in the list cached
     *         detections.
     */
    public boolean tagIdExists(int id)
    {
        // Iterate through all of the tags and check if any of them match the requested
        // ID
        for (AprilTagDetection tag : cachedTagDetections)
        {
            if (tag.id == id)
            {
                return true;
            }
        }

        return false;
    }

    public boolean tagIdExists(int[] ids)
    {
        for (AprilTagDetection tag : cachedTagDetections)
        {
            for (int possibleId : ids)
            {
                if (tag.id == possibleId)
                {
                    return true;
                }
            }
        }

        return false;
    }

    /**
     * Returns a specific AprilTagDetection and throws an error otherwise.
     * 
     * @param id
     *            The ID of the AprilTag to return.
     * @return The AprilTagDetection with the requested ID.
     */
    public Optional<AprilTagDetection> getDetection(int id)
    {
        for (AprilTagDetection tag : cachedTagDetections)
        {
            if (tag.id == id)
            {
                return Optional.of(tag);
            }
        }

        return Optional.empty();
    }

    public Optional<AprilTagDetection> getDetection(int[] ids)
    {
        for (AprilTagDetection tag : cachedTagDetections)
        {
            for (int possibleId : ids)
            {
                if (tag.id == possibleId)
                {
                    return Optional.of(tag);
                }
            }
        }

        return Optional.empty();
    }

    public class Goal
    {
        public Goal()
        {
        }

        public boolean anyVisible()
        {
            return tagIdExists(new int[]
            { 20, 24 });
        }

        public Optional<AprilTagDetection> getAny()
        {
            return getDetection(goalIds.get());
        }

        public Optional<AprilTagDetection> getSpecific(int id)
        {
            boolean idIsCorrect = false;

            for (int possibleId : goalIds.get())
            {
                if (possibleId == id)
                {
                    idIsCorrect = true;
                    break;
                }
            }

            if (!idIsCorrect)
            {
                throw new IllegalArgumentException("AprilTag ID " + id + " is not a valid goal Id");
            }

            return getDetection(id);
        }
    }

    public class Obelisk
    {
        public Obelisk()
        {
        }

        public boolean anyVisible()
        {
            return tagIdExists(new int[]
            { 21, 22, 23 });
        }

        public Optional<Integer> getId()
        {
            Optional<AprilTagDetection> tag = getDetection(obeliskIds.get());

            return tag.map(aprilTagDetection -> aprilTagDetection.id);
        }
    }

    /**
     * Closes the vision portal.
     */
    public void close()
    {
        if (visionPortal != null)
        {
            visionPortal.close();
        }
    }
}