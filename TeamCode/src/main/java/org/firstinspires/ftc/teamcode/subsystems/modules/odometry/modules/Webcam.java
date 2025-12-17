package org.firstinspires.ftc.teamcode.subsystems.modules.odometry.modules;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class Webcam
{
    private final AprilTagProcessor tagProcessor;
    private final VisionPortal visionPortal;

    private List<AprilTagDetection> cachedTagDetections = new ArrayList<>();

    public final Obelisk obelisk = new Obelisk();
    public final Goal goal = new Goal();

    public Webcam(WebcamName webcam)
    {
        this(webcam, DistanceUnit.METER, false);
    }

    protected Webcam(WebcamName webcam, DistanceUnit unit)
    {
        this(webcam, unit, false);
    }

    protected Webcam(WebcamName webcam, DistanceUnit unit, boolean showLiveView)
    {
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(showLiveView)
                .setDrawCubeProjection(showLiveView)
                .setDrawTagID(showLiveView)
                .setDrawTagOutline(showLiveView)
                .setLensIntrinsics(RobotConstants.Odometry.Webcam.LENS_FX, RobotConstants.Odometry.Webcam.LENS_FY, RobotConstants.Odometry.Webcam.LENS_CX, RobotConstants.Odometry.Webcam.LENS_CY)
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
     * Detections are cached for efficiency. Calling this method updates the cache to the latest frame.
     */
    public void updateDetections()
    {
        this.cachedTagDetections = tagProcessor.getDetections();
    }

    /**
     * Checks if an AprilTag with a certain ID is in the list of cached detections.
     * @param id The ID of the AprilTag to check.
     * @return Whether the AprilTag with the requested ID is in the list cached detections.
     */
    public boolean tagIdExists(int id)
    {
        // Iterate through all of the tags and check if any of them match the requested ID
        for (AprilTagDetection tag: cachedTagDetections)
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
        for (AprilTagDetection tag: cachedTagDetections)
        {
            for (int possibleId: ids)
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
     * @param id The ID of the AprilTag to return.
     * @return The AprilTagDetection with the requested ID.
     */
    public Optional<AprilTagDetection> getDetection(int id)
    {
        for (AprilTagDetection tag: cachedTagDetections)
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
        for (AprilTagDetection tag: cachedTagDetections)
        {
            for (int possibleId: ids)
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
        public Goal() {}

        public boolean anyVisible()
        {
            return tagIdExists(new int[]{20, 24});
        }

        public Optional<AprilTagDetection> getAny()
        {
            return getDetection(RobotConstants.AprilTags.GOAL_IDS);
        }

        public Optional<AprilTagDetection> getSpecific(int id)
        {
            boolean idIsCorrect = false;

            for (int possibleId: RobotConstants.AprilTags.GOAL_IDS)
            {
                if (possibleId == id)
                {
                    idIsCorrect = true;
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
        public Obelisk() {}

        public boolean anyVisible()
        {
            return tagIdExists(new int[]{21, 22, 23});
        }

        public Optional<Integer> getId()
        {
            Optional<AprilTagDetection> tag = getDetection(RobotConstants.AprilTags.OBELISK_IDS);

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