package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.exceptions.TooManyTagsDetectedException;
import org.firstinspires.ftc.teamcode.exceptions.NoTagsDetectedException;
import org.firstinspires.ftc.teamcode.exceptions.TagNotDetectedException;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

public class VisionHelper
{
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;
    private boolean showLiveView;
    private boolean showDebugText;

    private List<AprilTagDetection> detections;

    public VisionHelper(double[] lensIntrinsics, WebcamName webcamName, int debugLevel) // TODO: Remove debug level cus I swear it doesn't work/doesn't get used
    {
        // lens instrinsics dont worry about it
        double fx = lensIntrinsics[0];
        double fy = lensIntrinsics[1];
        double cx = lensIntrinsics[2];
        double cy = lensIntrinsics[3];

        if (debugLevel < 0 || debugLevel > 2)
        {
            throw new IllegalArgumentException(debugLevel + " is not within the valid range of 0-2.");
        }

        switch (debugLevel)
        {
            case 0: // production
            {
                showDebugText = false;
                showLiveView = false;
                break;
            }
            case 1: // only text
            {
                showDebugText = true;
                showLiveView = false;
                break;
            }
            case 2: // only live view
            {
                showDebugText = false;
                showLiveView = true;
                break;
            }
        }

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(showLiveView)
                .setDrawCubeProjection(showLiveView)
                .setDrawTagID(showLiveView)
                .setDrawTagOutline(showLiveView)
                .setLensIntrinsics(fx, fy, cx, cy)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(webcamName)
                //.setCameraResolution(new Size(RESOLUTION_HEIGHT, RESOLUTION_WIDTH)) this crashes the driverhub for some reason lol
                .enableLiveView(showLiveView)
                .build();

    }

    /**
     * Returns the cached detections. These are only the detections from the last checked updateDetections() frame.
     */
    private List<AprilTagDetection> getCachedDetections()
    {
        return this.detections;
    }

    /**
     * Detections are cached for efficiency. Calling this method updates the cache to the latest frame.
     */
    public void updateDetections()
    {
        this.detections = tagProcessor.getDetections();
    }

    /**
     * Checks if an AprilTag with a certain ID is in the cached detections.
     * @param id The ID of the AprilTag to check.
     * @return Whether the AprilTag with the requested ID is in the cached detections.
     */
    public boolean tagIdExists(int id)
    {
        List<AprilTagDetection> tags = getCachedDetections();

        // Iterate through all of the tags and check if any of them match the requested ID
        for (AprilTagDetection tag: tags)
        {
            if (tag.id == id)
            {
                return true;
            }
        }

        return false;
    }

    /**
     * Returns the first AprilTagDetection found and throws an error otherwise.
     * @return The first AprilTagDetection found.
     */
    public AprilTagDetection getSingleDetection() throws NoTagsDetectedException, TooManyTagsDetectedException
    {
        List<AprilTagDetection> detections = getCachedDetections();

        if (detections.isEmpty())
        {
            throw new NoTagsDetectedException();
        }
        else if (detections.size() > 1)
        {
            throw new TooManyTagsDetectedException(1, detections.size());
        }

        return detections.get(0);
    }

    /**
     * Returns a specific AprilTagDetection and throws an error otherwise.
     * @param id The ID of the AprilTag to return.
     * @return The AprilTagDetection with the requested ID.
     */
    public AprilTagDetection getSingleDetection(int id) throws NoTagsDetectedException, TagNotDetectedException
    {
        List<AprilTagDetection> detections = getCachedDetections();

        if (detections.isEmpty())
        {
            throw new NoTagsDetectedException();
        }
        else if (!tagIdExists(id))
        {
            throw new TagNotDetectedException(id);
        }

        return detections.get(0);
    }

    /**
     * Closes the vision portal.
     */
    public void close()
    {
        visionPortal.close();
    }
}
