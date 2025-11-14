package org.firstinspires.ftc.teamcode.robot;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.exceptions.TooManyTagsDetectedException;
import org.firstinspires.ftc.teamcode.exceptions.NoTagsDetectedException;
import org.firstinspires.ftc.teamcode.exceptions.TagNotFoundException;
import org.firstinspires.ftc.teamcode.exceptions.UnexpectedTagIDException;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

public class AprilTagWebcam
{
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;

    private List<AprilTagDetection> cachedTagDetections;

    /**
     * Creates a new VisionHelper.
     * @param lensIntrinsics The lens intrinsics of the camera.
     * @param webcamName     The name of the webcam.
     * @param showLiveView   Whether to show the live view of the camera.
     */
    public AprilTagWebcam(double[] lensIntrinsics, WebcamName webcamName, boolean showLiveView) // TODO: Remove debug level cus I swear it doesn't work/doesn't get used
    {
        // lens instrinsics dont worry about it
        double fx = lensIntrinsics[0];
        double fy = lensIntrinsics[1];
        double cx = lensIntrinsics[2];
        double cy = lensIntrinsics[3];

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(showLiveView)
                .setDrawCubeProjection(showLiveView)
                .setDrawTagID(showLiveView)
                .setDrawTagOutline(showLiveView)
                .setLensIntrinsics(fx, fy, cx, cy)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(webcamName)
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

    /**
     * Returns the first AprilTagDetection found and throws an error otherwise.
     * @return The first AprilTagDetection found.
     */
    public AprilTagDetection getSingleDetection() throws NoTagsDetectedException, TooManyTagsDetectedException
    {
        if (cachedTagDetections.isEmpty())
        {
            throw new NoTagsDetectedException();
        }
        else if (cachedTagDetections.size() > 1)
        {
            throw new TooManyTagsDetectedException(1, cachedTagDetections.size());
        }

        return cachedTagDetections.get(0);
    }

    /**
     * Returns a specific AprilTagDetection and throws an error otherwise.
     * @param id The ID of the AprilTag to return.
     * @return The AprilTagDetection with the requested ID.
     */
    public AprilTagDetection getSingleDetection(int id) throws NoTagsDetectedException, TagNotFoundException
    {
        if (cachedTagDetections.isEmpty())
        {
            throw new NoTagsDetectedException();
        }

        for (AprilTagDetection tag: cachedTagDetections)
        {
            if (tag.id == id)
            {
                return tag;
            }
        }

        throw new TagNotFoundException(id);
    }

    public AprilTagDetection getAnyDetection() throws NoTagsDetectedException
    {
        if (cachedTagDetections.isEmpty())
        {
            throw new NoTagsDetectedException();
        }

        return cachedTagDetections.get(0);
    }

    public int getAnyGoalId() throws NoTagsDetectedException, TagNotFoundException
    {
        if (cachedTagDetections.isEmpty())
        {
            throw new NoTagsDetectedException();
        }

        for (AprilTagDetection tag : cachedTagDetections)
        {
            if (tag.id == 20 || tag.id == 24)
            {
                return tag.id;
            }
        }

        throw new TagNotFoundException();
    }

    public double getComponentDistanceToTag(int tagId) throws NoTagsDetectedException, TagNotFoundException
    {
        AprilTagDetection tag = null;

        if (cachedTagDetections.isEmpty())
        {
            throw new NoTagsDetectedException();
        }

        for (AprilTagDetection possibleTag : cachedTagDetections)
        {
            if (possibleTag.id == tagId)
            {
                tag = possibleTag;
            }
        }

        if (tag == null)
        {
            throw new TagNotFoundException(tagId);
        }

        return Math.sqrt(Math.pow(tag.ftcPose.z, 2) + Math.pow(tag.ftcPose.y, 2));
    }

    /**
     * Returns the first AprilTagDetection with an obelisk ID and returns an error otherwise.
     * @return The first AprilTagDetection with an obelisk ID.
     */
    public AprilTagDetection scanObelisk() throws NoTagsDetectedException, UnexpectedTagIDException
    {
        if (cachedTagDetections.isEmpty())
        {
            throw new NoTagsDetectedException();
        }

        for (AprilTagDetection tag: cachedTagDetections)  // TODO: Add check to see if there is only 1 valid tag in the list since we might detect more than 1 tag on the obelisk
        {
            if (tag.id == 21 || tag.id == 22 || tag.id == 23)
            {
                return tag;
            }
        }

        throw new UnexpectedTagIDException();
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