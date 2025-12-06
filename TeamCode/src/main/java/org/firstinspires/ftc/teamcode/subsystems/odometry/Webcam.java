package org.firstinspires.ftc.teamcode.subsystems.odometry;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.exceptions.NoTagsDetectedException;
import org.firstinspires.ftc.teamcode.exceptions.TagNotFoundException;
import org.firstinspires.ftc.teamcode.exceptions.TooManyTagsDetectedException;
import org.firstinspires.ftc.teamcode.exceptions.UnexpectedTagIDException;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Webcam
{
    private final RobotHardware robot;
    private final AprilTagProcessor tagProcessor;
    private final VisionPortal visionPortal;

    private List<AprilTagDetection> cachedTagDetections;

    public Webcam(RobotHardware robot)
    {
        this(robot, false);
    }

    public Webcam(RobotHardware robot, boolean showLiveView)
    {
        this.robot = robot;

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(showLiveView)
                .setDrawCubeProjection(showLiveView)
                .setDrawTagID(showLiveView)
                .setDrawTagOutline(showLiveView)
                .setLensIntrinsics(RobotConstants.LENS_FX, RobotConstants.LENS_FY, RobotConstants.LENS_CX, RobotConstants.LENS_CY)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(robot.webcam)
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
    public AprilTagDetection getSingleDetection()
    {
        if (cachedTagDetections.isEmpty())
        {
            return null;
        }
        else if (cachedTagDetections.size() > 1)
        {
            return null;
        }

        return cachedTagDetections.get(0);
    }

    /**
     * Returns a specific AprilTagDetection and throws an error otherwise.
     * @param id The ID of the AprilTag to return.
     * @return The AprilTagDetection with the requested ID.
     */
    public AprilTagDetection getSingleDetection(int id)
    {
        if (cachedTagDetections.isEmpty())
        {
            return null;
        }

        for (AprilTagDetection tag: cachedTagDetections)
        {
            if (tag.id == id)
            {
                return tag;
            }
        }

        return null;
    }

    public AprilTagDetection getAnyDetection()
    {
        if (cachedTagDetections.isEmpty())
        {
            return null;
        }

        return cachedTagDetections.get(0);
    }

    public int getAnyGoalId()
    {
        if (cachedTagDetections.isEmpty())
        {
            return -1;
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

    public double getComponentDistanceToTag(int tagId)
    {
        AprilTagDetection tag = null;

        if (cachedTagDetections.isEmpty())
        {
            return -1;
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
            return -1;
        }

        return Math.sqrt(Math.pow(tag.ftcPose.z, 2) + Math.pow(tag.ftcPose.y, 2));
    }

    /**
     * Returns the first AprilTagDetection with an obelisk ID and returns an error otherwise.
     * @return The first AprilTagDetection with an obelisk ID.
     */
    public AprilTagDetection scanObelisk()
    {
        if (cachedTagDetections.isEmpty())
        {
            return null;
        }

        for (AprilTagDetection tag: cachedTagDetections)
        {
            if (tag.id == 21 || tag.id == 22 || tag.id == 23)
            {
                return tag;
            }
        }

        return null;
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