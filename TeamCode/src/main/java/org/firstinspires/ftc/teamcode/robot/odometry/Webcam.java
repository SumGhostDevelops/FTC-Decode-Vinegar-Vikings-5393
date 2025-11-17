package org.firstinspires.ftc.teamcode.robot.odometry;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Webcam
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
    public Webcam(WebcamName webcamName, double fx, double fy, double cx, double cy, boolean showLiveView) // TODO: Remove debug level cus I swear it doesn't work/doesn't get used
    {
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
}
