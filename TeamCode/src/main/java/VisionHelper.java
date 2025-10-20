import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;
import java.util.List;

public class VisionHelper
{
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;
    private boolean showLiveView;
    private boolean showDebugText;

    public VisionHelper(double[] lensIntrinsics, WebcamName webcamName, int debugLevel)
    {   double fx = lensIntrinsics[0];
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

    public List<AprilTagDetection> getDetections()
    {
        return tagProcessor.getDetections();
    }

    /*
    public boolean updateDriverHubData()
    {
        List<AprilTagDetection> currentDetections = getDetections();

        if (!currentDetections.isEmpty())
        {
            break;
        }
    }
     */

    public double get(String value)
    {
        List<AprilTagDetection> currentDetections = getDetections();
        if (currentDetections.isEmpty()) // send lie to warn if there are no detections
        {
            return 0;
        }

        AprilTagDetection tag = currentDetections.get(0);

        switch (value)
        {
            case "yaw":
            {
                return tag.ftcPose.yaw;
            }
            default:
            {
                throw new IllegalArgumentException(value + " is not within the valid range of 0-2.");
            }
        }
    }

    public void close()
    {
        visionPortal.close();
    }
}
