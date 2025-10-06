import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

public class VisionHelper
{
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;

    public VisionHelper(double[] lensIntrinsics, WebcamName webcamName, int debugLevel)
    {   double fx = lensIntrinsics[0];
        double fy = lensIntrinsics[1];
        double cx = lensIntrinsics[2];
        double cy = lensIntrinsics[3];

        if (showLiveView) // could be optimized
        {
            tagProcessor = new AprilTagProcessor.Builder()
                    .setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .setDrawTagID(true)
                    .setDrawTagOutline(true)
                    .setLensIntrinsics(fx, fy, cx, cy)
                    .build();

            visionPortal = new VisionPortal.Builder()
                    .addProcessor(tagProcessor)
                    .setCamera(webcamName)
                    //.setCameraResolution(new Size(RESOLUTION_HEIGHT, RESOLUTION_WIDTH)) this crashes the driverhub for some reason lol
                    .enableLiveView(true)
                    .build();
        }
        else
        {
            tagProcessor = new AprilTagProcessor.Builder()
                    .setLensIntrinsics(fx, fy, cx, cy)
                    .build();

            visionPortal = new VisionPortal.Builder()
                    .addProcessor(tagProcessor)
                    .setCamera(webcamName)
                    .build();
        }

        switch (debugLevel) // 0: no info on driverhub, 1: textual info, 2: image/video
        {

        }
    }
}
