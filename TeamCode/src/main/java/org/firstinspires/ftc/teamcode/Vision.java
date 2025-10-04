package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class Vision extends LinearOpMode
{

    final int RESOLUTION_WIDTH = 640;
    final int RESOLUTION_HEIGHT = 480;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // 1. Create the AprilTag processor using the static Builder.
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        // 2. Create the Vision Portal.
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // Replace "Webcam 1" with your webcam's name in the config
                //.setCameraResolution(new Size(RESOLUTION_HEIGHT, RESOLUTION_WIDTH))
                .enableLiveView(true)
                .build();

        waitForStart();

        while (!isStopRequested() && opModeIsActive())
        {
            // 3. Get a list of the current detections.
            List<AprilTagDetection> currentDetections = tagProcessor.getDetections();

            if (!currentDetections.isEmpty())
            {
                // Get the first detection from the list.
                AprilTagDetection tag = currentDetections.get(0);

                // Telemetry for the first detected tag
                telemetry.addData("Detected Tag ID", tag.id);
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);
            } else {
                telemetry.addLine("No AprilTags detected.");
            }

            telemetry.update();
        }

        // 4. Don't forget to close the vision portal when you're done.
        visionPortal.close();
    }
}