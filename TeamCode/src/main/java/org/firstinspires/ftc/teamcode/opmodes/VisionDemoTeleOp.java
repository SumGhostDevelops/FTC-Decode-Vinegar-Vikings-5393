package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

//@Disabled
@TeleOp
public class VisionDemoTeleOp extends LinearOpMode
{
    //final int RESOLUTION_WIDTH = 640;
    //final int RESOLUTION_HEIGHT = 480;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // create the AprilTag processor using the static Builder
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(1424.38, 1424.38, 637.325, 256.774)
                .build();

        // create the Vision Portal
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // Replace "Webcam 1" with your webcam's name in the config
                //.setCameraResolution(new Size(RESOLUTION_HEIGHT, RESOLUTION_WIDTH)) this crashes the driverhub for some reason lol
                .enableLiveView(true)
                .build();

        waitForStart();

        while (!isStopRequested() && opModeIsActive())
        {
            // get a list of the current detections
            List<AprilTagDetection> currentDetections = tagProcessor.getDetections();

            if (!currentDetections.isEmpty())
            {
                // get the first detection from the list; temp for now and we can get if it is a certain id
                AprilTagDetection tag = currentDetections.get(0);

                // telemetry for the first detected tag
                // telemtry.addData() allows it to be seen in the driverhub
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

        // close vision portal at the end
        visionPortal.close();
    }
}