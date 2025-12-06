package org.firstinspires.ftc.teamcode.subsystems.odometry;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Localization
{
    private final RobotHardware robot;
    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;

    // Placeholder for PedroPath/RoadRunner pose
    private final Pose2d currentPose = new Pose2d(0, 0, 0);
    private double headingOffset = 0;

    public Localization(RobotHardware robot)
    {
        this.robot = robot;

        // Initialize Vision
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(RobotConstants.LENS_FX, RobotConstants.LENS_FY, RobotConstants.LENS_CX, RobotConstants.LENS_CY)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(robot.webcam)
                .addProcessor(aprilTag)
                .build();
    }

    public void update()
    {
        // In a real implementation, this calls your PedroPath Follower.update()
        // For now, we just update IMU heading

        // Placeholder Odometry Update Logic using RobotHardware.parEncoder / perpEncoder
        // ...
    }

    public Pose2d getPose()
    {
        // If using PedroPath, return follower.getPose()
        return currentPose;
    }

    public double getHeading()
    {
        return robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + headingOffset;
    }

    public void resetHeading()
    {
        robot.imu.resetYaw();
        headingOffset = 0;
    }

    // --- Vision Logic ---

    /**
     * Returns the ID of the detected Obelisk tag (21, 22, or 23), or -1 if not found.
     */
    public int scanObelisk()
    {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections)
        {
            if (detection.metadata != null)
            {
                if (detection.id >= 21 && detection.id <= 23)
                {
                    return detection.id;
                }
            }
        }
        return -1;
    }

    public AprilTagDetection getDetection(int id)
    {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections)
        {
            if (detection.id == id) return detection;
        }
        return null;
    }

    public void close()
    {
        if (visionPortal != null) visionPortal.close();
    }
}