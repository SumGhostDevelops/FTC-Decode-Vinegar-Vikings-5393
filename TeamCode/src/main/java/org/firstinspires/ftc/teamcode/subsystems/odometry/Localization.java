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
    public final Webcam webcam;

    // Placeholder for PedroPath/RoadRunner pose
    private final Pose2d currentPose = new Pose2d(0, 0, 0);
    private double headingOffset = 0;

    public Localization(RobotHardware robot)
    {
        this.robot = robot;
        this.webcam = new Webcam(robot);
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
        return robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + headingOffset;
    }

    public void resetHeading()
    {
        robot.imu.resetYaw();
        headingOffset = 0;
        robot.telemetry.log().add("Reset heading!");
    }

    // --- Vision Logic ---

    public void close()
    {
        webcam.close();
    }
}