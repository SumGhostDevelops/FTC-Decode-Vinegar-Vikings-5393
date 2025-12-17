package org.firstinspires.ftc.teamcode.subsystems.modules.odometry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.modules.odometry.modules.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.modules.odometry.modules.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.Optional;

public class Odometry
{
    private final Telemetry telemetry;

    private final Webcam webcam;
    private final Pinpoint pinpoint;

    private Pose2D currentPose = new Pose2D(DistanceUnit.METER, 0.0, 0.0, AngleUnit.RADIANS, 0);

    public enum AngleType
    {
        SIGNED, // -180 to 180 or -pi to pi
        UNSIGNED, // 0 to 360 or 0 to 2pi
    }

    public Odometry(RobotHardware hw, Telemetry telemetry)
    {
        this.telemetry = telemetry;
        this.webcam = new Webcam(hw.webcam);
        this.pinpoint = hw.pinpoint;
    }

    public void update()
    {
        pinpoint.update();
        currentPose = convertPose(pinpoint.getPosition(), DistanceUnit.METER, AngleUnit.RADIANS);
    }

    public Pose2D getPose(DistanceUnit distanceUnit, AngleUnit angleUnit)
    {
        return convertPose(currentPose, distanceUnit, angleUnit);
    }

    public void updatePose(Pose2D pose)
    {
        pinpoint.setPosition(pose);
    }

    public double getHeading(AngleUnit angleUnit, AngleType angleType)
    {
        // Always returns the normalized heading
        double heading = currentPose.getHeading(angleUnit);

        // Protect against NaN/Infinite
        if (Double.isNaN(heading) || Double.isInfinite(heading)) return heading;

        /**
         * double fullCircle;
         * if (angleUnit == AngleUnit.DEGREES)
         * {
         *     fullCircle = 360.0;
         * }
         * else
         * {
         *     fullCircle = 2.0 * Math.PI;
         * }
         */

        double fullCircle = (angleUnit == AngleUnit.DEGREES) ? 360.0 : 2.0 * Math.PI;

        if (angleType == AngleType.SIGNED)
        {
            // Pose2D already provides a normalized signed heading; return as-is to preserve
            // boundary behavior (e.g. -180 vs +180 as Pose2D defines it).
            return heading;
        }
        else // UNSIGNED
        {
            // Convert signed heading to unsigned [0, fullCircle).
            double unsigned = heading;
            if (unsigned < 0)
            {
                unsigned += fullCircle;
            }
            // Guard against inputs outside expected range; ensure in [0, fullCircle)
            unsigned = ((unsigned % fullCircle) + fullCircle) % fullCircle;
            return unsigned;
        }
    }

    public static Pose2D toFtcFCS(Pose2D pose)
    {
        return pose;
    }

    /**
     * Resets the heading, but only if the robot is stationary.
     */
    public void resetHeading()
    {
        if (getVelocity(DistanceUnit.METER) > 0)
        {
            telemetry.log().add("Not updating the heading because the robot is not still.");
        }
        pinpoint.recalibrateIMU();
    }

    /**
     * @return The resultant velocity of the robot.
     */
    public double getVelocity(DistanceUnit distanceUnit)
    {
        return Math.hypot(pinpoint.getVelX(distanceUnit), pinpoint.getVelY(distanceUnit));
    }

    private Pose2D convertPose(Pose2D pose, DistanceUnit distanceUnit, AngleUnit angleUnit)
    {
        return new Pose2D(distanceUnit, pose.getX(distanceUnit), pose.getY(distanceUnit), angleUnit, pose.getHeading(angleUnit));
    }

    // --- Vision Logic ---

    public Optional<Integer> findObeliskId()
    {
        webcam.updateDetections();

        if (!webcam.obelisk.anyVisible())
        {
            return Optional.empty();
        }

        return webcam.obelisk.getId();
    }

    private Optional<AprilTagPoseFtc> findTagPose(int id)
    {
        return webcam.getDetection(id).map(aprilTagDetection -> aprilTagDetection.ftcPose);
    }

    public void close()
    {
        webcam.close();
    }
}
