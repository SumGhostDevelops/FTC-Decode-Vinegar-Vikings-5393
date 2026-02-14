package org.firstinspires.ftc.teamcode.controls.commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.odometry.Odometry;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;

public class OdometryCommands
{
    /**
     * Sets the current field-absolute heading as the driver's forward direction.
     * Use this AFTER localization to define forward for field-centric driving.
     */
    public static class SetDriverForwardFromCurrent extends InstantCommand
    {
        public SetDriverForwardFromCurrent(Odometry odometry)
        {
            super(odometry::setDriverForwardFromCurrent, odometry);
        }
    }

    /**
     * Attempts to localize the robot using AprilTag detection.
     * Logs success or failure to telemetry
     */
    public static class Localize extends InstantCommand
    {
        public Localize(Odometry odometry, Telemetry telemetry)
        {
            super(() ->
            {
                boolean success = odometry.localizeWithAprilTag();
                if (success)
                {
                    telemetry.log().add("Localization successful!");
                }
                else
                {
                    telemetry.log().add("Localization failed - no AprilTag detected");
                }
            });
        }
    }

    /**
     * Attempts to localize the robot using AprilTag detection.
     * Logs success or failure to telemetry and rumbles based on success.
     */
    public static class LocalizeWithRumble extends InstantCommand
    {
        public LocalizeWithRumble(Odometry odometry, Telemetry telemetry, Gamepad gamepad)
        {
            super(() ->
            {
                boolean success = odometry.localizeWithAprilTag();
                if (success)
                {
                    telemetry.log().add("Localization successful!");
                    gamepad.rumbleBlips(3);
                }
                else
                {
                    telemetry.log().add("Localization failed - no AprilTag detected");
                    gamepad.rumble(2000);
                }
            });
        }
    }

    /**
     * Attempts to localize the robot using AprilTag detection.
     * Logs raw robotPose values (x, y, heading) to telemetry for debugging.
     * This is useful for diagnosing localization accuracy issues.
     */
    public static class LocalizeWithDebugTelemetry extends InstantCommand
    {
        public LocalizeWithDebugTelemetry(Odometry odometry, Telemetry telemetry)
        {
            super(() ->
            {
                boolean success = odometry.localizeWithAprilTag();
                if (success)
                {
                    telemetry.log().add("Localization: SUCCESS");
                }
                else
                {
                    telemetry.log().add("Localization: FAILED - no AprilTag");
                }
            });
        }
    }

    public static class SetReferencePose extends InstantCommand
    {
        public SetReferencePose(Odometry odometry, Pose2d pose)
        {
            super(() ->
            {
                odometry.setReferencePose(pose);
            });
        }
    }
}