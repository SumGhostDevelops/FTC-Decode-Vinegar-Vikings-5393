package org.firstinspires.ftc.teamcode.controls.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.odometry.OdometryControlHub;

public class OdometryCommands
{
    /**
     * Sets the current field-absolute heading as the driver's forward direction.
     * Use this AFTER localization to define forward for field-centric driving.
     */
    public static class SetDriverForwardFromCurrent extends InstantCommand
    {
        public SetDriverForwardFromCurrent(OdometryControlHub odometry)
        {
            super(odometry::setDriverForwardFromCurrent, odometry);
        }
    }

    /**
     * Attempts to localize the robot using AprilTag detection.
     * Logs success or failure to telemetry, along with raw AprilTag data for debugging.
     */
    public static class Localize extends InstantCommand
    {
        public Localize(OdometryControlHub odometry, Telemetry telemetry)
        {
            super(() -> {
                // Get raw data before localization for debugging
                String rawData = odometry.getRawAprilTagData();
                if (rawData != null)
                {
                    telemetry.log().add("AprilTag: " + rawData);
                }

                boolean success = odometry.localize();
                if (success)
                {
                    telemetry.log().add("Localization successful!");
                }
                else
                {
                    telemetry.log().add("Localization failed - no AprilTag detected");
                }
            }, odometry);
        }
    }
}