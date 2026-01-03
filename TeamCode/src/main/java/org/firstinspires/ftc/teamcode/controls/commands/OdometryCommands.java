package org.firstinspires.ftc.teamcode.controls.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.odometry.OdometryControlHub;

public class OdometryCommands
{
    public static class SetForwardAngle extends InstantCommand
    {
        public SetForwardAngle(OdometryControlHub odometry)
        {
            super(odometry::setForwardAngle, odometry);
        }
    }
}