package org.firstinspires.ftc.teamcode.controls.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

import org.firstinspires.ftc.teamcode.subsystems.odometry.Odometry;

import java.util.function.Supplier;

public class TurretCommands
{
    public static class AimAbsolute extends CommandBase
    {
        private final Turret turret;

        private final Supplier<Angle> targetAngle;
        private final Supplier<Angle> robotAngle;

        public AimAbsolute(Turret turret, Supplier<Angle> targetAngle, Supplier<Angle> robotAngle)
        {
            this.turret = turret;
            this.targetAngle = targetAngle;
            this.robotAngle = robotAngle;
            addRequirements(turret);
        }

        @Override
        public void execute()
        {
            turret.aimAbsolute(targetAngle.get(), robotAngle.get());
        }

        @Override
        public void end(boolean interrupted)
        {
            turret.reset();
        }
    }

    public static class AimRelative extends CommandBase
    {
        private final Turret turret;

        private final Supplier<Angle> targetAngle;

        public AimRelative(Turret turret, Supplier<Angle> targetAngle)
        {
            this.turret = turret;
            this.targetAngle = targetAngle;
            addRequirements(turret);
        }

        @Override
        public void execute()
        {
            turret.aimRelative(targetAngle.get());
        }

        @Override
        public void end(boolean interrupted)
        {
            turret.reset();
        }
    }

    /**
     * Command that continuously aims the turret at a target field coordinate (e.g.,
     * goal).
     * Uses the robot's current pose from odometry to compute the relative bearing.
     */
    public static class AimToCoordinate extends CommandBase
    {
        private final Turret turret;
        private final FieldCoordinate targetCoord;
        private final Odometry odometry;

        /**
         * @param turret
         *            The turret subsystem
         * @param targetCoord
         *            The target field coordinate to aim at (e.g., team.goal.coord)
         * @param odometry
         *            The odometry subsystem
         */
        public AimToCoordinate(Turret turret, FieldCoordinate targetCoord, Odometry odometry)
        {
            this.turret = turret;
            this.targetCoord = targetCoord;
            this.odometry = odometry;
            addRequirements(turret);
        }

        @Override
        public void execute()
        {
            turret.aimToTarget(targetCoord, odometry);
        }

        @Override
        public void end(boolean interrupted)
        {
            turret.reset();
        }
    }
}
