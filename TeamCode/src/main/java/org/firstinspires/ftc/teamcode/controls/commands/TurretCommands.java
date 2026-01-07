package org.firstinspires.ftc.teamcode.controls.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;

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
     * Command that continuously aims the turret at a target field coordinate (e.g., goal).
     * Uses the robot's current pose from odometry to compute the relative bearing.
     */
    public static class AimToGoal extends CommandBase
    {
        private final Turret turret;
        private final FieldCoordinate targetCoord;
        private final Supplier<Pose2d> robotPose;

        /**
         * @param turret The turret subsystem
         * @param targetCoord The target field coordinate to aim at (e.g., team.goal.coord)
         * @param robotPose Supplier for the robot's current pose from odometry
         */
        public AimToGoal(Turret turret, FieldCoordinate targetCoord, Supplier<Pose2d> robotPose)
        {
            this.turret = turret;
            this.targetCoord = targetCoord;
            this.robotPose = robotPose;
            addRequirements(turret);
        }

        @Override
        public void execute()
        {
            turret.aimToCoordinate(targetCoord, robotPose.get());
        }

        @Override
        public void end(boolean interrupted)
        {
            turret.reset();
        }
    }
}
