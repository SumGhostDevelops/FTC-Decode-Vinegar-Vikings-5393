package org.firstinspires.ftc.teamcode.controls.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;

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
            turret.setTargetAbsolute(targetAngle.get(), robotAngle.get());
            turret.aim();
        }

        @Override
        public void end(boolean interrupted)
        {
            turret.setTargetRelative(new Angle(90, AngleUnit.DEGREES));
            turret.aim();
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
            turret.setTargetRelative(targetAngle.get());
            turret.aim();
        }

        @Override
        public void end(boolean interrupted)
        {
            turret.setTargetRelative(new Angle(90, AngleUnit.DEGREES));
            turret.aim();
        }
    }
}
