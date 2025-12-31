package org.firstinspires.ftc.teamcode.controls.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands
{
    public static class Manuever extends CommandBase
    {
        private final Drive drive;

        private final DoubleSupplier lateral;
        private final DoubleSupplier axial;
        private final DoubleSupplier yaw;
        private final Supplier<Angle> botAngle;

        public Manuever(Drive drive, DoubleSupplier lateral, DoubleSupplier axial, DoubleSupplier yaw, Supplier<Angle> botAngle)
        {
            this.drive = drive;
            this.lateral = lateral;
            this.axial = axial;
            this.yaw = yaw;
            this.botAngle = botAngle;
        }

        @Override
        public void execute()
        {
            drive.drive(lateral.getAsDouble(), axial.getAsDouble(), yaw.getAsDouble(), botAngle.get());
        }
    }

    public static class IncreaseSpeed extends CommandBase
    {
        private final Drive drive;

        public IncreaseSpeed(Drive drive)
        {
            this.drive = drive;
        }

        @Override
        public void initialize()
        {
            drive.increaseSpeed();
        }

        @Override
        public boolean isFinished()
        {
            return true;
        }
    }

    public static class DecreaseSpeed extends CommandBase
    {
        private final Drive drive;

        public DecreaseSpeed(Drive drive)
        {
            this.drive = drive;
        }

        @Override
        public void initialize()
        {
            drive.decreaseSpeed();
        }

        @Override
        public boolean isFinished()
        {
            return true;
        }
    }
}
