package org.firstinspires.ftc.teamcode.controls.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands
{
    public static class Manuever extends CommandBase
    {
        private final Drive drive;

        private final DoubleSupplier x;
        private final DoubleSupplier y;
        private final DoubleSupplier rx;
        private final Supplier<Angle> botAngle;

        /**
         *
         * @param drive
         * @param x Left Stick X; robot left/right
         * @param y Left Stick Y; robot forward/backward
         * @param rx Right Stick X; robot orientation
         * @param driverHeading
         */
        public Manuever(Drive drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rx, Supplier<Angle> driverHeading)
        {
            this.drive = drive;
            this.x = x;
            this.y = y;
            this.rx = rx;
            this.botAngle = driverHeading;
            addRequirements(drive);
        }

        @Override
        public void execute()
        {
            drive.drive(x.getAsDouble(), y.getAsDouble(), rx.getAsDouble(), botAngle.get());
        }

        @Override
        public void end(boolean interrupted)
        {
            drive.stop();
        }
    }

    public static class IncreaseSpeed extends InstantCommand
    {
        public IncreaseSpeed(Drive drive)
        {
            super(drive::increaseSpeed, drive);
        }
    }

    public static class DecreaseSpeed extends InstantCommand
    {
        public DecreaseSpeed(Drive drive)
        {
            super(drive::decreaseSpeed, drive);
        }
    }
}
