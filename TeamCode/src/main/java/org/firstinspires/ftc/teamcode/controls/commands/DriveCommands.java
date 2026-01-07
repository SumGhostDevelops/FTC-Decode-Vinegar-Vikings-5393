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

        private final DoubleSupplier lateral;
        private final DoubleSupplier axial;
        private final DoubleSupplier yaw;
        private final Supplier<Angle> botAngle;

        /**
         *
         * @param drive
         * @param lateral Left Stick Y; robot forward/backward
         * @param axial Left Stick X; robot left/right
         * @param yaw Right Stick X; robot orientation
         * @param driverHeading
         */
        public Manuever(Drive drive, DoubleSupplier lateral, DoubleSupplier axial, DoubleSupplier yaw, Supplier<Angle> driverHeading)
        {
            this.drive = drive;
            this.lateral = lateral;
            this.axial = axial;
            this.yaw = yaw;
            this.botAngle = driverHeading;
            addRequirements(drive);
        }

        @Override
        public void execute()
        {
            drive.drive(lateral.getAsDouble(), axial.getAsDouble(), yaw.getAsDouble(), botAngle.get());
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
