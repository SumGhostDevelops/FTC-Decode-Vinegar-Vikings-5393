package org.firstinspires.ftc.teamcode.controls.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing.Timer;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;

public class IntakeCommands
{
    public static class In extends CommandBase
    {
        protected final Intake intake;
        private final DoubleSupplier RPM;

        public In(org.firstinspires.ftc.teamcode.subsystems.Intake intake, DoubleSupplier RPM)
        {
            this.intake = intake;
            this.RPM = RPM;
            addRequirements(intake);
        }

        @Override
        public void execute()
        {
            intake.in(RPM.getAsDouble());
        }

        @Override
        public void end(boolean interrupted)
        {
            intake.stop();
        }
    }

    public static class Out extends CommandBase
    {
        private final Intake intake;
        private final DoubleSupplier power;

        public Out(org.firstinspires.ftc.teamcode.subsystems.Intake intake, DoubleSupplier power)
        {
            this.intake = intake;
            this.power = power;
            addRequirements(intake);
        }

        @Override
        public void execute()
        {
            intake.out(power.getAsDouble());
        }

        @Override
        public void end(boolean interrupted)
        {
            intake.stop();
        }
    }

    public static class TransferPreventForDuration extends CommandBase
    {
        private final Intake intake;
        private final double power;
        private final Timer timer;

        public TransferPreventForDuration(Intake intake, double power, double durationMs)
        {
            this.intake = intake;
            this.power = power;
            timer = new Timer((long) durationMs, TimeUnit.MILLISECONDS);
            addRequirements(intake);
        }

        @Override
        public void execute()
        {
            if (!timer.isTimerOn())
            {
                timer.start();
            }

            intake.out(power);
        }

        @Override
        public boolean isFinished()
        {
            return timer.done();
        }

        @Override
        public void end(boolean interrupted)
        {
            if (!interrupted) intake.stop();
        }
    }
}