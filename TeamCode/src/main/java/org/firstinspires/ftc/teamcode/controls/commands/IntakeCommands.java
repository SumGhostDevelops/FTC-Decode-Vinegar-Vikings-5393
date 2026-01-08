package org.firstinspires.ftc.teamcode.controls.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.function.DoubleSupplier;

public class IntakeCommands
{
    public static class In extends CommandBase
    {
        protected final Intake intake;
        private final DoubleSupplier power;

        public In(org.firstinspires.ftc.teamcode.subsystems.Intake intake, DoubleSupplier power)
        {
            this.intake = intake;
            this.power = power;
            addRequirements(intake);
        }

        @Override
        public void execute()
        {
            intake.in(power.getAsDouble());
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
        private final double durationMs;
        private long startTime;

        public TransferPreventForDuration(Intake intake, double power, double durationMs)
        {
            this.intake = intake;
            this.power = power;
            this.durationMs = durationMs;
            addRequirements(intake);
        }

        @Override
        public void execute()
        {
            startTime = System.currentTimeMillis();
            intake.out(power);
        }

        @Override
        public boolean isFinished()
        {
            return System.currentTimeMillis() - startTime >= durationMs;
        }

        @Override
        public void end(boolean interrupted)
        {
            intake.stop();
        }
    }
}