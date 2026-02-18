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

        public In(Intake intake, double power)
        {
            this(intake, () -> power);
        }

        public In(Intake intake, DoubleSupplier power)
        {
            this.intake = intake;
            this.power = power;
            addRequirements(intake);
        }

        @Override
        public void execute()
        {
            intake.intake(power.getAsDouble());
        }

        @Override
        public void end(boolean interrupted)
        {
            intake.stop();
        }
    }

    public static class Transfer extends CommandBase
    {
        protected final Intake intake;
        private final DoubleSupplier power;

        public Transfer(Intake intake, double power)
        {
            this(intake, () -> power);
        }

        public Transfer(Intake intake, DoubleSupplier power)
        {
            this.intake = intake;
            this.power = power;
            addRequirements(intake);
        }

        @Override
        public void execute()
        {
            intake.transfer(power.getAsDouble());
        }

        @Override
        public void end(boolean interrupted)
        {
            intake.stop();
        }
    }

    public static class Reverse extends CommandBase
    {
        private final Intake intake;
        private final double power;

        public Reverse(Intake intake, double power)
        {
            this.intake = intake;
            this.power = power;
            addRequirements(intake);
        }

        @Override
        public void execute()
        {
            intake.reverse(power);
        }

        @Override
        public void end(boolean interrupted)
        {
            intake.stop();
        }
    }
}