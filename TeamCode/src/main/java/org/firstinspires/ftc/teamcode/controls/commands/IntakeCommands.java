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
            intake.intake(power.getAsDouble());
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
            intake.reverse(power.getAsDouble());
        }

        @Override
        public void end(boolean interrupted)
        {
            intake.stop();
        }
    }
}