package org.firstinspires.ftc.teamcode.controls.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeCommands
{
    public static class In extends CommandBase
    {
        protected final Intake intake;
        private final double power;

        public In(Intake intake, double power)
        {
            this.intake = intake;
            this.power = power;
            addRequirements(intake);
        }

        @Override
        public void execute()
        {
            intake.intake(power);
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