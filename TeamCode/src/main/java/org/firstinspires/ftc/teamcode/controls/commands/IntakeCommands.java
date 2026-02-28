package org.firstinspires.ftc.teamcode.controls.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.function.DoubleSupplier;
import java.util.function.LongSupplier;

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

        private final LongSupplier waitTime;
        private long timestamp = 0;
        private boolean on = false;

        public Transfer(Intake intake, double power)
        {
            this(intake, () -> power);
        }

        public Transfer(Intake intake, DoubleSupplier power)
        {
            this(intake, power, () -> RobotConstants.Transfer.WAIT_BEFORE_TRANSFER);
        }

        public Transfer(Intake intake, DoubleSupplier power, LongSupplier waitTime)
        {
            this.intake = intake;
            this.power = power;
            this.waitTime = waitTime;
            addRequirements(intake);
        }

        @Override
        public void execute()
        {
            if (!on)
            {
                on = true;
                timestamp = System.currentTimeMillis();
            }

            if (System.currentTimeMillis() - timestamp < waitTime.getAsLong())
            {
                return;
            }

            intake.transfer(power.getAsDouble());
        }

        @Override
        public void end(boolean interrupted)
        {
            on = false;
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