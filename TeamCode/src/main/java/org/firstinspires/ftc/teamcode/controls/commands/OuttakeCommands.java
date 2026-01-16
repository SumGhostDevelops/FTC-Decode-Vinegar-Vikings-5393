package org.firstinspires.ftc.teamcode.controls.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class OuttakeCommands
{
    public static class On extends CommandBase
    {
        Outtake outtake;
        BooleanSupplier idleWhenEnd;

        public On(Outtake outtake, BooleanSupplier idleWhenEnd)
        {
            this.outtake = outtake;
            this.idleWhenEnd = idleWhenEnd;
            addRequirements(outtake);
        }

        @Override
        public void initialize()
        {
            if (idleWhenEnd.getAsBoolean())
            {
                outtake.idle();
            }
            else
            {
                outtake.off();
            }
        }

        @Override
        public void execute()
        {
            outtake.on();
        }

        @Override
        public void end(boolean interrupted)
        {
            if (idleWhenEnd.getAsBoolean())
            {
                outtake.idle();
            }
            else
            {
                outtake.off();
            }
        }
    }

    public static class Idle extends CommandBase
    {
        Outtake outtake;

        public Idle(Outtake outtake)
        {
            this.outtake = outtake;
            addRequirements(outtake);
        }

        @Override
        public void execute()
        {
            outtake.idle();
        }

        @Override
        public void end(boolean interrupted)
        {
            outtake.off();
        }
    }

    public static class Off extends InstantCommand
    {
        public Off(Outtake outtake)
        {
            super(outtake::off);
        }
    }

    public static class ChangeTargetRPM extends InstantCommand
    {
        public ChangeTargetRPM(Outtake outtake, double rpm)
        {
            super(() -> outtake.setTargetRPM(outtake.getTargetRPM() + rpm), outtake);
        }
    }

    public static class UpdateRPMBasedOnDistance extends CommandBase
    {
        private final Outtake outtake;
        private final Supplier<Distance> distance;

        public UpdateRPMBasedOnDistance(Outtake outtake, Supplier<Distance> distance)
        {
            this.outtake = outtake;
            this.distance = distance;
            // Note: Do not add requirements - this command should run alongside other outtake commands
        }

        @Override
        public void execute()
        {
            outtake.setTargetRPMFromDistance(distance.get());
        }
    }
}