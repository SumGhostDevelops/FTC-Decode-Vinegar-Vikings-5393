package org.firstinspires.ftc.teamcode.controls.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.Outtake;

public class OuttakeCommands
{
    public static class On extends CommandBase
    {
        Outtake outtake;

        public On(Outtake outtake)
        {
            this.outtake = outtake;
            addRequirements(outtake);
        }

        @Override
        public void execute()
        {
            outtake.on();
        }

        @Override
        public void end(boolean interrupted)
        {
            outtake.off();
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

    public static class ChangeTargetRPM extends InstantCommand
    {
        public ChangeTargetRPM(Outtake outtake, double rpm)
        {
            super(() -> outtake.setTargetRPM(outtake.getTargetRPM() + rpm), outtake);
        }
    }
}