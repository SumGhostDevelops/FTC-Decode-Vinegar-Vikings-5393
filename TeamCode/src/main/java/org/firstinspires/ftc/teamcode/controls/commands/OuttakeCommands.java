package org.firstinspires.ftc.teamcode.controls.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.Outtake;

public class OuttakeCommands
{
    public static class On extends InstantCommand
    {
        public On(Outtake outtake)
        {
            super(outtake::on, outtake);
        }
    }

    public static class Idle extends InstantCommand
    {
        public Idle(Outtake outtake)
        {
            super(outtake::idle, outtake);
        }
    }

    public static class Off extends InstantCommand
    {
        public Off(Outtake outtake)
        {
            super(outtake::off, outtake);
        }
    }

    public static class Toggle extends CommandBase
    {
        Outtake outtake;

        public Toggle(Outtake outtake)
        {
            this.outtake = outtake;
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

    public static class ChangeTargetRPM extends InstantCommand
    {
        public ChangeTargetRPM(Outtake outtake, double rpm)
        {
            super(() -> outtake.setTargetRPM(outtake.getTargetRPM() + rpm), outtake);
        }
    }
}