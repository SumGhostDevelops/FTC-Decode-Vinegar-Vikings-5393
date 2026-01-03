package org.firstinspires.ftc.teamcode.controls.commands;

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

    public static class Off extends InstantCommand
    {
        public Off(Outtake outtake)
        {
            super(outtake::off, outtake);
        }
    }

    public static class ChangeTargetRPM extends InstantCommand
    {
        public ChangeTargetRPM(Outtake outtake, int rpm)
        {
            super(() -> outtake.setRPM(outtake.getTargetRPM() + rpm), outtake);
        }
    }
}