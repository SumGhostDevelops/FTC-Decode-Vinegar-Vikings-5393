package org.firstinspires.ftc.teamcode.controls.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

public class TransferCommands
{
    public static class Open extends CommandBase
    {
        private final org.firstinspires.ftc.teamcode.subsystems.Transfer transfer;

        public Open(org.firstinspires.ftc.teamcode.subsystems.Transfer transfer)
        {
            this.transfer = transfer;
        }

        @Override
        public void initialize()
        {
            transfer.open();
        }

        @Override
        public boolean isFinished()
        {
            return true; // One-time action
        }
    }

    public static class Close extends CommandBase
    {
        private final org.firstinspires.ftc.teamcode.subsystems.Transfer transfer;

        public Close(org.firstinspires.ftc.teamcode.subsystems.Transfer transfer)
        {
            this.transfer = transfer;
        }

        @Override
        public void initialize()
        {
            transfer.close();
        }

        @Override
        public boolean isFinished()
        {
            return true; // One-time action
        }
    }
}