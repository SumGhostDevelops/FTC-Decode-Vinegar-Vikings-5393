package org.firstinspires.ftc.teamcode.controls.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Transfer;

public class TransferCommands
{
    /**
     * Allows balls to be moved into the outtake.
     */
    public static class Open extends CommandBase
    {
        private final Transfer transfer;

        public Open(Transfer transfer)
        {
            this.transfer = transfer;
            addRequirements(transfer);
        }

        @Override
        public void initialize()
        {
            transfer.close();
        }

        @Override
        public void execute()
        {
            transfer.open();
        }

        @Override
        public void end(boolean interrupted)
        {
            transfer.open();
        }
    }

    public static class CloseIntake extends CommandBase
    {
        private final Transfer transfer;

        public CloseIntake(Transfer transfer)
        {
            this.transfer = transfer;
            addRequirements(transfer);
        }

        @Override
        public void execute()
        {
            transfer.close();
        }
    }
}