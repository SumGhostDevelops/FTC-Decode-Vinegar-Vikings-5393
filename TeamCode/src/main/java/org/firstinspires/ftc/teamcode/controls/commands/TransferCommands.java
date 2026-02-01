package org.firstinspires.ftc.teamcode.controls.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.Transfer;

public class TransferCommands
{
    /**
     * Allows balls to be moved into the outtake.
     */
    public static class OpenOnce extends InstantCommand
    {
        public OpenOnce(Transfer transfer)
        {
            super(transfer::open);
        }
    }

    /**
     * Goes backward to close the transfer.
     */
    public static class CloseOnce extends InstantCommand
    {
        public CloseOnce(Transfer transfer)
        {
            super(transfer::close);
        }
    }

    /**
     * Opens the transfer while active, then closes it.
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
        public void execute()
        {
            transfer.open();
        }

        @Override
        public void end(boolean interrupted)
        {
            transfer.close();
        }
    }
}