package org.firstinspires.ftc.teamcode.controls.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
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
            transfer.close();
        }
    }

    public static class Close extends CommandBase
    {
        private final Transfer transfer;

        public Close(Transfer transfer)
        {
            this.transfer = transfer;
            addRequirements(transfer);
        }

        @Override
        public void execute()
        {
            transfer.close();
        }

        @Override
        public void end(boolean interrupted)
        {
            transfer.open();
        }
    }

    /**
     * Closes the transfer for a specified duration, then ends.
     * Useful for preventing accidental shots when outtake becomes not ready.
     */
    public static class CloseForDuration extends CommandBase
    {
        private final Transfer transfer;
        private final double durationMs;
        private long startTime;

        public CloseForDuration(Transfer transfer, double durationMs)
        {
            this.transfer = transfer;
            this.durationMs = durationMs;
            addRequirements(transfer);
        }

        @Override
        public void initialize()
        {
            startTime = System.currentTimeMillis();
            transfer.close();
        }

        @Override
        public void execute()
        {
            transfer.close();
        }

        @Override
        public boolean isFinished()
        {
            return System.currentTimeMillis() - startTime >= durationMs;
        }

        @Override
        public void end(boolean interrupted)
        {
            //if (!interrupted) transfer.open();
        }
    }

    public static class BallTransfer extends ParallelCommandGroup
    {
        public BallTransfer(Transfer transfer, Intake intake)
        {
            addCommands(
                    new IntakeCommands.In(intake, () -> 0.8),
                    new Open(transfer)
            );
            addRequirements(transfer, intake);
        }
    }
}