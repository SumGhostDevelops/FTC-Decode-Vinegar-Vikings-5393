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
    public static class OpenTransfer extends CommandBase
    {
        private final Transfer transfer;

        public OpenTransfer(Transfer transfer)
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
            transfer.close(Transfer.CloseType.TRANSFER);
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
            transfer.close(Transfer.CloseType.INTAKE);
        }

        // Never changes position when ending
    }

    public static class CloseTransfer extends CommandBase
    {
        private final Transfer transfer;

        public CloseTransfer(Transfer transfer)
        {
            this.transfer = transfer;
            addRequirements(transfer);
        }

        @Override
        public void execute()
        {
            transfer.close(Transfer.CloseType.TRANSFER);
        }

        // Never changes position when ending
    }

    public static class ShootingTransfer extends CommandBase
    {
        private final Transfer transfer;

        public ShootingTransfer(Transfer transfer) {
            this.transfer = transfer;
            addRequirements(transfer);
        }

        @Override
        public void execute()
        {
            transfer.open();
            try {
                Thread.sleep(600);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            transfer.close(Transfer.CloseType.TRANSFER);
            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            transfer.close(Transfer.CloseType.INTAKE);
            try {
                Thread.sleep(600);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        // Never changes position when ending
    }

    /**
     * Closes the transfer for a specified duration, then ends.
     * Useful for preventing accidental shots when outtake becomes not ready.
     */
    public static class CloseTransferForDuration extends CommandBase
    {
        private final Transfer transfer;
        private final double durationMs;
        private long startTime;

        public CloseTransferForDuration(Transfer transfer, double durationMs)
        {
            this.transfer = transfer;
            this.durationMs = durationMs;
            addRequirements(transfer);
        }

        @Override
        public void initialize()
        {
            transfer.close(Transfer.CloseType.TRANSFER);
        }

        @Override
        public void execute()
        {
            startTime = System.currentTimeMillis();
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
            if (!interrupted) transfer.open();
        }
    }

    public static class BallTransfer extends ParallelCommandGroup
    {
        public BallTransfer(Transfer transfer, Intake intake)
        {
            addCommands(
                    new IntakeCommands.In(intake, () -> 0.8),
                    new OpenTransfer(transfer)
            );
            addRequirements(transfer, intake);
        }
    }
}