package org.firstinspires.ftc.teamcode.controls.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

public class TransferCommands
{
    public static class Open extends CommandBase
    {
        private final Transfer transfer;

        public Open(Transfer transfer)
        {
            this.transfer = transfer;
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