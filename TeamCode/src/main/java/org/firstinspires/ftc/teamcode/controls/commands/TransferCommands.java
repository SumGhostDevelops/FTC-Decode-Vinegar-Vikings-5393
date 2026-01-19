package org.firstinspires.ftc.teamcode.controls.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.util.Timing.Timer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

import java.util.concurrent.TimeUnit;

public class TransferCommands
{
    /**
     * Default command that maintains the transfer in its closed intake position.
     * This ensures the transfer always returns to its initial position when no other commands are running.
     */
    public static class DefaultPosition extends CommandBase
    {
        private final Transfer transfer;

        public DefaultPosition(Transfer transfer)
        {
            this.transfer = transfer;
            addRequirements(transfer);
        }

        @Override
        public void execute()
        {
            transfer.close(Transfer.CloseType.FORWARD);
        }
    }

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
            transfer.close(Transfer.CloseType.BACKWARD);
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
            transfer.close(Transfer.CloseType.FORWARD);
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
            transfer.close(Transfer.CloseType.BACKWARD);
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
                Thread.sleep(RobotConstants.Transfer.TimerConstants.upTime);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            transfer.close(Transfer.CloseType.SHOOT);
            try {
                Thread.sleep(RobotConstants.Transfer.TimerConstants.downTime);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        // Never changes position when ending
    }

    public static class ShootingTransfer2 extends CommandBase
    {
        private final Transfer transfer;
        private final Timer timer;
        private final boolean[] stepsCompleted = new boolean[]{false, false, false};

        private Telemetry telemetry;

        public ShootingTransfer2(Transfer transfer, Telemetry telemetry) {
            this.transfer = transfer;
            this.telemetry = telemetry;
            timer = new Timer(RobotConstants.Transfer.TimerConstants.totalTime, TimeUnit.MILLISECONDS);
            timer.pause();
            addRequirements(transfer);
        }

        @Override
        public void execute()
        {
            if (!timer.isTimerOn())
            {
                timer.start();
            }

            if (!stepsCompleted[0])
            {
                transfer.open();
                stepsCompleted[0] = true;
            }
            else if (timer.elapsedTime() >= RobotConstants.Transfer.TimerConstants.upTime && !stepsCompleted[1])
            {
                transfer.close(Transfer.CloseType.SHOOT);
                stepsCompleted[1] = true;
            }
        }

        @Override
        public boolean isFinished()
        {
            telemetry.log().add("Transfer timer remaining", (int) timer.remainingTime());
            return timer.done();
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
        private final Timer timer;

        public CloseTransferForDuration(Transfer transfer, double durationMs)
        {
            this.transfer = transfer;
            timer = new Timer((long) durationMs, TimeUnit.MILLISECONDS);
            addRequirements(transfer);
        }

        @Override
        public void execute()
        {
            if (!timer.isTimerOn())
            {
                timer.start();
            }

            transfer.close();
        }

        @Override
        public boolean isFinished()
        {
            return timer.done();
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