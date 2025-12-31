package org.firstinspires.ftc.teamcode.controls;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

import java.util.function.DoubleSupplier;

public class Commands {
    public static class IntakeIn extends CommandBase {

        private final Intake intake;
        private final DoubleSupplier power;

        public IntakeIn(Intake intake, DoubleSupplier power) {
            this.intake = intake;
            this.power = power;
            addRequirements(intake);
        }

        @Override
        public void execute() {
            intake.in(power.getAsDouble());
        }

        @Override
        public void end(boolean interrupted) {
            intake.stop();
        }
    }

    public static class IntakeOut extends CommandBase {

        private final Intake intake;
        private final DoubleSupplier power;

        public IntakeOut(Intake intake, DoubleSupplier power) {
            this.intake = intake;
            this.power = power;
            addRequirements(intake);
        }

        @Override
        public void execute() {
            intake.out(power.getAsDouble());
        }

        @Override
        public void end(boolean interrupted) {
            intake.stop();
        }
    }

    public static class TransferEngage extends CommandBase {

        private final Transfer transfer;

        public TransferEngage(Transfer transfer) {
            this.transfer = transfer;
        }

        @Override
        public void initialize() {
            transfer.open();
        }

        @Override
        public boolean isFinished() {
            return true; // One-time action
        }
    }

    public static class TransferDisengage extends CommandBase {

        private final Transfer transfer;

        public TransferDisengage(Transfer transfer) {
            this.transfer = transfer;
        }

        @Override
        public void initialize() {
            transfer.close();
        }

        @Override
        public boolean isFinished() {
            return true; // One-time action
        }
    }


}
