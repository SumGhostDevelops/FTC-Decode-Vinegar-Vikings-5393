package org.firstinspires.ftc.teamcode.controls;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

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


}
