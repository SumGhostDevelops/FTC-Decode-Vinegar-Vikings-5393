package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.util.motors.MotorExPlus;

public class Intake extends SubsystemBase {

    private final MotorExPlus intake;

    public Intake(MotorExPlus intake) {
        this.intake = intake;
    }

    public void in(double power) {
        intake.set(power);
    }

    public void out(double power) {
        intake.set(-power);
    }

    public void stop() {
        intake.set(0);
    }

    public double getRPM() {
        return intake.getRPM();
    }
}
