package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.util.motors.MotorExPlus;

public class Intake extends SubsystemBase
{

    private final MotorExPlus intake;

    public Intake(MotorExPlus intake)
    {
        this.intake = intake;
    }

    public void in(double power)
    {
        intake.set(Math.abs(power));
    }

    public void out(double power)
    {
        intake.set(-Math.abs(power));
    }

    public void stop()
    {
        intake.stopMotor();
    }

    public double getRPM()
    {
        return intake.getRPM();
    }

    public double getAcceleration()
    {
        return intake.getRPMAcceleration();
    }
}
