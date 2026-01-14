package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.util.motors.MotorXP;

public class Intake extends SubsystemBase
{

    private final MotorXP intake;

    public Intake(MotorXP intake)
    {
        this.intake = intake;
    }

    public void in(double RPM)
    {
        intake.setRPM(Math.abs(RPM));
    }

    public void out(double RPM)
    {
        intake.setRPM(-Math.abs(RPM));
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
