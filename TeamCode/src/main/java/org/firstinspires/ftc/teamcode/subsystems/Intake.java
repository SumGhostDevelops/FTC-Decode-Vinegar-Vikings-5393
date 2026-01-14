package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.util.motors.MotorREx;

public class Intake extends SubsystemBase
{

    private final MotorREx intake;

    public Intake(MotorREx intake)
    {
        this.intake = intake;
    }

    public void in(double RPM)
    {
        intake.setRPM(Math.abs(RPM));
    }

    public void out(double power)
    {
        intake.setRPM(-Math.abs(power));
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
