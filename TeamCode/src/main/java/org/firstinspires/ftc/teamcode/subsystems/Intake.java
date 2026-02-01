package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.util.motors.PowerMotor;

public class Intake extends SubsystemBase
{

    private final PowerMotor intake;

    public Intake(PowerMotor intake)
    {
        this.intake = intake;
    }

    public void intake(double power)
    {
        intake.setPower(Math.abs(power));
    }

    public void reverse(double power)
    {
        intake.setPower(-Math.abs(power));
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