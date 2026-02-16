package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.firstinspires.ftc.teamcode.util.motors.PowerMotor;
import org.firstinspires.ftc.teamcode.util.motors.VelocityMotor;

public class Intake extends SubsystemBase
{

    private final VelocityMotor intake;

    public Intake(VelocityMotor intake)
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

    public void transfer(double power)
    {
        intake.setScale(Math.abs(power));
    }

    public void stop()
    {
        intake.stopMotor();
    }

    public double getRPM()
    {
        return intake.getMotorRPM();
    }

    public double getAcceleration()
    {
        return intake.getMotorRPMAcceleration();
    }
}