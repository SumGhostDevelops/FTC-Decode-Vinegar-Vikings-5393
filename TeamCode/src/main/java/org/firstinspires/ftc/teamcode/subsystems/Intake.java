package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.util.motors.PowerMotor;
import org.firstinspires.ftc.teamcode.util.motors.VelocityMotor;

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
        // setScale clamps to [0, maxRPM], so use setPower directly for reverse
        intake.setPower(-Math.abs(power));
    }

    public void transfer(double power)
    {
        intake.setPower(Math.abs(power));
    }

    public void stop()
    {
        intake.stopMotor();
    }

    @Override
    public void periodic()
    {
        intake.update();
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