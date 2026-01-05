// java
package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.util.motors.MotorExPlus;

public class Outtake extends SubsystemBase {

    public enum State
    {
        ON,
        IDLE,
        OFF,
    }

    private final MotorExPlus motor;
    private State state;

    private double targetRPM = 0;

    // Accept a single MotorExPlus (RobotContext should provide this)
    public Outtake(MotorExPlus motor)
    {
        this.motor = motor;
        this.state = State.OFF;
    }

    public void on()
    {
        if (state == State.ON)
        {
            return;
        }

        state = State.ON;
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setRPM(targetRPM);
    }

    /**
     * Runs the Outtake at half of its target RPM
     */
    public void idle()
    {
        if (state == State.IDLE)
        {
            return;
        }

        state = State.IDLE;
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setRPM(targetRPM / 2.0);
    }

    public void off()
    {
        if (state == State.OFF)
        {
            return;
        }

        state = State.OFF;
        motor.stopMotor();
    }

    public void setRPM(double rpm)
    {
        targetRPM = Math.min(6000, Math.max(0, rpm));
    }

    public double getTargetRPM()
    {
        return targetRPM;
    }

    public double getRPM()
    {
        return motor.getRPM();
    }

    public double getRPMAcceleration()
    {
        return motor.getRPMAcceleration();
    }

    public boolean isStable()
    {
        return motor.isStable();
    }

    @Override
    public void periodic()
    {
        motor.updateAcceleration();
    }
}