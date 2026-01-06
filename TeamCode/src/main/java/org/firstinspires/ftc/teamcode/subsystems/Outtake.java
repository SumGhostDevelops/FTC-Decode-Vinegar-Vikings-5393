package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.SubsystemBase;

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
    private double setRPM = targetRPM;

    // Tolerance to avoid tiny floating-point updates (adjust as needed)
    private static final double RPM_EPS = 1.0;

    public Outtake(MotorExPlus motor)
    {
        this.motor = motor;
        this.state = State.OFF;
    }

    public void on()
    {
        on(false);
    }

    public void on(boolean force)
    {
        if (state == State.ON && !force)
        {
            return;
        }

        state = State.ON;

        // Only change motor mode / RPM if actually needed
        double desired = targetRPM;
        if (Math.abs(desired - setRPM) > RPM_EPS || force)
        {
            motor.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setRPM(desired);
            setRPM = desired;
        }
    }

    /**
     * Runs the Outtake at half of its target RPM
     */
    public void idle()
    {
        idle(false);
    }

    public void idle(boolean force)
    {
        if (state == State.IDLE && !force)
        {
            return;
        }

        state = State.IDLE;

        double desired = targetRPM / 2.0;
        if (Math.abs(desired - setRPM) > RPM_EPS || force)
        {
            motor.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setRPM(desired);
            setRPM = desired;
        }
    }

    public void off()
    {
        if (state == State.OFF)
        {
            return;
        }

        state = State.OFF;
        motor.stopMotor();

        setRPM = 0;
    }

    public void setTargetRPM(double rpm)
    {
        double newRPM = Math.min(6000, Math.max(0, rpm));

        // Only apply changes when they are significant
        if (Math.abs(newRPM - targetRPM) <= RPM_EPS)
        {
            targetRPM = newRPM; // keep value but don't push small changes
            return;
        }

        targetRPM = newRPM;

        // If motor is active, compute desired rpm for current state and update only if needed
        double desired;
        switch (state)
        {
            case ON:
                desired = targetRPM;
                break;
            case IDLE:
                desired = targetRPM / 2.0;
                break;
            case OFF:
            default:
                // When OFF, do not push RPM to motor
                return;
        }

        if (Math.abs(desired - setRPM) > RPM_EPS)
        {
            motor.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setRPM(desired);
            setRPM = desired;
        }
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

    public State getState()
    {
        return state;
    }

    @Override
    public void periodic()
    {
        motor.updateAcceleration();
    }
}
