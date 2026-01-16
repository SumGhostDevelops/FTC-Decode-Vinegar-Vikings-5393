package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotMath;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.firstinspires.ftc.teamcode.util.motors.MotorXP;

public class Outtake extends SubsystemBase {

    public enum State
    {
        ON,
        IDLE,
        OFF,
    }

    private final MotorXP motor;
    private State state = State.OFF;

    private double targetRPM = RobotConstants.Outtake.BASE_RPM;
    private double setRPM = 0;

    private Distance lastDistance = new Distance(0, DistanceUnit.INCH);

    // Tolerance to avoid tiny floating-point updates (adjust as needed)
    private static final double RPM_EPS = 1.0;

    public Outtake(MotorXP motor)
    {
        this.motor = motor;
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

    public void setTargetRPMFromDistance(Distance dist)
    {
        if (!RobotConstants.Outtake.AUTO_DISTANCE_ADJUSMENT)
        {
            return;
        }

        Distance epsilon = new Distance(1, DistanceUnit.INCH);

        if (Math.abs(lastDistance.minus(dist).toUnit(epsilon.unit).magnitude) < epsilon.magnitude)
        {
            return;
        }

        lastDistance = dist;

        // regression is in inches
        setTargetRPM(RobotMath.Outtake.rpmLUT(dist));
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

    /**
     * @return If the outtake is ready for balls to be transferred and shot out
     */
    public boolean isReady()
    {
        return motor.isStable() && state.equals(State.ON);
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
