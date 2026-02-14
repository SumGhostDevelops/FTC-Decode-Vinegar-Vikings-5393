package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.util.math.BetterInterpLUT;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.firstinspires.ftc.teamcode.util.motors.VelocityMotorGroup;

import java.util.function.BooleanSupplier;

public class Outtake extends SubsystemBase
{

    public enum State
    {
        ON, IDLE, OFF,
    }

    private final VelocityMotorGroup motor;
    private State state = State.OFF;

    private final double baseRPM = RobotConstants.Outtake.BASE_RPM;
    private final double movingRPMRatio = RobotConstants.Outtake.RPM_WHILE_MOVING_RATIO;

    private double targetRPM = baseRPM;
    private double setRPM = 0;

    private final static Distance epsilon = new Distance(1, DistanceUnit.INCH);
    private Distance lastDistance = Distance.ZERO;

    // Tolerance to avoid tiny floating-point updates (adjust as needed)
    private static final double RPM_EPS = 1.0;

    // For reducing the RPM while moving
    private boolean rpmRatioEnabled = false;
    private double rpmRatio = movingRPMRatio;

    // If the RPM should be adjusted using Outtake.setTargetRPM(Distance)
    // Read dynamically to allow runtime changes via FTC Dashboard
    private boolean adjustWithDistance() { return RobotConstants.Outtake.AUTO_DISTANCE_ADJUSMENT; }

    private BooleanSupplier useLUT = () -> RobotConstants.Outtake.USE_LUT;

    /**
     * Inches -> RPM
     */
    private final BetterInterpLUT rpmLUT;

    public Outtake(VelocityMotorGroup motor)
    {
        this.motor = motor;

        // Inches -> RPM
        rpmLUT = BetterInterpLUT.builder()
                .add(52.37, 2845)
                .add(62.41, 3005)
                .add(71.18, 3090)
                .add(82.37, 3185)
                .add(121.67, 3790)
                .build();
    }

    public void on()
    {
        on(false);
    }

    public void on(boolean force)
    {
        if (state == State.ON && !force)
        {
            // Even if state is already ON, still check if RPM needs updating
            double desired = targetRPM;
            if (Math.abs(desired - setRPM) > RPM_EPS)
            {
                motor.setOutputTargetRPM(desired);
                setRPM = desired;
            }
            return;
        }

        state = State.ON;

        // Only change motor mode / RPM if actually needed
        double desired = targetRPM;
        if (Math.abs(desired - setRPM) > RPM_EPS || force)
        {
            motor.setOutputTargetRPM(desired);
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
            // Even if state is already IDLE, still check if RPM needs updating
            double desired = targetRPM / 2.0;
            if (Math.abs(desired - setRPM) > RPM_EPS)
            {
                motor.setOutputTargetRPM(desired);
                setRPM = desired;
            }
            return;
        }

        state = State.IDLE;

        double desired = targetRPM / 2.0;
        if (Math.abs(desired - setRPM) > RPM_EPS || force)
        {
            motor.setOutputTargetRPM(desired);
            setRPM = desired;
        }
    }

    public void off()
    {
        off(false);
    }

    public void off(boolean force)
    {
        if (state == State.OFF && !force)
        {
            return;
        }

        state = State.OFF;
        motor.stopMotor();

        setRPM = 0;
    }

    public void setState(State state)
    {
        setState(state, false);
    }

    /**
     *
     * @param state
     * @param force
     *            If this state change should be forcibly applied, even if it is the
     *            same as the current state
     */
    public void setState(State state, boolean force)
    {
        switch (state)
        {
            case ON:
                on(force);
                break;
            case IDLE:
                idle(force);
                break;
            case OFF:
                off();
                break;
        }
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

        if (rpmRatioEnabled)
            targetRPM = newRPM * rpmRatio;
        else
            targetRPM = newRPM;

        // If motor is active, compute desired rpm for current state and update only if
        // needed
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
            motor.setOutputTargetRPM(desired);
            setRPM = desired;
        }
    }

    public void setTargetRPM(Distance dist)
    {
        if (!adjustWithDistance())
        {
            return;
        }

        if (Math.abs(lastDistance.minus(dist).toUnit(epsilon.unit).magnitude) < epsilon.magnitude)
        {
            return;
        }

        lastDistance = dist;

        if (useLUT.getAsBoolean())
        {
            setTargetRPM(rpmLUT.get(lastDistance.getInch()));
        }
        else
        {
            setTargetRPM(regression(lastDistance.getInch()));
        }
    }

    private double regression(double inches)
    {
        return 2453 + 5.6*inches + 0.044*inches*inches;
    }

    public double getTargetRPM()
    {
        return targetRPM;
    }

    public double getMotorRPM()
    {
        return motor.getMotorRPM();
    }

    public double getMotorRPMAcceleration()
    {
        return motor.getMotorRPMAcceleration();
    }

    public double getFlywheelRPM()
    {
        return motor.getOutputRPM();
    }

    public double getFlywheelRPMAcceleration()
    {
        return motor.getOutputRPMAcceleration();
    }

    /**
     * @return If the outtake is ready for balls to be transferred and shot out
     */
    public boolean isStable()
    {
        return state == State.ON && motor.atSetPoint();
    }

    public State getState()
    {
        return state;
    }

    @Override
    public void periodic()
    {
        motor.update();
    }

    @Override
    public String toString()
    {
        String str = "State: " + state + " | Flywheel Target: " + targetRPM + " | Flywheel RPM: " + motor.getOutputRPM();

        return str;
    }
}
