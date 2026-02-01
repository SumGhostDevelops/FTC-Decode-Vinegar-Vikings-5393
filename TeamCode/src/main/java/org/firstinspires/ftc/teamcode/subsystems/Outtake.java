package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.util.math.BetterInterpLUT;
import org.firstinspires.ftc.teamcode.util.math.MathUtil;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.firstinspires.ftc.teamcode.util.motors.VelocityMotorGroup;

import org.firstinspires.ftc.teamcode.subsystems.odometry.Odometry;
import org.firstinspires.ftc.teamcode.util.measure.Ballistics;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FuturePose.FuturePoseResult;

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
    private boolean adjustWithDistance = RobotConstants.Outtake.AUTO_DISTANCE_ADJUSMENT;
    private final boolean useFuturePose = RobotConstants.Outtake.USE_FUTURE_POSE;

    /**
     * Inches -> RPM
     */
    private final BetterInterpLUT rpmLUT;

    public Outtake(VelocityMotorGroup motor)
    {
        this.motor = motor;

        // Inches -> RPM
        rpmLUT = BetterInterpLUT.builder()
                .add(0, 4100)
                .add(50.32, 4100)
                .add(59.63, 4300)
                .add(70.25, 4400)
                .add(86.79, 4800)
                .add(97.81, 5000)
                .add(108.49, 5400)
                .add(Math.hypot(144, 144), 5400)
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
            return;
        }

        state = State.ON;

        // Only change motor mode / RPM if actually needed
        double desired = targetRPM;
        if (Math.abs(desired - setRPM) > RPM_EPS || force)
        {
            motor.setTargetRPM(desired);
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
            motor.setTargetRPM(desired);
            setRPM = desired;
        }
    }

    public void off()
    {
        off(false);
    }

    public void off(boolean force)
    {
        if (state == State.OFF & !force)
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
            motor.setTargetRPM(desired);
            setRPM = desired;
        }
    }

    public void setTargetRPM(Distance dist)
    {
        if (!adjustWithDistance)
        {
            return;
        }

        if (Math.abs(lastDistance.minus(dist).toUnit(epsilon.unit).magnitude) < epsilon.magnitude)
        {
            return;
        }

        lastDistance = dist;

        setTargetRPM(rpmLUT.get(lastDistance.getInch()));
    }

    /**
     * Sets the target RPM based on the distance to a field target, optionally using
     * FuturePose based on RobotConstants.
     */
    public void setTargetRPM(FieldCoordinate target, Odometry odometry)
    {
        if (useFuturePose)
        {
            FuturePoseResult result = Ballistics.calculate(target, odometry);
            setTargetRPM(result.distance);
        }
        else
        {
            setTargetRPM(odometry.getPose().distanceTo(target));
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

}
