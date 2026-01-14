package org.firstinspires.ftc.teamcode.util.motors;

import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.util.RobotMath;

public class Motor2
{
    private final MotorEx motor;
    private final VoltageSensor battery;

    private double targetRPM = 0;
    private double currentAccel = 0.0;

    private final double rpmTolerance;
    private final double rpmAccelTolerance;

    // For calculating the acceleration of the motor
    private static final int TRACKER_WINDOW_SIZE = 7;
    private final MotorAccelerationTracker tracker = new MotorAccelerationTracker(TRACKER_WINDOW_SIZE);
    private final ElapsedTime accelTimer = new ElapsedTime();

    // The controller algorithm to use
    private final Controller controller;

    // PIDF Coefficients
    private double p = 1;
    private double i = 1;
    private double d = 1;
    private double f = 1;

    public Motor2(MotorEx motor, VoltageSensor battery)
    {
        this(motor, battery, 100, 250);
    }

    public Motor2(MotorEx motor, VoltageSensor battery, Controller controller)
    {
        this(motor, battery, 100, 250, controller);
    }

    public Motor2(MotorEx motor, VoltageSensor battery, double rpmTolerance, double rpmAccelTolerance)
    {
        this(motor, battery, rpmTolerance, rpmAccelTolerance, Controller.VELOCITY);
    }

    public Motor2(MotorEx motor, VoltageSensor battery, double rpmTolerance, double rpmAccelTolerance, Controller controller)
    {
        this.motor = motor;
        this.battery = battery;
        this.rpmTolerance = rpmTolerance;
        this.rpmAccelTolerance = rpmAccelTolerance;
        this.controller = controller;
        setRunMode();
    }

    public void set(double magnitude)
    {
        switch (controller)
        {
            case POWER:
                motor.motorEx.setPower(magnitude);
                break;
            case VELOCITY:
                motor.motorEx.setVelocity(rpmToTps(magnitude));
                break;
            case POSITION:
                motor.motorEx.setTargetPosition((int) magnitude);
                break;
            case VELO_BANG_BANG:
                break; // implment bangbang
        }
    }

    /**
     * @param power
     * @return
     */

    public void setPower(double power)
    {
        power = RobotMath.clamp(power, 0, 1);

        switch (controller)
        {
            case POWER:
            case POSITION:
                motor.motorEx.setPower(power);
                break;
            case VELOCITY:
                motor.motorEx.setVelocity(power * motor.getMaxRPM());
                break;
            case VELO_BANG_BANG:
                break; // implment bangbang
        }
    }

    public void setPosition(int ticks, double power)
    {
        if (!controller.equals(Controller.POSITION))
        {
            return;
        }

        set(ticks);
        setPower(power);
    }

    public int getPosition()
    {
        return motor.getCurrentPosition();
    }

    public double getRPM()
    {
        return tpsToRpm(motor.getCorrectedVelocity());
    }

    public double getRPMAcceleration()
    {
        return currentAccel;
    }

    public Controller getController()
    {
        return controller;
    }

    public boolean isStable()
    {
        switch (controller)
        {
            case POWER:
                return Math.abs(currentAccel) <= rpmAccelTolerance;
            case VELOCITY:
            case VELO_BANG_BANG:
                return Math.abs(getRPM() - targetRPM) <= rpmTolerance && Math.abs(currentAccel) <= rpmAccelTolerance;
            case POSITION:
                return motor.motorEx.isBusy();
            default:
                return false;
        }
    }

    public Motor.Encoder getEncoder()
    {
        return motor.encoder;
    }

    public void updateAcceleration()
    {
        try
        {
            double time = accelTimer.seconds();
            double rpm = getRPM();
            currentAccel = tracker.updateAndGetAcceleration(time, rpm);
        }
        catch (Exception e)
        {
            // Silently handle any errors
            currentAccel = 0.0;
        }
    }

    private double tpsToRpm(double tps)
    {
        double ppr = motor.getCPR();

        if (ppr == 0.0) return 0.0;
        return (tps / ppr) * 60.0;
    }

    private double rpmToTps(double rpm)
    {
        double ppr = motor.getCPR();

        return (rpm * ppr) / 60.0;
    }

    public void setZeroPowerBehavior(Motor.ZeroPowerBehavior behavior)
    {
        // Do not allow the zero power behavior for the BANG BANG controller to be changed, as it should always be set to float
        if (controller.equals(Controller.VELO_BANG_BANG))
        {
            return;
        }

        motor.setZeroPowerBehavior(behavior);
    }

    public void setDirection(Motor.Direction direction)
    {
        motor.encoder.setDirection(direction);
    }

    public void stopMotor()
    {
        motor.stopMotor();
    }

    private void setRunMode()
    {
        switch (controller)
        {
            case POWER:
                motor.setRunMode(Motor.RunMode.RawPower);
                break;
            case VELOCITY:
                motor.setRunMode(Motor.RunMode.VelocityControl);
                break;
            case POSITION:
                motor.setRunMode(Motor.RunMode.PositionControl);
                break;
            case VELO_BANG_BANG:
                motor.setRunMode(Motor.RunMode.RawPower);
                motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        }
    }

    public enum Controller
    {
        POWER,
        VELOCITY,
        POSITION,
        VELO_BANG_BANG
    }
}