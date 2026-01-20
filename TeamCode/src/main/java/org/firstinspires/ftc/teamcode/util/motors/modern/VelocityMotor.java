package org.firstinspires.ftc.teamcode.util.motors.modern;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.util.RobotMath;

public class VelocityMotor extends PowerMotor
{
    private final VoltageSensor battery;
    private double voltageCompensation;

    private Controller controller;
    // BangBangController

    private double targetRPM = 0.0;

    private double rpmCachingTolerance = 25;

    public VelocityMotor(MotorEx motorEx, VoltageSensor battery)
    {
        super(motorEx);
        this.battery = battery;

        motorEx.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        setController(Controller.Simple);
    }

    public VelocityMotor setVoltageCompensation(double volts)
    {
        super.setVoltageCompensation(volts);

        return this;
    }

    public VelocityMotor setController(Controller controller)
    {
        this.controller = controller;

        switch (controller)
        {
            case Simple:
                motorEx.setRunMode(Motor.RunMode.PositionControl);
                break;
            case BangBang:
                motorEx.setRunMode(Motor.RunMode.RawPower);
                break;
        }

        return this;
    }

    public VelocityMotor setRpmCachingTolerance(double rpm)
    {
        rpmCachingTolerance = RobotMath.clamp(25, 0, Double.POSITIVE_INFINITY);

        return this;
    }

    /**
     * Like setRPM, but maintains a scale of the motor's max RPM
     * @param scale
     */
    public void setScale(double scale)
    {
        scale = RobotMath.clamp(scale, 0, 1);

        setRPM(scale * motorEx.getMaxRPM());
    }

    public void setRPM(double rpm)
    {
        rpm = RobotMath.clamp(rpm, 0, motorEx.getMaxRPM());

        if (Math.abs(rpm - targetRPM) < rpmCachingTolerance) return;

        stopped = false;
        double voltageScale = getVoltageScale();

        switch (controller)
        {
            case Simple:
                motorEx.setVelocity(rpmToTps(rpm));
                break;
            case BangBang:
                // voltage scale here maybe
                break;
        }
    }

    public double getTargetRPM()
    {
        return targetRPM;
    }

    public double getRpmCachingTolerance()
    {
        return rpmCachingTolerance;
    }

    @Override
    public void stopMotor()
    {
        super.stopMotor();
        targetRPM = 0.0;
    }

    @Override
    public void update()
    {
        super.update();

        // enforce that velocity motors should always be set to float mode
        if (motorEx.motor.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.FLOAT)
        {
            motorEx.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        }

        if (!stopped) setRPM(targetRPM);
    }

    public enum Controller { Simple, BangBang }
}
