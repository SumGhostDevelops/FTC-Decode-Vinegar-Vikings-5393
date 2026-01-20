package org.firstinspires.ftc.teamcode.util.motors.modern;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.controller.SquIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.util.RobotMath;

public class PositionMotor extends PowerMotor
{
    private Controller controller = Controller.Simple;
    private SquIDFController squIDF = new SquIDFController(1.0, 1.0, 1.0, 1.0);

    private double targetDistance = 0.0;
    private double power = 1.0; // power used when turning, only for squIDF

    public PositionMotor(MotorEx motorEx, VoltageSensor battery)
    {
        super(motorEx);
        this.battery = battery;

        motorEx.setRunMode(Motor.RunMode.PositionControl);
    }

    public PositionMotor setVoltageCompensation(double volts)
    {
        super.setVoltageCompensation(volts);

        return this;
    }

    public PositionMotor usePower(double power)
    {
        this.power = RobotMath.clamp(power, 0, 1);

        return this;
    }

    public PositionMotor setController(Controller controller)
    {
        this.controller = controller;

        switch (controller)
        {
            case Simple:
                motorEx.setRunMode(Motor.RunMode.PositionControl);
                break;
            case SquIDF:
                motorEx.setRunMode(Motor.RunMode.RawPower);
                break;
        }

        return this;
    }

    public PositionMotor setPositionCoefficient(double kp)
    {
        motorEx.setPositionCoefficient(kp);
        squIDF.setP(kp);

        return this;
    }

    /**
     * F term is not used on the Simple controller
     * @param kp
     * @param ki
     * @param kd
     * @param kf
     * @return
     */
    public PositionMotor setPIDF(double kp, double ki, double kd, double kf)
    {
        motorEx.setVeloCoefficients(kp, ki, kd);
        squIDF.setPIDF(kp, ki, kd, kf);

        return this;
    }

    public PositionMotor setPIDF(PIDFCoefficients coefficients)
    {
        return setPIDF(coefficients.p, coefficients.i, coefficients.d, coefficients.f);
    }

    /**
     * for the position controller
     * @return
     */
    public PositionMotor setPositionTolerance(double tolerance)
    {

        motorEx.setPositionTolerance(tolerance);
        squIDF.setTolerance(tolerance);

        return this;
    }

    public void setTargetDistance(double targetDistance)
    {
        stopped = false;
        if (atSetPosition()) return;

        this.targetDistance = targetDistance;

        double voltageScale = getVoltageScale();

        switch (controller)
        {
            case Simple:
                motorEx.setTargetDistance(targetDistance);
                motorEx.set(power * voltageScale);
                break;
            case SquIDF:
                setPower(squIDF.calculate(motorEx.getDistance(), targetDistance) * power * voltageScale);
                break;
        }
    }

    public boolean atSetPosition()
    {
        switch (controller)
        {
            case Simple:
                return motorEx.atTargetPosition();
            case SquIDF:
                squIDF.calculate(motorEx.getDistance(), targetDistance);
                return squIDF.atSetPoint();
        }

        return false;
    }

    public Controller getController()
    {
        return controller;
    }

    @Override
    public void update()
    {
        super.update();

        if (atSetPosition())
        {
            stopMotor();
        }
        else
        {
            setTargetDistance(targetDistance);
        }
    }

    public enum Controller {Simple, SquIDF}
}