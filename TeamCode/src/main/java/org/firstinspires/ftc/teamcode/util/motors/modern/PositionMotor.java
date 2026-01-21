package org.firstinspires.ftc.teamcode.util.motors.modern;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.SquIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.util.MathUtil;

/**
 * The PositionMotor class extends the PowerMotor class to provide additional
 * functionality for controlling motor position using various control algorithms.
 */
public class PositionMotor extends PowerMotor
{
    // The type of position controller being used (e.g., PIDF, SquIDF)
    private PositionController controllerType = PositionController.PIDF;

    // The controller instance used for position control
    private PIDFController controller;

    // PIDF coefficients for the controller
    private PIDFCoefficients coefficients = new PIDFCoefficients(1, 1, 1, 1);

    // Target distance for the motor
    private double targetDistance = 0.0;

    // Power used when turning, only applicable for SquIDF controller
    private double power = 1.0;

    /**
     * Constructor to initialize the PositionMotor with a MotorEx instance and a VoltageSensor.
     * Sets the default controller type to PIDF and the motor run mode to RawPower.
     *
     * @param motorEx The MotorEx instance to be controlled.
     * @param battery The VoltageSensor for monitoring battery voltage.
     */
    public PositionMotor(MotorEx motorEx, VoltageSensor battery)
    {
        super(motorEx, battery);

        setControllerType(PositionController.PIDF);
        motorEx.setRunMode(Motor.RunMode.RawPower);
    }

    /**
     * Sets the power used when turning (only for SquIDF controller).
     *
     * @param power The desired power level (0 to 1).
     * @return The current PositionMotor instance for method chaining.
     */
    public PositionMotor usePower(double power)
    {
        this.power = MathUtil.clamp(power, 0, 1);

        return this;
    }

    /**
     * Sets the proportional coefficient (Kp) for the position controller.
     *
     * @param kp The proportional gain.
     * @return The current PositionMotor instance for method chaining.
     */
    public PositionMotor setPositionCoefficient(double kp)
    {
        coefficients.p = kp;
        return setPIDF(coefficients);
    }

    /**
     * Sets the PIDF coefficients for the position controller.
     *
     * @param coefficients The PIDFCoefficients object containing the PIDF values.
     * @return The current PositionMotor instance for method chaining.
     */
    public PositionMotor setPIDF(PIDFCoefficients coefficients)
    {
        this.coefficients = coefficients;
        controller.setCoefficients(coefficients);

        return this;
    }

    /**
     * Sets the PIDF coefficients for the position controller.
     * Note: The F term is not used in the SquIDF controller.
     *
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param kf Feedforward gain.
     * @return The current PositionMotor instance for method chaining.
     */
    public PositionMotor setPIDF(double kp, double ki, double kd, double kf)
    {
        return setPIDF(new PIDFCoefficients(kp, ki, kd, kf));
    }

    /**
     * Sets the position tolerance for the position controller.
     *
     * @param tolerance The tolerance for position control.
     * @return The current PositionMotor instance for method chaining.
     */
    public PositionMotor setPositionTolerance(double tolerance)
    {
        controller.setTolerance(tolerance);

        return this;
    }

    /**
     * Sets the target distance for the motor to reach.
     * Calculates the output power required to reach the target distance and applies it to the motor.
     *
     * @param targetDistance The desired target distance.
     */
    public void setTargetDistance(double targetDistance)
    {
        stopped = false;
        if (atSetPoint()) return;

        this.targetDistance = targetDistance;

        double voltageScale = getVoltageScale();

        double output = controller.calculate(motorEx.getDistance(), targetDistance) * power * voltageScale;
        setPower(MathUtil.clamp(output, -1, 1));
    }

    /**
     * Checks if the motor has reached the target position.
     *
     * @return True if the motor is at the set point, false otherwise.
     */
    public boolean atSetPoint()
    {
        return controller.atSetPoint();
    }

    /**
     * Gets the current type of position controller being used.
     *
     * @return The current PositionController type.
     */
    public PositionController getControllerType()
    {
        return controllerType;
    }

    /**
     * Sets the type of position controller to be used (e.g., PIDF, SquIDF).
     *
     * @param positionController The desired position controller type.
     * @return The current PositionMotor instance for method chaining.
     */
    public PositionMotor setControllerType(PositionController positionController)
    {
        this.controllerType = positionController;

        switch (positionController)
        {
            case PIDF:
                controller = new PIDFController(coefficients);
                break;
            case SquIDF:
                controller = new SquIDFController(coefficients);
                break;
        }
        return this;
    }

    /**
     * Updates the motor's position control loop.
     * Continuously calculates the output power to maintain or reach the target position.
     */
    @Override
    public void update()
    {
        super.update(); // Update sensors

        // Always calculate. The controller will return ~0 if at setpoint anyway.
        // If gravity pulls it off, the controller will automatically fight back.
        double output = controller.calculate(motorEx.getDistance(), targetDistance);

        output *= this.power; // The max power limit

        setPower(output);
    }

    /**
     * Enum representing the types of position controllers available.
     */
    public enum PositionController
    {PIDF, SquIDF}
}